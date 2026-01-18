`include "tlb.sv"
`include "ptw.sv"
`include "instr_cache.sv"

module fetch_stage import params_pkg::*; #(
  parameter int PPN_WIDTH         = params_pkg::PPN_WIDTH,
  parameter int MEM_SIZE          = params_pkg::MEM_SIZE,
  parameter int CACHE_LINE_BYTES  = params_pkg::CACHE_LINE_BYTES
)(
  input  logic clk_i,
  input  logic rst_i,
  input  logic vm_en_i,
  input  logic trap_bypass_mmu_i,
  input  logic ppn_is_present_i,
  input  logic mem_req_i,
  input  logic flush_i,
  input  logic alu_branch_taken_i,
  input  logic is_jump_i,
  input  logic dec_stall_i,
  input  logic mem_stall_i,
  input  data_t satp_data_i,
  input  vaddr_t next_pc_flush_i,
  input  vaddr_t pc_branch_offset_i,
  input  vaddr_t jump_address_i,

  // Memory interface (Cache to Memory)
  input  logic instr_valid_i,
  input  cacheline_t instr_line_i,
  output logic present_table_req_o,
  output logic rd_req_valid_o,
  output logic [PPN_WIDTH-1:0] present_table_ppn_o,
  output paddr_t mem_req_addr_o,
  output access_size_t req_access_size_o,

  // Memory arbiter grant signal
  input  logic mem_gnt_i,

  // Decode stage outputs
  output logic dec_valid_o,
  output logic dec_excpt_o,
  output vaddr_t dec_pc_o,
  output var excpt_cause_t dec_excpt_cause_o,
  output var instruction_t dec_instr_o
);

  typedef enum logic [2:0] {
    IDLE     = 3'b000,
    MEM_REQ  = 3'b001,
    MEM_WAIT = 3'b010,
    STALL    = 3'b100,
    FLUSH    = 3'b101
  } state_t;

  state_t state, state_d;
  vaddr_t pc, pc_d;
  paddr_t paddr;

  // Stall Buffer
  logic buffer_wr_en;
  logic error_buffer, error_buffer_d;
  vaddr_t pc_buffer, pc_buffer_d;
  excpt_cause_t error_cause_buffer, error_cause_buffer_d;
  instruction_t instr_buffer;

  logic mmu_enable;

  // ITLB wires
  logic itlb_hit;
  logic itlb_wr_en;
  logic [PPN_WIDTH-1:0] itlb_ppn;

  // IPTW wires
  logic iptw_req;
  logic iptw_valid;
  logic iptw_error;
  paddr_t iptw_paddr;

  // Cache signals
  logic cache_state_reset;
  logic cache_req;
  logic cache_hit;
  logic cache_ready;
  logic cache_rvalid;
  logic [31:0] cache_rdata;

  tlb i_tlb (
    .clk_i     (clk_i),
    .rst_i     (rst_i),
    .flush_i   (flush_i),
    .wr_en_i   (itlb_wr_en),
    .vpn_i     (pc[31:12]),
    .ppn_i     (iptw_paddr[19:12]),
    .hit_o     (itlb_hit),
    .ppn_o     (itlb_ppn)
  );

  ptw i_ptw (
    .req_i            (iptw_req),
    .ppn_is_present_i (ppn_is_present_i),
    .vaddr_i          (pc),
    .satp_data_i      (satp_data_i),
    .valid_o          (iptw_valid),
    .error_o          (iptw_error),
    .paddr_o          (iptw_paddr)
  );

  // --- Cache Instantiation ---
  instr_cache i_cache (
    .clk(clk_i),
    .rstn(rst_i),
    .state_reset(cache_state_reset),
    .cpu_req(cache_req),
    .cpu_addr(paddr),
    .cpu_size(WORD),
    .cpu_ready(cache_ready),
    .cpu_rdata(cache_rdata),
    .cpu_rvalid(cache_rvalid),
    .curr_cache_hit(cache_hit),
    .mem_req(rd_req_valid_o),
    .mem_addr(mem_req_addr_o),
    .mem_gnt(mem_gnt_i),
    .mem_rvalid(instr_valid_i),
    .mem_rdata(instr_line_i)
  );

  // --- Next State Logic ---
  always_comb begin : state_update
    // Default values
    state_d      = state;
    pc_d         = pc;

    itlb_wr_en    = 1'b0;

    iptw_req      = 1'b0;
    present_table_req_o = 1'b0;
    present_table_ppn_o = '0;

    cache_req         = 1'b0;
    cache_state_reset = 1'b0;

    buffer_wr_en         = 1'b0;
    pc_buffer_d          = pc;
    error_buffer_d       = 1'b0;
    error_cause_buffer_d = '{default: 0};

    dec_valid_o  = 1'b0;
    dec_excpt_o  = 1'b0;
    dec_pc_o     = pc;
    dec_instr_o  = instruction_t'('0);
    req_access_size_o = WORD;

    if (flush_i || alu_branch_taken_i || is_jump_i) begin
      buffer_wr_en = 1'b1;
      state_d      = state == MEM_WAIT ? FLUSH : MEM_REQ;
      if (flush_i) begin
        pc_buffer_d  = next_pc_flush_i;
        pc_d         = next_pc_flush_i;
      end else begin
        pc_d        = alu_branch_taken_i ? pc_branch_offset_i : jump_address_i;
        pc_buffer_d = alu_branch_taken_i ? pc_branch_offset_i : jump_address_i;
      end
    end else begin
      case (state)
        IDLE: begin
          state_d = MEM_REQ;
        end
        MEM_REQ: begin
          if (mmu_enable) begin
            if (itlb_hit) begin
              paddr = {itlb_ppn, pc[11:0]};
            end else begin
              iptw_req = 1'b1;
              present_table_req_o = 1'b1;
              present_table_ppn_o = iptw_paddr[19:12];

              if (iptw_valid) begin
                paddr      = iptw_paddr;
                itlb_wr_en = 1'b1;
              end
            end
          end else begin
            paddr = pc[19:0];
          end

          if (iptw_error) begin
            if (dec_stall_i) begin
              buffer_wr_en         = 1'b1;
              error_buffer_d       = 1'b1;
              error_cause_buffer_d = INSTR_PAGE_FAULT;
              state_d              = STALL;
            end else begin
              dec_valid_o        = 1'b1;
              dec_excpt_o        = 1'b1;
              dec_excpt_cause_o  = INSTR_PAGE_FAULT;
              dec_pc_o           = pc;
              state_d            = MEM_REQ;
            end
          end else begin
            cache_req   = 1'b1;

            if (cache_hit && cache_rvalid) begin
              if (dec_stall_i) begin
                buffer_wr_en = 1'b1;
                state_d      = STALL;
              end else begin
                dec_valid_o = 1'b1;
                dec_instr_o = instruction_t'(cache_rdata);
                dec_pc_o    = pc;

                pc_d        = (pc + 4) % MEM_SIZE;
                state_d     = MEM_REQ;
              end
            end else begin
              state_d   = MEM_WAIT;
            end
          end
        end
        MEM_WAIT: begin
          if (cache_rvalid) begin
            if (dec_stall_i) begin
              buffer_wr_en = 1'b1;
              state_d      = STALL;
            end else begin
              dec_valid_o = 1'b1;
              dec_instr_o = instruction_t'(cache_rdata);
              dec_pc_o    = pc;

              pc_d        = (pc + 4) % MEM_SIZE;
              state_d     = MEM_REQ;
            end
          end
        end
        FLUSH: begin
          if (cache_rvalid) begin
            pc_d    = pc_buffer;
            state_d = MEM_REQ;
          end
        end
        STALL: begin
          dec_valid_o       = 1'b1;
          dec_excpt_o       = error_buffer;
          dec_excpt_cause_o = error_cause_buffer;
          dec_instr_o       = instr_buffer;
          dec_pc_o          = pc_buffer;

          if (!dec_stall_i) begin
            pc_d    = (pc + 4) % MEM_SIZE;
            state_d = MEM_REQ;
          end
        end

        default: state_d = IDLE;
      endcase
    end
  end

  // --- Sequential Logic ---
  always_ff @(posedge clk_i) begin : flops
    if (!rst_i) begin
      state        <= IDLE;
      pc           <= 4096;
      error_buffer       <= 1'b0;
      error_cause_buffer <= '{default: 0};
      instr_buffer       <= '0;
      pc_buffer          <= '0;
    end else begin
      state <= state_d;
      pc    <= pc_d;

      if (buffer_wr_en) begin
        error_buffer       <= error_buffer_d;
        error_cause_buffer <= error_cause_buffer_d;
        instr_buffer       <= instruction_t'(cache_rdata);
        pc_buffer          <= pc_buffer_d;
      end
    end
  end

  assign mmu_enable = vm_en_i && !trap_bypass_mmu_i;

endmodule : fetch_stage
