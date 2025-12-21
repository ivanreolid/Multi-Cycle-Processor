`include "cpu.sv"
`include "mem.sv"

import params_pkg::*;

module tb;
  logic clk;
  logic rst;

  // Instruction Cache - Memory Interface
  logic                      icache_mem_req;
  logic [ADDR_WIDTH-1:0]     icache_mem_addr;
  logic                      icache_mem_gnt;
  logic                      icache_mem_rvalid;
  logic [127:0]              icache_mem_rdata;

  // Data Cache - Memory Interface
  logic                      dcache_mem_req;
  logic                      dcache_mem_we;
  logic [ADDR_WIDTH-1:0]     dcache_mem_addr;
  logic [127:0]              dcache_mem_wdata;
  logic                      dcache_mem_gnt;
  logic                      dcache_mem_rvalid;
  logic [127:0]              dcache_mem_rdata;

  logic [ADDR_WIDTH-1:0] model_pc, new_model_pc;

  logic [DATA_WIDTH-1:0] cpu_regs [32];
  logic [7:0] cpu_mem [MEM_SIZE];

  logic [DATA_WIDTH-1:0] model_regs [32];
  logic [7:0] model_mem [MEM_SIZE];

  logic cpu_instr_is_completed;
  instruction_t model_instr, cpu_wb_instr;
  logic [ADDR_WIDTH-1:0] cpu_wb_pc;

  logic [SHAMT_WIDTH-1:0] shamt;
  logic [DATA_WIDTH-1:0] offset_sign_extend;

  logic [ADDR_WIDTH-1:0] pc_plus_offset;

  bit error;
  string error_msg;

  int total_cycles;
  int instructions_executed;

  cpu i_cpu (
    .clk_i                          (clk),
    .rst_i                          (rst),
    // Instruction Cache Interface
    .icache_mem_req_o               (icache_mem_req),
    .icache_mem_addr_o              (icache_mem_addr),
    .icache_mem_gnt_i               (icache_mem_gnt),
    .icache_mem_rvalid_i            (icache_mem_rvalid),
    .icache_mem_rdata_i             (icache_mem_rdata),
    // Data Cache Interface
    .dcache_mem_req_o               (dcache_mem_req),
    .dcache_mem_we_o                (dcache_mem_we),
    .dcache_mem_addr_o              (dcache_mem_addr),
    .dcache_mem_wdata_o             (dcache_mem_wdata),
    .dcache_mem_gnt_i               (dcache_mem_gnt),
    .dcache_mem_rvalid_i            (dcache_mem_rvalid),
    .dcache_mem_rdata_i             (dcache_mem_rdata),
    // Debug Interface
    .debug_instr_is_completed_o     (cpu_instr_is_completed),
    .debug_regs_o                   (cpu_regs),
    .debug_pc_o                     (cpu_wb_pc),
    .debug_instr_o                  (cpu_wb_instr)
  );

  mem #(
    .MEM_SIZE                       (MEM_SIZE),
    .ADDR_WIDTH                     (ADDR_WIDTH),
    .LINE_BYTES                     (16)
  ) memory (
    .clk                            (clk),
    .rstn                           (rst),
    // Instruction Cache Interface
    .icache_req                     (icache_mem_req),
    .icache_addr                    (icache_mem_addr),
    .icache_gnt                     (icache_mem_gnt),
    .icache_rvalid                  (icache_mem_rvalid),
    .icache_rdata                   (icache_mem_rdata),
    // Data Cache Interface
    .dcache_req                     (dcache_mem_req),
    .dcache_we                      (dcache_mem_we),
    .dcache_addr                    (dcache_mem_addr),
    .dcache_wdata                   (dcache_mem_wdata),
    .dcache_gnt                     (dcache_mem_gnt),
    .dcache_rvalid                  (dcache_mem_rvalid),
    .dcache_rdata                   (dcache_mem_rdata),
    // Debug Interface
    .debug_mem_o                    (cpu_mem)
  );

  always_ff @(posedge clk) begin : cycles_count
    if (!rst)
      total_cycles <= 0;
    else
      total_cycles <= total_cycles + 1;
  end

  always_ff @(posedge clk) begin : check
    if (!rst) begin
      model_pc <= '0;
    end else if (cpu_instr_is_completed) begin
        execute_and_compare();
    end
  end

  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst = 0;
    instructions_executed = 0;

    initialize_registers();
    initialize_memories();

    repeat (2) @(posedge clk);
    rst = 1;
  end

  function void initialize_registers();
    for (int i = 0; i < 32; ++i) begin
      model_regs[i] = '0;
    end
  endfunction

  function void initialize_memories();
    //$readmemh("buffer_sum.mem", model_mem);
    //$readmemh("mem_copy.mem", model_mem);
    $readmemh("matrix_multiply.mem", model_mem);
  endfunction

  task automatic execute_and_compare();
    model_instr = { model_mem[model_pc + 3], model_mem[model_pc + 2], model_mem[model_pc + 1],
                    model_mem[model_pc] };
    new_model_pc = (model_pc + 4) % MEM_SIZE;
    error_msg = "";
    error = 1'b0;
    execute_model_instr();
    check_completed_instr();
    check_registers();
    print_check_result();
    ++instructions_executed;

    if (model_pc == 224) begin
      compare_memories();
      $display("CPI=%0.3f (total_cycles=%0d, instructions_executed=%0d)",
               real'(total_cycles) / real'(instructions_executed - 1), total_cycles,
               instructions_executed);
      $finish;
    end

    model_pc = new_model_pc;
  endtask

  task automatic check_completed_instr();
    if ((model_pc != cpu_wb_pc) || (model_instr != cpu_wb_instr)) begin
      error_msg = {error_msg, $sformatf(" CPU committed PC=0x%0h with instruction 0x%0h",
                   cpu_wb_pc, cpu_wb_instr)};
      error_msg = {error_msg, $sformatf(" (funct7=0x%0h rs2=0x%0h rs1=0x%0h funct3=0x%0h",
                  cpu_wb_instr.funct7, cpu_wb_instr.rs2, cpu_wb_instr.rs1, cpu_wb_instr.funct3)};
      error_msg = {error_msg, $sformatf(" rd=0x%0h opcode=%s)\n", cpu_wb_instr.rd,
                   opcode_to_string(cpu_wb_instr.opcode))};
      error = 1'b1;
    end
  endtask

  task check_registers();
    for (int i = 0; i < 32; ++i) begin
      if (model_regs[i] !== cpu_regs[i]) begin
        error_msg = {error_msg, $sformatf(" Register %0d: model=0x%0h, CPU=0x%0h\n", i,
                    model_regs[i], cpu_regs[i])};
        error = 1'b1;
      end
    end
  endtask

  task execute_model_instr();
    case (model_instr.opcode)
      R: begin
        if (model_instr.funct3 == 3'b000) begin //ADD, SUB or MUL
          case (model_instr.funct7)
            7'b0000000: model_regs[model_instr.rd] =
                        model_regs[model_instr.rs1] + model_regs[model_instr.rs2];
            7'b0100000: model_regs[model_instr.rd] =
                        model_regs[model_instr.rs1] - model_regs[model_instr.rs2];
            7'b0000001: model_regs[model_instr.rd] =
                        model_regs[model_instr.rs1] * model_regs[model_instr.rs2];
          endcase
        end
      end
      LOAD: begin
        offset_sign_extend = {{20{model_instr[31]}}, model_instr[31:20]};
        if (model_instr.funct3 == 3'b000) begin            // LB
          model_regs[model_instr.rd] = {24'b0, model_mem[offset_sign_extend +
                                        model_regs[model_instr.rs1]]};
        end else if (model_instr.funct3 == 3'b010) begin  // LW
          model_regs[model_instr.rd] = {
            model_mem[(offset_sign_extend + model_regs[model_instr.rs1]) + 3],
            model_mem[(offset_sign_extend + model_regs[model_instr.rs1]) + 2],
            model_mem[(offset_sign_extend + model_regs[model_instr.rs1]) + 1],
            model_mem[(offset_sign_extend + model_regs[model_instr.rs1])]};
        end
      end
      STORE: begin
        offset_sign_extend = {{20{model_instr[31]}}, model_instr[31:25], model_instr[11:7]};
        if (model_instr.funct3 == 3'b000) begin           // SB
          model_mem[offset_sign_extend + model_regs[model_instr.rs1]] =
                   model_regs[model_instr.rs2][7:0];
        end else if (model_instr.funct3 == 3'b010) begin  // SW
          model_mem[(offset_sign_extend + model_regs[model_instr.rs1])]
                   = model_regs[model_instr.rs2][7:0];
          model_mem[(offset_sign_extend + model_regs[model_instr.rs1]) + 1]
                   = model_regs[model_instr.rs2][15:8];
          model_mem[(offset_sign_extend + model_regs[model_instr.rs1]) + 2]
                   = model_regs[model_instr.rs2][23:16];
          model_mem[(offset_sign_extend + model_regs[model_instr.rs1]) + 3]
                   = model_regs[model_instr.rs2][31:24];
        end
      end
      BRANCH: begin
        offset_sign_extend = {{19{model_instr[31]}}, model_instr[31], model_instr[7],
                             model_instr[30:25], model_instr[11:8], 1'b0};
        pc_plus_offset = (model_pc + offset_sign_extend) % MEM_SIZE;
        case (model_instr.funct3)
          3'b000: new_model_pc = model_regs[model_instr.rs1] == model_regs[model_instr.rs2] ? // BEQ
                                 pc_plus_offset : new_model_pc;
          3'b001: new_model_pc = model_regs[model_instr.rs1] != model_regs[model_instr.rs2] ? // BNE
                                 pc_plus_offset : new_model_pc;
          3'b100: new_model_pc = model_regs[model_instr.rs1] < model_regs[model_instr.rs2] ?  // BLT
                         pc_plus_offset : new_model_pc;
          3'b101: new_model_pc = model_regs[model_instr.rs1] >= model_regs[model_instr.rs2] ? // BGE
                         pc_plus_offset : new_model_pc;
          default: new_model_pc = model_pc;
        endcase
      end
      JAL: begin
        offset_sign_extend = {{11{model_instr[31]}}, model_instr[31], model_instr[19:12],
                               model_instr[20], model_instr[30:21], 1'b0};
        new_model_pc = model_pc + offset_sign_extend;
        model_regs[model_instr.rd] = model_pc + 4;
      end
      IMMEDIATE: begin
        shamt              = {model_instr[25], model_instr.rs2};
        offset_sign_extend = {{20{model_instr[31]}}, model_instr[31:20]};
        case (model_instr.funct3)
          3'b000 : model_regs[model_instr.rd] = model_regs[model_instr.rs1] + offset_sign_extend;
          3'b001 : model_regs[model_instr.rd] = model_regs[model_instr.rs1] << shamt;
          default: model_regs[model_instr.rd] = model_regs[model_instr.rd];
        endcase
      end
      AUIPC: begin
        offset_sign_extend = {{model_instr[31]}, model_instr[31:12] << 12};
        model_regs[model_instr.rd] = model_pc + offset_sign_extend;
      end
    endcase
  endtask

  task print_check_result();
    if (error) begin
      $write("t=%0t: Something went wrong with PC=0x%0h and instruction 0x%0h (funct7=0x%0h",
             $time, model_pc, model_instr, model_instr.funct7);
      $write(" rs1=0x%0h rs2=0x%0h funct3=0x%0h rd=0x%0h opcode=%s)\n%s", model_instr.rs1,
             model_instr.rs2, model_instr.funct3, model_instr.rd,
             opcode_to_string(model_instr.opcode), error_msg);
    end else begin
      $write("t=%0t: PC=0x%0h with instruction 0x%0h (funct7=0x%0h rs1=0x%0h rs2=0x%0h",
             $time, model_pc, model_instr, model_instr.funct7, model_instr.rs1,
             model_instr.rs2);
      $write(" funct3=0x%0h rd=0x%0h opcode=%s) executed correctly\n", model_instr.funct3,
             model_instr.rd, opcode_to_string(model_instr.opcode));
    end
  endtask

  task compare_memories();
    error_msg = "";
    error = 1'b0;
    for (int i = 0; i < MEM_SIZE; i++) begin
      if (model_mem[i] !== cpu_mem[i]) begin
        error = 1'b1;
        error_msg = {error_msg, $sformatf("@0x%0h model=0x%0h, cpu=0x%0h\n", i, model_mem[i],
                    cpu_mem[i])};
      end
    end
    if (error)
      $display("Mismatch on memories at the end of the test\n%s", error_msg);
    else
      $display("Memories are equal at the end of the test");
  endtask

endmodule
