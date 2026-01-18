import params_pkg::*; 

module StoreBuffer #(
  parameter int ROB_ENTRY_WIDTH = 4,
  parameter int BUFFER_INDEX_WIDTH = 2,
  parameter int BUFFER_SIZE = 2**BUFFER_INDEX_WIDTH,
  parameter int DATA_WIDTH = 32,
  parameter int ADDR_WIDTH = 32,
  parameter int REGISTER_WIDTH = 5
) (
  input  logic clk_i,
  input  logic rst_i,
  
  input  logic finish_i,
  output logic done_o,
  
  output logic sb_full_o,
  
  input  logic                       new_store_valid_i,
  input  logic [ROB_ENTRY_WIDTH-1:0] new_store_rob_idx_i,
  input  logic [ADDR_WIDTH-1:0]      new_store_addr_i,
  input  logic [DATA_WIDTH-1:0]      new_store_data_i,
  input  access_size_t               new_store_size_i,
  
  input  logic                       rob_commit_valid_i,
  input  logic [ROB_ENTRY_WIDTH-1:0] rob_commit_idx_i,
  
  input  logic                       load_check_valid_i,
  input  logic [ADDR_WIDTH-1:0]      load_addr_i,
  input  access_size_t               load_size_i,
  output logic                       load_forward_valid_o,
  output logic [DATA_WIDTH-1:0]      load_forward_data_o,
  output logic                       load_must_wait_o,
  
  output logic                       dcache_store_req_o,
  output logic [ADDR_WIDTH-1:0]      dcache_store_addr_o,
  output logic [DATA_WIDTH-1:0]      dcache_store_data_o,
  output access_size_t               dcache_store_size_o,
  input  logic                       dcache_store_ack_i
);

  typedef struct packed {
    logic                       valid;
    logic                       committed;
    logic [ROB_ENTRY_WIDTH-1:0] rob_idx;
    logic [ADDR_WIDTH-1:0]      addr;
    logic [DATA_WIDTH-1:0]      data;
    access_size_t               size;
  } sb_entry_t;
  
  sb_entry_t [BUFFER_SIZE-1:0] entries;
  
  logic [BUFFER_INDEX_WIDTH-1:0] head_ptr;
  logic [BUFFER_INDEX_WIDTH-1:0] tail_ptr;
  logic [BUFFER_INDEX_WIDTH:0]   count;
  
  logic finish_mode;
  logic [3:0] drain_idle_counter;  
  
  assign sb_full_o = (count >= BUFFER_SIZE);
  
  assign done_o = finish_mode && (count == 0) && (drain_idle_counter >= 4'd5);
  
  logic head_can_issue;
  logic head_committed_or_committing;

  
  always_comb begin
  head_committed_or_committing = entries[head_ptr].committed;
  
  if (rob_commit_valid_i && entries[head_ptr].valid && 
      entries[head_ptr].rob_idx == rob_commit_idx_i) begin
    head_committed_or_committing = 1'b1;
  end
end

assign head_can_issue = (count > 0) && entries[head_ptr].valid && 
                        (head_committed_or_committing || finish_mode);
  
  assign dcache_store_req_o = head_can_issue;
  assign dcache_store_addr_o = entries[head_ptr].addr;
  assign dcache_store_data_o = entries[head_ptr].data;
  assign dcache_store_size_o = entries[head_ptr].size;
  
  function automatic logic [DATA_WIDTH-1:0] extract_bytes(
    input logic [DATA_WIDTH-1:0] store_data,
    input logic [ADDR_WIDTH-1:0] store_addr,
    input logic [ADDR_WIDTH-1:0] load_addr,
    input access_size_t load_size
  );
    logic [1:0] offset;
    offset = load_addr[1:0] - store_addr[1:0];
    
    case (load_size)
      BYTE: begin
        case (offset)
          2'b00: return {{24{1'b0}}, store_data[7:0]};
          2'b01: return {{24{1'b0}}, store_data[15:8]};
          2'b10: return {{24{1'b0}}, store_data[23:16]};
          2'b11: return {{24{1'b0}}, store_data[31:24]};
        endcase
      end
      HALF: begin
        return (offset[1]) ? {{16{1'b0}}, store_data[31:16]} : {{16{1'b0}}, store_data[15:0]};
      end
      WORD: return store_data;
      default: return '0;
    endcase
  endfunction
  
  function automatic logic check_overlap(
    input logic [ADDR_WIDTH-1:0] store_addr,
    input access_size_t store_size,
    input logic [ADDR_WIDTH-1:0] load_addr,
    input access_size_t load_size
  );
    logic [ADDR_WIDTH-1:0] store_end, load_end;
    
    case (store_size)
      BYTE: store_end = store_addr + 1;
      HALF: store_end = store_addr + 2;
      WORD: store_end = store_addr + 4;
      default: store_end = store_addr;
    endcase
    
    case (load_size)
      BYTE: load_end = load_addr + 1;
      HALF: load_end = load_addr + 2;
      WORD: load_end = load_addr + 4;
      default: load_end = load_addr;
    endcase
    
    return (store_addr < load_end) && (load_addr < store_end);
  endfunction
  
  integer i;
  logic [BUFFER_INDEX_WIDTH-1:0] search_idx;
  logic fwd_found;
  logic [DATA_WIDTH-1:0] fwd_data;
  logic fwd_wait;
  
  always_comb begin
    fwd_found = 1'b0;
    fwd_data = '0;
    fwd_wait = 1'b0;
    
    if (load_check_valid_i && count > 0) begin
      search_idx = tail_ptr - 1'b1;
      
      for (i = 0; i < BUFFER_SIZE; i++) begin
        if (entries[search_idx].valid && !fwd_found && !fwd_wait) begin
          if (check_overlap(entries[search_idx].addr, entries[search_idx].size, 
                           load_addr_i, load_size_i)) begin
            if ((entries[search_idx].addr == load_addr_i) && 
                (entries[search_idx].size == load_size_i)) begin
              fwd_found = 1'b1;
              fwd_data = entries[search_idx].data;
            end
            else if (entries[search_idx].size == WORD || 
                    (entries[search_idx].size == HALF && load_size_i == BYTE)) begin
              fwd_found = 1'b1;
              fwd_data = extract_bytes(entries[search_idx].data, entries[search_idx].addr, 
                                      load_addr_i, load_size_i);
            end
            else begin
              fwd_wait = 1'b1;
            end
          end
        end
        
        if (search_idx == head_ptr) break;
        search_idx = search_idx - 1'b1;
      end
    end
  end
  
  assign load_forward_valid_o = fwd_found;
  assign load_forward_data_o = fwd_data;
  assign load_must_wait_o = fwd_wait;
  
  integer j;
  logic [BUFFER_INDEX_WIDTH:0] count_delta;
  
  always_ff @(posedge clk_i) begin
    if (!rst_i) begin
      head_ptr <= '0;
      tail_ptr <= '0;
      count <= '0;
      finish_mode <= 1'b0;
      drain_idle_counter <= '0;
      
      for (j = 0; j < BUFFER_SIZE; j++) begin
        entries[j].valid <= 1'b0;
        entries[j].committed <= 1'b0;
        entries[j].rob_idx <= '0;
        entries[j].addr <= '0;
        entries[j].data <= '0;
        entries[j].size <= WORD;
      end
      
    end else begin
      
      count_delta = '0;
      
      // FINISH MODE
      if (finish_i && !finish_mode) begin
        finish_mode <= 1'b1;
        drain_idle_counter <= '0;
      end
      
      // Drain idle counter
      if (finish_mode) begin
        if (count == 0) begin
          if (drain_idle_counter < 4'd10) begin
            drain_idle_counter <= drain_idle_counter + 1'b1;
          end
        end else begin
          drain_idle_counter <= '0;
        end
      end
      
      // ROB COMMIT
      if (rob_commit_valid_i) begin
        for (j = 0; j < BUFFER_SIZE; j++) begin
          if (entries[j].valid && (entries[j].rob_idx == rob_commit_idx_i)) begin
            entries[j].committed <= 1'b1;
          end
        end
      end
      
      if (new_store_valid_i && !sb_full_o && !finish_mode) begin
        entries[tail_ptr].valid <= 1'b1;
        entries[tail_ptr].committed <= 1'b0;
        entries[tail_ptr].rob_idx <= new_store_rob_idx_i;
        entries[tail_ptr].addr <= new_store_addr_i;
        entries[tail_ptr].data <= new_store_data_i;
        entries[tail_ptr].size <= new_store_size_i;
        
        tail_ptr <= tail_ptr + 1'b1;
        count_delta = count_delta + 1'b1;
      end
      
      // DRAIN - cache ack 
      if (dcache_store_ack_i && head_can_issue) begin
        entries[head_ptr].valid <= 1'b0;
        entries[head_ptr].committed <= 1'b0;
        
        head_ptr <= head_ptr + 1'b1;
        count_delta = count_delta - 1'b1;
      end
      
      count <= count + count_delta;
    end
  end




endmodule