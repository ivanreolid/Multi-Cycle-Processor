`include "cpu.sv"
`include "imem.sv"

import params_pkg::*;

module tb;
  localparam MEM_SIZE = 4096;

  logic clk;
  logic rst;

  // CPU - IMEM communication wires
  logic imem_instr_valid;
  instruction_t imem_instr;
  logic imem_req_valid;
  logic [ADDR_WIDTH-1:0] pc;

  logic [ADDR_WIDTH-1:0] model_pc, new_model_pc;

  logic [DATA_WIDTH-1:0] cpu_regs [0:31];
  
  logic [DATA_WIDTH-1:0] model_regs [0:31];
  logic [DATA_WIDTH-1:0] model_imem [0:MEM_SIZE-1];
  logic [DATA_WIDTH-1:0] model_dmem [0:MEM_SIZE-1];

  logic cpu_instr_is_completed;
  instruction_t model_instr, cpu_wb_instr;
  logic [ADDR_WIDTH-1:0] cpu_wb_pc;

  logic [DATA_WIDTH-1:0] offset_sign_extend;

  logic [ADDR_WIDTH-1:0] pc_plus_offset;

  bit error;
  string error_msg;

  cpu i_cpu (
    .clk_i                          (clk),
    .rst_i                          (rst),
    .imem_instr_valid_i             (imem_instr_valid),
    .imem_instr_i                   (imem_instr),
    .imem_req_valid_o               (imem_req_valid),
    .pc_o                           (pc),
    .debug_instr_is_completed_o     (cpu_instr_is_completed),
    .debug_regs_o                   (cpu_regs),
    .debug_pc_o                     (cpu_wb_pc),
    .debug_instr_o                  (cpu_wb_instr)
  );

  imem #(
    .MEM_SIZE                       (MEM_SIZE),
    .ADDR_WIDTH                     (ADDR_WIDTH),
    .DATA_WIDTH                     (DATA_WIDTH)
  ) imem (
    .clk_i                          (clk),
    .rst_i                          (rst),
    .req_valid_i                    (imem_req_valid),
    .address_i                      (pc),
    .instruction_valid_o            (imem_instr_valid),
    .instruction_o                  (imem_instr)
  );

  always_ff @(posedge clk) begin : check
    if (!rst) begin
      model_pc = 1;
    end else begin
      if (cpu_instr_is_completed)
        execute_and_compare();
    end
  end

  initial begin
    clk = 1;
    rst = 0;
 
    initialize_registers();
    initialize_memories();

    #5 clk = 0;
    #5 clk = 1; rst = 1;
    #5 clk = 0;

    for (int i = 0; i < 400; ++i) begin
      #5 clk = 1;
      #5 clk = 0;
    end
  end

  function void initialize_registers();
    for (int i = 0; i < 32; ++i) begin
      model_regs[i] = i;
    end
  endfunction

  function void initialize_memories();
    for (int i = 0; i < MEM_SIZE; ++i) begin
      model_imem[i] = i * 100;
      model_dmem[i] = i;
    end
    model_imem[1] = 32'h4470;      // ADD r1, r2 -> r7
    model_imem[2] = 32'h40B23;     // SUB r6, r5 -> r18
    model_imem[3] = 32'h446F1;     // LW @17(r3) -> r15
    model_imem[4] = 32'hFFFE1981;  // LW @-17(r12) -> r24
    model_imem[5] = 32'h36212;     // SW r1 -> @13(r17)
    model_imem[6] = 32'hFFFF50E2;  // SW r14 -> @-3(r8)
    model_imem[7] = 32'hA0DF9;     // BEQ r16, r6, 63
    model_imem[8] = 32'h98DF9;     // BEQ r6, r6, 63
    model_imem[61] = 32'h427A;     // BNE r1, r1, 3
    model_imem[62] = 32'h4CAA;     // BNE r1, r6, 10
    model_imem[71] = 32'hFFF98D69; // BEQ r6, r6, -10
    model_imem[72] = 32'h390AB;    // BLT r14, r8, 10
    model_imem[73] = 32'h21CAB;    // BLT r8, r14, 10
    model_imem[83] = 32'h6BEAC;    // BGE r26, r31, 10
    model_imem[84] = 32'h7F4AC;    // BGE r31, r26, 10
    //model_imem[94] = 32'h7800D;    // JMP r30
    model_imem[94] = 32'h400D;     // JMP r1
  endfunction

  task automatic execute_and_compare();
    model_instr = model_imem[model_pc];
    new_model_pc = (model_pc + 1) % MEM_SIZE;
    error_msg = "";
    error = 1'b0;
    execute_model_instr();
    check_completed_instr();
    check_registers();
    print_check_result();
    model_pc = new_model_pc;
  endtask

  task automatic check_completed_instr();
    if (model_instr != cpu_wb_instr) begin
      error_msg = {error_msg, $sformatf(" CPU committed PC=0x%0h with instruction 0x%0h (free=0x%0h ra=0x%0h rb=0x%0h rd=0x%0h opcode=%s)\n",
                  cpu_wb_pc, cpu_wb_instr, cpu_wb_instr.free, cpu_wb_instr.ra, cpu_wb_instr.rb,
                  cpu_wb_instr.rd, opcode_to_string(cpu_wb_instr.opcode))};
      error = 1'b1;
    end
  endtask

  task check_registers();
    for (int i = 0; i < 32; ++i) begin
      if (model_regs[i] != cpu_regs[i]) begin
        error_msg = {error_msg, $sformatf(" Register %0d: model=0x%0h, CPU=0x%0h\n", i, model_regs[i], cpu_regs[i])};
        error = 1'b1;
      end
    end
  endtask

  task execute_model_instr();
    offset_sign_extend = {{(DATA_WIDTH-(INSTR_WIDTH-14)){model_instr[INSTR_WIDTH-1]}}, model_instr[INSTR_WIDTH-1:14]};
    pc_plus_offset = (model_pc + {model_instr.free, model_instr.rd}) % MEM_SIZE;
    case (model_instr.opcode)
      ADD: model_regs[model_instr.rd] = model_regs[model_instr.ra] + model_regs[model_instr.rb]; 
      LW:  model_regs[model_instr.rd] = model_dmem[offset_sign_extend + model_instr.rb];
      SW:  model_dmem[offset_sign_extend + model_instr.rb] = model_instr.rd;
      SUB: model_regs[model_instr.rd] = model_regs[model_instr.ra] - model_regs[model_instr.rb];
      AND: model_regs[model_instr.rd] = model_regs[model_instr.ra] & model_regs[model_instr.rb];
      MUL: model_regs[model_instr.rd] = model_regs[model_instr.ra] * model_regs[model_instr.rb];
      DIV: model_regs[model_instr.rd] = model_regs[model_instr.ra] / model_regs[model_instr.rb];
      OR:  model_regs[model_instr.rd] = model_regs[model_instr.ra] | model_regs[model_instr.rb];
      XOR: model_regs[model_instr.rd] = model_regs[model_instr.ra] ^ model_regs[model_instr.rb];
      BEQ: new_model_pc = model_regs[model_instr.ra] == model_regs[model_instr.rb] ? pc_plus_offset : new_model_pc;
      BNE: new_model_pc = model_regs[model_instr.ra] != model_regs[model_instr.rb] ? pc_plus_offset : new_model_pc;
      BLT: new_model_pc = model_regs[model_instr.ra] <  model_regs[model_instr.rb] ? pc_plus_offset : new_model_pc;
      BGE: new_model_pc = model_regs[model_instr.ra] >= model_regs[model_instr.rb] ? pc_plus_offset : new_model_pc;
      JMP: new_model_pc = model_regs[model_instr.ra];
      //default:
    endcase
  endtask

  task print_check_result();
    if (error) begin
      $display("t=%0t: Something went wrong with PC=0x%0h and instruction 0x%0h (free=0x%0h ra=0x%0h rb=0x%0h rd=0x%0h opcode=%s):\n%s", $time, model_pc, model_instr, model_instr.free, model_instr.ra, model_instr.rb, model_instr.rd, opcode_to_string(model_instr.opcode),
               error_msg);
    end else begin
      $display("t=%0t: PC=0x%0h with instruction 0x%0h (free=0x%0h ra=0x%0h rb=0x%0h rd=0x%0h opcode=%s) executed correctly", $time, model_pc,
               model_instr, model_instr.free, model_instr.ra, model_instr.rb, model_instr.rd, opcode_to_string(model_instr.opcode));
    end
  endtask

endmodule
