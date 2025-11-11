`include "cpu.sv"
`include "imem.sv"

import params_pkg::*;

module tb;
  localparam MEM_SIZE = 4096;

  logic clk;
  logic rst;

  // CPU - IMEM communication wires
  logic mem_data_valid, mem_data_is_instr;
  logic [DATA_WIDTH-1:0] mem_data;
  logic rd_req_valid, wr_req_valid, req_is_instr;
  logic [ADDR_WIDTH-1:0] req_address;
  logic [DATA_WIDTH-1:0] wr_data;
  access_size_t req_access_size;

  logic [ADDR_WIDTH-1:0] model_pc, new_model_pc;

  logic [DATA_WIDTH-1:0] cpu_regs [32];
  logic [7:0] cpu_mem [MEM_SIZE];

  logic [DATA_WIDTH-1:0] model_regs [32];
  logic [7:0] model_mem [MEM_SIZE];

  logic cpu_instr_is_completed;
  instruction_t model_instr, cpu_wb_instr;
  logic [ADDR_WIDTH-1:0] cpu_wb_pc;

  logic [DATA_WIDTH-1:0] offset_sign_extend;

  logic [ADDR_WIDTH-1:0] pc_plus_offset;

  bit error;
  string error_msg;

  int total_cycles;
  int instructions_executed;

  cpu i_cpu (
    .clk_i                          (clk),
    .rst_i                          (rst),
    .mem_data_valid_i               (mem_data_valid),
    .mem_data_is_instr_i            (mem_data_is_instr),
    .mem_data_i                     (mem_data),
    .rd_req_valid_o                 (rd_req_valid),
    .wr_req_valid_o                 (wr_req_valid),
    .req_is_instr_o                 (req_is_instr),
    .req_address_o                  (req_address),
    .wr_data_o                      (wr_data),
    .req_access_size_o              (req_access_size),
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
    .rd_req_valid_i                 (rd_req_valid),
    .wr_req_valid_i                 (wr_req_valid),
    .req_is_instr_i                 (req_is_instr),
    .address_i                      (req_address),
    .wr_data_i                      (wr_data),
    .access_size_i                  (req_access_size),
    .data_valid_o                   (mem_data_valid),
    .data_is_instr_o                (mem_data_is_instr),
    .data_o                         (mem_data),
    .debug_mem_o                    (cpu_mem)
  );

  always_ff @(posedge clk) begin : check
    if (!rst) begin
      model_pc <= '0;
    end else begin
      if (cpu_instr_is_completed)
        execute_and_compare();
    end
  end

  initial begin
    clk = 1;
    rst = 0;

    total_cycles = 0;
    instructions_executed = 0;

    initialize_registers();
    initialize_memories();

    #5 clk = 0;
    #5 clk = 1; rst = 1;
    #5 clk = 0;

    for (int i = 0; i < 20000; ++i) begin
      #5 clk = 1;
      #5 clk = 0;
      ++total_cycles;
    end
  end

  function void initialize_registers();
    for (int i = 0; i < 32; ++i) begin
      model_regs[i] = '0;
    end
  endfunction

  function void initialize_memories();
    for (int i = 0; i < MEM_SIZE; ++i) begin
      model_mem[i] = 8'h0;
    end
    /*model_mem[1] = 32'h4470;      // ADD r1, r2 -> r7
    model_mem[2] = 32'h40B23;     // SUB r6, r5 -> r18
    model_mem[3] = 32'h446F1;     // LW @17(r3) -> r15
    model_mem[4] = 32'hFFFE1981;  // LW @-17(r12) -> r24
    model_mem[5] = 32'h36212;     // SW r1 -> @13(r17)
    model_mem[6] = 32'hFFFF50E2;  // SW r14 -> @-3(r8)
    model_mem[7] = 32'hA0DF9;     // BEQ r16, r6, 63
    model_mem[8] = 32'h98DF9;     // BEQ r6, r6, 63
    model_mem[61] = 32'h427A;     // BNE r1, r1, 3
    model_mem[62] = 32'h4CAA;     // BNE r1, r6, 10
    model_mem[71] = 32'hFFF98D69; // BEQ r6, r6, -10
    model_mem[72] = 32'h390AB;    // BLT r14, r8, 10
    model_mem[73] = 32'h21CAB;    // BLT r8, r14, 10
    model_mem[83] = 32'h6BEAC;    // BGE r26, r31, 10
    model_mem[84] = 32'h7F4AC;    // BGE r31, r26, 10
    model_mem[94] = 32'h400D;     // JMP r1*/

    model_mem[1] = 32'h446F1;
    model_mem[2] = 32'h4470;
    model_mem[3] = 32'h4470;
    model_mem[4] = 32'h4470;
    model_mem[5] = 32'h4470;
    model_mem[6] = 32'h4470;
    model_mem[7] = 32'h4470;
    model_mem[8] = 32'h4470;
    model_mem[9] = 32'h4470;
    model_mem[10] = 32'h4470;

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

    /*if (model_pc == 9)
      $finish;*/

    if (model_pc == 61) begin
      $display("CPI=%0.3f (total_cycles=%0d, instructions_executed=%0d)",
               real'(total_cycles) / real'(instructions_executed - 1), total_cycles,
               instructions_executed);
      $finish;
    end

    model_pc = new_model_pc;
  endtask

  task automatic check_completed_instr();
    if (model_instr != cpu_wb_instr) begin
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
        offset_sign_extend = {{20{model_instr[31]}}, model_instr[31:20]};
        case (model_instr.funct3)
          3'b000 : model_regs[model_instr.rd] = model_regs[model_instr.rs1] + offset_sign_extend;
          default: model_regs[model_instr.rd] = model_regs[model_instr.rd];
        endcase
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
      if (model_mem[i] != cpu_mem[i]) begin
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
