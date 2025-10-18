import params_pkg::*;

module imem #(
  parameter int MEM_SIZE   = params_pkg::MEM_SIZE,
  parameter int ADDR_WIDTH = params_pkg::ADDR_WIDTH,
  parameter int DATA_WIDTH = params_pkg::DATA_WIDTH
)(
  input  logic [ADDR_WIDTH-1:0] address_i,
  output logic [DATA_WIDTH-1:0] instruction_o
);

  logic [DATA_WIDTH-1:0] mem [0:MEM_SIZE-1];

  assign instruction_o = mem[address_i];

  initial begin
    // TODO: Populate memory with correct instructions
    for (int i = 0; i < MEM_SIZE; ++i) begin
      mem[i] = i * 100;
    end
    mem[1] = 32'h4470;      // ADD r1, r2 -> r7
    mem[2] = 32'h40B23;     // SUB r16, r5 -> r18
    mem[3] = 32'h446F1;     // LW @17(r3) -> r15
    mem[4] = 32'hFFFE1981;  // LW @-8(r12) -> r24
    mem[5] = 32'h36212;     // SW r1 -> @13(r17)
    mem[6] = 32'hFFFF50E2;  // SW r14 -> @-3(r8)
    mem[7] = 32'hA0DF9;     // BEQ r16, r6, 63
    mem[8] = 32'h98DF9;     // BEQ r6, r6, 63
    mem[61] = 32'h427A;     // BNE r1, r1, 3
    mem[62] = 32'h4CAA;     // BNE r1, r6, 10
    mem[71] = 32'hFFF98D69; // BEQ r6, r6, -10
    mem[72] = 32'h390AB;    // BLT r14, r8, 10
    mem[73] = 32'h21CAB;    // BLT r8, r14, 10
    mem[83] = 32'h6BEAC;    // BGE r26, r31, 10
    mem[84] = 32'h7F4AC;    // BGE r31, r26, 10
    mem[94] = 32'h7800D;    // JMP r30
  end

endmodule
