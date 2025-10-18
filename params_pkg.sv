package params_pkg;

  parameter int OPCODE_WIDTH = 4;
  parameter int REGISTER_WIDTH = 5;
  parameter int INSTR_WIDTH = 32;
  parameter int ADDR_WIDTH  = 32;
  parameter int DATA_WIDTH  = 32;
  parameter int MEM_SIZE = 4096;

  typedef enum logic[3:0] {
    ADD = 4'b0000,
    LW  = 4'b0001,
    SW  = 4'b0010,
    SUB = 4'b0011,
    MUL = 4'b0100,
    DIV = 4'b0101,
    AND = 4'b0110,
    OR  = 4'b0111,
    XOR = 4'b1000,
    BEQ = 4'b1001,
    BNE = 4'b1010,
    BLT = 4'b1011,
    BGE = 4'b1100,
    JMP = 4'b1101
  } opcode;

  function string opcode_to_string(opcode op);
    case (op)
      ADD: opcode_to_string = "ADD";
      LW : opcode_to_string = "LW";
      SW : opcode_to_string = "SW";
      SUB: opcode_to_string = "SUB";
      MUL: opcode_to_string = "MUL";
      DIV: opcode_to_string = "DIV";
      AND: opcode_to_string = "AND";
      OR : opcode_to_string = "OR";
      XOR: opcode_to_string = "XOR";
      BEQ: opcode_to_string = "BEQ";
      BNE: opcode_to_string = "BNE";
      BLT: opcode_to_string = "BLT";
      BGE: opcode_to_string = "BGE";
      JMP: opcode_to_string = "JMP";
      default: opcode_to_string = "???";
    endcase
  endfunction

  typedef struct packed {
    logic [12:0] free;
    logic [REGISTER_WIDTH-1:0]  ra;
    logic [REGISTER_WIDTH-1:0]  rb;
    logic [REGISTER_WIDTH-1:0]  rd;
    opcode opcode;
  } instruction_t;

endpackage
