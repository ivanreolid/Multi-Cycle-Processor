package params_pkg;

  parameter int OPCODE_WIDTH   = 4;
  parameter int REGISTER_WIDTH = 5;
  parameter int SHAMT_WIDTH    = 6;
  parameter int INSTR_WIDTH    = 32;
  parameter int ADDR_WIDTH     = 32;
  parameter int DATA_WIDTH     = 32;
  parameter int MEM_SIZE       = 200000;

  typedef enum logic[6:0] {
    R         = 7'b0110011,
    LOAD      = 7'b0000011,
    STORE     = 7'b0100011,
    BRANCH    = 7'b1100011,
    JAL       = 7'b1101111,
    IMMEDIATE = 7'b0010011,
    AUIPC     = 7'b0010111
  } opcode;

  typedef struct packed {
    logic is_mul;
    logic is_instr_wb_alu;
    logic is_instr_mem;
    logic is_branch;
    logic rs1_needed;
    logic rs2_needed;
    logic [4:0] rs1;
    logic [4:0] rs2;
  } hazard_ctrl_t;

  typedef enum logic [1:0] {
    BYTE  = 2'b00,
    HALF  = 2'b01,
    WORD  = 2'b10
  } access_size_t;

  function string opcode_to_string(opcode op);
    case (op)
      R         : opcode_to_string = "R";
      LOAD      : opcode_to_string = "LOAD";
      STORE     : opcode_to_string = "STORE";
      BRANCH    : opcode_to_string = "BRANCH";
      JAL       : opcode_to_string = "JAL";
      IMMEDIATE : opcode_to_string = "IMMEDIATE";
      AUIPC     : opcode_to_string = "AUIPC";
      default   : opcode_to_string = "???";
    endcase
  endfunction

  typedef struct packed {
    logic [6:0] funct7;
    logic [4:0] rs2;
    logic [4:0] rs1;
    logic [2:0] funct3;
    logic [4:0] rd;
    opcode opcode;
  } instruction_t;

endpackage
