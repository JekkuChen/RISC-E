module STAGE_DECODE (
    input wire clock,
    
    input wire [31:0] inst,    // 32 bit instuction from memory
    input wire reg_write_enable,
    input wire [31:0] data_rd,
    input wire [4:0] addr_rd,

    output reg [6:0] opcode,
    output reg [4:0] rd,
    output reg [2:0] funct3,
    output reg [6:0] funct7,
    output reg [31:0] imm,
    output reg [4:0] shamt,

    output wire [31:0] data_rs1,
    output wire [31:0] data_rs2,

    output reg [4:0] addr_rs1,
    output reg [4:0] addr_rs2,

    output wire rs1_dependency,
    output wire rs2_dependency
);
    
    localparam LUI = 7'b0110111;    // U-Type
    localparam AUIPC = 7'b0010111;  // U-Type
    localparam JAL = 7'b1101111;     // J-Type
    localparam BRANCH = 7'b1100011;  // B-Type
    localparam JALR = 7'b1100111;    // I-Type
    localparam LOAD = 7'b0000011;    // I-Type
    localparam OP_IMM = 7'b0010011;  // I-Type
    localparam STORE = 7'b0100011;   // S-Type
    localparam OP = 7'b0110011;      // R-Type
    localparam ECALL = 7'b1110011;   // R-Type


    wire u_type;
    wire j_type;
    wire b_type;
    wire i_type;
    wire s_type;
    wire r_type;

    assign u_type = (opcode == LUI) || (opcode == AUIPC);
    assign j_type = (opcode == JAL);
    assign b_type = (opcode == BRANCH);
    assign i_type = (opcode == JALR) || (opcode == LOAD) || (opcode == OP_IMM);
    assign s_type = (opcode == STORE);
    assign r_type = (opcode == OP) || (opcode == ECALL);

    assign rs1_dependency = (i_type || rs2_dependency);
    assign rs2_dependency = (r_type || s_type || b_type); 

    always @(*) begin // Immediates

        // Defaults
        imm[0] = 1'b0;              // B/U/J type
        imm[4:1] = inst[11:8];      // B/S type
        imm[10:5] = inst[30:25];    // B/S/I/J type
        imm[11] = inst[31];         // S/I type
        imm[19:12] = {8{inst[31]}};      // B/S/I type
        imm[30:20] = {11{inst[31]}};      // B/S/I/J  type
        imm[31] = inst[31];         // All types

        // For immediate 0
        case(1)
            u_type: begin
                imm[11:1] = 11'b0;
                imm[30:12] = inst[30:12];   
            end
            j_type: begin
                imm[4:1] = inst[24:21];
                imm[11] = inst[20];
                imm[19:12] = inst[19:12];  
            end
            i_type: begin
                imm[0] = inst[20];
                imm[4:1] = inst[24:21];
            end
            s_type: begin
                imm[0] = inst[7];
            end
            b_type: begin
                imm[11] = inst[7];
            end
            default:;
        endcase
    end

    always @(*) begin // Decoder
        opcode = inst[6:0];
        rd = inst[11:7];
        funct3 = inst[14:12];
        addr_rs1 = inst[19:15];
        addr_rs2 = inst[24:20];
        funct7 = inst[31:25]; 
        shamt = inst[24:20]; 
    end

  register_file reg_file(
    .clock(clock),
    .addr_rs1(addr_rs1),
    .addr_rs2(addr_rs2),
    .addr_rd(addr_rd),
    .data_rd(data_rd),
    .R_data_rs1(data_rs1),
    .R_data_rs2(data_rs2),
    .write_enable(reg_write_enable)    
  );

endmodule
