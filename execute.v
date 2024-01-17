module STAGE_EXECUTE (
    input wire [31:0] pc,  // pc - 32 bits

    input wire [6:0] opcode,
    input wire [31:0] rs1,
    input wire [31:0] rs2,
    input wire [2:0] funct3,
    input wire [6:0] funct7,
    input wire [31:0] imm,
    input wire [4:0] shamt,
    input wire [4:0] addr_rd,

    output reg [31:0] alu_res,
    output reg br_taken,
    output wire reg_write_back,
    output wire dmem_read_write
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

    assign reg_write_back = (r_type || i_type || u_type || j_type) && (addr_rd != 0);
    assign dmem_read_write = s_type;

    // ALU RES
    always @(*) begin
        case(1)
            u_type: begin
                if (opcode == AUIPC) begin
                    alu_res = imm + pc;
                end else begin
                    alu_res = imm;
                end
            end
            r_type: begin
                    alu_res = rs1 + rs2; // ECALL will result in a 0
                    case(funct3) 
                        3'b000: begin 
                            if (funct7[5]) alu_res = rs1 - rs2; // SUB
                            else alu_res = rs1 + rs2; // ADD
                        end
                        3'b001: alu_res = rs1 << rs2[4:0]; // SLL
                        3'b010: alu_res = ($signed(rs1) < $signed(rs2)) ? 1 : 0; // SLT
                        3'b011: alu_res = (rs1 < rs2) ? 1 : 0; // SLTU
                        3'b100: alu_res = rs1 ^ rs2 ; // XOR
                        3'b101: begin 
                            alu_res = rs1 >> rs2[4:0]; // SRL
                            if (funct7[5]) alu_res = $signed(rs1) >>> rs2[4:0]; // SRA
                        end
                        3'b110: alu_res = rs1 | rs2; // OR
                        3'b111: alu_res = rs1 & rs2; // AND
                    endcase
            end
            i_type: begin
                if (opcode == OP_IMM) begin
                    alu_res = rs1 + imm;
                    case(funct3) 
                        3'b000: alu_res = rs1 + imm; // ADDI
                        3'b001: alu_res = rs1 << shamt; // SLLI
                        3'b010: alu_res = ($signed(rs1) < $signed(imm)) ? 1 : 0; // SLTI
                        3'b011: alu_res = (rs1 < imm) ? 1 : 0; // SLTIU 
                        3'b100: alu_res = rs1 ^ imm ; // XORI
                        3'b101: begin 
                            alu_res = rs1 >> shamt; // SRLI
                            if (funct7[5]) alu_res = $signed(rs1) >>> shamt; // SRAI
                        end
                        3'b110: alu_res = rs1 | imm; // ORI
                        3'b111: alu_res = rs1 & imm; // ANDI
                    endcase
                end else begin // Load or JALR instruction
                    alu_res = imm + rs1;
                end
            end
            s_type: begin
                alu_res = imm + rs1; // SW, SH, SB instructions
            end
            default: begin // j_type and b_type
                alu_res = imm + pc;
            end
        endcase
    end

    // BR Taken
    always @(*) begin
        if (opcode == BRANCH) begin
            case(funct3) 
                3'b000: br_taken = (rs1 == rs2) ? 1'b1 : 1'b0; // BEQ
                3'b001: br_taken = (rs1 != rs2) ? 1'b1 : 1'b0; // BNE
                3'b100: br_taken = ($signed(rs1) < $signed(rs2)) ? 1'b1 : 1'b0; // BLT
                3'b101: br_taken = ($signed(rs1) >= $signed(rs2)) ? 1'b1 : 1'b0; // BGE
                3'b110: br_taken = (rs1 < rs2) ? 1'b1 : 1'b0; // BLTU
                3'b111: br_taken = (rs1 >= rs2) ? 1'b1 : 1'b0; // BGEU
                default: br_taken = 1'b0;
            endcase
        end else if ((opcode == JALR) || (opcode == JAL)) begin
            br_taken = 1;
        end else begin
            br_taken = 0;   // defaults to branch not taken
        end
    end
endmodule