module STAGE_WRITEBACK  (
  input wire [31:0] pc,
  input wire [6:0] opcode,
  input wire [2:0] funct3,
  input wire [31:0] alu_res,
  input wire [31:0] dmem_out,

  output reg [31:0] new_reg_data

);  

  localparam LOAD = 7'b0000011;    // I-Type
  localparam JALR = 7'b1100111;    // I-Type
  localparam JAL = 7'b1101111;     // J-Type


  always @(*) begin // To strip and zero/bit extend Loads
    if (opcode == LOAD) begin
      case (funct3) 
        3'b000: new_reg_data = {{24{dmem_out[7]}}, dmem_out[7:0]}; // LB
        3'b001: new_reg_data = {{16{dmem_out[15]}}, dmem_out[15:0]}; // LH
        3'b010: new_reg_data = dmem_out[31:0]; // LW
        3'b100: new_reg_data = {{24{1'b0}}, dmem_out[7:0]}; // LBU
        3'b101: new_reg_data = {{16{1'b0}}, dmem_out[15:0]}; // LHU
        default: new_reg_data = dmem_out[31:0]; // LW
      endcase
    end else if ((opcode == JAL) || (opcode == JALR)) begin
      new_reg_data = pc + 4;
    end else begin // If it isn't load or jump, writeback uses alu_res, if store or branch, writeback enable will be reset
      new_reg_data = alu_res;
    end
  end


endmodule