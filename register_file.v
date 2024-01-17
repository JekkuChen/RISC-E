module register_file
(
    input wire clock,
    input wire [4:0] addr_rs1,
    input wire [4:0] addr_rs2,
    input wire [4:0] addr_rd,
    input wire [31:0] data_rd,
    input wire write_enable,

    output reg [31:0] R_data_rs1,
    output reg [31:0] R_data_rs2 
);

(* ram_style = "block" *) reg [31:0] registers [31:0];

initial begin : reg_init
    integer ii;
    for (ii = 0; ii < 32; ii = ii + 1) begin
        registers[ii] = 0;
    end
    registers[2] = 32'h01000000 + `MEM_DEPTH;
end


always @(posedge clock) begin
    R_data_rs1 <= registers[addr_rs1];
    R_data_rs2 <= registers[addr_rs2];
end

always @(posedge clock) begin
    if (write_enable) begin
        registers[addr_rd] <= data_rd;
    end
end



endmodule
