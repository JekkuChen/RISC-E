module imemory(
    input             clock,
    input      [31:0] address,
    input      [31:0] data_in,
    input             read_write,
    input             enable,
    output reg [31:0] data_out
);
    wire       [31:0] address_2;
    wire       [31:0] ea;
    wire       [31:0] ea_2;
    wire       [31:0] data_in_2;
    wire              read_write_2;
    localparam        START_ADDR = 32'h01000000;

    (* ram_style = "block" *) reg [31:0] mem[0:`MEM_DEPTH / 4 - 1];

    initial begin
        $readmemh(`MEM_PATH, mem);
    end

    assign ea = (address - START_ADDR) >> 2;
    assign ea_2 = (address_2 - START_ADDR) >> 2;

    always @(posedge clock) begin
        if (enable) begin
            if (read_write_2 == 1) begin
                mem[ea_2] <= data_in_2;
            end
            data_out <= mem[ea];
        end
    end

endmodule