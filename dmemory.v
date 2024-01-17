module byte_mem(
    input                                 clock,
    input      [$clog2(`MEM_DEPTH/4)-1:0] address,
    input      [7:0]                      data_in,
    input                                 read_write,
    output reg [7:0]                      data_out
);

    (* ram_style = "block" *) reg[7:0] mem[0:`MEM_DEPTH / 4 - 1];

    always @(posedge clock) begin
        if (read_write == 1) begin
            mem[address] <= data_in;
        end
        data_out = mem[address];
    end
endmodule

module dmemory(
    input  wire        clock,
    input  wire        read_write,
    input  wire [31:0] address,
    input  wire [1:0]  access_size,
    input  wire [31:0] data_in,
    output wire [31:0] data_out
);

    wire [31:0] address_2;
    wire [31:0] data_in_2;
    wire        read_write_2;

    localparam  START_ADDR = 32'h01000000;
    wire [31:0] addr_off   = address - START_ADDR;
    wire [31:0] addr_off_2 = address_2 - START_ADDR;

    wire [$clog2(`MEM_DEPTH / 4) - 1:0] ea;
    wire [$clog2(`MEM_DEPTH / 4) - 1:0] ea_2;

    wire [31:0] data_out_sig;
    wire [31:0] data_in_rot;
    wire [1:0]  ea_off;
    wire [3:0]  be, be_rot, be_pre;
    reg  [4:0]  shiftamt;
    reg  [4:0]  shiftamt_reg;

    assign ea       = addr_off[$clog2(`MEM_DEPTH/4) + 2 - 1 : 2];
    assign ea_off   = addr_off[1:0];
    assign ea_2     = addr_off_2[$clog2(`MEM_DEPTH/4) + 2 - 1 : 2];
    assign data_out = (data_out_sig  >> shiftamt_reg) | (data_out_sig << (32 - shiftamt_reg));
    assign be       = (access_size == 2'h2 ? 4'b1111 :
                      (access_size == 2'h1 ? 4'b0011 :
                      (access_size == 2'h0 ? 4'b0001 : 4'b0000)));
    assign be_pre = (be << ea_off);
    assign be_rot = be_pre | (be >> (4 - ea_off));
    assign data_in_rot = (data_in << shiftamt) | (data_in >> (32 - shiftamt));


    genvar i;
    generate
    for(i = 0; i < 4; i = i + 1) begin : bm
        wire [7:0] data_out;
        reg  [7:0] bm_data_in;
        reg        rw;

        reg  [$clog2(`MEM_DEPTH / 4) - 1 : 0] addr;

        always @(*) begin
            if(read_write_2) begin
                addr = ea_2;
                bm_data_in = data_in_2[i * 8 +: 8];
                rw = 1;
            end else begin
                bm_data_in = data_in_rot[i * 8 +: 8];
                rw = be_rot[i] && read_write;
                if(be_pre[i] == 1'b0 && be_rot[i] == 1'b1) begin
                    addr = ea + 1;
                end else begin
                    addr = ea;
                end
            end
        end
        byte_mem memory(
            .clock(clock),
            .address(addr),
            .data_in(bm_data_in),
            .data_out(data_out_sig[i * 8 +: 8]),
            .read_write(rw)
        );
    end
    endgenerate

    always @(*) begin
        shiftamt = {3'b0, ea_off};
        shiftamt = shiftamt << 3;

    end

    always @(posedge clock) begin
        shiftamt_reg <= shiftamt;
    end
endmodule
