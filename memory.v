module STAGE_MEMORY (
  input wire clock,
  input wire [31:0] address,
  input wire [31:0] data_in,
  input wire read_write,
  input wire [1:0] access_size,

  output wire [31:0] data_out
);

  dmemory dmem(
    .clock(clock),
    .address(address),
    .data_in(data_in),
    .data_out(data_out),
    .access_size(access_size),
    .read_write(read_write)
  );

endmodule