module STAGE_FETCH (
  input wire clock,
  input wire [31:0] pc,
  input wire [31:0] imem_data_in,
  input wire imem_read_write,
  input wire imem_enable,
  output wire [31:0] imem_data_out
);

  imemory imem(
    .clock(clock),
    .address(pc),
    .data_in(imem_data_in),
    .data_out(imem_data_out),
    .read_write(imem_read_write),
    .enable(imem_enable)
  );

endmodule