module pd(
  input clock,
  input reset
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

  localparam NOP = 32'h13; // Pseudocode for NOP command 

  // Reg/Wires for Kill/Stall
  wire d_kill;
  wire x_kill;
  wire f_d_stall;

  reg X_kill;
  reg D_kill;
  
  reg f_d_stall_rs1;
  reg f_d_stall_rs2;

  // PC
  reg [31:0] F_pc; 
  reg [31:0] D_pc; 
  reg [31:0] X_pc;
  reg [31:0] M_pc; 
  reg [31:0] W_pc;  

  wire [31:0] f_in_pc; 
  wire [31:0] d_in_pc; 
  wire [31:0] x_in_pc;
  wire [31:0] m_in_pc; 
  wire [31:0] w_in_pc;  

  // FETCH
  reg [31:0] F_imem_data_in;
  reg F_imem_read_write;
  reg f_imem_enable;

  wire [31:0] f_in_imem_data_in;
  wire f_in_imem_read_write;
  wire f_in_imem_enable;
  wire [31:0] f_out_imem_data_out;
  wire f_in_branch_taken;
  wire [31:0] f_in_new_pc;

  // DECODE
  wire [6:0] d_out_opcode;
  wire [4:0] d_out_rd;
  wire [2:0] d_out_funct3;
  wire [6:0] d_out_funct7;
  wire [31:0] d_out_imm;
  wire [4:0] d_out_shamt;
  
  wire [4:0] d_out_addr_rs1;
  wire [4:0] d_out_addr_rs2;

  wire [31:0] d_out_data_rs1;
  wire [31:0] d_out_data_rs2;

  reg [31:0] d_in_inst;
  reg D_pipeline_valid;

  wire [31:0] d_in_data_rd;
  wire [4:0] d_in_addr_rd;
  wire d_in_reg_write_enable;

  wire d_rs1_dependency;
  wire d_rs2_dependency;

  // EXECUTE
  reg [6:0] X_opcode;
  reg [2:0] X_funct3;
  reg [6:0] X_funct7;
  reg [31:0] X_imm;
  reg [4:0] X_shamt;
  reg [4:0] X_addr_rd;

  reg [4:0] X_addr_rs1;
  reg [4:0] X_addr_rs2;

  reg [31:0] x_data_rs1;
  reg [31:0] x_data_rs2;

  wire [6:0] x_in_opcode;
  wire [31:0] x_in_data_rs1;
  wire [31:0] x_in_data_rs2;
  wire [2:0] x_in_funct3;
  wire [6:0] x_in_funct7;
  wire [31:0] x_in_imm;
  wire [4:0] x_in_shamt;
  wire [4:0] x_in_addr_rd;

  wire [31:0] x_out_alu_res;
  wire x_out_br_taken;
  wire x_out_reg_write_back;
  wire x_out_dmem_read_write;

  // MEMORY
  reg [6:0] M_opcode;
  reg [2:0] M_funct3;
  reg [31:0] M_data_in;
  reg [31:0] M_address; // alu result
  reg M_reg_write_back;
  reg M_dmem_read_write;
  reg [4:0] M_addr_rd;
  
  reg [4:0] M_addr_rs2;
  reg [4:0] M_addr_rs1;

  reg [31:0] m_data_in;

  wire [31:0] m_in_dmem_data_in;
  wire [31:0] m_out_dmem_data_out;
  wire [31:0] m_in_dmem_address;
  wire [1:0] m_in_dmem_access_size;
  wire m_in_dmem_read_write;

  // WRITEBACK 
  reg [6:0] W_opcode;
  reg [2:0] W_funct3;
  reg [31:0] W_alu_res; 
  reg W_reg_write_back;
  reg [4:0] W_addr_rd;
  
  wire [6:0] w_in_opcode;
  wire [2:0] w_in_funct3;
  wire [31:0] w_in_memory_data_out;
  wire [31:0] w_in_alu_res;
  wire [31:0] w_out_reg_data;

  // Kill
  assign f_d_stall = f_d_stall_rs1 | f_d_stall_rs2;
  assign d_kill = f_in_branch_taken; // WIP I believe this is correct but may not be depending on how it is changed
  assign x_kill = f_in_branch_taken | f_d_stall; // Notice that when we stall we give X a NOP, similar to a kill

  // PC Sequential Block
  assign f_in_pc = F_pc; 
  assign d_in_pc = D_pc; 
  assign x_in_pc = X_pc;
  assign m_in_pc = M_pc; 
  assign w_in_pc = W_pc;  

  always @(posedge clock) begin
    if (reset) begin
      F_pc <= 32'h01000000;
      D_pc <= 32'h01000000;
      X_pc <= 32'h01000000;
      M_pc <= 32'h01000000;
      W_pc <= 32'h01000000;
    end else begin
      if (f_in_branch_taken) begin // Branching to new value should have take precedence over a stall (basically kill)
        F_pc <= f_in_new_pc;
      end else if (!f_d_stall) begin
        F_pc <= F_pc + 4; 
      end

      if (!f_d_stall) begin
        D_pc <= F_pc;
      end

      X_pc <= D_pc;
      M_pc <= X_pc;
      W_pc <= M_pc;
    end
  end

  // FETCH STAGE
  assign f_in_imem_data_in = F_imem_data_in;
  assign f_in_imem_read_write = F_imem_read_write;
  assign f_in_imem_enable = f_imem_enable;
  assign f_in_branch_taken = x_out_br_taken;
  assign f_in_new_pc = x_out_alu_res;

  always @(posedge clock) begin
    if (reset) begin
			F_imem_data_in <= 32'b0;
			F_imem_read_write <= 1'b0;
    end else begin
    end
  end

  always @(*) begin
    if (f_d_stall) begin
      f_imem_enable = 1'b0;
    end else begin
      f_imem_enable = 1'b1;
    end
  end

  STAGE_FETCH fetch (
    .clock(clock),
    .pc(f_in_pc),
    .imem_data_in(f_in_imem_data_in),
    .imem_read_write(f_in_imem_read_write),
    .imem_enable(f_in_imem_enable),
    .imem_data_out(f_out_imem_data_out)
  );

 // DECODE STAGE
  assign d_in_reg_write_enable = W_reg_write_back;
  assign d_in_data_rd = w_out_reg_data;
  assign d_in_addr_rd = W_addr_rd;

  always @(posedge clock) begin
    if (reset) begin
      D_kill <= 1'b0;
      D_pipeline_valid <= 1'b0;
    end else begin     
      D_pipeline_valid <= 1'b1;
      if (d_kill) begin
        D_kill <= 1'b1;
      end else begin
        D_kill <= 1'b0;
      end
    end
  end

  always @(*) begin
    if (D_kill || !D_pipeline_valid) begin
      d_in_inst = NOP;
    end else begin
      d_in_inst = f_out_imem_data_out; // If we are stalling, iMem holds f_out_imem_data_out
    end
  end

  always @(*) begin 

    f_d_stall_rs1 = 0;
    f_d_stall_rs2 = 0;

    if (d_rs1_dependency) begin // RS1 Stall
      if ((d_out_addr_rs1 == X_addr_rd) && x_out_reg_write_back ) begin // RS1 in X
        if (X_opcode == LOAD) begin
          f_d_stall_rs1 = 1;
        end
      end else if ((d_out_addr_rs1 == M_addr_rd) && (M_reg_write_back)) begin // RS1 in M
        f_d_stall_rs1 = 0;
      end else if ((d_out_addr_rs1 == W_addr_rd) && (W_reg_write_back)) begin // RS1 in WB
        f_d_stall_rs1 = 1;
      end
    end

    if (d_rs2_dependency) begin // RS2 Stall
      if ((d_out_addr_rs2 == X_addr_rd) && x_out_reg_write_back) begin // RS2 in X
        if ((X_opcode == LOAD) && (d_out_opcode != STORE)) begin
          f_d_stall_rs2 = 1;
        end 
      end else if ((d_out_addr_rs2 == M_addr_rd) && (M_reg_write_back)) begin // RS2 in M
        f_d_stall_rs2 = 0;
      end else if ((d_out_addr_rs2 == W_addr_rd) && (W_reg_write_back)) begin // RS2 in WB
        f_d_stall_rs2 = 1;
      end 
    end
  end

  STAGE_DECODE decode(
    .clock(clock),
    .inst(d_in_inst),
    .reg_write_enable(d_in_reg_write_enable),
    .addr_rd(d_in_addr_rd),
    .data_rd(d_in_data_rd),

    .opcode(d_out_opcode),
    .rd(d_out_rd),
    .funct3(d_out_funct3),
    .funct7(d_out_funct7),
    .imm(d_out_imm),
    .shamt(d_out_shamt),
    .addr_rs1(d_out_addr_rs1),
    .addr_rs2(d_out_addr_rs2),
    .data_rs1(d_out_data_rs1),
    .data_rs2(d_out_data_rs2),

    .rs1_dependency(d_rs1_dependency),
    .rs2_dependency(d_rs2_dependency)
  );

  // Execute

  assign x_in_opcode = X_opcode;
  assign x_in_data_rs1 = x_data_rs1;
  assign x_in_data_rs2 = x_data_rs2;
  assign x_in_funct3 = X_funct3;
  assign x_in_funct7 = X_funct7;
  assign x_in_imm = X_imm;
  assign x_in_shamt = X_shamt;
  assign x_in_addr_rd = X_addr_rd;

  always @(*) begin
    if (!X_kill) begin
        x_data_rs1 = d_out_data_rs1; // Since BRAM (d_out_data_rs1 is correct cycle)
        x_data_rs2 = d_out_data_rs2; // Since BRAM (d_out_data_rs2 is correct cycle)
    end else begin
        x_data_rs1 = 32'b0;
        x_data_rs2 = 32'b0;
    end


    if ((X_addr_rs1 == M_addr_rd) && (M_reg_write_back) && (M_opcode != LOAD)) begin // M -> X
      x_data_rs1 = M_address;
    end else if ((X_addr_rs1 == W_addr_rd) && (W_reg_write_back)) begin // W -> X 
      x_data_rs1 = w_out_reg_data; // This might need some optimization
    end


    if ((X_addr_rs2 == M_addr_rd) && (M_reg_write_back) && (M_opcode != LOAD)) begin // M -> X
      x_data_rs2 = M_address;
    end else if ((X_addr_rs2 == W_addr_rd) && (W_reg_write_back)) begin // W -> X
      x_data_rs2 = w_out_reg_data; // This might need some optimization
    end 
  end

  always @(posedge clock) begin
    if (reset) begin
      X_opcode <= 0;
      X_funct3 <= 0;
      X_funct7 <= 0;
      X_imm <= 0;
      X_shamt <= 0;
      X_addr_rd <= 0;
      X_addr_rs1 <= 0;
      X_addr_rs2 <= 0;
      X_kill <= 0;
    end else begin
      if (x_kill) begin
        X_addr_rd <= 5'b0;
        X_opcode <= 7'b0010011;
        X_funct3 <= 3'b000;
        X_funct7 <= 7'b0;
        X_imm <= 32'b0;
        X_shamt <= 5'b0;
        X_addr_rs1 <= 5'b0;
        X_addr_rs2 <= 5'b0;   
        X_kill <= 1'b1;     
      end else begin
        X_addr_rd <= d_out_rd;
        X_opcode <= d_out_opcode;
        X_funct3 <= d_out_funct3;
        X_funct7 <= d_out_funct7;
        X_imm <= d_out_imm;
        X_shamt <= d_out_shamt;
        X_addr_rs1 <= d_out_addr_rs1;
        X_addr_rs2 <= d_out_addr_rs2;
        X_kill <= 1'b0;
      end

    end
  end

  STAGE_EXECUTE execute(
    .pc(x_in_pc),  // pc - 32 bits
    .opcode(x_in_opcode),
    .rs1(x_in_data_rs1),
    .rs2(x_in_data_rs2),
    .funct3(x_in_funct3),
    .funct7(x_in_funct7),
    .imm(x_in_imm),
    .shamt(x_in_shamt),
    .addr_rd(x_in_addr_rd),
    .alu_res(x_out_alu_res),
    .br_taken(x_out_br_taken),
    .reg_write_back(x_out_reg_write_back),
    .dmem_read_write(x_out_dmem_read_write)
  );

  // MEMORY STAGE
  assign m_in_dmem_address = M_address;
  assign m_in_dmem_data_in = m_data_in;
  assign m_in_dmem_access_size = M_funct3[1:0];
  assign m_in_dmem_read_write = M_dmem_read_write ; // We only write on Store command

  always @(*) begin
    m_data_in = M_data_in;

    if ((M_addr_rs2 != 5'b0) && (M_addr_rs2 == W_addr_rd) && (W_reg_write_back)) begin // W -> M
      m_data_in = w_out_reg_data; // This might need some optimization
    end

  end

  always @(posedge clock) begin
    if (reset) begin
      M_opcode <= 0;
      M_funct3 <= 0;
      M_address <= 0;
      M_data_in <= 0;
      M_reg_write_back <= 0;
      M_dmem_read_write <= 0;
      M_addr_rd <= 0;
      M_addr_rs2 <= 0;
      M_addr_rs1 <= 0;
    end else begin
      // Not Used in Memory
      M_reg_write_back <= x_out_reg_write_back;
      M_opcode <= X_opcode;
      M_addr_rd <= X_addr_rd;
      M_addr_rs2 <= X_addr_rs2;
      M_addr_rs1 <= X_addr_rs1;

      // Used in Memory
      M_dmem_read_write <= x_out_dmem_read_write;
      M_funct3 <= X_funct3;
      M_address <= x_out_alu_res;
      M_data_in <= x_in_data_rs2;


    end
  end

  STAGE_MEMORY memory(
    .clock(clock),
    .address(m_in_dmem_address),
    .data_in(m_in_dmem_data_in),
    .read_write(m_in_dmem_read_write),
    .access_size(m_in_dmem_access_size),
    .data_out(m_out_dmem_data_out)
  );

  // WRITEBACK STAGE

  assign w_in_opcode = W_opcode;
  assign w_in_funct3 = W_funct3;
  assign w_in_memory_data_out = m_out_dmem_data_out;
  assign w_in_alu_res = W_alu_res;

  always @(posedge clock) begin
    if (reset) begin
      W_opcode <= 0;
      W_funct3 <= 0;
      W_alu_res <= 0;
      W_reg_write_back <= 0; 
      W_addr_rd <= 0;
    end else begin
      // Not used in Writeback
      W_reg_write_back <= M_reg_write_back;
      W_addr_rd <= M_addr_rd;

      // Used in Writeback
      W_opcode <= M_opcode;
      W_funct3 <= M_funct3;
      W_alu_res <= M_address; // Also used for new_pc
    end
  end

  STAGE_WRITEBACK writeback (
    .pc(w_in_pc),
    .opcode(w_in_opcode),
    .funct3(w_in_funct3),
    .alu_res(w_in_alu_res),
    .dmem_out(w_in_memory_data_out),
    .new_reg_data(w_out_reg_data)
  );

endmodule
