module cpu(input clk, input rst_n, input load, input start, input [15:0] instr,
           output waiting, output [15:0] out, output N, output V, output Z);

    wire [15:0] sximm5, sximm8; //mdata;
    wire [15:0] ir;
    wire [1:0] reg_sel, ALU_op, shift_op, wb_sel;
    wire [2:0] opcode, r_addr, w_addr;
    //wire [7:0] pc;
    wire w_en, en_A, en_B, en_C, en_status, sel_A, sel_B, Z_out, V_out, N_out;

    reg [15:0] mdata = 16'd0;
    reg [7:0] pc = 8'd0;

    ireg instr_reg(.clk, .instr, .load, .ir);
    idecoder instr_dec(.ir, .reg_sel, .opcode, .ALU_op, .shift_op, .sximm5, .sximm8, .r_addr, .w_addr);
    datapath dp(.clk, .mdata, .pc, .wb_sel, .w_addr, .w_en, .r_addr, .en_A, .en_B, .shift_op, .sel_A, .sel_B, .ALU_op, .en_C, .en_status, .sximm8, .sximm5, .datapath_out(out), .Z_out(Z), .N_out(N), .V_out(V));
    controller control(.clk, .rst_n, .start, .opcode, .ALU_op, .shift_op, .Z, .N, .V, .waiting, .reg_sel, .wb_sel, .w_en, .en_A, .en_B, .en_C, .en_status, .sel_A, .sel_B);

endmodule: cpu


module ireg(input clk, input [15:0] instr, input load, output reg [15:0] ir);
    always_ff @(posedge clk) begin
        if (load) begin
            ir <= instr;
        end
    end
endmodule: ireg

module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr);
  // your implementation here
  assign r_addr = (reg_sel == 2'b00) ? ir[2:0] : ((reg_sel == 2'b01) ? ir[7:5] : ((reg_sel == 2'b10)? ir[10:8] : 3'd0));
  assign w_addr = (reg_sel == 2'b00) ? ir[2:0] : ((reg_sel == 2'b01) ? ir[7:5] : ((reg_sel == 2'b10)? ir[10:8] : 3'd0)); 
  assign shift_op = ir[4:3];
  assign sximm8 = $signed(ir[7:0]);
  assign sximm5 = $signed(ir[4:0]);
  assign ALU_op = ir[12:11];
  assign opcode = ir[15:13];

endmodule: idecoder

module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, input en_status,
		input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out, output Z_out, output N_out, output V_out);
  
    reg [15:0] w_data;
    reg [15:0] r_data;
    reg [15:0] val_A;
    reg [15:0] val_B;
    reg [15:0] B_out;
    reg [15:0] input_A;
    reg [15:0] input_B;
    reg [15:0] ALU_out;
    reg Z, N, V;
    //reg [15:0] mux_out;

    //writeback_mux mux_writeback(.mdata, .sximm8, .pc, .datapath_out, .mux_out(w_data), .wb_sel); 
    always_comb begin
        case(wb_sel)
        2'b00: w_data = datapath_out;
        2'b01: w_data = {8'd0, pc};
        2'b10: w_data = sximm8;
        2'b11: w_data = mdata;
        default: w_data = 16'bxxxxxxxxxxxxxxxx;
        endcase 
    end

    regfile register(.w_data, .w_addr, .w_en, .r_addr, .clk, .r_data);
    reg_en A(.clk, .in(r_data), .en(en_A), .out(input_A));
    reg_en B(.clk, .in(r_data), .en(en_B), .out(B_out));
    shifter shift(.shift_in(B_out), .shift_op, .shift_out(input_B));
    mux_A multiplexer_A(.sel_A, .A_in(input_A), .val_A);
    mux_B multiplexer_B(.sel_B, .B_in(input_B), .sximm5, .val_B);
    ALU alu(.val_A, .val_B, .ALU_op, .ALU_out, .Z, .N, .V);
    status status_reg(.clk, .Z, .N, .V, .en_status, .Z_out, .N_out, .V_out);
    reg_en C(.clk, .in(ALU_out), .en(en_C), .out(datapath_out));

endmodule: datapath

module writeback_mux(input [15:0] mdata, input [15:0] sximm8, input [7:0] pc, input [15:0] datapath_out, output reg [15:0] mux_out, input [1:0] wb_sel);  //datapath_out = C in figure
//can also use assign statment-- check later
always_comb begin
    case(wb_sel)
    2'b00: mux_out = datapath_out;
    2'b01: mux_out = {8'd0, pc};
    2'b10: mux_out = sximm8;
    2'b11: mux_out = mdata;
    default: mux_out = 16'bxxxxxxxxxxxxxxxx;
    endcase 
end
endmodule: writeback_mux 

module regfile(input [15:0] w_data, input [2:0] w_addr, input w_en, input [2:0] r_addr, input clk, output [15:0] r_data);
    reg [15:0] m[0:7];
    assign r_data = m[r_addr];
    always_ff @(posedge clk) if (w_en) m[w_addr] <= w_data;
endmodule: regfile

module reg_en(input clk, input [15:0] in, input en, output reg [15:0] out); 
    always_ff @(posedge clk) begin
        if (en) begin
            out <= in;
        end
    end
endmodule: reg_en

module shifter(input [15:0] shift_in, input [1:0] shift_op, output reg [15:0] shift_out);
    always_comb begin
        case (shift_op) 
            2'b00: shift_out = shift_in;
            2'b01: shift_out = shift_in << 1;
            2'b10: shift_out = shift_in >> 1;
            2'b11: begin
                shift_out = shift_in >> 1;
                shift_out[15] = shift_in[15];
            end
            default: shift_out = 16'bxxxxxxxxxxxxxxxx;
        endcase
    end
endmodule: shifter

module mux_A(input sel_A, input [15:0] A_in, output [15:0] val_A);   //A_in = A_out
    assign val_A = sel_A ? 16'd0 : A_in;
endmodule

module mux_B (input sel_B, input [15:0] B_in, input [15:0] sximm5, output [15:0] val_B); // B_in = shift_out
    assign val_B = sel_B ? sximm5 : B_in;
endmodule

module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out, output Z, output N, output V); 

 reg [15:0] alu_out;
 assign ALU_out = alu_out;

  always_comb begin 
        case(ALU_op)
        2'b00: alu_out = val_A + val_B;
        2'b01: alu_out = val_A - val_B;
        2'b10: alu_out = val_A & val_B;
        2'b11: alu_out = ~val_B;
        default: alu_out = 16'bxxxxxxxxxxxxxxxx;
        endcase
  end
  
  assign Z = (ALU_out == 16'd0)? 1'b1: 1'b0;
  assign N = (ALU_out[15] == 1'b1)? 1'b1 : 1'b0; //or check for < 16'd0
  //overflow bit is 1 when both inputs are positive but output is "negative" or both inputs are negative and output is "positive"
  assign V = (~ALU_out[15] & val_A[15] & val_B[15]) | (ALU_out[15] & ~val_A[15] & ~val_B[15]);
endmodule: ALU

module status(input clk, input Z, input N, input V, input en_status, output reg Z_out, output reg N_out, output reg V_out);

    always_ff @(posedge clk) begin
        if (en_status) begin
            Z_out <= Z;
            N_out <= N;
            V_out <= V;
        end
    end
endmodule: status

module controller(input clk, input rst_n, input start,
                  input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  input Z, input N, input V,
                  output waiting,
                  output [1:0] reg_sel, output [1:0] wb_sel, output w_en,
                  output en_A, output en_B, output en_C, output en_status,
                  output sel_A, output sel_B);
  // your implementation here

  reg waiting_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x;
  reg [1:0] reg_sel_x, wb_sel_x;

  assign waiting = waiting_x;
  assign w_en = w_en_x;
  assign en_A = en_A_x;
  assign en_B = en_B_x;
  assign en_C = en_C_x;
  assign en_status = en_status_x;
  assign sel_A = sel_A_x;
  assign sel_B = sel_B_x;
  assign reg_sel = reg_sel_x;
  assign wb_sel = wb_sel_x;

  enum reg [3:0] {
    Waiting = 4'b0000,
    Decode = 4'b0001,
    WriteImm8 = 4'b0010,
    StoreRn = 4'b0011,
    StoreRm = 4'b0100,
    Add = 4'b0101,
    Cmp = 4'b0110,
    And = 4'b0111,
    Mvn = 4'b1000,
    sh_Rm = 4'b1001,
    Write_final = 4'b1010} state;

    always_ff @(posedge clk) begin
      if (~rst_n) begin
          state <= Waiting;
      end else begin
        
        case(state)
        Waiting: if(start == 1'b1) state <= Decode;
                 else state <= Waiting;

        Decode: if(opcode == 3'b110 && ALU_op == 2'b10) state <= WriteImm8;
                else if({opcode,ALU_op} == {3'b101, 2'b11} || {opcode,ALU_op} == {3'b110, 2'b00}) state <= StoreRm;
                else state <= StoreRn;

        WriteImm8: state <= Waiting;

        StoreRn: state <= StoreRm;

        StoreRm: if(opcode == 3'b101 && ALU_op == 2'b00) state <= Add;
        else if (opcode == 3'b101 && ALU_op == 2'b01) state <= Cmp;
        else if (opcode == 3'b101 && ALU_op == 2'b10) state <= And;
        else if (opcode == 3'b101 && ALU_op == 2'b11) state <= Mvn;
        else state <= sh_Rm;

        Add: state <= Write_final;

        Cmp: state <= Write_final;

        And: state <= Write_final;

        Mvn: state <= Write_final;

        sh_Rm: state <= Write_final;

        Write_final: state <= Waiting;

        default: state <= Waiting;
      endcase
      end
    end

    always_comb begin

      casex(state)

        Waiting: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b1, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        Decode: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};
        WriteImm8: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'b10, 2'b10, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
        StoreRn: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'b10, 2'bxx, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
        StoreRm: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'b00, 2'bxx, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0};
        Add: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0};
        Cmp: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0};
        And: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0};
        Mvn: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'bx, 1'b0};
        sh_Rm: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'bxx, 2'bxx, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'bx, 1'b0};
        Write_final: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'b0, 2'b01, 2'b00, 1'b1, 1'b0, 1'b0, 1'bx, 1'b0, 1'b0, 1'b0};
        default: {waiting_x, reg_sel_x, wb_sel_x, w_en_x, en_A_x, en_B_x, en_C_x, en_status_x, sel_A_x, sel_B_x} = {1'bx, 2'bxx, 2'bxx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx, 1'bx};

      endcase
    end

endmodule: controller


