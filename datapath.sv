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