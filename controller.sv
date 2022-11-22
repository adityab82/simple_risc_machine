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
