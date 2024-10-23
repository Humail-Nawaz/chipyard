xmodule FactorialMMIOBlackBox
  #(parameter WIDTH = 32)  // Set default bit width
   (
    input                  clock,
    input                  reset,
    output                 input_ready,
    input                  input_valid,
    input [WIDTH-1:0]      n,           // Input value for which factorial is to be computed
    input                  output_ready,
    output reg             output_valid,
    output reg [WIDTH-1:0] result,   // Computed factorial value
    output reg             busy
    );

  // Internal registers
  reg [WIDTH-1:0] temp_n;
  reg [WIDTH-1:0] factorial;
  reg [1:0] state;

  // Define states
  localparam IDLE = 2'b00;
  localparam CALCULATE = 2'b01;
  localparam DONE = 2'b10;

  // Assign output signals
  assign input_ready = (state == IDLE);
  
  always @(posedge clock or posedge reset) begin
    if (reset) begin
      state <= IDLE;
      busy <= 0;
      result <= 0;
      factorial <= 1;
      temp_n <= 0;
      output_valid <= 0;
    end else begin
      case (state)
        IDLE: begin
          output_valid <= 0;
          if (input_valid && !busy) begin
            temp_n <= n;  // Load input n
            factorial <= 1;
            busy <= 1;
            state <= CALCULATE;
          end
        end
        CALCULATE: begin
          if (temp_n > 1) begin
            factorial <= factorial * temp_n;
            temp_n <= temp_n - 1;
          end else begin
            result <= factorial;
            state <= DONE;
          end
        end
        DONE: begin
          busy <= 0;
          output_valid <= 1;
          if (output_ready) begin
            state <= IDLE;  // Go back to IDLE once result is read
          end
        end
      endcase
    end
  end
endmodule

