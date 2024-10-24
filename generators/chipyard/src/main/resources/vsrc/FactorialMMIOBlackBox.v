module FactorialMMIOBlackBox
  #(parameter WIDTH = 32)
   (
    input                  clock,
    input                  reset,
    output                 input_ready,
    input                  input_valid,
    input  [WIDTH-1:0]     x,           // Input number for which factorial is to be calculated
    input                  output_ready,
    output                 output_valid,
    output reg [WIDTH-1:0] factorial,   // Output for the factorial result
    output                 busy
    );

   // State encoding
   localparam S_IDLE = 2'b00, S_RUN = 2'b01, S_DONE = 2'b10;

   // Registers
   reg [1:0] state;
   reg [WIDTH-1:0] counter;            // Counter for factorial multiplication
   reg [WIDTH-1:0] result;             // Register to hold factorial result

   // Outputs
   assign input_ready = (state == S_IDLE);
   assign output_valid = (state == S_DONE);
   assign busy = (state != S_IDLE);

   // Main state machine for calculating factorial
   always @(posedge clock or posedge reset) begin
      if (reset) begin
         state <= S_IDLE;
         result <= 0;
         factorial <= 0;
         counter <= 0;
      end
      else begin
         case (state)
            S_IDLE: begin
               if (input_valid) begin
                  state <= S_RUN;
                  result <= 1;            // Start with 1 for factorial calculation
                  counter <= x;           // Initialize counter with input value
               end
            end

            S_RUN: begin
               if (counter > 1) begin
                  result <= result * counter; // Multiply to calculate factorial
                  counter <= counter - 1;     // Decrement the counter
               end
               else begin
                  state <= S_DONE;         // Once the factorial is done, move to DONE state
               end
            end

            S_DONE: begin
               if (output_ready) begin
                  factorial <= result;      // Output the factorial result
                  state <= S_IDLE;          // Go back to idle once output is read
               end
            end
         endcase
      end
   end
endmodule
