// DOC include start: GCD portlist
module AdderMMIOBlackBox
  #(parameter WIDTH)
   (
    input                  clock,
    input                  reset,
    output                 input_ready,
    input                  input_valid,
    input [WIDTH-1:0]      x,
    input [WIDTH-1:0]      y,
    input                  output_ready,
    output                 output_valid,
    output reg [WIDTH-1:0] gcd,
    output                 busy
    );
// DOC include end: Adder portlist


endmodule // AdderMMIOBlackBox
