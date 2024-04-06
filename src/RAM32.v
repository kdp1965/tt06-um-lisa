
module RAM32
(
   input wire CLK,
   input wire [3:0] WE0,
   input wire EN0,
   input wire [4:0] A0,
   input wire [31:0] Di0,
   output wire [31:0] Do0
);

  D_RAM32 d_ram32_i
  (
      .clka (CLK),
      .wea  (WE0),
      .ena  (EN0),
      .addra (A0),
      .dina  (Di0),
      .douta (Do0)
  );

endmodule

