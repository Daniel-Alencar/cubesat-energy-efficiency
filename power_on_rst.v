module power_on_rst(
    input clk_in,
    //output rst_out,
    output n_rst_out
); 
// Declaração do sinal de reset (ativo durante os primeiros 8 ciclos de clock)   
reg [7:0] rst_reg = 8'hFF;
// Deslocamento de bit
always @(posedge clk_in)
rst_reg <= rst_reg >> 1;
// Direcionamento dos registradores para as saídas
// assign rst_out = rst_reg[0];
assign n_rst_out = ~rst_reg[0];
endmodule