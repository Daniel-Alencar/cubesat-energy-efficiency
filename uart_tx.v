module uart_tx #(parameter DATA_BITS=8, STOP_BITS=1, OVERSAMPLING=16)(
    input clk_in, n_rst, uart_en,
    input [DATA_BITS-1:0] data_in,
    output ready_out, valid_out, tx
);
// Declaração dos estados simbólicos
localparam reg [1:0]
idle = 2'b00,
start = 2'b01,
data = 2'b10,
stop = 2'b11;
// Declaração dos sinais
reg tx_reg = 1'b1;
reg next_tx;
reg ready_reg, next_ready;
reg valid_reg, next_valid;
reg [DATA_BITS-1:0] data_reg, next_data;
reg [1:0] state, next_state;
reg [$clog2((OVERSAMPLING*2)-1)-1:0] clk_cnt, next_clk;
reg [2:0] bit_cnt, next_bit;
// Registradores da máquina de estados para o transmissor UART
always @(posedge clk_in, negedge n_rst) begin
    if (~n_rst) begin
        state <= idle;
        tx_reg <= 1'b1;
        ready_reg <= 1'b0;
        valid_reg <= next_valid;
        data_reg <= 0;
        clk_cnt <= 0;
        bit_cnt <= 0;
    end
    else begin
        state <= next_state;
        tx_reg <= next_tx;
        ready_reg <= next_ready;
        valid_reg <= next_valid;
        clk_cnt <= next_clk;
        bit_cnt <= next_bit;
        data_reg <= next_data;
    end
end
// Lógica combinacional da máquina de estados para o transmissor UART
always @(*) begin
    next_state = state;
    next_tx = tx_reg;
    next_ready = ready_reg;
    next_valid = valid_reg;
    next_clk = clk_cnt;
    next_bit = bit_cnt;
    next_data = data_reg;
    case (state)
    idle: begin
        next_tx = 1'b1;
        next_ready = 1'b1;
        if (uart_en) begin
            next_data = data_in;
            next_clk = 0;            
            next_state = start;
        end
    end
    start: begin
        next_ready = 1'b0;
        next_tx = 1'b0;
        if (clk_cnt == OVERSAMPLING-1) begin
            next_clk = 0;
            next_bit = 0;
            next_state = data;
        end
        else
        next_clk = clk_cnt + 1;
    end
    data: begin
        next_tx = data_reg[0];
        if (clk_cnt == OVERSAMPLING-1) begin
            next_clk = 0;
            next_data = data_reg >> 1;
            if (bit_cnt == DATA_BITS-1) begin
                next_valid = 1'b1;
                next_state = stop;
            end
            else
            next_bit = bit_cnt + 1;
        end
        else
        next_clk = clk_cnt + 1;
    end
    stop: begin
        next_valid = 1'b0;
        next_tx = 1'b1;
        if (clk_cnt == (OVERSAMPLING*STOP_BITS)-1)
        next_state = idle;
        else
        next_clk = clk_cnt + 1;
    end
    endcase
end
// Direcionamento dos registradores para as saídas
assign tx = tx_reg;
assign ready_out = ready_reg;
assign valid_out = valid_reg;
endmodule