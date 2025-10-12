module MPU6050_ctrl (
    // clock principal do sistema
    // reset ativo baixo
    input clk_in, n_rst,
    // sinal do módulo I2C indicando que está pronto para nova operação
    input i2c_ready_in, 
    // indica que a escrita I2C foi concluída com sucesso
    i2c_wr_valid_in, 
    // indica que um byte de leitura I2C está disponível
    i2c_rd_valid_in,
    // dados lidos do dispositivo I2C
    input [7:0] i2c_rd_data_in,
    // sinais do módulo UART
    input uart_rx_valid_in,
    input [7:0] uart_rx_data_in,
    // uart está pronta para receber um novo byte
    input uart_tx_ready_in, 
    // uart transmitiu com sucesso um byte
    uart_tx_valid_in,
    // define operação de leitura (1) ou escrita (0)
    output i2c_rd_wr, 
    // define se a operação I2C é contínua (1) ou única (0)
    i2c_continuous, 
    // habilita uma nova operação I2C
    i2c_en,
    // endereço do registrador no MPU6050
    output [6:0] i2c_address,
    // número de bytes a serem lidos ou escritos
    output [5:0] i2c_data_bytes,
    // dados a serem escritos no dispositivo I2C
    output [7:0] i2c_wr_data,
    // habilita envio de byte
    output uart_tx_en,
    // dado a ser transmitido
    output [7:0] uart_tx_data
);

// ==================== REGISTRADORES DO MPU6050 ====================
// AD0 = GND

// endereço I²C do dispositivo
localparam reg [6:0] ADDRESS_MPU6050   = 7'b1101000;
// endereço do registrador para acordar o chip
localparam reg [7:0] PWR_MGMT_1        = 8'h6B;
// configuração de filtros e escalas
localparam reg [7:0] SMPLRT_DIV        = 8'h19;
localparam reg [7:0] CONFIG            = 8'h1A;
localparam reg [7:0] GYRO_CONFIG       = 8'h1B;
localparam reg [7:0] ACCEL_CONFIG      = 8'h1C;
// primeiro byte dos dados de aceleração
localparam reg [7:0] ACCEL_XOUT_H      = 8'h3B;

// ==================== VALORES DE CONFIGURAÇÃO =====================
// desativa o sleep mode
localparam reg [7:0] PWR_MGMT_1_DATA   = 8'h00;
// taxa de amostragem: 1kHz / (1 + 7) = 125Hz
localparam reg [7:0] SMPLRT_DIV_DATA   = 8'h07;
// filtro passa-baixa digital em 44Hz
localparam reg [7:0] CONFIG_DATA       = 8'h03;
// ±250°/s
localparam reg [7:0] GYRO_CONFIG_DATA  = 8'h00;
// ±2g
localparam reg [7:0] ACCEL_CONFIG_DATA = 8'h00;

// ==================== DEFINIÇÃO DE ESTADOS ========================
localparam reg [4:0]
// estado inicial, espera início da configuração
IDLE             = 5'd0,
// escreve no PWR_MGMT_1 para tirar o chip do modo sleep
WAKEUP           = 5'd1,
WAKEUP_PARAM     = 5'd2,
// escreve taxa de amostragem
CONF_SMPLRT      = 5'd3,
CONF_SMPLRT_PARAM= 5'd4,
// escreve configuração do DLPF (filtro digital)
CONF_CONFIG      = 5'd5,
CONF_CONFIG_PARAM= 5'd6,
// escreve configuração do giroscópio
CONF_GYRO        = 5'd7,
CONF_GYRO_PARAM  = 5'd8,
// escreve configuração do acelerômetro
CONF_ACCEL       = 5'd9,
CONF_ACCEL_PARAM = 5'd10,
// envia endereço inicial para leitura dos dados
READ_ADDR        = 5'd11,
// recebe bytes do i2c
READ_DATA        = 5'd12,
// guarda byte no buffer
STORE_BYTE       = 5'd13,
// envia os bytes pela UART
UART_SEND        = 5'd14,
// espera um tempo antes de próxima leitura
TIMER            = 5'd15;

// ==================== PARÂMETROS ====================
// contador para gerar um atraso de 1s entre leituras
// (considerando clock de 96MHz)
localparam TIMER_PARAM = 96*10**6; // 1 segundo
// quantidade total de bytes a serem lidos do MPU6050
localparam TOTAL_BYTES = 6;        // X, Y, Z (2 bytes cada)

// ==================== REGISTRADORES INTERNOS ====================
reg [4:0] state, next_state;
reg [26:0] clk_cnt, next_clk;
reg [6:0] i2c_address_reg, next_i2c_address;
reg [7:0] i2c_wr_data_reg, next_i2c_wr_data;
reg [5:0] i2c_data_bytes_reg, next_i2c_data_bytes;
reg i2c_rd_wr_reg, next_i2c_rd_wr;
reg i2c_continuous_reg, next_i2c_continuous;
reg i2c_en_reg, next_i2c_en;
reg [7:0] data_buffer [0:TOTAL_BYTES-1];
reg [2:0] byte_cnt, next_byte_cnt;
reg uart_tx_en_reg, next_uart_tx_en;
reg [7:0] uart_tx_data_reg, next_uart_tx_data;
reg writting_reg, next_writting;

// ==================== FLIP-FLOPS ====================
always @(posedge clk_in, negedge n_rst)
if (~n_rst) begin
    state <= IDLE;
    clk_cnt <= 0;
    i2c_address_reg <= 0;
    i2c_wr_data_reg <= 0;
    i2c_data_bytes_reg <= 0;
    i2c_rd_wr_reg <= 1'b0;
    i2c_continuous_reg <= 1'b0;
    i2c_en_reg <= 1'b0;
    uart_tx_data_reg <= 0;
    uart_tx_en_reg <= 1'b0;
    writting_reg <= 1'b0;
    byte_cnt <= 0;
end
else begin
    state <= next_state;
    clk_cnt <= next_clk;
    i2c_address_reg <= next_i2c_address;
    i2c_wr_data_reg <= next_i2c_wr_data;
    i2c_data_bytes_reg <= next_i2c_data_bytes;
    i2c_rd_wr_reg <= next_i2c_rd_wr;
    i2c_continuous_reg <= next_i2c_continuous;
    i2c_en_reg <= next_i2c_en;
    uart_tx_data_reg <= next_uart_tx_data;
    uart_tx_en_reg <= next_uart_tx_en;
    writting_reg <= next_writting;
    byte_cnt <= next_byte_cnt;
end

// ==================== MÁQUINA DE ESTADOS ====================
always @(*) begin
    next_state = state;
    next_clk = clk_cnt;
    next_i2c_address = i2c_address_reg;
    next_i2c_wr_data = i2c_wr_data_reg;
    next_i2c_data_bytes = i2c_data_bytes_reg;
    next_i2c_rd_wr = i2c_rd_wr_reg;
    next_i2c_continuous = i2c_continuous_reg;
    next_i2c_en = i2c_en_reg;
    next_uart_tx_data = uart_tx_data_reg;
    next_uart_tx_en = uart_tx_en_reg;
    next_writting = writting_reg;
    next_byte_cnt = byte_cnt;

    case (state)
    IDLE: begin
        next_uart_tx_en = 1'b0;
        if (i2c_ready_in) begin
            next_i2c_address = ADDRESS_MPU6050;
            next_state = WAKEUP;
        end
    end

    // ======= WAKEUP =======
    WAKEUP: if (i2c_ready_in) begin
        next_i2c_wr_data = PWR_MGMT_1;
        next_i2c_rd_wr = 1'b0;
        next_i2c_data_bytes = 2;
        next_i2c_continuous = 1'b1;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = WAKEUP_PARAM;
    end

    WAKEUP_PARAM:
    if (i2c_wr_valid_in) begin
        next_i2c_wr_data = PWR_MGMT_1_DATA;
        next_state = CONF_SMPLRT;
    end

    // ======= CONFIGURAÇÃO =======
    CONF_SMPLRT:
    if (i2c_ready_in) begin
        next_i2c_wr_data = SMPLRT_DIV;
        next_i2c_data_bytes = 2;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = CONF_SMPLRT_PARAM;
    end

    CONF_SMPLRT_PARAM:
    if (i2c_wr_valid_in) begin
        next_i2c_wr_data = SMPLRT_DIV_DATA;
        next_state = CONF_CONFIG;
    end

    CONF_CONFIG:
    if (i2c_ready_in) begin
        next_i2c_wr_data = CONFIG;
        next_i2c_data_bytes = 2;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = CONF_CONFIG_PARAM;
    end

    CONF_CONFIG_PARAM:
    if (i2c_wr_valid_in) begin
        next_i2c_wr_data = CONFIG_DATA;
        next_state = CONF_GYRO;
    end

    CONF_GYRO:
    if (i2c_ready_in) begin
        next_i2c_wr_data = GYRO_CONFIG;
        next_i2c_data_bytes = 2;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = CONF_GYRO_PARAM;
    end

    CONF_GYRO_PARAM:
    if (i2c_wr_valid_in) begin
        next_i2c_wr_data = GYRO_CONFIG_DATA;
        next_state = CONF_ACCEL;
    end

    CONF_ACCEL:
    if (i2c_ready_in) begin
        next_i2c_wr_data = ACCEL_CONFIG;
        next_i2c_data_bytes = 2;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = CONF_ACCEL_PARAM;
    end

    CONF_ACCEL_PARAM:
    if (i2c_wr_valid_in) begin
        next_i2c_wr_data = ACCEL_CONFIG_DATA;
        next_state = READ_ADDR;
    end

    // ======= LEITURA DE DADOS =======
    READ_ADDR:
    if (i2c_ready_in) begin
        next_i2c_wr_data = ACCEL_XOUT_H;
        next_i2c_data_bytes = 1;
        next_i2c_rd_wr = 1'b0;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = READ_DATA;
    end

    READ_DATA:
    if (i2c_ready_in) begin
        next_i2c_rd_wr = 1'b1;
        next_i2c_data_bytes = TOTAL_BYTES;
        next_i2c_continuous = 1'b1;
        next_i2c_en = 1'b1;
        next_writting = 1'b1;
        next_byte_cnt = 0;
    end
    else if (writting_reg && ~i2c_ready_in) begin
        next_i2c_en = 1'b0;
        next_writting = 1'b0;
        next_state = STORE_BYTE;
    end

    STORE_BYTE:
    if (i2c_rd_valid_in) begin
        data_buffer[byte_cnt] = i2c_rd_data_in;
        if (byte_cnt == TOTAL_BYTES-1)
            next_state = UART_SEND;
        else
            next_byte_cnt = byte_cnt + 1;
    end

    UART_SEND:
    if (uart_tx_ready_in) begin
        next_uart_tx_data = data_buffer[byte_cnt];
        next_uart_tx_en = 1'b1;
        next_writting = 1'b1;
    end
    else if (writting_reg && ~uart_tx_ready_in) begin
        next_uart_tx_en = 1'b0;
        next_writting = 1'b0;
        if (byte_cnt == TOTAL_BYTES-1)
            next_state = TIMER;
        else
            next_byte_cnt = byte_cnt + 1;
    end

    TIMER:
    if (clk_cnt == TIMER_PARAM-1) begin
        next_clk = 0;
        next_state = READ_ADDR;
    end
    else
        next_clk = clk_cnt + 1;

    endcase
end

// ==================== ASSOCIAÇÃO ÀS SAÍDAS ====================
assign i2c_rd_wr = i2c_rd_wr_reg;
assign i2c_continuous = i2c_continuous_reg;
assign i2c_en = i2c_en_reg;
assign i2c_address = i2c_address_reg;
assign i2c_data_bytes = i2c_data_bytes_reg;
assign i2c_wr_data = i2c_wr_data_reg;
assign uart_tx_en = uart_tx_en_reg;
assign uart_tx_data = uart_tx_data_reg;

endmodule
