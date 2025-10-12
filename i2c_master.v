module i2c_master #(parameter CLK_IN_FREQ_MHZ=10, SCL_FREQ_KHZ=100, RETRY_NUM=3)(
    input clk_in, n_rst, rd_wr_in, continuous_in, enable_in,
    input [6:0] address_in,
    // limite máximo de 32 bytes sequenciais
    // se refere a quantidade de bytes a serem lidos ou escritos
    input [5:0] data_bytes_in,
    input [7:0] wr_data_in,
    output ready_out, wr_valid_out, rd_valid_out,
    output [7:0] rd_data_out,
    output [3:0] value_state,
    inout SCL, SDA
);
/*
Cálculo dos parâmetros OVERSAMPLING e T_HD_CNT, 
considerando CLK_IN_FREQ entre 10 MHz e 100 MHz,
múltiplo de 10 MHz, e SCL_FREQ entre 1 kHz e 400 kHz
*/
// Número de ciclos do clk_in para gerar um ciclo SCL
localparam OVERSAMPLING = (CLK_IN_FREQ_MHZ*10**6 / (SCL_FREQ_KHZ*10**3));
// t_HD_DAT = 200 ns
// Tempo mínimo de hold para SDA antes da borda de SCL (especificado no padrão I2C)
localparam T_HD_DAT = 2 * (CLK_IN_FREQ_MHZ / 10);
// Declaração dos estados simbólicos
localparam reg [3:0]

// espera por enable_in para iniciar transação
IDLE = 4'b0000,
// gera a condição de start (SDA ↓ enquanto SCL = 1)
START = 4'b0001,
// envia endereço do escravo + bit R/W
ADDRESS = 4'b0010,
// espera pelo ACK/NACK do escravo
ADDRESS_ACK = 4'b0011,
// envia dados para o escravo (8 bits)
WRITE_DATA = 4'b0100,
// espera pelo ACK/NACK do escravo após cada byte
WRITE_DATA_ACK = 4'b0101,
// recebe dados do escravo (8 bits)
READ_DATA = 4'b0110,
// envia ACK/NACK para o escravo após cada byte
READ_DATA_ACK = 4'b0111,
// gera start repetido
REP_START = 4'b1000,
// tenta restransmissão se houve erro (NACK)
RETRY = 4'b1001,
// gera a condição de stop (SDA ↑ enquanto SCL = 1)
STOP = 4'b1010;

// Declaração dos sinais
wire SCL_line;
reg [3:0] state, next_state;
reg [$clog2(RETRY_NUM)-1:0] retry_cnt, next_retry;
reg SCL_reg, next_SCL;
reg SDA_reg, next_SDA;
reg [$clog2(OVERSAMPLING)-1:0] clk_cnt, next_clk;
reg [2:0] bit_cnt, next_bit;
reg [5:0] byte_cnt, next_byte;
reg [6:0] address_reg, next_address;
reg rd_wr_reg, next_rd_wr;
reg [7:0] wr_data_reg, next_wr_data;
reg [7:0] rd_data_reg, next_rd_data;
reg ready_reg, next_ready;
reg wr_valid_reg, next_wr_valid;
reg rd_valid_reg, next_rd_valid;
reg [7:0] word_reg, next_word;
reg ack_reg, next_ack;

/*
Verificação da disponibilidade da linha SCL: período mínimo
para "bus free condition": ~1/2 ciclo de SCL em nível alto
*/
// Verifica se SCL está livre (em HIGH por meio ciclo)
reg [$clog2(OVERSAMPLING)-1:0] SCL_busy_cnt = 0;
reg SCL_busy_reg = 1'b1;
always @(posedge clk_in)
// Se SCL_line estiver em nível baixo, zera o contador e indica que SCL está ocupado
if (~SCL_line) begin
    SCL_busy_cnt <= 0;
    SCL_busy_reg <= 1'b1;
end
// Se SCL_line estiver em nível alto por meio ciclo, indica que SCL está livre
// Dúvida: por que OVERSAMPLING-1? Não seria OVERSAMPLING/2-1?
else if (SCL_busy_cnt == OVERSAMPLING-1) begin
    SCL_busy_cnt <= 0;
    SCL_busy_reg <= 1'b0;
end
// Se SCL_line estiver em nível alto, incrementa o contador
else
SCL_busy_cnt <= SCL_busy_cnt + 1;

// Esse bloco implementa a detecção de sincronismo entre o clock que o master quer gerar e o clock que realmente está no barramento, permitindo lidar com clock stretching ou distorções no sinal e garantindo que o contador só avance quando o sinal físico de SCL realmente mudou para o valor esperado.
/*
Verificação do sincronismo entre SCL_reg e SCL_line, feita no flanco
negativo do clk_in para pausar a contagem de clock no próximo flanco de subida
*/
reg sync_reg = 1'b0;
always @(negedge clk_in)
if (SCL_reg == SCL_line)
sync_reg = 1'b1;
else
sync_reg = 1'b0;


// Registradores para a máquina de estados do comunicador I2C
always @(posedge clk_in, negedge n_rst)
if (~n_rst) begin
    state <= IDLE;
    retry_cnt <= 0;
    SCL_reg <= 1'b1;
    SDA_reg <= 1'b1;
    clk_cnt <= 0;
    bit_cnt <= 0;
    byte_cnt <= 0;
    address_reg <= 0;
    rd_wr_reg <= 1'b0;
    wr_data_reg <= 0;
    rd_data_reg <= 0;
    ready_reg <= 1'b0;
    wr_valid_reg <= 1'b0;
    rd_valid_reg <= 1'b0;
    word_reg <= 0;
    ack_reg <= 1'b1;
end
else begin
    state <= next_state;
    retry_cnt <= next_retry;
    SCL_reg <= next_SCL;
    SDA_reg <= next_SDA;
    clk_cnt <= next_clk;
    bit_cnt <= next_bit;
    byte_cnt <= next_byte;
    address_reg <= next_address;
    rd_wr_reg <= next_rd_wr;
    wr_data_reg <= next_wr_data;
    rd_data_reg <= next_rd_data;
    ready_reg <= next_ready;
    wr_valid_reg <= next_wr_valid;
    rd_valid_reg <= next_rd_valid;
    word_reg <= next_word;
    ack_reg <= next_ack;
end

// Lógica combinacional para as transições entre estados
always @(*) begin
    next_state = state;
    next_retry = retry_cnt;
    next_clk = clk_cnt;
    next_bit = bit_cnt;
    next_byte = byte_cnt;
    case (state)
    IDLE:
    // Fica parado esperando o sinal enable_in 
    // e um número válido de bytes (data_bytes_in > 0)
    // Se recebeu um comando, zera contadores e vai para START
    if (ready_reg && enable_in && data_bytes_in > 0) begin
        next_clk = 0;
        next_retry = 0;
        next_byte = data_bytes_in;
        next_state = START;
    end
    // Espera um período inteiro de OVERSAMPLING antes de ir para ADDRESS
    START:
    if (clk_cnt == OVERSAMPLING-1) begin
        next_clk = 0;
        next_bit = 0;
        next_state = ADDRESS;
    end
    else
    next_clk = clk_cnt + 1;
    // Faz a contagem dos bits enviados antes de antes de mudar para ADDRESS_ACK
    ADDRESS:
    // Se detectar que a linha SDA não corresponde ao que esperava, vai para RETRY
    // Dúvida: por que 78% de OVERSAMPLING?
    if (clk_cnt == (78*OVERSAMPLING/100)-1 && SDA_reg != SDA_line)    
        next_state = RETRY;
    // Quando termina de enviar os bits, muda para ADDRESS_ACK
    else if (clk_cnt == OVERSAMPLING-1) begin
        next_clk = 0;
        if (bit_cnt == 7)
            next_state = ADDRESS_ACK;
        else
            next_bit = bit_cnt + 1;
    end
    else if (sync_reg)
        next_clk = clk_cnt + 1;

    // Lê o ACK/NACK do escravo
    // Se recebeu ACK (ack_reg == 0), decide se vai escrever (WRITE_DATA) ou ler (READ_DATA)
    // Se recebeu NACK, tenta novamente (RETRY)
    ADDRESS_ACK:
    if (clk_cnt == OVERSAMPLING-1) begin
        if (~ack_reg) begin
            next_clk = 0;
            next_bit = 0;
            if (~rd_wr_in)
            next_state = WRITE_DATA;
            else
            next_state = READ_DATA;
        end
        else
        next_state = RETRY;
    end
    else if (sync_reg)
    next_clk = clk_cnt + 1;

    // Vai deslocando os bits (bit_cnt vai de 0 a 7)
    // Quando terminar o byte, decrementa byte_cnt e vai para WRITE_DATA_ACK
    WRITE_DATA:
    // Dúvida: por que 78% de OVERSAMPLING?
    // Se detectar que a linha SDA não corresponde ao que esperava, vai para RETRY
    if (clk_cnt == (78*OVERSAMPLING/100)-1 && SDA_reg != SDA_line)
    next_state = RETRY;
    else if (clk_cnt == OVERSAMPLING-1) begin
        next_clk = 0;
        if (bit_cnt == 7) begin
            next_byte = byte_cnt - 1;           
            next_state = WRITE_DATA_ACK;
        end
        else
        next_bit = bit_cnt + 1;
    end
    else if (sync_reg)
    next_clk = clk_cnt + 1;

    // Se ACK recebido e ainda há bytes: envia o próximo byte (WRITE_DATA) ou faz start repetido (REP_START)
    // Se ACK recebido e não há mais bytes: faz STOP
    // Se NACK recebido: tenta novamente (RETRY)
    WRITE_DATA_ACK:
    if (clk_cnt == OVERSAMPLING-1) begin
        next_clk = 0;
        if (~ack_reg)
            if (byte_cnt > 0) begin
                next_bit = 0;
                case (continuous_in)
                1'b0: next_state = REP_START;
                1'b1: next_state = WRITE_DATA;
                endcase
            end
            else
            next_state = STOP;
        else
        next_state = RETRY;
    end
    else if (sync_reg)
    next_clk = clk_cnt + 1;

    // Vai fazendo a contagem de bits lidos até formar um byte
    // Quando chega no 8º bit vai para o READ_DATA_ACK
    READ_DATA:
    if (clk_cnt == OVERSAMPLING-1) begin
        next_clk = 0;
        if (bit_cnt == 7) begin
            next_byte = byte_cnt - 1;            
            next_state = READ_DATA_ACK;
        end
        else
        next_bit = bit_cnt + 1;
    end
    else if (sync_reg)
    next_clk = clk_cnt + 1;

    // Se ainda há bytes, continua fazendo a leitura
    // Se não há mais bytes, vai para o STOP
    READ_DATA_ACK:
    if (clk_cnt == OVERSAMPLING-1) begin
        next_clk = 0;
        if (byte_cnt > 0) begin
            next_bit = 0;
            case (continuous_in)
            1'b0: next_state = REP_START;
            1'b1: next_state = READ_DATA;
            endcase
        end
        else
        next_state = STOP;
    end
    else if (sync_reg)
    next_clk = clk_cnt + 1;

    // Gera uma nova condição de START sem dar STOP
    // Usado quando quer mudar direção (de write para read, por exemplo)
    REP_START:
    if (clk_cnt == (55*OVERSAMPLING/100)-1) begin
        next_clk = 0;
        next_state = START;
    end
    else
    next_clk = clk_cnt + 1;

    // Se excedeu o número máximo de tentativas, volta para IDLE
    // Se não está livre, tenta novamente (START)
    RETRY:
    if (retry_cnt == RETRY_NUM)
    next_state = IDLE;
    else if (~SCL_busy_reg) begin
        next_clk = 0;
        next_bit = 0;
        next_retry = retry_cnt + 1;
        next_state = START;
    end

    STOP:
    if (clk_cnt == OVERSAMPLING-1)
    next_state = IDLE;
    else if (sync_reg)
    next_clk = clk_cnt + 1;
    endcase
end

// Lógica combinacional para o clock serial SCL
always @(*) begin
    next_SCL = SCL_reg;
    case (state)
        IDLE:
            next_SCL = 1'b1;
        START:
            if (clk_cnt == OVERSAMPLING-1)
                next_SCL = 1'b0;
        REP_START:
            if (clk_cnt == (55*OVERSAMPLING/100)-1)
                next_SCL = 1'b1;
        RETRY:
            next_SCL = 1'b1;
        STOP:
            if (clk_cnt == (55*OVERSAMPLING/100)-1)
                next_SCL = 1'b1;
        default:
            if (clk_cnt == (55*OVERSAMPLING/100)-1)
                next_SCL = 1'b1;
            else if (clk_cnt == OVERSAMPLING-1)
                next_SCL = 1'b0;
    endcase
end

// Lógica combinacional para a linha serial de dados SDA e para os demais registradores
always @(*) begin
    next_SDA = SDA_reg;
    next_address = address_reg;
    next_rd_wr = rd_wr_reg;
    next_wr_data = wr_data_reg;
    next_rd_data = rd_data_reg;
    next_ready = ready_reg;
    next_wr_valid = wr_valid_reg;
    next_rd_valid = rd_valid_reg;
    next_word = word_reg;
    next_ack = ack_reg;
    case (state)
    IDLE:
    if (ready_reg && enable_in && data_bytes_in > 0) begin
        next_address = address_in;
        next_rd_wr = rd_wr_in;

        // se operação = escrita
        if (~rd_wr_in)     
        next_wr_data = wr_data_in;
        next_ready = 1'b0;
    end
    else begin
        if (~SCL_busy_reg)
        next_ready = 1'b1;
        next_wr_valid = 1'b0;
        next_rd_valid = 1'b0;
        next_SDA = 1'b1;
    end
    START: begin
        if (clk_cnt == (55*OVERSAMPLING/100)-1)
        next_SDA = 1'b0;
        next_word = {address_reg, rd_wr_reg};
    end
    ADDRESS:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)
    next_SDA = word_reg[7];
    else if (clk_cnt == (78*OVERSAMPLING/100)-1)
    next_word = word_reg << 1;
    ADDRESS_ACK:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)
    // permite verificar o ADDRESS_ACK
    next_SDA = 1'b1;            
    else if (clk_cnt == (78*OVERSAMPLING/100)-1)
    next_ack = SDA_line;
    else if (clk_cnt == OVERSAMPLING-1 && ~ack_reg && ~rd_wr_in)
    next_word = {wr_data_reg};
    WRITE_DATA:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)
    next_SDA = word_reg[7];
    else if (clk_cnt == (78*OVERSAMPLING/100)-1)
    next_word = word_reg << 1;
    WRITE_DATA_ACK:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)
    // permite verificar o WRITE_DATA_ACK
    next_SDA = 1'b1;           
    else if (clk_cnt == (78*OVERSAMPLING/100)-1) begin
        next_ack = SDA_line;
        next_wr_valid = ~SDA_line;
    end
    else if (clk_cnt == (78*OVERSAMPLING/100))
    next_wr_valid = 1'b0;
    else if (clk_cnt == OVERSAMPLING-1 && ~ack_reg && byte_cnt > 0) begin
        next_wr_data = wr_data_in;
        if (continuous_in)
        next_word = wr_data_in;
    end
    READ_DATA:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)    
    // permite leitura da linha SDA em modo contínuo
    next_SDA = 1'b1;            
    else if (clk_cnt == (78*OVERSAMPLING/100)-1)
    next_word[7] = SDA_line;
    else if (clk_cnt == OVERSAMPLING-1)
    next_word = (word_reg << 1) | (word_reg >> (7));
    READ_DATA_ACK:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)    
        if (byte_cnt == 0)
        // NACK
        next_SDA = 1'b1;        
        else
        // ACK
        next_SDA = 1'b0;        
    else if (clk_cnt == (78*OVERSAMPLING/100)-1) begin
        next_rd_data = word_reg;
        next_rd_valid = 1'b1;
    end
    else if (clk_cnt == 78*OVERSAMPLING/100)
    next_rd_valid = 1'b0;
    REP_START: begin
        next_address = address_in;
        next_rd_wr = rd_wr_in;
        // se operação = escrita
        if (~rd_wr_in)     
        next_wr_data = wr_data_in;
        // t_hd_dat = 200 ns
        if (clk_cnt == T_HD_DAT-1)    
        next_SDA = 1'b1;
    end
    RETRY:
    next_SDA = 1'b1;
    STOP:
    // t_hd_dat = 200 ns
    if (clk_cnt == T_HD_DAT-1)
    next_SDA = 1'b0;
    else if (clk_cnt == OVERSAMPLING-1)
    next_SDA = 1'b1;
    endcase
end
// Direcionamento dos registradores para as saídas
assign ready_out = ready_reg;
assign wr_valid_out = wr_valid_reg;
assign rd_valid_out = rd_valid_reg;
assign SCL = SCL_reg ? 1'bZ : 1'b0;
assign SCL_line = SCL;
assign SDA = SDA_reg ? 1'bZ : 1'b0;
assign SDA_line = SDA;
assign rd_data_out = rd_data_reg;
assign value_state = state;
endmodule