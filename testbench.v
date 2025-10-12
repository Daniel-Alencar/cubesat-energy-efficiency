`timescale 1ns / 1ps

module testbench;

    // ==================== PARÂMETROS E SINAIS ====================

    // Parâmetros do Clock de entrada (Device Clock) -> 25 MHz
    localparam CLK_PERIOD = 40; // Período de 40 ns (1 / 25 MHz)

    // Parâmetros da UART (para o monitor)
    localparam UART_CYCLES_PER_BIT = 2604;

    // Sinais para conectar ao DUT (Device Under Test)
    reg   dev_clk;
    reg   rx;
    wire  tx;
    wire  SCL_wire;
    wire  SDA_wire;

    // ==================== PULL-UPS VIRTUAIS PARA O I2C ====================
    // Simula os resistores de pull-up necessários no barramento I2C.
    pullup(SCL_wire);
    pullup(SDA_wire);

    // ==================== INSTANCIAÇÃO DO DUT ====================

    design_1_wrapper dut (
        .dev_clk(dev_clk),
        .rx(rx),
        .tx(tx),
        .SCL(SCL_wire),
        .SDA(SDA_wire)
    );

    // ==================== GERADOR DE CLOCK ====================

    initial begin
        dev_clk = 0;
        forever #(CLK_PERIOD / 2) dev_clk = ~dev_clk;
    end

    // ==================== LÓGICA PRINCIPAL DA SIMULAÇÃO ====================

    initial begin
        $display("=========================================================");
        $display("INICIANDO SIMULAÇÃO DO MPU6050 EM %0t ns", $time);
        $display("=========================================================");

        rx <= 1'b1;
        repeat (20) @(posedge dev_clk);
        $display("[%0t ns] Reset interno do DUT concluído.", $time);

        #(1200 * 1000 * 1000);

        $display("=========================================================");
        $display("FIM DA SIMULAÇÃO EM %0t ns", $time);
        $display("=========================================================");
        $finish;
    end

    // ==================== MONITOR UART ====================
    task uart_monitor;
        reg [7:0] received_byte;
        integer i;
        forever begin
            @(negedge tx);
            $display("[%0t ns] UART Monitor: Start bit detectado!", $time);
            #(UART_CYCLES_PER_BIT * CLK_PERIOD / 2);

            for (i = 0; i < 8; i = i + 1) begin
                #(UART_CYCLES_PER_BIT * CLK_PERIOD);
                received_byte[i] = tx;
            end

            #(UART_CYCLES_PER_BIT * CLK_PERIOD);
            if (tx == 1'b1) begin
                 $display("[%0t ns] UART Monitor: Byte recebido = 0x%h ('%c')", $time, received_byte, received_byte);
            end else begin
                 $display("[%0t ns] UART Monitor: ERRO! Stop bit não encontrado.", $time);
            end
        end
    endtask

    initial begin
        uart_monitor;
    end

    // ==================== SIMULADOR I2C SLAVE (MPU6050) ====================

    localparam SLAVE_ADDR = 7'b1101000;
    reg [7:0] mpu_memory [255:0];
    reg [7:0] addr_pointer;
    reg sda_tb_out;
    reg sda_tb_en;

    assign SDA_wire = sda_tb_en ? sda_tb_out : 1'bz;

    initial begin
        for (integer i = 0; i < 256; i = i + 1) begin
            mpu_memory[i] = 8'h00;
        end
        mpu_memory[8'h3B] = 8'hDE;
        mpu_memory[8'h3C] = 8'hAD;
        mpu_memory[8'h3D] = 8'hBE;
        mpu_memory[8'h3E] = 8'hEF;
        mpu_memory[8'h3F] = 8'hC0;
        mpu_memory[8'h40] = 8'hDE;
    end

    task i2c_get_bit;
        output bit_val;
        begin
            @(posedge SCL_wire);
            bit_val = SDA_wire;
            @(negedge SCL_wire);
        end
    endtask

    task i2c_send_bit;
        input bit_val;
        begin
            @(posedge SCL_wire);
            sda_tb_out <= bit_val;
            sda_tb_en  <= 1'b1;
            @(negedge SCL_wire);
        end
    endtask

    task i2c_release_sda;
        begin
            sda_tb_en <= 1'b0;
        end
    endtask

    reg [7:0] received_byte;
    reg ack_bit;

    initial begin
        sda_tb_en <= 1'b0; // Começa com a linha SDA libertada
        forever begin
            // 1. Espera por uma condição de START
            @(negedge SDA_wire);
            if (SCL_wire === 1'b1) begin
                $display("[%0t ns] I2C Slave: START detectado!", $time);

                // 2. Recebe 8 bits (endereço + R/W)
                received_byte = 0;
                for (integer i = 0; i < 8; i = i + 1) begin
                    i2c_get_bit(received_byte[7-i]);
                end

                // 3. Verifica o endereço e envia ACK
                if (received_byte[7:1] == SLAVE_ADDR) begin
                    $display("[%0t ns] I2C Slave: Endereço 0x%h recebido. R/W=%b.", $time, received_byte[7:1], received_byte[0]);
                    i2c_send_bit(0); // Envia ACK (0)
                    i2c_release_sda();

                    // 4. Lida com a operação de Escrita ou Leitura
                    if (received_byte[0] == 1'b0) begin // Master WRITE
                        // Recebe o endereço do registrador
                        for (integer i = 0; i < 8; i = i + 1) begin
                           i2c_get_bit(addr_pointer[7-i]);
                        end
                        $display("[%0t ns] I2C Slave: Endereço de registrador 0x%h recebido.", $time, addr_pointer);
                        i2c_send_bit(0); // Envia ACK
                        i2c_release_sda();

                        // Recebe o byte de dados
                        for (integer i = 0; i < 8; i = i + 1) begin
                           i2c_get_bit(received_byte[7-i]);
                        end
                        mpu_memory[addr_pointer] <= received_byte;
                        $display("[%0t ns] I2C Slave: Dado 0x%h escrito no endereço 0x%h.", $time, received_byte, addr_pointer);
                        i2c_send_bit(0); // Envia ACK
                        i2c_release_sda();

                    end else begin // Master READ
                        // Envia múltiplos bytes
                        begin : master_read_loop
                            forever begin
                                // Envia 8 bits de dados
                                for (integer i = 0; i < 8; i = i + 1) begin
                                    i2c_send_bit(mpu_memory[addr_pointer][7-i]);
                                end
                                i2c_release_sda();
                                $display("[%0t ns] I2C Slave: Enviando dado 0x%h do endereço 0x%h.", $time, mpu_memory[addr_pointer], addr_pointer);
                                addr_pointer <= addr_pointer + 1;
                                
                                // Recebe o bit de ACK/NACK do master
                                i2c_get_bit(ack_bit);
                                if (ack_bit == 1'b1) begin // NACK
                                    $display("[%0t ns] I2C Slave: NACK recebido do master. Fim da leitura.", $time);
                                    disable master_read_loop;
                                end else begin
                                    $display("[%0t ns] I2C Slave: ACK recebido do master.", $time);
                                end
                            end
                        end
                    end
                end else begin
                    $display("[%0t ns] I2C Slave: Endereço não corresponde (0x%h).", $time, received_byte[7:1]);
                    i2c_send_bit(1); // Envia NACK (1)
                    i2c_release_sda();
                end
            end
        end
    end

endmodule

