`timescale 1ns / 1ps

module testbench;

    // ==================== PARÂMETROS E SINAIS ====================

    // Parâmetros do Clock de entrada (Device Clock) -> 25 MHz
    localparam CLK_PERIOD = 40; // Período de 40 ns (1 / 25 MHz)

    // Parâmetros da UART (para o monitor)
    // Baud Rate = 9600. Clock do sistema = 96MHz. Oversampling = 10000.
    // O testbench usa o clock de 25MHz, então precisamos calcular o período do bit
    // em ciclos do clock de 25MHz.
    // Período do Bit = 1 / 9600 = 104167 ns
    // Ciclos por Bit = 104167 ns / 40 ns = 2604.175. Usaremos 2604.
    localparam UART_CYCLES_PER_BIT = 2604;

    // Sinais para conectar ao DUT (Device Under Test)
    reg   dev_clk;
    reg   rx;
    wire  tx;
    wire  SCL_wire;
    wire  SDA_wire;

    // ==================== INSTANCIAÇÃO DO DUT ====================

    // O wrapper do seu design é o nosso Device Under Test
    design_1_wrapper dut (
        .dev_clk(dev_clk),
        .rx(rx),
        .tx(tx),
        .SCL(SCL_wire),
        .SDA(SDA_wire)
    );

    // ==================== GERADOR DE CLOCK ====================

    // Gera um clock contínuo de 25 MHz
    initial begin
        dev_clk = 0;
        forever #(CLK_PERIOD / 2) dev_clk = ~dev_clk;
    end

    // ==================== LÓGICA PRINCIPAL DA SIMULAÇÃO ====================

    initial begin
        #100000000; // 100 ms de delay inicial para estabilização

        $display("=========================================================");
        $display("INICIANDO SIMULAÇÃO DO MPU6050 EM %0t ns", $time);
        $display("=========================================================");

        // Inicia com a linha RX em idle (alta)
        rx <= 1'b1;

        // Aguarda alguns ciclos para o reset interno (power_on_rst) estabilizar
        repeat (20) @(posedge dev_clk);
        $display("[%0t ns] Reset interno do DUT concluído.", $time);

        // A simulação irá rodar por 1.2 segundos de tempo simulado para
        // observar a inicialização e pelo menos um ciclo de leitura de dados.
        #(1200 * 1000 * 1000);

        $display("=========================================================");
        $display("FIM DA SIMULAÇÃO EM %0t ns", $time);
        $display("=========================================================");
        $finish;
    end

    // ==================== MONITOR UART ====================
    // Tarefa que escuta a linha 'tx' e decodifica os bytes enviados

    task uart_monitor;
        reg [7:0] received_byte;
        integer i;
        forever begin
            // Espera pelo start bit (transição de 1 para 0)
            @(negedge tx);
            $display("[%0t ns] UART Monitor: Start bit detectado!", $time);

            // Aguarda meio período de bit para centralizar a amostragem
            #(UART_CYCLES_PER_BIT * CLK_PERIOD / 2);

            // Lê os 8 bits de dados
            for (i = 0; i < 8; i = i + 1) begin
                #(UART_CYCLES_PER_BIT * CLK_PERIOD);
                received_byte[i] = tx;
            end

            // Aguarda o stop bit
            #(UART_CYCLES_PER_BIT * CLK_PERIOD);
            if (tx == 1'b1) begin
                 $display("[%0t ns] UART Monitor: Byte recebido = 0x%h ('%c')", $time, received_byte, received_byte);
            end else begin
                 $display("[%0t ns] UART Monitor: ERRO! Stop bit não encontrado.", $time);
            end
        end
    endtask

    // // Inicia o monitor UART em um processo paralelo
    // initial begin
    //     uart_monitor;
    // end


    // // ==================== SIMULADOR I2C SLAVE (MPU6050) ====================
    // // Este bloco age como o MPU6050, respondendo às requisições do master.

    // // Endereço I2C do MPU6050
    // localparam SLAVE_ADDR = 7'b1101000;

    // // Memória interna para simular os registradores do MPU6050
    // reg [7:0] mpu_memory [255:0];
    // reg [7:0] addr_pointer;

    // // Sinais para controlar a linha SDA
    // reg sda_tb_out; // O que o nosso slave quer colocar na linha SDA
    // reg sda_tb_en;  // Habilita a saída do slave na linha SDA

    // // Modelo de barramento I2C tri-state
    // assign SDA_wire = sda_tb_en ? sda_tb_out : 1'bz;
    // // O slave não controla o SCL, apenas o escuta
    // assign SCL_wire = 1'bz;

    // // Inicialização dos registradores do MPU6050
    // initial begin
    //     // Inicializa a memória com um valor padrão
    //     for (integer i = 0; i < 256; i = i + 1) begin
    //         mpu_memory[i] = 8'h00;
    //     end
    //     // Coloca valores fictícios nos registradores de dados do acelerômetro
    //     mpu_memory[8'h3B] = 8'hDE; // ACCEL_XOUT_H
    //     mpu_memory[8'h3C] = 8'hAD; // ACCEL_XOUT_L
    //     mpu_memory[8'h3D] = 8'hBE; // ACCEL_YOUT_H
    //     mpu_memory[8'h3E] = 8'hEF; // ACCEL_YOUT_L
    //     mpu_memory[8'h3F] = 8'hC0; // ACCEL_ZOUT_H
    //     mpu_memory[8'h40] = 8'hDE; // ACCEL_ZOUT_L
    // end

    // // Tarefas auxiliares para o I2C Slave
    // task i2c_get_byte;
    //     output [7:0] byte;
    //     reg [7:0] data_byte;
    //     begin
    //         for (integer i = 0; i < 8; i = i + 1) begin
    //             @(posedge SCL_wire);
    //             @(negedge SCL_wire);
    //             data_byte[7-i] = SDA_wire;
    //         end
    //         byte = data_byte;
    //     end
    // endtask

    // task i2c_send_byte;
    //     input [7:0] byte;
    //     begin
    //         for (integer i = 0; i < 8; i = i + 1) begin
    //             @(posedge SCL_wire);
    //             sda_tb_out <= byte[7-i];
    //             sda_tb_en  <= 1'b1;
    //             @(negedge SCL_wire);
    //         end
    //         // Release the bus for ACK
    //         sda_tb_en <= 1'b0;
    //     end
    // endtask

    // task i2c_send_ack;
    //     begin
    //         @(posedge SCL_wire);
    //         sda_tb_out <= 1'b0; // ACK is low
    //         sda_tb_en  <= 1'b1;
    //         @(negedge SCL_wire);
    //         sda_tb_en  <= 1'b0; // Release bus
    //     end
    // endtask

    // task i2c_get_ack;
    //     output ack_val;
    //     begin
    //          @(posedge SCL_wire);
    //          ack_val = SDA_wire;
    //          @(negedge SCL_wire);
    //     end
    // endtask

    // // Variáveis para o processo principal do I2C Slave
    // reg [7:0] received_addr_byte;
    // reg [7:0] received_data_byte;
    // reg ack_from_master;

    // // Processo principal do I2C Slave
    // initial begin
    //     forever begin
    //         // 1. Wait for START condition
    //         wait (SCL_wire === 1'b1);
    //         @(negedge SDA_wire);
    //         $display("[%0t ns] I2C Slave: START detectado!", $time);

    //         // 2. Get Address byte
    //         i2c_get_byte(received_addr_byte);

    //         // 3. Check Address and send ACK/NACK
    //         if (received_addr_byte[7:1] == SLAVE_ADDR) begin
    //             $display("[%0t ns] I2C Slave: Endereço 0x%h recebido. R/W=%b.", $time, received_addr_byte[7:1], received_addr_byte[0]);
    //             i2c_send_ack;

    //             // 4. Handle Write or Read operation
    //             if (received_addr_byte[0] == 1'b0) begin // Master WRITE
    //                 master_write_loop: begin
    //                     // Get register address
    //                     i2c_get_byte(addr_pointer);
    //                     $display("[%0t ns] I2C Slave: Endereço de registrador 0x%h recebido.", $time, addr_pointer);
    //                     i2c_send_ack;

    //                     // Loop to get data bytes
    //                     while(1) begin
    //                         // A simple check for stop condition before trying to get a byte
    //                         if (SCL_wire === 1'b1 && SDA_wire === 1'b1) begin
    //                             disable master_write_loop;
    //                         end
                            
    //                         i2c_get_byte(received_data_byte);
    //                         mpu_memory[addr_pointer] <= received_data_byte;
    //                         $display("[%0t ns] I2C Slave: Dado 0x%h escrito no endereço 0x%h.", $time, received_data_byte, addr_pointer);
    //                         addr_pointer <= addr_pointer + 1;
    //                         i2c_send_ack;
    //                     end
    //                 end
    //             end else begin // Master READ
    //                 master_read_loop: begin
    //                     // Loop to send data bytes
    //                     while(1) begin
    //                         i2c_send_byte(mpu_memory[addr_pointer]);
    //                         $display("[%0t ns] I2C Slave: Enviando dado 0x%h do endereço 0x%h.", $time, mpu_memory[addr_pointer], addr_pointer);
    //                         addr_pointer <= addr_pointer + 1;

    //                         // Get ACK from master
    //                         i2c_get_ack(ack_from_master);
    //                         if (ack_from_master == 1'b1) begin // NACK
    //                             $display("[%0t ns] I2C Slave: NACK recebido do master. Fim da leitura.", $time);
    //                             disable master_read_loop;
    //                         end else begin
    //                             $display("[%0t ns] I2C Slave: ACK recebido do master.", $time);
    //                         end
    //                     end
    //                 end
    //             end
    //         end else begin
    //             // Address mismatch - do nothing (NACK is implied by releasing the bus)
    //             $display("[%0t ns] I2C Slave: Endereço não corresponde (0x%h).", $time, received_addr_byte[7:1]);
    //         end
            
    //         wait (SCL_wire === 1'b1 && SDA_wire === 1'b1);
    //         $display("[%0t ns] I2C Slave: STOP detectado!", $time);
    //     end
    // end

endmodule

