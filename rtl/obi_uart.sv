module obi_uart #(
    parameter integer ADDR_WIDTH = 32,
    parameter integer DATA_WIDTH = 32
) (
    input logic clk_i,
    input logic rstn_i,

    input   logic                     obi_areq_i,
    output  logic                     obi_agnt_o,
    input   logic [ADDR_WIDTH-1:0]    obi_aaddr_i,
    input   logic [DATA_WIDTH-1:0]    obi_awdata_i,

    input   logic                     obi_awe_i,
    input   logic [DATA_WIDTH/8-1:0]  obi_abe_i,

    output  logic                     obi_rvalid_o,
    input   logic                     obi_rready_i,
    output  logic [DATA_WIDTH-1:0]    obi_rdata_o,
    output  logic                     obi_rerr_o,

    // Output complete
    output  logic                     tx_o
);
    
    localparam integer UartConfRegOffset = 0;
    localparam integer UartSpeedRegOffset = 4;
    localparam integer UartTxRegOffset = 8;
    localparam integer UartStatusRegOffset = 12;

    logic [DATA_WIDTH-1:0] uart_conf_reg;
    logic [DATA_WIDTH-1:0] uart_speed_reg;
    logic [DATA_WIDTH-1:0] uart_tx_reg;
    logic [DATA_WIDTH-1:0] uart_status_reg;

    logic tx_done;
    logic tx_empty;

    // OBI wrapper signals
    typedef enum logic {
        ADDR,
        RESP
    } state_t;
    state_t state, next_state;

    logic obi_a_fire;
    logic obi_r_fire;
    logic capture;


    // Write interface signals
    logic wr_en[2:0]; 
    logic [DATA_WIDTH-1:0] write_data_mask;


    // Read interface signals
    logic rd_en;
    logic [DATA_WIDTH-1:0] read_data_mux_out;

    // BEGIN: OBI wrapper
    register  #(
        .DTYPE(state_t),
        .RESET_VALUE(ADDR)     
    ) obi_fsm_state_reg
        (
        .clk(clk_i),
        .rstn(rstn_i),
        .ce(1'b1), // Always enable to capture the input state
        .in(next_state),
        .out(state)
    );

    assign obi_a_fire = obi_areq_i && obi_agnt_o;
    assign obi_r_fire = obi_rready_i && obi_rvalid_o;

    // State transition logic
    always_comb begin : OBI_SLAVE_next_state
        next_state = state;
        case (state)
            ADDR: begin
                if (obi_a_fire) begin // handshake for address phase, when there is a valid request and the slave is granted access to the bus, move to response phase
                    next_state = RESP;
                end
            end
            RESP: begin
                if (obi_r_fire) begin // handshake for response phase, when there is a valid response and the master is ready to accept it, move back to address phase
                    next_state = ADDR;
                end
            end
        endcase
    end

    assign obi_agnt_o = (state == ADDR); // Grant access to the bus during address phase
    assign obi_rvalid_o = (state == RESP); // Grant access to the bus during response phase

    // END: OBI wrapper

    // BEGIN: OBI write interface
    
    assign wr_en[0] = obi_a_fire & obi_awe_i & (obi_aaddr_i[6:0] == UartConfRegOffset); // Needs to ensure write request is valid and handshake occured in address phase
    assign wr_en[1] = obi_a_fire & obi_awe_i & (obi_aaddr_i[6:0] == UartSpeedRegOffset); // Write enable for compare low register
    assign wr_en[2] = obi_a_fire & obi_awe_i & (obi_aaddr_i[6:0] == UartTxRegOffset); // Write enable for compare high register

    assign write_data_mask = {{8{obi_abe_i[3]}},{8{obi_abe_i[2]}},{8{obi_abe_i[1]}},{8{obi_abe_i[0]}}}; 


    register  #(
        .DTYPE(logic [DATA_WIDTH-1:0]),
        .RESET_VALUE('0)     
    ) timer_conf_reg
        (
        .clk(clk_i),
        .rstn(rstn_i),
        .ce(wr_en[0]),
        .in(obi_awdata_i & write_data_mask),
        .out(uart_conf_reg)
    );


    register  #(
        .DTYPE(logic [DATA_WIDTH-1:0]),
        .RESET_VALUE('0)     
    ) compare_low_reg
        (
        .clk(clk_i),
        .rstn(rstn_i),
        .ce(wr_en[1]),
        .in(obi_awdata_i & write_data_mask), // Apply byte-enable mask to the incoming data
        .out(uart_speed_reg)
    );

    interface_circuit #(
        .DATA_WIDTH(DATA_WIDTH)
    ) uart_tx_buffer (
        .clock(clk_i),
        .reset(~rstn_i),
        .r_input(obi_awdata_i), 
        .write_req(wr_en[2]),
        .read_req(tx_done), 
        .rx_empty(tx_empty), 
        .r_out(uart_tx_reg) 
    );
    // END: OBI write interface

    // BEGIN: OBI read interface
    assign rd_en = obi_a_fire & !obi_awe_i ; 

    assign uart_status_reg = {{31{1'b0}}, tx_empty}; // Indicate if the transmit buffer is empty

    always_comb begin 
        read_data_mux_out = '0; // Default to zero
        if(rd_en) begin
            case(obi_aaddr_i[6:0])
                UartConfRegOffset: read_data_mux_out = uart_conf_reg;
                UartSpeedRegOffset: read_data_mux_out = uart_speed_reg;
                UartTxRegOffset: read_data_mux_out = uart_tx_reg;
                UartStatusRegOffset: read_data_mux_out = uart_status_reg;
                default: read_data_mux_out = '0; // Default to zero for unmapped addresses
            endcase
        end 
    end

    // Register to hold the read data during the response phase
    register  #(
        .DTYPE(logic [DATA_WIDTH-1:0]),
        .RESET_VALUE('0)     
    ) read_data_reg
        (
        .clk(clk_i),
        .rstn(rstn_i),
        .ce(obi_a_fire), // Capture the read data at the beginning of the response phase
        .in(read_data_mux_out),
        .out(obi_rdata_o)
    );
    // END: logic for OBI read interface

    // UART logic

    transmitter_system transmitter_system_inst (
        .clock(clk_i),
        .reset(~rstn_i),
        .limit(uart_speed_reg[15:0]), // Use lower 16 bits of the speed register as the baud rate limit
        .tx_start(uart_conf_reg[0] & !tx_empty), // Start transmission when there is data in the buffer
        .data_in(uart_tx_reg[7:0]), // Use lower 8 bits of the tx register as the data to transmit
        .tx(tx_o),
        .tx_done(tx_done)
    );


    // error handling logic
    always_comb begin 
        obi_rerr_o = 1'b0; // Default to no error
        if (state == RESP) begin // Only check for read errors during response phase for read transactions
            case(obi_aaddr_i[6:0])
                UartConfRegOffset, UartSpeedRegOffset, UartTxRegOffset, UartStatusRegOffset: obi_rerr_o = 1'b0; // Valid addresses
                default: obi_rerr_o = 1'b1; // Invalid address
            endcase
        end
    end

endmodule

// Instantiation template:
// obi_uart_v2 #(
//     .ADDR_WIDTH(32),
//     .DATA_WIDTH(32)
// ) obi_uart_v2_inst (
//     .clk_i       (),
//     .rstn_i      (),
//     .obi_areq_i  (),
//     .obi_agnt_o  (),
//     .obi_aaddr_i (),
//     .obi_awdata_i(),
//     .obi_awe_i   (),
//     .obi_abe_i   (),
//     .obi_rvalid_o(),
//     .obi_rready_i(),
//     .obi_rdata_o (),
//     .obi_rerr_o  (),
//     .tx_o        ()
// );


module baud_rate_generator // General Purpose counter        
    #(parameter PRESCALER_WIDTH = 4)
    (
        input logic clock,
        input logic reset,
        input logic [PRESCALER_WIDTH-1:0] limit,
        output logic baud_rate_tick
    );

    logic [PRESCALER_WIDTH-1:0] count;

    // when the counter reaches the limit, the sample_tick signal is generated

    always_ff @(posedge clock) begin
        if(reset) begin
            count <= 0;
        end else begin
            if(count == limit-1) begin
                count <= 0;
            end else begin
                count <= count + 1;
            end
        end
    end

    assign baud_rate_tick = (count == limit-1);
endmodule

module uart_fsm #(
    parameter DATA_WIDTH = 8
    ) 
(
    input logic clock,
    input logic reset,
    input logic [DATA_WIDTH-1:0] data_in,
    input logic baud_rate_tick,
    input logic tx_start,
    output logic tx,
    output logic tx_done,
    output logic baud_rst // used for baud rate generator reset
);

    // define the states
    typedef enum logic [1:0] { // binary encoding
        IDLE,
        START,
        DATA,
        STOP
    } state_uart_t;
    
 
    state_uart_t state, next_state;

    // signal declarations 
    logic [DATA_WIDTH-1:0] b_reg, b_reg_next;
    logic [3:0] n_counter, n_counter_next; // counter for number of symbols 
    logic tx_done_next, tx_reg, tx_reg_next;


    // state register
    always_ff @(posedge clock) begin
        if (reset) begin
            state <= IDLE;
            b_reg <= 0;
            n_counter <= 0;
            tx_reg <= 1; // idle state state of the tx line
        end
        else begin
            state <= next_state;
            b_reg <= b_reg_next;
            n_counter <= n_counter_next;
            tx_reg <= tx_reg_next;
        end
    end

    // state transition logic
    always_comb begin
        next_state = state;
        b_reg_next = b_reg;
        n_counter_next = n_counter;
        tx_done = 0;
        tx_reg_next = tx_reg;
        baud_rst = 1'b0;

        case (state)
            IDLE : begin
                if(tx_start) begin
                    next_state = START;
                    b_reg_next = data_in;
                    baud_rst = 1'b1;
                end
            end 
            START : begin
                tx_reg_next = 1'b0;
                if (baud_rate_tick) begin
                    next_state = DATA;
                    n_counter_next = 0;
                end
            end
            DATA : begin
                tx_reg_next = b_reg[0];
                if (baud_rate_tick) begin
                    if (n_counter == DATA_WIDTH-1) begin
                        next_state = STOP;
                    end
                    else begin
                        n_counter_next = n_counter + 1;
                        b_reg_next = {1'b0, b_reg[7:1]};
                    end
                end
            end
            STOP : begin
                tx_reg_next = 1'b1;
                if (baud_rate_tick) begin
                    begin
                        next_state = IDLE;
                        tx_done= 1'b1;
                    end
                end
            end
        endcase
    end

    assign tx = tx_reg;
endmodule

module transmitter_system(
    input logic clock,
    input logic reset,
    input logic [15:0] limit, 
    input logic tx_start,
    input logic [7:0] data_in,
    output logic tx,
    output logic tx_done
);

    logic baud_rate_tick;
    logic baud_rst;
    logic local_reset;

    
    assign local_reset = reset | baud_rst;

    baud_rate_generator #(
        .PRESCALER_WIDTH(16)
    ) baud_rate_generator_inst (
        .clock(clock),
        .reset(local_reset),
        .limit(limit),
        .baud_rate_tick(baud_rate_tick)
    );

    uart_fsm #(
        .DATA_WIDTH(8)
    ) uart_fsm_inst (
        .clock(clock),
        .reset(reset),
        .data_in(data_in),
        .baud_rate_tick(baud_rate_tick),
        .tx_start(tx_start),
        .tx(tx),
        .tx_done(tx_done),
        .baud_rst(baud_rst)
    );


endmodule


module interface_circuit #(
    parameter DATA_WIDTH = 8
) (
    input logic clock, 
    input logic reset,
    input logic [DATA_WIDTH-1:0] r_input, 
    input logic write_req, // receiving done  
    input logic read_req, // read uart request 
    output logic rx_empty,
    output logic [DATA_WIDTH-1:0] r_out
);
    
    // one word buffer 
    always_ff @(posedge clock) begin : OneWordBuffer
        if (reset) begin
            r_out <= 0;
        end else begin
            if (write_req) begin
                r_out <= r_input;
            end
        end
    end

    // rx_empty signal generation 
    logic counter;

    always_ff @( posedge clock ) begin : blockName
        if(reset) begin
            counter <= 0;
        end else begin
            if (write_req) begin
                counter <= 1; // data is written to the buffer, not empty anymore
            end else if (read_req) begin
                counter <= 0; // data is read from the buffer, empty again
            end
        end
    end

    assign rx_empty = counter == 0;

endmodule




// Instantiation template:
// obi_uart_v2 #(
//     .ADDR_WIDTH(32),
//     .DATA_WIDTH(32)
// ) obi_uart_v2_inst (
//     .clk_i       (),
//     .rstn_i      (),
//     .obi_areq_i  (),
//     .obi_agnt_o  (),
//     .obi_aaddr_i (),
//     .obi_awdata_i(),
//     .obi_awe_i   (),
//     .obi_abe_i   (),
//     .obi_rvalid_o(),
//     .obi_rready_i(),
//     .obi_rdata_o (),
//     .obi_rerr_o  (),
//     .tx_o        ()
// );
