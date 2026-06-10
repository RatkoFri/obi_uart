
module uart_system(
    input logic clock,
    input logic reset,
    input logic [31:0] limit, 
    input logic tx_start,
    input logic rx_start,
    input logic [7:0] data_in,
    output logic tx,
    output logic tx_done,
    input logic rx,
    output logic [7:0] data_out,
    output logic rx_done
);

    logic baud_rate_tick;
    logic baud_rst;
    logic local_reset;

    
    assign local_reset = reset | baud_rst;

    baud_rate_generator #(
        .PRESCALER_WIDTH(32)
    ) baud_rate_transmitter (
        .clock(clock),
        .reset(local_reset),
        .limit(limit),
        .baud_rate_tick(baud_rate_tick)
    );

    uart_transmitter_fsm #(
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

    logic local_rx_done;


    
    logic [31:0] limit_receiver;
    logic sample_tick_receiver;
    // 16 times the limit for transmitter
    // over-sampling for receiver to achieve better stability
    assign limit_receiver = limit >> 4; 

    logic baud_rst_receiver, baud_rst_local2; 
    assign baud_rst_receiver = reset | baud_rst_local2; // Reset the receiver's baud rate generator

    baud_rate_generator #(
        .PRESCALER_WIDTH(32)
    ) baud_rate_receiver (
        .clock(clock),
        .reset(baud_rst_receiver),
        .limit(limit_receiver),
        .baud_rate_tick(sample_tick_receiver)
    );

    uart_receiver_fsm #(
        .DBITS(8),
        .SBITS(1)
    ) uart_receiver_fsm_inst (
        .clock(clock),
        .reset(reset),
        .sample_tick(sample_tick_receiver),
        .rx(rx),
        .data_out(data_out),
        .baud_rst(baud_rst_local2), // not used in this FSM, but could be used to reset the baud rate generator if needed
        .rx_done(local_rx_done)  
    );

    // if rx_start == 0  
    // block the reading from uart
    assign rx_done = local_rx_done & rx_start;

endmodule

module uart_transmitter_fsm #(
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


module uart_receiver_fsm #(
    parameter DBITS = 8,
    parameter SBITS = 1
) (
    input logic clock,
    input logic reset,
    input logic sample_tick,
    input logic rx, 
    output logic [DBITS-1:0] data_out,
    output logic baud_rst, // used for baud rate generator reset
    output logic rx_done
    );

    // define the parameters
    localparam STOP_TICKS = SBITS*16;
    

    // define the states
    typedef enum logic [1:0] { // binary encoding
        IDLE,
        START,
        DATA,
        STOP
    } state_uart_t;
    
    state_uart_t state, next_state;

    // signal declarations 
    logic [DBITS-1:0] shift_reg, shift_reg_next;
    logic [3:0] s_counter, s_counter_next; // counter for sample_tick
    logic [3:0] n_counter, n_counter_next; // counter for number of symbols 
    logic rx_done_next, baud_rst_next;
    // state register
    always_ff @(posedge clock) begin : state_reg
        if (reset) begin
            state <= IDLE;
            shift_reg <= 0;
            s_counter <= 0;
            n_counter <= 0;
            rx_done <= 0;
            baud_rst <= 0;
        end
        else begin
            state <= next_state;
            shift_reg <= shift_reg_next;
            s_counter <= s_counter_next;
            n_counter <= n_counter_next;
            rx_done <= rx_done_next;
            baud_rst <= baud_rst_next;
        end
    end

    // next state logic
    always_comb begin : next_state_logic
        // default values, otherwise we will have a latch
        // need to cover all the cases 
        next_state = state;
        rx_done_next = 0;
        shift_reg_next = shift_reg;
        s_counter_next = s_counter;
        n_counter_next = n_counter;
        baud_rst_next = 0;
        
        case (state)
            IDLE : begin
                if (rx == 0) begin
                    next_state = START;
                    s_counter_next = 0;
                    rx_done_next = 0;
                    baud_rst_next = 1'b1; // reset the baud rate generator to synchronize with the start bit
                end
            end
            START : begin
                if(sample_tick) begin
                    if (s_counter == 7) begin
                        // cannot do n_counter = 0, two blocks power the same signal 
                        n_counter_next = 0;
                        s_counter_next = 0;
                        // do not forget to update state 
                        next_state = DATA;
                    end else begin
                        s_counter_next = s_counter + 1;
                    end
                end
            end 
            DATA : begin
                if(sample_tick) begin
                   if (s_counter == 15) begin
                        s_counter_next = 0;
                        shift_reg_next = {rx,shift_reg[DBITS-1:1]};
                        if (n_counter == DBITS-1) begin
                            next_state = STOP;
                        end else begin
                            n_counter_next = n_counter + 1;
                        end
                   end  else begin
                        s_counter_next = s_counter + 1;
                   end
                end
            end
            STOP : begin
                if (sample_tick) begin
                    if (s_counter == STOP_TICKS - 1) begin
                        rx_done_next = 1;
                        next_state = IDLE;
                    end else begin
                        s_counter_next = s_counter + 1;
                    end
                end
            end
        endcase
    end

    // output 
    assign data_out = shift_reg;

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

    // instantiate register
    
    register #(
        .DTYPE(logic [DATA_WIDTH-1:0]),
        .RESET_VALUE(0)
    ) one_word_buffer (
        .clk(clock),
        .rstn(~reset),
        .ce(write_req),
        .in(r_input),
        .out(r_out)
    );


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

