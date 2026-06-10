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
    output  logic                     tx_o,
    input   logic                     rx_i
);
    
    localparam integer UartConfRegOffset = 0;
    localparam integer UartSpeedRegOffset = 4;
    localparam integer UartTxRegOffset = 8;
    localparam integer UartRxRegOffset = 12;
    localparam integer UartStatusRegOffset = 16;

    logic [DATA_WIDTH-1:0] uart_conf_reg;
    logic [DATA_WIDTH-1:0] uart_speed_reg;
    logic [DATA_WIDTH-1:0] uart_tx_reg;
    logic [DATA_WIDTH-1:0] uart_rx_reg;
    logic [DATA_WIDTH-1:0] uart_status_reg;

    logic tx_done;
    logic tx_empty;
    logic rx_done;
    logic [7:0] uart_rx_data; // Data received from the UART receiver FSM, 8 bits for one byte of data
    logic rx_empty;
    logic rd_reg_en;

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
    assign rd_reg_en = rd_en & (obi_aaddr_i[6:0] == UartRxRegOffset); 
    
    
    
    interface_circuit #(
        .DATA_WIDTH(DATA_WIDTH)
    ) uart_rx_buffer (
        .clock(clk_i),
        .reset(~rstn_i),
        .r_input({{(DATA_WIDTH-8){1'b0}}, uart_rx_data}), 
        .write_req(rx_done),
        .read_req(rd_reg_en), 
        .rx_empty(rx_empty), 
        .r_out(uart_rx_reg) 
    );


    assign uart_status_reg = {{30{1'b0}},!rx_empty, tx_empty}; // Indicate if the transmit buffer is empty

    always_comb begin 
        read_data_mux_out = '0; // Default to zero
        if(rd_en) begin
            case(obi_aaddr_i[6:0])
                UartConfRegOffset: read_data_mux_out = uart_conf_reg;
                UartSpeedRegOffset: read_data_mux_out = uart_speed_reg;
                UartTxRegOffset: read_data_mux_out = uart_tx_reg;
                UartRxRegOffset: read_data_mux_out = uart_rx_reg;
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

    uart_system uart_inst (
        .clock(clk_i),
        .reset(~rstn_i),
        .limit(uart_speed_reg), 
        .tx_start(wr_en[2]), // Start transmission when there is a write to the Tx register
        .rx_start(1'b1), // Always ready to receive data
        .data_in(uart_tx_reg[7:0]), // Only the least significant byte is used for transmission
        .tx(tx_o),
        .tx_done(tx_done),
        .rx(rx_i),
        .data_out(uart_rx_data), // Capture the received data into uart_rx_data
        .rx_done(rx_done)
    );


    // error handling logic
    always_comb begin 
        obi_rerr_o = 1'b0; // Default to no error
        if (state == RESP) begin // Only check for read errors during response phase for read transactions
            case(obi_aaddr_i[6:0])
                UartConfRegOffset, UartSpeedRegOffset, UartTxRegOffset, UartStatusRegOffset, UartRxRegOffset: obi_rerr_o = 1'b0; // Valid addresses
                default: obi_rerr_o = 1'b1; // Invalid address
            endcase
        end
    end

endmodule

// Instantiation template:
// obi_uart #(
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
//     .tx_o        (),
//     .rx_i        ()
// );

