`timescale 1ns / 1ps

`include "Constants.v"

module CtrlVelST(
	input  wire                 clk,

    /*-----------------<control signals>-----------------*/
    // power
    input  wire                 enc_ctrl_enable,

    // controller mode
    input  wire          [2: 0] enc_ctrl_mode,

    // feedbacks
    input  wire          [15:0] cur_fb,
    input  wire                 cur_fb_ready,
    input  wire          [25:0] enc_fb,
    input  wire                 enc_dir_fb,
    input  wire                 enc_val_ready,

    // command
    input  wire                 enc_dir_cmd,
    input  wire          [25:0] enc_cmd,

    // errors
    output reg  signed   [31:0] err_sync,
    output reg  signed   [31:0] err_prev,
    output reg  signed   [31:0] err_i,

    // gains
    input  wire signed   [31:0] Kp,
    input  wire signed   [31:0] Ki,
    input  wire signed   [31:0] Kd,
    input  wire          [6 :0] Kp_shift, 
    input  wire          [6 :0] Ki_shift,
    input  wire          [6 :0] Kd_shift,
    input  wire signed   [15:0] up_clamp,
    input  wire signed   [15:0] ui_clamp,
    input  wire signed   [15:0] ud_clamp,
    input  wire signed   [15:0] upid_clamp,
    input  wire signed   [15:0] ui_windup,

    // d term update 
    input  wire                 d_update,

    // DAC busy
    input  wire                 dac_busy,

    // output to DAC
    output reg                  velCtrlReady,   
    output reg           [15:0] velCtrlOutput,
    

    /*---------------<system identification>-------------*/
    input wire                  sys_idt_enable,
    input wire           [15:0] cur_step,

    /*-------------------<debug signals>-----------------*/
    output wire  signed   [15:0] p16_debug,
    output wire  signed   [15:0] i16_debug,
    output wire  signed   [15:0] d16_debug,
    output wire           [15:0] pid16_debug,
    output wire                  sat_debug
);

// ----------------------------------------
// local parameters
// ----------------------------------------
// if greater than the largest positive number,
// or lower than the smallest negative number,
// then set intermediate output to below constants
parameter signed max_pos16 = 16'sh7fff;
parameter signed min_neg16 = 16'sh8000;
parameter signed max_pos32 = 32'sh7fffffff;
parameter signed min_neg32 = 32'sh80000000;

parameter cur_offset = 16'h8000;

// ----------------------------------------
// trigger signals
// ----------------------------------------
// encoder period new value trigger
reg  enc_val_ready_sync;
reg  enc_val_ready_prev;
wire enc_val_ready_trigger;

always @(posedge(clk)) 
begin
    enc_val_ready_sync <= enc_val_ready;
    enc_val_ready_prev <= enc_val_ready_sync;
end
assign enc_val_ready_trigger = (!enc_val_ready_prev) && enc_val_ready_sync;

// d term update new value trigger
reg  d_update_sync;
reg  d_update_prev;
wire d_update_trigger;

always @(posedge(clk)) 
begin
    d_update_sync <= d_update;
    d_update_prev <= d_update_sync;
end
assign d_update_trigger = (!d_update_prev) && d_update_sync;

// ----------------------------------------
// debug
// ----------------------------------------
assign p16_debug = p16;
assign i16_debug = i16;
assign d16_debug = d16;
assign pid16_debug = p16_debug + d16_debug + i16_debug;
assign sat_debug = velCtrlSaturate;

// ----------------------------------------
// combinatorial logics
// ----------------------------------------
// 64 bits
reg signed [63:0] p64;
reg signed [63:0] i64;
reg signed [63:0] d64;

// 16 bits
wire signed [15:0] p16;
wire signed [15:0] i16;
wire signed [15:0] d16;
wire signed [15:0] pid16;

// 18 bit
reg signed [17:0] pid18;

// truncate to 16 bits to fit current bit format
assign p16 = (p64 >=  up_clamp)  ?  up_clamp :
             ((p64 <= -up_clamp) ? -up_clamp :
             ((p64 >= max_pos16) ? max_pos16 :
             ((p64 <= min_neg16) ? min_neg16 : p64[15:0])));

assign i16 = (i64 >=  ui_clamp)  ?  ui_clamp :
             ((i64 <= -ui_clamp) ? -ui_clamp :
             ((i64 >= max_pos16) ? max_pos16 :
             ((i64 <= min_neg16) ? min_neg16 : i64[15:0])));

assign d16 = (d64 >=  ud_clamp)  ?  ud_clamp :
             ((d64 <= -ud_clamp) ? -ud_clamp :
             ((d64 >= max_pos16) ? max_pos16 :
             ((d64 <= min_neg16) ? min_neg16 : d64[15:0])));

assign pid16 = (pid18 >= max_pos16)  ? max_pos16 :
               ((pid18 <= min_neg16) ? min_neg16 : pid18[15:0]);

// integral error overflow detection
wire signed [31:0] err_i_tmp;
assign             err_i_tmp = err_i + err_sync;

wire   overflow;
assign overflow = (~err_i_tmp[31] & err_i[31] & err_sync[31]) | 
                  (err_i_tmp[31] & (~err_i[31]) & (~err_sync[31]));

// control output satuaration detection
reg   velCtrlSaturate;

// --------------------------------------------------
// implementation
//
// NOTE:
//      P term runs at sysclk, which is ~ 50 MHz. There is no need to do this as DAC module can
//      only process data at ~340kHz (49.152MHz/128).
//      I term, here as current feedback, is dependent on the sampling frequency of ADC module,
//      which runs at ~120kHz.
//      D term should not be computed that fast using sysclk because the difference might not
//      be significant between each sysclk edge. For now, we could use the same rate as ADC.
// --------------------------------------------------

/*<state machine>*/
reg [3:0] state;
parameter
    ST_CAL_IDLE  = 4'd0,
    ST_CAL_ERR   = 4'd1,
    ST_CAL_64    = 4'd2,
    ST_CAL_SUM   = 4'd3,
    ST_CAL_CLAMP = 4'd4,
    ST_CAL_OUT   = 4'd5,
    ST_SYS_IDT   = 4'd6;
    

initial 
begin
    state = ST_CAL_IDLE;
end

/*
* Theory of Operation:
* <ST_CAL_idle> : waiting for command, e.g. system identify, PID procedure, reset, etc.
* <ST_CAL_ERR>  : when encoder feedback is ready, computer error                 ---> <ST_CAL_64>   loop itself if not new input
* <ST_CAL_64>   : error is computed, calculate 64 bit P, I, D control output     ---> <ST_CAL_SUM>
* <ST_CAL_SUM>  : sum clamped 16 bits P, I, D to calculate final control output; ---> <ST_CAL_OUT>
* <ST_CAL_OUT>  : based on saturation detection, update error integral;          ---> <ST_CAL_ERR>
* <ST_SYS_IDT>  : system identification
*/

always @(posedge(clk)) begin
    case (state)

        ST_CAL_IDLE:
        begin
            if (enc_ctrl_enable && enc_val_ready_trigger)
            begin
                // controller enabled, new encoder feedback received, compute error
                velCtrlReady <= 0;

                state <= ST_CAL_ERR;
            end

            if (enc_ctrl_enable && !enc_val_ready_trigger) 
            begin
                velCtrlReady <= 0; // TODO: should we reset ready bit here?

                // controller enabled, no new value received, wait here
                state <= ST_CAL_IDLE;
            end

            if (!enc_ctrl_enable && !sys_idt_enable) 
            begin
                velCtrlReady <= 0;

                // controller disabled, reset error etc.
                err_i <= 0;
                err_sync <= 0;

                state <= ST_CAL_IDLE;
            end

            if (!enc_ctrl_enable && sys_idt_enable) 
            begin
                velCtrlReady <= 0;
                
                // controller disabled, in system identification mode, i.e. command step current
                state <= ST_SYS_IDT;
            end
        end


        ST_CAL_ERR: // 1
        begin
            err_sync <= {6'b0, enc_cmd} - {6'b0, enc_fb}; 

            state <= ST_CAL_64;
        end


        ST_CAL_64: // 2
        begin
            // P output
            p64 <= (Kp * err_sync) >>> Kp_shift;
            // I output
            i64 <= (Ki * err_i) >>> Ki_shift;
            // D output
            d64 <= (Kd * (err_sync - err_prev)) >>> Kd_shift;

            state <= ST_CAL_SUM;
        end


        // NOTE: p16 + d16 + i16 might overflow 16'sh7fff, need overflow detection on this
        ST_CAL_SUM: // 3
        begin
            pid18 <= p16 + d16 + i16;

            state <= ST_CAL_CLAMP;
        end
        

        ST_CAL_CLAMP: // 4
        begin
            velCtrlOutput <= (pid16 >=  upid_clamp) ?  upid_clamp + cur_offset :
                             ((pid16 <= -upid_clamp) ? -upid_clamp + cur_offset : pid16 + cur_offset);

            velCtrlSaturate <= (pid16 >=  upid_clamp) ?  1 : 
                               ((pid16 <= -upid_clamp) ?  1 : 0);

            velCtrlReady  <= 1'b1;

            state <= ST_CAL_OUT;
        end


        ST_CAL_OUT: // 4
        begin
            err_i <= velCtrlSaturate ?  err_i :
                            overflow ? (err_sync[31] ? min_neg32 : max_pos32) :
                                        err_i_tmp;

            velCtrlSaturate <= 0;
            
            state <= ST_CAL_IDLE;
        end


        ST_SYS_IDT:
        begin
            // system identification mode disabled, return to IDLE state
            if (!sys_idt_enable)
            begin
                state <= ST_CAL_IDLE;
            end

            velCtrlOutput <= (cur_step <= 16'h7000) ? 16'h7000 : 
                             (cur_step >= 16'h9000) ? 16'h9000 : cur_step;

            velCtrlReady  <= 1'b1;

            state <= ST_SYS_IDT;
        end

    endcase
end

// update previous error, for D control

initial
begin
    err_prev = 0;
end

always @(posedge(clk))
begin
    if (enc_ctrl_enable && d_update_trigger) // update at ADC sampling frequency, could use other rate if needed
    begin
        // update previous tracking error
        err_prev <= err_sync;
    end
end
// TODO: predictive control -- add acceleration to current velocity to estimate velocity at next clock cycle

endmodule










