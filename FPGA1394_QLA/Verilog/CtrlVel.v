`timescale 1ns / 1ps

`include "Constants.v"

module CtrlVel(
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
    input  wire          [15:0] upid_clamp,
    input  wire signed   [15:0] ui_windup,

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

// ----------------------------------------
// controller internal signals
// ----------------------------------------
// 64 bits
wire signed [63:0] p64;
wire signed [63:0] i64;
wire signed [63:0] d64;

// 16 bits
wire signed [15:0] p16;
wire signed [15:0] i16;
wire signed [15:0] d16;

// truncate to 16 bits to fit current bit format
assign p16 = p64 >  up_clamp ?  up_clamp :
             p64 < -up_clamp ? -up_clamp :
             p64 > max_pos16 ? max_pos16 :
             p64 < min_neg16 ? min_neg16 :
             p64[15:0];

assign d16 = d64 >  ud_clamp ?  ud_clamp :
             d64 < -ud_clamp ? -ud_clamp :
             d64 > max_pos16 ? max_pos16 :
             d64 < min_neg16 ? min_neg16 :
             d64[15:0];

assign i16 = i64 >  ui_clamp ?  ui_clamp :
             i64 < -ui_clamp ? -ui_clamp :
             i64 > max_pos16 ? max_pos16 :
             i64 < min_neg16 ? min_neg16 :
             i64[15:0];

// control output as wire, used in intergral windup
wire [15:0] velCtrlOutputWire;

assign velCtrlOutputWire = p16 + d16 + i16 + 16'h8000;

// ----------------------------------------
// integral error overflow & saturate handle
// ----------------------------------------
// integral error overflow detection
wire signed [31:0] err_i_tmp;
assign err_i_tmp = err_i + err_sync;

wire overflow;
assign overflow = (~err_i_tmp[31] & err_i[31] & err_sync[31]) | 
                  (err_i_tmp[31] & (~err_i[31]) & (~err_sync[31]));

// control output satuaration detection
wire velCtrlSaturate;
assign velCtrlSaturate = (velCtrlOutputWire >=  upid_clamp) ?  1 : 
                         (velCtrlOutputWire <= -upid_clamp) ?  1 : 0;

// TODO: add the following as a second mode of windup
//  (err_sync <= -ui_windup) || (err_sync >= ui_windup);

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

// current feedback new value trigger
reg  cur_fb_ready_sync;
reg  cur_fb_ready_prev;
wire cur_fb_ready_trigger;

always @(posedge(clk)) 
begin
    cur_fb_ready_sync <= cur_fb_ready;
    cur_fb_ready_prev <= cur_fb_ready_sync;
end
assign cur_fb_ready_trigger = (!cur_fb_ready_prev) && cur_fb_ready_sync;

// ----------------------------------------
// debug
// ----------------------------------------
assign p16_debug = p16;
assign i16_debug = i16;
assign d16_debug = d16;
assign pid16_debug = p16_debug + d16_debug + i16_debug;
assign sat_debug = velCtrlSaturate;

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

// -- P output
assign p64 = (Kp * err_sync) >>> Kp_shift;

// -- I output
assign i64 = (Ki * err_i_tmp) >>> Ki_shift;

// -- D output
assign d64 = (Kd * (err_sync - err_prev)) >>> Kd_shift;

// update previous error, for D control
always @(posedge(clk))
begin
    if (cur_fb_ready_trigger) // update at ADC sampling frequency, could use other rate if needed
    begin
        // update previous tracking error
        err_prev <= err_sync;
    end
end

// update sync error, integral error, and contorller output
always @(posedge(clk))
begin

    // controller disabled, in system identification mode, i.e. command step current
    if (!enc_ctrl_enable && sys_idt_enable) 
    begin
        velCtrlOutput <= (cur_step <= 16'h6800) ? 16'h6800 : 
                         (cur_step >= 16'h8800) ? 16'h8800 : cur_step;

        velCtrlReady  <= dac_busy ? 0 : 1'b1;
    end

    // controller enabled, new measured value received, in classic PID mode
	else if (enc_ctrl_enable && enc_val_ready_trigger && enc_ctrl_mode == `OFF_ENC_CLASSIC) 
    begin

        // <EXPERIMENTAL IMPLEMENTATION>

        // -- update current tracking error
        // check if direction is matched, if match, error is the difference; if match, error is the sum
        // err_sync <= (enc_dir_cmd == enc_dir_fb) ? {6'b0, enc_cmd} - {6'b0, enc_fb} : 32'h03FFFFFF - {6'b0, enc_fb} + {6'b0, enc_cmd};
        err_sync <= {6'b0, enc_cmd} - {6'b0, enc_fb}; 
                                                  
        // -- update error integration
        // check if control output would saturate, if so, don't accumulate error here due to windup
        err_i <= velCtrlSaturate ? err_i :
                 overflow ? (err_sync[31] ? min_neg32 : max_pos32) :
                 err_i_tmp;

        // -- ctrl output
        velCtrlOutput <= (velCtrlOutputWire >=  upid_clamp) ?  upid_clamp : 
                         (velCtrlOutputWire <= -upid_clamp) ? -upid_clamp : 
                          velCtrlOutputWire;

        velCtrlReady  <= dac_busy ? 0 : 1'b1;

	end 
    
    // controller enabled, new measured value received, in current-feedback-dependent mode
    else if (enc_ctrl_enable && enc_val_ready_trigger && enc_ctrl_mode == `OFF_ENC_DEPENDENT) 
    begin

		// update current tracking error
        err_sync <= {6'b0, enc_cmd} - {6'b0, enc_fb};

        // ctrl output
        velCtrlOutput <= (p16 + d16 + cur_fb >  upid_clamp) ?  upid_clamp : 
                         (p16 + d16 + cur_fb < -upid_clamp) ? -upid_clamp : 
                          p16 + d16 + cur_fb;

        velCtrlReady <= dac_busy ? 0 : 1'b1;

	end 
    
    // controller enabled, no new measured value
    else if (enc_ctrl_enable && !enc_val_ready_trigger) 
    begin

		velCtrlReady <= 0;

	end 
    
    // controller disabled
    else if (!enc_ctrl_enable && !sys_idt_enable) 
    begin

        err_i <= 0;
        err_sync <= 0;

        velCtrlReady <= 0;

	end
end

// TODO: predictive control -- add acceleration to current velocity to estimate velocity at next clock cycle

endmodule










