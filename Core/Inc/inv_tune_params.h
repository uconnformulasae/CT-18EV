#ifndef INV_TUNE_PARAMS_H
#define INV_TUNE_PARAMS_H

#include <stdint.h>

/*
 * CAN parameter addresses (CAN Protocol V6.3 2.3.4, CM SW manual 0A-0163-02),
 * in RMS GUI export order. The CRC runs over them in this order, so only
 * ever append. tools/inv_tune_decode.py reads names and scaling from the
 * comments here, keep them as "Name (unit) xScale, signed".
 *
 * In neither doc, so not read: Angle_Advance_Factor, Amp_Hours,
 * Valet_Mode_Speed, U2C_Enable/Setpoint, LV_Indicator Threshold/Hysteresis,
 * User_EEPROM_1-3.
 */
static const uint16_t inv_tune_addrs[] = {
    174, /* Serial_Number */
    241, /* PWM_Nominal_Freq (kHz) */
    246, /* PWM_Minimum_Variable_Freq (kHz) */
    247, /* PWM_Maximum_Variable_Freq (kHz) */
    245, /* PWM_Stall_Freq (kHz) */
    250, /* PWM_Var_Freq_Mode */
    248, /* PWM_Var_Dwell_Rate (3 ms) */
    150, /* Motor_Type */
    106, /* Veh_Flux (Wb) x1000 */
    152, /* Gamma_Adjust (deg) x10, signed */
    142, /* Run_Mode */
    143, /* Inv_Cmd_Mode */
    149, /* Key_Switch_Mode */
    140, /* Precharge_Bypassed */
    170, /* Relay_Output_State */
    173, /* Discharge_Enable */
    141, /* CAN_ID_Offset */
    144, /* CAN_Extended_Msg_ID */
    171, /* CAN_J1939_Option_Active */
    177, /* CAN_OBD2_Enable */
    178, /* CAN_BMS_Limit_Enable */
    146, /* CAN_Command_Message_Active */
    147, /* CAN_Bit_Rate (kbps) */
    148, /* CAN_ACTIVE_MSGS Lo_Word */
    237, /* CAN_ACTIVE_MSGS Hi_Word */
    235, /* CAN_Fast_Msg_Rate (ms) */
    236, /* CAN_Slow_Msg_Rate (ms) */
    158, /* CAN_Diag_Data_Tx_Active */
    159, /* CAN_Inv_Enab_Switch_Active */
    172, /* CAN_TimeOut (3 ms) */
    233, /* CAN_Slave_Cmd_ID */
    234, /* CAN_Slave_Dir */
    238, /* CAN_Debounce_Counter_Max */
    239, /* CAN_Debounce_Up_Count */
    240, /* CAN_Debounce_Down_Count */
    100, /* IQ_Limit (A) x10 */
    101, /* ID_Limit (A) x10 */
    102, /* DC_Volt_Limit (V) x10 */
    103, /* DC_Volt_Hyst (V) x10 */
    104, /* DC_UnderVolt_Thresh (V) x10 */
    112, /* Inv_OverTemp_Limit (C) x10 */
    113, /* Mtr_OverTemp_Limit (C) x10 */
    114, /* Zero_Torque_Temp (C) x10 */
    115, /* Full_Torque_Temp (C) x10 */
    203, /* RTD_Selection */
    132, /* Accel_Pedal_Flipped */
    120, /* Pedal_Lo (V) x100 */
    121, /* Accel_Min (V) x100 */
    122, /* Coast_Lo (V) x100 */
    123, /* Coast_Hi (V) x100 */
    124, /* Accel_Max (V) x100 */
    125, /* Pedal_Hi (V) x100 */
    129, /* Motor_Torque_Limit (Nm) x10 */
    130, /* Regen_Torque_Limit (Nm) x10 */
    131, /* Braking_Torque_Limit (Nm) x10 */
    164, /* Kp_Torque x10000 */
    165, /* w1_Torque (rad/s), Ki slot */
    166, /* Kd_Torque x100 */
    167, /* Klp_Torque x10000 */
    168, /* Torque_Rate_Limit (Nm) x10 */
    111, /* Motor_Overspeed (RPM) */
    128, /* Max_Speed (RPM) */
    126, /* Regen_Fade_Speed (RPM) */
    127, /* Break_Speed (RPM) */
    160, /* Kp_Speed x100 */
    161, /* w1_Speed (rad/s), Ki slot */
    162, /* Kd_Speed x100 */
    163, /* Klp_Speed x10000 */
    169, /* Speed_Rate_Limit (RPM/s) */
    187, /* Shudder_Compensation_Enable */
    188, /* Kp_Shudder x100 */
    189, /* TCLAMP_Shudder (Nm) x10 */
    190, /* Shudder_Filter_Freq (Hz) x10 */
    191, /* Shudder_Speed_Fade (RPM) */
    192, /* Shudder_Speed_Lo (RPM) */
    193, /* Shudder_Speed_Hi (RPM) */
    180, /* Brake_Mode */
    199, /* Brake_Switch_Bypassed */
    181, /* Brake_Lo (V) x100 */
    182, /* Brake_Min (V) x100 */
    183, /* Brake_Max (V) x100 */
    184, /* Brake_Hi (V) x100 */
    185, /* Regen_Ramp_Rate (s) x1000 */
    186, /* Brake_Pedal_Flipped */
    251, /* Self_Sense_Assist_Enabled */
    256, /* Brake_Thresh_Lo (V) x100 */
    257, /* Brake_Thresh_Hi (V) x100 */
    22,  /* Analog_Input_Pull_Up_Control, live value, general parameter */
    286, /* DCLink_FW_Reg_Use */
    283, /* Active_Short_Circuit_Enabled */
    284, /* Active_Short_Circuit_Delay (us) */
    285, /* Active_Short_Circuit_User_Off */
};

#endif /* INV_TUNE_PARAMS_H */
