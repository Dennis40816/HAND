/*
 * Copyright (c) 2024 Dennis Liu, dennis48161025@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdint.h>
#include <stdbool.h>

/* bos1901 register address */
typedef enum bos1901_register_t
{
  /* REFERENCE (FIFO) */
  BOS1901_REG_REFERENCE = 0x00,
  BOS1901_REG_FIFO = 0x00,

  /* ION_BL */
  BOS1901_REG_ION_BL = 0x01,

  /* DEADTIME (ID) */
  BOS1901_REG_DEADTIME = 0x02,
  BOS1901_REG_ID = 0x02,

  /* KP */
  BOS1901_REG_KP = 0x03,

  /* KPA_KI */
  BOS1901_REG_KPA_KI = 0x04,

  /* CONFIG */
  BOS1901_REG_CONFIG = 0x05,

  /* PARCAP */
  BOS1901_REG_PARCAP = 0x06,

  /* SUP_RISE */
  BOS1901_REG_SUP_RISE = 0x07,

  /* DAC */
  BOS1901_REG_DAC = 0x08,

  /* INVALID from 0x09 to 0x0B */

  /* IC_STATUS (Read only) */
  BOS1901_REG_IC_STATUS = 0x0C,

  /* SENSE (Read only) */
  BOS1901_REG_SENSE = 0x0D,

  /* TRIM */
  BOS1901_REG_TRIM = 0x0E,

  /* END OF MAIN REGISTER */
  BOS1901_REG_END_OF_MAIN = 0x0F,

  /* START of SUB REGISTER (can be only access by config) */
  BOS1901_SUB_REG_START = 0x0F,

  /* DISABLE SDO (always return 0x0000) */
  BOS1901_SUB_REG_DISABLE_SDO = 0x10,

  /* PERIOD */
  BOS1901_SUB_REG_PHASE = 0x12,

  /* H-bridge, STM-STATE, OUTPUT_VOLT_ERR */
  BOS1901_SUB_REG_OUTPUT_ERR = 0x13,

  /* PI Current status (based on Rsense = 0.5 Ohm) */
  BOS1901_SUB_REG_CURRENT_PI = 0x14,

  /* DESIRED OUTPUT AMP */
  BOS1901_SUB_REG_OUTPUT_AMP = 0x1A,

  /* END OF SUB REG */
  BOS1901_SUB_REG_END = 0x1B,
} bos1901_register_t;

/* (CONFIG), bos1901 config related enum */
typedef enum bos1901_play_sample_rate_t
{
  BOS1901_PLAY_1024_KSPS,
  BOS1901_PLAY_512_KSPS,
  BOS1901_PLAY_256_KSPS,
  BOS1901_PLAY_128_KSPS,
  BOS1901_PLAY_64_KSPS,
  BOS1901_PLAY_32_KSPS,
  BOS1901_PLAY_16_KSPS,
  BOS1901_PLAY_8_KSPS,
  BOS1901_PLAY_END,
  BOS1901_PLAY_DEFAULT = BOS1901_PLAY_1024_KSPS,
} bos1901_play_sample_rate_t;

/* (CONFIG) */
typedef enum bos1901_ds_mode_t
{
  BOS1901_DS_IDLE_MODE,
  BOS1901_DS_SLEEP_MODE,
  BOS1901_DS_END,
  BOS1901_DS_DEFAULT = BOS1901_DS_IDLE_MODE,
} bos1901_ds_mode_t;

/* (CONFIG) */
typedef enum bos1901_oe_status_t
{
  BOS1901_OE_DISABLE,
  BOS1901_OE_ENABLE,
  BOS1901_OE_END,
  BOS1901_OE_DEFAULT = BOS1901_OE_DISABLE,
} bos1901_oe_status_t;

/* (CONFIG) */
typedef enum bos1901_rst_t
{
  BOS1901_RST_NORMAL,
  BOS1901_RST_RESET,
  BOS1901_RST_END,
  BOS1901_RST_DEFAULT = BOS1901_RST_NORMAL,
} bos1901_rst_t;

/* (CONFIG), write protect during OE is enable */
typedef enum bos1901_lock_t
{
  BOS1901_LOCK_DISABLE,
  BOS1901_LOCK_ENABLE,
  BOS1901_LOCK_END,
  BOS1901_LOCK_DEFAULT = BOS1901_LOCK_DISABLE,
} bos1901_lock_t;

/* (KP), square wave or continuous wave */
typedef enum bos1901_sq_t
{
  BOS1901_SQ_CONTINUOUS_WAVE,
  BOS1901_SQ_SQUARE_WAVE,
  BOS1901_SQ_END,
  BOS1901_SQ_DEFAULT = BOS1901_SQ_CONTINUOUS_WAVE,
} bos1901_sq_t;

/* UPI (PARCAP), must set enable when the power delivery net can't consume
 * backforward power */
typedef enum bos1901_upi_t
{
  BOS1901_UPI_DISABLE,
  BOS1901_UPI_ENABLE,
  BOS1901_UPI_END,
  BOS1901_UPI_DEFAULT = BOS1901_UPI_DISABLE,
} bos1901_upi_t;

/* LMI (PARCAP), set to OVERRIDE if Rsense < 0.32 Ohm */
typedef enum bos1901_lmi_t
{
  BOS1901_LMI_RSENSE,    ///< decided by Rsense value
  BOS1901_LMI_OVERRIDE,  ///< decided by BOS1901 default value (typical value:
                         ///< 815 mA)
  BOS1901_LMI_END,
  BOS1901_LMI_DEFAULT = BOS1901_LMI_OVERRIDE,
} bos1901_lmi_t;

/* CP5 (PARCAP), 5V internal pump */
typedef enum bos1901_cp5_t
{
  BOS1901_CP5_OFF,
  BOS1901_CP5_ON,
  BOS1901_CP5_END,
  BOS1901_CP5_DEFAULT = BOS1901_CP5_ON,
} bos1901_cp5_t;

/* CALIBRATION (PARCAP) */
typedef enum bos1901_cal_t
{
  BOS1901_CAL_DISABLE,  ///< not recommended
  BOS1901_CAL_ENABLE,
  BOS1901_CAL_END,
  BOS1901_CAL_DEFAULT = BOS1901_CAL_ENABLE,
} bos1901_cal_t;

/* SENSE (SUP_RISE) */
typedef enum bos1901_sense_t
{
  BOS1901_SENSE_DISABLE,
  BOS1901_SENSE_ENABLE,
  BOS1901_SENSE_END,
  BOS1901_SENSE_DEFAULT = BOS1901_SENSE_DISABLE,
} bos1901_sense_t;

/* STATE (IC_STATUS) */
typedef enum bos1901_state_t
{
  BOS1901_STATE_IDLE,
  BOS1901_STATE_CALIBRATION,
  BOS1901_STATE_RUN,
  BOS1901_STATE_ERR,
  BOS1901_STATE_END,
  BOS1901_STATE_DEFAULT = BOS1901_STATE_IDLE,
} bos1901_state_t;

/* OVV (IC_STATUS) */
typedef enum bos1901_ovv_t
{
  BOS1901_OVV_OK,
  BOS1901_OVV_OV,  ///< error, over voltage
  BOS1901_OVV_END,
  BOS1901_OVV_DEFAULT = BOS1901_OVV_OK,
} bos1901_ovv_t;

/* OVT (IC_STATUS) */
typedef enum bos1901_ovt_t
{
  BOS1901_OVT_OK,
  BOS1901_OVT_OT,  ///< error, ic over heated
  BOS1901_OVT_END,
  BOS1901_OVT_DEFAULT = BOS1901_OVT_OK,
} bos1901_ovt_t;

/* FULL (IC_STATUS) */
typedef enum bos1901_fifo_full_t
{
  BOS1901_FIFO_FULL_NOT_FULL,
  BOS1901_FIFO_FULL,
  BOS1901_FIFO_FULL_END,
  BOS1901_FIFO_FULL_DEFAULT = BOS1901_FIFO_FULL_NOT_FULL,
} bos1901_fifo_full_t;

/* EMPTY (IC_STATUS) */
typedef enum bos1901_fifo_empty_t
{
  BOS1901_FIFO_EMPTY_NOT_EMPTY,
  BOS1901_FIFO_EMPTY_EMPTY,
  BOS1901_FIFO_EMPTY_END,
  BOS1901_FIFO_EMPTY_DEFAULT = BOS1901_FIFO_EMPTY_EMPTY,
} bos1901_fifo_empty_t;

/* TODO: v2 */
typedef enum bos1901_trimrw_t
{
  bos1901_trimrw_place_holder
} bos1901_trimrw_t;

/* TODO: v2 */
typedef enum bos1901_sdobp_t
{
  bos1901_sdobp_place_holder
} bos1901_sdobp_t;

/* sub register enum start, there's no default value for sub register */

typedef enum bos1901_h_phase_t
{
  BOS1901_H_PHASE_POSITIVE,
  BOS1901_H_PHASE_NEGATIVE,
  BOS1901_H_PHASE_END,
} bos1901_h_phase_t;

typedef enum bos1901_polarity_t
{
  BOS1901_POLARITY_POSITIVE,
  BOS1901_POLARITY_NEGATIVE,
  BOS1901_POLARITY_END
} bos1901_polarity_t;

typedef enum bos1901_stm_state_t
{
  BOS1901_STM_STATE_IDLE,
  BOS1901_STM_STATE_CALIBRATE,
  BOS1901_STM_STATE_RUN,
  BOS1901_STM_STATE_OVERVOLT,
  BOS1901_STM_STATE_END,
} bos1901_stm_state_t;

typedef enum bos1901_data_transmit_direction_t
{
  BOS1901_DATA_TX,  // foramt tx for write reg
  BOS1901_DATA_RX,  // foramt tx for read reg
} bos1901_data_transmit_direction_t;

/**
 * @brief Device configuration structure for BOS1901.
 *
 * This structure holds platform-independent device configuration settings.
 */
typedef struct bos1901_dev_config_t
{
  // Platform-independent device configuration fields
  bos1901_upi_t upi;
  bos1901_sq_t sq;
  bos1901_cal_t cal;
  bos1901_lock_t write_protect_lock;
  bos1901_lmi_t lmi;
  bos1901_ds_mode_t ds;
  bos1901_cp5_t cp5;
  bos1901_play_sample_rate_t play_mode;

  /* TODO: v2 trim related */
  // bos1901_trimrw_t trimrw;
  // bos1901_sdobp_t sdobp;

} bos1901_dev_config_t;

/* TODO: v1 record the result of reading register */
typedef struct bos1901_dev_register_status_t
{
} bos1901_dev_register_status_t;

/* modified by user */
/**
 *
 * OE: bos1901_device_output_enable()
 * RST: bos1901_device_reset()
 * SENSE bos1901_device_sense_enable()
 *
 *
 *
 *
 */

/* user must not modify this struct */
typedef struct bos1901_private_status_t
{
  bool output_enable;  // oe
  bool sense_enable;   // sense
} bos1901_private_status_t;

/* TODO: v1, create something like

// Define struct for each register's bit fields

// Address 0x0: REFERENCE
typedef struct {
    unsigned int FIFO : 12;
    unsigned int : 4; // Reserved
} ReferenceRegister;

// Address 0x1: ION_BL
typedef struct {
    unsigned int FSWMAX : 2;
    unsigned int SB : 2;
    unsigned int I_ON_SCALE : 8;
    unsigned int : 4; // Reserved
} IonBlRegister;

// Address 0x2: DEADTIME
typedef struct {
    unsigned int DHS : 7;
    unsigned int : 9; // Reserved
} DeadtimeRegister;

// Address 0x3: KP
typedef struct {
    unsigned int KP : 11;
    unsigned int SQ : 1;
    unsigned int : 4; // Reserved
} KpRegister;

// Address 0x4: KPA_KI
typedef struct {
    unsigned int KPA : 8;
    unsigned int KIBASE : 4;
    unsigned int : 4; // Reserved
} KpaKiRegister;

// Address 0x5: CONFIG
typedef struct {
    unsigned int BC : 5;
    unsigned int LOCK : 1;
    unsigned int RST : 1;
    unsigned int OE : 1;
    unsigned int DS : 1;
    unsigned int PLAY : 3;
    unsigned int : 4; // Reserved
} ConfigRegister;

// Address 0x6: PARCAP
typedef struct {
    unsigned int UPI : 1;
    unsigned int LMI : 1;
    unsigned int CP5 : 1;
    unsigned int CAL : 1;
    unsigned int PARCAP : 4;
    unsigned int : 8; // Reserved
} ParcapRegister;

// Address 0x7: SUP_RISE
typedef struct {
    unsigned int TI_RISE : 6;
    unsigned int : 2; // Reserved
    unsigned int VDD : 5;
    unsigned int SENSE : 3;
} SupRiseRegister;

// Address 0x8: DAC
typedef struct {
    unsigned int DAC_HS : 6;
    unsigned int : 2; // Reserved
    unsigned int DAC_LS : 6;
    unsigned int : 2; // Reserved
} DacRegister;

// Address 0xC: IC_STATUS
typedef struct {
    unsigned int FIFO_SPACE : 6;
    unsigned int EMPTY : 1;
    unsigned int FULL : 1;
    unsigned int OVT : 1;
    unsigned int OVV : 1;
    unsigned int STATE : 2;
    unsigned int : 4; // Reserved
} IcStatusRegister;

// Address 0xD: SENSE
typedef struct {
    unsigned int VFEEDBACK : 10;
    unsigned int STATE : 2;
    unsigned int : 4; // Reserved
} SenseRegister;

// Address 0xE: TRIM
typedef struct {
    unsigned int TRIM_REG : 3;
    unsigned int : 1; // Reserved
    unsigned int TRIM_OSC : 6;
    unsigned int SDOBP : 2;
    unsigned int TRIMRW : 2;
    unsigned int : 2; // Reserved
} TrimRegister;

// Define union for all registers
typedef union {
    uint16_t raw[15]; // Raw data for all registers
    struct {
        ReferenceRegister reference;   // 0x0
        IonBlRegister ion_bl;          // 0x1
        DeadtimeRegister deadtime;     // 0x2
        KpRegister kp;                 // 0x3
        KpaKiRegister kpa_ki;          // 0x4
        ConfigRegister config;         // 0x5
        ParcapRegister parcap;         // 0x6
        SupRiseRegister sup_rise;      // 0x7
        DacRegister dac;               // 0x8
        uint16_t reserved_0x9;         // 0x9 - Reserved
        uint16_t reserved_0xA;         // 0xA - Reserved
        uint16_t reserved_0xB;         // 0xB - Reserved
        IcStatusRegister ic_status;    // 0xC
        SenseRegister sense;           // 0xD
        TrimRegister trim;             // 0xE
    } fields;
} Bos1901Registers;

int main() {
    // Example usage
    Bos1901Registers registers = {0};

    // Set some fields
    registers.fields.reference.FIFO = 0xABC;
    registers.fields.ion_bl.FSWMAX = 3;
    registers.fields.ic_status.FULL = 1;

    // Print some values
    printf("FIFO: 0x%03X\n", registers.fields.reference.FIFO);
    printf("FSWMAX: %u\n", registers.fields.ion_bl.FSWMAX);
    printf("IC_STATUS FULL: %u\n", registers.fields.ic_status.FULL);

    return 0;
}


 */