#ifndef ADXL343_H__
#define ADXL343_H__

#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include <sdk_errors.h>
#include <stdlib.h>


/* REGISTERS */
#define ADXL343_REG_DEVID		(0x00)	// Device ID 
#define ADXL343_REG_THRESH_TAP		(0x1D)	// Tap threshold 
#define ADXL343_REG_OFSX		(0x1E)	// X-axis offset 
#define ADXL343_REG_OFSY		(0x1F)	// Y-axis offset 
#define ADXL343_REG_OFSZ		(0x20)	// Z-axis offset 
#define ADXL343_REG_DUR			(0x21)	// Tap duration 
#define ADXL343_REG_LATENT		(0x22)	// Tap latency 
#define ADXL343_REG_WINDOW		(0x23)	// Tap window 
#define ADXL343_REG_THRESH_ACT		(0x24)	// Activity threshold 
#define ADXL343_REG_THRESH_INACT	(0x25)	// Inactivity threshold 
#define ADXL343_REG_TIME_INACT		(0x26)	// Inactivity time 
#define ADXL343_REG_ACT_INACT_CTL	(0x27)	// Axis enable control for activity and inactivity detection
#define ADXL343_REG_THRESH_FF		(0x28)	// Free-fall threshold 
#define ADXL343_REG_TIME_FF		(0x29)	// Free-fall time 
#define ADXL343_REG_TAP_AXES		(0x2A)	// Axis control for single/double tap 
#define ADXL343_REG_ACT_TAP_STATUS	(0x2B)	// Source for single/double tap 
#define ADXL343_REG_BW_RATE		(0x2C)	// Data rate and power mode control 
#define ADXL343_REG_POWER_CTL		(0x2D)	// Power-saving features control 
#define ADXL343_REG_INT_ENABLE		(0x2E)	// Interrupt enable control 
#define ADXL343_REG_INT_MAP		(0x2F)	// Interrupt mapping control 
#define ADXL343_REG_INT_SOURCE		(0x30)	// Source of interrupts 
#define ADXL343_REG_DATA_FORMAT		(0x31)	// Data format control 
#define ADXL343_REG_DATAX0		(0x32)	// X-axis data 0 
#define ADXL343_REG_DATAX1		(0x33)	// X-axis data 1 
#define ADXL343_REG_DATAY0		(0x34)	// Y-axis data 0 
#define ADXL343_REG_DATAY1		(0x35)	// Y-axis data 1 
#define ADXL343_REG_DATAZ0		(0x36)	// Z-axis data 0 
#define ADXL343_REG_DATAZ1		(0x37)	// Z-axis data 1 
#define ADXL343_REG_FIFO_CTL		(0x38)	// FIFO control 
#define ADXL343_REG_FIFO_STATUS 	(0x39)	// FIFO status 


#define ADXL343_BIT_LOW_POWER   (0b10000)     // Low power bit, band width

#define ADXL343_BIT_LINK        (0b100000)    // Link bit, power control
#define ADXL343_BIT_AUTOSLEEP   (0b10000)     // Auto sleep bit, power control
#define ADXL343_BIT_MEASURE     (0b1000)      // measure active bit, power control
#define ADXL343_BIT_SLEEP       (0b100)       // Sleep bit, power control

#define ADXL343_BIT_DATA_READY_EN  (0b10000000)  // data ready bit, Interrupt enable control
#define ADXL343_BIT_SINGLE_TAP_EN  (0b1000000)   // single tab bit, Interrupt enable control
#define ADXL343_BIT_DOUBLE_TAP_EN  (0b100000)    // double tab bit, Interrupt enable control
#define ADXL343_BIT_ACTIVITY_EN    (0b10000)     // activity bit, Interrupt enable control
#define ADXL343_BIT_INACTIVITY_EN  (0b1000)      // inactivity bit, Interrupt enable control
#define ADXL343_BIT_FREE_FALL_EN   (0b100)       // free fall bit, Interrupt enable control
#define ADXL343_BIT_WATERMARK_EN   (0b10)        // watermark bit, Interrupt enable control
#define ADXL343_BIT_OVERRUN_EN     (0b1)         // overrun bit, Interrupt enable control

#define ADXL343_BIT_DATA_READY_MAP  (0b10000000)  // data ready bit, Interrupt mapping
#define ADXL343_BIT_SINGLE_TAP_MAP  (0b1000000)   // single tab bit, Interrupt mapping
#define ADXL343_BIT_DOUBLE_TAP_MAP  (0b100000)    // double tab bit, Interrupt mapping
#define ADXL343_BIT_ACTIVITY_MAP    (0b10000)     // activity bit, Interrupt mapping
#define ADXL343_BIT_INACTIVITY_MAP  (0b1000)      // inactivity bit, Interrupt mapping
#define ADXL343_BIT_FREE_FALL_MAP   (0b100)       // free fall bit, Interrupt mapping
#define ADXL343_BIT_WATERMARK_MAP   (0b10)        // watermark bit, Interrupt mapping
#define ADXL343_BIT_OVERRUN_MAP     (0b1)         // overrun bit, Interrupt mapping

#define ADXL343_BIT_SELF_TEST   (0b10000000)  // self test bit , Data format
#define ADXL343_BIT_SPI         (0b1000000)   // SPI bit, Data format
#define ADXL343_BIT_INT_INVERT  (0b100000)    // invert interrupts bit, Data format
#define ADXL343_BIT_FULL_RES    (0b1000)      // full resolution bit, Data format
#define ADXL343_BIT_JUSTIFY     (0b100)       // justify bit (MSB), Data format


#define ADXL343_MG2G_MULTIPLIER 	(0.004) // 4mg per lsb

#define ADXL343_DEVID_ID 		(0b11100101) // Device ID

#define SPI_READ         (0b10000000)  // 0b1000 0000
#define SPI_READ_MULTI   (0b11000000)  // 0b1100 0000
#define SPI_WRITE        (0x0)         // 0b0000 0000
#define SPI_WRITE_MULTI  (0b01000000) // 0b0000 0000

#define N_BYTES_FOR_XYZ_READ  (7) // start address + 3 * 2 byte = 7
#define N_BYTES_FOR_ONE_READ  (3) // start address + 1 * 2 byte = 3


/** Used with register 0x2C (ADXL343_REG_BW_RATE) to set bandwidth */
typedef enum {
  ADXL343_DATARATE_3200_HZ = 0b1111,  // 3200Hz Bandwidth
  ADXL343_DATARATE_1600_HZ = 0b1110,  // 1600Hz Bandwidth
  ADXL343_DATARATE_800_HZ  = 0b1101,  //  800Hz Bandwidth
  ADXL343_DATARATE_400_HZ  = 0b1100,  //  400Hz Bandwidth
  ADXL343_DATARATE_200_HZ  = 0b1011,  //  200Hz Bandwidth
  ADXL343_DATARATE_100_HZ  = 0b1010,  //  100Hz Bandwidth
  ADXL343_DATARATE_50_HZ   = 0b1001,  //   50Hz Bandwidth
  ADXL343_DATARATE_25_HZ   = 0b1000,  //   25Hz Bandwidth
  ADXL343_DATARATE_12_5_HZ = 0b0111,  // 12.5Hz Bandwidth
  ADXL343_DATARATE_6_25HZ  = 0b0110,  // 6.25Hz Bandwidth
  ADXL343_DATARATE_3_13_HZ = 0b0101,  // 3.13Hz Bandwidth
  ADXL343_DATARATE_1_56_HZ = 0b0100,  // 1.56Hz Bandwidth
  ADXL343_DATARATE_0_78_HZ = 0b0011,  // 0.78Hz Bandwidth
  ADXL343_DATARATE_0_39_HZ = 0b0010,  // 0.39Hz Bandwidth
  ADXL343_DATARATE_0_20_HZ = 0b0001,  // 0.20Hz Bandwidth
  ADXL343_DATARATE_0_10_HZ = 0b0000   // 0.10Hz Bandwidth (default value)
} dataRate_t;

/** Used with register 0x2D (ADXL343_REG_POWER_CTL) to set readings frequency in sleep mode */
typedef enum {
  ADXL343_WAKEUP_8_HZ  = 0b00, // 8Hz 
  ADXL343_WAKEUP_4_HZ  = 0b01, // 4Hz
  ADXL343_WAKEUP_2_HZ  = 0b10, // 2Hz (default)
  ADXL343_WAKEUP_1_HZ  = 0b11  // 1Hz
} wakeup_t;

/** Used with register 0x31 (ADXL343_REG_DATA_FORMAT) to set g range */
typedef enum {
  ADXL343_RANGE_16_G = 0b11, // +/- 16g 
  ADXL343_RANGE_8_G  = 0b10, // +/-  8g 
  ADXL343_RANGE_4_G  = 0b01, // +/-  4g 
  ADXL343_RANGE_2_G  = 0b00  // +/-  2g (default value) 
} range_t;

/** Possible interrupts sources on the ADXL343. */
union int_config {
  uint8_t value; // Composite 8-bit value of the bitfield.
  struct {
    uint8_t overrun    : 1; // Bit 0
    uint8_t watermark  : 1; // Bit 1
    uint8_t freefall   : 1; // Bit 2
    uint8_t inactivity : 1; // Bit 3
    uint8_t activity   : 1; // Bit 4
    uint8_t double_tap : 1; // Bit 5
    uint8_t single_tap : 1; // Bit 6
    uint8_t data_ready : 1; // Bit 7
  } bits;                   // Individual bits in the bitfield.
};

/** Possible interrupt pin outputs on the ADXL343. */
typedef enum { ADXL343_INT1 = 0, ADXL343_INT2 = 1 } int_pin;

typedef enum {
  all_axis,
  x_axis,
  y_axis,
  z_axis,
} e_acc_axis;


uint8_t ADXL343_init(nrf_drv_spi_t const * const p_instance);
void ADXL343_getXYZValues(int16_t *data);

#endif // ADXL343_H__