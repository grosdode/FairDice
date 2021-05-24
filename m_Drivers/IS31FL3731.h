#ifndef IS31FL3731_H__
#define IS31FL3731_H__

#include <sdk_errors.h>
#include <stdbool.h>

#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#define IS31FL3731_ADDR_DEFAULT IS31FL3731_ADDR_AD_GND
#define IS31FL3731_ADDR_AD_GND  0b1110100
#define IS31FL3731_ADDR_AD_VCC  0b1110111
#define IS31FL3731_ADDR_AD_SCL  0b1110101
#define IS31FL3731_ADDR_AD_SDA  0b1110110

#define IS31FL3731_COMMANDREGISTER          0xFD  // Command Register to choose register page
#define IS31FL3731_PAGE_FUNCTIONREG         0x0B  // Point to page 9 (Function Registers)

#define IS31FL3731_FRAME_REG_ONOFF_L        0x00  // LED Control Registers store the on or off state of each LED
#define IS31FL3731_FRAME_REG_ONOFF_H        0x11
#define IS31FL3731_FRAME_REG_BLINK_L        0x12  // Blink Control Registers configure the blink function of each LED
#define IS31FL3731_FRAME_REG_BLINK_H        0x23
#define IS31FL3731_FRAME_REG_PWM_L          0x24  // PWM Registers modulate the 144 LEDs in 256 steps (formula page 10)
#define IS31FL3731_FRAME_REG_PWM_H          0xB3

#define IS31FL3731_REG_CONFIG               0x00  // Configuration Register		Configure the operation mode
#define IS31FL3731_REG_PICTUREFRAME         0x01  // Picture Display Register		Set the display frame in Picture Mode
#define IS31FL3731_REG_AUTOPLAYCONTROL1     0x02  // Auto Play Control Register 1	Set the way of display in Auto Frame Play Mode
#define IS31FL3731_REG_AUTOPLAYCONTROL2     0x03  // Auto Play Control Register 2	Set the delay time in Auto Frame Play Mode
#define IS31FL3731_REG_DISPLAYOPTION        0x05  // Display Option Register		Set the display option
#define IS31FL3731_REG_AUDIOSYNC            0x06  // Audio Synchronization Register	Set audio synchronization function
#define IS31FL3731_REG_FRAMESTATE           0x07  // Frame State Register		Store the frame display information
#define IS31FL3731_REG_BREATHCONTROL1       0x08  // Breath Control Register 1		Set fade in and fade out time for breath function
#define IS31FL3731_REG_BREATHCONTROL2       0x09  // Breath Control Register 2		Set the breath function
#define IS31FL3731_REG_SHUTDOWN             0x0A  // Shutdown Register			Set software shutdown mode
#define IS31FL3731_REG_AGCCONTROL           0x0B  // AGC Control Register		Set the AGC function and the audio gain.
#define IS31FL3731_REG_AUDIOADC             0x0C  // Audio ADC Rate Register		Set the ADC sample rate of the input signal


#define IS31FL3731_REG_CONFIG_PICTUREMODE   (0b00 << 3) // Picture Mode 
#define IS31FL3731_REG_CONFIG_AUTOPLAYMODE  (0b01 << 3) // Auto Frame Play Mode
#define IS31FL3731_REG_CONFIG_AUDIOPLAYMODE (0b10 << 3) // Audio Frame Play Mode

#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNSPLAYENDLESS	0x0        // Auto Play endless
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS1LOOP	(0x1 << 4) // Auto Play 1 loop
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS2LOOPS	(0x2 << 4) // Auto Play 2 loops
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS3LOOPS	(0x3 << 4) // Auto Play 3 loops
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS4LOOPS	(0x4 << 4) // Auto Play 4 loops
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS5LOOPS	(0x5 << 4) // Auto Play 5 loops
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS6LOOPS	(0x6 << 4) // Auto Play 6 loops
#define IS31FL3731_REG_AUTOPLAYCONTROL1_CNS7LOOPS	(0x7 << 4) // Auto Play 7 loops

#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNSALLFRAMES	0x0 // play all frames
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS1FRAME	0x1 // play 1 frame
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS2FRAMES	0x2 // play 2 frames
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS3FRAMES	0x3 // play 3 frames
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS4FRAMES	0x4 // play 4 frames
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS5FRAMES	0x5 // play 5 frames
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS6FRAMES	0x6 // play 6 frames
#define IS31FL3731_REG_AUTOPLAYCONTROL1_FNS7FRAMES	0x7 // play 7 frames

#define IS31FL3731_REG_DISPLAYOPTION_INDEPENDENTINTENSITY (0 << 5)  // Set the intensity of each frame independently
#define IS31FL3731_REG_DISPLAYOPTION_FRAME1INTENSITY      (1 << 5)  // Use intensity setting of frame 1 for all other frames
#define IS31FL3731_REG_DISPLAYOPTION_BLINKDIS             (0 << 4)  // Blink Disable
#define IS31FL3731_REG_DISPLAYOPTION_BLINKEN              (1 << 4)  // Blink Enable

#define IS31FL3731_REG_AUDIOSYNC_AEDIS  0 // Audio synchronization disable
#define IS31FL3731_REG_AUDIOSYNC_AEEN   1 // Enable audio signal to modulate the intensity of the matrix

#define IS31FL3731_REG_BREATHCONTROL1_FOT_SHIFT 4 // Shift value to shift the Fade Out time

#define IS31FL3731_REG_BREATHCONTROL2_BREATHDIS 0       // Breath Disable
#define IS31FL3731_REG_BREATHCONTROL2_BREATHEN (1 << 4) // Breath Enable

#define IS31FL3731_REG_SHUTDOWN_SWSHUTDOWN  0 // Software Shutdown Mode
#define IS31FL3731_REG_SHUTDOWN_NORMALMODE  1 // Normal Operation

#define IS31FL3731_REG_AGCCONTROL_SLOWMODE  0         // Audio gain control slow mode
#define IS31FL3731_REG_AGCCONTROL_FASTMODE  (1 << 4)  // Audio gain control fast mode
#define IS31FL3731_REG_AGCCONTROL_DIS       0         // Audio gain control disable
#define IS31FL3731_REG_AGCCONTROL_EN        (1 << 3)  // Audio gain control enable
#define IS31FL3731_REG_AGCCONTROL_AGS0DB    0b000     // Audio Gain selection 0dB
#define IS31FL3731_REG_AGCCONTROL_AGS3DB    0b001     // Audio Gain selection 3dB
#define IS31FL3731_REG_AGCCONTROL_AGS6DB    0b010     // Audio Gain selection 6dB
#define IS31FL3731_REG_AGCCONTROL_AGS9DB    0b011     // Audio Gain selection 9dB
#define IS31FL3731_REG_AGCCONTROL_AGS12DB   0b100     // Audio Gain selection 12dB
#define IS31FL3731_REG_AGCCONTROL_AGS15DB   0b101     // Audio Gain selection 15dB
#define IS31FL3731_REG_AGCCONTROL_AGS18DB   0b110     // Audio Gain selection 18dB
#define IS31FL3731_REG_AGCCONTROL_AGS21DB   0b111     // Audio Gain selection 21dB



// frame numbers (Command Register, Frame Start, Picture Display, Current Frame Display)
typedef enum {
  IS31FL3731_FRAME1  = 0x0, 
  IS31FL3731_FRAME2  = 0x1, 
  IS31FL3731_FRAME3  = 0x2, 
  IS31FL3731_FRAME4  = 0x3, 
  IS31FL3731_FRAME5  = 0x4, 
  IS31FL3731_FRAME6  = 0x5, 
  IS31FL3731_FRAME7  = 0x6, 
  IS31FL3731_FRAME8  = 0x7, 
} IS31FL3731_frame_t;



#define IS31FL3731_NUMBER_OF_LEDS 144

#define DEFAULT_PWM_VAULE         5


void IS31FL3731_select_page(uint8_t address, uint8_t page);
void IS31FL3731_write_register(uint8_t address, uint8_t page, uint8_t reg, uint8_t data);
void IS31FL3731_selct_picture_display_frame(uint8_t address, uint8_t frame);
void IS31FL3731_set_frame_PWM(uint8_t address, uint8_t frame, uint8_t pwm);
void IS31FL3731_set_global_brightness(uint8_t brightness);
void IS31FL3731_write_global_brightness(uint8_t address);
uint8_t IS31FL3731_get_global_brightness();
void IS31FL3731_clear_frame(uint8_t address, uint8_t frame);
void IS31FL3731_set_audioSync(uint8_t address, bool sync);
void IS31FL3731_load_std_numbers(uint8_t address, uint8_t offset);
void IS31FL3731_sw_shutdown(uint8_t address, bool shutdown);
static void IS31FL3731_write_8x8_frame(uint8_t address, uint8_t frame,const uint8_t * data);
void IS31FL3731_show_number(uint8_t address, uint8_t number, uint8_t type);
void IS31FL3731_start_autoplay(uint8_t address);
void IS31FL3731_init(const nrf_drv_twi_t * const twi, uint8_t address);
//void IS31FL3731_drawPixel(int16_t x, int16_t y, uint16_t color);
//void IS31FL3731_clear(void);
//
//void IS31FL3731_setLEDPWM(uint8_t lednum, uint8_t pwm, uint8_t bank);
//void IS31FL3731_audioSync(bool sync);
//void IS31FL3731_setFrame(uint8_t b);
//void IS31FL3731_displayFrame(uint8_t frame);
//
//
//void IS31FL3731_selectBank(uint8_t bank);
//void IS31FL3731_writeRegister8(uint8_t bank, uint8_t reg, uint8_t data);
//uint8_t IS31FL3731_readRegister8(uint8_t bank, uint8_t reg);

#endif // IS31FL3731_H__