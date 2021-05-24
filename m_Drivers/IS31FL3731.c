#include "IS31FL3731.h"

#include "numbers.h"

const nrf_drv_twi_t *Twi = 0;
static uint8_t LED_PWM_value = DEFAULT_PWM_VAULE;


/**@brief Function to select a register page
 *
 * @details This function can be used to select one of the 8 frame register pages
 *          or to select the function register page. See data sheet page 9
 *
 * @param[in]   address   display driver IC address
 * @param[in]   page      register page
 */
void IS31FL3731_select_page(uint8_t address, uint8_t page) 
{
  uint8_t tempData[] = { IS31FL3731_COMMANDREGISTER,
                         page};

  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(Twi, address, tempData, sizeof(tempData), false);
  APP_ERROR_CHECK(err_code);  
}

/**@brief Function to write data to a single register
 *
 * @details This function can be used to write data in a specific register on a given page
 *
 * @param[in]   address   display driver IC address
 * @param[in]   page      register page
 * @param[in]   reg       register to write
 * @param[in]   data      data to write
 */
void IS31FL3731_write_register(uint8_t address, uint8_t page, uint8_t reg, uint8_t data) 
{
  IS31FL3731_select_page(address, page);

  uint8_t tempData[] = { reg,
                         data};
  ret_code_t err_code;
  err_code = nrf_drv_twi_tx(Twi, address, tempData, sizeof(tempData), false);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function to select a frame
 *
 * @details This function can be used to select a frame, which is showed in picture mode
 *
 * @param[in]   address   display driver IC address
 * @param[in]   frame     frame to select (0-7)
 */
void IS31FL3731_selct_picture_display_frame(uint8_t address, uint8_t frame) 
{
  if (frame > 7) 
    frame = 0;

  IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_PICTUREFRAME, frame);
}

/**@brief Function to set the PWM value for a frame
 *
 * @details This function can be used to set the PWM value / brightness for all 
 *          LEDs in the given frame
 *
 * @param[in]   address   display driver IC address
 * @param[in]   frame     frame (0-7)
 * @param[in]   pwm       PWM value for all LEDs (0-255)
 */
void IS31FL3731_set_frame_PWM(uint8_t address, uint8_t frame, uint8_t pwm)
{
  ret_code_t err_code;
  static const uint8_t numberOfPWMRegs = IS31FL3731_FRAME_REG_PWM_H-IS31FL3731_FRAME_REG_PWM_L;

  uint8_t pwmtempData[numberOfPWMRegs+1];
  IS31FL3731_select_page(address, frame);

  pwmtempData[0] = IS31FL3731_FRAME_REG_PWM_L;

  for(uint8_t i = 1; i < numberOfPWMRegs+1; i++)
  {
    pwmtempData[i] = pwm;
  }

  err_code = nrf_drv_twi_tx(Twi, address, pwmtempData, numberOfPWMRegs+1, false);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function to set the brightness of all LEDs an IC
 *
 * @details This function can be used to set the brightness off all LEDs and ICs,
 *          but not write it to the ICs
 *
 * @param[in]   brightness  PWM value for the LEDs (0-255)
 */
void IS31FL3731_set_global_brightness(uint8_t brightness)
{
  LED_PWM_value = brightness;
}

/**@brief Function to write the global brightness to all LEDs of a IC
 *
 * @details This function can be used to set the brightness off frames of all LEDs in a IC
 *
 * @param[in]   address     display driver IC address
 */
void IS31FL3731_write_global_brightness(uint8_t address)
{
  for(uint8_t i = 0; i < 8; i++)
    IS31FL3731_set_frame_PWM(address, i, LED_PWM_value);
}

/**@brief Function to get the brightness of all LEDs an IC
 *
 * @details This function can be used to get the global brightness off all LEDs and ICs.
 *
 * @param[out]   brightness  PWM value for the LEDs (0-255)
 */
uint8_t IS31FL3731_get_global_brightness()
{
  return LED_PWM_value;
}

/**@brief Function to clear a frame
 *
 * @details This function can be used to turn off all LEDs in a given frame and set 
 *          the PWM value / brightness for all LEDs in the given frame to the default value
 *
 * @param[in]   address   display driver IC address
 * @param[in]   frame     frame (0-7)
 */
void IS31FL3731_clear_frame(uint8_t address, uint8_t frame) 
{
  IS31FL3731_select_page(address, frame);

  static const uint8_t numberOfOnOffRegs = IS31FL3731_FRAME_REG_ONOFF_H-IS31FL3731_FRAME_REG_ONOFF_L;
  uint8_t onOffTempData[numberOfOnOffRegs+1];
  ret_code_t err_code;

  onOffTempData[0] = IS31FL3731_FRAME_REG_ONOFF_L;

  for(uint8_t i = 1; i < numberOfOnOffRegs+1; i++)
  {
    onOffTempData[i] = 0xff;
  }

  err_code = nrf_drv_twi_tx(Twi, address, onOffTempData, numberOfOnOffRegs+1, false);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function to set audio sync mode
 *
 * @details This function can be used to turn on or off the audio sync feature
 *
 * @param[in]   address   display driver IC address
 * @param[in]   sync      turn on(true) or off(false)
 */
void IS31FL3731_set_audioSync(uint8_t address, bool sync) 
{
  if (sync) 
    IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_AUDIOSYNC, 1);
  else 
    IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_AUDIOSYNC, 0);
}

/**@brief Function to load the standard number (1-6)
 *
 * @details This function can be used to load frame 1-6 with the standard dice symbols (1-6).
 *          The offset parameter can be used to change the start symbol.
 *
 * @param[in]   address   display driver IC address
 * @param[in]   offset    offset to shift first symbol 
 */
void IS31FL3731_load_std_numbers(uint8_t address, uint8_t offset)
{
  static const uint8_t numberOfOnOffRegs = IS31FL3731_FRAME_REG_ONOFF_H-IS31FL3731_FRAME_REG_ONOFF_L;
  uint8_t onOffTempData[numberOfOnOffRegs+1];
  ret_code_t err_code;

  onOffTempData[0] = IS31FL3731_FRAME_REG_ONOFF_L;

  for(uint8_t j = 0; j < 6; j++)
  {
    uint8_t first_symbol  = (j+offset)%6+1;
    uint8_t second_symbol;
    if(first_symbol <= 5)
      second_symbol = first_symbol+1;
    else
      second_symbol = 1;
  
    IS31FL3731_select_page(address, j);

    for(uint8_t i = 1; i < 9; i++)
    {
      onOffTempData[i*2-1] = Numbers_symbols[first_symbol][i-1];
      onOffTempData[i*2] = Numbers_symbols[second_symbol][i-1];
    }

    err_code = nrf_drv_twi_tx(Twi, address, onOffTempData, numberOfOnOffRegs+1, false);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function to activate software shutdown
 *
 * @details This function can be used to activate the software shutdown. All LEDs off
 *          but the registers are still read and writable
 *
 * @param[in]   address   display driver IC address
 * @param[in]   shutdown  shutdown yes or no
 */
void IS31FL3731_sw_shutdown(uint8_t address, bool shutdown)
{
  if(shutdown)
    IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_SHUTDOWN, IS31FL3731_REG_SHUTDOWN_SWSHUTDOWN);
  else
    IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_SHUTDOWN, IS31FL3731_REG_SHUTDOWN_NORMALMODE);
}

static void IS31FL3731_write_8x8_frame(uint8_t address, uint8_t frame,const uint8_t * data)
{
  uint8_t onOffTempData[2*8+1];
  ret_code_t err_code;

  onOffTempData[0] = IS31FL3731_FRAME_REG_ONOFF_L;

  IS31FL3731_select_page(address, frame);

  for(uint8_t i = 1; i < 9; i++)
  {
    onOffTempData[i*2-1] = data[i-1];
    onOffTempData[i*2] = data[i-1];
  }

  err_code = nrf_drv_twi_tx(Twi, address, onOffTempData, 2*8+1, false);
  APP_ERROR_CHECK(err_code);
}

void IS31FL3731_show_number(uint8_t address, uint8_t number, uint8_t type)
{
  if(type == 0 && number >= 1 && number <= 6)
    IS31FL3731_write_8x8_frame(address, IS31FL3731_FRAME7, Numbers_symbols[number]);
  else
    IS31FL3731_write_8x8_frame(address, IS31FL3731_FRAME7, Numbers_symbols[1]);

  // select frame to display 
  IS31FL3731_selct_picture_display_frame(address, IS31FL3731_FRAME7);

  // picture mode
  IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_CONFIG, IS31FL3731_REG_CONFIG_PICTUREMODE);
}

void IS31FL3731_start_autoplay(uint8_t address)
{
  // auto play mode
  IS31FL3731_write_register(address, 
                            IS31FL3731_PAGE_FUNCTIONREG, 
                            IS31FL3731_REG_CONFIG, 
                            IS31FL3731_REG_CONFIG_AUTOPLAYMODE
                            |IS31FL3731_FRAME1);
}

/**@brief Function to initialize the IS31FL3731
 *
 * @details This function have to be called first, to initialize the IS31FL3731
 *
 * @param[in]   twi       pointer to the twi instance
 * @param[in]   address   display driver IC address
 */
void IS31FL3731_init(const nrf_drv_twi_t * const twi, uint8_t address)
{
  Twi = twi;
  volatile uint8_t test = 0;

  // shutdown
  IS31FL3731_sw_shutdown(address, 1);

  nrf_delay_ms(10);

  // picture mode
//  IS31FL3731_write_register(address, IS31FL3731_PAGE_FUNCTIONREG, IS31FL3731_REG_CONFIG, IS31FL3731_REG_CONFIG_PICTUREMODE);

  // select frame to display 
//  IS31FL3731_selct_picture_display_frame(address, IS31FL3731_FRAME7);

  // set the PWM value for all LEDs
  IS31FL3731_write_global_brightness(address);

  // all LEDs off & PWM = X
  IS31FL3731_clear_frame(address, IS31FL3731_FRAME7);
  
  // turn off audio sync
  IS31FL3731_set_audioSync(address, false);

  // load number 1-6 in frame 0-5
  IS31FL3731_load_std_numbers(address,0);

//  // auto play mode
//  IS31FL3731_write_register(address, 
//                            IS31FL3731_PAGE_FUNCTIONREG, 
//                            IS31FL3731_REG_CONFIG, 
//                            IS31FL3731_REG_CONFIG_AUTOPLAYMODE
//                            |IS31FL3731_FRAME1);
  // Play 6 Frames endless
  IS31FL3731_write_register(address, 
                            IS31FL3731_PAGE_FUNCTIONREG, 
                            IS31FL3731_REG_AUTOPLAYCONTROL1, 
                            IS31FL3731_REG_AUTOPLAYCONTROL1_FNS6FRAMES);
  // Set Frame delay 
  IS31FL3731_write_register(address, 
                            IS31FL3731_PAGE_FUNCTIONREG, 
                            IS31FL3731_REG_AUTOPLAYCONTROL2, 
                            1);  // 1 * 11ms = 11ms
  // Use intensity setting of frame 1 for all other frames
  IS31FL3731_write_register(address, 
                            IS31FL3731_PAGE_FUNCTIONREG, 
                            IS31FL3731_REG_DISPLAYOPTION, 
                            IS31FL3731_REG_DISPLAYOPTION_FRAME1INTENSITY);
  // Set fade in and fade our time
  IS31FL3731_write_register(address, 
                            IS31FL3731_PAGE_FUNCTIONREG, 
                            IS31FL3731_REG_BREATHCONTROL1, 
                            (2 << IS31FL3731_REG_BREATHCONTROL1_FOT_SHIFT)  // 2^X * 26ms -> 2^2 * 26ms = 104ms
                            | 2);                                           // 2^X * 26ms -> 2^2 * 26ms = 104ms
  // Breath/Fade mode enable
  IS31FL3731_write_register(address, 
                            IS31FL3731_PAGE_FUNCTIONREG, 
                            IS31FL3731_REG_BREATHCONTROL2, 
                            IS31FL3731_REG_BREATHCONTROL2_BREATHEN);
  // auto play mode
  IS31FL3731_start_autoplay(address);

  // out of software shutdown
//  IS31FL3731_sw_shutdown(address, 0);
}

