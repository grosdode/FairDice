#include "ADXL343.h"

const nrf_drv_spi_t *Spi = 0;

/***************************************************************************//**
 * @brief
 * This function manage SPI communication with more than one read write request
 *
 * @param[in] send
 *  Pointer to the data to transmit
 *
 * @param[out] receive
 *  Pointer to received Data (optional)
 *
 * @param[in] length
 *  amount of data to read or write (send and receive have to be greater or
 *  equal to length to avoid memory errors)
 *
 * @return
 *  Nordic ret_code_t
 ******************************************************************************/
ret_code_t ADXL343_spiCommunicate(uint8_t *send, uint8_t *receive, uint8_t length) 
{
  return nrf_drv_spi_transfer(Spi , send, length, receive, length);
}

/***************************************************************************//**
 * @brief
 * This function write a single byte to a register via SPI
 *
 * @param[in] startAddress
 *  Register address to write
 *
 * @param[in] value
 *  Value to write to the Registers
 *
 ******************************************************************************/
void ADXL343_write_single(uint8_t startAddress, uint8_t value)
{
  uint8_t sendBuffer[2];
  sendBuffer[0] = SPI_WRITE | startAddress;       // set register address
  sendBuffer[1] = value;                          // set new register value

  ret_code_t err_code;
  err_code = ADXL343_spiCommunicate(sendBuffer, 0, 2);  // communicate
  APP_ERROR_CHECK(err_code);
}

/***************************************************************************//**
 * @brief
 * This function writes several bytes to the registers via SPI
 * a bit slow because of the dynamic memory allocation
 *
 * @param[in] startAddress
 *  First Register Address to write
 *
 * @param[in] data
 *  Pointer to the values to write to the Registers
 *
 * @param[in] length
 * Number of register to write to
 *
 * @return
 *  0 every thing is OK
 ******************************************************************************/
uint8_t ADXL343_write_multi(uint8_t startAddress, uint8_t *data, uint8_t length)
{
  uint8_t *sendBuffer = malloc((length+1)*sizeof(uint8_t));  // allocate memory for sendBuffer
  if (0 == sendBuffer)
    {return 1;}                     // return if there is not enough memory
  for(int i = 1; i < (length+1); i++)  
    {sendBuffer[i] = data[i-1];}    // fill the sendBuffer

  sendBuffer[0] = SPI_WRITE_MULTI | startAddress; // set register address
  
  ret_code_t err_code;
  err_code = ADXL343_spiCommunicate(sendBuffer, 0, (length+1));  // communicate
  APP_ERROR_CHECK(err_code);
  
  /* free the memory */
  free(sendBuffer);

  return 0;
}

/***************************************************************************//**
 * @brief
 * This function read a single byte from a register via SPI
 *
 * @param[in] startAddress
 *  Register Address to read
 *
 * @param[in] data
 *  Pointer to received Data
 *
 ******************************************************************************/
void ADXL343_read_single(uint8_t startAddress, uint8_t *data)
{
  uint8_t sendBuffer[2];
  sendBuffer[0] = SPI_READ | startAddress;  // set register address
  sendBuffer[1] = 0;

  uint8_t receiveBuffer[2] = {0,0}; 
  
  ret_code_t err_code;
  err_code = ADXL343_spiCommunicate(sendBuffer, receiveBuffer, 2);  // communicate
  APP_ERROR_CHECK(err_code);

  data[0] = receiveBuffer[1]; // write back the received data
}

 /***************************************************************************//**
 * @brief
 * This function reads several bytes from the registers via SPI
 * a bit slow because of the dynamic memory allocation
 *
 * @param[in] startAddress
 *  First Register Address to write
 *
 * @param[in] data
 *  Pointer to received Data
 *
 * @param[in] length
 * Number of register to write to
 *
 * @return
 *  0 every thing is OK
 ******************************************************************************/
uint8_t ADXL343_read_multi(uint8_t startAddress, uint8_t *data, uint8_t length)
{
  uint8_t *sendBuffer = malloc((length+1)*sizeof(uint8_t));  // allocate memory for sendBuffer
  if (0 == sendBuffer)
    {return 1;}                     // return if there is not enough memory
  for(uint16_t i = 1; i < (length+1); i++)  
    {sendBuffer[i] = 0;}            // clear sendBuffer

  sendBuffer[0] = SPI_READ_MULTI | startAddress;  // set register address

  uint8_t *receiveBuffer = malloc((length+1)*sizeof(uint8_t));  // allocate memory for receiveBuffer
  if (0 == receiveBuffer)
  {
    free(sendBuffer);   // free the sendBuffer memory if there is not enough memory
    return 2;           // return if there is not enough memory
  }                     
  for(uint16_t i = 0; i < (length+1); i++)  
    {receiveBuffer[i] = 0;}         // clear receiveBuffer
  
  ret_code_t err_code;
  err_code = ADXL343_spiCommunicate(sendBuffer, receiveBuffer, (length+1));  // communicate
  APP_ERROR_CHECK(err_code);

  for(uint16_t i = 1; i < (length+1); i++)  // write back the received data
  {
    data[i-1] = receiveBuffer[i];
  }
  
  /* free the memory */
  free(receiveBuffer);
  free(sendBuffer);

  return 0;
}


/***************************************************************************//**
 * @brief
 * This function check if the ADXL343 contains the right ID
 *
 * @return
 *  0 every thing is OK
 ******************************************************************************/
uint8_t ADXL343_check_id()
{
  uint8_t id;

  ADXL343_read_single(ADXL343_REG_DEVID, &id);

  if(id != ADXL343_DEVID_ID)  // Analog Devices ID
    return 1;
  
  return 0;
}



/***************************************************************************//**
 * @brief
 * This function initialize the ADXL343
 *
 * @param[in] p_instance
 *  Pointer to a nrf_drv_spi_t reference
 *
 * @return
 *  0 every thing is OK
 ******************************************************************************/
uint8_t ADXL343_init(nrf_drv_spi_t const * const p_instance)
{
  Spi = p_instance;       // set the global nrf_drv_spi_t value
  if(Spi == 0)
    return 1;             // return if nrf_drv_spi_t is NULL

  /* setting up the Registers commented out the register that don't have to change from default */
//  ADXL343_write_single(ADXL343_REG_THRESH_TAP,0);
//  ADXL343_write_single(ADXL343_REG_OFSX,0);
//  ADXL343_write_single(ADXL343_REG_OFSY,0);
//  ADXL343_write_single(ADXL343_REG_OFSZ,0);
//  ADXL343_write_single(ADXL343_REG_DUR,0);
//  ADXL343_write_single(ADXL343_REG_LATENT,0);
//  ADXL343_write_single(ADXL343_REG_WINDOW,0);
  ADXL343_write_single(ADXL343_REG_THRESH_ACT,24);    // 64 * 62.5mg/LSB = 4g
  ADXL343_write_single(ADXL343_REG_THRESH_INACT,20);  // 64 * 62.5mg/LSB = 1.25g
  ADXL343_write_single(ADXL343_REG_TIME_INACT,1);     // 1 * 1sec. = 1sec
  ADXL343_write_single(ADXL343_REG_ACT_INACT_CTL,0b01110111); // dc all axis
//  ADXL343_write_single(ADXL343_REG_THRESH_FF,0);
//  ADXL343_write_single(ADXL343_REG_TIME_FF,0);
//  ADXL343_write_single(ADXL343_REG_TAP_AXES,0);
  ADXL343_write_single(ADXL343_REG_BW_RATE,ADXL343_BIT_LOW_POWER|ADXL343_DATARATE_12_5_HZ); // 12.5Hz low power mode
  ADXL343_write_single(ADXL343_REG_INT_MAP,ADXL343_BIT_INACTIVITY_MAP); // INACTIVITY maped to INT2
  ADXL343_write_single(ADXL343_REG_DATA_FORMAT,ADXL343_RANGE_16_G); // 16g resolution
//  ADXL343_write_single(ADXL343_REG_FIFO_CTL,0);

  /* Last registers to set */
  ADXL343_write_single(ADXL343_REG_POWER_CTL, ADXL343_BIT_LINK
                                             |ADXL343_BIT_MEASURE
                                             |ADXL343_BIT_AUTOSLEEP
                                             |ADXL343_BIT_SLEEP
                                             |ADXL343_WAKEUP_8_HZ);
  ADXL343_write_single(ADXL343_REG_INT_ENABLE,ADXL343_BIT_ACTIVITY_EN|ADXL343_BIT_INACTIVITY_EN); // Interrupt sources

   // read test data
  int16_t testData[3] = {0,0,0};
  ADXL343_getXYZValues(testData);
  
  uint8_t read_interrupts;
  ADXL343_read_single(ADXL343_REG_INT_SOURCE, &read_interrupts);

  return 0;
}


/***************************************************************************//**
 * @brief
 *  This function read the values from the Sensor and convert them to int16_t,
 *  because the sensor response 16Bit with 4 dummy bits
 *
 * @param[out] data
 *  Pointer to the three 16bit accelerator values
 *
 ******************************************************************************/
void ADXL343_getXYZValues(int16_t *data)
{
  uint8_t tempt_write[N_BYTES_FOR_XYZ_READ] = {SPI_READ_MULTI | ADXL343_REG_DATAX0};  // write buffer
  for(uint8_t i =1; i < N_BYTES_FOR_XYZ_READ; i++) {tempt_write[i] = 0;}  // clear write buffer
  uint8_t raw_data[N_BYTES_FOR_XYZ_READ]  = {0};  // read buffer

  /* read all value registers DATAX0 - DATAZ1 */
  ret_code_t err_code;
  err_code = ADXL343_spiCommunicate(tempt_write, raw_data, N_BYTES_FOR_XYZ_READ); 
  APP_ERROR_CHECK(err_code);


  /* 8bit to 16bit */
  for(int i = 1; i < N_BYTES_FOR_XYZ_READ; i+=2)
  {
    data[(i-1)/2] =   (raw_data[i+1] << 8) | raw_data[i];
  }
}

/***************************************************************************//**
 * @brief
 *  This function read the values from the Sensor and convert them to int16_t,
 *  because the sensor response 16Bit with 4 dummy bits
 *
 * @param[out] data
 *  Pointer to the 16bit accelerator value
 *
 * @param[in] axis
 *  the axis to read
 *
 ******************************************************************************/
void ADXL343_getAxisValues(int16_t *data, e_acc_axis axis)
{
  uint8_t tempt_write[N_BYTES_FOR_ONE_READ];  // write buffer
  for(uint8_t i =0; i < N_BYTES_FOR_ONE_READ; i++) {tempt_write[i] = 0;} // clear write buffer
  uint8_t raw_data[N_BYTES_FOR_ONE_READ];     // read buffer
  for(uint8_t i =0; i < N_BYTES_FOR_ONE_READ; i++) {raw_data[i] = 0;}    // clear read buffer

  /* chose rigth register */
  switch (axis) 
  {
  case x_axis:
    tempt_write[0] = SPI_READ_MULTI | ADXL343_REG_DATAX0;
    break;
  case y_axis:
    tempt_write[0] = SPI_READ_MULTI | ADXL343_REG_DATAY0;
    break;
  case z_axis:
    tempt_write[0] = SPI_READ_MULTI | ADXL343_REG_DATAZ0;
    break;
  default:
    tempt_write[0] = SPI_READ_MULTI | ADXL343_REG_DATAX0;
    break;
  }

  /* read value from registers ?DATA_H and ?DATA_L */
  ret_code_t err_code;
  err_code = ADXL343_spiCommunicate(tempt_write, raw_data, N_BYTES_FOR_ONE_READ); 
  APP_ERROR_CHECK(err_code);

  /* 8bit to 16bit */
  data[0] = (raw_data[2] << 8) | raw_data[1];
}


void ADXL343_dump_all_registers()
{
  uint8_t data[29];
  ADXL343_read_single(ADXL343_REG_DEVID,data);
  NRF_LOG_INFO("Register 0: %d",data[0]);

  if(ADXL343_read_multi(ADXL343_REG_THRESH_TAP, data, 29))
  {
    NRF_LOG_ERROR("Fail while ADXL343_read_multi");
  }
  else
  {
    for(uint8_t i = 0; i < 29; i++)
    {
      NRF_LOG_INFO("Register %d: %d",29+i,data[i]);
    }
  }
}

/***************************************************************************//**
 * @brief
 *  This function set the Sensor in standby mode or normal mode
 *
 ******************************************************************************/
void ADXL343_setSensorPowermode(bool standby)
{
//  if(standby)
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_STANDBY);        // Standby
//  else
//  {
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_FULL_BANDWIDTH); // Full bandwidth measurement mode (continuous)
////    nrf_delay_ms(FILTER_SETTLE_TIME_370); // wait for filter settling
//  }
}


//void ADXL343_changeAccelerometerFilter(e_acc_filter filter)
//{
//  //         ODR 400Hz   800Hz   1600Hz  3200Hz  6400Hz 
//  // corner1 ft= 1.90Hz  3.81Hz  7.61Hz  15.24Hz 30.48Hz
//  // corner2 ft= 0.97Hz  1.94Hz  3.89Hz  7.79Hz  15.58Hz
//  // corner3 ft= 0.49Hz  0.98Hz  1.97Hz  3.94Hz  7.88Hz 
//  // corner4 ft= 0.24Hz  0.49Hz  0.99Hz  1.98Hz  3.96Hz
//  switch (filter) 
//  {
//  case ADXL343_f_off:
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_FULL_BANDWIDTH | ADXL343_HPF_DISABLE);
//    break;
//  case ADXL343_f_corner1:
//    ADXL343_write_single(ADXL343_R_HPF, ADXL343_HPF_CORNER1);
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_FULL_BANDWIDTH);
//    break;
//  case ADXL343_f_corner2:
//    ADXL343_write_single(ADXL343_R_HPF, ADXL343_HPF_CORNER2);
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_FULL_BANDWIDTH);
//    break;
//  case ADXL343_f_corner3:
//    ADXL343_write_single(ADXL343_R_HPF, ADXL343_HPF_CORNER3);
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_FULL_BANDWIDTH);
//    break;
//  case ADXL343_f_corner4:
//    ADXL343_write_single(ADXL343_R_HPF, ADXL343_HPF_CORNER4);
//    ADXL343_write_single(ADXL343_R_POWER_CTL, ADXL343_MODE_FULL_BANDWIDTH);
//    break;
//  default:
//    break;
//  }
//}
//
//void ADXL343_changeAccelerometerLpFilter(e_acc_lp_filter lpFilter)
//{
//  switch (lpFilter) 
//  {
//  case ADXL343_f_bandwidth_200HZ:
//    ADXL343_write_single(ADXL343_R_MEASURE, ADXL343_BANDWIDTH_200HZ | ADXL343_LOW_NOISE); // anti aliasing +low noise
//    break;
//  case ADXL343_f_bandwidth_400HZ:
//    ADXL343_write_single(ADXL343_R_MEASURE, ADXL343_BANDWIDTH_400HZ | ADXL343_LOW_NOISE); // anti aliasing +low noise
//    break;
//  case ADXL343_f_bandwidth_800HZ:
//    ADXL343_write_single(ADXL343_R_MEASURE, ADXL343_BANDWIDTH_800HZ | ADXL343_LOW_NOISE); // anti aliasing +low noise
//    break;
//  case ADXL343_f_bandwidth_1600HZ:
//    ADXL343_write_single(ADXL343_R_MEASURE, ADXL343_BANDWIDTH_1600HZ | ADXL343_LOW_NOISE); // anti aliasing +low noise
//    break;
//  case ADXL343_f_bandwidth_3200HZ:
//    ADXL343_write_single(ADXL343_R_MEASURE, ADXL343_BANDWIDTH_3200HZ | ADXL343_LOW_NOISE); // anti aliasing +low noise
//    break;
//  default:
//    break;
//  }
//}

