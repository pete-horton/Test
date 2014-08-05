/**********************************************************************
 * © 2014 Scorpion Automotive Ltd.
 *
 * FileName:        Accelerometer.c
 * Dependencies:    
 * Processor:       ARM-STM32F100RC
 * Compiler:        IAR embedded workbench for ARM 7.20.2.7431
 *                  IAR embedded workbench common compenents 7.1.1.3267
 *
 * Author           Manish Patel
 * Date             30/07/14
 * Project          PH Edit
 * Subassembly      
 *
 *
 * REVISION HISTORY:
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author            Date        Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 **********************************************************************/

#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "STM32f10x_spi.h"
#include "Accelerometer.h"

/*
***** extern Variable defs ********************
*/
/*
***** end extern Variable defs ********************
*/

/*
*  start macros
*/

/*
*  End macros
*/

/*
****** Variable defs ********************
*/
bool b_AccelerometerDataReady;

/*
****** End Variable defs ********************
*/

/*
****** START FUNCTION PROTOTYPES ******************
*/

/*
****** END FUNCTION PROTOTYPES ********************
*/
/****************************************************************************
*   Name                init_SPI1        
*
*   Description         init SPI for 4-wire mode
*                       
*  Inputs  None
*
*   Outputs None
*
*   Returns 
*
*   Changes:
*   Date    Name        Detail
*
*
*****************************************************************************/
void init_SPI1(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_InitTypeDef SPI_InitStruct;

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );       // enable clock for used IO pins
  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );        // enable SPI1
  
  SPI_StructInit( &SPI_InitStruct );                         // clear SPI struct
  
  /* configure pins used by SPI1
   * PA5 = SCK
   * PA6 = MISO
   * PA7 = MOSI
  */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          // alternate function push pull
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 ;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;         // alternate function push pull
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);                                              // set PA4 CS high

  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 15;
  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_SSOutputCmd(SPI1, ENABLE);
  SPI_Cmd(SPI1, ENABLE);
 }       // end init_SPI1


/****************************************************************************
*   Name                init_Accelerometer        
*
*   Description         init the LIS3DH
*                       
*  Inputs  None
*
*   Outputs None
*
*   Returns 
*
*   Changes:
*   Date    Name        Detail
*
*
*****************************************************************************/
void init_Accelerometer(void)
{
  volatile uint8_t  uc_SPIRxData   = 0;
  volatile uint8_t  uc_DataToWrite = 0;

  b_AccelerometerDataReady = FALSE;
  
  uc_DataToWrite = LIS3DH_ODR_100Hz | LIS3DH_XEN | LIS3DH_YEN; 
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG1, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG1 );

  uc_DataToWrite = 0;
  uc_DataToWrite = LIS3DH_HP_NORMAL | LIS3DH_HPIS1;      //LIS3DH_HP_NORMAL;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG2, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG2 );

  uc_DataToWrite = 0;
  uc_DataToWrite = LIS3DH_I1_AOI1;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG3, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG3 );

  uc_DataToWrite = LIS3DH_BDU;
  uc_DataToWrite = LIS3DH_FSC_2G | LIS3DH_HRES;      //LIS3DH_BDU | LIS3DH_FSC_2G;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG4, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG4 );

  uc_DataToWrite = 0;
  uc_DataToWrite = LIS3DH_LIR_1;        //LIS3DH_LIR_1;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG5, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG5 );

  uc_DataToWrite = 0;
  uc_DataToWrite = 0x00;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG6, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG6 );

  uc_DataToWrite = 0;
  uc_DataToWrite = 0x00;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_REFERENCE, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_REFERENCE );

  uc_DataToWrite = 0;
  uc_DataToWrite = 0x10;        //0x00;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_THS, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_THS );

  uc_DataToWrite = 0;
  uc_DataToWrite = 0x00;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_DURATION, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_DURATION );

  uc_DataToWrite = 0;
  uc_DataToWrite = LIS3DH_INT1_OR | LIS3DH_INT1_YHE | LIS3DH_INT1_XHE;     // | LIS3DH_INT1_YHE | LIS3DH_INT1_YLE | LIS3DH_INT1_XHE | LIS3DH_INT1_XLE;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_CFG, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_CFG );

  uc_DataToWrite = 0;
  uc_DataToWrite = 0x00;
  SPI1_WriteToReg( LIS3DH_WRITE_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG5, uc_DataToWrite  );
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_CTRL_REG5 );
  
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_INT1_SRC );    //clear the int in MEMS

}       // end init_Accelerometer

/****************************************************************************
*  Name         SPI1_ReadFromReg  
*
*  Description  Reads data at the address specified in the accelerometer
*                       
*  Inputs       ui_AddrToSend - Address to obtain data from
*
*  Outputs 
*
*  Returns     Data rxed           
*
*  Changes:
*  Date    Name        Detail
*
*
*****************************************************************************/
uint8_t SPI1_ReadFromReg( uint16_t ui_AddrToSend )
{
  volatile uint8_t  uc_RxedData;
  
  ui_AddrToSend <<= 8;          // MSB is read/write + auto inc addr + 6bit address 
  
  uc_RxedData = SPI_I2S_ReceiveData( SPI1 );                                // flush rx buffer
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);                                            // clear PA4 CS low
  SPI_I2S_SendData(SPI1,  ui_AddrToSend );                      
  while( 0 == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) );                // wait until tx complete
  while( 0 == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) );               // wait until rx complete
  
  uc_RxedData = SPI_I2S_ReceiveData( SPI1 );                                    // get rxed data
  while( 1 == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_BSY ) );                // wait until SPI not busy
  GPIO_SetBits(GPIOA, GPIO_Pin_4);                                              // set PA4 CS high
  return( uc_RxedData );
}       // end SPI1_ReadFromReg

/****************************************************************************
*  Name          SPI1_WriteToReg      
*
*  Description   Write data at the address specified in the accelerometer
*                       
*  Inputs        ui_AddrToSend
*                uc_DataToWrite
*
*  Outputs None
*
*  Returns 
*
*  Changes:
*  Date    Name        Detail
*
*
*****************************************************************************/
void SPI1_WriteToReg( uint16_t ui_AddrToSend, uint8_t uc_DataToWrite )
{
  volatile uint16_t ui_AddrDataToSend;

  ui_AddrDataToSend = (ui_AddrToSend <<= 8) | uc_DataToWrite;                   // MSB is read/write + auto inc addr + 6bit address 
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);                                            // clear PA4 CS low
  SPI_I2S_SendData(SPI1,  ui_AddrDataToSend );                      
  while( 0 == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) );                // wait until tx complete
  while( 1 == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_BSY ) );                // wait until SPI not busy
  GPIO_SetBits(GPIOA, GPIO_Pin_4);                                              // set PA4 CS high
}       // end SPI1_WriteToReg

/****************************************************************************
*   Name                init_AccelerometerISR        
*
*   Description         init the port A0 as the ISR from LIS3DH MEMS
*                       
*  Inputs  None
*
*   Outputs None
*
*   Returns 
*
*   Changes:
*   Date    Name        Detail
*
*
*****************************************************************************/
void init_AccelerometerISR( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure port A 0 as input floating */
//  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE );       // enable clock for used IO pins

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Connect EXTI Line to portA GPIO Pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  /* Configure ISR EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  EXTI_GenerateSWInterrupt(EXTI_Line0);                         // enable sw int

}       // end init_AccelerometerISR


/****************************************************************************
*   Name                AccelerometerCurrentData        
*
*   Description         
*                       
*  Inputs  None
*
*   Outputs None
*
*   Returns 
*
*   Changes:
*   Date    Name        Detail
*
*
*****************************************************************************/
void AccelerometerCurrentData( AccelerometerRaw_t* ps_NewData )
{
  uint8_t  uc_SPIRxData;
  uint8_t  uc_SPIXLData;
  uint8_t  uc_SPIXHData;
  uint8_t  uc_SPIYLData;
  uint8_t  uc_SPIYHData;
  uint8_t  uc_SPIZLData;
  uint8_t  uc_SPIZHData;
  
  uc_SPIRxData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_STATUS_REG );
   
  if( LIS3DH_DATA_AVAIL == (uc_SPIRxData & LIS3DH_DATA_AVAIL) ||  LIS3DH_DATA_OVERRUN == (uc_SPIRxData & LIS3DH_DATA_OVERRUN) )
  {
    uc_SPIXLData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_OUT_X_L );
    uc_SPIXHData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_OUT_X_H );

    uc_SPIYLData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_OUT_Y_L );
    uc_SPIYHData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_OUT_Y_H );

    uc_SPIZLData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_OUT_Z_L );
    uc_SPIZHData = SPI1_ReadFromReg( LIS3DH_READ_REG | LIS3DH_NO_INC_ADDR | LIS3DH_OUT_Z_H );

    ps_NewData->AXIS_X = (uc_SPIXHData << 8) | uc_SPIXLData;
    ps_NewData->AXIS_Y = (uc_SPIYHData << 8) | uc_SPIYLData;
    ps_NewData->AXIS_Z = (uc_SPIZHData << 8) | uc_SPIZLData;
  }
}       // end AccelerometerCurrentData
