/**
  ******************************************************************************
  * @file    Examples/GPIOToggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "STM32f10x_spi.h"
#include "Accelerometer.h"


/** @addtogroup Examples
  * @{
  */
extern bool b_AccelerometerDataReady;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  static uint8_t  uc_SpeedCnt2;
  AccelerometerRaw_t AccelerometerData;
  
  
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, DISABLE);  

  /* Initialize Leds LD3 and LD4 mounted on STM32VLDISCOVERY board */
  STM32vldiscovery_LEDInit(LED3);
  STM32vldiscovery_LEDInit(LED4);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
  init_SPI1();                  // set up the SPI1
  init_Accelerometer();         // set up the Accelerometer
  init_AccelerometerISR();      // set up the LIS3DH isr

  while (1)
  {
#if 0
    /* Turn on LD2 and LD3 */
//    STM32vldiscovery_LEDOn(LED3);
//    STM32vldiscovery_LEDOn(LED4);    
    STM32vldiscovery_LEDToggle(LED3);

   /* Insert delay */
    Delay(50);
    /* Turn off LD3 and LD4 */
//    STM32vldiscovery_LEDOff(LED3);
//    STM32vldiscovery_LEDOff(LED4);
    STM32vldiscovery_LEDToggle(LED4);
    /* Insert delay */
    Delay(100);
#endif
//#if 0

  if( TRUE == b_AccelerometerDataReady )
  {
    b_AccelerometerDataReady = FALSE;
    AccelerometerCurrentData( &AccelerometerData );
     
    if( AccelerometerData.AXIS_X > 0 )
    {
      STM32vldiscovery_LEDOn(LED3);     // green
    }
    else
    {
      STM32vldiscovery_LEDOff(LED3);
    }
   
    if( AccelerometerData.AXIS_Y > 0 )
    { 
      STM32vldiscovery_LEDOn(LED4);     // blue 
    }
    else
    {
      STM32vldiscovery_LEDOff(LED4);
    }
   
 //    if( i_SPIZaxisData > 0 )
 //    { 
 //      STM32vldiscovery_LEDOn(LED4);     // blue 
 //    }
 //    else
 //    {
 //      STM32vldiscovery_LEDOff(LED4);
 //     }
    }
    else
    {
     uc_SpeedCnt2++;
    }
  }
//#endif   
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
