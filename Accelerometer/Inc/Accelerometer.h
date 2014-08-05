/**********************************************************************
 * © 2014 Scorpion Automotive Ltd.
 *
 * FileName:        Accelerometer.h
 * Dependencies:    
 * Processor:       ARM-STM32F100RC
 * Compiler:        IAR embedded workbench for ARM 7.20.2.7431
 *                  IAR embedded workbench common compenents 7.1.1.3267
 *
 * Author           Manish Patel
 * Date             30/07/14
 * Project          
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
#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#ifdef __cplusplus
 extern "C" {
#endif

/*
****** Start VAR Defs ********************
*/

   /*
****** END VAR Defs ********************
*/
typedef struct { int16_t AXIS_X;
                 int16_t AXIS_Y;
                 int16_t AXIS_Z;
               } AccelerometerRaw_t;   

typedef struct { uint16_t AUX_1;
                 uint16_t AUX_2;
                 uint16_t AUX_3;
               } LIS3DH_Aux123Raw_t;

/*
******  MACROS ********************
*/
/* full scale setting - register & mask */
#define LIS3DH_READ_REG          (0x80)
#define LIS3DH_WRITE_REG         (0x00)
#define LIS3DH_INC_ADDR          (0x40)
#define LIS3DH_NO_INC_ADDR       (0x00)
#define LIS3DH_ADDR_BITS         (0x3F)
#define LIS3DH_DATA_AVAIL        (0x08)
#define LIS3DH_DATA_OVERRUN      (0x80)
#define LIS3DH_WHO_REG1          (0x0F)

// LIS3DH_CTRL_REG1 
enum {
       LIS3DH_CTRL_REG1                = 0x20,
       LIS3DH_ODR_POWERDOWN            = 0x00,
       LIS3DH_ODR_1Hz                  = 0x10,
       LIS3DH_ODR_10Hz                 = 0x20, 
       LIS3DH_ODR_25Hz                 = 0x30,
       LIS3DH_ODR_50Hz                 = 0x40,
       LIS3DH_ODR_100Hz                = 0x50,
       LIS3DH_ODR_200Hz                = 0x60,
       LIS3DH_ODR_400Hz                = 0x70,
       LIS3DH_ODR_1600Hz_LP            = 0x80,
       LIS3DH_ODR_1K25Hz_NP_5KHZ_LP    = 0x90,
       LIS3DH_LPENT                    = 0x08, 
       LIS3DH_ZEN                      = 0x04, 
       LIS3DH_YEN                      = 0x02, 
       LIS3DH_XEN                      = 0x01,
      }; 

// LIS3DH_CTRL_REG2 
enum { 
       LIS3DH_CTRL_REG2                = 0x21,
       LIS3DH_HP_NORMAL_RESET          = 0x00,
       LIS3DH_HP_REF                   = 0x40,
       LIS3DH_HP_NORMAL                = 0x80,
       LIS3DH_HP_AOUTORESET            = 0xC0,
       LIS3DH_HPCF1                    = 0x20,
       LIS3DH_HPCF2                    = 0x10,
       LIS3DH_FDS                      = 0x08,
       LIS3DH_HP_CLICK                 = 0x04,
       LIS3DH_HPIS2                    = 0x02,
       LIS3DH_HPIS1                    = 0x01,
     };

// LIS3DH_CTRL_REG3 
enum { 
       LIS3DH_CTRL_REG3                = 0x22,
       LIS3DH_I1_CLICK                 = 0x80,
       LIS3DH_I1_AOI1                  = 0x40,
       LIS3DH_I1_AOI2                  = 0x20,
       LIS3DH_I1_DRDY1                 = 0x10,
       LIS3DH_I1_DRDY2                 = 0x08,
       LIS3DH_I1_WTM                   = 0x04,
       LIS3DH_I1_OVERRUN               = 0x02,
     };

// LIS3DH_CTRL_REG4 
enum { 
       LIS3DH_CTRL_REG4                = 0x23,
       LIS3DH_BDU                      = 0x80,
       LIS3DH_BLE_BE                   = 0x40,
       LIS3DH_FSC_2G                   = 0x00,
       LIS3DH_FSC_4G                   = 0x10,
       LIS3DH_FSC_8G                   = 0x20,
       LIS3DH_FSC_16G                  = 0x30,
       LIS3DH_HRES                     = 0x08,
       LIS3DH_ST_NORMAL                = 0x00,
       LIS3DH_ST_TEST0                 = 0x02,
       LIS3DH_ST_TEST1                 = 0x04,
       LIS3DH_SIM                      = 0x01,
     };

// LIS3DH_CTRL_REG5 
enum { 
       LIS3DH_CTRL_REG5                = 0x24,
       LIS3DH_BOOT                     = 0x80,
       LIS3DH_FIFO_EN                  = 0x40,
       LIS3DH_LIR_1                    = 0x08,
       LIS3DH_D4D_1                    = 0x04,
     };

// LIS3DH_CTRL_REG6 
enum { 
       LIS3DH_CTRL_REG6                = 0x25,
       LIS3DH_I2_CLICK_EN              = 0x80,
       LIS3DH_I2_INT1                  = 0x40,
       LIS3DH_BOOT_I2                  = 0x10,
       LIS3DH_INT_ACTIVE_HL            = 0x02,
     };

#define LIS3DH_REFERENCE               (0x26)

// LIS3DH_STATUS_REG 
enum { 
       LIS3DH_STATUS_REG               = 0x27,
       LIS3DH_ZYXOR                    = 0x80,
       LIS3DH_ZOR                      = 0x40,
       LIS3DH_YOR                      = 0x20,
       LIS3DH_XOR                      = 0x10,
       LIS3DH_ZYXDA                    = 0x08,
       LIS3DH_ZDA                      = 0x04,
       LIS3DH_YDA                      = 0x02,
       LIS3DH_XDA                      = 0x01,
    };
#define LIS3DH_OUT_X_L                  (0x28)
#define LIS3DH_OUT_X_H                  (0x29)
#define LIS3DH_OUT_Y_L                  (0x2a)
#define LIS3DH_OUT_Y_H                  (0x2b)
#define LIS3DH_OUT_Z_L                  (0x2c)
#define LIS3DH_OUT_Z_H                  (0x2d)

enum { 
       LIS3DH_FIFO_CTRL_REG            = 0x2E,
       LIS3DH_FIFO_BYPASS              = 0x00,
       LIS3DH_FIFO_FIFO                = 0x40,
       LIS3DH_FIFO_STREAM              = 0x80,
       LIS3DH_FIFO_TRIG                = 0xC0,
       LIS3DH_TRIG_SEL_INT1            = 0x00,
       LIS3DH_TRIG_SEL_INT2            = 0x20,
     };

enum { 
       LIS3DH_FIFO_SRC_REG             = 0x2F,
       LIS3DH_FIFO_WTM                 = 0x80,
       LIS3DH_FIFO_OVR                 = 0x40,
       LIS3DH_FIFO_EMPTY               = 0x20,
    };
// LIS3DH_INT1_CFG
enum { 
       LIS3DH_INT1_CFG                 = 0x30,
       LIS3DH_INT1_OR                  = 0x00,
       LIS3DH_INT1_6D_MOTION           = 0x40,
       LIS3DH_INT1_AND                 = 0x80,
       LIS3DH_INT1_6D_POS              = 0xC0,
       LIS3DH_INT1_ZHE                 = 0x20,
       LIS3DH_INT1_ZLE                 = 0x10,
       LIS3DH_INT1_YHE                 = 0x08,
       LIS3DH_INT1_YLE                 = 0x04,
       LIS3DH_INT1_XHE                 = 0x02,
       LIS3DH_INT1_XLE                 = 0x01,
       LIS3DH_INT1_ALL_LO              = (LIS3DH_INT1_ZLE | LIS3DH_INT1_YLE | LIS3DH_INT1_XLE),
       LIS3DH_INT1_ALL_HI              = (LIS3DH_INT1_ZHE | LIS3DH_INT1_YHE | LIS3DH_INT1_XHE),
     };
       
// LIS3DH_INT1_SRC
enum { 
       LIS3DH_INT1_SRC                 = 0x31,
       LIS3DH_I1_SRC_IRQ               = 0x40,
       LIS3DH_I1_SRC_ZH                = 0x20,
       LIS3DH_I1_SRC_ZL                = 0x10,
       LIS3DH_I1_SRC_YH                = 0x08,
       LIS3DH_I1_SRC_YL                = 0x04,
       LIS3DH_I1_SRC_XH                = 0x02,
       LIS3DH_I1_SRC_XL                = 0x01,
     };
       
#define LIS3DH_INT1_THS          (0x32)
#define LIS3DH_INT1_DURATION     (0x33)



typedef enum { LIS3DH_TRIG_INT1                 = 0x00,
               LIS3DH_TRIG_INT2                 = 0x01
             } LIS3DH_TrigInt_t;

typedef enum { LIS3DH_SPI_4_WIRE               = 0x00,
               LIS3DH_SPI_3_WIRE               = 0x01
             } LIS3DH_SPIMode_t;   

typedef enum { LIS3DH_X_ENABLE                 = 0x01,
               LIS3DH_X_DISABLE                = 0x00,
               LIS3DH_Y_ENABLE                 = 0x02,
               LIS3DH_Y_DISABLE                = 0x00,
               LIS3DH_Z_ENABLE                 = 0x04,
               LIS3DH_Z_DISABLE                = 0x00
             } LIS3DH_AXISenable_t;

typedef enum { LIS3DH_INT1_6D_4D_DISABLE       = 0x00,
               LIS3DH_INT1_6D_ENABLE           = 0x01,
               LIS3DH_INT1_4D_ENABLE           = 0x02
             } LIS3DH_INT_6D_4D_t;

typedef enum { LIS3DH_UP_SX                    = 0x44, 
               LIS3DH_UP_DX                    = 0x42,
               LIS3DH_DW_SX                    = 0x41,
               LIS3DH_DW_DX                    = 0x48,
               LIS3DH_TOP                      = 0x60,
               LIS3DH_BOTTOM                   = 0x50  
             } LIS3DH_POSITION_6D_t;

typedef enum { LIS3DH_INT_MODE_OR              = 0x00,
               LIS3DH_INT_MODE_6D_MOVEMENT     = 0x01,
               LIS3DH_INT_MODE_AND             = 0x02,
               LIS3DH_INT_MODE_6D_POSITION     = 0x03
             } LIS3DH_Int1Mode_t; 

/*
****** END MACROS ********************
*/

/*
****** START FUNCTION PROTOTYPES ********************
*/
void init_SPI1(void);
void init_Accelerometer(void);
void init_AccelerometerISR( void );

uint8_t SPI1_ReadFromReg( uint16_t ui_DataToSend );
void SPI1_WriteToReg( uint16_t ui_AddrToSend, uint8_t uc_DataToWrite );
void AccelerometerCurrentData( AccelerometerRaw_t* ps_NewData );

/*
****** END FUNCTION PROTOTYPES ********************
*/

#endif /* __ACCELEROMETER_H */