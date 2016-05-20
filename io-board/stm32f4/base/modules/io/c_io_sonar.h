/**
  ******************************************************************************
  * @file    modules/io/c_io_sonar.h
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    11-fevereiro-2014
  * @brief   Implementação da leitura do sonar XL-MaxSonar-EZ MB1200.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_IO_SONAR_H
#define C_IO_SONAR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#include "c_common_gpio.h"
#include "c_common_uart.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Definitions----------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define SERIAL_SONAR 1
//#define MB1200
#define HCRS04
/* Exported macro ------------------------------------------------------------*/
/* Exported variables---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void c_io_sonar_init();
float  c_io_sonar_read();

#ifdef __cplusplus
}
#endif

#endif //C_IO_SONAR_H


///**
//  ******************************************************************************
//  * @file    modules/io/c_io_sonar.h
//  * @author  Patrick Jose Pereira
//  * @version V1.0.0
//  * @date    11-fevereiro-2014
//  * @brief   Implementação da leitura do sonar XL-MaxSonar-EZ MB1200.
//  *****************************************************************************/
//
///* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef C_IO_SONAR_H
//#define C_IO_SONAR_H
//
///* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_conf.h"
//
//#include "c_common_gpio.h"
//#include "c_common_uart.h"
//
//#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>
//
//#ifdef __cplusplus
// extern "C" {
//#endif
//
///* Definitions----------------------------------------------------------------*/
//#define SERIAL 0 //change serial or analog read
///* Exported types ------------------------------------------------------------*/
///* Exported constants --------------------------------------------------------*/
///* Exported macro ------------------------------------------------------------*/
//
///* Exported functions ------------------------------------------------------- */
//void c_io_sonar_init();
//float  c_io_sonar_read();
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif //C_IO_SONAR_H
