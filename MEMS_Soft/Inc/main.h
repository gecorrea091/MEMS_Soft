/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define UMB_TX_Pin GPIO_PIN_10
#define UMB_TX_GPIO_Port GPIOB
#define UMB_RX_Pin GPIO_PIN_11
#define UMB_RX_GPIO_Port GPIOB
#define G1_CS__Pin GPIO_PIN_8
#define G1_CS__GPIO_Port GPIOH
#define G2_CS__Pin GPIO_PIN_9
#define G2_CS__GPIO_Port GPIOH
#define G3_CS__Pin GPIO_PIN_10
#define G3_CS__GPIO_Port GPIOH
#define G4_CS__Pin GPIO_PIN_11
#define G4_CS__GPIO_Port GPIOH
#define TEL_TX_Pin GPIO_PIN_9
#define TEL_TX_GPIO_Port GPIOA
#define TEL_RX_Pin GPIO_PIN_10
#define TEL_RX_GPIO_Port GPIOA
#define ISO_MICRO_G__Pin GPIO_PIN_13
#define ISO_MICRO_G__GPIO_Port GPIOH
#define ISO_LIFT_OFF__Pin GPIO_PIN_14
#define ISO_LIFT_OFF__GPIO_Port GPIOH
#define ISO_NAV_ON__Pin GPIO_PIN_15
#define ISO_NAV_ON__GPIO_Port GPIOH

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define HMC_Addr 0x3C
#define HMC_ID_Reg_A 0x0A
#define HMC_Data_Reg 0x03
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define LIS_Read_All 0xA8
#define LIS_1_Addr 0x30
#define LIS_2_Addr 0x30
#define LIS_3_Addr 0x32
#define LIS_4_Addr 0x32
#define TMP_1_Addr 0x90
#define TMP_2_Addr 0x90
#define TMP_3_Addr 0x92
#define TMP_4_Addr 0x92 
#define TMP_Temp_Reg 0x00

#define Completo 1
#define placa_1e2 0
#define placa_3e4 1

//Escalas
#define Hi_limit_14bit	0x1B32 //85% de 0x1FFF (maior positivo de 14 bits em complemento de 2)
#define Low_limit_14bit	0x0D70 //42% de 0x1FFF

#define Hi_limit_16bit	0x6CCC //85% de 0x7FFF
#define Low_limit_16bit 0x35C2 //42% de 0x7FFF

#define giro_delay_to_down_scale		100
#define giro_1	1
#define giro_2	2
#define giro_3	3
#define giro_4	4

#define lis_delay_to_down_scale		100
#define lis_1	1
#define lis_2	2
#define lis_3	3
#define lis_4	4
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
