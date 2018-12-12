
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
uint16_t Giro_escala(uint8_t escala_atual, int16_t dado_giro, uint8_t giro);
uint8_t Acel_escala(uint8_t escala_atual, volatile int16_t x, volatile int16_t y, volatile int16_t z, uint8_t lis);
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

NAND_HandleTypeDef hnand1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef enum {
	spi_espera_i = 0,
	desativa_mag_3,
	espera_desativa_mag_3,
	desativa_mag_4,
	espera_desativa_mag_4,
	ativa_mag_1,
	espera_ativa_mag_1,
	ativa_mag_2,
	espera_ativa_mag_2
}tipo_en_spi_i;

tipo_en_spi_i en_spi_i = spi_espera_i;

typedef enum {
	spi_espera_f = 0,
	desativa_mag_1,
	espera_desativa_mag_1,
	desativa_mag_2,
	espera_desativa_mag_2,
	ativa_mag_3,
	espera_ativa_mag_3,
	ativa_mag_4,
	espera_ativa_mag_4
}tipo_en_spi_f;

tipo_en_spi_f en_spi_f = spi_espera_f;

typedef enum {
	le_giro_espera = 0,
	pede_giro,
	espera_pede_giro,
	le_giro_pede_temp,
	espera_le_giro_pede_temp,
	le_temp,
	espera_le_temp,
	le_stat,
	espera_le_stat
}tipo_giro_spi;

tipo_giro_spi le_giro_1 = le_giro_espera;
tipo_giro_spi le_giro_2 = le_giro_espera;
tipo_giro_spi le_giro_3 = le_giro_espera;
tipo_giro_spi le_giro_4 = le_giro_espera;

typedef enum {
	le_atm_espera = 0,
	le_acel,
	espera_le_acel,
	norm_acel,
	le_tmp,
	espera_le_tmp,
	le_mag,
	espera_le_mag,
	lis_escala,
	espera_lis_escala
}tipo_le_atm;

__IO tipo_le_atm le_atm_placa_1 = le_atm_espera;
tipo_le_atm le_atm_placa_2 = le_atm_espera;
tipo_le_atm le_atm_placa_3 = le_atm_espera;
tipo_le_atm le_atm_placa_4 = le_atm_espera;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t Mode_Reg_Config[2] = { 0x02, 0x00 };
uint8_t CTRL_RegB_Config[2] = { 0x01, 0x00 };
uint8_t CTRL_REG1_Config[2] = { 0x20, 0x27 };
uint8_t CTRL_REG4_Config[2] = { 0x23, 0xC0 };

typedef struct {
	uint8_t lsb;
	uint8_t msb;
}tipo_struct_sensor;

typedef union {
	tipo_struct_sensor str_data;
	int16_t data;
}tipo_union_sensor;

typedef struct {
	unsigned int giro : 14;
	unsigned int EA : 1;
	unsigned int ND : 1;
	unsigned int sinais: 16;
}tipo_dado_giro;

typedef struct {
	unsigned int stat_A : 7;
	unsigned int ND_A : 1;
	unsigned int stat_B : 7;
	unsigned int ND_B : 1;
	unsigned int : 16;
}tipo_stat_giro;

typedef union {
	tipo_union_sensor stat_16;
	tipo_stat_giro stat_full;
}tipo_union_stat_giro;

tipo_union_stat_giro giro1e2, giro3e4;

typedef struct {
	unsigned int giro_1_i : 2;
	unsigned int giro_2_i : 2;
	unsigned int giro_3_i : 2;
	unsigned int giro_4_i : 2;
	unsigned int giro_1_f : 2;
	unsigned int giro_2_f : 2;
	unsigned int giro_3_f : 2;
	unsigned int giro_4_f : 2;
	unsigned int : 16;
}tipo_escala_giro;

typedef union {
	tipo_union_sensor escala_16;
	tipo_escala_giro escala_full;
}tipo_union_escalas;

tipo_union_escalas escala_giro;

typedef union {
	tipo_union_sensor union_giro; //16 bits
	tipo_dado_giro dado_giro; // 32 bits
}tipo_union_giro;

tipo_union_giro union_giro;

typedef struct {
	unsigned int micro_g : 1;
	unsigned int lift_off : 1;
	unsigned int : 6;
	unsigned int Lis_1 : 2;
	unsigned int Lis_2 : 2;
	unsigned int Lis_3 : 2;
	unsigned int Lis_4 : 2;
	unsigned int : 16;
}tipo_sinais_escala_acel;

typedef union {
	tipo_union_sensor escala_16;
	tipo_sinais_escala_acel escala_full;
}tipo_union_sinais_escalas;

tipo_union_sinais_escalas sinais_escala_acel;

typedef struct {
	unsigned int n_1 : 4;
	unsigned int n_2 : 4;
	unsigned int n_3 : 4;
	unsigned int n_4 : 4;
	unsigned int : 16;
}tipo_nibbles;

typedef union {
	tipo_nibbles nibble;
	uint16_t dado;
}tipo_hex_to_ascii;

typedef struct {
	tipo_union_sensor h_1;
	tipo_union_sensor h_2;
}tipo_half_contador;

typedef union {
	tipo_half_contador half;
	uint32_t word;
}tipo_contador;

volatile tipo_contador contador;

typedef struct {
	tipo_union_sensor lis_1_x;
	tipo_union_sensor lis_1_y;
	tipo_union_sensor lis_1_z;
	tipo_union_sensor lis_2_x;
	tipo_union_sensor lis_2_y;
	tipo_union_sensor lis_2_z;
	tipo_union_sensor lis_3_x;
	tipo_union_sensor lis_3_y;
	tipo_union_sensor lis_3_z;
	tipo_union_sensor lis_4_x;
	tipo_union_sensor lis_4_y;
	tipo_union_sensor lis_4_z;	
	tipo_union_sensor hmc_1_x;
	tipo_union_sensor hmc_1_y;
	tipo_union_sensor hmc_1_z;
	tipo_union_sensor hmc_2_x;
	tipo_union_sensor hmc_2_y;
	tipo_union_sensor hmc_2_z;
	tipo_union_sensor hmc_3_x;
	tipo_union_sensor hmc_3_y;
	tipo_union_sensor hmc_3_z;
	tipo_union_sensor hmc_4_x;
	tipo_union_sensor hmc_4_y;
	tipo_union_sensor hmc_4_z;
	tipo_union_sensor giro_1_i;
	tipo_union_sensor giro_2_i;
	tipo_union_sensor giro_3_i;
	tipo_union_sensor giro_4_i;
	tipo_union_sensor temp_1_i;
	tipo_union_sensor temp_2_i;
	tipo_union_sensor temp_3_i;
	tipo_union_sensor temp_4_i;
	tipo_union_sensor giro_1_f;
	tipo_union_sensor giro_2_f;
	tipo_union_sensor giro_3_f;
	tipo_union_sensor giro_4_f;
	tipo_union_sensor temp_1_f;
	tipo_union_sensor temp_2_f;
	tipo_union_sensor temp_3_f;
	tipo_union_sensor temp_4_f;
	tipo_union_sensor tmp_1;
	tipo_union_sensor tmp_2;
	tipo_union_sensor tmp_3;
	tipo_union_sensor tmp_4;
	tipo_union_sensor sinais_escala_acel;
	tipo_union_sensor escala_giros;
	tipo_union_sensor stat_1e2_i;
	tipo_union_sensor stat_3e4_i;
	tipo_union_sensor stat_1e2_f;
	tipo_union_sensor stat_3e4_f;
	tipo_union_sensor chk_sum;
}tipo_struct_dados;

typedef union {
	tipo_struct_dados str_sensor;
	tipo_union_sensor Vetor_dados[sizeof(tipo_struct_dados) / 2];
}tipo_boi;

tipo_boi sensor = { 0 };
tipo_boi shadow = { 0 };
tipo_struct_dados media[8] = { 0 };
uint8_t p = 0;

typedef struct {
	unsigned int Escala_Giro_1 : 2;
	unsigned int Escala_Giro_2 : 2;
	unsigned int Escala_Giro_3 : 2;
	unsigned int Escala_Giro_4 : 2;
	unsigned int Escala_Acel_1 : 2;
	unsigned int Escala_Acel_2 : 2;
	unsigned int Escala_Acel_3 : 2;
	unsigned int Escala_Acel_4 : 2;
	unsigned int : 6;
	unsigned int Muda_escala_Giro : 1;
	unsigned int Muda_escala_Acel : 1;
	unsigned int Troca_Giro_1 : 1;
	unsigned int Troca_Giro_2 : 1;
	unsigned int Troca_Giro_3 : 1;
	unsigned int Troca_Giro_4 : 1;
	unsigned int Troca_Acel_1 : 1;
	unsigned int Troca_Acel_2 : 1;
	unsigned int Troca_Acel_3 : 1;
	unsigned int Troca_Acel_4 : 1;
}tipo_escala_sensores;

tipo_escala_sensores escala_sensor;

void calc_media(void);
void hex_to_ascii(tipo_union_sensor dado, char *buffer);

uint8_t HMC_1_Data[6] = { 0 }, LIS_1_Data[6] = { 0 }, TMP_1_Data[2] = { 0 };
uint8_t HMC_2_Data[6] = { 0 }, LIS_2_Data[6] = { 0 }, TMP_2_Data[2] = { 0 };
uint8_t HMC_3_Data[6] = { 0 }, LIS_3_Data[6] = { 0 }, TMP_3_Data[2] = { 0 };
uint8_t HMC_4_Data[6] = { 0 }, LIS_4_Data[6] = { 0 }, TMP_4_Data[2] = { 0 };
int16_t acc_x = 0, acc_y = 0, acc_z = 0, hmc_x = 0, hmc_y = 0, hmc_z = 0;
uint32_t i;
uint32_t Tempo = 0;
uint16_t spiTx[10] = { 0 }, spiRx[10] = { 0 };
uint16_t dummy = 0x0000;
float Temp = 0, adis_temp = 0, adis_gyro = 0;
float facc_x = 0, facc_y = 0, facc_z = 0, fhmc_x = 0, fhmc_y = 0, fhmc_z = 0;
uint8_t Inicializacao = 0;
uint8_t send_1 = 0, send_2 = 0;
tipo_union_sensor read_stat;

volatile char data_buffer[sizeof(tipo_struct_dados) * 2 + 12];
volatile char buffer_out[sizeof(tipo_struct_dados) * 2 + 12];

uint8_t vez_de_quem = placa_1e2;
uint8_t micro_g = 0;
uint8_t lift_off = 0;
volatile uint8_t contagem_giro_1 = 0, contagem_giro_2 = 0, contagem_giro_3 = 0, contagem_giro_4 = 0;
volatile uint8_t contagem_lis_1 = 0, contagem_lis_2 = 0, contagem_lis_3 = 0, contagem_lis_4 = 0;

int32_t soma;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemCoreClockUpdate();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_FSMC_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SPI_ENABLE(&hspi1);

  HAL_Delay(100);

  HAL_Delay(1);
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // Envia Dummy pela SPI (envia zero)  

  HAL_Delay(1); 
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1); 
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(5);

  //Cada vez que inicia o sensor, o primeiro byte ele não entende, por isso é enviado esse dummy no início.
  /*
	 Config Placa 1
					*/
  HAL_I2C_Master_Transmit_DMA(&hi2c1, LIS_1_Addr, CTRL_REG1_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1); 
  HAL_I2C_Master_Transmit_DMA(&hi2c1, LIS_1_Addr, CTRL_REG4_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  //SENS_AVG
  spiTx[0] = 0x3601 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3801 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3904 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);

  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3201 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// GPIO_CTRL = DIO1 como saída ( nível baixo )
  HAL_Delay(1);
  spiTx[0] = 0x3301 | 0x8000; //SETA_DIO1_ALTO
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
  HAL_Delay(5); // Aguarda DIO1 subir 
  HAL_I2C_Master_Transmit_DMA(&hi2c1, HMC_Addr, CTRL_RegB_Config, 2);
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c1, HMC_Addr, Mode_Reg_Config, 2);	// Continuous measurement mode
  HAL_Delay(1);
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// DIO1 baixo
  HAL_Delay(1);


  /*
	 Config Placa 2
					*/
  HAL_I2C_Master_Transmit_DMA(&hi2c2, LIS_2_Addr, CTRL_REG1_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c2, LIS_2_Addr, CTRL_REG4_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  //SENS_AVG
  spiTx[0] = 0x3601 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3801 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3904 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);

  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3201 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// GPIO_CTRL = DIO1 como saída ( nível baixo )
  HAL_Delay(1);
  spiTx[0] = 0x3301 | 0x8000; //SETA_DIO1_ALTO
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
  HAL_Delay(5); // Aguarda DIO1 subir 
  HAL_I2C_Master_Transmit_DMA(&hi2c2, HMC_Addr, CTRL_RegB_Config, 2);
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c2, HMC_Addr, Mode_Reg_Config, 2);	// Continuous measurement mode
  HAL_Delay(1);
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// DIO1 baixo
  HAL_Delay(1);

  /*
	 Config Placa 3
					*/
  HAL_I2C_Master_Transmit_DMA(&hi2c1, LIS_3_Addr, CTRL_REG1_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c1, LIS_3_Addr, CTRL_REG4_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  //SENS_AVG
  spiTx[0] = 0x3601 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3801 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3904 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);

  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3201 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// GPIO_CTRL = DIO1 como saída ( nível baixo )
  HAL_Delay(1);
  spiTx[0] = 0x3301 | 0x8000; //SETA_DIO1_ALTO
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
  HAL_Delay(5); // Aguarda DIO1 subir 
  HAL_I2C_Master_Transmit_DMA(&hi2c1, HMC_Addr, CTRL_RegB_Config, 2);
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c1, HMC_Addr, Mode_Reg_Config, 2);	// Continuous measurement mode
  HAL_Delay(1);
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// DIO1 baixo
  HAL_Delay(1);

  /*
	 Config Placa 4
					*/
  HAL_I2C_Master_Transmit_DMA(&hi2c2, LIS_4_Addr, CTRL_REG1_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c2, LIS_4_Addr, CTRL_REG4_Config, 2); // ctrl_reg1 = modo normal, 50 Hz, x, y e z ligados
  HAL_Delay(1);
  //SENS_AVG
  spiTx[0] = 0x3601 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3801 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3904 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);

  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); // GPIO_CTRL = Linhas de I/O para baixo
  HAL_Delay(1);
  spiTx[0] = 0x3201 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// GPIO_CTRL = DIO1 como saída ( nível baixo )
  HAL_Delay(1);
  spiTx[0] = 0x3301 | 0x8000; //SETA_DIO1_ALTO
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
  HAL_Delay(5); // Aguarda DIO1 subir 
  HAL_I2C_Master_Transmit_DMA(&hi2c2, HMC_Addr, CTRL_RegB_Config, 2);
  HAL_Delay(1);
  HAL_I2C_Master_Transmit_DMA(&hi2c2, HMC_Addr, Mode_Reg_Config, 2);	// Continuous measurement mode
  HAL_Delay(1);
  spiTx[0] = 0x3300 | 0x8000;
  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);	// DIO1 baixo
  HAL_Delay(5);

  for (uint8_t loop = 0; loop < 216; loop++)
  {
	  data_buffer[loop] = 0x3F;
  }

  data_buffer[sizeof(tipo_struct_dados) * 2 + 11] = '\r';
  data_buffer[sizeof(tipo_struct_dados) * 2 + 10] = '\n';
  data_buffer[sizeof(tipo_struct_dados) * 2 + 9] = '#';
  data_buffer[0] = '@';

  sensor.str_sensor.escala_giros.data = 0;
  sensor.str_sensor.sinais_escala_acel.data = 0;
  contador.word = 0;
  Tempo = 4;
  p = 7;
  Inicializacao = Completo;
  
  
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  switch (en_spi_i)
	  {
	  case desativa_mag_3:
		  en_spi_i = espera_desativa_mag_3;
		  spiTx[0] = 0x3300 | 0x8000;
		  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case desativa_mag_4:
		  en_spi_i = espera_desativa_mag_4;
		  spiTx[0] = 0x3300 | 0x8000;
		  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1); 
		  break;
	  case ativa_mag_1:
		  en_spi_i = espera_ativa_mag_1;
		  spiTx[0] = 0x3301 | 0x8000;
		  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case ativa_mag_2:
		  en_spi_i = espera_ativa_mag_2;
		  spiTx[0] = 0x3301 | 0x8000;
		  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  default:
		  break;
	  }

	  switch (en_spi_f)
	  {
	  case desativa_mag_1:
		  en_spi_f = espera_desativa_mag_1;
		  spiTx[0] = 0x3300 | 0x8000;
		  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case desativa_mag_2:
		  en_spi_f = espera_desativa_mag_2;
		  spiTx[0] = 0x3300 | 0x8000;
		  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case ativa_mag_3:
		  en_spi_f = espera_ativa_mag_3;
		  spiTx[0] = 0x3301 | 0x8000;
		  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case ativa_mag_4:
		  en_spi_f = espera_ativa_mag_4;
		  spiTx[0] = 0x3301 | 0x8000;
		  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  default:
		  break;
	  }

	  switch (le_giro_1)
	  {
	  case pede_giro:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = 0x0400; //Giro
		  le_giro_1 = espera_pede_giro;
		  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case le_giro_pede_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_1 = espera_le_giro_pede_temp;
		  spiTx[0] = 0x0C00; //TEMP_OUT
		  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_1_i.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_1_f.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  break;
	  case le_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_1 = espera_le_temp;
		  spiTx[0] = 0x3C00; //DIAG_STAT
		  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_1_i.data, 1); //   <--- Le temperatura do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_1_f.data, 1); //   <--- Le temperatura do giro
		  }
		  break;
	  case le_stat:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = Giro_escala((uint8_t)escala_sensor.Escala_Giro_1, (int16_t)union_giro.dado_giro.giro, giro_1); //DUMMY
		  if (spiTx[0] == 0xB801) {
			  escala_sensor.Escala_Giro_1 = 0;
		  }
		  else if (spiTx[0] == 0xB802) {
			  escala_sensor.Escala_Giro_1 = 1;
		  }
		  else if (spiTx[0] == 0xB804) {
			  escala_sensor.Escala_Giro_1 = 2;
		  }
		  le_giro_1 = espera_le_stat;
		  HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &read_stat, 1); //   <--- Le status do giro
		  break;
	  default:
		  break;
	  }

	  switch (le_giro_2)
	  {
	  case pede_giro:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = 0x0400; //Giro
		  le_giro_2 = espera_pede_giro;
		  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case le_giro_pede_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_2 = espera_le_giro_pede_temp;
		  spiTx[0] = 0x0C00; //TEMP_OUT
		  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_2_i.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_2_f.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  break;
	  case le_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_2 = espera_le_temp;
		  spiTx[0] = 0x3C00; //DUMMY
		  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_2_i.data, 1); //   <--- Le temperatura do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_2_f.data, 1); //   <--- Le temperatura do giro
		  }
		  break;
	  case le_stat:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = Giro_escala((uint8_t)escala_sensor.Escala_Giro_2, (int16_t)union_giro.dado_giro.giro, giro_2); //DUMMY
		  if (spiTx[0] == 0xB801) {
			  escala_sensor.Escala_Giro_2 = 0;
		  }
		  else if (spiTx[0] == 0xB802) {
			  escala_sensor.Escala_Giro_2 = 1;
		  }
		  else if (spiTx[0] == 0xB804) {
			  escala_sensor.Escala_Giro_2 = 2;
		  }
		  le_giro_2 = espera_le_stat;
		  HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &read_stat, 1); //   <--- Le status do giro
		  break;
	  default:
		  break;
	  }

	  switch (le_giro_3)
	  {
	  case pede_giro:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = 0x0400; //Giro
		  le_giro_3 = espera_pede_giro;
		  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case le_giro_pede_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_3 = espera_le_giro_pede_temp;
		  spiTx[0] = 0x0C00; //TEMP_OUT
		  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_3_i.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_3_f.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  break;
	  case le_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_3 = espera_le_temp;
		  spiTx[0] = 0x3C00; //DUMMY
		  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_3_i.data, 1); //   <--- Le temperatura do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_3_f.data, 1); //   <--- Le temperatura do giro
		  }
		  break;
	  case le_stat:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = Giro_escala((uint8_t)escala_sensor.Escala_Giro_3, (int16_t)union_giro.dado_giro.giro, giro_3); //DUMMY
		  if (spiTx[0] == 0xB801) {
			  escala_sensor.Escala_Giro_3 = 0;
		  }
		  else if (spiTx[0] == 0xB802) {
			  escala_sensor.Escala_Giro_3 = 1;
		  }
		  else if (spiTx[0] == 0xB804) {
			  escala_sensor.Escala_Giro_3 = 2;
		  }
		  le_giro_3 = espera_le_stat;
		  HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &read_stat, 1); //   <--- Le status do giro
		  break;
	  default:
		  break;
	  }

	  switch (le_giro_4)
	  {
	  case pede_giro:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = 0x0400; //Giro
		  le_giro_4 = espera_pede_giro;
		  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &dummy, 1);
		  break;
	  case le_giro_pede_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_4 = espera_le_giro_pede_temp;
		  spiTx[0] = 0x0C00; //TEMP_OUT
		  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_4_i.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].giro_4_f.data, 1); //   <--- Le a velocidade angular do giro
		  }
		  break;
	  case le_temp:
		  for (i = 0; i <= 100; i++);
		  le_giro_4 = espera_le_temp;
		  spiTx[0] = 0x3C00; //DUMMY
		  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
		  if (vez_de_quem == placa_1e2) {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_4_i.data, 1); //   <--- Le temperatura do giro
		  }
		  else {
			  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &media[p].temp_4_f.data, 1); //   <--- Le temperatura do giro
		  }
		  break;
	  case le_stat:
		  for (i = 0; i <= 100; i++);
		  spiTx[0] = Giro_escala((uint8_t)escala_sensor.Escala_Giro_4, (int16_t)union_giro.dado_giro.giro, giro_4); //DUMMY
		  if (spiTx[0] == 0xB801) {
			  escala_sensor.Escala_Giro_4 = 0;
		  }
		  else if (spiTx[0] == 0xB802) {
			  escala_sensor.Escala_Giro_4 = 1;
		  }
		  else if (spiTx[0] == 0xB804) {
			  escala_sensor.Escala_Giro_4 = 2;
		  }
		  le_giro_4 = espera_le_stat;
		  HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_RESET);
		  HAL_SPI_TransmitReceive_IT(&hspi1, spiTx, &read_stat, 1); //   <--- Le status do giro
		  break;
	  default:
		  break;
	  }

	  switch (le_atm_placa_1)
	  {
	  case le_acel:
		  le_atm_placa_1 = espera_le_acel;
		  HAL_I2C_Mem_Read_DMA(&hi2c1, LIS_1_Addr, LIS_Read_All, I2C_MEMADD_SIZE_8BIT, LIS_1_Data, 6);
		  break;
	  case le_tmp:
		  le_atm_placa_1 = espera_le_tmp;
		  HAL_I2C_Mem_Read_DMA(&hi2c1, TMP_1_Addr, TMP_Temp_Reg, I2C_MEMADD_SIZE_8BIT, TMP_1_Data, 2);
		  break;
	  case le_mag:
		  le_atm_placa_1 = espera_le_mag;
		  HAL_I2C_Mem_Read_DMA(&hi2c1, HMC_Addr, HMC_Data_Reg, I2C_MEMADD_SIZE_8BIT, HMC_1_Data, 6);
		  break;
	  case lis_escala:
		  CTRL_REG4_Config[1] = Acel_escala((uint8_t)escala_sensor.Escala_Acel_1, (int16_t)sensor.str_sensor.lis_1_x.data, (int16_t)sensor.str_sensor.lis_1_y.data, (int16_t)sensor.str_sensor.lis_1_z.data, lis_1);
		  if(CTRL_REG4_Config[1] != 0){
			  if (CTRL_REG4_Config[1] == 0xC0) {
				  escala_sensor.Escala_Acel_1 = 0;
			  }
			  else if (CTRL_REG4_Config[1] == 0xD0) {
				  escala_sensor.Escala_Acel_1 = 1;
			  }
			  else if (CTRL_REG4_Config[1] == 0xF0) {
				  escala_sensor.Escala_Acel_1 = 2;
			  }
			  le_atm_placa_1 = espera_lis_escala;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, LIS_1_Addr, CTRL_REG4_Config, 2);
		  }
		  else {
			  le_atm_placa_1 = le_mag;
		  }
		  break;
	  default:
		  break;
	  }

	  switch (le_atm_placa_2)
	  {
	  case le_acel:
		  le_atm_placa_2 = espera_le_acel;
		  HAL_I2C_Mem_Read_DMA(&hi2c2, LIS_2_Addr, LIS_Read_All, I2C_MEMADD_SIZE_8BIT, LIS_2_Data, 6);
		  break;
	  case le_tmp:
		  le_atm_placa_2 = espera_le_tmp;
		  HAL_I2C_Mem_Read_DMA(&hi2c2, TMP_2_Addr, TMP_Temp_Reg, I2C_MEMADD_SIZE_8BIT, TMP_2_Data, 2);
		  break;
	  case le_mag:
		  le_atm_placa_2 = espera_le_mag;
		  HAL_I2C_Mem_Read_DMA(&hi2c2, HMC_Addr, HMC_Data_Reg, I2C_MEMADD_SIZE_8BIT, HMC_2_Data, 6);
		  break;
	  case lis_escala:
		  CTRL_REG4_Config[1] = Acel_escala((uint8_t)escala_sensor.Escala_Acel_2, (int16_t)sensor.str_sensor.lis_2_x.data, (int16_t)sensor.str_sensor.lis_2_y.data, (int16_t)sensor.str_sensor.lis_2_z.data, lis_2);
		  if (CTRL_REG4_Config[1] != 0) {
			  if (CTRL_REG4_Config[1] == 0xC0) {
				  escala_sensor.Escala_Acel_2 = 0;
			  }
			  else if (CTRL_REG4_Config[1] == 0xD0) {
				  escala_sensor.Escala_Acel_2 = 1;
			  }
			  else if (CTRL_REG4_Config[1] == 0xF0) {
				  escala_sensor.Escala_Acel_2 = 2;
			  }
			  le_atm_placa_2 = espera_lis_escala;
			  HAL_I2C_Master_Transmit_DMA(&hi2c2, LIS_2_Addr, CTRL_REG4_Config, 2);
		  }
		  else {
			  le_atm_placa_2 = le_mag;
		  }
		  break;
	  default:
		  break;
	  }

	  switch (le_atm_placa_3)
	  {
	  case le_acel:
		  le_atm_placa_3 = espera_le_acel;
		  HAL_I2C_Mem_Read_DMA(&hi2c1, LIS_3_Addr, LIS_Read_All, I2C_MEMADD_SIZE_8BIT, LIS_3_Data, 6);
		  break;
	  case le_tmp:
		  le_atm_placa_3 = espera_le_tmp;
		  HAL_I2C_Mem_Read_DMA(&hi2c1, TMP_3_Addr, TMP_Temp_Reg, I2C_MEMADD_SIZE_8BIT, TMP_3_Data, 2);
		  break;
	  case le_mag:
		  le_atm_placa_3 = espera_le_mag;
		  HAL_I2C_Mem_Read_DMA(&hi2c1, HMC_Addr, HMC_Data_Reg, I2C_MEMADD_SIZE_8BIT, HMC_3_Data, 6);
		  break;
	  case lis_escala:
		  CTRL_REG4_Config[1] = Acel_escala((uint8_t)escala_sensor.Escala_Acel_3, (int16_t)sensor.str_sensor.lis_3_x.data, (int16_t)sensor.str_sensor.lis_3_y.data, (int16_t)sensor.str_sensor.lis_3_z.data, lis_3);
		  if (CTRL_REG4_Config[1] != 0) {
			  if (CTRL_REG4_Config[1] == 0xC0) {
				  escala_sensor.Escala_Acel_3 = 0;
			  }
			  else if (CTRL_REG4_Config[1] == 0xD0) {
				  escala_sensor.Escala_Acel_3 = 1;
			  }
			  else if (CTRL_REG4_Config[1] == 0xF0) {
				  escala_sensor.Escala_Acel_3 = 2;
			  }
			  le_atm_placa_3 = espera_lis_escala;
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, LIS_3_Addr, CTRL_REG4_Config, 2);
		  }
		  else {
			  le_atm_placa_3 = le_mag;
		  }
		  break;
	  default:
		  break;
	  }

	  switch (le_atm_placa_4)
	  {
	  case le_acel:
		  le_atm_placa_4 = espera_le_acel;
		  HAL_I2C_Mem_Read_DMA(&hi2c2, LIS_4_Addr, LIS_Read_All, I2C_MEMADD_SIZE_8BIT, LIS_4_Data, 6);
		  break;
	  case le_tmp:
		  le_atm_placa_4 = espera_le_tmp;
		  HAL_I2C_Mem_Read_DMA(&hi2c2, TMP_4_Addr, TMP_Temp_Reg, I2C_MEMADD_SIZE_8BIT, TMP_4_Data, 2);
		  break;
	  case le_mag:
		  le_atm_placa_4 = espera_le_mag;
		  HAL_I2C_Mem_Read_DMA(&hi2c2, HMC_Addr, HMC_Data_Reg, I2C_MEMADD_SIZE_8BIT, HMC_4_Data, 6);
		  break;
	  case lis_escala:
		  CTRL_REG4_Config[1] = Acel_escala((uint8_t)escala_sensor.Escala_Acel_4, (int16_t)sensor.str_sensor.lis_4_x.data, (int16_t)sensor.str_sensor.lis_4_y.data, (int16_t)sensor.str_sensor.lis_4_z.data, lis_4);
		  if (CTRL_REG4_Config[1] != 0) {
			  if (CTRL_REG4_Config[1] == 0xC0) {
				  escala_sensor.Escala_Acel_4 = 0;
			  }
			  else if (CTRL_REG4_Config[1] == 0xD0) {
				  escala_sensor.Escala_Acel_4 = 1;
			  }
			  else if (CTRL_REG4_Config[1] == 0xF0) {
				  escala_sensor.Escala_Acel_4 = 2;
			  }
			  le_atm_placa_4 = espera_lis_escala;
			  HAL_I2C_Master_Transmit_DMA(&hi2c2, LIS_4_Addr, CTRL_REG4_Config, 2);
		  }
		  else {
			  le_atm_placa_4 = le_mag;
		  }
		  break;
	  default:
		  break;
	  }
	  
	  if (send_1 == 1) {

		  calc_media();

		  sensor.str_sensor.escala_giros.data = escala_giro.escala_16.data;

		  hex_to_ascii(sensor.str_sensor.lis_1_x, &data_buffer[1]);
		  hex_to_ascii(sensor.str_sensor.lis_1_y, &data_buffer[5]);
		  hex_to_ascii(sensor.str_sensor.lis_1_z, &data_buffer[9]);
		  hex_to_ascii(sensor.str_sensor.lis_2_x, &data_buffer[13]);
		  hex_to_ascii(sensor.str_sensor.lis_2_y, &data_buffer[17]);
		  hex_to_ascii(sensor.str_sensor.lis_2_z, &data_buffer[21]);
		  hex_to_ascii(sensor.str_sensor.lis_3_x, &data_buffer[25]);
		  hex_to_ascii(sensor.str_sensor.lis_3_y, &data_buffer[29]);
		  hex_to_ascii(sensor.str_sensor.lis_3_z, &data_buffer[33]);
		  hex_to_ascii(sensor.str_sensor.lis_4_x, &data_buffer[37]);
		  hex_to_ascii(sensor.str_sensor.lis_4_y, &data_buffer[41]);
		  hex_to_ascii(sensor.str_sensor.lis_4_z, &data_buffer[45]);
		  hex_to_ascii(sensor.str_sensor.hmc_1_x, &data_buffer[49]);
		  hex_to_ascii(sensor.str_sensor.hmc_1_y, &data_buffer[53]);
		  hex_to_ascii(sensor.str_sensor.hmc_1_z, &data_buffer[57]);
		  hex_to_ascii(sensor.str_sensor.hmc_2_x, &data_buffer[61]);
		  hex_to_ascii(sensor.str_sensor.hmc_2_y, &data_buffer[65]);
		  hex_to_ascii(sensor.str_sensor.hmc_2_z, &data_buffer[69]);
		  hex_to_ascii(sensor.str_sensor.hmc_3_x, &data_buffer[73]);
		  hex_to_ascii(sensor.str_sensor.hmc_3_y, &data_buffer[77]);
		  hex_to_ascii(sensor.str_sensor.hmc_3_z, &data_buffer[81]);
		  hex_to_ascii(sensor.str_sensor.hmc_4_x, &data_buffer[85]);
		  hex_to_ascii(sensor.str_sensor.hmc_4_y, &data_buffer[89]);
		  hex_to_ascii(sensor.str_sensor.hmc_4_z, &data_buffer[93]);
		  hex_to_ascii(sensor.str_sensor.giro_1_i, &data_buffer[97]);
		  hex_to_ascii(sensor.str_sensor.giro_2_i, &data_buffer[101]);
		  hex_to_ascii(sensor.str_sensor.giro_3_i, &data_buffer[105]);
		  hex_to_ascii(sensor.str_sensor.giro_4_i, &data_buffer[109]);
		  hex_to_ascii(sensor.str_sensor.temp_1_i, &data_buffer[113]);
		  hex_to_ascii(sensor.str_sensor.temp_2_i, &data_buffer[117]);
		  hex_to_ascii(sensor.str_sensor.temp_3_i, &data_buffer[121]);
		  hex_to_ascii(sensor.str_sensor.temp_4_i, &data_buffer[125]);
		  hex_to_ascii(sensor.str_sensor.giro_1_f, &data_buffer[129]);
		  hex_to_ascii(sensor.str_sensor.giro_2_f, &data_buffer[133]);
		  hex_to_ascii(sensor.str_sensor.giro_3_f, &data_buffer[137]);
		  hex_to_ascii(sensor.str_sensor.giro_4_f, &data_buffer[141]);
		  hex_to_ascii(sensor.str_sensor.temp_1_f, &data_buffer[145]);
		  hex_to_ascii(sensor.str_sensor.temp_2_f, &data_buffer[149]);
		  hex_to_ascii(sensor.str_sensor.temp_3_f, &data_buffer[153]);
		  hex_to_ascii(sensor.str_sensor.temp_4_f, &data_buffer[157]);
		  hex_to_ascii(sensor.str_sensor.tmp_1, &data_buffer[161]);
		  hex_to_ascii(sensor.str_sensor.tmp_1, &data_buffer[165]);
		  hex_to_ascii(sensor.str_sensor.tmp_3, &data_buffer[169]);
		  hex_to_ascii(sensor.str_sensor.tmp_1, &data_buffer[173]);
		  hex_to_ascii(sensor.str_sensor.sinais_escala_acel, &data_buffer[177]);
		  hex_to_ascii(sensor.str_sensor.escala_giros, &data_buffer[181]);
		  hex_to_ascii(sensor.str_sensor.stat_1e2_i, &data_buffer[185]);
		  hex_to_ascii(sensor.str_sensor.stat_3e4_i, &data_buffer[189]);
		  hex_to_ascii(sensor.str_sensor.stat_1e2_f, &data_buffer[193]);
		  hex_to_ascii(sensor.str_sensor.stat_3e4_f, &data_buffer[197]);
		  for (uint16_t chk = 0; chk < 50; chk++)
		  {
			  sensor.str_sensor.chk_sum.data += (uint16_t)sensor.Vetor_dados[chk].data;
		  }
		  sensor.str_sensor.chk_sum.data += (uint16_t)contador.word;
		  hex_to_ascii(sensor.str_sensor.chk_sum, &data_buffer[201]);
		  hex_to_ascii(contador.half.h_2, &data_buffer[205]);
		  hex_to_ascii(contador.half.h_1, &data_buffer[209]);

		  for (uint16_t t = 0; t < (sizeof(tipo_struct_dados)*2 +12); t++)
		  {
			  buffer_out[t] = data_buffer[t];
		  }
		  send_1 = 0;
		  HAL_UART_Transmit_DMA(&huart1, buffer_out, sizeof(buffer_out));
		  //HAL_UART_Transmit_DMA(&huart3, buffer_out, sizeof(buffer_out));
	  }

	  /*
	  else if (Adis_Call == 14) {
		  Adis_Call = 0;
		  adis_temp = ((float)((int16_t)(spiRx[0] << 4)) * 0.1453 / 16) + 25;
		  adis_gyro = (float)((int16_t)(spiRx[1] << 2)) * 0.07326 / 4;
		  acc_x = (int16_t)((((uint16_t)LIS_Data[2]) << 8) + ((uint16_t)LIS_Data[1]));
		  acc_y = (int16_t)((((uint16_t)LIS_Data[4]) << 8) + ((uint16_t)LIS_Data[3]));
		  acc_z = (int16_t)((((uint16_t)LIS_Data[6]) << 8) + ((uint16_t)LIS_Data[5]));
		  facc_x = (float)acc_x * 12.0 / 4096.0 / 16.0;
		  facc_y = (float)acc_y * 12.0 / 4096.0 / 16.0;
		  facc_z = (float)acc_z * 12.0 / 4096.0 / 16.0;
		  fhmc_x = (float)hmc_x * 1.76 / 4096.0 / 16.0;
		  fhmc_y = (float)hmc_y * 1.76 / 4096.0 / 16.0;
		  fhmc_z = (float)hmc_z * 1.76 / 4096.0 / 16.0;
		  Temp = (float)((int16_t)((((uint16_t)TEMP_Data[0]) << 8) + ((uint16_t)TEMP_Data[1]))) / 256.0;
		  hmc_x = (int16_t)((((uint16_t)HMC_Data[0]) << 8) + ((uint16_t)HMC_Data[1]));
		  hmc_y = (int16_t)((((uint16_t)HMC_Data[2]) << 8) + ((uint16_t)HMC_Data[3]));
		  hmc_z = (int16_t)((((uint16_t)HMC_Data[4]) << 8) + ((uint16_t)HMC_Data[5]));
	  }
	  */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 240000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 240000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, G1_CS__Pin|G2_CS__Pin|G3_CS__Pin|G4_CS__Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : G1_CS__Pin G2_CS__Pin G3_CS__Pin G4_CS__Pin */
  GPIO_InitStruct.Pin = G1_CS__Pin|G2_CS__Pin|G3_CS__Pin|G4_CS__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : ISO_MICRO_G__Pin ISO_LIFT_OFF__Pin ISO_NAV_ON__Pin */
  GPIO_InitStruct.Pin = ISO_MICRO_G__Pin|ISO_LIFT_OFF__Pin|ISO_NAV_ON__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  FSMC_NAND_PCC_TimingTypeDef ComSpaceTiming;
  FSMC_NAND_PCC_TimingTypeDef AttSpaceTiming;

  /** Perform the NAND1 memory initialization sequence
  */
  hnand1.Instance = FSMC_NAND_DEVICE;
  /* hnand1.Init */
  hnand1.Init.NandBank = FSMC_NAND_BANK2;
  hnand1.Init.Waitfeature = FSMC_NAND_PCC_WAIT_FEATURE_ENABLE;
  hnand1.Init.MemoryDataWidth = FSMC_NAND_PCC_MEM_BUS_WIDTH_8;
  hnand1.Init.EccComputation = FSMC_NAND_ECC_DISABLE;
  hnand1.Init.ECCPageSize = FSMC_NAND_ECC_PAGE_SIZE_256BYTE;
  hnand1.Init.TCLRSetupTime = 0;
  hnand1.Init.TARSetupTime = 0;
  /* hnand1.Config */
  hnand1.Config.PageSize = 0;
  hnand1.Config.SpareAreaSize = 0;
  hnand1.Config.BlockSize = 0;
  hnand1.Config.BlockNbr = 0;
  hnand1.Config.PlaneNbr = 0;
  hnand1.Config.PlaneSize = 0;
  hnand1.Config.ExtraCommandEnable = DISABLE;
  /* ComSpaceTiming */
  ComSpaceTiming.SetupTime = 252;
  ComSpaceTiming.WaitSetupTime = 252;
  ComSpaceTiming.HoldSetupTime = 252;
  ComSpaceTiming.HiZSetupTime = 252;
  /* AttSpaceTiming */
  AttSpaceTiming.SetupTime = 252;
  AttSpaceTiming.WaitSetupTime = 252;
  AttSpaceTiming.HoldSetupTime = 252;
  AttSpaceTiming.HiZSetupTime = 252;

  if (HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void) {
	Tempo++;
	if (Inicializacao == Completo) {
		if (Tempo % 5 == 0) {
			if (vez_de_quem == placa_1e2) {
				en_spi_i = desativa_mag_3;
			}
			else if (vez_de_quem == placa_3e4) {
				en_spi_f = desativa_mag_1;
			}
		}
		else if (Tempo % 5 == 1) {
			if (vez_de_quem == placa_1e2) {
				if (p == 0) {
					send_1 = 1;
				}
			}
		}
		else if (Tempo % 5 == 4) {
			if (vez_de_quem == placa_1e2) {
				le_atm_placa_1 = lis_escala;
				le_atm_placa_2 = lis_escala;
				vez_de_quem = placa_3e4;
			}
			else if (vez_de_quem == placa_3e4) {
				le_atm_placa_3 = lis_escala;
				le_atm_placa_4 = lis_escala;
				vez_de_quem = placa_1e2;
			}
		}
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if (Inicializacao != Completo) {
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
	}

	switch (en_spi_i)
	{
	case espera_desativa_mag_3:
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		en_spi_i = desativa_mag_4;
		break;
	case espera_desativa_mag_4:
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
		en_spi_i = ativa_mag_1;
		break;
	case espera_ativa_mag_1:
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		en_spi_i = ativa_mag_2;
		break;
	case espera_ativa_mag_2:
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		en_spi_i = spi_espera_i;
		le_giro_1 = pede_giro;
		p++;
		p = p % 8;
		break;
	default:
		break;
	}

	switch (en_spi_f)
	{
	case espera_desativa_mag_1:
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		en_spi_f = desativa_mag_2;
		break;
	case espera_desativa_mag_2:
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		en_spi_f = ativa_mag_3;
		break;
	case espera_ativa_mag_3:
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		en_spi_f = ativa_mag_4;
		break;
	case espera_ativa_mag_4:
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
		en_spi_f = spi_espera_f;
		le_giro_1 = pede_giro;
		break;
	default:
		break;
	}

	switch (le_giro_1)
	{
	case espera_pede_giro:
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		le_giro_1 = le_giro_pede_temp;
		break;
	case espera_le_giro_pede_temp:
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		if (vez_de_quem == placa_1e2) {
			union_giro.union_giro.data = media[p].giro_1_i.data;
			media[p].giro_1_i.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro1e2.stat_full.ND_A = union_giro.dado_giro.ND;
		}
		else {
			union_giro.union_giro.data = media[p].giro_1_f.data;
			media[p].giro_1_f.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro1e2.stat_full.ND_A = union_giro.dado_giro.ND;
		}
		le_giro_1 = le_temp;
		break;
	case espera_le_temp:
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		le_giro_1 = le_stat;
		break;
	case espera_le_stat:
		HAL_GPIO_WritePin(G1_CS__GPIO_Port, G1_CS__Pin, GPIO_PIN_SET);
		giro1e2.stat_full.stat_A = read_stat.str_data.lsb;
		le_giro_1 = le_giro_espera;
		le_giro_2 = pede_giro;
		break;
	default:
		break;
	}

	switch (le_giro_2)
	{
	case espera_pede_giro:
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		le_giro_2 = le_giro_pede_temp;
		break;
	case espera_le_giro_pede_temp:
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		if (vez_de_quem == placa_1e2) {
			union_giro.union_giro.data = media[p].giro_2_i.data;
			media[p].giro_2_i.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro1e2.stat_full.ND_B = union_giro.dado_giro.ND;
		}
		else {
			union_giro.union_giro.data = media[p].giro_2_f.data;
			media[p].giro_2_f.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro1e2.stat_full.ND_B = union_giro.dado_giro.ND;
		}
		le_giro_2 = le_temp;
		break;
	case espera_le_temp:
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		le_giro_2 = le_stat;
		break;
	case espera_le_stat:
		HAL_GPIO_WritePin(G2_CS__GPIO_Port, G2_CS__Pin, GPIO_PIN_SET);
		giro1e2.stat_full.stat_B = read_stat.str_data.lsb;
		le_giro_2 = le_giro_espera;
		le_giro_3 = pede_giro;
		break;
	default:
		break;
	}

	switch (le_giro_3)
	{
	case espera_pede_giro:
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		le_giro_3 = le_giro_pede_temp;
		break;
	case espera_le_giro_pede_temp:
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		if (vez_de_quem == placa_1e2) {
			union_giro.union_giro.data = media[p].giro_3_i.data;
			media[p].giro_3_i.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro3e4.stat_full.ND_A = union_giro.dado_giro.ND;
		}
		else {
			union_giro.union_giro.data = media[p].giro_3_f.data;
			media[p].giro_3_f.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro3e4.stat_full.ND_A = union_giro.dado_giro.ND;
		}
		le_giro_3 = le_temp;
		break;
	case espera_le_temp:
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		le_giro_3 = le_stat;
		break;
	case espera_le_stat:
		HAL_GPIO_WritePin(G3_CS__GPIO_Port, G3_CS__Pin, GPIO_PIN_SET);
		giro3e4.stat_full.stat_A = read_stat.str_data.lsb;
		le_giro_3 = le_giro_espera;
		le_giro_4 = pede_giro;
		break;
	default:
		break;
	}

	switch (le_giro_4)
	{
	case espera_pede_giro:
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
		le_giro_4 = le_giro_pede_temp;
		break;
	case espera_le_giro_pede_temp:
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
		if (vez_de_quem == placa_1e2) {
			union_giro.union_giro.data = media[p].giro_4_i.data;
			media[p].giro_4_i.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro3e4.stat_full.ND_B = union_giro.dado_giro.ND;
		}
		else {
			union_giro.union_giro.data = media[p].giro_4_f.data;
			media[p].giro_4_f.data = ((int16_t)(union_giro.dado_giro.giro << 2)) / 4;
			giro3e4.stat_full.ND_B = union_giro.dado_giro.ND;
		}
		le_giro_4 = le_temp;
		break;
	case espera_le_temp:
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
		le_giro_4 = le_stat;
		break;
	case espera_le_stat:
		HAL_GPIO_WritePin(G4_CS__GPIO_Port, G4_CS__Pin, GPIO_PIN_SET);
		sinais_escala_acel.escala_full.lift_off = HAL_GPIO_ReadPin(ISO_LIFT_OFF__GPIO_Port, ISO_LIFT_OFF__Pin);
		sinais_escala_acel.escala_full.micro_g = HAL_GPIO_ReadPin(ISO_MICRO_G__GPIO_Port, ISO_MICRO_G__Pin);
		media[p].sinais_escala_acel.data = sinais_escala_acel.escala_16.data;
		giro3e4.stat_full.stat_B = read_stat.str_data.lsb;
		if (vez_de_quem == placa_1e2) {
			escala_giro.escala_full.giro_1_i = escala_sensor.Escala_Giro_1;
			escala_giro.escala_full.giro_2_i = escala_sensor.Escala_Giro_2;
			escala_giro.escala_full.giro_3_i = escala_sensor.Escala_Giro_3;
			escala_giro.escala_full.giro_4_i = escala_sensor.Escala_Giro_4;
			media[p].stat_1e2_i = giro1e2.stat_16;
			media[p].stat_3e4_i = giro3e4.stat_16;
		}
		else {
			escala_giro.escala_full.giro_1_f = escala_sensor.Escala_Giro_1;
			escala_giro.escala_full.giro_2_f = escala_sensor.Escala_Giro_2;
			escala_giro.escala_full.giro_3_f = escala_sensor.Escala_Giro_3;
			escala_giro.escala_full.giro_4_f = escala_sensor.Escala_Giro_4;
			media[p].stat_1e2_f = giro1e2.stat_16;
			media[p].stat_3e4_f = giro3e4.stat_16;
		}
		le_giro_4 = le_giro_espera;
		break;
	default:
		break;
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		switch (le_atm_placa_1)
		{
		case espera_le_acel:
			media[p].lis_1_x.str_data.msb = LIS_1_Data[0];
			media[p].lis_1_x.str_data.lsb = LIS_1_Data[1];
			media[p].lis_1_y.str_data.msb = LIS_1_Data[2];
			media[p].lis_1_y.str_data.lsb = LIS_1_Data[3];
			media[p].lis_1_z.str_data.msb = LIS_1_Data[4];
			media[p].lis_1_z.str_data.lsb = LIS_1_Data[5];
			le_atm_placa_1 = le_tmp;
			break;
		case espera_le_tmp:
			media[p].tmp_1.str_data.msb = TMP_1_Data[0];
			media[p].tmp_1.str_data.lsb = TMP_1_Data[1];
			le_atm_placa_1 = le_atm_espera;
			break;
		case espera_le_mag:
			media[p].hmc_1_x.str_data.msb = HMC_1_Data[0];
			media[p].hmc_1_x.str_data.lsb = HMC_1_Data[1];
			media[p].hmc_1_y.str_data.msb = HMC_1_Data[2];
			media[p].hmc_1_y.str_data.lsb = HMC_1_Data[3];
			media[p].hmc_1_z.str_data.msb = HMC_1_Data[4];
			media[p].hmc_1_z.str_data.lsb = HMC_1_Data[5];
			le_atm_placa_1 = le_acel;
			break;
		case espera_lis_escala:
			sinais_escala_acel.escala_full.Lis_1 = escala_sensor.Escala_Acel_1;
			le_atm_placa_1 = le_mag;
			break;
		default:
			break;
		}
		switch (le_atm_placa_3)
		{
		case espera_le_acel:
			media[p].lis_3_x.str_data.msb = LIS_3_Data[0];
			media[p].lis_3_x.str_data.lsb = LIS_3_Data[1];
			media[p].lis_3_y.str_data.msb = LIS_3_Data[2];
			media[p].lis_3_y.str_data.lsb = LIS_3_Data[3];
			media[p].lis_3_z.str_data.msb = LIS_3_Data[4];
			media[p].lis_3_z.str_data.lsb = LIS_3_Data[5];
			le_atm_placa_3 = le_tmp;
			break;
		case espera_le_tmp:
			media[p].tmp_3.str_data.msb = TMP_3_Data[0];
			media[p].tmp_3.str_data.lsb = TMP_3_Data[1];
			le_atm_placa_3 = le_atm_espera;
			break;
		case espera_le_mag:
			media[p].hmc_3_x.str_data.msb = HMC_3_Data[0];
			media[p].hmc_3_x.str_data.lsb = HMC_3_Data[1];
			media[p].hmc_3_y.str_data.msb = HMC_3_Data[2];
			media[p].hmc_3_y.str_data.lsb = HMC_3_Data[3];
			media[p].hmc_3_z.str_data.msb = HMC_3_Data[4];
			media[p].hmc_3_z.str_data.lsb = HMC_3_Data[5];
			le_atm_placa_3 = le_acel;
			break;
		case espera_lis_escala:
			sinais_escala_acel.escala_full.Lis_3 = escala_sensor.Escala_Acel_3;
			le_atm_placa_3 = le_mag;
			break;
		default:
			break;
		}
	}
	else if (hi2c == &hi2c2) {
		switch (le_atm_placa_2)
		{
		case espera_le_acel:
			media[p].lis_2_x.str_data.msb = LIS_2_Data[0];
			media[p].lis_2_x.str_data.lsb = LIS_2_Data[1];
			media[p].lis_2_y.str_data.msb = LIS_2_Data[2];
			media[p].lis_2_y.str_data.lsb = LIS_2_Data[3];
			media[p].lis_2_z.str_data.msb = LIS_2_Data[4];
			media[p].lis_2_z.str_data.lsb = LIS_2_Data[5];
			le_atm_placa_2 = le_tmp;
			break;
		case espera_le_tmp:
			media[p].tmp_2.str_data.msb = TMP_2_Data[0];
			media[p].tmp_2.str_data.lsb = TMP_2_Data[1];
			le_atm_placa_2 = le_atm_espera;
			break;
		case espera_le_mag:
			media[p].hmc_2_x.str_data.msb = HMC_2_Data[0];
			media[p].hmc_2_x.str_data.lsb = HMC_2_Data[1];
			media[p].hmc_2_y.str_data.msb = HMC_2_Data[2];
			media[p].hmc_2_y.str_data.lsb = HMC_2_Data[3];
			media[p].hmc_2_z.str_data.msb = HMC_2_Data[4];
			media[p].hmc_2_z.str_data.lsb = HMC_2_Data[5];
			le_atm_placa_2 = le_acel;
			break;
		case espera_lis_escala:
			sinais_escala_acel.escala_full.Lis_2 = escala_sensor.Escala_Acel_2;
			le_atm_placa_2 = le_mag;
			break;
		default:
			break;
		}
		switch (le_atm_placa_4)
		{
		case espera_le_acel:
			media[p].lis_4_x.str_data.msb = LIS_4_Data[0];
			media[p].lis_4_x.str_data.lsb = LIS_4_Data[1];
			media[p].lis_4_y.str_data.msb = LIS_4_Data[2];
			media[p].lis_4_y.str_data.lsb = LIS_4_Data[3];
			media[p].lis_4_z.str_data.msb = LIS_4_Data[4];
			media[p].lis_4_z.str_data.lsb = LIS_4_Data[5];
			le_atm_placa_4 = le_tmp;
			break;
		case espera_le_tmp:
			media[p].tmp_4.str_data.msb = TMP_4_Data[0];
			media[p].tmp_4.str_data.lsb = TMP_4_Data[1];
			le_atm_placa_4 = le_atm_espera;
			send_2 = 1;
			break;
		case espera_le_mag:
			media[p].hmc_4_x.str_data.msb = HMC_4_Data[0];
			media[p].hmc_4_x.str_data.lsb = HMC_4_Data[1];
			media[p].hmc_4_y.str_data.msb = HMC_4_Data[2];
			media[p].hmc_4_y.str_data.lsb = HMC_4_Data[3];
			media[p].hmc_4_z.str_data.msb = HMC_4_Data[4];
			media[p].hmc_4_z.str_data.lsb = HMC_4_Data[5];
			le_atm_placa_4 = le_acel;
			break;
		case espera_lis_escala:
			sinais_escala_acel.escala_full.Lis_4 = escala_sensor.Escala_Acel_4;
			le_atm_placa_4 = le_mag;
			break;
		default:
			break;
		}
	}
}

void hex_to_ascii(tipo_union_sensor dado, char *buffer) {
	tipo_hex_to_ascii data_hex; 
	data_hex.dado = dado.data;
	if (data_hex.nibble.n_4 < 0xA) {
		buffer[0] = data_hex.nibble.n_4 + 0x30;
	}
	else buffer[0] = data_hex.nibble.n_4 + 0x37;

	if (data_hex.nibble.n_3 < 0xA) { 
		buffer[1] = data_hex.nibble.n_3 + 0x30;
	}
	else buffer[1] = data_hex.nibble.n_3 + 0x37;

	if (data_hex.nibble.n_2 < 0xA) {
		buffer[2] = data_hex.nibble.n_2 + 0x30;
	}
	else buffer[2] = data_hex.nibble.n_2 + 0x37;
	if (data_hex.nibble.n_1 < 0xA) { 
		buffer[3] = data_hex.nibble.n_1 + 0x30; 
	}
	else buffer[3] = data_hex.nibble.n_1 + 0x37;
}

uint16_t Giro_escala(uint8_t escala_atual, volatile int16_t dado_giro, uint8_t giro) {
	uint8_t contagem;
	uint16_t ret;
	switch (giro) {
	case giro_1:
		contagem = contagem_giro_1;
		break;
	case giro_2:
		contagem = contagem_giro_2;
		break;
	case giro_3:
		contagem = contagem_giro_3;
		break; 
	case giro_4:
		contagem = contagem_giro_4;
		break;
	default:
		break;
	}
	dado_giro = ((int16_t)(dado_giro << 2)) / 4;
	if (dado_giro < 0) {
		dado_giro = 0 - dado_giro;
	}
	if (dado_giro > Hi_limit_14bit) {
		if (escala_atual == 0) {
			ret = (0x3802 | 0x8000); //+-160 graus por segundo
		}
		else if (escala_atual == 1) {
			ret = (0x3804 | 0x8000);  //+-320 graus por segundo
		}
		else ret = 0x0000;
	}
	else if (dado_giro < Low_limit_14bit) {
		if (escala_atual != 0) {
			contagem++;
			if (contagem >= giro_delay_to_down_scale) {
				if (escala_atual == 1) {
					escala_atual = 0;
					contagem = 0;
					ret = (0x3801 | 0x8000); //+-160 graus por segundo
				}
				else if (escala_atual == 2) {
					escala_atual = 1;
					contagem = 0;
					ret = (0x3802 | 0x8000);  //+-320 graus por segundo
				}
			}
			else ret = 0x0000;
		}
		else ret = 0x0000;
	}
	switch (giro) {
	case giro_1:
		contagem_giro_1 = contagem;
		break;
	case giro_2:
		contagem_giro_2 = contagem;
		break;
	case giro_3:
		contagem_giro_3 = contagem;
		break;
	case giro_4:
		contagem_giro_4 = contagem;
		break;
	default:
		break;
	}
	return ret;
}

uint8_t Acel_escala(uint8_t escala_atual, volatile int16_t x, volatile int16_t y, volatile int16_t z, uint8_t lis) {
	uint8_t contagem;
	int16_t dado_lis;
	uint16_t ret;
	switch (lis) {
	case lis_1:
		contagem = contagem_lis_1;
		break;
	case lis_2:
		contagem = contagem_lis_2;
		break;
	case lis_3:
		contagem = contagem_lis_3;
		break;
	case lis_4:
		contagem = contagem_lis_4;
		break;
	default:
		break;
	}
	if (x < 0) {
		x = 0 - x;
	}
	if (y < 0) {
		y = 0 - y;
	}
	if (z < 0) {
		z = 0 - z;
	}
	if (x >= y) {
		if (x >= z) {
			dado_lis = x;
		}
		else dado_lis = z;
	}
	else {
		if (y >= z) {
			dado_lis = y;
		}
		else dado_lis = z;
	}

	if (dado_lis > Hi_limit_16bit) {
		contagem = 0;
		if (escala_atual == 0) {
			ret = 0xD0; //+-12 g
		}
		else if (escala_atual == 1) {
			ret = 0xF0;  //+-24 g
		}
		else ret = 0x00;
	}
	else if (dado_lis < Low_limit_16bit) {
		if (escala_atual != 0) {
			contagem++;
			if (contagem >= giro_delay_to_down_scale) {
				if (escala_atual == 1) {
					escala_atual = 0;
					contagem = 0;
					ret = 0xC0; //+-6 g
				}
				else if (escala_atual == 2) {
					escala_atual = 1;
					contagem = 0;
					ret = 0xD0;  //+-12 g
				}
			}
			else ret = 0x00;
		}
		else ret = 0x00;
	}
	else {
		contagem = 0;
		ret = 0x00;
	}
	switch (lis) {
	case lis_1:
		contagem_lis_1 = contagem;
		break;
	case lis_2:
		contagem_lis_2 = contagem;
		break;
	case lis_3:
		contagem_lis_3 = contagem;
		break;
	case lis_4:
		contagem_lis_4 = contagem;
		break;
	default:
		break;
	}
	return ret;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	contador.word++;
	sensor.str_sensor.chk_sum.data = 0;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{

}

void calc_media(void) {
	//acel
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_1_x.data;
	}
	sensor.str_sensor.lis_1_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_1_y.data;
	}
	sensor.str_sensor.lis_1_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_1_z.data;
	}
	sensor.str_sensor.lis_1_z.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_2_x.data;
	}
	sensor.str_sensor.lis_2_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_2_y.data;
	}
	sensor.str_sensor.lis_2_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_2_z.data;
	}
	sensor.str_sensor.lis_2_z.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_3_x.data;
	}
	sensor.str_sensor.lis_3_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_3_y.data;
	}
	sensor.str_sensor.lis_3_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_3_z.data;
	}
	sensor.str_sensor.lis_3_z.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_4_x.data;
	}
	sensor.str_sensor.lis_4_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_4_y.data;
	}
	sensor.str_sensor.lis_4_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].lis_4_z.data;
	}
	sensor.str_sensor.lis_4_z.data = (int16_t)(soma >> 3);
	soma = 0;
	//tmp`s
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].tmp_1.data;
	}
	sensor.str_sensor.tmp_1.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].tmp_2.data;
	}
	sensor.str_sensor.tmp_2.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].tmp_3.data;
	}
	sensor.str_sensor.tmp_3.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].tmp_4.data;
	}
	sensor.str_sensor.tmp_4.data = (int16_t)(soma >> 3);
	soma = 0;
	//mags
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_1_x.data;
	}
	sensor.str_sensor.hmc_1_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_1_y.data;
	}
	sensor.str_sensor.hmc_1_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_1_z.data;
	}
	sensor.str_sensor.hmc_1_z.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_2_x.data;
	}
	sensor.str_sensor.hmc_2_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_2_y.data;
	}
	sensor.str_sensor.hmc_2_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_2_z.data;
	}
	sensor.str_sensor.hmc_2_z.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_3_x.data;
	}
	sensor.str_sensor.hmc_3_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_3_y.data;
	}
	sensor.str_sensor.hmc_3_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_3_z.data;
	}
	sensor.str_sensor.hmc_3_z.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_4_x.data;
	}
	sensor.str_sensor.hmc_4_x.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_4_y.data;
	}
	sensor.str_sensor.hmc_4_y.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].hmc_4_z.data;
	}
	sensor.str_sensor.hmc_4_z.data = (int16_t)(soma >> 3);
	soma = 0;
	//giros
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_1_i.data;
	}
	sensor.str_sensor.giro_1_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_2_i.data;
	}
	sensor.str_sensor.giro_2_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_3_i.data;
	}
	sensor.str_sensor.giro_3_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_4_i.data;
	}
	sensor.str_sensor.giro_4_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_1_i.data;
	}
	sensor.str_sensor.temp_1_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_2_i.data;
	}
	sensor.str_sensor.temp_2_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_3_i.data;
	}
	sensor.str_sensor.temp_3_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_4_i.data;
	}
	sensor.str_sensor.temp_4_i.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_1_f.data;
	}
	sensor.str_sensor.giro_1_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_2_f.data;
	}
	sensor.str_sensor.giro_2_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_3_f.data;
	}
	sensor.str_sensor.giro_3_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].giro_4_f.data;
	}
	sensor.str_sensor.giro_4_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_1_f.data;
	}
	sensor.str_sensor.temp_1_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_2_f.data;
	}
	sensor.str_sensor.temp_2_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_3_f.data;
	}
	sensor.str_sensor.temp_3_f.data = (int16_t)(soma >> 3);
	soma = 0;
	for (uint8_t h = 0; h < 8; h++)
	{
		soma += media[h].temp_4_f.data;
	}
	sensor.str_sensor.temp_4_f.data = (int16_t)(soma >> 3);
	soma = 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
