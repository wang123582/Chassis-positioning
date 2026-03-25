#include "main.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "AS5048.h"
#include "IM_TEST.h"
#include "YIS130.h"

#include <stdio.h>
#include <string.h>

int add = 0;
int times = 0;

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

typedef struct struct_message
{
  uint8_t header;
  uint8_t parity;
  uint8_t data[6];
  uint8_t footer;
} DataPacket;

DataPacket DataRe;
uint8_t USART_FLAG = 0;

uint8_t USART1_RX_BUF[100];
uint16_t USART1_RX_STA = 0;
uint8_t aRxBuffer1[1];
UART_HandleTypeDef UART1_Handler;

uint8_t rcv_buf[64] = {0};
int rcv_err = 3;
char mpu_buff[128];
uint16_t rxclear = 0;
int rcv_flag = 0;
int rst_temp = 0;

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, rcv_buf, 8);

  AS5048_init(1, &hspi1, GPIOA, GPIO_PIN_4);
  AS5048_init(2, &hspi2, GPIOB, GPIO_PIN_12);

  mpu_data[0].cali = 1;
  mpu_data[0].vel[0] = 0;
  mpu_data[0].vel[1] = 0;
  mpu_data[0].REAL_YAW_SET = 0;
  mpu_data[0].REAL_YAW_MARK = 0;

  can_filter_init();
  IM_TEST_initialize();

  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Base_Start_IT(&htim11);

  while (1)
  {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Rcv_IdleCallback(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    HAL_UART_DMAStop(&huart1);
    rcv_flag = 1;
  }
}

int Rcv_DealData(void)
{
  if (1 == rcv_flag)
  {
    if (0x0F == rcv_buf[0] && 0xAA == rcv_buf[7])
    {
      DATARELOAD(rcv_buf);
    }
    else if (0xBB == rcv_buf[0] && 0xCC == rcv_buf[7])
    {
      HAL_GPIO_WritePin(RST_CTRL_GPIO_Port, RST_CTRL_Pin, GPIO_PIN_SET);
      HAL_Delay(500);
      HAL_GPIO_WritePin(RST_CTRL_GPIO_Port, RST_CTRL_Pin, GPIO_PIN_RESET);
      DATARELOAD(rcv_buf);
    }
    for (int i = 0; i < 8; i++)
    {
      rcv_buf[i] = 0;
    }

    rcv_flag = 0;
    HAL_UART_Receive_DMA(&huart1, rcv_buf, 8);
    return 0;
  }

  return -1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&htim14))
  {
  }

  if (htim == (&htim13))
  {
  }

  if (htim == (&htim11))
  {
    if (mpu_data[0].cali == 1)
    {
      if (times >= 500)
      {
        add++;
        AS5048_getREGValue(1);
        AS5048_dataUpdate(1);
        AS5048_getREGValue(2);
        AS5048_dataUpdate(2);

        mpu_data[0].REAL_YAW = mpu_data[0].YAW_ANGLE;

        rtU.W1 = -AS5048s[1].delta_dis;
        rtU.W2 = AS5048s[0].delta_dis;
        rtU.DEG = mpu_data[0].REAL_YAW;

        mpu_data[0].Y_tt += rtY.YOUT;
        mpu_data[0].X_tt += rtY.XOUT;
        mpu_data[0].REAL_Y = mpu_data[0].Y_tt * 0.014373f;
        mpu_data[0].REAL_X = mpu_data[0].X_tt * 0.014373f;

        Rcv_DealData();

        if (add >= 50)
        {
          memset(mpu_buff, 0, sizeof(mpu_buff));
          int mpu_len = sprintf(mpu_buff,
                                "bc a1=%.2fdeg a2=%.2fdeg d1=%d d2=%d\r\n",
                                AS5048s[0].angle * 360.0f / 16384.0f,
                                AS5048s[1].angle * 360.0f / 16384.0f,
                                AS5048s[0].delta_dis,
                                AS5048s[1].delta_dis);
          HAL_UART_Transmit_DMA(&huart1, (uint8_t *)mpu_buff, mpu_len);
          add = 0;
        }
      }
      else
      {
        AS5048_getREGValue(1);
        AS5048_dataUpdate(1);
        AS5048_getREGValue(2);
        AS5048_dataUpdate(2);
        mpu_data[0].vel[0] = 0;
        mpu_data[0].vel[1] = 0;
        times++;
      }

      IM_TEST_step();
    }
  }
}

void ClearUARTErrors(USART_TypeDef *USARTx)
{
  if (USARTx->SR & USART_SR_PE)
  {
    (void)USARTx->DR;
  }
  if (USARTx->SR & USART_SR_FE)
  {
    (void)USARTx->DR;
  }
  if (USARTx->SR & USART_SR_NE)
  {
    (void)USARTx->DR;
  }
  if (USARTx->SR & USART_SR_ORE)
  {
    (void)USARTx->DR;
  }

  USARTx->CR1 |= USART_CR1_UE;
  USARTx->CR3 |= USART_CR3_EIE;
#ifdef USART_CR3_RXFTIE
  USARTx->CR3 |= USART_CR3_RXFTIE;
#endif
  HAL_UART_Receive_DMA(&huart1, rcv_buf, 8);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif