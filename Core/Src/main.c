/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <alloca.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BITS_PER_BIT 3
#define NUM_RESET_BYTES 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void hsv_to_rgb(double h, double s, double l, uint8_t *r, uint8_t *g, uint8_t *b);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t *write_byte(uint8_t byte, uint8_t *buffer) {
  int expanded = 0;
  for (int i=7; i>=0; i--) {
    expanded <<= 3;

    if (byte & (1 << i)) {
      expanded |= 0b110;
    }
    else {
      expanded |= 0b100;
    }
  }

  *buffer = expanded >> 16;
  *(buffer + 1) = (expanded >> 8) & 0xFF;
  *(buffer + 2) = expanded & 0xFF;
  return buffer + 3;
}


// returns the size of memory required by the framebuffer. the framebuffer
// expects you to allocate at least this amount of memory, and then call
// new_framebuffer with it.
size_t get_framebuffer_size(size_t num_leds) {
  size_t spi_buffer_size = num_leds * 3 * SPI_BITS_PER_BIT + NUM_RESET_BYTES;
  return spi_buffer_size;
}

// make a new framebuffer. backing_memory must be a pointer to a usable
// region of memory at least get_framebuffer_size(num_leds) bytes big.
FrameBuffer new_framebuffer(size_t num_leds, void *backing_memory) {

  // initialize reset bytes
  size_t spi_buffer_size = get_framebuffer_size(num_leds);
  memset(backing_memory + spi_buffer_size - NUM_RESET_BYTES, 0, NUM_RESET_BYTES);

  FrameBuffer fb = {
    .spi_buffer = (uint8_t *) backing_memory,
    .spi_buffer_size = spi_buffer_size,
    .write_ptr = (uint8_t *) backing_memory,
    .num_leds = num_leds,
  };

  return fb;
}

// resets the position of the write index, allowing you to
// fill the framebuffer with new values
void clear_framebuffer(FrameBuffer *fb) {
  fb->write_ptr = fb->spi_buffer;
}

// sets the color of the next led in the chain, up to fb.num_leds.
// returns -1 if all leds have been set already, 0 otherwise.
int framebuffer_push_rgb(FrameBuffer *fb, uint8_t red, uint8_t green, uint8_t blue) {

  // // reserve 100 bytes for resetting
  // reserve 100 bytes for resetting
  size_t writable_spi_buffer_size = fb->spi_buffer_size - NUM_RESET_BYTES;
  if (fb->write_ptr >= (fb->spi_buffer + writable_spi_buffer_size)) {
    return -1;
  }

  fb->write_ptr = write_byte(green, fb->write_ptr);
  fb->write_ptr = write_byte(red, fb->write_ptr);
  fb->write_ptr = write_byte(blue, fb->write_ptr);

  return 0;
}

int framebuffer_push_hsv(FrameBuffer *fb, uint8_t h, uint8_t s, uint8_t v) {
    /**
     * Convert an HSV color value to RGB. Conversion formula
     * adapted from http://en.wikipedia.org/wiki/HSV_color_space.
     * Assumes h, s, and v are contained in the set [0, 1] and
     * returns r, g, and b in the set [0, 255].
     *
     * based on: https://axonflux.com/handy-rgb-to-hsl-and-rgb-to-hsv-color-model-c
     */

    uint16_t h6 = h * 6;
    uint8_t i = h6 >> 8; // integer part
    uint8_t f = h6 & 0xFF; // fractional part

    uint8_t p = v * (256 - s) >> 8;
    uint8_t q = v * (256 - ((f * s) >> 8)) >> 8;
    uint8_t t = v * (256 - (((256 - f) * s) >> 8)) >> 8;

    double select[] = {v, q, p, p, t, v};
    uint8_t r = select[(int) i % 6];
    uint8_t g = select[((int) i + 2) % 6];
    uint8_t b = select[((int) i + 4) % 6];

    return framebuffer_push_rgb(fb, r, g, b);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  size_t num_leds = 100;
  size_t backing_buffer_size = get_framebuffer_size(num_leds);
  void *backing_buffer = alloca(backing_buffer_size);
  FrameBuffer framebuffer = new_framebuffer(num_leds, backing_buffer);

  void *renderer_data = init_renderer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  render_frame(renderer_data, &framebuffer, HAL_GetTick());
  while (1)
  {

    if (HAL_SPI_Transmit_DMA(&hspi1, framebuffer.spi_buffer, framebuffer.spi_buffer_size) != HAL_OK) {
      Error_Handler();
    }

    render_frame(renderer_data, &framebuffer, HAL_GetTick());

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
