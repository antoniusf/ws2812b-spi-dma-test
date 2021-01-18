/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// A framebuffer structure for displaying data on ws2812b leds.
// Creating a new framebuffer is a two-step process, since the framebuffer
// needs internal memory but doesn't want to allocate it itself. Therefore,
// you must first request the amount of memory required by the framebuffer
// with a call to get_framebuffer_size(num_leds), allocate this memory however
// you wish, and then hand it to new_framebuffer(num_leds, memory) to get your
// framebuffer. Note that num_leds must be the same in both calls (obviously).
//
// Operation:
//
// The framebuffer maintains an internal write pointer, so that you can set
// all LEDs in sequence with calls to framebuffer_push_{rgb, hsl}(). To reset
// this write pointer, so that the next call to framebuffer_push will start with
// the first LED again, use framebuffer_clear().
//
// The framebuffer's internal memory (spi_buffer) contains data that can be sent
// directly via the SPI interface, provided that the frequencies are set up correctly.
// spi_buffer_size gives the size of this buffer, in bytes.
//
// Note that framebuffer_clear() *only* resets the write pointer, and does not actually
// clear the internal buffer. This means that the data will still be available
// for sending out via SPI even after framebuffer_clear() has been called.
typedef struct FrameBuffer {
  uint8_t *spi_buffer;
  size_t spi_buffer_size;
  uint8_t *write_ptr;
  size_t num_leds;
} FrameBuffer;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// renderer interface:
//
// the renderer is expected to provide two functions: init_renderer is meant
// to allow the renderer to set up internal state. It may return a pointer to
// this memory, and this pointer will later be provided to all other calls.
// If the renderer does not need to keep state, it may return NULL.
//
// render_frame is called when a new frame needs to be rendered. the renderer
// is provided with its internal state, a time value in milliseconds, starting
// at an arbitrary offset, and a framebuffer instance to write to. note that
// the framebuffer may have to be cleared before writing to it.
void *init_renderer(void);
void render_frame(void *data, FrameBuffer *fb, uint32_t time);

void clear_framebuffer(FrameBuffer *fb);
int framebuffer_push_rgb(FrameBuffer *fb, uint8_t red, uint8_t green, uint8_t blue);
int framebuffer_push_hsv(FrameBuffer *fb, double h, double s, double l);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
