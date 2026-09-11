/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"
#include "platform.h"
#include "fctasks.h"
#include "fcapi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* A task-creation failure happens before normal logging is available. Encode
 * the failed stage as N short blue flashes followed by one long green flash. */
volatile UINT g_startup_fault_stage __attribute__((used, externally_visible)) = 0U;
volatile UINT g_thread_stack_error_count __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_thread_stack_error_thread __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_thread_stack_error_start __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_thread_stack_error_end __attribute__((used, externally_visible)) = 0U;

static void startup_fault(UINT stage);

static void thread_stack_error_handler(TX_THREAD *thread_ptr)
{
  g_thread_stack_error_count++;
  g_thread_stack_error_thread = (uint32_t)(uintptr_t)thread_ptr;
  if (thread_ptr != TX_NULL)
  {
    g_thread_stack_error_start =
        (uint32_t)(uintptr_t)thread_ptr->tx_thread_stack_start;
    g_thread_stack_error_end =
        (uint32_t)(uintptr_t)thread_ptr->tx_thread_stack_end;
  }
  startup_fault(7U);
}

static void startup_fault(UINT stage)
{
  g_startup_fault_stage = stage;
  __disable_irq();
  while (1)
  {
    blink(Blue, false, (fi32)stage);
    blink(Green, true, 1);
  }
}

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  (void)tx_thread_stack_error_notify(thread_stack_error_handler);

#if defined(TELEMETRY_ENABLED) || defined(FAKESTATION)
  ret = create_telemetry_task(memory_ptr);
  if (ret != TX_SUCCESS)
  {
    startup_fault(1U);
  }
#endif

  ret = create_recovery_task(memory_ptr);
  if (ret != TX_SUCCESS)
  {
    startup_fault(2U);
  }

  ret = create_dma_task(memory_ptr);
  if (ret != TX_SUCCESS)
  {
    startup_fault(3U);
  }

  ret = create_evaluation_task(memory_ptr);
  if (ret != TX_SUCCESS)
  {
    startup_fault(4U);
  }

  ret = create_distribution_task(memory_ptr);
  if (ret != TX_SUCCESS)
  {
    startup_fault(5U);
  }

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */
  
  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  startup_fault(6U);

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
