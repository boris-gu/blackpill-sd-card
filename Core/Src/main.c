/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFF_TX_SIZE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for sd_task */
osThreadId_t sd_taskHandle;
const osThreadAttr_t sd_task_attributes = {
  .name = "sd_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);
void start_sd_task(void *argument);

/* USER CODE BEGIN PFP */
HAL_StatusTypeDef uart_tx_dma(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef change_sd_dma_direction(uint32_t direction);
uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sd_task */
  sd_taskHandle = osThreadNew(start_sd_task, NULL, &sd_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef uart_tx_dma(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size) {
  while (HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX ||
         HAL_UART_GetState(huart) == HAL_UART_STATE_BUSY_TX_RX) {
  }
  return HAL_UART_Transmit_DMA(huart, pData, Size);
}

HAL_StatusTypeDef change_sd_dma_direction(uint32_t direction) {
  HAL_StatusTypeDef ret = HAL_OK;
  if (HAL_DMA_GetState(&hdma_sdio) == HAL_DMA_STATE_BUSY){
    ret = HAL_DMA_Abort(&hdma_sdio);
  }
  
  if (ret == HAL_OK) {
    ret = HAL_DMA_DeInit(&hdma_sdio);
    if (ret == HAL_OK) {
      hdma_sdio.Init.Direction = direction;
      ret = HAL_DMA_Init(&hdma_sdio);
    }
  }
  return ret;
}

uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks) {
  // Меняем направление DMA на периферия->память
  if (change_sd_dma_direction(DMA_PERIPH_TO_MEMORY) == HAL_OK) {
    if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t *)pData, ReadAddr, NumOfBlocks) == HAL_OK) {
      return MSD_OK;
    }
  }
  return MSD_ERROR;
}

uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks) {
  // Меняем направление DMA на память->периферия
  if (change_sd_dma_direction(DMA_MEMORY_TO_PERIPH) == HAL_OK) {
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)pData, WriteAddr, NumOfBlocks) == HAL_OK) {
      return MSD_OK;
    }
  }
  return MSD_ERROR;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_sd_task */
/**
  * @brief  Function implementing the sd_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_sd_task */
void start_sd_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t sd_cd_state_prev = GPIO_PIN_SET;
  uint8_t sd_cd_state = GPIO_PIN_SET;
  uint8_t mount_state = FR_NOT_READY;
  char buff_tx[BUFF_TX_SIZE];

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  TickType_t ticks_period = pdMS_TO_TICKS(200);
  TickType_t current_ticks = xTaskGetTickCount();
  for (;;) {
    sd_cd_state = HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin);
    // Карта изменила свое состояние (вставлена/вытащена)
    if (sd_cd_state != sd_cd_state_prev) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, sd_cd_state);
      // Карта была вставлена
      if (sd_cd_state == GPIO_PIN_RESET) {
        snprintf(buff_tx, BUFF_TX_SIZE, "\n\nSD card is inserted\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
        /*
         * Повторная инициализация
         *
         * XXX: Раньше тут была проверка
         *      if (hsd.State == HAL_SD_STATE_RESET)
         *      но она не работала, реинициализировалась карта даже во время первого монтирования,
         *      хотя, по идее, так быть не должно.
         * 
         *      Во время инициализации SD вызывается следующий каскад функций:
         *      SD_initialize
         *      | BSP_SD_Init
         *        | HAL_SD_Init
         *          | HAL_SD_MspInit
         *          | HAL_SD_InitCard
         *      Но где конкретно FatFS вызывает SD_initialize выяснить не удалось.
         *      Поэтому, когда карта вытаскивается, я размонтирую ее и деинициализирую,
         *      а при повторной вставке карты инициализирую заново и монтирую.
         */        
        HAL_Delay(1000); // Чтобы пользователь плотно вставил карту
        snprintf(buff_tx, BUFF_TX_SIZE, "ReInit: ");
        HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
        // Для инициализации карты необходимо включить режим 1bit
        hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
        mount_state = HAL_SD_Init(&hsd);
        if (mount_state == FR_OK) {
          snprintf(buff_tx, BUFF_TX_SIZE, "OK\n");
          HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
          /*
           * Включение режима 4bit
           * XXX: Почему-то работает не на всех картах
           *      При необходимости закомментировать и
           *      поменять режим в CubeMX
           * 
           *      Почему-то просто закомментировать недостаточно, надо лезть в CubeMX.
           *      Если, в первый раз после включения stm, вставить карту,
           *      которая не работает в режиме 4bit, то она не смонтируется, потом все нормально.
           *      Как будто есть еще где-то кусок кода, который переводит карту в режим 4bit,
           *      но только во время первого монтирования
           */
          mount_state = HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);
          snprintf(buff_tx, BUFF_TX_SIZE, "4BIT: %d\n", mount_state);
          HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);

          // Монтируем
          snprintf(buff_tx, BUFF_TX_SIZE, "Mount: ");
          HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
          mount_state = f_mount(&SDFatFS, SDPath, 1);
          if (mount_state == FR_OK) {
            snprintf(buff_tx, BUFF_TX_SIZE, "OK\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);

            // Вывод данных о карте
            sniprintf(buff_tx, BUFF_TX_SIZE, "---\nCard Type:    %lu\n", hsd.SdCard.CardType);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            sniprintf(buff_tx, BUFF_TX_SIZE, "Card Version: %lu\n", hsd.SdCard.CardVersion);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            sniprintf(buff_tx, BUFF_TX_SIZE, "Class:        %lu\n", hsd.SdCard.Class);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            sniprintf(buff_tx, BUFF_TX_SIZE, "Block size:   %lu\n", hsd.SdCard.BlockSize);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            sniprintf(buff_tx, BUFF_TX_SIZE, "Block num:    %lu\n", hsd.SdCard.BlockNbr);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            /*
             * XXX: Отключена возможность вывода чисел с плавающей точкой и uint64_t
             *      Для первого
             *      https://stackoverflow.com/questions/28334435/stm32-printf-float-variable
             *      Для второго
             *      https://community.st.com/t5/stm32cubeide-mcus/wrong-result-when-printing-a-int64-t-value-using-stm32cubeide/td-p/148913
             */
            sniprintf(buff_tx, BUFF_TX_SIZE, "Card size:    %d MB\n",
                      (uint16_t)((uint64_t)hsd.SdCard.BlockSize * hsd.SdCard.BlockNbr / 1000000));
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);

            // Запись в файл
            uint8_t fil_res;
            sniprintf(buff_tx, BUFF_TX_SIZE, "---\nOpen file: ");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            /*
             * По хорошему тут тоже сделать проверки, получилось или нет, но было лень
             * Поэтому я просто вывожу возвращаемое значение
             */
            fil_res = f_open(&SDFile, "hello.txt", FA_WRITE | FA_CREATE_ALWAYS);
            sniprintf(buff_tx, BUFF_TX_SIZE, "%d\n", fil_res);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);

            sniprintf(buff_tx, BUFF_TX_SIZE, "Write: ");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            fil_res = f_write(&SDFile, "Hello, World!", 13, NULL);
            sniprintf(buff_tx, BUFF_TX_SIZE, "%d\n", fil_res);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);

            sniprintf(buff_tx, BUFF_TX_SIZE, "Close file: ");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            fil_res = f_close(&SDFile);
            sniprintf(buff_tx, BUFF_TX_SIZE, "%d\n", fil_res);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);

            sniprintf(buff_tx, BUFF_TX_SIZE, "END\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
          } else {
            // Не удалось смонтировать
            snprintf(buff_tx, BUFF_TX_SIZE, "ERROR %d\n", mount_state);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            // TODO: Наверное, тут надо размонтировать
          }
        } else {
          // Не удалось инициализировать
          snprintf(buff_tx, BUFF_TX_SIZE, "ERROR %d\n", mount_state);
          HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
        }
      } else {
        // Карта была вытащена
        snprintf(buff_tx, BUFF_TX_SIZE, "\n\nSD card is removed\n");
        HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
        if (mount_state == FR_OK) {
          // Размонтируем, если карта была смонтирована
          snprintf(buff_tx, BUFF_TX_SIZE, "Unmount: ");
          HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
          mount_state = f_mount(NULL, SDPath, 0);
          if (mount_state == FR_OK) {
            // Деинициализация
            snprintf(buff_tx, BUFF_TX_SIZE, "OK\n");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            snprintf(buff_tx, BUFF_TX_SIZE, "DeInit: ");
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            if (HAL_SD_DeInit(&hsd) == HAL_OK) {
              snprintf(buff_tx, BUFF_TX_SIZE, "OK\n");
            } else {
              snprintf(buff_tx, BUFF_TX_SIZE, "ERROR %d\n", mount_state);
            }
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
            mount_state = FR_NOT_READY;
          } else {
            snprintf(buff_tx, BUFF_TX_SIZE, "ERROR %d\n", mount_state);
            HAL_UART_Transmit(&huart1, (uint8_t *)buff_tx, strlen(buff_tx), 20);
          }
        }
      }
      sd_cd_state_prev = sd_cd_state;
    }
    vTaskDelayUntil(&current_ticks, ticks_period);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
