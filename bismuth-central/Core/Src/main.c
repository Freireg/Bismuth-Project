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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
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
CAN_HandleTypeDef hcan1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

uint8_t						        ch1TxData[8], ch1RxData[8];
CAN_RxHeaderTypeDef			  ch1RxHeader;
CAN_TxHeaderTypeDef			  ch1TxHeader;
uint32_t						      ch1TxMailbox;

TaskHandle_t canManagerHandle = NULL;
TaskHandle_t task1Handle = NULL;
TaskHandle_t task2Handle = NULL;
xQueueHandle canMessageQueue = NULL;
xQueueHandle task1InputQueue = NULL, task1OutputQueue = NULL;
xQueueHandle task2InputQueue = NULL, task2OutputQueue = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void entryCanManager(void *argument);
void entryTask1(void *argument);
void entryTask2(void *argument);
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
  MX_CAN1_Init();
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  BaseType_t xReturned;
  xReturned = xTaskCreate(entryTask1, "task1", 128*2, NULL, tskIDLE_PRIORITY, &task1Handle);
  if (xReturned != pdPASS) {
    /* Task creation failed */
    Error_Handler();
  }
  xReturned = xTaskCreate(entryTask2, "task2", 128, NULL, tskIDLE_PRIORITY, &task2Handle);
  if (xReturned != pdPASS) {
    /* Task creation failed */
    Error_Handler();
  }
  xReturned = xTaskCreate(entryCanManager, "canManager", 128*2, NULL, tskIDLE_PRIORITY, &canManagerHandle);
  if (xReturned != pdPASS) {
    /* Task creation failed */
    Error_Handler();
  }
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canFilterConfig;

  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank = 3;  // which filter bank to use from the assigned ones
  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canFilterConfig.FilterIdHigh = 0xFFF<<16;
  canFilterConfig.FilterIdLow = 0;
  canFilterConfig.FilterMaskIdHigh = 0xFFF<<16;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.SlaveStartFilterBank = 10;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin ORANGE_LED_Pin RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void entryTask1(void *argument) {
	bismuthTaskMessage inputData 	= {0};
	bismuthTaskMessage outputData	= {0};

	union {
		float f;
		uint8_t b[4];
	} distanceRead;


    task1InputQueue = xQueueCreate(8, sizeof(bismuthTaskMessage));
    if (task1InputQueue == NULL) {
      /* Queue creation failed */
      Error_Handler();
    }
    task1OutputQueue = xQueueCreate(8, sizeof(bismuthTaskMessage));
    if (task1OutputQueue == NULL) {
      /* Queue creation failed */
      Error_Handler();
    }

    for(;;) {
      /* Send a Remote Request Frame */
      xQueueSend(task1OutputQueue, &outputData, 0);
      /* Receive CAN Frame from manager */
	  if(xQueueReceive(task1InputQueue, &inputData, portMAX_DELAY) == pdTRUE) {
		// Process the received message
		distanceRead.b[0] = inputData.data[0];
		distanceRead.b[1] = inputData.data[1];
		distanceRead.b[2] = inputData.data[2];
		distanceRead.b[3] = inputData.data[3];

		if (distanceRead.f > 20.0) {
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		}
		else if (distanceRead.f > 15.0) {
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		}
		else if (distanceRead.f > 10.0) {
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		}
		else if (10.0 > distanceRead.f) {
			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		}

	  }
	  vTaskDelay(10);
	 }
}
void entryTask2(void *argument) {
	bismuthTaskMessage inputData 	= {0};
	bismuthTaskMessage outputData	= {0};
	uint8_t systemHalt = 0x00;

	UNUSED(outputData);

	task2InputQueue = xQueueCreate(4, sizeof(bismuthTaskMessage));
  if (task2InputQueue == NULL) {
    /* Queue creation failed */
    Error_Handler();
  }
  task2OutputQueue = xQueueCreate(4, sizeof(bismuthTaskMessage));
  if (task2OutputQueue == NULL) {
    /* Queue creation failed */
    Error_Handler();
  }
  for(;;) {
	  if (xQueueReceive(task2InputQueue, &inputData, portMAX_DELAY) == pdTRUE) {
		  // Process the received message
		  if (!systemHalt) {
		  vTaskSuspend(task1Handle);
		  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		  systemHalt = 0x01;
		  } else {
			  vTaskResume(task1Handle);
			  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
			  systemHalt = 0x00;
		  }
        __asm("nop");
      }
	    vTaskDelay(10);
  }
}
void entryCanManager(void *argument) {

  BaseType_t xTaskWokenByReceive = pdFALSE;
  bismuthCANMessage canMessage;
  bismuthTaskMessage task1Data, task2Data;

  canMessageQueue = xQueueCreate(8, sizeof(bismuthCANMessage));
  if (canMessageQueue == NULL) {
    /* Queue creation failed */
    Error_Handler();
  }
  /*---Dummy Data---*/
  ch1TxHeader.IDE 		= CAN_ID_STD;
  ch1TxHeader.StdId		= 0x446;
  ch1TxHeader.RTR		= CAN_RTR_REMOTE;
  ch1TxHeader.DLC		= 3;

  ch1TxData[0]			= 24;
  ch1TxData[1]			= 96;
  ch1TxData[2]			= 26;
  /*----------------*/

  HAL_CAN_Start(&hcan1);

  while (1)
  {
	/* Enable CAN interrupts notifications */
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		  Error_Handler();
	  }
    /*--- Test routine to trigger peripheral device---*/
//	  if (HAL_CAN_AddTxMessage(&hcan1, &ch1TxHeader, ch1TxData, &ch1TxMailbox) != HAL_OK) {
//		  Error_Handler();
//	  }
    /*---End of test routine---*/

    /* Wait for a CAN frame to be received */
    if (xQueueReceiveFromISR(canMessageQueue, &canMessage, &xTaskWokenByReceive) == pdTRUE) {
      /* Redirect the received CAN message to its specific task queue */
      switch (canMessage.messageHeader.StdId) 
      {
        case 0x224:
//          HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
          __asm("nop");
          xQueueSend(task1InputQueue, (bismuthTaskMessage *)canMessage.data, 0);
          break;
        case 0x103:
//          HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
          __asm("nop");
          xQueueSend(task2InputQueue, (bismuthTaskMessage *)canMessage.data, 0);
          break;
        default:
        	break;
      }
    }

    if (xQueueReceive(task1OutputQueue, &task1Data, 10) == pdTRUE) {
    	  ch1TxHeader.IDE 		= CAN_ID_STD;
    	  ch1TxHeader.StdId		= 0x446;
    	  ch1TxHeader.RTR		= CAN_RTR_REMOTE;
    	  ch1TxHeader.DLC		= 0;

    	  if (HAL_CAN_AddTxMessage(&hcan1, &ch1TxHeader, ch1TxData, &ch1TxMailbox) != HAL_OK) {
    		  Error_Handler();
    	  }
    }
	  vTaskDelay(50);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  BaseType_t xHigherPriorityTaskWoken;
  bismuthCANMessage canMessage;

  /* We have not woken a task at the start of the ISR. */
  xHigherPriorityTaskWoken = pdFALSE;

  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canMessage.messageHeader, canMessage.data) != HAL_OK) {
		Error_Handler();
	}

  xQueueSendFromISR(canMessageQueue, &canMessage, &xHigherPriorityTaskWoken);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  if (htim->Instance == TIM1)
  {
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
#ifdef USE_FULL_ASSERT
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
