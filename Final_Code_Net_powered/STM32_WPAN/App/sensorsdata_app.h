/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service1_app.h
  * @author  MCD Application Team
  * @brief   Header for service1_app.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSORSDATA_APP_H
#define SENSORSDATA_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  SENSORSDATA_CONN_HANDLE_EVT,
  SENSORSDATA_DISCON_HANDLE_EVT,

  /* USER CODE BEGIN Service1_OpcodeNotificationEvt_t */

  /* USER CODE END Service1_OpcodeNotificationEvt_t */

  SENSORSDATA_LAST_EVT,
} SENSORSDATA_APP_OpcodeNotificationEvt_t;

typedef struct
{
  SENSORSDATA_APP_OpcodeNotificationEvt_t          EvtOpcode;
  uint16_t                                 ConnectionHandle;

  /* USER CODE BEGIN SENSORSDATA_APP_ConnHandleNotEvt_t */

  /* USER CODE END SENSORSDATA_APP_ConnHandleNotEvt_t */
} SENSORSDATA_APP_ConnHandleNotEvt_t;
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void SENSORSDATA_APP_Init(void);
void SENSORSDATA_APP_EvtRx(SENSORSDATA_APP_ConnHandleNotEvt_t *p_Notification);
/* USER CODE BEGIN EFP */
void sendData(float temperature, float humidity, uint16_t co2_data, uint16_t nh3_data);
void sendData2(uint16_t temperature, uint16_t humidity, uint16_t co2_data, uint16_t nh3_data);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*SENSORSDATA_APP_H */
