/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GlobalVariable.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint16_t ArrayPin[7] = {
		SW_F1_Pin,SW_Left_Pin,SW_Right_Pin,  // GPIOB
		RN_SW_Pin, SW_Up_Pin,SW_Down_Pin, SW_F2_Pin 				// GPIOE
};
#define ConsDebounce				100

#define ConsDebounceS1				1
#define ConsDebounceS2				1
#define ConsDebounceRNA				1
#define ConsDebounceRNB				1
#define ConsDebounceSupplyFB		100

#define ConsDebounceENSF1			3
#define ConsDebounceENSF3			1
#define ConsDebounceENSF4			1
#define ConsDebounceENSF5			100
#define ConsDebounceENSF6			100
#define ConsDebounceENSF7			100
#define ConsDebounceENSF8			100

#define ConsLockMenu 				3000


#define ComboBoxAuto					1
#define ComboBoxStart					2
#define ComboBoxReverseInPhi2			3
#define ComboBoxPaireReverse			4
#define ComboBoxPaireStart				5
#define ComboBoxPaireStop				6
#define ComboBoxReverseInPhi1Start  	7
#define ComboBoxReverseInPhi1Reverse	8
#define ComboBoxReverseInPhi1Complete	9
#define ComboBoxReverseInPhi1Completed	10

#define ComboBoxPaireReverseInPhi11And2 			11
#define ComboBoxPaireStartInPhi1And2				12
#define ComboBoxPaireReverseInPhi11And2Complete 	13
#define ComboBoxPaireReverseInPhi11And2Completed	14

//const uint32_t ArrayPort[7] = {
//								RN_SW_GPIO_Port,SW_Up_GPIO_Port,SW_Right_GPIO_Port
//								SW_Down_GPIO_Port, SW_Left_GPIO_Port, SW_F1_GPIO_Port, SW_F2_GPIO_Port
//};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
char FlagRNSw_G, FlagRNA_G, FlagRNB_G, FlagRNRight, FlagRNLeft, FlagRNNone, FlagStatusRNA_G, FlagStatusRNB_G, FlagRNSWSet = 0,FlagHome_G,FlagStart_G = 0;
char FlagMagneticTrigger_G, RotationStatus_G = Right, FlagSwitchTrigger_G;
unsigned  TimerHome;
int16_t CntRN_G = 1;
char FlagDisplay_G = 1, MaxMenu_G = MainMenu;
uint8_t Phi_G,ArrayPhiBackup_G[5],Temp[5],i,FlagReverse_G;
int8_t NS3To0_G, NS1To2_G, NS0To1_G, NS2To3_G;
int8_t NR0To3_G, NR2To1_G, NR1To0_G, NR3To2_G;
uint16_t CntTimerRN, CntTimerPB;
uint8_t FlagTimerRN, StatusPB, FlagUseKeypad_G = 1;
uint8_t FlagSwUp_G, FlagSwRight_G,FlagSwLeft_G, FlagSwDown_G, FlagSwF1_G, FlagSwF2_G;
uint8_t FlagSupplyFB_G, FlagRPM_G = 0;
//uint32_t CNTRPM_G;
uint16_t CntTimerDebounceS1, CntTimerDebounceS2, CntTimerDebounceStart;
uint8_t StatusGPIOSensor1, StatusGPIOSensor2, StatusSensor1, StatusSensor2;
uint8_t FlagEdgeDetectionS1, FlagEdgeDetectionS2, FlagEdgeDetectionStart;
uint8_t StatusSupplyFB, CntTimerDebounceSupplyFB, FlagEdgeDetectionSupplyFB, StatusGPIOSupplyFB = SET;

uint8_t	CntTimerDebounceENSF1, CntTimerDebounceENSF3, CntTimerDebounceENSF4, CntTimerDebounceENSF5, CntTimerDebounceENSF6, CntTimerDebounceENSF7, CntTimerDebounceENSF8,
CntTimerDebounceRNA, CntTimerDebounceRNB;
uint8_t FlagEdgeDetectionENSF1 ,FlagEdgeDetectionENSF3, FlagEdgeDetectionENSF4, FlagEdgeDetectionENSF5, FlagEdgeDetectionENSF6, FlagEdgeDetectionENSF7,
FlagEdgeDetectionENSF8, FlagEdgeDetectionRNA, FlagEdgeDetectionRNB;
uint8_t StatusENSF1, StatusENSF3, StatusENSF4, StatusENSF5, StatusENSF6, StatusENSF7, StatusENSF8,
StatusRNA, StatusRNB;
uint8_t StatusGPIOENSF1, StatusGPIOENSF2, StatusGPIOENSF3, StatusGPIOENSF4, StatusGPIOENSF5, StatusGPIOENSF6, StatusGPIOENSF7, StatusGPIOENSF8,
StatusGPIORNA, StatusGPIORNB;
uint8_t StatusComboBox_G, PreviousStatusComboBox = ComboBoxStop_G;
uint8_t ComboBoxCntFlagStop, ComboBoxFlagStop;
uint8_t CntTim7, CntMinutes_G, CntHour_G;
uint32_t CntTimeRPM_G;
uint8_t KypadDirection_G;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Keypad (void);
void RNSwitch (void);
void DobbyAlgorithm(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	DobbyAlgorithm();
	if(FlagUseKeypad_G == 0)
		RNSwitch();
	if(FlagUseKeypad_G == 1)
		Keypad();
	if(ComboBoxFlagStop == 1)
	{
		ComboBoxCntFlagStop++;
		HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
		if(ComboBoxCntFlagStop > 150)
		{
			ComboBoxCntFlagStop = 0;
			HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			ComboBoxFlagStop = 0;
		}
	}
	if(FlagExpiration_G == 1)
		if(CntDelayTimer_G % 100000 == 0)
			Cntexpiration_G++;
	//	CNTRPM_G++;
	CntTimeRPM_G++;
	CntTimerPB++;
	CntDelayTimer_G = CntDelayTimer_G +1;
	CntTimerRN++;
	if(CntTimerRN > 150)
		FlagTimerRN = 1;

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(Sensor1_Pin) != RESET)
	{
		if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_RESET)
		{
			Phi_G = 0;

			//      if(RotationStatus_G == Left) //Start in Phi = 3
			//      {
			//        FlagReverse_G = 1;
			//        if(NR0To3_G == -2)
			//        {
			//          NS3To0_G = 0;
			//          NR0To3_G = 0;
			//        }
			//        else
			//          NS3To0_G = +2;
			//      }

			RotationStatus_G = Right;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];
			ArrayPhiBackup_G[0] = 0;

			if(StatusGPIOENSF2 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_RESET);
			}
		}
		if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_RESET) //30-330 Degree
		{
			Phi_G = 3;
			if(RotationStatus_G == Right) //Reverse in Phi = 0
			{
				FlagReverse_G = 1;
				//        if(NS3To0_G == +2)
				//        {
				//          NR0To3_G = 0;
				//          NS3To0_G = 0;
				//        }
				//        else
				NR0To3_G = -2;
			}
			RotationStatus_G = Left;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 3;

			if(StatusGPIOENSF3 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			}

		}
		if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_SET)
		{
			Phi_G = 2;

			if(RotationStatus_G == Left) //Start in Phi = 1
			{
				FlagReverse_G = 1;
				NS1To2_G = +2;
				//        NR2To1_G = 0;
			}

			RotationStatus_G = Right;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 2;

			if(StatusGPIOENSF2 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			}

		}
		if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_SET)
		{
			Phi_G = 1;
			if(RotationStatus_G == Right) //Reverse in Phi = 2
			{
				FlagReverse_G = 1;
				NR2To1_G = -1;
				//        NS1To2_G = 0;
			}
			RotationStatus_G = Left;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 1;
			FlagMagneticTrigger_G = 1;
			FlagRPM_G = 1;
			if(FlagMagneticTrigger_G == 1 && CntProgramLine > 0 && FlagExpireProgram == 0)
			{
				FlagMagneticTrigger_G = 0;
				switch (RotationStatus_G)
				{
				case Left:
					if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 0)
					{
						MagnetCommand(-1); // n = n - 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();
						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 1)
					{
						FlagReverse_G = 0;
						N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
						N_G = N_G - 1;
						MagnetCommand(N_G); // n = n - 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();

						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					break;
				case Right:
					if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 0)
					{
						MagnetCommand(1); // n = n + 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();

						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 1)
					{
						FlagReverse_G = 0;
						N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
						N_G = N_G + 1;
						MagnetCommand(N_G); // n = n + 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();

						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					break;
				default:
					break;
				}
				// if(StatusComboBox_G == 13) // ComboBoxPaireReverseInPhi11And2Complete
				// {
					// BackupCntProgramLine2 = BackupCntProgramLine;
					// MagnetPaireCommand(+1);
				// }
			}

			// ComboBox
			if(StatusGPIOENSF3 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			}

		}
		FlagUpdateTroubleshooting_G = 1;
		FlagEdgeDetectionS1 = 0;
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Sensor1_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(Sensor2_Pin) != RESET)
	{
		if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_RESET)
		{
			Phi_G = 0;

			//      if(RotationStatus_G == Right) //Reverse in Phi = 1
			//      {
			//        FlagReverse_G = 1;
			//        if(NS0To1_G == +2)
			//        {
			//          NR1To0_G = 0;
			//          NS0To1_G = 0;
			//        }
			//        else
			//          NR1To0_G = -2;
			//      }

			RotationStatus_G = Left;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 0;
			if(StatusGPIOENSF3 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			}
		}
		if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_RESET)
		{
			Phi_G = 1;
			if(RotationStatus_G == Left) //Start in Phi = 0
			{
				FlagReverse_G = 1;
				//        if(NR1To0_G == -2)
				//        {
				//          NS0To1_G = 0;
				//          NR1To0_G = 0;
				//        }
				//        else
				NS0To1_G = +2;
			}
			RotationStatus_G = Right;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 1;
			if(StatusGPIOENSF2 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_RESET);
			}
		}

		if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_SET)
		{
			Phi_G = 2;
			if(RotationStatus_G == Right) //Reverse in Phi = 3
			{
				FlagReverse_G = 1;
				NR3To2_G = -2;
				//        NS2To3_G = 0;
			}
			RotationStatus_G = Left;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 2;

			// ComboBox
			if(StatusGPIOENSF3 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			}
		}
		if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port, Sensor2_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port, Sensor1_Pin) == GPIO_PIN_SET)
		{
			Phi_G = 3;
			if(RotationStatus_G == Left) //Start in Phi = 2
			{
				FlagReverse_G = 1;
				NS2To3_G = +1;
				//        NR3To2_G = 0;
			}
			RotationStatus_G = Right;
			memset(Temp, 0, sizeof(Temp));

			for(i = 0; i < 3; i++)
				Temp[i + 1] = ArrayPhiBackup_G[i];

			for(i = 1; i < 4; i++)
				ArrayPhiBackup_G[i] = Temp[i];

			ArrayPhiBackup_G[0] = 3;
			FlagMagneticTrigger_G = 1;
			FlagRPM_G = 1;
			if(FlagMagneticTrigger_G == 1 && CntProgramLine > 0 && FlagExpireProgram == 0)
			{
				FlagMagneticTrigger_G = 0;
				switch (RotationStatus_G)
				{
				case Left:
					if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 0)
					{
						MagnetCommand(-1); // n = n - 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();
						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 1)
					{
						FlagReverse_G = 0;
						N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
						N_G = N_G - 1;
						MagnetCommand(N_G); // n = n - 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();

						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					break;
				case Right:
					if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 0)
					{
						MagnetCommand(1); // n = n + 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();

						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 1)
					{
						FlagReverse_G = 0;
						N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
						N_G = N_G + 1;
						MagnetCommand(N_G); // n = n + 1
						if(FlagMagnetDisplay == 1)
							MagnetDisplay();

						NR0To3_G = 0;
						NR2To1_G = 0;
						NR1To0_G = 0;
						NR3To2_G = 0;
						NS3To0_G = 0;
						NS1To2_G = 0;
						NS0To1_G = 0;
						NS2To3_G = 0;
					}
					break;
				default:
					break;
				}
				// if(StatusComboBox_G == 13) // ComboBoxPaireReverseInPhi11And2Complete
				// {
					// BackupCntProgramLine2 = BackupCntProgramLine;
					// MagnetPaireCommand(+1);
				// }
			}

			// ComboBox
			if(StatusGPIOENSF2 == GPIO_PIN_SET)
			{
				HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			}
		}
		FlagUpdateTroubleshooting_G = 1;
		FlagEdgeDetectionS2 = 0;
	}

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Sensor2_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	DobbyAlgorithm2();
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(ENSF1_Pin) != RESET)
	{
		if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET)
		{
			StatusGPIOENSF1 = GPIO_PIN_SET;
			HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
		}
		if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_RESET)
		{
			StatusGPIOENSF1 = GPIO_PIN_RESET;
		}
	}

	if(__HAL_GPIO_EXTI_GET_IT(ENSF2_Pin) != RESET)
	{
		if(HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET)
		{
			StatusGPIOENSF2 = GPIO_PIN_SET;
		}
		if(HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_RESET)
		{
			StatusGPIOENSF2 = GPIO_PIN_RESET;
		}
	}

	if(__HAL_GPIO_EXTI_GET_IT(ENSF3_Pin) != RESET)
	{
		if(HAL_GPIO_ReadPin(ENSF3_GPIO_Port,ENSF3_Pin) == GPIO_PIN_SET)
		{
			StatusGPIOENSF3 = GPIO_PIN_SET;
			if(RotationStatus_G == Right  && ArrayPhiBackup_G[0] == 1) //Reverse in Phi = 1
			{
				RotationStatus_G = Left;
				NR1To0_G = -1;
				FlagReverse_G = 1;
				FlagMagneticTrigger_G = 1;
				FlagRPM_G = 1;
				if(FlagMagneticTrigger_G == 1 && CntProgramLine > 0 && FlagExpireProgram == 0)
				{
					FlagMagneticTrigger_G = 0;
					switch (RotationStatus_G)
					{
					case Left:
						if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 0)
						{
							MagnetCommand(-1); // n = n - 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();
							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 1)
						{
							FlagReverse_G = 0;
							N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
							N_G = N_G - 1;
							MagnetCommand(N_G); // n = n - 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();

							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						break;
					case Right:
						if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 0)
						{
							MagnetCommand(1); // n = n + 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();

							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 1)
						{
							FlagReverse_G = 0;
							N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
							N_G = N_G + 1;
							MagnetCommand(N_G); // n = n + 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();

							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						break;
					default:
						break;
					}
					// if(StatusComboBox_G == 13) // ComboBoxPaireReverseInPhi11And2Complete
					// {
					// BackupCntProgramLine2 = BackupCntProgramLine;
					// MagnetPaireCommand(+1);
					// }
				}
			}
		}
		if(HAL_GPIO_ReadPin(ENSF3_GPIO_Port,ENSF3_Pin) == GPIO_PIN_RESET)
		{
			StatusGPIOENSF3 = GPIO_PIN_RESET;
			if(RotationStatus_G == Left  && ArrayPhiBackup_G[0] == 3) //Start in Phi = 3
			{
				RotationStatus_G = Right;
				NS3To0_G = +1;
				FlagReverse_G = 1;
				FlagMagneticTrigger_G = 1;
				FlagRPM_G = 1;
				if(FlagMagneticTrigger_G == 1 && CntProgramLine > 0 && FlagExpireProgram == 0)
				{
					FlagMagneticTrigger_G = 0;
					switch (RotationStatus_G)
					{
					case Left:
						if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 0)
						{
							MagnetCommand(-1); // n = n - 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();
							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						if(ArrayPhiBackup_G[0] == 1 && FlagReverse_G == 1)
						{
							FlagReverse_G = 0;
							N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
							N_G = N_G - 1;
							MagnetCommand(N_G); // n = n - 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();

							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						break;
					case Right:
						if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 0)
						{
							MagnetCommand(1); // n = n + 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();

							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						if(ArrayPhiBackup_G[0] == 3 && FlagReverse_G == 1)
						{
							FlagReverse_G = 0;
							N_G = NR0To3_G + NR2To1_G + NR1To0_G + NR3To2_G + NS3To0_G + NS1To2_G + NS0To1_G + NS2To3_G;
							N_G = N_G + 1;
							MagnetCommand(N_G); // n = n + 1
							if(FlagMagnetDisplay == 1)
								MagnetDisplay();

							NR0To3_G = 0;
							NR2To1_G = 0;
							NR1To0_G = 0;
							NR3To2_G = 0;
							NS3To0_G = 0;
							NS1To2_G = 0;
							NS0To1_G = 0;
							NS2To3_G = 0;
						}
						break;
					default:
						break;
					}
					// if(StatusComboBox_G == 13) // ComboBoxPaireReverseInPhi11And2Complete
					// {
						// BackupCntProgramLine2 = BackupCntProgramLine;
						// MagnetPaireCommand(+1);
					// }
				}
			}
		}
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENSF3_Pin);
  HAL_GPIO_EXTI_IRQHandler(ENSF2_Pin);
  HAL_GPIO_EXTI_IRQHandler(ENSF1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
	if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	{
		/* Refresh Error */
		Error_Handler();
	}
	CntTim7++;
	if(CntTim7 == 240)
	{
		CntTim7 = 0;
		CntMinutes_G = CntMinutes_G + 1;
		if(CntMinutes_G == 60)
		{
			CntMinutes_G = 0;
			CntHour_G = CntHour_G + 1;
			if(CntHour_G == 25)
				CntHour_G = 0;
		}
	}
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Keypad()
{
	switch(StatusPB)
	{
	case 0:
		for(char iKeypad = 0; iKeypad < 3; iKeypad ++)
		{
			if(HAL_GPIO_ReadPin(GPIOB,ArrayPin[iKeypad]) == GPIO_PIN_RESET)
			{
				StatusPB = iKeypad + 1;
				CntTimerPB = 0;
			}
		}
		for(char iKeypad = 3; iKeypad < 7; iKeypad ++)
		{
			if(HAL_GPIO_ReadPin(GPIOE,ArrayPin[iKeypad]) == GPIO_PIN_RESET)
			{
				StatusPB = iKeypad + 1;
				CntTimerPB = 0;
			}
		}
		break;
	case 4:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(RN_SW_GPIO_Port,RN_SW_Pin) == GPIO_PIN_RESET)
				StatusPB = 31;
			else
				StatusPB = 0;
		break;
	case 31:
		if(HAL_GPIO_ReadPin(RN_SW_GPIO_Port,RN_SW_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagRNSw_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	case 5:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(SW_Up_GPIO_Port,SW_Up_Pin) == GPIO_PIN_RESET)
				StatusPB = 32;
			else
				StatusPB = 0;
		break;
	case 32:
		if(HAL_GPIO_ReadPin(SW_Up_GPIO_Port,SW_Up_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagSwUp_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	case 3:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(SW_Right_GPIO_Port,SW_Right_Pin) == GPIO_PIN_RESET)
				StatusPB = 33;
			else
				StatusPB = 0;
		break;
	case 33:
		if(HAL_GPIO_ReadPin(SW_Right_GPIO_Port,SW_Right_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagSwRight_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	case 2:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(SW_Left_GPIO_Port,SW_Left_Pin) == GPIO_PIN_RESET)
				StatusPB = 34;
			else
				StatusPB = 0;
		break;
	case 34:
		if(HAL_GPIO_ReadPin(SW_Left_GPIO_Port,SW_Left_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagSwLeft_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	case 6:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(SW_Down_GPIO_Port,SW_Down_Pin) == GPIO_PIN_RESET)
				StatusPB = 35;
			else
				StatusPB = 0;
		break;
	case 35:
		if(HAL_GPIO_ReadPin(SW_Down_GPIO_Port,SW_Down_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagSwDown_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	case 1:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(SW_F1_GPIO_Port,SW_F1_Pin) == GPIO_PIN_RESET)
				StatusPB = 36;
			else
				StatusPB = 0;
		break;
	case 36:
		if(HAL_GPIO_ReadPin(SW_F1_GPIO_Port,SW_F1_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagSwF1_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	case 7:
		if(CntTimerPB > ConsDebounce)
			if(HAL_GPIO_ReadPin(SW_F2_GPIO_Port,SW_F2_Pin) == GPIO_PIN_RESET)
				StatusPB = 37;
			else
				StatusPB = 0;
		break;
	case 37:
		if(CntTimerPB > ConsLockMenu)
		{
			StatusPB = 0;
			FlagSwF2_G = 1;
			CntDelayTimer_G = 0;
			FlagHome_G = 1;
		}
		if(HAL_GPIO_ReadPin(SW_F2_GPIO_Port,SW_F2_Pin) == GPIO_PIN_SET)
		{
			StatusPB = 0;
			FlagSwF2_G = 1;
			FlagDisplay_G = 1;
			CntDelayTimer_G = 0;
		}
		break;
	default:
		break;
	}
}
void RNSwitch(void)
{
	switch(StatusPB)
	{
	case 0:
		if(HAL_GPIO_ReadPin(RN_SW_GPIO_Port, RN_SW_Pin) == GPIO_PIN_RESET)
		{
			StatusPB = 1;
			CntTimerPB = 0;
		}
		break;
	case 1:
		if(CntTimerPB > ConsDebounce)
		{
			if(HAL_GPIO_ReadPin(RN_SW_GPIO_Port,RN_SW_Pin) == GPIO_PIN_RESET)
				StatusPB = 2;
			else
				StatusPB = 0;
		}
		break;
	case 2:
		if(HAL_GPIO_ReadPin(RN_SW_GPIO_Port,RN_SW_Pin) == GPIO_PIN_SET)
		{
			FlagRNSw_G = 1;
			FlagDisplay_G = 1;
			StatusPB = 0;
			CntDelayTimer_G = 0;
		}
		break;
	default:
		break;
	}
}
void DobbyAlgorithm(void)
{

	CntTimerDebounceSupplyFB++;

	if(CntDelayTimer_G > 1000)
	{
		switch(StatusSupplyFB)
		{
		case 0:
			if(HAL_GPIO_ReadPin(SupplyFB_GPIO_Port, SupplyFB_Pin) == GPIO_PIN_RESET)
			{
				StatusSupplyFB = 1;
				CntTimerDebounceSupplyFB = 0;
			}
			if(HAL_GPIO_ReadPin(SupplyFB_GPIO_Port, SupplyFB_Pin) == GPIO_PIN_SET)
			{
				StatusSupplyFB = 2;
				CntTimerDebounceSupplyFB = 0;
			}
			break;
		case 1:
			if(CntTimerDebounceSupplyFB > ConsDebounceSupplyFB)
			{
				if(HAL_GPIO_ReadPin(SupplyFB_GPIO_Port,SupplyFB_Pin) == GPIO_PIN_RESET)
				{
					if(StatusGPIOSupplyFB == GPIO_PIN_SET)
						FlagEdgeDetectionSupplyFB = SET;
					else if(StatusGPIOSupplyFB == GPIO_PIN_RESET)
						FlagEdgeDetectionSupplyFB = RESET;
					StatusGPIOSupplyFB = GPIO_PIN_RESET;
				}
				StatusSupplyFB = 0;
			}
			break;
		case 2:
			if(CntTimerDebounceSupplyFB > ConsDebounceSupplyFB)
			{
				if(HAL_GPIO_ReadPin(SupplyFB_GPIO_Port,SupplyFB_Pin) == GPIO_PIN_SET)
				{
					if(StatusGPIOSupplyFB == GPIO_PIN_SET)
						FlagEdgeDetectionSupplyFB = RESET;
					else if(StatusGPIOSupplyFB == GPIO_PIN_RESET)
						FlagEdgeDetectionSupplyFB = SET;
					StatusGPIOSupplyFB = GPIO_PIN_SET;
				}
				StatusSupplyFB = 0;
			}
			break;
		default:
			break;
		}
		if(FlagEdgeDetectionSupplyFB == SET && StatusGPIOSupplyFB == GPIO_PIN_SET)
		{
			FlagSupplyFB_G = 2;
			FlagEdgeDetectionSupplyFB = RESET;
		}
		if(FlagEdgeDetectionSupplyFB == SET && StatusGPIOSupplyFB == GPIO_PIN_RESET)
		{
			FlagSupplyFB_G = 1;
			FlagEdgeDetectionSupplyFB = RESET;
		}
	}

}
void DobbyAlgorithm2(void)
{
	CntTimerDebounceRNA++;
	CntTimerDebounceRNB++;

	CntTimerDebounceENSF1++;
	CntTimerDebounceENSF3++;
	CntTimerDebounceENSF4++;
	CntTimerDebounceENSF5++;
	CntTimerDebounceENSF6++;
	CntTimerDebounceENSF7++;
	CntTimerDebounceENSF8++;
	/********************	RNA	**************************/
	switch(StatusRNA)
	{
	case 0:
		if(HAL_GPIO_ReadPin(RN_A_GPIO_Port, RN_A_Pin) == GPIO_PIN_RESET)
		{
			StatusRNA = 1;
			CntTimerDebounceRNA = 0;
		}
		if(HAL_GPIO_ReadPin(RN_A_GPIO_Port, RN_A_Pin) == GPIO_PIN_SET)
		{
			StatusRNA = 2;
			CntTimerDebounceRNA = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceRNA > ConsDebounceRNA)
		{
			if(HAL_GPIO_ReadPin(RN_A_GPIO_Port,RN_A_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIORNA == GPIO_PIN_SET)
					FlagEdgeDetectionRNA = SET;
				else if(StatusGPIORNA == GPIO_PIN_RESET)
					FlagEdgeDetectionRNA = RESET;
				StatusGPIORNA = GPIO_PIN_RESET;
			}
			StatusRNA = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceRNA > ConsDebounceRNA)
		{
			if(HAL_GPIO_ReadPin(RN_A_GPIO_Port,RN_A_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIORNA == GPIO_PIN_SET)
					FlagEdgeDetectionRNA = RESET;
				else if(StatusGPIORNA == GPIO_PIN_RESET)
					FlagEdgeDetectionRNA = SET;
				StatusGPIORNA = GPIO_PIN_SET;
			}
			StatusRNA = 0;
		}
		break;
	default:
		break;
	}
	/********************	RNB	**************************/
	switch(StatusRNB)
	{
	case 0:
		if(HAL_GPIO_ReadPin(RN_B_GPIO_Port, RN_B_Pin) == GPIO_PIN_RESET)
		{
			StatusRNB = 1;
			CntTimerDebounceRNB = 0;
		}
		if(HAL_GPIO_ReadPin(RN_B_GPIO_Port, RN_B_Pin) == GPIO_PIN_SET)
		{
			StatusRNB = 2;
			CntTimerDebounceRNB = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceRNB > ConsDebounceRNB)
		{
			if(HAL_GPIO_ReadPin(RN_B_GPIO_Port,RN_B_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIORNB == GPIO_PIN_SET)
					FlagEdgeDetectionRNB = SET;
				else if(StatusGPIORNB == GPIO_PIN_RESET)
					FlagEdgeDetectionRNB = RESET;
				StatusGPIORNB = GPIO_PIN_RESET;
			}
			StatusRNB = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceRNB > ConsDebounceRNB)
		{
			if(HAL_GPIO_ReadPin(RN_B_GPIO_Port,RN_B_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIORNB == GPIO_PIN_SET)
					FlagEdgeDetectionRNB = RESET;
				else if(StatusGPIORNB == GPIO_PIN_RESET)
					FlagEdgeDetectionRNB = SET;
				StatusGPIORNB = GPIO_PIN_SET;
			}
			StatusRNB = 0;
		}
		break;
	default:
		break;
	}

	/********************	ENSF1	**************************/
	switch(StatusENSF1)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port, ENSF1_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF1 = 1;
			CntTimerDebounceENSF1 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port, ENSF1_Pin) == GPIO_PIN_SET)
		{
			StatusENSF1 = 2;
			CntTimerDebounceENSF1 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF1 > ConsDebounceENSF1)
		{
			if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF1 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF1 = SET;
				else if(StatusGPIOENSF1 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF1 = RESET;
				StatusGPIOENSF1 = GPIO_PIN_RESET;
			}
			StatusENSF1 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF1 > ConsDebounceENSF1)
		{
			if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF1 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF1 = RESET;
				else if(StatusGPIOENSF1 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF1 = SET;
				StatusGPIOENSF1 = GPIO_PIN_SET;
			}
			StatusENSF1 = 0;
		}
		break;
	default:
		break;
	}
	/********************	ENSF3	**************************/
	switch(StatusENSF3)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF3_GPIO_Port, ENSF3_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF3 = 1;
			CntTimerDebounceENSF3 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF3_GPIO_Port, ENSF3_Pin) == GPIO_PIN_SET)
		{
			StatusENSF3 = 2;
			CntTimerDebounceENSF3 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF3 > ConsDebounceENSF3)
		{
			if(HAL_GPIO_ReadPin(ENSF3_GPIO_Port,ENSF3_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF3 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF3 = SET;
				else if(StatusGPIOENSF3 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF3 = RESET;
				StatusGPIOENSF3 = GPIO_PIN_RESET;
			}
			StatusENSF3 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF3 > ConsDebounceENSF3)
		{
			if(HAL_GPIO_ReadPin(ENSF3_GPIO_Port,ENSF3_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF3 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF3 = RESET;
				else if(StatusGPIOENSF3 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF3 = SET;
				StatusGPIOENSF3 = GPIO_PIN_SET;
			}
			StatusENSF3 = 0;
		}
		break;
	default:
		break;
	}

	/********************	ENSF4	**************************/
	switch(StatusENSF4)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF4_GPIO_Port, ENSF4_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF4 = 1;
			CntTimerDebounceENSF4 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF4_GPIO_Port, ENSF4_Pin) == GPIO_PIN_SET)
		{
			StatusENSF4 = 2;
			CntTimerDebounceENSF4 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF4 > ConsDebounceENSF4)
		{
			if(HAL_GPIO_ReadPin(ENSF4_GPIO_Port,ENSF4_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF4 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF4 = SET;
				else if(StatusGPIOENSF4 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF4 = RESET;
				StatusGPIOENSF4 = GPIO_PIN_RESET;
			}
			StatusENSF4 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF4 > ConsDebounceENSF4)
		{
			if(HAL_GPIO_ReadPin(ENSF4_GPIO_Port,ENSF4_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF4 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF4 = RESET;
				else if(StatusGPIOENSF4 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF4 = SET;
				StatusGPIOENSF4 = GPIO_PIN_SET;
			}
			StatusENSF4 = 0;
		}
		break;
	default:
		break;
	}

	/********************	ENSF5	**************************/
	switch(StatusENSF5)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF5_GPIO_Port, ENSF5_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF5 = 1;
			CntTimerDebounceENSF5 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF5_GPIO_Port, ENSF5_Pin) == GPIO_PIN_SET)
		{
			StatusENSF5 = 2;
			CntTimerDebounceENSF5 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF5 > ConsDebounceENSF5)
		{
			if(HAL_GPIO_ReadPin(ENSF5_GPIO_Port,ENSF5_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF5 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF5 = SET;
				else if(StatusGPIOENSF5 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF5 = RESET;
				StatusGPIOENSF5 = GPIO_PIN_RESET;
			}
			StatusENSF5 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF5 > ConsDebounceENSF5)
		{
			if(HAL_GPIO_ReadPin(ENSF5_GPIO_Port,ENSF5_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF5 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF5 = RESET;
				else if(StatusGPIOENSF5 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF5 = SET;
				StatusGPIOENSF5 = GPIO_PIN_SET;
			}
			StatusENSF5 = 0;
		}
		break;
	default:
		break;
	}

	/********************	ENSF6	**************************/
	switch(StatusENSF6)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF6_GPIO_Port, ENSF6_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF6 = 1;
			CntTimerDebounceENSF6 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF6_GPIO_Port, ENSF6_Pin) == GPIO_PIN_SET)
		{
			StatusENSF6 = 2;
			CntTimerDebounceENSF6 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF6 > ConsDebounceENSF6)
		{
			if(HAL_GPIO_ReadPin(ENSF6_GPIO_Port,ENSF6_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF6 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF6 = SET;
				else if(StatusGPIOENSF6 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF6 = RESET;
				StatusGPIOENSF6 = GPIO_PIN_RESET;
			}
			StatusENSF6 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF6 > ConsDebounceENSF6)
		{
			if(HAL_GPIO_ReadPin(ENSF6_GPIO_Port,ENSF6_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF6 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF6 = RESET;
				else if(StatusGPIOENSF6 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF6 = SET;
				StatusGPIOENSF6 = GPIO_PIN_SET;
			}
			StatusENSF6 = 0;
		}
		break;
	default:
		break;
	}

	/********************	ENSF7	**************************/
	switch(StatusENSF7)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF7_GPIO_Port, ENSF7_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF7 = 1;
			CntTimerDebounceENSF7 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF7_GPIO_Port, ENSF7_Pin) == GPIO_PIN_SET)
		{
			StatusENSF7 = 2;
			CntTimerDebounceENSF7 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF7 > ConsDebounceENSF7)
		{
			if(HAL_GPIO_ReadPin(ENSF7_GPIO_Port,ENSF7_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF7 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF7 = SET;
				else if(StatusGPIOENSF7 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF7 = RESET;
				StatusGPIOENSF7 = GPIO_PIN_RESET;
			}
			StatusENSF7 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF7 > ConsDebounceENSF7)
		{
			if(HAL_GPIO_ReadPin(ENSF7_GPIO_Port,ENSF7_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF7 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF7 = RESET;
				else if(StatusGPIOENSF7 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF7 = SET;
				StatusGPIOENSF7 = GPIO_PIN_SET;
			}
			StatusENSF7 = 0;
		}
		break;
	default:
		break;
	}

	/********************	ENSF8	**************************/
	switch(StatusENSF8)
	{
	case 0:
		if(HAL_GPIO_ReadPin(ENSF8_GPIO_Port, ENSF8_Pin) == GPIO_PIN_RESET)
		{
			StatusENSF8 = 1;
			CntTimerDebounceENSF8 = 0;
		}
		if(HAL_GPIO_ReadPin(ENSF8_GPIO_Port, ENSF8_Pin) == GPIO_PIN_SET)
		{
			StatusENSF8 = 2;
			CntTimerDebounceENSF8 = 0;
		}
		break;
	case 1:
		if(CntTimerDebounceENSF8 > ConsDebounceENSF8)
		{
			if(HAL_GPIO_ReadPin(ENSF8_GPIO_Port,ENSF8_Pin) == GPIO_PIN_RESET)
			{
				if(StatusGPIOENSF8 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF8 = SET;
				else if(StatusGPIOENSF8 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF8 = RESET;
				StatusGPIOENSF8 = GPIO_PIN_RESET;
			}
			StatusENSF8 = 0;
		}
		break;
	case 2:
		if(CntTimerDebounceENSF8 > ConsDebounceENSF8)
		{
			if(HAL_GPIO_ReadPin(ENSF8_GPIO_Port,ENSF8_Pin) == GPIO_PIN_SET)
			{
				if(StatusGPIOENSF8 == GPIO_PIN_SET)
					FlagEdgeDetectionENSF8 = RESET;
				else if(StatusGPIOENSF8 == GPIO_PIN_RESET)
					FlagEdgeDetectionENSF8 = SET;
				StatusGPIOENSF8 = GPIO_PIN_SET;
			}
			StatusENSF8 = 0;
		}
		break;
	default:
		break;
	}

	// if(FlagEdgeDetectionS1 == SET)
	// {
	// if(StatusGPIOSensor1 == GPIO_PIN_RESET && StatusGPIOSensor2 == GPIO_PIN_RESET)
	// {
	// Phi_G = 0;

	// //      if(RotationStatus_G == Left) //Start in Phi = 3
	// //      {
	// //        FlagReverse_G = 1;
	// //        if(NR0To3_G == -2)
	// //        {
	// //          NS3To0_G = 0;
	// //          NR0To3_G = 0;
	// //        }
	// //        else
	// //          NS3To0_G = +2;
	// //      }

	// RotationStatus_G = Right;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 0;
	// }
	// if(StatusGPIOSensor1 == GPIO_PIN_SET && StatusGPIOSensor2 == GPIO_PIN_RESET) //30-330 Degree
	// {
	// Phi_G = 3;
	// if(RotationStatus_G == Right) //Reverse in Phi = 0
	// {
	// FlagReverse_G = 1;
	// //        if(NS3To0_G == +2)
	// //        {
	// //          NR0To3_G = 0;
	// //          NS3To0_G = 0;
	// //        }
	// //        else
	// NR0To3_G = -2;
	// }
	// RotationStatus_G = Left;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 3;

	// }

	// if(StatusGPIOSensor1 == GPIO_PIN_SET && StatusGPIOSensor2 == GPIO_PIN_SET)
	// {
	// Phi_G = 2;

	// if(RotationStatus_G == Left) //Start in Phi = 1
	// {
	// FlagReverse_G = 1;
	// NS1To2_G = +2;
	// //        NR2To1_G = 0;
	// }

	// RotationStatus_G = Right;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 2;
	// }
	// if(StatusGPIOSensor1 == GPIO_PIN_RESET && StatusGPIOSensor2 == GPIO_PIN_SET)
	// {
	// Phi_G = 1;
	// if(RotationStatus_G == Right) //Reverse in Phi = 2
	// {
	// FlagReverse_G = 1;
	// NR2To1_G = -1;
	// //        NS1To2_G = 0;
	// }
	// RotationStatus_G = Left;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 1;
	// FlagMagneticTrigger_G = 1;
	// FlagRPM_G = 1;

	// // ComboBox


	// }
	// FlagUpdateTroubleshooting_G = 1;
	// FlagEdgeDetectionS1 = 0;
	// }

	// if(FlagEdgeDetectionS2 == SET)
	// {
	// if(StatusGPIOSensor2 == GPIO_PIN_RESET && StatusGPIOSensor1 == GPIO_PIN_RESET)
	// {
	// Phi_G = 0;

	// //      if(RotationStatus_G == Right) //Reverse in Phi = 1
	// //      {
	// //        FlagReverse_G = 1;
	// //        if(NS0To1_G == +2)
	// //        {
	// //          NR1To0_G = 0;
	// //          NS0To1_G = 0;
	// //        }
	// //        else
	// //          NR1To0_G = -2;
	// //      }

	// RotationStatus_G = Left;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 0;
	// }
	// if(StatusGPIOSensor2 == GPIO_PIN_SET && StatusGPIOSensor1 == GPIO_PIN_RESET)
	// {
	// Phi_G = 1;

	// if(RotationStatus_G == Left) //Start in Phi = 0
	// {
	// FlagReverse_G = 1;
	// //        if(NR1To0_G == -2)
	// //        {
	// //          NS0To1_G = 0;
	// //          NR1To0_G = 0;
	// //        }
	// //        else
	// NS0To1_G = +2;
	// }
	// RotationStatus_G = Right;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 1;
	// }

	// if(StatusGPIOSensor2 == GPIO_PIN_SET && StatusGPIOSensor1 == GPIO_PIN_SET)
	// {
	// Phi_G = 2;
	// if(RotationStatus_G == Right) //Reverse in Phi = 3
	// {
	// FlagReverse_G = 1;
	// NR3To2_G = -2;
	// //        NS2To3_G = 0;
	// }
	// RotationStatus_G = Left;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 2;

	// // ComboBox
	// }
	// if(StatusGPIOSensor2 == GPIO_PIN_RESET && StatusGPIOSensor1 == GPIO_PIN_SET)
	// {
	// Phi_G = 3;
	// if(RotationStatus_G == Left) //Start in Phi = 2
	// {
	// FlagReverse_G = 1;
	// NS2To3_G = +1;
	// //        NR3To2_G = 0;
	// }
	// RotationStatus_G = Right;
	// memset(Temp, 0, sizeof(Temp));

	// for(i = 0; i < 3; i++)
	// Temp[i + 1] = ArrayPhiBackup_G[i];

	// for(i = 1; i < 4; i++)
	// ArrayPhiBackup_G[i] = Temp[i];

	// ArrayPhiBackup_G[0] = 3;
	// FlagMagneticTrigger_G = 1;
	// FlagRPM_G = 1;

	// // ComboBox
	// }
	// FlagUpdateTroubleshooting_G = 1;
	// FlagEdgeDetectionS2 = 0;
	// }  /* USER CODE END EXTI3_IRQn 0 */

	if(FlagEdgeDetectionENSF1 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF1 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusGPIOENSF5 == GPIO_PIN_SET && StatusGPIOENSF4 == GPIO_PIN_RESET && StatusComboBox_G == ComboBoxStop_G)
			// {
			// StatusComboBox_G = ComboBoxPaireStartInPhi1And2;
			// PreviousStatusComboBox = ComboBoxStop_G;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
		}
		if(StatusGPIOENSF1 == GPIO_PIN_RESET)
		{

		}
		FlagEdgeDetectionENSF1 = 0;
	}

	if(FlagEdgeDetectionENSF3 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF3 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusComboBox_G == ComboBoxPaireReverseInPhi11And2Completed && PreviousStatusComboBox == ComboBoxPaireReverseInPhi11And2Complete)
			// {
			// StatusComboBox_G = ComboBoxPaireStart;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && HAL_GPIO_ReadPin(ENSF4_GPIO_Port,ENSF4_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(ENSF5_GPIO_Port,ENSF5_Pin) == GPIO_PIN_RESET)
			// {
			// StatusComboBox_G = ComboBoxPaireStart;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
		}
		if(StatusGPIOENSF3 == GPIO_PIN_RESET)
		{
		}
		FlagEdgeDetectionENSF3 = 0;
	}

	if(FlagEdgeDetectionENSF4 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF4 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET && StatusGPIOENSF5 == GPIO_PIN_SET)
			// {
			// StatusComboBox_G = ComboBoxAuto;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
			// }
			// if(StatusGPIOENSF4 == GPIO_PIN_RESET)
			// {
			// StatusComboBox_G = ComboBoxStop_G;
			// ComboBoxFlagStop = 1;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
		}
		FlagEdgeDetectionENSF4 = 0;
	}

	if(FlagEdgeDetectionENSF5 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF5 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusComboBox_G == ComboBoxReverseInPhi2)
			// {
			// StatusComboBox_G = ComboBoxStop_G;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusComboBox_G == ComboBoxStart)
			// {
			// StatusComboBox_G = ComboBoxStop_G;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
			/* 			if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
						&& StatusComboBox_G == ComboBoxPaireReverse)
				{
					StatusComboBox_G = ComboBoxPaireStop;
					HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


					HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
				} */

			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusComboBox_G == ComboBoxPaireStart)
			// {
			// StatusComboBox_G = ComboBoxStop_G;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }

			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusComboBox_G == ComboBoxReverseInPhi1Completed && PreviousStatusComboBox == ComboBoxReverseInPhi1Complete && Phi_G == 1)
			// {
			// StatusComboBox_G = ComboBoxStop_G;
			// PreviousStatusComboBox = ComboBoxStop_G;

			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
		}
		if(StatusGPIOENSF5 == GPIO_PIN_RESET)
		{
		}
		FlagEdgeDetectionENSF5 = 0;
	}

	// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
	// && StatusComboBox_G == ComboBoxReverseInPhi1Completed && PreviousStatusComboBox == ComboBoxReverseInPhi1Complete && Phi_G == 1
	// && StatusGPIOENSF5 == GPIO_PIN_SET)
	// {
	// StatusComboBox_G = ComboBoxStop_G;
	// PreviousStatusComboBox = ComboBoxStop_G;

	// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


	// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
	// }
	if(FlagEdgeDetectionENSF6 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF6 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusGPIOENSF5 == GPIO_PIN_SET && StatusGPIOENSF4 == GPIO_PIN_RESET  && StatusComboBox_G == ComboBoxStop_G)
			// {
			// StatusComboBox_G = ComboBoxStart;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
		}
		if(StatusGPIOENSF6 == GPIO_PIN_RESET)
		{
		}
		FlagEdgeDetectionENSF6 = 0;
	}

	if(FlagEdgeDetectionENSF7 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF7 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusGPIOENSF5 == GPIO_PIN_SET && StatusGPIOENSF4 == GPIO_PIN_RESET && StatusComboBox_G == ComboBoxStop_G)
			// {
			// StatusComboBox_G = ComboBoxPaireStartInPhi1And2;
			// PreviousStatusComboBox = ComboBoxStop_G;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusComboBox_G == ComboBoxPaireReverseInPhi11And2Completed && PreviousStatusComboBox == ComboBoxPaireReverseInPhi11And2Complete)
			// {
			// StatusComboBox_G = ComboBoxPaireStart;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
		}
		if(StatusGPIOENSF7 == GPIO_PIN_RESET)
		{
		}
		FlagEdgeDetectionENSF7 = 0;
	}

	if(FlagEdgeDetectionENSF8 == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIOENSF8 == GPIO_PIN_SET)
		{
			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusGPIOENSF5 == GPIO_PIN_SET && StatusGPIOENSF4 == GPIO_PIN_RESET && Phi_G == 2 && StatusComboBox_G == ComboBoxStop_G)
			// {
			// StatusComboBox_G = ComboBoxReverseInPhi2;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_SET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }

			// if(HAL_GPIO_ReadPin(ENSF1_GPIO_Port,ENSF1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(ENSF2_GPIO_Port,ENSF2_Pin) == GPIO_PIN_SET
			// && StatusGPIOENSF5 == GPIO_PIN_SET && StatusGPIOENSF4 == GPIO_PIN_RESET && Phi_G == 1 && StatusComboBox_G == ComboBoxStop_G)
			// {
			// StatusComboBox_G = ComboBoxReverseInPhi1Start;
			// PreviousStatusComboBox = ComboBoxStop_G;
			// HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_RESET);
			// HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_RESET);


			// HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
			// HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
			// }
		}
		if(StatusGPIOENSF8 == GPIO_PIN_RESET)
		{
		}
		FlagEdgeDetectionENSF8 = 0;
	}

	if(FlagEdgeDetectionRNA == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIORNA == GPIO_PIN_SET)
		{

		}
		if(StatusGPIORNA == GPIO_PIN_RESET)
		{
			if(StatusGPIORNB == GPIO_PIN_SET) //Right
			{
				if(CntRN_G == 1)
					CntRN_G = MaxMenu_G;
				CntRN_G = CntRN_G - 1;

				//        StatusMainMenu_G = CntRN_G;
				KypadDirection_G = KeypadDown;
				FlagDisplay_G = 1;
			}
		}
		FlagEdgeDetectionRNA = 0;
	}

	if(FlagEdgeDetectionRNB == SET)
	{
		FlagUpdateTroubleshooting_G = 1;
		if(StatusGPIORNB == GPIO_PIN_SET)
		{

		}
		if(StatusGPIORNB == GPIO_PIN_RESET)
		{
			if(StatusGPIORNA == GPIO_PIN_SET) //Left
			{
				CntRN_G = CntRN_G + 1;
				if(CntRN_G == MaxMenu_G)
					CntRN_G = 1;

				KypadDirection_G = KeypadUp;
				FlagDisplay_G = 1;
			}
		}
		FlagEdgeDetectionRNB = 0;
	}
}

/* USER CODE END 1 */
