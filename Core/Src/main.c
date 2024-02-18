/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "fatfs.h"
#include "iwdg.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GlobalVariable.h"
#include <stdlib.h>
#include <stdio.h>

#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "stdbool.h"
#include "eeprom.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Newbranch		40
#define Commit4			4
#define Commit3			3
#define Commit2			2
#define MaxLine         3700
#define MaxMagnet       28
//#define MaxColorWeaving 4
#define MaxProgram      100
#define MaxCharacter    30

#define MaxPattern	    2
#define MaxCharNameSamplingPattern		20
#define MaxNameSamplingPattern			4

#define ConsMenuLockTime    5 * 60 * 1000
#define ConsDeadline	(1000 * 60 * 60) / 100  // Max Hour = 3600 H


#define LocalReset_Pin    GPIO_PIN_RESET
#define LocalSet_Pin      GPIO_PIN_SET

#define MaxCharPassword		3

#define A						0.5
#define B						1399

#define InfA					0.2
#define InfB					1398

#define MaxWeftPerCentimeter	99
#define DefineShiftA			0
#define DefineShiftB			1
#define DefineShiftC			2
#define DefineShiftD			3
//GPIO_PIN_RESET
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t Version[20]="  Sulzer = 1.0.38  ";
FATFS mynewdiskFatFs; /* File system object for User logical drive */
FIL MyFile; /* File object */
char mynewdiskPath[4]; /* User logical drive path */

uint32_t wbytes; /* File write counts */
uint8_t wtext[] = "text to write logical disk"; /* File write buffer */
uint8_t ArrayProgramLine[MaxLine][MaxMagnet];
char Buffer[10], StatusMainMenu_G = 1 , StatusSubMenu_G = 1, TemporaryBuffer[MaxCharacter + 10], ArrayProgramName[MaxProgram + MaxNameSamplingPattern][MaxCharacter];
char FlagProcessUSB, FlagMountUSB, FlagFirstRun = 0, FlagWriteUSB, FlagReadUSB, CntProgramName, StatusInitialize;
uint32_t CntDelayTimer_G;
unsigned char SelectedProgram[5], LastLineNumber[20];
char FlagTemporary,TempStr[10],TempStr2[20];
int32_t StatusUSBEnum, CntProgramLine,ArrayAdressProgram[MaxProgram], ArrayCntProgramLine[MaxProgram];
//const struct {
//
//	GPIO_TypeDef Port[28];
//} Gpio = {};
//const GPIO_TypeDef
//GPIO_TypeDef Port[28] = {CMND1_GPIO_Port, CMND2_GPIO_Port};

// GPIO_TypeDef GpioPort[28] = {CMND1_GPIO_Port, CMND2_GPIO_Port, CMND3_GPIO_Port, CMND4_GPIO_Port, CMND5_GPIO_Port, CMND6_GPIO_Port, CMND7_GPIO_Port, CMND8_GPIO_Port,
//			   	   	   	   	 CMND9_GPIO_Port, CMND10_GPIO_Port, CMND11_GPIO_Port, CMND12_GPIO_Port, CMND13_GPIO_Port, CMND14_GPIO_Port, CMND15_GPIO_Port, CMND16_GPIO_Port,
//							 CMND17_GPIO_Port, CMND18_GPIO_Port, CMND19_GPIO_Port, CMND20_GPIO_Port, CMND21_GPIO_Port, CMND22_GPIO_Port, CMND23_GPIO_Port, CMND24_GPIO_Port,
//							 CMND25_GPIO_Port, CMND26_GPIO_Port, CMND27_GPIO_Port, CMND28_GPIO_Port};
 uint16_t CMNDPin[MaxMagnet] = {
		CMND1_Pin, CMND2_Pin, CMND3_Pin, CMND4_Pin, CMND5_Pin, CMND6_Pin, CMND7_Pin, CMND8_Pin, CMND9_Pin, // GPIOE
		CMND10_Pin, CMND11_Pin, CMND12_Pin, CMND13_Pin, CMND14_Pin, CMND15_Pin, CMND16_Pin, CMND17_Pin, CMND18_Pin,// GPIOB
		CMND19_Pin, CMND20_Pin, CMND21_Pin, CMND22_Pin, CMND23_Pin, CMND24_Pin, CMND25_Pin, CMND26_Pin, // GPIOD
		CMND27_Pin, CMND28_Pin // GPIOC
};
uint16_t TCMNDPin[4] = {
		T1CMND_Pin, T2CMND_Pin, T3CMND_Pin, // GPIOC
		T4CMND_Pin // GPIOA
};
int16_t BackupCntProgramLine = 0, BackupCntProgramLine2;
FRESULT Res;
UINT SizeWrite;
int8_t N, FlagException;
int8_t N_G, FlagTestMagnet;
bool Error;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0030, 0x0002, 0x0004, 0x0006, 0x0008, 0x00A, 0x000C, 0x000E, 0x0010};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t VarValue,VarDataTmp = 0;
uint16_t EEpromValidPage, RPM;
uint8_t FlagMagnetDisplay, FlagTroubleshooting, FlagUpdateTroubleshooting_G;
uint8_t FlagSelectMenuUsb, BackupFlagMenuUsb[5];
uint8_t CntPattern;
uint16_t CntWriteProgramLine, test1, test2;
uint8_t SelectedEditProgram, CntActiveMagnet, CntEditActiveMagnet, FlagMagnetDirection, FlagMagnetDirection;
uint8_t FlagLockMenu, FlagMenu = 1;
uint16_t Cntexpiration_G = 0;


const uint16_t ArrayAdressSector[120] = {
		0, 35,	70,	105,	140,	175,	210,	245,	280,	315,	350,
		385,	420,	455,	490,	525,	560,	595,	630,	665,
		700,	735,	770,	805,	840,	875,	910,	945,	980,
		1015,	1050,	1085,	1120,	1155,	1190,	1225,	1260,	1295,
		1330,	1365,	1400,	1435,	1470,	1505,	1540,	1575,	1610,
		1645,	1680,	1715,	1750,	1785,	1820,	1855,	1890,	1925,
		1960,	1995,	2030,	2065,	2100,	2135,	2170,	2205,	2240,
		2275,	2310,	2345,	2380,	2415,	2450,	2485,	2520,	2555,
		2590,	2625,	2660,	2695,	2730,	2765,	2800,	2835,	2870,
		2905,	2940,	2975,	3010,	3045,	3080,	3115,	3150,	3185,
		3220,	3255,	3290,	3325,	3360,	3395,	3430,	3465,	3500};
const char ArrayNameSamplingPattern[MaxNameSamplingPattern][MaxCharNameSamplingPattern] = {"S01 Zero Harnesses", "S02 Raise Harnesses", "S03 1:1", "S04 2:2"};
const uint8_t ArrayLineSamplingPattern[MaxNameSamplingPattern][MaxPattern][MaxMagnet] = {
		{{'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'}},
		{{'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'}},
		{{'1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0'},
				{'0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1', '0', '1'}},
				{{'1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0'},
						{'0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1', '0', '0', '1', '1'}}
};
const uint8_t ArrayLineNumberSamplingPattern[MaxNameSamplingPattern] = {1, 1, 2, 2};
const uint8_t ID[] = "ID = RASP00026      ";
uint8_t CntEditNumPassword[4] = {0,0,0,0}, CntEditPassword;
uint32_t NumPassword, RandomeValue, ORGPassword, InfPassword;
uint8_t FlagExpiration_G, FlagExpireProgram = 0;
uint8_t CntWeftPerCentimeter, CntEditWeftPerCentimeter;
float FloatWeftPerCentimeter,FloatCntWeftPerCentimeter,FloatTest;
uint16_t Int1WeftPerCentimeter, Int2WeftPerCentimeter;
uint32_t IntWeftPerCentimeter;
uint8_t CntEditNumWeftPerCentimeter[5] = {0,0,0,0,0};
const uint8_t Lock[8] = {
		0b01110,
		0b10001,
		0b10001,
		0b11111,
		0b11011,
		0b11011,
		0b11111,
		0b00000
		};
const uint8_t Arrows[8] = {24,28,30,31,31,30,28,24};
uint32_t CntTotalShiftRPM, CntTotalRPM;
float FloatCntTotalShiftRPM, FloatCntTotalRPM;
uint16_t CntRPM, ShiftStatus;
uint8_t CntEditeRPMMenu,CntEditNumRPMMenu[3];
uint16_t NumRPMMenu, Efficiency, IntTest;
uint8_t FlagShiftMenuDisplay;
const char ArrayMenuName[MainMenu][20] ={"Main Menu         "
										,"USB Setting       "
										,"RUN Setting       "
										,"Default Setting   "
										,"Troubleshooting   "
										,"Write Program     "
										,"Edit Program      "
										,"Sampling Pattern  "
										,"Active Magnet     "
										,"Magnet Direction  "
										,"Validity Date     "
										,"Woven Fabric      "
										,"RPM               "
										,"woven roll        "};
char ArrayLCDDisplay[4][20];
char LCDDispalyPosition[4]={1, 2, 3, 4};
int8_t CntMenui, CntMenuj, LcdYPosition;
uint8_t CntEditeWovenRollMenu, CntEditNumWovenRollMenu[3], FlagActiveWovenRoll;
uint16_t NumWovenRollMenu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
bool InitializeBuffer (void);
bool InitializeBuffer2 (void);
void MagnetCommand (int8_t ShiftArrayProgramLine);
void MagnetPaireCommand (int8_t ShiftArrayProgramLine);
void ResetCMND(void);
void MagnetDisplay(void);
void MagneSetAndReset(uint8_t CntMagnetSetAndReset);
void MagnetInitWriteProgram(void);
void CircularMenu(void);
void MCUFlashInit(void);
void SPIFlashInit(void);
void PhiStatusInit(void);
void ComboBoxStatusInit(void);
void ValidStatusInit(void);
void USBStatusInit(void);
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
//	Gpio.Port[0] =  CMND1_GPIO_Port;
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
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  MX_TIM7_Init();
  MX_RNG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(Watchdog_GPIO_Port, Watchdog_Pin, GPIO_PIN_RESET);
  /* Unlock the Flash Program Erase controller */
  MCUFlashInit();

  LCD_Init();
  LCD_Clear();
  W25qxx_Init();

  HAL_Delay(100);

  USBStatusInit();

  SPIFlashInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /************************     Initialize Buffer       ***********************/
  W25qxx_WriteDisable();
  Error = InitializeBuffer2();
  /************************		Initialize Buffer		***********************/
  PhiStatusInit();
  /***********************	Initialize ComboBox		***********************/
  ComboBoxStatusInit();
  /****************************************************************************/
  MagnetCommand(0);
  MX_USB_HOST_Process();

  HAL_GPIO_WritePin(Watchdog_GPIO_Port, Watchdog_Pin, GPIO_PIN_RESET);

  ValidStatusInit();

  LCD_CreateChar(0, Lock);
  LCD_CreateChar(1, Arrows);

//  CntTotalRPM = 0;
  while (1)
  {
		/**************************        Main Program    ************************/
		if(FlagSupplyFB_G == 1)
		{
			FlagMenu = 200; // Wrong Detection
			FlagDisplay_G = 1;
			FlagSupplyFB_G = 0;
			ResetCMND();
			HAL_FLASH_Unlock();
			if((EE_WriteVariable(VirtAddVarTab[0],  BackupCntProgramLine)) != HAL_OK)
			{
				Error_Handler();
			}
			if((EE_WriteVariable(VirtAddVarTab[1],  Cntexpiration_G)) != HAL_OK)
			{
				Error_Handler();
			}
			IntWeftPerCentimeter = FloatWeftPerCentimeter;
			Int1WeftPerCentimeter = IntWeftPerCentimeter & 0x0000FFFF;
			Int2WeftPerCentimeter = IntWeftPerCentimeter >> 16;
			if((EE_WriteVariable(VirtAddVarTab[2],  Int1WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}
			if((EE_WriteVariable(VirtAddVarTab[3],  Int2WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}

//			IntWeftPerCentimeter = CntTotalRPM;
			Int1WeftPerCentimeter = CntTotalShiftRPM & 0x0000FFFF;
			Int2WeftPerCentimeter = CntTotalShiftRPM >> 16;
			if((EE_WriteVariable(VirtAddVarTab[4],  Int1WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}
			if((EE_WriteVariable(VirtAddVarTab[5],  Int2WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}

			Int1WeftPerCentimeter = CntMinutes_G;
			if((EE_WriteVariable(VirtAddVarTab[6],  Int1WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}
			Int1WeftPerCentimeter = CntHour_G;
			if((EE_WriteVariable(VirtAddVarTab[7],  Int1WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}

			Int1WeftPerCentimeter = CntTotalRPM & 0x0000FFFF;
			Int2WeftPerCentimeter = CntTotalRPM >> 16;
			if((EE_WriteVariable(VirtAddVarTab[8],  Int1WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}
			if((EE_WriteVariable(VirtAddVarTab[9],  Int2WeftPerCentimeter)) != HAL_OK)
			{
				Error_Handler();
			}

			HAL_FLASH_Lock();
			HAL_GPIO_WritePin(Watchdog_GPIO_Port, Watchdog_Pin, GPIO_PIN_RESET);
		}
		if(FlagSupplyFB_G == 2)
		{
			HAL_NVIC_SystemReset();
		}
		/***********************        Switch Trigger  ********************/

		/**************************************************************************/
		/**************************        Home Switch     ************************/
		if(CntDelayTimer_G > ConsMenuLockTime && FlagLockMenu == 0)
			FlagHome_G = 1;

		if(FlagHome_G == 1)
		{
			if(FlagLockMenu == 1 && FlagSwF2_G == 1 && HAL_GPIO_ReadPin(SW_F1_GPIO_Port,SW_F1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(SW_Right_GPIO_Port,SW_Right_Pin) == GPIO_PIN_SET) // Unlock Menu
			{
				FlagSwF2_G = 0;
				FlagLockMenu = 0;
				MaxMenu_G = MainMenu;
				FlagMenu = 1;
				FlagShiftMenuDisplay = 0;
				CntRN_G = 1;
				FlagDisplay_G = 1;
			}
			else if(FlagLockMenu == 0 && FlagSwF2_G == 1 && HAL_GPIO_ReadPin(SW_F1_GPIO_Port,SW_F1_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(SW_Right_GPIO_Port,SW_Right_Pin) == GPIO_PIN_SET) // Lock Menu
			{
				FlagSwF2_G = 0;
				FlagLockMenu = 1;
				MaxMenu_G = LockedMainMenu;
				if(FlagMenu == 220)
					FlagMenu = 220;
				else
					FlagMenu = 1;
				CntRN_G = 1;
				FlagDisplay_G = 1;
			}
			else if(FlagMenu == 1 && FlagSwF2_G == 1 && HAL_GPIO_ReadPin(SW_F1_GPIO_Port,SW_F1_Pin) == GPIO_PIN_RESET)
			{
				FlagDisplay_G = 1;
				FloatWeftPerCentimeter = 0;
			}
			else if((FlagMenu == 1 || FlagMenu == 220) && FlagSwF2_G == 1 && HAL_GPIO_ReadPin(SW_Right_GPIO_Port,SW_Right_Pin) == GPIO_PIN_RESET)
			{
				FlagDisplay_G = 1;
				ShiftStatus = ShiftStatus + 1;
				if(ShiftStatus > 3)
					ShiftStatus = 0;
				CntTotalShiftRPM = 0;
				FloatCntTotalShiftRPM = CntTotalShiftRPM;
				CntMinutes_G = 0;
				CntHour_G = 0;
				FloatTest = CntTotalRPM;
				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"%.1f",FloatTest / 1000);
				LCD_Puts(10,1,TempStr);

				memset(TempStr, 0, sizeof(TempStr));
				if(CntHour_G < 10)
					sprintf(TempStr,"0%d",CntHour_G);
				else if(CntHour_G > 9)
					sprintf(TempStr,"%d",CntHour_G);
				LCD_Puts(6, 2, TempStr);

				LCD_Puts(8, 2, ":");

				memset(TempStr, 0, sizeof(TempStr));
				if(CntMinutes_G < 10)
					sprintf(TempStr,"0%d",CntMinutes_G);
				else if(CntMinutes_G > 9)
					sprintf(TempStr,"%d",CntMinutes_G);
				LCD_Puts(9, 2, TempStr);

				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"%.1f   ",FloatCntTotalShiftRPM / 1000);
				LCD_Puts(10,3,TempStr);

				if(Efficiency < 10)
					sprintf(TempStr,"%d%%  ",Efficiency);
				else if(Efficiency > 10 && Efficiency < 100)
					sprintf(TempStr,"%d%% ",Efficiency);
				else if(Efficiency == 100)
					sprintf(TempStr,"%d%%",Efficiency);
				LCD_Puts(15, 0, TempStr);
			}
			if(CntDelayTimer_G > ConsMenuLockTime && FlagLockMenu == 0)
			{
				FlagLockMenu = 1;
				MaxMenu_G = LockedMainMenu;
				if(FlagMenu == 220)
					FlagMenu = 220;
				else
					FlagMenu = 1;
				CntRN_G = 1;
				FlagDisplay_G = 1;
			}
			LCD_CursorOff();
			LCD_BlinkOff();
			FlagSwF1_G = 0;
			FlagSwF2_G = 0;
			FlagHome_G = 0;
			CntDelayTimer_G = 0;
			//			MaxMenu_G = MainMenu;
			//			FlagMenu  = 1;
			//			CntRN_G = 1;
			//			FlagRNSw_G = 0;
			//			FlagDisplay_G = 1;
		}

		if(FlagDisplay_G == 1)
		{
			FlagDisplay_G = 0;
			/**************************         Magnet Menu       ************************/
			if(FlagMenu == 0)
			{
				FlagMagnetDisplay = 1;
				MagnetDisplay();
				if(FlagRNSw_G == 1 || FlagSwF1_G == 1)
				{
					FlagMagnetDisplay = 0;
					FlagMenu  = 1; //Magnet Display
					MaxMenu_G = MainMenu; // (2 - 6) for USBMenu
					CntRN_G = 1;
					FlagDisplay_G = 1;
					FlagSwF1_G = 0;
					FlagSwF2_G = 0;
					FlagRNSw_G = 0;
				}
			}
			/**************************         Main Menu       ************************/
			if(FlagMenu == 1)
			{
				if(FlagSwDown_G == 1)
				{
					FlagSwF2_G = 0;
					FlagSwDown_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
					KypadDirection_G = KeypadUp;
				}
				if(FlagSwUp_G == 1)
				{
					FlagSwF2_G = 0;
					FlagSwUp_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
					KypadDirection_G = KeypadDown;
				}
				CircularMenu();
				StatusMainMenu_G = CntRN_G;
				switch(StatusMainMenu_G)
				{
				case 0 :

					break;
				case 1:
					LCD_Clear();

					LCD_Puts(0, 0, "RayaSanat");
					if(FlagLockMenu == 1)
					{
						MaxMenu_G = LockedMainMenu;
						LCD_PutCustom(19,0,0); //Lock Shape
					}

					LCD_Puts(0,1,ArrayProgramName[SelectedProgram[0]]);

					LCD_Puts(0, 3, "LN=");
					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d    ",BackupCntProgramLine + 1);
					LCD_Puts(3,3,TempStr);

					LCD_Puts(9, 3, "WT=");
					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%.2f  ",FloatWeftPerCentimeter / 100);
					LCD_Puts(12,3,TempStr);

					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"RPM = %d   ",RPM);
					LCD_Puts(0,2,TempStr);

					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"AM=%d ",CntActiveMagnet);
					LCD_Puts(14,2,TempStr);

					if(FlagMagnetDirection == 1)
						LCD_Puts(19,2,"D");
					else if(FlagMagnetDirection == 0)
						LCD_Puts(19,2,"R");

					FlagSwF2_G = 0;
					if(FlagRNSw_G == 1)
					{
						FlagMenu  = 0; //Magnet Display
						MaxMenu_G = MagnetMenu; // (2 - 6) for USBMenu
						CntRN_G = 1;
						FlagDisplay_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
					}
					if(FlagSwF1_G == 1)
					{
						FlagMenu  = 220; //Shift Display
						MaxMenu_G = ShiftMenu; // (2 - 6) for USBMenu
						LCD_Clear();
						if(ShiftStatus == DefineShiftA)
							LCD_Puts(0,0,"Shift= A");
						else if(ShiftStatus == DefineShiftB)
							LCD_Puts(0,0,"Shift= B");
						else if(ShiftStatus == DefineShiftC)
							LCD_Puts(0,0,"Shift= C");
						else if(ShiftStatus == DefineShiftD)
							LCD_Puts(0,0,"Shift= D");

						memset(TempStr, 0, sizeof(TempStr));
						FloatCntTotalRPM = CntTotalRPM;
						sprintf(TempStr,"%.1f  ",FloatCntTotalRPM / 1000);
						LCD_Puts(10,1,TempStr);

						memset(TempStr, 0, sizeof(TempStr));
						if(CntHour_G < 10)
							sprintf(TempStr,"0%d",CntHour_G);
						else if(CntHour_G > 9)
							sprintf(TempStr,"%d",CntHour_G);
						LCD_Puts(6, 2, TempStr);

						LCD_Puts(8, 2, ":");

						memset(TempStr, 0, sizeof(TempStr));
						if(CntMinutes_G < 10)
							sprintf(TempStr,"0%d",CntMinutes_G);
						else if(CntMinutes_G > 9)
							sprintf(TempStr,"%d",CntMinutes_G);
						LCD_Puts(9, 2, TempStr);
						if(FlagLockMenu == 1)
						{
							MaxMenu_G = LockedMainMenu;
	//						LCD_Puts(14, 0, "Locked");
							LCD_PutCustom(19,0,0); //Lock Shape
						}

						memset(TempStr, 0, sizeof(TempStr));
						sprintf(TempStr,"%.1f   ",FloatCntTotalShiftRPM / 1000);
						LCD_Puts(10,3,TempStr);

						FloatTest = (CntMinutes_G * NumRPMMenu + CntHour_G * 60 * NumRPMMenu) / 100;
						if(IntTest == 0)
							IntTest = 1;
						FloatTest = FloatCntTotalShiftRPM /  FloatTest;
						Efficiency = FloatTest;
						if(Efficiency > 100)
							Efficiency = 100;
						if(Efficiency < 10)
							sprintf(TempStr,"%d%%  ",Efficiency);
						else if(Efficiency > 10 && Efficiency < 100)
							sprintf(TempStr,"%d%% ",Efficiency);
						else if(Efficiency == 100)
							sprintf(TempStr,"%d%%",Efficiency);
						LCD_Puts(15, 0, TempStr);
						CntRN_G = 1;
						FlagDisplay_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
					}
					break;
				case 2:
//					LCD_Clear();
//					for(CntMenuj = 0; CntMenuj < 4; CntMenuj++)
//						LCD_Puts(1, CntMenuj, ArrayLCDDisplay[CntMenuj]);
//					LCD_PutCustom(0,LcdYPosition,1); // Arrows

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = USBMenu; // (2 - 6) for USBMenu
						FlagMenu  = 2;
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				case 3:
//					LCD_Clear();
//					LCD_Puts(0, 0, "USB Setting        ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Run Setting     ");
//					LCD_Puts(0, 2, "Default Setting    ");
//					LCD_Puts(0, 3, "Troubleshooting    ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
//						MaxMenu_G = RunMenu + CntProgramName;
						FlagMenu  = 11; //(11 - 20) for Run Menu
						CntRN_G = 1;
						FlagDisplay_G = 1;
						LCD_Clear();
						LCD_Puts(0, 0, "Please Wait ... ");
						InitializeBuffer();
						MaxMenu_G = RunMenu + CntProgramName;
					}
					break;
				case 4:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Run Setting         ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Default Setting  ");
//					LCD_Puts(0, 2, "Troubleshooting     ");
//					LCD_Puts(0, 3, "Write Program      ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						MaxMenu_G = DefaultMenu;
						FlagMenu  = 7; //(7 - 10) for Default Menu
						CntRN_G = 1;
						FlagDisplay_G = 1;
					}
					break;
				case 5:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Default Setting     ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Troubleshooting  ");
//					LCD_Puts(0, 2, "Write Program       ");
//					LCD_Puts(0, 3, "Edit Program        ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						MaxMenu_G = TroubleshootingMenu;
						FlagMenu  = 21; //(21 - 30) for Troubleshooting Menu
						CntRN_G = 1;
						FlagDisplay_G = 1;
					}
					break;
				case 6:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Troubleshooting     ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Write Program    ");
//					LCD_Puts(0, 2, "Edit Program        ");
//					LCD_Puts(0, 3, "Sampling Pattern    ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = WriteProgramMenu;
						FlagMenu  = 31; //(31 - 40) for Write Program Menu
						CntRN_G = 2;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				case 7:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Write Program       ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Edit Program     ");
//					LCD_Puts(0, 2, "Sampling Pattern    ");
//					LCD_Puts(0, 3, "Active Magnet       ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;

						FlagMenu  = 41; //(41 - 50) for Edit Program Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						LCD_Clear();
						LCD_Puts(0, 0, "Please Wait ... ");
						InitializeBuffer();
						MaxMenu_G = EditProgramMenu2 + CntProgramName;
					}
					break;
				case 8:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Edit Program        ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Sampling Pattern ");
//					LCD_Puts(0, 2, "Active Magnet       ");
//					LCD_Puts(0, 3, "Magnet Direction    ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = SamplingPatternMenu + MaxNameSamplingPattern;
						FlagMenu  = 51; //(51 - 60) for Sampling Pattern Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				case 9:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Sampling Pattern    ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Active Magnet    ");
//					LCD_Puts(0, 2, "Magnet Direction    ");
//					LCD_Puts(0, 3, "Validity date       ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = ActiveMagnetMenu;
						FlagMenu  = 61; //(61 - 70) for Active Magnet Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						CntEditActiveMagnet = CntActiveMagnet;
					}
					break;
				case 10:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Active Magnet       ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Magnet Direction ");
//					LCD_Puts(0, 2, "Validity date       ");
//					LCD_Puts(0, 3, "Woven fabric        ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = MagnetDirectionMenu;
						FlagMenu  = 71; //(71 - 80) for Magnet Direction Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				case 11:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Magnet Direction    ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Validity date     ");
//					LCD_Puts(0, 2, "Woven fabric        ");
//					LCD_Puts(0, 3, "RPM                 ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						HAL_RNG_GenerateRandomNumber(&hrng, &RandomeValue);
						RandomeValue = RandomeValue >> 20;

						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = ValidityDateMenu;
						FlagMenu  = 81; //(81 - 90) for Magnet Direction Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						LCD_Clear();
					}
					break;
				case 12:
//					LCD_Clear();
//					LCD_Puts(0, 0, "Validity date       ");
//					LCD_PutCustom(0,1,1); // Arrows
//					LCD_Puts(1, 1, " Woven fabric       ");
//					LCD_Puts(0, 2, "RPM                 ");
//					LCD_Puts(0, 3, "Main Menu           ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = WovenfabricMenu;
						FlagMenu  = 91; //(91 - 100) for Woven fabric Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
//						CntEditWeftPerCentimeter = FloatCntWeftPerCentimeter;
						CntEditWeftPerCentimeter = 0;
						LCD_Clear();
					}
					break;
				case 13:
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = RPMMenu;
						FlagMenu  = 101; //(101 - 110) for Woven fabric Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
//						CntEditWeftPerCentimeter = FloatCntWeftPerCentimeter;
						CntEditeRPMMenu = 0;
						LCD_Clear();
					}
					break;
				case 14:
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						MaxMenu_G = WovenRollMenu;
						FlagMenu  = 101; //(111 - 120) for Woven fabric Menu
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						//						CntEditWeftPerCentimeter = FloatCntWeftPerCentimeter;
						CntEditeRPMMenu = 0;
						LCD_Clear();
					}
					break;
				default:
					break;
				}
			}
			/*************************          USB Menu       *****************/
			if(FlagMenu == 2)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 2;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0 :
					break;
				case 1:
					LCD_Clear();
					LCD_PutCustom(0,0,1); // Arrows
					LCD_Puts(1, 0, " Write To USB     ");
					LCD_Puts(0, 1, "Read From USB       ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = USBMenu_Write;
						FlagMenu  = 3; // USB Menu --> Write
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				case 2:
					LCD_Clear();
					LCD_Puts(0, 0, "Write To USB        ");
					LCD_PutCustom(0,1,1); // Arrows
					LCD_Puts(1, 1, " Read From USB    ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = USBMenu_Read;
						FlagMenu  = 4; // USB Menu --> Read
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				default:
					break;
				}
			}
			// USB Menu --> Write
			if(FlagMenu == 3)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = USBMenu;
					FlagMenu  = 2;
					CntRN_G = 1;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0 :
					break;
				case 1:
					LCD_Clear();
					LCD_PutCustom(0,0,1); // Arrows
					LCD_Puts(1,0," Please Stop ");
					LCD_Puts(0,1,"Machine And Continue");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						LCD_Clear();
						LCD_Puts(0,0,"Please Wait ...");
						if(FlagSelectMenuUsb == 0)
						{
							memset(BackupFlagMenuUsb, 0, sizeof(BackupFlagMenuUsb));
							BackupFlagMenuUsb[0] = FlagMenu;

							W25qxx_WriteEnable();
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
							HAL_Delay(100);
							W25qxx_EraseSector(W25qxx_PageToSector(64000));
							HAL_Delay(100);
							W25qxx_WritePage(BackupFlagMenuUsb, 64000, 0, 3);
							W25qxx_WriteDisable();
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);               //  Write Protection Enable             //  Write Protection Enable

							//							FlagMenu = 1;
							HAL_Delay(100);
							HAL_NVIC_SystemReset();
							HAL_Delay(1000);
						}
						if(FlagFirstRun == 1)
						{
							USBH_LL_Connect(&hUsbHostFS);
						}
						FlagFirstRun = 1;
						FlagProcessUSB = 1;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						CntDelayTimer_G = 0;
						FlagReadUSB = 0;
						FlagWriteUSB = 1;
					}
					break;
				default:
					break;
				}
			}
			// USB Menu --> Read
			if(FlagMenu == 4)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = USBMenu;
					FlagMenu  = 2;
					CntRN_G = 2;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0 :
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Please Stop Machine And Continue");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						LCD_Clear();
						LCD_Puts(0,0,"Please Wait ...");
						if(FlagSelectMenuUsb == 0)
						{
							memset(BackupFlagMenuUsb, 0, sizeof(BackupFlagMenuUsb));
							BackupFlagMenuUsb[0] = FlagMenu;

							W25qxx_WriteEnable();
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
							HAL_Delay(100);
							W25qxx_EraseSector(W25qxx_PageToSector(64000));
							HAL_Delay(100);
							W25qxx_WritePage(BackupFlagMenuUsb, 64000, 0, 3);
							W25qxx_WriteDisable();
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);               //  Write Protection Enable             //  Write Protection Enable

							//							FlagMenu = 1;
							HAL_Delay(100);
							HAL_NVIC_SystemReset();
							HAL_Delay(1000);
						}
						if(FlagFirstRun == 1)
						{
							USBH_LL_Connect(&hUsbHostFS);
						}
						FlagFirstRun = 1;
						FlagProcessUSB = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						CntDelayTimer_G = 0;
						FlagReadUSB = 1;
						FlagWriteUSB = 0;
					}
					break;
				default:
					break;
				}
			}
			/********************************************************************/
			/**************************       Default Menu       *****************/
			if(FlagMenu == 7)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 4;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0 :
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Stop Machine");
					LCD_Puts(0,1,"Delete Memory?");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						LCD_Clear();
						LCD_Puts(0,0,"Please Wait ...");
						HAL_Delay(2000);

						W25qxx_WriteEnable();
						W25qxx_EraseChip();


						LCD_Clear();
						LCD_Puts(0, 0, "Chip Erase ");
						LCD_Puts(0, 1, "Successfully ...");
						HAL_Delay(2000);

						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_WritePage("Pr00 Undefined", ArrayAdressSector[0] + 1, 0,  MaxCharacter);
						W25qxx_WritePage("Continue", ArrayAdressSector[0] + 1, MaxCharacter, MaxCharacter);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);               //  Write Protection Enable

						HAL_Delay(100);

						ResetCMND();
						HAL_FLASH_Unlock();
						BackupCntProgramLine = 0;
						if((EE_WriteVariable(VirtAddVarTab[0],  BackupCntProgramLine)) != HAL_OK)
						{
							Error_Handler();
						}
						HAL_FLASH_Lock();

						InitializeBuffer();

						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65096));
						HAL_Delay(100);
						memset(TempStr, 0, sizeof(TempStr));
						TempStr[0] = FlagExpiration_G;
						W25qxx_WritePage(TempStr, 65096, 0, 1);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);

						MaxMenu_G = MainMenu;
						FlagMenu  = 1; // Main Menu
						CntRN_G = 4;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				default:
					break;
				}
			}
			/**************************       Run Menu       *****************/
			if(FlagMenu == 11)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 3;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
					LCD_Clear();
					LCD_Puts(0,0,ArrayProgramName[StatusSubMenu_G - 1]);
					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d",ArrayCntProgramLine[StatusSubMenu_G - 1]);
					LCD_Puts(5,1,"Ln");
					LCD_Puts(8,1,TempStr);

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						BackupCntProgramLine = 0;

						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;

						memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						CntProgramLine = 0;
						SelectedProgram[0] = StatusSubMenu_G - 1;

						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65000));
						HAL_Delay(100);
						W25qxx_WritePage(SelectedProgram, 65000, 0, 1);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable
						HAL_FLASH_Unlock();
						if((EE_WriteVariable(VirtAddVarTab[0],  BackupCntProgramLine)) != HAL_OK)
						{
							Error_Handler();
						}
						HAL_FLASH_Lock();
						HAL_Delay(100);
						memset(ArrayProgramLine, 0, sizeof(ArrayProgramLine));
						for(int i = 1; i < MaxLine; i++)
						{
							W25qxx_ReadPage(TemporaryBuffer, ArrayAdressProgram[SelectedProgram[0]] + i / 8, (i % 8) * MaxCharacter, MaxCharacter);
							if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
							{
								i = MaxLine ;
								break;
							}
							strncpy(ArrayProgramLine[CntProgramLine], TemporaryBuffer, MaxMagnet);
							CntProgramLine = CntProgramLine + 1;
							memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						}
						MagnetCommand(0);
					}
					break;
				}
			}
			/**************************       Troubleshooting Menu       *****************/
			if(FlagMenu == 21)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					FlagTroubleshooting = 0;
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 5;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
					}
					LCD_Clear();
					FlagTroubleshooting = 1;
					FlagUpdateTroubleshooting_G = 1;
					break;
				case 2:
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
					}
					LCD_Clear();
					FlagTroubleshooting = 2;
					FlagUpdateTroubleshooting_G = 1;
					break;
				}
			}
			/**************************       Write Program Menu       *****************/
			if(FlagMenu == 31)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 6;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
					LCD_Clear();
					LCD_Puts(0,0,"Select The Name:");
					LCD_Puts(0,1,"Pattern ");

					if(FlagSwRight_G == 1)
					{
						FlagSwRight_G = 0;
						CntPattern = CntPattern + 1;
					}

					if(CntPattern == 10)
						CntPattern = 0;

					if(FlagSwLeft_G == 1)
					{
						FlagSwLeft_G = 0;
						if(CntPattern == 0)
							CntPattern = 10;
						CntPattern = CntPattern - 1;
					}

					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d   ",CntPattern);
					LCD_Puts(8,1,TempStr);
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu;
						FlagMenu  = 32;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						CntWriteProgramLine = 0;
						CntProgramLine = 1;
						memset(ArrayProgramLine, '0', sizeof(ArrayProgramLine));
						LCD_Clear();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				}
			}
			/**************************       Edit Program Menu       *****************/
			if(FlagMenu == 32)
			{
				if(FlagSwRight_G == 1)
				{
					FlagSwRight_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwLeft_G == 1)
				{
					FlagSwLeft_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = 1;
					CntWriteProgramLine = CntWriteProgramLine + 1;
					if(CntWriteProgramLine >= CntProgramLine)
						CntWriteProgramLine = 0;
					MagnetInitWriteProgram();
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					CntRN_G = 1;
					if(CntWriteProgramLine == 0)
						CntWriteProgramLine = CntProgramLine;
					CntWriteProgramLine = CntWriteProgramLine - 1;
					MagnetInitWriteProgram();
				}
				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"M%d ",CntRN_G);
				LCD_Puts(6,0,TempStr);
				LCD_Puts(10,0,":");

				memset(TempStr, 0, sizeof(TempStr));
				if(CntProgramLine <= 9 && CntProgramLine >= 0)
					sprintf(TempStr,"Mx%d    ",CntProgramLine);
				else if(CntProgramLine > 9 && CntProgramLine <= 99)
					sprintf(TempStr,"Mx%d   ",CntProgramLine);
				else if(CntProgramLine > 99 && CntProgramLine <= 999)
					sprintf(TempStr,"Mx%d  ",CntProgramLine);
				else if(CntProgramLine > 999 && CntProgramLine <= 3999)
					sprintf(TempStr,"Mx%d",CntProgramLine);
				else
					sprintf(TempStr,"Error",CntProgramLine);
				LCD_Puts(14,3,TempStr);


				memset(TempStr, 0, sizeof(TempStr));
				if(CntWriteProgramLine <= 9 )
					sprintf(TempStr,"L%d    ",CntWriteProgramLine + 1);
				else if(CntWriteProgramLine > 9 && CntWriteProgramLine <= 99)
					sprintf(TempStr,"L%d   ",CntWriteProgramLine + 1);
				else if(CntWriteProgramLine > 99 && CntWriteProgramLine <= 999)
					sprintf(TempStr,"L%d  ",CntWriteProgramLine + 1);
				else if(CntWriteProgramLine > 999 && CntWriteProgramLine <= 3999)
					sprintf(TempStr,"L%d ",CntWriteProgramLine + 1);
				else
					sprintf(TempStr,"Error",CntWriteProgramLine);
				LCD_Puts(0,0,TempStr);
				if(FlagSwRight_G == 1)
				{
					FlagSwRight_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwLeft_G == 1)
				{
					FlagSwLeft_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				MagneSetAndReset(CntRN_G);
				if(FlagRNSw_G == 1)
					FlagRNSw_G =0;
				if(FlagSwF2_G == 1)
				{
					FlagSwF2_G = 0;
					FlagMenu  = 33;
					MaxMenu_G = F2MassageWriteProgramMenu;
					CntRN_G = 1;
					FlagDisplay_G = 1;
					LCD_Clear();
					LCD_CursorOff();
					LCD_BlinkOff();
				}
				if(FlagSwF1_G == 1)
				{
					FlagSwF1_G = 0;
					FlagMenu  = 34;
					MaxMenu_G = F1MassageWriteProgramMenu;
					CntRN_G = 1;
					FlagDisplay_G = 1;
					LCD_Clear();
					LCD_CursorOff();
					LCD_BlinkOff();
				}
			}
			/**************************       F2 Massage Write Program Menu       *****************/
			if(FlagMenu == 33)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Save?");
					LCD_PutCustom(0,1,1); // Arrows
					LCD_Puts(1,1," F1:NO");
					LCD_Puts(13,1,"F2:Yes");
					LCD_Puts(0,2,"Yes & Create Line");
					LCD_Puts(0,3,"Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu;
						memset(ArrayProgramLine[CntWriteProgramLine], '0', sizeof(ArrayProgramLine[CntWriteProgramLine]));
						FlagMenu  = 32;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				case 2:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Save?");
					LCD_Puts(0,1,"F1:NO");
					LCD_PutCustom(10,1,1); // Arrows
					LCD_Puts(11,1," F2:Yes");
					LCD_Puts(0,2,"Yes & Create Line");
					LCD_Puts(0,3,"Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu;
						FlagMenu  = 32;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						CntProgramLine = CntWriteProgramLine + 1;

						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				case 3:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Save?");
					LCD_Puts(0,1,"F1:NO");
					LCD_Puts(13,1,"F2:Yes");
					LCD_PutCustom(0,2,1); // Arrows
					LCD_Puts(1,2," Yes & Create Line");
					LCD_Puts(0,3,"Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu;
						CntWriteProgramLine = CntWriteProgramLine + 1;
						CntProgramLine = CntWriteProgramLine + 1;
						FlagMenu  = 32;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				case 4:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Save?");
					LCD_Puts(0,1,"F1:NO");
					LCD_Puts(13,1,"F2:Yes");
					LCD_Puts(0,2,"Yes & Create Line");
					LCD_PutCustom(0,3,1); // Arrows
					LCD_Puts(1,3," Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu;
						if(CntWriteProgramLine >= 0)
						{
							memset(ArrayProgramLine[CntWriteProgramLine], '0', sizeof(ArrayProgramLine[CntWriteProgramLine]));
							if(CntWriteProgramLine > 0)
								CntWriteProgramLine = CntWriteProgramLine - 1;
						}
						CntProgramLine = CntWriteProgramLine + 1;
						FlagMenu  = 32;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				default:
					break;
				}
			}
			/**************************       F1 Massage Write Program Menu       *****************/
			if(FlagMenu == 34)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Save And Exit?");
					LCD_Puts(0,1,">> F1:NO");
					LCD_Puts(0,2,"F2:Yes");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						LCD_CursorOff();
						LCD_BlinkOff();
						LCD_Clear();
						LCD_Puts(0, 0, "Please Wait ...");
						InitializeBuffer();
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 6;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;

					}
					break;
				case 2:
					LCD_Clear();
					LCD_Puts(0,0,"Save And Exit?");
					LCD_Puts(0,1,"F1:NO");
					LCD_Puts(0,2,">> F2:Yes");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{

						LCD_CursorOff();
						LCD_BlinkOff();
						LCD_Clear();
						LCD_Puts(0, 0, "Please Wait ...");
						W25qxx_WriteEnable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable

						for(int i = ArrayAdressSector[CntProgramName]; i < ArrayAdressSector[CntProgramName + 1]; i++)
							W25qxx_EraseSector(i);

						test1 = 0;
						test2 = ArrayAdressSector[CntProgramName] * 16  + 1;
						memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						sprintf(TemporaryBuffer,"Pr %d Pattern %d", CntProgramName + 1, CntPattern);
						W25qxx_WritePage(TemporaryBuffer, test2, test1, MaxCharacter);

						for(int i = 1; i < CntWriteProgramLine + 2; i++)
						{
							test1 = (i % 8) * MaxCharacter;
							test2 = ArrayAdressSector[CntProgramName] * 16 + + i / 8 + 1;
							W25qxx_WritePage(ArrayProgramLine[i - 1], test2, test1, MaxMagnet);
						}
						test1 = ((CntWriteProgramLine + 2) % 8) * MaxCharacter;
						test2 = ArrayAdressSector[CntProgramName] * 16 + + (CntWriteProgramLine + 2) / 8 + 1;
						W25qxx_WritePage("Continue", test2, test1, 8);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable

						InitializeBuffer();
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 6;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				default:
					break;
				}
			}
			/**************************       Edit Program Menu2       *****************/
			if(FlagMenu == 41)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 7;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
					LCD_Clear();
					LCD_Puts(0,0,ArrayProgramName[StatusSubMenu_G - 1]);
					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d",ArrayCntProgramLine[StatusSubMenu_G - 1]);
					LCD_Puts(5,1,"Ln");
					LCD_Puts(8,1,TempStr);

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						BackupCntProgramLine = 0;

						MaxMenu_G = EditProgramMenu3;
						FlagMenu  = 42;
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;

						memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						CntProgramLine = 0;
						CntWriteProgramLine = 0;
						//						SelectedProgram[0] = StatusSubMenu_G - 2;
						SelectedEditProgram = StatusSubMenu_G - 1;
						for(int i = 1; i < MaxLine; i++)
						{
							W25qxx_ReadPage(TemporaryBuffer, ArrayAdressProgram[SelectedEditProgram] + i / 8, (i % 8) * MaxCharacter, MaxCharacter);
							if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
							{
								i = MaxLine ;
								break;
							}
							strncpy(ArrayProgramLine[CntProgramLine], TemporaryBuffer, MaxMagnet);
							CntProgramLine = CntProgramLine + 1;
							memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						}
						//						MagnetCommand(0);
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				}
			}
			/**************************       Edit Program Menu3       *****************/
			if(FlagMenu == 42)
			{

				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = 1;
					CntWriteProgramLine = CntWriteProgramLine + 1;
					if(CntWriteProgramLine >= CntProgramLine)
						CntWriteProgramLine = 0;
					MagnetInitWriteProgram();
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					CntRN_G = 1;
					if(CntWriteProgramLine == 0)
						CntWriteProgramLine = CntProgramLine;
					CntWriteProgramLine = CntWriteProgramLine - 1;
					MagnetInitWriteProgram();
				}
				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"M%d ",CntRN_G);
				LCD_Puts(6,0,TempStr);
				LCD_Puts(10,0,":");

				memset(TempStr, 0, sizeof(TempStr));
				if(CntProgramLine <= 9 && CntProgramLine > 0)
					sprintf(TempStr,"Mx%d    ",CntProgramLine);
				else if(CntProgramLine > 9 && CntProgramLine <= 99)
					sprintf(TempStr,"Mx%d   ",CntProgramLine);
				else if(CntProgramLine > 99 && CntProgramLine <= 999)
					sprintf(TempStr,"Mx%d  ",CntProgramLine);
				else if(CntProgramLine > 999 && CntProgramLine <= 3999)
					sprintf(TempStr,"Mx%d",CntProgramLine);
				else
					sprintf(TempStr,"Error",CntWriteProgramLine);
				LCD_Puts(14,3,TempStr);

				memset(TempStr, 0, sizeof(TempStr));
				if(CntWriteProgramLine <= 9 )
					sprintf(TempStr,"L%d    ",CntWriteProgramLine + 1);
				else if(CntWriteProgramLine > 9 && CntWriteProgramLine <= 99)
					sprintf(TempStr,"L%d   ",CntWriteProgramLine + 1);
				else if(CntWriteProgramLine > 99 && CntWriteProgramLine <= 999)
					sprintf(TempStr,"L%d  ",CntWriteProgramLine + 1);
				else if(CntWriteProgramLine > 999 && CntWriteProgramLine <= 3999)
					sprintf(TempStr,"L%d ",CntWriteProgramLine + 1);
				else
					sprintf(TempStr,"Error",CntWriteProgramLine);
				LCD_Puts(0,0,TempStr);

				MagneSetAndReset(CntRN_G);
				if(FlagRNSw_G == 1)
					FlagRNSw_G =0;
				if(FlagSwF2_G == 1)
				{
					FlagSwF2_G = 0;
					FlagMenu  = 43;
					MaxMenu_G = F2MassageEditProgramMenu;
					CntRN_G = 1;
					FlagDisplay_G = 1;
					LCD_Clear();
					LCD_CursorOff();
					LCD_BlinkOff();
				}
				if(FlagSwF1_G == 1)
				{
					FlagSwF1_G = 0;
					FlagMenu  = 44;
					MaxMenu_G = F1MassageEditProgramMenu;
					CntRN_G = 1;
					FlagDisplay_G = 1;
					LCD_Clear();
					LCD_CursorOff();
					LCD_BlinkOff();
				}
			}
			/**************************       F2 Massage Write Program Menu       *****************/
			if(FlagMenu == 43)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = EditProgramMenu3;
					//						memset(ArrayProgramLine[CntWriteProgramLine], '0', sizeof(ArrayProgramLine[CntWriteProgramLine]));
					FlagMenu  = 42;
					CntRN_G = 1;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
					MagnetInitWriteProgram();
					LCD_CursorOn();
					LCD_BlinkOn();
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Change?");
					LCD_Puts(0,1,">> F1:Back");
					LCD_Puts(0,2,"Yes & Create Line");
					LCD_Puts(0,3,"Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu3;
						//						memset(ArrayProgramLine[CntWriteProgramLine], '0', sizeof(ArrayProgramLine[CntWriteProgramLine]));
						FlagMenu  = 42;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				case 2:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Save?");
					LCD_Puts(0,1,"F1:NO");
					LCD_Puts(0,2,">> Yes & Create Line");
					LCD_Puts(0,3,"Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu3;

						if(CntProgramLine < 4000)
							CntProgramLine = CntProgramLine + 1;
						CntWriteProgramLine = CntWriteProgramLine + 1;
						for(uint16_t j = CntProgramLine - 1; j > CntWriteProgramLine; j--)
							strncpy(ArrayProgramLine[j], ArrayProgramLine[j - 1], MaxMagnet);

						memset(ArrayProgramLine[CntWriteProgramLine], '0', sizeof(ArrayProgramLine[CntWriteProgramLine]));

						FlagMenu  = 42;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				case 3:
					LCD_Clear();
					LCD_Puts(0,0,"Do you Want To Save?");
					LCD_Puts(0,1,"F1:NO");
					LCD_Puts(0,2,"Yes & Create Line");
					LCD_Puts(0,3,">> Yes & Delete Line");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = EditProgramMenu3;

						for(uint16_t j = CntWriteProgramLine; j < CntProgramLine; j++)
							strncpy(ArrayProgramLine[j], ArrayProgramLine[j + 1], MaxMagnet);

						if(CntProgramLine > 1)
							CntProgramLine = CntProgramLine - 1;
						if(CntWriteProgramLine > 0)
							CntWriteProgramLine = CntWriteProgramLine - 1;

						FlagMenu  = 42;
						CntRN_G = 1;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
						MagnetInitWriteProgram();
						LCD_CursorOn();
						LCD_BlinkOn();
					}
					break;
				default:
					break;
				}
			}
			/**************************       F1 Massage Write Program Menu       *****************/
			if(FlagMenu == 44)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Do You Want To Exit?");
					LCD_Puts(0,1,">> F1:NO");
					LCD_Puts(0,2,"F2:Yes");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						LCD_CursorOff();
						LCD_BlinkOff();
						LCD_Clear();
						LCD_Puts(0, 0, "Please Wait ...");
						InitializeBuffer();
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 7;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;

					}
					break;
				case 2:
					LCD_Clear();
					LCD_Puts(0,0,"Do You Want To Exit?");
					LCD_Puts(0,1,"F1:NO");
					LCD_Puts(0,2,">> F2:Yes");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{

						LCD_CursorOff();
						LCD_BlinkOff();
						LCD_Clear();
						LCD_Puts(0, 0, "Please Wait ...");

						W25qxx_WriteEnable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable

						for(int i = ArrayAdressSector[SelectedEditProgram]; i < ArrayAdressSector[SelectedEditProgram + 1]; i++)
							W25qxx_EraseSector(i);

						test1 = 0;
						test2 = ArrayAdressSector[SelectedEditProgram] * 16  + 1;
						memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						W25qxx_WritePage(ArrayProgramName[SelectedEditProgram], test2, test1, MaxCharacter);

						for(int i = 1; i < CntProgramLine + 1; i++)
						{
							test1 = (i % 8) * MaxCharacter;
							test2 = ArrayAdressSector[SelectedEditProgram] * 16 + + i / 8 + 1;
							W25qxx_WritePage(ArrayProgramLine[i - 1], test2, test1, MaxMagnet);
						}
						test1 = ((CntProgramLine + 1) % 8) * MaxCharacter;
						test2 = ArrayAdressSector[SelectedEditProgram] * 16 + + (CntProgramLine + 1) / 8 + 1;
						W25qxx_WritePage("Continue", test2, test1, 8);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable

						InitializeBuffer();
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 7;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;
					}
					break;
				default:
					break;
				}
			}
			/**************************       Sampling Pattern Menu    *****************/
			if(FlagMenu == 51)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 8;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagRNSw_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
					LCD_Clear();
					LCD_Puts(0,0,ArrayNameSamplingPattern[StatusSubMenu_G - 1]);
					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d",ArrayLineNumberSamplingPattern[StatusSubMenu_G - 1]);
					LCD_Puts(5,1,"Ln");
					LCD_Puts(8,1,TempStr);

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						BackupCntProgramLine = 0;

						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 8;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagRNSw_G = 0;
						FlagDisplay_G = 1;

						CntProgramLine = 0;
						SelectedProgram[0] = StatusSubMenu_G - 1 + MaxProgram;

						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65000));
						HAL_Delay(100);
						W25qxx_WritePage(SelectedProgram, 65000, 0, 1);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable
						HAL_FLASH_Unlock();
						if((EE_WriteVariable(VirtAddVarTab[0],  BackupCntProgramLine)) != HAL_OK)
						{
							Error_Handler();
						}
						HAL_FLASH_Lock();
						HAL_Delay(100);

						memset(ArrayProgramLine, 0, sizeof(ArrayProgramLine));
						for(int i = 0; i < ArrayLineNumberSamplingPattern[StatusSubMenu_G - 1]; i++)
						{
							strncpy(ArrayProgramLine[i], ArrayLineSamplingPattern[StatusSubMenu_G - 1][i], MaxMagnet);
							CntProgramLine = CntProgramLine + 1;
						}
						MagnetCommand(0);
					}
					break;
				}
			}
			/**************************       Active Magnet Menu       *****************/
			if(FlagMenu == 61)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwRight_G = 0;
					FlagSwLeft_G = 0;
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwRight_G = 0;
					FlagSwLeft_G = 0;
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}

				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					LCD_Clear();
					LCD_Puts(0,0,"Select Active Magnet");
					LCD_CursorOn();
					LCD_BlinkOn();

					if(FlagSwRight_G == 1)
					{
						FlagSwDown_G = 0;
						FlagSwUp_G = 0;
						FlagSwRight_G = 0;
						CntEditActiveMagnet = CntEditActiveMagnet + 1;
					}

					if(CntEditActiveMagnet == MaxMagnet + 1)
						CntEditActiveMagnet = 0;

					if(FlagSwLeft_G == 1)
					{
						FlagSwLeft_G = 0;
						FlagSwDown_G = 0;
						FlagSwUp_G = 0;
						if(CntEditActiveMagnet == 0)
							CntEditActiveMagnet = MaxMagnet + 1;
						CntEditActiveMagnet = CntEditActiveMagnet - 1;
					}

					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d  ",CntEditActiveMagnet);
					LCD_Puts(9,1,TempStr);
					LCD_Puts(9,1,"");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 9;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;

						CntActiveMagnet = CntEditActiveMagnet;
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65032));
						HAL_Delay(100);
						memset(TempStr, 0, sizeof(TempStr));
						TempStr[0] = CntActiveMagnet;
						W25qxx_WritePage(TempStr, 65032, 0, 1);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable

						ResetCMND();
						MagnetCommand(0);
						LCD_CursorOff();
						LCD_BlinkOff();
					}
					break;
				default:
					break;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 9;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
					LCD_CursorOff();
					LCD_BlinkOff();
				}
			}
			/**************************       Magnet Direction Menu    *****************/
			if(FlagMenu == 71)
			{
				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntRN_G = CntRN_G + 1;
					if(CntRN_G == MaxMenu_G)
						CntRN_G = 1;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntRN_G == 1)
						CntRN_G = MaxMenu_G;
					CntRN_G = CntRN_G - 1;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 10;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
					LCD_Clear();
					LCD_PutCustom(0,0,1); // Arrows
					LCD_Puts(1, 0, " Right To Left    ");
					LCD_Puts(0, 1, "Left  To Right      ");

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 10;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;

						FlagMagnetDirection = 1;
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65064));
						HAL_Delay(100);
						memset(TempStr, 0, sizeof(TempStr));
						TempStr[0] = FlagMagnetDirection;
						W25qxx_WritePage(TempStr, 65064, 0, 1);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable

						ResetCMND();
						MagnetCommand(0);

					}
					break;
				case 2:
					LCD_Clear();
					LCD_Puts(0, 0, "Right To Left       ");
					LCD_PutCustom(0,1,1); // Arrows
					LCD_Puts(1, 1, " Left To Right   ");
					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 10;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;

						FlagMagnetDirection = 0;
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65064));
						HAL_Delay(100);
						memset(TempStr, 0, sizeof(TempStr));
						TempStr[0] = FlagMagnetDirection;
						W25qxx_WritePage(TempStr, 65064, 0, 1);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable

						ResetCMND();
						MagnetCommand(0);
					}
					break;
				default:
					break;
				}
			}
			/**************************		  Validity Data			   *****************/
			if(FlagMenu == 81)
			{
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 11;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
					LCD_CursorOff();
					LCD_BlinkOff();
				}
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				case 1:
//					LCD_Clear();
					if(FlagExpiration_G == 1)
					{
						LCD_CursorOn();
						LCD_BlinkOn();

						LCD_Puts(0, 0, "THP=");
						memset(TempStr, 0, sizeof(TempStr));
						sprintf(TempStr,"%d  ",Cntexpiration_G);
						LCD_Puts(4,0,TempStr);

						LCD_Puts(11, 0, "PVD=");
						memset(TempStr, 0, sizeof(TempStr));
						sprintf(TempStr,"%d ",ConsDeadline);
						LCD_Puts(15,0,TempStr);

						memset(TempStr2, 0, sizeof(TempStr2));
						sprintf(TempStr2,"RNG = %d     ",RandomeValue);
						LCD_Puts(0, 1, TempStr2);
						ORGPassword = A * RandomeValue + B;
						InfPassword = InfA * RandomeValue + InfB;
						if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
						{
							NumPassword = CntEditNumPassword[0] * 1000 + CntEditNumPassword[1] * 100 + CntEditNumPassword[2] * 10 + CntEditNumPassword[3] * 1;
							if(NumPassword == ORGPassword || NumPassword == InfPassword)
							{
								if(NumPassword == InfPassword)
								{
									FlagExpiration_G = 0;
									HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
									W25qxx_WriteEnable();
									HAL_Delay(100);
									W25qxx_EraseSector(W25qxx_PageToSector(65096));
									HAL_Delay(100);
									memset(TempStr, 0, sizeof(TempStr));
									TempStr[0] = FlagExpiration_G;
									W25qxx_WritePage(TempStr, 65096, 0, 1);
									HAL_Delay(100);
									W25qxx_WriteDisable();
									HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable
								}
								else
								{
									FlagExpiration_G = 1;
									HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
									W25qxx_WriteEnable();
									HAL_Delay(100);
									W25qxx_EraseSector(W25qxx_PageToSector(65096));
									HAL_Delay(100);
									memset(TempStr, 0, sizeof(TempStr));
									TempStr[0] = FlagExpiration_G;
									W25qxx_WritePage(TempStr, 65096, 0, 1);
									HAL_Delay(100);
									W25qxx_WriteDisable();
									HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);

									HAL_FLASH_Unlock();
									Cntexpiration_G = 0;
									if((EE_WriteVariable(VirtAddVarTab[1],  Cntexpiration_G)) != HAL_OK)
									{
										Error_Handler();
									}
									HAL_FLASH_Lock();
								}
								FlagRNSw_G = 0;
								FlagSwF2_G = 0;
								FlagSwF1_G = 0;
								FlagDisplay_G = 1;
							}
							else
							{
								LCD_Puts(7,3,"Error");
								FlagRNSw_G = 0;
								FlagSwF2_G = 0;
								FlagSwF1_G = 0;
							}
						}

						if(FlagSwRight_G == 1)
						{
							FlagSwRight_G = 0;
							CntEditPassword = CntEditPassword + 1;
						}

						if(CntEditPassword == MaxCharPassword + 1)
							CntEditPassword = 0;

						if(FlagSwLeft_G == 1)
						{
							FlagSwLeft_G = 0;
							if(CntEditPassword == 0)
								CntEditPassword = MaxCharPassword + 1;
							CntEditPassword = CntEditPassword - 1;
						}

						if(FlagSwUp_G == 1)
						{
							FlagSwUp_G = 0;
							CntEditNumPassword[CntEditPassword] = CntEditNumPassword[CntEditPassword] + 1;
							if(CntEditNumPassword[CntEditPassword] == 9 + 1)
								CntEditNumPassword[CntEditPassword] = 0;
						}
						if(FlagSwDown_G == 1)
						{
							FlagSwDown_G = 0;
							if(CntEditNumPassword[CntEditPassword] == 0)
								CntEditNumPassword[CntEditPassword] = 9 + 1;
							CntEditNumPassword[CntEditPassword] = CntEditNumPassword[CntEditPassword] - 1;
						}
						memset(TempStr, 0, sizeof(TempStr));
						sprintf(TempStr,"%d",CntEditNumPassword[CntEditPassword]);
						LCD_Puts(7 + CntEditPassword,2,"");
						LCD_Put(CntEditNumPassword[CntEditPassword] + 48);

						LCD_Puts(7 + 0,2,"");
						LCD_Put(CntEditNumPassword[0] + 48);
						LCD_Puts(7 + 1,2,"");
						LCD_Put(CntEditNumPassword[1] + 48);
						LCD_Puts(7 + 2,2,"");
						LCD_Put(CntEditNumPassword[2] + 48);
						LCD_Puts(7 + 3,2,"");
						LCD_Put(CntEditNumPassword[3] + 48);
						LCD_Puts(7 + CntEditPassword,2,"");
					}
					else
					{
						LCD_Clear();
						LCD_CursorOff();
						LCD_BlinkOff();
						LCD_Puts(0, 0, "PVD = INF");
					}
					break;
				default:
					break;
				}
			}
			/**************************		  Woven fabric			   *****************/
			if(FlagMenu == 91)
			{
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
//					LCD_Clear();
					LCD_Puts(0,0,"Weft per centimeter");
					LCD_CursorOn();
					LCD_BlinkOn();

					if(FlagSwRight_G == 1)
					{
						FlagSwRight_G = 0;
						CntEditWeftPerCentimeter = CntEditWeftPerCentimeter + 1;
					}

					if(CntEditWeftPerCentimeter > 3 )
						CntEditWeftPerCentimeter = 0;

					if(FlagSwLeft_G == 1)
					{
						FlagSwLeft_G = 0;
						if(CntEditWeftPerCentimeter == 0)
							CntEditWeftPerCentimeter = 3 + 1;
						CntEditWeftPerCentimeter = CntEditWeftPerCentimeter - 1;
					}
					if(FlagSwUp_G == 1)
					{
						FlagSwUp_G = 0;
						CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] = CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] + 1;
						if(CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] == 9 + 1)
							CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] = 0;
					}
					if(FlagSwDown_G == 1)
					{
						FlagSwDown_G = 0;
						if(CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] == 0)
							CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] = 9 + 1;
						CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] = CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] - 1;
					}
					memset(TempStr, 0, sizeof(TempStr));
					sprintf(TempStr,"%d",CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter]);

					LCD_Puts(7 + 2,2,"/");
					LCD_Puts(7 + 0,2,"");
					LCD_Put(CntEditNumWeftPerCentimeter[0] + 48);
					LCD_Puts(7 + 1,2,"");
					LCD_Put(CntEditNumWeftPerCentimeter[1] + 48);
					LCD_Puts(8 + 2,2,"");
					LCD_Put(CntEditNumWeftPerCentimeter[2] + 48);
					LCD_Puts(8 + 3,2,"");
					LCD_Put(CntEditNumWeftPerCentimeter[3] + 48);

					if(CntEditWeftPerCentimeter < 2)
					{
						LCD_Puts(7 + CntEditWeftPerCentimeter,2,"");
						LCD_Put(CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] + 48);
						LCD_Puts(7 + CntEditWeftPerCentimeter,2,"");
					}

					if(CntEditWeftPerCentimeter > 1)
					{
						LCD_Puts(8 + CntEditWeftPerCentimeter,2,"");
						LCD_Put(CntEditNumWeftPerCentimeter[CntEditWeftPerCentimeter] + 48);
						LCD_Puts(8 + CntEditWeftPerCentimeter,2,"");
					}

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;

						FloatCntWeftPerCentimeter = 0;
						FloatTest = CntEditNumWeftPerCentimeter[2];
						FloatTest = (FloatTest / 10);
						FloatCntWeftPerCentimeter = FloatTest;
						FloatTest = CntEditNumWeftPerCentimeter[3];
						FloatTest = FloatTest / 100;
						FloatCntWeftPerCentimeter = FloatTest + FloatCntWeftPerCentimeter;
						FloatCntWeftPerCentimeter = CntEditNumWeftPerCentimeter[0] * 10 + CntEditNumWeftPerCentimeter[1] + FloatCntWeftPerCentimeter;
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65128));
						HAL_Delay(100);
						W25qxx_WritePage(CntEditNumWeftPerCentimeter, 65128, 0, 4);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable

//						ResetCMND();
//						MagnetCommand(0);
						LCD_CursorOff();
						LCD_BlinkOff();
					}
					break;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 12;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
					LCD_CursorOff();
					LCD_BlinkOff();
				}
			}
			/**************************		  RPM			   *****************/
			if(FlagMenu == 101)
			{
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
//					LCD_Clear();
					LCD_Puts(4,0,"Enter RPM");
					LCD_CursorOn();
					LCD_BlinkOn();

					if(FlagSwRight_G == 1)
					{
						FlagSwRight_G = 0;
						CntEditeRPMMenu = CntEditeRPMMenu + 1;
					}

					if(CntEditeRPMMenu > 2 )
						CntEditeRPMMenu = 0;

					if(FlagSwLeft_G == 1)
					{
						FlagSwLeft_G = 0;
						if(CntEditeRPMMenu == 0)
							CntEditeRPMMenu = 2 + 1;
						CntEditeRPMMenu = CntEditeRPMMenu - 1;
					}
					if(FlagSwUp_G == 1)
					{
						FlagSwUp_G = 0;
						CntEditNumRPMMenu[CntEditeRPMMenu] = CntEditNumRPMMenu[CntEditeRPMMenu] + 1;
						if(CntEditNumRPMMenu[CntEditeRPMMenu] == 9 + 1)
							CntEditNumRPMMenu[CntEditeRPMMenu] = 0;
					}
					if(FlagSwDown_G == 1)
					{
						FlagSwDown_G = 0;
						if(CntEditNumRPMMenu[CntEditeRPMMenu] == 0)
							CntEditNumRPMMenu[CntEditeRPMMenu] = 9 + 1;
						CntEditNumRPMMenu[CntEditeRPMMenu] = CntEditNumRPMMenu[CntEditeRPMMenu] - 1;
					}
					LCD_Puts(7 + 0,2,"");
					LCD_Put(CntEditNumRPMMenu[0] + 48);
					LCD_Puts(7 + 1,2,"");
					LCD_Put(CntEditNumRPMMenu[1] + 48);
					LCD_Puts(7 + 2,2,"");
					LCD_Put(CntEditNumRPMMenu[2] + 48);

					LCD_Puts(7 + CntEditeRPMMenu,2,"");
					LCD_Put(CntEditNumRPMMenu[CntEditeRPMMenu] + 48);
					LCD_Puts(7 + CntEditeRPMMenu,2,"");

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;


						NumRPMMenu = CntEditNumRPMMenu[0] * 100 + CntEditNumRPMMenu[1] * 10 + CntEditNumRPMMenu[2];
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65160));
						HAL_Delay(100);
						W25qxx_WritePage(CntEditNumRPMMenu, 65160, 0, 3);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable
						LCD_CursorOff();
						LCD_BlinkOff();
					}
					break;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 13;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
					LCD_CursorOff();
					LCD_BlinkOff();
				}
			}
			/**************************		  Woven Roll	   *****************/
			if(FlagMenu == 111)
			{
				StatusSubMenu_G = CntRN_G;
				switch(StatusSubMenu_G)
				{
				case 0:
					break;
				default:
					//					LCD_Clear();
					LCD_Puts(4,0,"Enter Woven Roll");
					LCD_CursorOn();
					LCD_BlinkOn();

					if(FlagSwRight_G == 1)
					{
						FlagSwRight_G = 0;
						CntEditeWovenRollMenu = CntEditeWovenRollMenu + 1;
					}

					if(CntEditeWovenRollMenu > 2 )
						CntEditeWovenRollMenu = 0;

					if(FlagSwLeft_G == 1)
					{
						FlagSwLeft_G = 0;
						if(CntEditeWovenRollMenu == 0)
							CntEditeWovenRollMenu = 2 + 1;
						CntEditeWovenRollMenu = CntEditeWovenRollMenu - 1;
					}
					if(FlagSwUp_G == 1)
					{
						FlagSwUp_G = 0;
						CntEditNumWovenRollMenu[CntEditeWovenRollMenu] = CntEditNumWovenRollMenu[CntEditeWovenRollMenu] + 1;
						if(CntEditNumWovenRollMenu[CntEditeWovenRollMenu] == 9 + 1)
							CntEditNumWovenRollMenu[CntEditeWovenRollMenu] = 0;
					}
					if(FlagSwDown_G == 1)
					{
						FlagSwDown_G = 0;
						if(CntEditNumWovenRollMenu[CntEditeWovenRollMenu] == 0)
							CntEditNumWovenRollMenu[CntEditeWovenRollMenu] = 9 + 1;
						CntEditNumWovenRollMenu[CntEditeWovenRollMenu] = CntEditNumWovenRollMenu[CntEditeWovenRollMenu] - 1;
					}
					LCD_Puts(7 + 0,2,"");
					LCD_Put(CntEditNumWovenRollMenu[0] + 48);
					LCD_Puts(7 + 1,2,"");
					LCD_Put(CntEditNumWovenRollMenu[1] + 48);
					LCD_Puts(7 + 2,2,"");
					LCD_Put(CntEditNumWovenRollMenu[2] + 48);

					LCD_Puts(7 + CntEditeWovenRollMenu,2,"");
					LCD_Put(CntEditNumWovenRollMenu[CntEditeWovenRollMenu] + 48);
					LCD_Puts(7 + CntEditeWovenRollMenu,2,"");

					if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
					{
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;


						NumWovenRollMenu = CntEditNumWovenRollMenu[0] * 100 + CntEditNumWovenRollMenu[1] * 10 + CntEditNumWovenRollMenu[2];
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65192));
						HAL_Delay(100);
						W25qxx_WritePage(CntEditNumWovenRollMenu, 65192, 0, 3);
						HAL_Delay(100);
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable
						LCD_CursorOff();
						LCD_BlinkOff();
					}
					break;
				}
				if(FlagSwF1_G == 1)
				{
					MaxMenu_G = MainMenu;
					FlagMenu  = 1;
					CntRN_G = 14;
					FlagRNSw_G = 0;
					FlagSwF2_G = 0;
					FlagSwF1_G = 0;
					FlagDisplay_G = 1;
					LCD_CursorOff();
					LCD_BlinkOff();
				}
			}
			/**************************       Supply Error       	   *****************/
			if(FlagMenu == 200)
			{
				LCD_Clear();
				LCD_Puts(0,0,"Supply Error(001) !!!");
				//				HAL_Delay(5000);
				//				HAL_NVIC_SystemReset();
			}
			/**************************       expiration Error    	   *****************/
			if(FlagMenu == 210)
			{
//				LCD_Clear();
				LCD_Puts(0,0,"Please Set Password!");
				memset(TempStr2, 0, sizeof(TempStr2));
				sprintf(TempStr2,"RNG = %d     ",RandomeValue);
				LCD_Puts(0, 2, TempStr2);
				LCD_CursorOn();
				LCD_BlinkOn();
				ORGPassword = A * RandomeValue + B;
				InfPassword = InfA * RandomeValue + InfB;
				if(FlagRNSw_G == 1 || FlagSwF2_G == 1)
				{
					NumPassword = CntEditNumPassword[0] * 1000 + CntEditNumPassword[1] * 100 + CntEditNumPassword[2] * 10 + CntEditNumPassword[3] * 1;
					if(NumPassword == ORGPassword || NumPassword == InfPassword)
					{
						if(NumPassword == InfPassword)
						{
							FlagExpiration_G = 0;
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
							W25qxx_WriteEnable();
							HAL_Delay(100);
							W25qxx_EraseSector(W25qxx_PageToSector(65096));
							HAL_Delay(100);
							memset(TempStr, 0, sizeof(TempStr));
							TempStr[0] = FlagExpiration_G;
							W25qxx_WritePage(TempStr, 65096, 0, 1);
							HAL_Delay(100);
							W25qxx_WriteDisable();
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);                 //  Write Protection Enable
						}
						else
						{
							FlagExpiration_G = 1;
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
							W25qxx_WriteEnable();
							HAL_Delay(100);
							W25qxx_EraseSector(W25qxx_PageToSector(65096));
							HAL_Delay(100);
							memset(TempStr, 0, sizeof(TempStr));
							TempStr[0] = FlagExpiration_G;
							W25qxx_WritePage(TempStr, 65096, 0, 1);
							HAL_Delay(100);
							W25qxx_WriteDisable();
							HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);
						}
						HAL_FLASH_Unlock();
						Cntexpiration_G = 0;
						if((EE_WriteVariable(VirtAddVarTab[1],  Cntexpiration_G)) != HAL_OK)
						{
							Error_Handler();
						}
						HAL_FLASH_Lock();

						memset(CntEditNumPassword, 0, sizeof(CntEditNumPassword));
						FlagExpireProgram = 0;

						LCD_CursorOff();
						LCD_BlinkOff();
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
						FlagDisplay_G = 1;
					}
					else
					{
						LCD_Puts(7,3,"Error");
						FlagRNSw_G = 0;
						FlagSwF2_G = 0;
						FlagSwF1_G = 0;
					}
				}


				if(FlagSwRight_G == 1)
				{
					FlagSwRight_G = 0;
					CntEditPassword = CntEditPassword + 1;
				}

				if(CntEditPassword == MaxCharPassword + 1)
					CntEditPassword = 0;

				if(FlagSwLeft_G == 1)
				{
					FlagSwLeft_G = 0;
					if(CntEditPassword == 0)
						CntEditPassword = MaxCharPassword + 1;
					CntEditPassword = CntEditPassword - 1;
				}

				if(FlagSwUp_G == 1)
				{
					FlagSwUp_G = 0;
					CntEditNumPassword[CntEditPassword] = CntEditNumPassword[CntEditPassword] + 1;
					if(CntEditNumPassword[CntEditPassword] == 9 + 1)
						CntEditNumPassword[CntEditPassword] = 0;
				}
				if(FlagSwDown_G == 1)
				{
					FlagSwDown_G = 0;
					if(CntEditNumPassword[CntEditPassword] == 0)
						CntEditNumPassword[CntEditPassword] = 9 + 1;
					CntEditNumPassword[CntEditPassword] = CntEditNumPassword[CntEditPassword] - 1;
				}
				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"%d",CntEditNumPassword[CntEditPassword]);
				LCD_Puts(7 + CntEditPassword,1,"");
				LCD_Put(CntEditNumPassword[CntEditPassword] + 48);
				LCD_Puts(7 + CntEditPassword,1,"");
				//				HAL_Delay(5000);
				//				HAL_NVIC_SystemReset();
			}
			/**************************		 Shift Menu					****************/
			if(FlagMenu == 220)
			{
				FlagShiftMenuDisplay = 1;
				if(ShiftStatus == DefineShiftA)
					LCD_Puts(0,0,"Shift= A");
				else if(ShiftStatus == DefineShiftB)
					LCD_Puts(0,0,"Shift= B");
				else if(ShiftStatus == DefineShiftC)
					LCD_Puts(0,0,"Shift= C");
				else if(ShiftStatus == DefineShiftD)
					LCD_Puts(0,0,"Shift= D");
				LCD_Puts(12,0,"E=");
				LCD_Puts(0,1,"Total KP=");
				LCD_Puts(0,2,"Time=");
				LCD_Puts(0,3,"Shift KP=");

				if(FlagLockMenu == 1)
				{
					MaxMenu_G = LockedMainMenu;
					LCD_PutCustom(19,0,0); //Lock Shape
				}
				if(FlagRNSw_G == 1 || FlagSwF1_G == 1)
				{
					FlagShiftMenuDisplay = 0;
					FlagMenu  = 1; //Magnet Display
					MaxMenu_G = MainMenu; // (2 - 6) for USBMenu
					CntRN_G = 1;
					FlagDisplay_G = 1;
					FlagSwF1_G = 0;
					FlagSwF2_G = 0;
					FlagRNSw_G = 0;
				}
			}
		}

		/***************************	RPM		**********************************/
		if(CntTimeRPM_G >= 3000 && (FlagMenu == 1 || FlagMenu == 220) && StatusMainMenu_G == 1 && FlagMagneticTrigger_G == 0)
		{
			RPM = 20 * CntRPM;
			CntTotalShiftRPM = CntRPM + CntTotalShiftRPM;
			FloatCntTotalShiftRPM = CntTotalShiftRPM;

			CntTotalRPM = CntRPM + CntTotalRPM;
			if(CntTotalRPM >= 100000000)
				CntTotalRPM = 0;
			FloatCntTotalRPM = CntTotalRPM;

			IntTest = (CntMinutes_G * NumRPMMenu + CntHour_G * 60 * NumRPMMenu) / 100;
			if(IntTest == 0)
				IntTest = 1;
			FloatTest = IntTest;
			FloatTest = FloatCntTotalShiftRPM /  FloatTest;
			Efficiency = FloatTest;
			if(Efficiency > 100)
				Efficiency = 100;

			CntRPM = 0;
			CntTimeRPM_G = 0;
			if(FlagShiftMenuDisplay == 0)
			{

				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"RPM = %d  ",RPM);
				LCD_Puts(0,2,TempStr);
			}
			if(FlagShiftMenuDisplay == 1)
			{
				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"%.1f ",FloatCntTotalRPM / 1000);
				LCD_Puts(10,1,TempStr);

				memset(TempStr, 0, sizeof(TempStr));
				if(CntHour_G < 10)
					sprintf(TempStr,"0%d",CntHour_G);
				else if(CntHour_G > 9)
					sprintf(TempStr,"%d",CntHour_G);
				LCD_Puts(6, 2, TempStr);

				LCD_Puts(8, 2, ":");

				memset(TempStr, 0, sizeof(TempStr));
				if(CntMinutes_G < 10)
					sprintf(TempStr,"0%d",CntMinutes_G);
				else if(CntMinutes_G > 9)
					sprintf(TempStr,"%d",CntMinutes_G);
				LCD_Puts(9, 2, TempStr);

				memset(TempStr, 0, sizeof(TempStr));
				sprintf(TempStr,"%.1f   ",FloatCntTotalShiftRPM / 1000);
				LCD_Puts(10,3,TempStr);


				if(Efficiency < 10)
					sprintf(TempStr,"%d%%  ",Efficiency);
				else if(Efficiency > 10 && Efficiency < 100)
					sprintf(TempStr,"%d%% ",Efficiency);
				else if(Efficiency == 100)
					sprintf(TempStr,"%d%%",Efficiency);
				LCD_Puts(15, 0, TempStr);

			}
		}
		/*************************************************************************/
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
		if(FlagProcessUSB == 1)
		{
			MX_USB_HOST_Process();
			if(CntDelayTimer_G > 6000)
				FlagMountUSB = 1;
		}

		if(FlagMountUSB == 1)
		{
			Res = f_mount(&mynewdiskFatFs, (TCHAR const*)USBHPath, 0);
			if(Res == FR_OK)
			{
				/**********************         Write To USB    ****************/
				if(FlagWriteUSB == 1)
				{
					FlagWriteUSB = 0;
					FlagReadUSB = 0;

					memset(BackupFlagMenuUsb, 0, sizeof(BackupFlagMenuUsb));
					BackupFlagMenuUsb[0]  = 1;
					W25qxx_WriteEnable();
					HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
					HAL_Delay(100);
					W25qxx_EraseSector(W25qxx_PageToSector(64000));
					HAL_Delay(100);
					W25qxx_WritePage(BackupFlagMenuUsb, 64000, 0, 3);
					W25qxx_WriteDisable();
					HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);               //  Write Protection Enable

					FlagSelectMenuUsb = 0;
					Res = f_open(&MyFile, "Output.txt", FA_CREATE_ALWAYS | FA_WRITE);
					if(Res == FR_OK)
					{
						LCD_Clear();
						LCD_Puts(0, 0, "USB Connect ");
						LCD_Puts(0, 1, "Successfully ...");

						HAL_Delay(2000);

						LCD_Clear();
						LCD_Puts(0,0,"Read From Memory ...");

						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);//  Write Protection Enable

						HAL_Delay(2000);
						memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						for(int j = 0; j < CntProgramName; j++)
						{
							for(int i = 0; i < MaxLine; i++)
							{
								memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
								W25qxx_ReadPage(TemporaryBuffer, ArrayAdressProgram[j] + i / 8, (i % 8) * MaxCharacter, MaxCharacter);
								if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
									break;
								f_puts(TemporaryBuffer,&MyFile);
							}
						}
						f_puts("END",&MyFile);
						f_close(&MyFile);
						LCD_Clear();
						LCD_Puts(0, 0, "Write To USB ");
						LCD_Puts(0, 1, "Successfully ...");

						USBH_LL_Disconnect(&hUsbHostFS) ;
						MX_USB_HOST_Process();
						HAL_Delay(3000);
						FlagMountUSB = 0;
						FlagProcessUSB = 0;
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagDisplay_G = 1;


					}
					else
					{
						LCD_Clear();
						LCD_Puts(0,0,"USB Not Connected");
						USBH_LL_Disconnect(&hUsbHostFS) ;
						MX_USB_HOST_Process();
						HAL_Delay(3000);
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagDisplay_G = 1;
						FlagProcessUSB = 0;
						FlagMountUSB = 0;
					}
				}
				/****************************************************************/
				/**********************         Read From USB    ****************/
				if(FlagReadUSB == 1)
				{
					FlagWriteUSB = 0;
					FlagReadUSB = 0;

					memset(BackupFlagMenuUsb, 0, sizeof(BackupFlagMenuUsb));
					BackupFlagMenuUsb[0]  = 1;
					W25qxx_WriteEnable();
					HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
					HAL_Delay(100);
					W25qxx_EraseSector(W25qxx_PageToSector(64000));
					HAL_Delay(100);
					W25qxx_WritePage(BackupFlagMenuUsb, 64000, 0, 3);
					W25qxx_WriteDisable();
					HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);               //  Write Protection Enable

					FlagSelectMenuUsb = 0;
					Res = f_open(&MyFile, "Raya.txt", FA_READ);
					if(Res == FR_OK)
					{
						LCD_Clear();
						LCD_Puts(0, 0, "USB Connect");
						LCD_Puts(0, 1, "Successfully ...");

						HAL_Delay(2000);

						LCD_Clear();
						LCD_Puts(0,0,"Chip Erasing ...");
						HAL_Delay(1000);
						W25qxx_EraseChip();

						LCD_Clear();
						LCD_Puts(0,0,"Read From USB ...");
						W25qxx_WriteEnable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						HAL_Delay(2000);

						SelectedProgram[0] = 0;
						W25qxx_WritePage(SelectedProgram, 65000, 0, 1);

						memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
						for(int i = 0; i < MaxProgram; i++)
							memset(ArrayProgramName[i], 0, sizeof(ArrayProgramName[i]));
						CntProgramName = 0;
						FlagTemporary = 0;
						for(int j = 0; j < MaxProgram; j++)
						{
							if(FlagTemporary == 1)
							{
								test2 = ArrayAdressSector[j] * 16 + 1;
								W25qxx_WritePage(TemporaryBuffer, test2, 0, MaxCharacter);
							}
							for(int i = FlagTemporary; i < MaxLine; i++)
							{
								memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
								f_gets(TemporaryBuffer, MaxCharacter, &MyFile);

								if(TemporaryBuffer[0] == 'P' || TemporaryBuffer[0] == 'p')
								{
									strncpy(ArrayProgramName[CntProgramName], TemporaryBuffer, MaxCharacter);
									CntProgramName = CntProgramName + 1;
									if(FlagTemporary == 1)
									{
										test1 = (i % 8) * MaxCharacter;
										test2 = ArrayAdressSector[j] * 16 + i / 8 + 1;
										W25qxx_WritePage("Continue", test2, test1, 8);
										break;
									}

									FlagTemporary = 1;
								}

								if(TemporaryBuffer[0] == 'E' || TemporaryBuffer[0] == 'e')
								{
									test1 = (i % 8) * MaxCharacter;
									test2 = ArrayAdressSector[j] * 16 + i / 8 + 1;
									W25qxx_WritePage("Continue", test2, test1, 8);
									j = MaxProgram;
									break;
								}
								test1 = (i % 8) * MaxCharacter;
								test2 = ArrayAdressSector[j] * 16 + i / 8 + 1;
								W25qxx_WritePage(TemporaryBuffer, test2, test1, MaxCharacter);
							}
						}
						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);               //  Write Protection Enable

						f_close(&MyFile);
						LCD_Clear();
						LCD_Puts(0, 0, "Read From USB");
						LCD_Puts(0, 1, "Successfully ...");

						USBH_LL_Disconnect(&hUsbHostFS) ;
						MX_USB_HOST_Process();
						BackupCntProgramLine = 0;
						HAL_Delay(2000);

						LCD_Clear();
						LCD_Puts(0,0,"Initialize Buffer ...");
						InitializeBuffer();

						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_SET);                 //  Write Protection Disable
						W25qxx_WriteEnable();
						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65096));
						HAL_Delay(100);
						memset(TempStr, 0, sizeof(TempStr));
						TempStr[0] = FlagExpiration_G;
						W25qxx_WritePage(TempStr, 65096, 0, 1);
						HAL_Delay(100);


						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65128));
						HAL_Delay(100);
						W25qxx_WritePage(CntEditNumWeftPerCentimeter, 65128, 0, 4);
						HAL_Delay(100);

						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65160));
						HAL_Delay(100);
						W25qxx_WritePage(CntEditNumRPMMenu, 65160, 0, 3);
						HAL_Delay(100);

						HAL_Delay(100);
						W25qxx_EraseSector(W25qxx_PageToSector(65192));
						HAL_Delay(100);
						W25qxx_WritePage(CntEditNumWovenRollMenu, 65192, 0, 3);
						HAL_Delay(100);

						W25qxx_WriteDisable();
						HAL_GPIO_WritePin(GPIOC, SPI_WP_Pin, GPIO_PIN_RESET);

						HAL_Delay(1000);
						BackupCntProgramLine = 0;
						MagnetCommand(0);
						FlagMountUSB = 0;
						FlagProcessUSB = 0;
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagDisplay_G = 1;
					}
					else
					{
						LCD_Clear();
						LCD_Puts(0,0,"USB Not Connected");
						USBH_LL_Disconnect(&hUsbHostFS) ;
						MX_USB_HOST_Process();
						HAL_Delay(3000);
						MaxMenu_G = MainMenu;
						FlagMenu  = 1;
						CntRN_G = 1;
						FlagDisplay_G = 1;
						FlagProcessUSB = 0;
						FlagMountUSB = 0;
					}
				}
				/****************************************************************/
			}
		}

		/***************	FlagTroubleshooting Page 1		****************/
		if(FlagTroubleshooting == 1 && FlagUpdateTroubleshooting_G == 1)
		{
			FlagUpdateTroubleshooting_G = 0;
			//			LCD_Clear();
			if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_SET)
				LCD_Puts(0,0,"S1 = 1");
			if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_RESET)
				LCD_Puts(0,0,"S1 = 0");

			if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_SET)
				LCD_Puts(10,0,"S2 = 1");
			if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_RESET)
				LCD_Puts(10,0,"S2 = 0");

			if(HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin) == GPIO_PIN_SET)
				LCD_Puts(0,1,"Start = 1");
			if(HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin) == GPIO_PIN_RESET)
				LCD_Puts(0,1,"Start = 0");

			if(HAL_GPIO_ReadPin(Reverse_GPIO_Port,Reverse_Pin) == GPIO_PIN_SET)
				LCD_Puts(10,1,"Reverse =1");
			if(HAL_GPIO_ReadPin(Reverse_GPIO_Port,Reverse_Pin) == GPIO_PIN_RESET)
				LCD_Puts(10,1,"Reverse =0");

			if(RotationStatus_G == Right)
				LCD_Puts(0,2,"D = Right");
			if(RotationStatus_G == Left)
				LCD_Puts(0,2,"D = Left ");

			switch(Phi_G)
			{
			case 0:
				LCD_Puts(10,2,"Phi = 0 ");
				break;
			case 1:
				LCD_Puts(10,2,"Phi = 1");
				break;
			case 2:
				LCD_Puts(10,2,"Phi = 2");
				break;
			case 3:
				LCD_Puts(10,2,"Phi = 3");
				break;
			default:
				break;
			}
			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"%d  ",BackupCntProgramLine + 1);
			LCD_Puts(5,3,TempStr);
			LCD_Puts(0,3,"LN =");

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"RPM=%d   ",RPM);
			LCD_Puts(10,3,TempStr);
		}
		/***************	FlagTroubleshooting Page 2		****************/
		if(FlagTroubleshooting == 2 && FlagUpdateTroubleshooting_G == 1)
		{
			FlagUpdateTroubleshooting_G = 0;
			//			LCD_Clear();
			//NR0To3_G, NR2To1_G, NR1To0_G, NR3To2_G, NS3To0_G, NS1To2_G, NS0To1_G, NS2To3_G, N_G
			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"R03=%d ",NR0To3_G);
			LCD_Puts(0,0,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"R21=%d ",NR2To1_G);
			LCD_Puts(7,0,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"R10=%d ",NR1To0_G);
			LCD_Puts(14,0,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"R32=%d ",NR3To2_G);
			LCD_Puts(0,1,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"S30=%d ",NS3To0_G);
			LCD_Puts(7,1,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"S12=%d ",NS1To2_G);
			LCD_Puts(14,1,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"S01=%d ",NS0To1_G);
			LCD_Puts(0,2,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"S23=%d ",NS2To3_G);
			LCD_Puts(7,2,TempStr);

			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"NG=%d ",N_G);
			LCD_Puts(14,2,TempStr);

			switch(Phi_G)
			{
			case 0:
				LCD_Puts(0,3,"P=0(30-270)        ");
				break;
			case 1:
				LCD_Puts(0,3,"P=1(270-300||330)  ");
				break;
			case 2:
				LCD_Puts(0,3,"P=2(300-330||330-0)");
				break;
			case 3:
				LCD_Puts(0,3,"P=3(330-30||0-30)  ");
				break;
			default:
				break;
			}
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
bool InitializeBuffer (void)
{
	CntProgramLine = 0;
	CntProgramName = 0;
	FlagTemporary = 0;
	memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
	memset(ArrayProgramLine, 0, sizeof(ArrayProgramLine));
	memset(ArrayProgramName, 0, sizeof(ArrayProgramName));
	memset(ArrayAdressProgram, 0, sizeof(ArrayAdressProgram));
	memset(SelectedProgram, 0, sizeof(SelectedProgram));
	for(int j = 0; j < MaxProgram; j++)
	{
		for(long int i = 0; i < MaxLine; i++)
		{
			test1 = (i % 8) * MaxCharacter;
			test2 = ArrayAdressSector[j] * 16 + i / 8 + 1;
			W25qxx_ReadPage(TemporaryBuffer, test2, test1, MaxCharacter);
			if(TemporaryBuffer[0] != 'P' && TemporaryBuffer[0] != 'p' && i == 0)
			{
				j = MaxProgram;
				break;
			}
			else if(TemporaryBuffer[0] == 'P' || TemporaryBuffer[0] == 'p')
			{
				strncpy(ArrayProgramName[CntProgramName], TemporaryBuffer, MaxCharacter);
				ArrayAdressProgram[CntProgramName] = ArrayAdressSector[j] * 16 + i / 8 + 1;

				//				if(FlagTemporary == 1)
				//					ArrayCntProgramLine[CntProgramName - 1] = CntProgramLine - 1;

				CntProgramName = CntProgramName + 1;
				//				FlagTemporary = 1;
				CntProgramLine = 0;
			}

			if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
			{
				ArrayCntProgramLine[CntProgramName - 1] = CntProgramLine - 1;
				break;
			}
			memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));

			//			if(FlagTemporary == 1)
			CntProgramLine = CntProgramLine + 1;
		}
	}
	/********************	Sampling Pattern	*********************/
	for(int i = 0; i < MaxNameSamplingPattern; i++)
		strncpy(ArrayProgramName[MaxProgram + i], ArrayNameSamplingPattern[i], MaxCharNameSamplingPattern);
	/****************************************************************/


	W25qxx_ReadPage(SelectedProgram, 65000, 0, 1);
	if(SelectedProgram[0] > CntProgramName && SelectedProgram[0] == 0xff)
		SelectedProgram[0] = 0;
	if(SelectedProgram[0] < CntProgramName)
	{
		CntProgramLine = 0;
		if(CntProgramName > 0)
		{
			for(long int i = 1; i < MaxLine; i++)
			{
				test1 = (i % 8) * MaxCharacter;
				W25qxx_ReadPage(TemporaryBuffer, ArrayAdressProgram[SelectedProgram[0]] + i / 8 , test1, MaxCharacter);
				if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
				{
					i = MaxLine ;
					break;
				}
				strncpy(ArrayProgramLine[CntProgramLine], TemporaryBuffer, MaxMagnet);
				CntProgramLine = CntProgramLine + 1;
				memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
			}
		}
	}
	else if(SelectedProgram[0] > CntProgramName && SelectedProgram[0] < 0xff)
	{
		CntProgramLine = 0;
		memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
		TemporaryBuffer[0] = SelectedProgram[0] - MaxProgram;
		if(TemporaryBuffer[0] >= MaxNameSamplingPattern || TemporaryBuffer[0] < 0)
			TemporaryBuffer[0] = 2;
		for(int i = 0; i < ArrayLineNumberSamplingPattern[TemporaryBuffer[0]]; i++)
		{
			strncpy(ArrayProgramLine[i], ArrayLineSamplingPattern[TemporaryBuffer[0]][i], MaxMagnet);
			CntProgramLine = CntProgramLine + 1;
		}
	}
	//	W25qxx_ReadPage(LastLineNumber, 65001, 0, 4); // BackupCntProgramLine
	//	BackupCntProgramLine = atoi(LastLineNumber);
}

bool InitializeBuffer2 (void)
{
	int j2;
	CntProgramLine = 0;
	CntProgramName = 0;
	FlagTemporary = 0;
//	for(int j = 0; j < MaxProgram; j++)
//	{
	W25qxx_ReadPage(SelectedProgram, 65000, 0, 1);
	if(SelectedProgram[0] >= MaxProgram + MaxNameSamplingPattern)
		SelectedProgram[0] = 0;

	if(SelectedProgram[0] < MaxProgram)
	{
		j2 = SelectedProgram[0];
		for(long int i = 0; i < MaxLine; i++)
		{
			test1 = (i % 8) * MaxCharacter;
			test2 = ArrayAdressSector[j2] * 16 + i / 8 + 1;
			W25qxx_ReadPage(TemporaryBuffer, test2, test1, MaxCharacter);
			if(TemporaryBuffer[0] != 'P' && TemporaryBuffer[0] != 'p' && i == 0)
			{
				//			j = MaxProgram;
				break;
			}
			else if(TemporaryBuffer[0] == 'P' || TemporaryBuffer[0] == 'p')
			{
				strncpy(ArrayProgramName[SelectedProgram[0]], TemporaryBuffer, MaxCharacter);
				ArrayAdressProgram[SelectedProgram[0]] = ArrayAdressSector[j2] * 16 + i / 8 + 1;

				//				if(FlagTemporary == 1)
				//					ArrayCntProgramLine[CntProgramName - 1] = CntProgramLine - 1;

				CntProgramName = CntProgramName + 1;
				//				FlagTemporary = 1;
				CntProgramLine = 0;
			}

			if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
			{
				ArrayCntProgramLine[CntProgramName - 1] = CntProgramLine - 1;
				break;
			}
			memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));

			//			if(FlagTemporary == 1)
			CntProgramLine = CntProgramLine + 1;
		}
	}
//	}
	/********************	Sampling Pattern	*********************/
	for(int i = 0; i < MaxNameSamplingPattern; i++)
		strncpy(ArrayProgramName[MaxProgram + i], ArrayNameSamplingPattern[i], MaxCharNameSamplingPattern);
	/****************************************************************/


//	W25qxx_ReadPage(SelectedProgram, 65000, 0, 1);
//	if(SelectedProgram[0] > CntProgramName && SelectedProgram[0] == 0xff)
//		SelectedProgram[0] = 0;
	if(SelectedProgram[0] < MaxProgram)
	{
		CntProgramLine = 0;
		if(CntProgramName > 0)
		{
			for(long int i = 1; i < MaxLine; i++)
			{
				test1 = (i % 8) * MaxCharacter;
				W25qxx_ReadPage(TemporaryBuffer, ArrayAdressProgram[SelectedProgram[0]] + i / 8 , test1, MaxCharacter);
				if(TemporaryBuffer[0] == 'C' || TemporaryBuffer[0] == 'c')
				{
					i = MaxLine ;
					break;
				}
				strncpy(ArrayProgramLine[CntProgramLine], TemporaryBuffer, MaxMagnet);
				CntProgramLine = CntProgramLine + 1;
				memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
			}
		}
	}
	else if(SelectedProgram[0] > MaxProgram && SelectedProgram[0] <= 0xff)
	{
		CntProgramLine = 0;
		memset(TemporaryBuffer, 0, sizeof(TemporaryBuffer));
		TemporaryBuffer[0] = SelectedProgram[0] - MaxProgram;
		if(TemporaryBuffer[0] >= MaxNameSamplingPattern || TemporaryBuffer[0] < 0)
			TemporaryBuffer[0] = 2;
		for(int i = 0; i < ArrayLineNumberSamplingPattern[TemporaryBuffer[0]]; i++)
		{
			strncpy(ArrayProgramLine[i], ArrayLineSamplingPattern[TemporaryBuffer[0]][i], MaxMagnet);
			CntProgramLine = CntProgramLine + 1;
		}
	}
	//	W25qxx_ReadPage(LastLineNumber, 65001, 0, 4); // BackupCntProgramLine
	//	BackupCntProgramLine = atoi(LastLineNumber);
}

void MagnetCommand (int8_t ShiftArrayProgramLine)
{
	BackupCntProgramLine = BackupCntProgramLine + ShiftArrayProgramLine;
	if(BackupCntProgramLine >= CntProgramLine)
		BackupCntProgramLine = BackupCntProgramLine - CntProgramLine;

	if(BackupCntProgramLine >= CntProgramLine)
		BackupCntProgramLine = BackupCntProgramLine - CntProgramLine;

	if(BackupCntProgramLine >= CntProgramLine)
		BackupCntProgramLine = BackupCntProgramLine - CntProgramLine;

	if(BackupCntProgramLine < 0)
		BackupCntProgramLine = BackupCntProgramLine + CntProgramLine;

	if(BackupCntProgramLine < 0)
		BackupCntProgramLine = BackupCntProgramLine + CntProgramLine;

	if(BackupCntProgramLine < 0)
		BackupCntProgramLine = BackupCntProgramLine + CntProgramLine;

	if(FlagMagnetDirection == 1)
	{
		for(int j = 0; j < CntActiveMagnet + 1; j++)
		{
			switch(j)
			{
			case 0:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND1_GPIO_Port, CMND1_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND1_GPIO_Port, CMND1_Pin, LocalSet_Pin);
				break;
			case 1:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND2_GPIO_Port, CMND2_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND2_GPIO_Port, CMND2_Pin, LocalSet_Pin);
				break;
			case 2:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND3_GPIO_Port, CMND3_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND3_GPIO_Port, CMND3_Pin, LocalSet_Pin);
				break;
			case 3:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND4_GPIO_Port, CMND4_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND4_GPIO_Port, CMND4_Pin, LocalSet_Pin);
				break;
			case 4:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND5_GPIO_Port, CMND5_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND5_GPIO_Port, CMND5_Pin, LocalSet_Pin);
				break;
			case 5:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND6_GPIO_Port, CMND6_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND6_GPIO_Port, CMND6_Pin, LocalSet_Pin);
				break;
			case 6:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND7_GPIO_Port, CMND7_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND7_GPIO_Port, CMND7_Pin, LocalSet_Pin);
				break;
			case 7:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND8_GPIO_Port, CMND8_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND8_GPIO_Port, CMND8_Pin, LocalSet_Pin);
				break;
			case 8:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND9_GPIO_Port, CMND9_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND9_GPIO_Port, CMND9_Pin, LocalSet_Pin);
				break;
			case 9:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND10_GPIO_Port, CMND10_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND10_GPIO_Port, CMND10_Pin, LocalSet_Pin);
				break;
			case 10:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND11_GPIO_Port, CMND11_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND11_GPIO_Port, CMND11_Pin, LocalSet_Pin);
				break;
			case 11:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND12_GPIO_Port, CMND12_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND12_GPIO_Port, CMND12_Pin, LocalSet_Pin);
				break;
			case 12:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND13_GPIO_Port, CMND13_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND13_GPIO_Port, CMND13_Pin, LocalSet_Pin);
				break;
			case 13:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND14_GPIO_Port, CMND14_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND14_GPIO_Port, CMND14_Pin, LocalSet_Pin);
				break;
			case 14:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND15_GPIO_Port, CMND15_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND15_GPIO_Port, CMND15_Pin, LocalSet_Pin);
				break;
			case 15:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND16_GPIO_Port, CMND16_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND16_GPIO_Port, CMND16_Pin, LocalSet_Pin);
				break;
			case 16:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND17_GPIO_Port, CMND17_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND17_GPIO_Port, CMND17_Pin, LocalSet_Pin);
				break;
			case 17:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND18_GPIO_Port, CMND18_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND18_GPIO_Port, CMND18_Pin, LocalSet_Pin);
				break;
			case 18:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND19_GPIO_Port, CMND19_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND19_GPIO_Port, CMND19_Pin, LocalSet_Pin);
				break;
			case 19:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND20_GPIO_Port, CMND20_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND20_GPIO_Port, CMND20_Pin, LocalSet_Pin);
				break;
			case 20:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND21_GPIO_Port, CMND21_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND21_GPIO_Port, CMND21_Pin, LocalSet_Pin);
				break;
			case 21:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND22_GPIO_Port, CMND22_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND22_GPIO_Port, CMND22_Pin, LocalSet_Pin);
				break;
			case 22:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND23_GPIO_Port, CMND23_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND23_GPIO_Port, CMND23_Pin, LocalSet_Pin);
				break;
			case 23:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND24_GPIO_Port, CMND24_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND24_GPIO_Port, CMND24_Pin, LocalSet_Pin);
				break;
			case 24:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND25_GPIO_Port, CMND25_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND25_GPIO_Port, CMND25_Pin, LocalSet_Pin);
				break;
			case 25:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND26_GPIO_Port, CMND26_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND26_GPIO_Port, CMND26_Pin, LocalSet_Pin);
				break;
			case 26:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND27_GPIO_Port, CMND27_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND27_GPIO_Port, CMND27_Pin, LocalSet_Pin);
				break;
			case 27:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND28_GPIO_Port, CMND28_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND28_GPIO_Port, CMND28_Pin, LocalSet_Pin);
				break;
			default:
				break;
			}
		}
	}
//	for(int j = 0; j < MaxColorWeaving; j++)
//	{
//		switch(j)
//		{
//		case 0:
//		case 1:
//		case 2:
//			if(ArrayProgramLine[BackupCntProgramLine][MaxMagnet + j] == '0')
//				HAL_GPIO_WritePin(GPIOC, TCMNDPin[j], LocalReset_Pin);
//			if(ArrayProgramLine[BackupCntProgramLine][MaxMagnet + j] == '1')
//				HAL_GPIO_WritePin(GPIOC, TCMNDPin[j], LocalSet_Pin);
//			break;
//		case 3:
//			if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
//				HAL_GPIO_WritePin(GPIOA, TCMNDPin[j], LocalReset_Pin);
//			if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
//				HAL_GPIO_WritePin(GPIOA, TCMNDPin[j], LocalSet_Pin);
//			break;
//		}
//	}
	if(FlagMagnetDirection == 0)
	{
		for(int j = 0; j < CntActiveMagnet + 1; j++)
		{
			switch(j)
			{
			case 0:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND28_GPIO_Port, CMND28_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND28_GPIO_Port, CMND28_Pin, LocalSet_Pin);
				break;
			case 1:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND27_GPIO_Port, CMND27_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND27_GPIO_Port, CMND27_Pin, LocalSet_Pin);
				break;
			case 2:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND26_GPIO_Port, CMND26_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND26_GPIO_Port, CMND26_Pin, LocalSet_Pin);
				break;
			case 3:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND25_GPIO_Port, CMND25_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND25_GPIO_Port, CMND25_Pin, LocalSet_Pin);
				break;
			case 4:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND24_GPIO_Port, CMND24_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND24_GPIO_Port, CMND24_Pin, LocalSet_Pin);
				break;
			case 5:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND23_GPIO_Port, CMND23_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND23_GPIO_Port, CMND23_Pin, LocalSet_Pin);
				break;
			case 6:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND22_GPIO_Port, CMND22_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND22_GPIO_Port, CMND22_Pin, LocalSet_Pin);
				break;
			case 7:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND21_GPIO_Port, CMND21_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND21_GPIO_Port, CMND21_Pin, LocalSet_Pin);
				break;
			case 8:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND20_GPIO_Port, CMND20_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND20_GPIO_Port, CMND20_Pin, LocalSet_Pin);
				break;
			case 9:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND19_GPIO_Port, CMND19_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND19_GPIO_Port, CMND19_Pin, LocalSet_Pin);
				break;
			case 10:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND18_GPIO_Port, CMND18_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND18_GPIO_Port, CMND18_Pin, LocalSet_Pin);
				break;
			case 11:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND17_GPIO_Port, CMND17_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND17_GPIO_Port, CMND17_Pin, LocalSet_Pin);
				break;
			case 12:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND16_GPIO_Port, CMND16_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND16_GPIO_Port, CMND16_Pin, LocalSet_Pin);
				break;
			case 13:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND15_GPIO_Port, CMND15_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND15_GPIO_Port, CMND15_Pin, LocalSet_Pin);
				break;
			case 14:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND14_GPIO_Port, CMND14_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND14_GPIO_Port, CMND14_Pin, LocalSet_Pin);
				break;
			case 15:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND13_GPIO_Port, CMND13_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND13_GPIO_Port, CMND13_Pin, LocalSet_Pin);
				break;
			case 16:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND12_GPIO_Port, CMND12_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND12_GPIO_Port, CMND12_Pin, LocalSet_Pin);
				break;
			case 17:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND11_GPIO_Port, CMND11_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND11_GPIO_Port, CMND11_Pin, LocalSet_Pin);
				break;
			case 18:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND10_GPIO_Port, CMND10_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND10_GPIO_Port, CMND10_Pin, LocalSet_Pin);
				break;
			case 19:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND9_GPIO_Port, CMND9_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND9_GPIO_Port, CMND9_Pin, LocalSet_Pin);
				break;
			case 20:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND8_GPIO_Port, CMND8_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND8_GPIO_Port, CMND8_Pin, LocalSet_Pin);
				break;
			case 21:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND7_GPIO_Port, CMND7_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND7_GPIO_Port, CMND7_Pin, LocalSet_Pin);
				break;
			case 22:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND6_GPIO_Port, CMND6_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND6_GPIO_Port, CMND6_Pin, LocalSet_Pin);
				break;
			case 23:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND5_GPIO_Port, CMND5_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND5_GPIO_Port, CMND5_Pin, LocalSet_Pin);
				break;
			case 24:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND4_GPIO_Port, CMND4_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND4_GPIO_Port, CMND4_Pin, LocalSet_Pin);
				break;
			case 25:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND3_GPIO_Port, CMND3_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND3_GPIO_Port, CMND3_Pin, LocalSet_Pin);
				break;
			case 26:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND2_GPIO_Port, CMND2_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND2_GPIO_Port, CMND2_Pin, LocalSet_Pin);
				break;
			case 27:
				if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
					HAL_GPIO_WritePin(CMND1_GPIO_Port, CMND1_Pin, LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
					HAL_GPIO_WritePin(CMND1_GPIO_Port, CMND1_Pin, LocalSet_Pin);
				break;
			default:
				break;
			}
		}
	}

	if((FlagMenu == 220 || FlagMenu == 1) && StatusMainMenu_G == 1)
	{
		if(FlagMenu == 1)
		{
			memset(TempStr, 0, sizeof(TempStr));
			sprintf(TempStr,"%d  ",BackupCntProgramLine + 1);
			LCD_Puts(3,3,TempStr);
		}
		if(FlagRPM_G == 1)
		{
			FlagRPM_G = 0;
			CntRPM = CntRPM + 1;
			if(RotationStatus_G == Right)
			{
				FloatWeftPerCentimeter = 1 / FloatCntWeftPerCentimeter + FloatWeftPerCentimeter;
				if(FlagMenu == 1)
				{
					if(FloatWeftPerCentimeter / 100 > NumWovenRollMenu)
						HAL_GPIO_WritePin(Watchdog_GPIO_Port, Watchdog_Pin, GPIO_PIN_RESET);
					if(FlagActiveWovenRoll == 1)
					{
						LCD_Puts(9, 3, "WR=");
						memset(TempStr, 0, sizeof(TempStr));
						sprintf(TempStr,"%.2f  ",FloatWeftPerCentimeter / 100);
						LCD_Puts(12,3,TempStr);
					}
					else
					{
						LCD_Puts(9, 3, "WT=");
						memset(TempStr, 0, sizeof(TempStr));
						sprintf(TempStr,"%.2f  ",FloatWeftPerCentimeter / 100);
						LCD_Puts(12,3,TempStr);
					}
				}

			}
		}
	}
}

void ResetCMND(void)
{
	for(int j2 = 0; j2 < MaxMagnet; j2++)
	{
		switch(j2)
		{
		case 0:
		case 1:
		case 2:
		case 25:
		case 26:
		case 27:
			HAL_GPIO_WritePin(GPIOB, CMNDPin[j2], GPIO_PIN_RESET);
			break;
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
			HAL_GPIO_WritePin(GPIOD, CMNDPin[j2], GPIO_PIN_RESET);
			break;
		case 11:
		case 12:
		case 13:
			HAL_GPIO_WritePin(GPIOC, CMNDPin[j2], GPIO_PIN_RESET);
			break;
		case 14:
		case 15:
			HAL_GPIO_WritePin(GPIOA, CMNDPin[j2], GPIO_PIN_RESET);
			break;
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
		case 24:
			HAL_GPIO_WritePin(GPIOE, CMNDPin[j2], GPIO_PIN_RESET);
			break;
		default:
			break;
		}
	}
}
void MagnetDisplay(void)
{
	LCD_Clear();
	for(int j3 = 0; j3 < CntActiveMagnet; j3++)
	{
		switch(j3)
		{
		case 0:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(0,0,"1");
			break;
		case 1:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(3,0,"2");
			break;
		case 2:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(6,0,"3");
			break;
		case 3:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(9,0,"4");
			break;
		case 4:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(12,0,"5");
			break;
		case 5:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(15,0,"6");
			break;
		case 6:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(18,0,"7");
			break;
		case 7:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(0,1,"8");
			break;
		case 8:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(3,1,"9");
			break;
		case 9:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(6,1,"10");
			break;
		case 10:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(9,1,"11");
			break;
		case 11:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(12,1,"12");
			break;
		case 12:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(15,1,"13");
			break;
		case 13:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(18,1,"14");
			break;
		case 14:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(0,2,"15");
			break;
		case 15:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(3,2,"16");
			break;
		case 16:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(6,2,"17");
			break;
		case 17:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(9,2,"18");
			break;
		case 18:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(12,2,"19");
			break;
		case 19:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(15,2,"20");
			break;
		case 20:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(18,2,"21");
			break;
		case 21:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(0,3,"22");
			break;
		case 22:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(3,3,"23");
			break;
		case 23:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(6,3,"24");
			break;
		case 24:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(9,3,"25");
			break;
		case 25:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(12,3,"26");
			break;
		case 26:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(15,3,"27");
			break;
		case 27:
			if(ArrayProgramLine[BackupCntProgramLine][j3] == '1')
				LCD_Puts(18,3,"28");
			break;
		default:
			break;
		}
	}
}

void MagneSetAndReset(uint8_t CntMagnetSetAndReset)
{
	switch(CntMagnetSetAndReset - 1)
	{
	case 0:
		LCD_Puts(11, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(11, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(11, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(11, 0, "1");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(11, 0, "");
		}
		break;
	case 1:
		LCD_Puts(12, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(12, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(12, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(12, 0, "2");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(12, 0, "");
		}
		break;
	case 2:
		LCD_Puts(13, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(13, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(13, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(13, 0, "3");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(13, 0, "");
		}
		break;
	case 3:
		LCD_Puts(14, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(14, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(14, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(14, 0, "4");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(14, 0, "");
		}
		break;
	case 4:
		LCD_Puts(15, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(15, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(15, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(15, 0, "5");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(15, 0, "");
		}
		break;
	case 5:
		LCD_Puts(16, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(16, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(16, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(16, 0, "6");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(16, 0, "");
		}
		break;
	case 6:
		LCD_Puts(17, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(17, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(17, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(17, 0, "7");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(17, 0, "");
		}
		break;
	case 7:
		LCD_Puts(18, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(18, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(18, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(18, 0, "8");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(18, 0, "");
		}
		break;
	case 8:
		LCD_Puts(19, 0, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(19, 0, " ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(19, 0, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(19, 0, "9");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(19, 0, "");
		}
		break;
	case 9:
		LCD_Puts(0, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(0, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(0, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(0, 1, "10");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(0, 1, "");
		}
		break;
	case 10:
		LCD_Puts(3, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(3, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(3, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(3, 1, "11");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(3, 1, "");
		}
		break;
	case 11:
		LCD_Puts(6, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(6, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(6, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(6, 1, "12");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(6, 1, "");
		}
		break;
	case 12:
		LCD_Puts(9, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(9, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(9, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(9, 1, "13");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(9, 1, "");
		}
		break;
	case 13:
		LCD_Puts(12, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(12, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(12, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(12, 1, "14");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(12, 1, "");
		}
		break;
	case 14:
		LCD_Puts(15, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(15, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(15, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(15, 1, "15");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(15, 1, "");
		}
		break;
	case 15:
		LCD_Puts(18, 1, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(18, 1, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(18, 1, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(18, 1, "16");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(18, 1, "");
		}
		break;
	case 16:
		LCD_Puts(0, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(0, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(0, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(0, 2, "17");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(0, 2, "");
		}
		break;
	case 17:
		LCD_Puts(3, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(3, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(3, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(3, 2, "18");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(3, 2, "");
		}
		break;
	case 18:
		LCD_Puts(6, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(6, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(6, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(6, 2, "19");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(6, 2, "");
		}
		break;
	case 19:
		LCD_Puts(9, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(9, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(9, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(9, 2, "20");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(9, 2, "");
		}
		break;
	case 20:
		LCD_Puts(12, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(12, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(12, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(12, 2, "21");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(12, 2, "");
		}
		break;
	case 21:
		LCD_Puts(15, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(15, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(15, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(15, 2, "22");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(15, 2, "");
		}
		break;
	case 22:
		LCD_Puts(18, 2, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(18, 2, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(18, 2, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(18, 2, "23");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(18, 2, "");
		}
		break;
	case 23:
		LCD_Puts(0, 3, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(0, 3, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(0, 3, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(0, 3, "24");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(0, 3, "");
		}
		break;
	case 24:
		LCD_Puts(3, 3, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(3, 3, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(3, 3, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(3, 3, "25");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(3, 3, "");
		}
		break;
	case 25:
		LCD_Puts(6, 3, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(6, 3, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(6, 3, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(6, 3, "26");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(6, 3, "");
		}
		break;
	case 26:
		LCD_Puts(9, 3, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(9, 3, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(9, 3, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(9, 3, "27");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(9, 3, "");
		}
		break;
	case 27:
		LCD_Puts(12, 3, "");
		if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '1')
		{
			LCD_Puts(12, 3, "  ");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '0';
			LCD_Puts(12, 3, "");
		}
		else if(FlagRNSw_G == 1 && ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] == '0')
		{
			LCD_Puts(12, 3, "28");
			ArrayProgramLine[CntWriteProgramLine][CntMagnetSetAndReset - 1] = '1';
			LCD_Puts(12, 3, "");
		}
		break;
	default:
		break;
	}

}
void MagnetInitWriteProgram(void)
{
	LCD_Clear();
	for(int j3 = 0; j3 < MaxMagnet; j3++)
	{
		switch(j3)
		{
		case 0:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(11, 0, "1");
			break;
		case 1:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(12, 0, "2");
			break;
		case 2:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(13, 0, "3");
			break;
		case 3:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(14, 0, "4");
			break;
		case 4:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(15, 0, "5");
			break;
		case 5:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(16, 0, "6");
			break;
		case 6:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(17, 0, "7");
			break;
		case 7:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(18, 0, "8");
			break;
		case 8:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(19, 0, "9");
			break;
		case 9:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(0, 1, "10");
			break;
		case 10:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(3, 1, "11");
			break;
		case 11:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(6, 1, "12");
			break;
		case 12:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(9, 1, "13");
			break;
		case 13:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(12, 1, "14");
			break;
		case 14:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(15, 1, "15");
			break;
		case 15:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(18, 1, "16");
			break;
		case 16:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(0, 2, "17");
			break;
		case 17:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(3, 2, "18");
			break;
		case 18:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(6, 2, "19");
			break;
		case 19:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(9, 2, "20");
			break;
		case 20:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(12, 2, "21");
			break;
		case 21:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(15, 2, "22");
			break;
		case 22:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(18, 2, "23");
			break;
		case 23:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(0, 3, "24");
			break;
		case 24:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(3, 3, "25");
			break;
		case 25:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(6, 3, "26");
			break;
		case 26:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(9, 3, "27");
			break;
		case 27:
			if(ArrayProgramLine[CntWriteProgramLine][j3] == '1')
				LCD_Puts(12, 3, "28");
			break;
		default:
			break;
		}
	}
}

void MagnetPaireCommand (int8_t ShiftArrayProgramLine)
{
	BackupCntProgramLine2 = BackupCntProgramLine2 + ShiftArrayProgramLine;
	if(BackupCntProgramLine2 >= CntProgramLine)
		BackupCntProgramLine2 = BackupCntProgramLine2 - CntProgramLine;

	if(BackupCntProgramLine2 >= CntProgramLine)
		BackupCntProgramLine2 = BackupCntProgramLine2 - CntProgramLine;

	if(BackupCntProgramLine2 >= CntProgramLine)
		BackupCntProgramLine2 = BackupCntProgramLine2 - CntProgramLine;

	if(BackupCntProgramLine2 < 0)
		BackupCntProgramLine2 = BackupCntProgramLine2 + CntProgramLine;

	if(BackupCntProgramLine2 < 0)
		BackupCntProgramLine2 = BackupCntProgramLine2 + CntProgramLine;

	if(BackupCntProgramLine2 < 0)
		BackupCntProgramLine2 = BackupCntProgramLine2 + CntProgramLine;

	if(FlagMagnetDirection == 1)
	{
		for(int j = 0; j < 20; j++)
		{
			switch(j)
			{
			case 0:
			case 1:
			case 2:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOB, CMNDPin[j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOB, CMNDPin[j], LocalSet_Pin);
				break;
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
			case 10:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOD, CMNDPin[j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOD, CMNDPin[j], LocalSet_Pin);
				break;
			case 11:
			case 12:
			case 13:
			case 24:
			case 25:
			case 26:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOC, CMNDPin[j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOC, CMNDPin[j], LocalSet_Pin);
				break;
			case 14:
			case 15:
			case 27:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOA, CMNDPin[j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOA, CMNDPin[j], LocalSet_Pin);
				break;
			case 16:
			case 17:
			case 18:
			case 19:
			case 20:
			case 21:
			case 22:
			case 23:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOE, CMNDPin[j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOE, CMNDPin[j], LocalSet_Pin);
				break;
			default:
				break;
			}
		}
	}
//	for(int j = 0; j < MaxColorWeaving; j++)
//	{
//		switch(j)
//		{
//		case 0:
//		case 1:
//		case 2:
//			if(ArrayProgramLine[BackupCntProgramLine][MaxMagnet + j] == '0')
//				HAL_GPIO_WritePin(GPIOC, TCMNDPin[j], LocalReset_Pin);
//			if(ArrayProgramLine[BackupCntProgramLine][MaxMagnet + j] == '1')
//				HAL_GPIO_WritePin(GPIOC, TCMNDPin[j], LocalSet_Pin);
//			break;
//		case 3:
//			if(ArrayProgramLine[BackupCntProgramLine][j] == '0')
//				HAL_GPIO_WritePin(GPIOA, TCMNDPin[j], LocalReset_Pin);
//			if(ArrayProgramLine[BackupCntProgramLine][j] == '1')
//				HAL_GPIO_WritePin(GPIOA, TCMNDPin[j], LocalSet_Pin);
//			break;
//		}
//	}
	if(FlagMagnetDirection == 0)
	{
		for(int j = 0; j < 20; j++)
		{
			switch(j)
			{
			case 27:
			case 26:
			case 25:
			case 0:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOC, CMNDPin[MaxMagnet -1 -j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOC, CMNDPin[MaxMagnet -1 -j], LocalSet_Pin);
				break;
			case 24:
			case 23:
			case 22:
			case 21:
			case 20:
			case 19:
			case 18:
			case 17:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOD, CMNDPin[MaxMagnet -1 -j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOD, CMNDPin[MaxMagnet -1 -j], LocalSet_Pin);
				break;
			case 16:
			case 15:
			case 14:
			case 13:
			case 12:
			case 11:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOB, CMNDPin[MaxMagnet -1 -j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOB, CMNDPin[MaxMagnet -1 -j], LocalSet_Pin);
				break;
			case 10:
			case 9:
			case 8:
			case 7:
			case 6:
			case 5:
			case 4:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOE, CMNDPin[MaxMagnet -1 -j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOE, CMNDPin[MaxMagnet -1 -j], LocalSet_Pin);
				break;
			case 3:
			case 2:
			case 1:
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '1')
					HAL_GPIO_WritePin(GPIOA, CMNDPin[MaxMagnet -1 -j], LocalReset_Pin);
				if(ArrayProgramLine[BackupCntProgramLine2][j] == '0')
					HAL_GPIO_WritePin(GPIOA, CMNDPin[MaxMagnet -1 -j], LocalSet_Pin);
				break;
			default:
				break;
			}
		}
	}

//	if(FlagMenu == 1 && StatusMainMenu_G == 1)
//	{
//		memset(TempStr, 0, sizeof(TempStr));
//		sprintf(TempStr,"%d  ",BackupCntProgramLine + 1);
//		LCD_Puts(14,3,TempStr);
//		if(FlagRPM_G == 1)
//		{
//			FlagRPM_G = 0;
//			RPM = 60000 / CNTRPM_G;
//			CNTRPM_G = 0;
//			memset(TempStr, 0, sizeof(TempStr));
//			sprintf(TempStr,"RPM = %d   ",RPM);
//			LCD_Puts(0,2,TempStr);
//		}
//	}
}
void CircularMenu(void)
{
	CntMenuj = 0;
	if(KypadDirection_G == KeypadDown)
	{
		KypadDirection_G = KeypadDefault;
		if(CntRN_G == LCDDispalyPosition[0] || CntRN_G == LCDDispalyPosition[1] || CntRN_G == LCDDispalyPosition[2] || CntRN_G == LCDDispalyPosition[3])
			CntMenui = CntRN_G;
		else
		{
			for(CntMenui = CntRN_G; CntMenui < CntRN_G + 4; CntMenui++)
			{
				if(CntMenui > MainMenu - 1)
					LCDDispalyPosition[CntMenuj] = CntMenui - MainMenu - 1;
				else
					LCDDispalyPosition[CntMenuj] = CntMenui;
				CntMenuj = CntMenuj + 1;
			}
		}
	}
	if(KypadDirection_G == KeypadUp)
	{
		KypadDirection_G = KeypadDefault;
		if(CntRN_G == LCDDispalyPosition[0] || CntRN_G == LCDDispalyPosition[1] || CntRN_G == LCDDispalyPosition[2] || CntRN_G == LCDDispalyPosition[3])
			CntMenui = CntRN_G;
		else
		{
			for(CntMenui = CntRN_G; CntMenui > CntRN_G - 4; CntMenui--)
			{
				if(CntMenui < 1)
					LCDDispalyPosition[3 - CntMenuj] = CntMenui + MainMenu - 1 ;
				else
					LCDDispalyPosition[3 - CntMenuj] = CntMenui;
				CntMenuj = CntMenuj + 1;
			}
		}
	}
	memset(ArrayLCDDisplay, 0, sizeof(ArrayLCDDisplay));
	for(CntMenuj = 0; CntMenuj < 4; CntMenuj++)
		strncpy(ArrayLCDDisplay[CntMenuj], ArrayMenuName[LCDDispalyPosition[CntMenuj] - 1], 18);

	for(CntMenuj = 0; CntMenuj < 4; CntMenuj++)
		if(LCDDispalyPosition[CntMenuj] == CntRN_G)
			LcdYPosition = CntMenuj;

	LCD_Clear();
	for(CntMenuj = 0; CntMenuj < 4; CntMenuj++)
	{
		if(CntMenuj == LcdYPosition)
			LCD_Puts(2, CntMenuj, ArrayLCDDisplay[CntMenuj]);
		else
			LCD_Puts(0, CntMenuj, ArrayLCDDisplay[CntMenuj]);

	}
	LCD_PutCustom(0,LcdYPosition,1); // Arrows
}
void MCUFlashInit(void)
{
	HAL_FLASH_Unlock();
	  /* EEPROM Init */
	  if( EE_Init() != EE_OK)
	  {
		  Error_Handler();
	  }

	  if((EE_ReadVariable(VirtAddVarTab[0],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  BackupCntProgramLine = VarDataTab[0];

	  memset(VarDataTab, 0, sizeof(VarDataTab));
	  if((EE_ReadVariable(VirtAddVarTab[1],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  Cntexpiration_G = VarDataTab[0];

	  memset(VarDataTab, 0, sizeof(VarDataTab));
	  if((EE_ReadVariable(VirtAddVarTab[2],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  Int1WeftPerCentimeter = VarDataTab[0];

	  memset(VarDataTab, 0, sizeof(VarDataTab));
	  if((EE_ReadVariable(VirtAddVarTab[3],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  Int2WeftPerCentimeter = VarDataTab[0];

	  IntWeftPerCentimeter = Int2WeftPerCentimeter;
	  IntWeftPerCentimeter = IntWeftPerCentimeter <<16;
	  IntWeftPerCentimeter = Int1WeftPerCentimeter + IntWeftPerCentimeter;
	  FloatWeftPerCentimeter = IntWeftPerCentimeter;

	  if((EE_ReadVariable(VirtAddVarTab[4],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  Int1WeftPerCentimeter = VarDataTab[0];

	  memset(VarDataTab, 0, sizeof(VarDataTab));
	  if((EE_ReadVariable(VirtAddVarTab[5],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  Int2WeftPerCentimeter = VarDataTab[0];

	  CntTotalShiftRPM = Int2WeftPerCentimeter;
	  CntTotalShiftRPM = CntTotalShiftRPM <<16;
	  CntTotalShiftRPM = Int1WeftPerCentimeter + CntTotalShiftRPM;
	  FloatCntTotalShiftRPM = CntTotalShiftRPM;


	  if((EE_ReadVariable(VirtAddVarTab[6],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  CntMinutes_G = VarDataTab[0];
	  if(CntMinutes_G > 59)
		  CntMinutes_G = 59;

	  if((EE_ReadVariable(VirtAddVarTab[7],  &VarDataTab[0])) != HAL_OK)
	  {
		  Error_Handler();
	  }
	  CntHour_G = VarDataTab[0];
	  if(CntMinutes_G > 24)
		  CntMinutes_G = 24;

	  if((EE_ReadVariable(VirtAddVarTab[8],  &VarDataTab[0])) != HAL_OK)
	   {
	 	  Error_Handler();
	   }
	   Int1WeftPerCentimeter = VarDataTab[0];

	   memset(VarDataTab, 0, sizeof(VarDataTab));
	   if((EE_ReadVariable(VirtAddVarTab[9],  &VarDataTab[0])) != HAL_OK)
	   {
	 	  Error_Handler();
	   }
	   Int2WeftPerCentimeter = VarDataTab[0];

	   CntTotalRPM = Int2WeftPerCentimeter;
	   CntTotalRPM = CntTotalRPM <<16;
	   CntTotalRPM = Int1WeftPerCentimeter + CntTotalRPM;
	   if(CntTotalRPM >= 100000000)
		   CntTotalRPM = 0;

	   FloatCntTotalRPM = CntTotalRPM;

	  HAL_FLASH_Lock();

}
void SPIFlashInit(void)
{
	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(TempStr, 65032, 0, 1);
	CntActiveMagnet = TempStr[0];
	if(CntActiveMagnet > MaxMagnet + 1)
		CntActiveMagnet = MaxMagnet;

	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(TempStr, 65064, 0, 1);
	FlagMagnetDirection = TempStr[0];
	if(FlagMagnetDirection > 1)
		FlagMagnetDirection = 1;

	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(TempStr, 65096, 0, 1);
	FlagExpiration_G = TempStr[0];
	if(FlagExpiration_G > 1)
		FlagExpiration_G = 1;

	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(TempStr, 65096, 0, 1);
	FlagExpiration_G = TempStr[0];
	if(FlagExpiration_G > 1)
		FlagExpiration_G = 1;

	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(CntEditNumWeftPerCentimeter, 65128, 0, 4);
	if(CntEditNumWeftPerCentimeter[0] > 9)
		CntEditNumWeftPerCentimeter[0] = 1;

	if(CntEditNumWeftPerCentimeter[1] > 9)
		CntEditNumWeftPerCentimeter[1] = 0;

	if(CntEditNumWeftPerCentimeter[2] > 9)
		CntEditNumWeftPerCentimeter[2] = 0;

	if(CntEditNumWeftPerCentimeter[3] > 9)
		CntEditNumWeftPerCentimeter[3] = 0;

	if(CntEditNumWeftPerCentimeter[4] > 9)
		CntEditNumWeftPerCentimeter[4] = 0;
	FloatCntWeftPerCentimeter = 0;
	FloatTest = CntEditNumWeftPerCentimeter[2];
	FloatTest = (FloatTest / 10);
	FloatCntWeftPerCentimeter = FloatTest;
	FloatTest = CntEditNumWeftPerCentimeter[3];
	FloatTest = FloatTest / 100;
	FloatCntWeftPerCentimeter = FloatTest + FloatCntWeftPerCentimeter;
	FloatCntWeftPerCentimeter = CntEditNumWeftPerCentimeter[0] * 10 + CntEditNumWeftPerCentimeter[1] + FloatCntWeftPerCentimeter;

	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(CntEditNumRPMMenu, 65160, 0, 3);
	if(CntEditNumRPMMenu[0] > 9)
		CntEditNumRPMMenu[0] = 2;

	if(CntEditNumRPMMenu[1] > 9)
		CntEditNumRPMMenu[1] = 5;

	if(CntEditNumRPMMenu[2] > 9)
		CntEditNumRPMMenu[2] = 0;
	NumRPMMenu = CntEditNumRPMMenu[0] * 100 + CntEditNumRPMMenu[1] * 10 + CntEditNumRPMMenu[2];

	memset(TempStr, 0, sizeof(TempStr));
	W25qxx_ReadPage(CntEditNumWovenRollMenu, 65192, 0, 3);
	if(CntEditNumWovenRollMenu[0] > 9)
		CntEditNumWovenRollMenu[0] = 2;

	if(CntEditNumWovenRollMenu[1] > 9)
		CntEditNumWovenRollMenu[1] = 5;

	if(CntEditNumWovenRollMenu[2] > 9)
		CntEditNumWovenRollMenu[2] = 0;
	NumWovenRollMenu = CntEditNumWovenRollMenu[0] * 100 + CntEditNumWovenRollMenu[1] * 10 + CntEditNumWovenRollMenu[2];
	if(NumWovenRollMenu == 0)
		FlagActiveWovenRoll = 0;
	else
		FlagActiveWovenRoll = 1;
}
void PhiStatusInit(void)
{
	/************************     Initialize Rotation       *********************/
	//  if(HAL_GPIO_ReadPin(Reverse_GPIO_Port,Reverse_Pin) == GPIO_PIN_SET)
	//    RotationStatus_G = Left;
	//
	//  if(HAL_GPIO_ReadPin(Start_GPIO_Port,Start_Pin) == GPIO_PIN_SET)
	//    RotationStatus_G = Right;
	/****************************************************************************/
	/************************     Initialize Phi       **************************/
	if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_RESET)
		Phi_G = 0;
	if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_RESET)
		Phi_G = 1;
	if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_SET)
		Phi_G = 2;
	if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin) == GPIO_PIN_SET)
		Phi_G = 3;
	/****************************************************************************/
}
void ComboBoxStatusInit(void)
{
	StatusComboBox_G = ComboBoxStop_G;
	HAL_GPIO_WritePin(S1CMND_GPIO_Port, S1CMND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S2CMND_GPIO_Port, S2CMND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S3CMND_GPIO_Port, S3CMND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S4T5CMND_GPIO_Port, S4T5CMND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S5T6CMND_GPIO_Port, S5T6CMND_Pin, GPIO_PIN_SET);


	HAL_GPIO_WritePin(S6T7CMND_GPIO_Port, S6T7CMND_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S7T8CMND_GPIO_Port, S7T8CMND_Pin, GPIO_PIN_SET);
}
void ValidStatusInit(void)
{
	if(Cntexpiration_G > ConsDeadline && FlagExpiration_G == 1)
	{
		LCD_Clear();
		LCD_Puts(7, 1, "0000");
		FlagMenu = 210;
		FlagExpireProgram = 1;
		HAL_GPIO_WritePin(Watchdog_GPIO_Port, Watchdog_Pin, GPIO_PIN_RESET);
	}

	if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	{
		/* Refresh Error */
		Error_Handler();
	}

	HAL_RNG_GenerateRandomNumber(&hrng, &RandomeValue);
	RandomeValue = RandomeValue >> 20;
}
void USBStatusInit(void)
{
	memset(BackupFlagMenuUsb, 0, sizeof(BackupFlagMenuUsb));
	W25qxx_ReadPage(BackupFlagMenuUsb, 64000, 0, 3);
	if(BackupFlagMenuUsb[0] == 3)		// USB Menu --> Write
	{
		FlagMenu  = 3;
		FlagSelectMenuUsb = 1;
		FlagRNSw_G = 1;
		//		LCD_Clear();
		LCD_Puts(0,0,"Please Wait ...");
	}
	else if(BackupFlagMenuUsb[0] == 4) // USB Menu --> Read
	{
		FlagMenu  = 4;
		FlagSelectMenuUsb = 1;
		FlagRNSw_G = 1;
		//		LCD_Clear();
		LCD_Puts(0,0,"Please Wait ...");
	}
	else
	{
		LCD_Puts(0, 0, "    Raya Sanat      ");
		LCD_Puts(0, 1, "    03134330856     ");
		LCD_Puts(0, 2, "    09131701438     ");
		LCD_Puts(0, 3, Version);
		FlagLockMenu = 1;
		MaxMenu_G = LockedMainMenu;
		HAL_Delay(100);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
