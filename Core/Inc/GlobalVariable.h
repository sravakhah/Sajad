
#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MainMenu                	14 + 1
#define LockedMainMenu				1  + 1
#define USBMenu                 	2  + 1
#define USBMenu_Write           	1  + 1
#define USBMenu_Read            	1  + 1
#define RunMenu                 	0  + 1
#define MagnetMenu					1  + 1
#define ShiftMenu					1  + 1

#define DefaultMenu             	1  + 1
#define TroubleshootingMenu			2  + 1
#define WriteProgramMenu			1  + 1
#define EditProgramMenu				28 + 1
#define F2MassageWriteProgramMenu 	4  + 1
#define F1MassageWriteProgramMenu 	2  + 1
#define EditProgramMenu2			0  + 1
#define EditProgramMenu3			28 + 1
#define F2MassageEditProgramMenu 	3  + 1
#define F1MassageEditProgramMenu 	3  + 1
#define SamplingPatternMenu			0  + 1
#define ActiveMagnetMenu			1  + 1
#define MagnetDirectionMenu			2  + 1
#define ValidityDateMenu			1  + 1
#define WovenfabricMenu				1  + 1
#define RPMMenu						1  + 1
#define WovenRollMenu				1  + 1

/******	ComboBox	*****/
#define ComboBoxStop_G				0


#define Left                    0
#define Right                   1
/***********************/
#define KeypadUp				1
#define KeypadDown				3
#define KeypadDefault			0
/* USER CODE END PD */

extern int16_t CntRN_G;
extern char FlagMenu_G, StatusMainMenu_G, FlagDisplay_G, FlagMenu_G, MaxMenu_G, FlagSwitchTrigger_G;
extern char FlagRNSw_G, FlagRNA_G, FlagRNB_G, FlagStatusRNA_G, FlagStatusRNB_G, FlagHome_G, FlagMagneticTrigger_G, RotationStatus_G;
extern uint8_t Phi_G, ArrayPhiBackup_G[5],FlagReverse_G;

extern USBH_HandleTypeDef hUsbHostFS;
extern uint32_t CntDelayTimer_G, CntTimeRPM_G;;
extern int8_t NR0To3_G, NR2To1_G, NR1To0_G, NR3To2_G, NS3To0_G, NS1To2_G, NS0To1_G, NS2To3_G, N_G;
extern uint8_t FlagSwUp_G, FlagSwRight_G,FlagSwLeft_G, FlagSwDown_G, FlagSwF1_G, FlagSwF2_G, FlagUseKeypad_G;
extern uint8_t FlagSupplyFB_G, FlagRPM_G, FlagUpdateTroubleshooting_G;
extern uint8_t StatusComboBox_G;
extern uint8_t CntMinutes_G, CntHour_G;
extern uint16_t Cntexpiration_G;
extern uint8_t FlagExpiration_G;
extern uint8_t KypadDirection_G;
/*			Airjet		*/
extern uint8_t FlagMagnetDisplay, FlagExpireProgram;
extern int32_t CntProgramLine;
