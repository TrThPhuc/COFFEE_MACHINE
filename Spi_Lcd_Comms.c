/*
 * Spi_Lcd_Comms.c
 *
 *  Created on: June 9, 2021
 *      Author: Davi-Engineering
 */

//============================================================================
//============================================================================
// FILE:        SPI_LcdComms.c
// TITLE:       Kernel for Communicate Host to LCD through SPI interface
// LCD TYPE:    ST7567
// MACHINE:     TM4C123(ARM M4)
// FUNCTION:    Cafe machine GUI
//============================================================================
//============================================================================
#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"

#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/eeprom.h"

#include "TCA9539.h"
#include "TCA9539_hw_memmap.h"
#include "Coffee_Machine.h"

extern TCA9539Regs TCA9539_IC1;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// System Defines
//---------------------------------------------------------------
#define Kernel_LCD_pr
#define NumOfPage       10
#define InitPageDisplay 3
#define AddDataEeprom   0x00

//#define debuglcd
//------------------------------- Status machine --------------------------------
extern _Bool Error, WarningFlag;

extern Error_t ErrorMachine[16], WarningMachine[5];

//#define uDma_SSI0
// ---------------------Shared function prototypes of Interface LCD -------------------------------
extern void Disp_Str_5x8_Image(volatile uint8_t page, volatile uint8_t column,
                               uint8_t *text, uint8_t *Image);
extern void Disp_Str_8x16_Image(uint8_t page, uint8_t column, uint8_t *text);
extern void Disp_20x20_Image(uint8_t page, uint8_t column, uint8_t *iPtr,
                             uint8_t *Image);
extern void disp_bitmap(uint8_t page, uint8_t col, uint8_t *obj);
extern void LCD_Write_Cmd(uint8_t cmd);
extern void LCD_Write_Data(uint8_t cmd);
extern void disp_bitmap(uint8_t page, uint8_t col, uint8_t *obj);
extern void Copy_bitExImage(void *ptr);
extern uint8_t coeff_change;
#ifdef uDma_SSI0
extern uint8_t *LCD_IMAGE_Send, *LCD_IMAGE_Write;
extern uint8_t LCD_IMAGE_Pri[];
extern uint8_t LCD_IMAGE_Sec[];
#endif
volatile uint8_t currentcount = 0;

// -------------------------------- KERNEL -------------------------------------
#define ReadMenuB TCA9539_IC1.TCA9539_Input.all & (Menu_Bt)
#define ReadUpB TCA9539_IC1.TCA9539_Input.all & (Up_Bt)
#define ReadDownB TCA9539_IC1.TCA9539_Input.all & (Down_Bt)

#define pageNumEspresso1        6
#define pageNumEspresso2        7
#define pageNumSpecial1         8
#define pageNumSpecial2         9
#define pageNumTimesBalde       11
#define pageNumTimeExtract      12

#define     AckPrAdjIndex(x, p)                 \
do                                              \
{                                               \
    if(Pr_Index == x && current_page == p)      \
    Ack_PrAdj = 1;                              \
    else                                        \
    Ack_PrAdj = 0;                              \
}while(0)
typedef enum mathType_t MathType;
void SerialCommsInit(void);
void SerialHostComms(void);

void GetButtonCmd(void);    // Read state of command button
void CmdInterpreter(void);  // Command interpreter

// Function prototypes for Command dispatcher
void ButtonSet_cmd(void);
void ButtonDown_cmd(void);
void ButtonUp_cmd(void);

// Function prototypes for display
void PageDisplay();         // Select page display
void Page0_Display(void);   // Page 1 display
void Page1_Display(void);   // Page 2 display
void Page2_Display(void);   // Page 2 display
void Page3_Display(void);   // Page 2 display
void Page4_Display(void);   // Page 2 display
void Page5_Display(void);   // Page 2 display
void Page6_Display(void);   // Page 2 display
void Page7_Display(void);   // Page 2 display
void Page8_Display(void);
void Page9_Display(void);
void Page10_Display(void);
void Page11_Display(void);

void defaultcmd(void);
void pageWelcom_Display(void);
void Page_Cursor_Display(uint8_t page, uint8_t pos);   // Display cursor
void DisplayError(void);
void DisplayWarning(void);
void PageManual_Display(void);
static inline void DisplayInfoInProcess(void);
inline void PrintNum(MathType typePar, _Bool cur, void *Num);
// Function prototype for EEPROM emulator
extern void SaveParameterToEEPROM(void);
extern void ReadFromEEPROM(void);

// Variable declarations
// ------------------------------------------------------------
//FRAMEWORK variable
void (*CmdDispatcher)(void) = defaultcmd;   // Dispatcher command button
void (*LCD_TaskPointer)(void);              // Task pointer
void (*PagePointer)(void);                  // Page pointer
extern void clearBuffer(void *ptr);
extern void LCD_Write_Dat(uint8_t cmd);
/*
 * VrTimer0 refresh page
 * VrTimer1 hold button set(home) lcd
 * VrTimer2 boot page
 * VrTimer3 display temp
 */

extern int16_t VrTimer1[8];

extern void LCD_Address_Set(uint8_t page, uint8_t column);
extern uint8_t LCD_IMAGE[1024];
extern uint8_t bitmapCoffeeE1[];
extern uint8_t bitmapCoffeeE2[];
uint8_t Pr_Index = 0, NumOfPr, Offset = 0;    // Indicator  parameter
char *strMode;
uint8_t idMask;
uint8_t countList = 0;    // Count variable used to change page fuction
uint32_t *ObjSelect, *ObjSelectStep, *ObjSelectMax, *ObjSelectMin; // Object selected and step Object pointer
bool LoopValue = false;
uint8_t ObjSelectFlag = 0;

uint8_t current_page = 0, page_change = 1, layer = 0;
uint8_t CursorPosCol[NumOfPage] =
        { 124, 90, 80, 95, 124, 124, 124, 124, 124, 50 };

// Acknowledge Button event
int8_t Ack_BSet = 0, Ack_Bdown = 0, Ack_Bup = 0, Ack_Enter = 0, Ack_Return = 0,
        Ack_reset = 0; // Used to avoid duplicate problem
volatile uint8_t BsetFlag, BdownFlag, BupFlag;     // Flag used for scan process
// Acknowledge Flag
uint8_t Ack_PrAdj = 0;
uint8_t Up_window_index = 3, Down_window_index = 0; // Windown display - 4 line
extern bool calibWeightFlag, calibWeightStr;
extern uint8_t calibWeightObj, ManualObjSelected;
extern uint16_t *dataSentList[]; //  Kernel terminal connect to monitor variable
extern bool En, idleMachine, InCleanning, InProcess;
extern bool InManualTest;
extern uint32_t *Pr_Packet[NumberOfParameter]; // Kernel terminal connect to parameter
union NumConvert_u
{
    float floatNum;
    uint32_t unintNum;

};
union NumConvert_u NumConvert;
enum mathType_t
{
    intMath, floatMath, boolMath,
};
MathType InMode_Mathtype, InTemp_Mathtype;

uint32_t Pr_PacketCopy[NumberOfParameter];         // Array copy pratameter
_Bool Pr_PacketCopyMask[NumberOfParameter] = { };
float Pr_Gui_Packet[10];            // Array for Gui display parameter
uint8_t save_pr, cpy_pr;
extern _Bool HoldMsgFlag;
// --------------------------Manual Test-Control Module -----------------------------------------
typedef enum MotorSlect_e
{
    GrindMotorSelect, PressMotorSelect, PumpMotorSelect

} MotorSlect;
uint32_t maMotorSlected, maIncrease = 1, maMax = 3, maMin;

// --------------------------User variable used for display -----------------------------------------
_Bool boostFlag = 1, HomePage = 0;
char *Str_Display;
char Str_Temp[16];
char Str_ErrorNum[10], Str_WarningNum[10];
char CurStr[3] = " <";
uint8_t blinkCurVr;
_Bool blinkCur;
uint8_t holdBtVrTimeDown = 25, holdBtVrTimeUp = 25;
char *LCD_String_Page1[] = { "Thong so: ", "Canh bao", "Thong tin may",
                             "Thong tin loi", "Thu cong" };

char *LCD_String_Page6[NumOfParInEachMode] = { "Pre-infusion1(s): ",
                                               "Pre-infusion2(s): ",
                                               "MotorInfusion: ",
                                               "SpeedMotor: ",
                                               "Nuoc ra(xung): ",
                                               "Tgian xay(s): ", "Do ep(mm): ", "Speed: " }; //

char *LCD_String_Page2[10] = { "Espresso 1 ly", "Espresso 2 ly",
                               "Special 1 ly ", "Special 2 ly", "Gioi Han",
                               "Nhiet do" };

char *LCD_String_PageManual[] = { "Motor ", "Valve Brew", "Valve CoffeeOut",
                                  "Valve BackRinse", "Valve SteamIn",
                                  "Valve Drain", "VolumeMeter" };

char *LCD_String_Page0[16] =
        { "Da san sang", "Dang ve sinh", "Tha thuoc ve sinh",
          "Dang ve sinh thuoc", "thuoc", "Espresso", "Espresso", "Special",
          "Special", "Dang khoi dong", "Do hoc ba", "hoc dung ba", "Ma Loi:",
          " Dang ngam coi", "Wr:" };
uint8_t id_Page0 = 0xFF, idModeRunning, idPage0Display[8];
char LCD_PosStr_page0[16] = { 34, 32, 15, 10, 55, 16, 8, 20, 10, 24, 35, 35, 5,
                              25 };
MathType Int_Format_parameter[NumOfParInEachMode] = { intMath, intMath,
                                                      boolMath, floatMath,
                                                      intMath, floatMath,
                                                      intMath, floatMath };
MathType Int_Format_TempParameter[NumOfParTemperature] = { floatMath, floatMath,
                                                           intMath, intMath,
                                                           intMath };
union NumConvert_u InMode_StepChangeValue[NumOfParInEachMode] = {
        { .unintNum = 1 }, { .unintNum = 1 }, { .unintNum = 0 }, { .floatNum =
                0.01 },
        { .unintNum = 1 }, { .floatNum = 0.1 }, { .unintNum = 1 }, { .floatNum =
                0.01 } };

// preInfusion, volume Pulse, grinding dur, Temperature, Pitch
union NumConvert_u InMode_MaxParameter[NumOfParInEachMode] = {
        { .unintNum = 50 }, { .unintNum = 50 }, { .unintNum = 0 }, { .floatNum =
                0.6 },
        { .unintNum = 500 }, { .floatNum = 50.0 }, { .unintNum = 30 }, {
                .floatNum = 0.98 } };
union NumConvert_u InMode_MinParameter[NumOfParInEachMode] = {
        { .unintNum = 4 }, { .unintNum = 1 }, { .unintNum = 0 }, { .floatNum =
                0.2 },
        { .unintNum = 20 }, { .floatNum = 5.0 }, { .unintNum = 11 }, {
                .floatNum = 0.3 } };
uint32_t MaxTimesBlade = 65000, MinTimesBlade = 2000, stepTimesBalde = 100;
uint32_t MaxTimesExtract = 90000, MinTimesExtract = 2000,
        stepTimesExtract = 100;
uint32_t MaxExtract = 50, MinExtract = 15, stepExtract = 1;
union NumConvert_u InTemp_MaxParameter[NumOfParTemperature] = {
        { .floatNum = 125.0 }, { .floatNum = 130 }, { .unintNum = 95 }, {
                .unintNum = 99 },
        { .unintNum = 99 } };
union NumConvert_u InTemp_MinParameter[NumOfParTemperature] = {
        { .floatNum = 60.0 }, { .floatNum = 60 }, { .unintNum = 60 }, {
                .unintNum = 5 },
        { .unintNum = 1 } };
union NumConvert_u InTemp_StepChangeValue[NumOfParTemperature] = {
        { .floatNum = 0.5 }, { .floatNum = 1.0 }, { .unintNum = 1 }, {
                .unintNum = 1 },
        { .unintNum = 1 } };
float LCD_Step_Calib = 0.1f;
char str[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x7C, 0x38,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
extern uint8_t bitmap[];
typedef struct Node Node_t;
typedef struct
{
    uint8_t page;
    Node_t *item[8];
    Node_t *returnN;
} Node;

Node Menu;
Node Mode, Warning, InfoMachine, ErrorList, ManualControl;
Node Espresso_1, Espresso_2, Decatt_1, Decatt_2;
Node LimitTime, WarmingSeting;
Node GrindModule, ExtractionModule;
Node *NodeSelected;
Node *MenuList[] = { &Mode, &Warning, &InfoMachine, &ErrorList, &ManualControl,
NULL };

Node *ModeList[] = { &Espresso_1, &Espresso_2, &Decatt_1, &Decatt_2, &LimitTime,
                     &WarmingSeting };
Node *WarningList[] = { &GrindModule, &ExtractionModule };

#define InitalizeModeParDisplay(_offset, _strMode)          \
do                                                          \
{                                                           \
    Offset = _offset;                                       \
    strMode = _strMode;                                     \
}while(0)                                                   \

void AddNode(Node *Node_pr, uint8_t _page, Node **Node_ch, Node *_returnN)
{
    uint8_t i;
    Node_pr->page = _page;

    for (i = 0; i < 6; i++)
    {
        if (Node_ch != NULL)
            Node_pr->item[i] = (Node_t*) Node_ch[i];
        else
            Node_pr->item[i] = NULL;
    }
    Node_pr->returnN = (Node_t*) _returnN;

}
void MenuInitialize()
{
    AddNode(&Menu, 1, (Node**) &MenuList, NULL);
    AddNode(&Mode, 2, (Node**) &ModeList, &Menu);
    AddNode(&Espresso_1, 6, NULL, &Mode);
    AddNode(&Espresso_2, 7, NULL, &Mode);
    AddNode(&Decatt_1, 8, NULL, &Mode);
    AddNode(&Decatt_2, 9, NULL, &Mode);
    AddNode(&LimitTime, 14, NULL, &Mode);   // 10 - calibration
    AddNode(&WarmingSeting, 15, NULL, &Mode);

    AddNode(&Warning, 3, (Node**) &WarningList, &Menu);
    AddNode(&GrindModule, 11, NULL, &Warning);
    AddNode(&ExtractionModule, 12, NULL, &Warning);

    AddNode(&InfoMachine, 4, NULL, &Menu);
    AddNode(&ErrorList, 5, NULL, &Menu);
    AddNode(&ManualControl, 13, NULL, &Menu);

}
void EnterNode()
{

    Ack_Enter = 0;
    Node *ptr = (Node*) NodeSelected->item[Pr_Index];
    if (ptr == NULL)
        return;
    NodeSelected = ptr;
    current_page = NodeSelected->page;
    page_change = 1;
    ObjSelectFlag = 0;

}
void ReturnNode()
{
    Ack_Return = 0;
    Node *ptr = (Node*) NodeSelected->returnN;
    if (ptr == NULL)
        return;
    NodeSelected = (Node*) NodeSelected->returnN;
    current_page = NodeSelected->page;
    page_change = 1;
    ObjSelectFlag = 0;

}
void SerialCommsInit(void)
{

    LCD_TaskPointer = &GetButtonCmd;
    PagePointer = &pageWelcom_Display;
    MenuInitialize();
    NodeSelected = &Menu;
}
void SerialHostComms(void)
{
    (*LCD_TaskPointer)();
}
void GetButtonCmd(void)
{

// Read external gpio ic
    BsetFlag = ReadMenuB;
    BupFlag = ReadUpB;
    BdownFlag = ReadDownB;

    if (BsetFlag == 0)
    {
        CmdDispatcher = &ButtonSet_cmd;                     // Set button
    }
    else if (BupFlag == 0)
    {

        CmdDispatcher = &ButtonUp_cmd;                      // Up button
    }

    else if (BdownFlag == 0)
    {

        CmdDispatcher = &ButtonDown_cmd;                    // Down button
    }
    else
    {
        if (Ack_reset == 1)
        {
            if (VrTimer1[holdButtonHome] > 500)
            {
                Ack_reset = 0;
                ObjSelectFlag = 0;
                current_page = 0;
                VrTimer1[displayBootPage] = 150; // VrTimer2 for display boot page
                boostFlag = 1;

                NodeSelected = &Menu;
            }
            else
                VrTimer1[holdButtonHome]++;
        }
        CmdDispatcher = &defaultcmd;
    }
    if (BsetFlag == Menu_Bt)
    {
        Ack_BSet = 0;

    }
    if (BdownFlag == Down_Bt)
    {
        Ack_Bdown = 0;
        VrTimer1[holdButtonDown] = 0;
        holdBtVrTimeDown = 25;
    }
    if (BupFlag == Up_Bt)
    {
        holdBtVrTimeUp = 25;
        Ack_Bup = 0;
        VrTimer1[holdButtonUp] = 0;
    }

    LCD_TaskPointer = &CmdInterpreter;

}
void defaultcmd()
{
// asm("nope");
}
void CmdInterpreter(void)
{
    (*CmdDispatcher)();
    LCD_TaskPointer = &PageDisplay;

}
void ButtonSet_cmd()
{

    if (Ack_BSet == 0)
    {

        VrTimer1[holdButtonHome] = 0;
        Ack_BSet = 1;
        Ack_reset = 1;

        if (Ack_PrAdj == 1)
        {
            save_pr = 1;
            if (ObjSelectFlag == 0)
            {
                ObjSelect =
                        (current_page != 13) ?
                                (&Pr_PacketCopy[Pr_Index + Offset]) :
                                (&maMotorSlected);

                switch (current_page)
                {
                case pageNumEspresso1:
                case pageNumEspresso2:
                case pageNumSpecial1:
                case pageNumSpecial2:
                    InMode_Mathtype = Int_Format_parameter[Pr_Index];
                    ObjSelectStep =
                            (uint32_t*) &InMode_StepChangeValue[Pr_Index];
                    ObjSelectMax = (uint32_t*) &InMode_MaxParameter[Pr_Index];
                    ObjSelectMin = (uint32_t*) &InMode_MinParameter[Pr_Index];
                    break;
                case pageNumTimesBalde:
                    ObjSelectStep = (uint32_t*) &stepTimesBalde;
                    ObjSelectMax = (uint32_t*) &MaxTimesBlade;
                    ObjSelectMin = (uint32_t*) &MinTimesBlade;
                    break;
                case pageNumTimeExtract:

                    ObjSelectStep = (uint32_t*) &stepTimesExtract;
                    ObjSelectMax = (uint32_t*) &MaxTimesExtract;
                    ObjSelectMin = (uint32_t*) &MinTimesExtract;
                    break;

                case 13:

                    InMode_Mathtype = intMath;
                    LoopValue = 1;
                    ObjSelectStep = &maIncrease;
                    ObjSelectMax = &maMax;
                    ObjSelectMin = &maMin;
                    break;
                case 14:
                {
                    ObjSelectStep = &stepExtract;
                    uint8_t objectIndexExtract_Max = 36,
                            objectIndexExtract_Min = 37;

                    (Pr_Index == 0) ?
                            (MaxExtract = 50, MinExtract =
                                    Pr_PacketCopy[objectIndexExtract_Min]) :
                            (MaxExtract = Pr_PacketCopy[objectIndexExtract_Max], MinExtract =
                                    15);

                    ObjSelectMax = &MaxExtract;
                    ObjSelectMin = &MinExtract;
                    break;
                }
                case 15:
                    InMode_Mathtype = Int_Format_TempParameter[Pr_Index];
                    ObjSelectStep =
                            (uint32_t*) &InTemp_StepChangeValue[Pr_Index];
                    ObjSelectMax = (uint32_t*) &InTemp_MaxParameter[Pr_Index];
                    ObjSelectMin = (uint32_t*) &InTemp_MinParameter[Pr_Index];
                    break;

                }
                ObjSelectFlag = 1;
            }
            else if (ObjSelectFlag == 1)
            {

                ObjSelectFlag = 0;
                InMode_Mathtype = intMath;
                ObjSelect = 0x00;
            }
        }

        if (current_page == 0)
        {
            current_page = 1;
            page_change = 1;
            HomePage = 0;
        }
        else
        {

            if (ReadDownB)
                EnterNode();
            else
                ReturnNode();
        }
    }
}
void ButtonDown_cmd()
{

    /*    if (BdownFlag == Down_Bt)
     {
     CmdDispatcher = &defaultcmd;
     Ack_Bdown = 0;
     VrTimer1[holdButtonDown] = 0;
     holdBtVrTimeDown = 25;
     }*/
    if (current_page == 0)
        return;
    if (Ack_Bdown == 0)
    {
        Ack_Bdown = 1;
        VrTimer1[holdButtonHome] = 0;
        if (ObjSelectFlag == 1)
        {
            switch (InMode_Mathtype)
            {
            case floatMath:
            {
                _Bool lessThanMin_fValue = ((*(float*) ObjSelect
                        <= *(float*) ObjSelectMin));
                if (lessThanMin_fValue)
                    break;
                NumConvert.floatNum = (*(float*) ObjSelect
                        - *(float*) ObjSelectStep);
                *ObjSelect = NumConvert.unintNum;
                break;
            }
            case intMath:
            {
                _Bool lessThanMin_iValue = *ObjSelect <= *ObjSelectMin;
                if (lessThanMin_iValue)
                {
                    if (LoopValue)
                        *ObjSelect = *ObjSelectMax;
                    else
                        break;
                }

                *ObjSelect = *ObjSelect - *ObjSelectStep;
                break;
            }
            case boolMath:
                *ObjSelect = !(*ObjSelect);
                break;
            };
        }
        else if (ObjSelectFlag == 0)
        {

            Pr_Index++;
            if (Pr_Index >= NumOfPr)
            {
                Pr_Index = 0;
                Up_window_index = 3;
                Down_window_index = 0;
            }
            if (Pr_Index > Up_window_index)
            {
                Up_window_index = Pr_Index;
                Down_window_index = Up_window_index - 3;
            }
            // uint8_t pageNum = 13;
            //AckPrAdjIndex(0, pageNum);
        }
    }
    else
    {
        if (VrTimer1[holdButtonDown] > holdBtVrTimeDown)
        {
            holdBtVrTimeDown = 20;
            VrTimer1[holdButtonDown] = 0;
            Ack_Bdown = 0;
        }
        else
            VrTimer1[holdButtonDown]++;

    }

}
void ButtonUp_cmd()
{

    /*    if (BupFlag == Up_Bt)
     {
     holdBtVrTimeUp = 25;
     CmdDispatcher = &defaultcmd;
     Ack_Bup = 0;
     VrTimer1[holdButtonUp] = 0;
     }*/
    if (current_page == 0)
        return;
    if (Ack_Bup == 0)
    {
        VrTimer1[holdButtonHome] = 0;
        Ack_Bup = 1;
        if (ObjSelectFlag == 1)
        {
            switch (InMode_Mathtype)
            {
            case floatMath:
            {
                _Bool largerThanMin_fValue = ((*(float*) ObjSelect
                        > *(float*) ObjSelectMax));
                if (largerThanMin_fValue)
                    break;
                NumConvert.floatNum = (*(float*) ObjSelect
                        + *(float*) ObjSelectStep);
                *ObjSelect = NumConvert.unintNum;
                break;
            }
            case intMath:
            {
                _Bool largerThanMin_iValue = *ObjSelect >= *ObjSelectMax;
                if (largerThanMin_iValue)
                {
                    if (LoopValue)
                        *ObjSelect = *ObjSelectMin;
                    else
                        break;
                }
                *ObjSelect = *ObjSelect + *ObjSelectStep;
                break;
            }
            case boolMath:
                *ObjSelect = !(*ObjSelect);
                break;
            };
        }
        else if (ObjSelectFlag == 0)
        {
            if (Pr_Index <= 0)
            {
                Down_window_index = 0;
                Up_window_index = 3;
            }
            else
                Pr_Index--;
            if (Pr_Index < Down_window_index)
            {
                Down_window_index = Pr_Index;
                Up_window_index = Down_window_index + 3;
            }
            //uint8_t pageNum = 13;
            // AckPrAdjIndex(0, pageNum);
        }
    }
    else
    {
        if (VrTimer1[holdButtonUp] > holdBtVrTimeUp)
        {
            holdBtVrTimeUp = 20;
            VrTimer1[holdButtonUp] = 0;
            Ack_Bup = 0;
        }
        else
            VrTimer1[holdButtonUp]++;
    }
}
void PageDisplay()
{
    uint16_t i;

    if (calibWeightStr)
    {
        ObjSelect = (uint32_t*) &Pr_PacketCopy[calibWeightObj];
        ObjSelectStep = (uint32_t*) &LCD_Step_Calib;
        ObjSelectMax = (uint32_t*) &InMode_MaxParameter[5];
        ObjSelectMin = (uint32_t*) &InMode_MinParameter[5];
        ObjSelectFlag = 1;
        calibWeightStr = HomePage = 0;
        current_page = 10;
        page_change = 1;
        Ack_reset = 1;
        InMode_Mathtype = floatMath;
    }
    if (page_change == 1)
    {
        LoopValue = 0;
        Pr_Index = 0;
        Down_window_index = 0;
        Up_window_index = 3;
        if (save_pr)
        {
            save_pr = 0;
            // Rewrite parameter to system when return page 0
            for (i = 0; i < NumberOfParameter; i++)
            {
                *Pr_Packet[i] = Pr_PacketCopy[i];
            }
#ifdef SaveEeprom
            uint32_t *ui32Ptr = Pr_PacketCopy;
            EEPROMProgram(ui32Ptr, AddDataEeprom, NumberOfParameter * 4); // Polling method
#endif

        }
        if (current_page == 1 || current_page == 10)
        {
            for (i = 0; i < NumberOfParameter; i++)
            {
                Pr_PacketCopy[i] = *(uint32_t*) Pr_Packet[i];
            }
        }

        switch (current_page)
        {
        case 0:
            PagePointer =
                    (boostFlag) ? (&pageWelcom_Display) : (&Page0_Display);
            Ack_PrAdj = 0;
            break;
        case 1:
            PagePointer = &Page1_Display;
            Ack_PrAdj = 0;
            NumOfPr = 5;
            break;
        case 2:
            PagePointer = &Page2_Display;
            NumOfPr = 6;
            Ack_PrAdj = 0;
            break;
        case 3:
            PagePointer = &Page3_Display;
            NumOfPr = 2;
            Ack_PrAdj = 0;
            break;
        case 4:
            PagePointer = &Page4_Display;
            Ack_PrAdj = 0;
            break;
        case 5:
            PagePointer = &Page5_Display;
            Ack_PrAdj = 0;
            break;

        case 11:
            PagePointer = &Page8_Display;
            Offset = 32;
            Ack_PrAdj = 1;
            NumOfPr = 2;
            break;
        case 12:
            PagePointer = &Page9_Display;
            Offset = 34;
            Ack_PrAdj = 1;
            NumOfPr = 2;
            break;
        case 13:
            PagePointer = &PageManual_Display;
            NumOfPr = 7;
            break;
        case 14:
            PagePointer = &Page10_Display;
            Offset = 36;
            Ack_PrAdj = 1;
            NumOfPr = 2;
            break;

        case 15:
            PagePointer = &Page11_Display;
            Offset = 38;
            Ack_PrAdj = 1;
            NumOfPr = NumOfParTemperature;
            break;

        case 6:
            InitalizeModeParDisplay(0, "Espresso 1");
            goto Skip;
        case 7:
            InitalizeModeParDisplay(8, "Espresso 2");
            goto Skip;
        case 8:
            InitalizeModeParDisplay(16, "Special 1");
            goto Skip;
        case 9:
            InitalizeModeParDisplay(24, "Special 2");
            Skip: Ack_PrAdj = 1;
            NumOfPr = NumOfParInEachMode;
            PagePointer = &Page6_Display;
            break;
        case 10:
            PagePointer = &Page7_Display;
            break;

        }
        page_change = 0;

    }
    if (boostFlag != 0)
// VrTimer2 for display boot page
        (VrTimer1[displayBootPage] > 250) ?
                (boostFlag = 0, page_change = 1) : VrTimer1[displayBootPage]++;
//---------------------Refesh page--------------------------------------
    if (VrTimer1[refreshPage] > 12)   // VrTimer for refresh page
    {
        VrTimer1[refreshPage] = 0;
#ifdef uDma_SSI0


        if (currentcount == 0)
        {
            LCD_IMAGE_Send = LCD_IMAGE_Pri;
            LCD_IMAGE_Write = LCD_IMAGE_Sec;
            currentcount = 1;

        }

        else
        {
            currentcount = 0;
            LCD_IMAGE_Send = LCD_IMAGE_Sec;
            LCD_IMAGE_Write = LCD_IMAGE_Pri;

        }
       clearBuffer((void*) LCD_IMAGE_Write);

       LCD_Write_Dat(0);
        (*PagePointer)();


#endif
#ifndef uDma_SSI0
        (*PagePointer)();
        if (uDMAChannelModeGet(UDMA_CHANNEL_SW) == UDMA_MODE_STOP)
        {

            disp_bitmap(0, 0, (uint8_t*) LCD_IMAGE);
            clearBuffer((void*) LCD_IMAGE);
        }

#endif
    }
    else
    {
        VrTimer1[refreshPage]++;
    }
    LCD_TaskPointer = &GetButtonCmd;
}
void Vertical_Display(void)
{
    uint8_t temp_page = 7;
    uint8_t temp_col = 33;
    LCD_IMAGE[128 * temp_page + temp_col] = 0xFF;
    temp_col = 64;
    LCD_IMAGE[128 * temp_page + temp_col] = 0xFF;
    temp_col = 94;
    LCD_IMAGE[128 * temp_page + temp_col] = 0xFF;
}
void Page0_Display(void)
{
    uint8_t i;
    static bool upload;
    if (upload == 0)
    {
        if (!HoldMsgFlag && !InProcess)
            Copy_bitExImage((void*) bitmap);
        upload = 1;

    }
    uint32_t ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if (ui32Mode == UDMA_MODE_STOP && upload == 1)
    {
        upload = 0;
        HomePage = 1;

        static float tempGui, tempBolier;
        if (VrTimer1[displayTemperature] >= 5)
        {
            VrTimer1[displayTemperature] = 0;
            tempGui = *(float*) dataSentList[HotWaterTemp];
            tempBolier = *(float*) dataSentList[BoilerTemp];
        }
        else
            VrTimer1[displayTemperature]++;

        sprintf(Str_Temp, "%.1f", tempGui);     // hot water temperature
        Disp_Str_5x8_Image(7, 30, (uint8_t*) Str_Temp, LCD_IMAGE);
        sprintf(Str_Temp, "%.1f", tempBolier);     // hot water temperature
        Disp_Str_5x8_Image(7, 65, (uint8_t*) Str_Temp, LCD_IMAGE);

#ifdef debuglcd
        sprintf(Str_Temp, "%.0f", *(float*) dataSentList[ExtractionTime]);  // extraction time
        Disp_Str_5x8_Image(7, 15, (uint8_t*) Str_Temp, LCD_IMAGE);

        sprintf(Str_Temp, "%.0f", *(float*) dataSentList[tempExtrude]); // pulse volumeter
        Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);
#endif
        sprintf(Str_Temp, "%.0f", *(float*) dataSentList[PulseCount]); // pulse volumeter
        Disp_Str_5x8_Image(7, 1, (uint8_t*) Str_Temp, LCD_IMAGE);
        if (calibWeightFlag)
            Disp_Str_5x8_Image(5, 10, "Weight calibration", LCD_IMAGE);
        for (i = 0; i < 5; i++)
        {
            id_Page0 = idPage0Display[i];
            switch (id_Page0)
            {
            case 3: // Dang ve sinh thuoc (2 id)
            case 10: // ve sinh hoc ba (2 id)
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                /*                Str_Display = LCD_String_Page0[id_Page0 + 1];
                 Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0 + 1],
                 (uint8_t*) Str_Display, LCD_IMAGE);*/
                break;

                // Display image make cofffee process
            case 5: // Espresso 1
                Disp_20x20_Image(3, 82, (uint8_t*) bitmapCoffeeE1,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 6: // Espresso 2
                Disp_20x20_Image(3, 78, (uint8_t*) bitmapCoffeeE1,
                                 (uint8_t*) LCD_IMAGE);
                Disp_20x20_Image(3, 103, (uint8_t*) bitmapCoffeeE1,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 7: // Special 1
                Disp_20x20_Image(3, 90, (uint8_t*) bitmapCoffeeE2,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 8: // Special 2
                Disp_20x20_Image(3, 78, (uint8_t*) bitmapCoffeeE2,
                                 (uint8_t*) LCD_IMAGE);
                Disp_20x20_Image(3, 103, (uint8_t*) bitmapCoffeeE2,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 0:  // Da xan sang
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 255:   // None display image
                break;

                // Goto skip
                // Display mode coffee + Grind time + Extraction time
                skip: Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_8x16_Image(3, LCD_PosStr_page0[id_Page0],
                                    (uint8_t*) Str_Display);
                ////////////////////////////////////////////////////////////////
                DisplayInfoInProcess();
                ////////////////////////////////////////////////////////////////
                break;
            case 1: // Dang ve sinh
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 2:     // Tha thuoc ve sinh
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
                //case 4:
            case 9:     // Dang khoi dong
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 12:    // ma loi
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(7, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                DisplayError();
                break;
            case 13:
                // Ngam coi
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 14:
                // Canh bao
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(7, 95, (uint8_t*) Str_Display, LCD_IMAGE);
                DisplayWarning();
                break;
            }
        }
// Display number of cups
        if (idPage0Display[3] == 0xFE)
        {
            sprintf(Str_Temp, "%d", *dataSentList[cupsEspresso_1]);
            Disp_Str_5x8_Image(8, 9, (uint8_t*) Str_Temp, LCD_IMAGE);

            sprintf(Str_Temp, "%d", *dataSentList[cupsEspresso_2]);
            Disp_Str_5x8_Image(8, 39, (uint8_t*) Str_Temp, LCD_IMAGE);

            sprintf(Str_Temp, "%d", *dataSentList[cupsSpecial_1]);
            Disp_Str_5x8_Image(8, 69, (uint8_t*) Str_Temp, LCD_IMAGE);

            sprintf(Str_Temp, "%d", *dataSentList[cupsSpecial_2]);
            Disp_Str_5x8_Image(8, 99, (uint8_t*) Str_Temp,
                               (uint8_t*) LCD_IMAGE);
            Vertical_Display();

        }
    }
}
void Page1_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;
    Str_Display = "CAI DAT: ";
    Disp_Str_5x8_Image(1, 45, (uint8_t*) Str_Display, LCD_IMAGE);

//Str_Display = "Thong so: ";
    Str_Display = LCD_String_Page1[id];
    Disp_Str_5x8_Image(2, 2, (uint8_t*) Str_Display, LCD_IMAGE);
// disp_bitmap(2,50,(uint8_t*)str);
//Str_Display = "Canh bao: ";
    Str_Display = LCD_String_Page1[id + 1];
    Disp_Str_5x8_Image(4, 2, (uint8_t*) Str_Display, LCD_IMAGE);
//Str_Display = "Thong tin may: ";
    Str_Display = LCD_String_Page1[id + 2];
    Disp_Str_5x8_Image(6, 2, (uint8_t*) Str_Display, LCD_IMAGE);
//Str_Display = "Thong tin loi: ";
    Str_Display = LCD_String_Page1[id + 3];
    Disp_Str_5x8_Image(8, 2, (uint8_t*) Str_Display, LCD_IMAGE);
//-----------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(1, pointer_pos);
}
void Page2_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;
    Str_Display = "Thong so cai dat: ";
    Disp_Str_5x8_Image(1, 15, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = LCD_String_Page2[id];
    Disp_Str_5x8_Image(2, 2, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = LCD_String_Page2[id + 1];
    Disp_Str_5x8_Image(4, 2, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = LCD_String_Page2[id + 2];
    Disp_Str_5x8_Image(6, 2, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = LCD_String_Page2[id + 3];
    Disp_Str_5x8_Image(8, 2, (uint8_t*) Str_Display, LCD_IMAGE);

//-----------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(2, pointer_pos);
}
void Page3_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;

    Str_Display = "Canh bao";
    Disp_Str_5x8_Image(1, 40, (uint8_t*) Str_Display, LCD_IMAGE);
    Str_Display = "Cum xay: ";
    Disp_Str_5x8_Image(2, 2, (uint8_t*) Str_Display, LCD_IMAGE);
    Str_Display = "Cum chiec suat: ";
    Disp_Str_5x8_Image(4, 2, (uint8_t*) Str_Display, LCD_IMAGE);

//------------------------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(3, pointer_pos);

}
void Page4_Display(void)
{
    char strsrc[32];
    Str_Display = "Thong tin may";
    Disp_Str_5x8_Image(1, 25, (uint8_t*) Str_Display, LCD_IMAGE);

    strcpy(strsrc, "Nha sx: ");
    strcat(strsrc, (char*) dataSentList[Brand]);
    Disp_Str_5x8_Image(2, 1, (uint8_t*) strsrc, LCD_IMAGE);

    strcpy(strsrc, "Seri number: ");
    strcat(strsrc, (char*) dataSentList[SeriNumber]);
    Disp_Str_5x8_Image(4, 1, (uint8_t*) strsrc, LCD_IMAGE);

    strcpy(strsrc, "Model: ");
    strcat(strsrc, (char*) dataSentList[Model]);
    Disp_Str_5x8_Image(6, 1, (uint8_t*) strsrc, LCD_IMAGE);

    strcpy(strsrc, "Ngay SX: ");
    strcat(strsrc, (char*) dataSentList[ProductDate]);
    Disp_Str_5x8_Image(8, 1, (uint8_t*) strsrc, LCD_IMAGE);

}
void Page5_Display(void)
{
    uint8_t id, i, j = 0, pointer_pos;
    NumOfPr = 0;
    id = Down_window_index;
    uint8_t erStackDisplay[16] = { };
    Str_Display = "Thong tin loi";
    Disp_Str_5x8_Image(1, 20, (uint8_t*) Str_Display, LCD_IMAGE);

    for (i = 0; i < 16; i++)
    {
        if (ErrorMachine[i].ErrorFlag == NULL)
            continue;
        if (*ErrorMachine[i].ErrorFlag == true)
        {
            erStackDisplay[j] = i;
            j++;
            NumOfPr = j;
        }

    }
    for (i = id; i < id + NumOfPr; i++)
    {
        uint8_t erid = erStackDisplay[i];
        uint8_t pos = 2 * (i - id) + 2;
        Str_Display = ErrorMachine[erid].ErrorMsg;
        Disp_Str_5x8_Image(pos, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    }
//---------------------------------------------------------------------------------------------
    if (!Error)
        return;
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(4, pointer_pos);

}
void Page6_Display(void)
{
    uint8_t id, pointer_pos, i;

    uint8_t pInsertIsInterger[NumOfObjWindow];
    id = Down_window_index;
    pointer_pos = (Pr_Index - id);
    for (i = 0; i < NumOfObjWindow; i++)
        pInsertIsInterger[i] = (i == pointer_pos) ? true : false;
    char strsrc[32];
//--------------------------------------------------------------------------------
    Disp_Str_5x8_Image(1, 35, (uint8_t*) strMode, LCD_IMAGE);
    PrintNum(Int_Format_parameter[id], pInsertIsInterger[0],
             &Pr_PacketCopy[id + Offset]);
    strcpy(strsrc, LCD_String_Page6[id]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(2, 1, (uint8_t*) strsrc, LCD_IMAGE);
//--------------------------------------------------------------------------------
    PrintNum(Int_Format_parameter[id + 1], pInsertIsInterger[1],
             &Pr_PacketCopy[id + 1 + Offset]);
    strcpy(strsrc, LCD_String_Page6[id + 1]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(4, 1, (uint8_t*) strsrc, LCD_IMAGE);
//--------------------------------------------------------------------------------

    PrintNum(Int_Format_parameter[id + 2], pInsertIsInterger[2],
             &Pr_PacketCopy[id + 2 + Offset]);
    strcpy(strsrc, LCD_String_Page6[id + 2]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(6, 1, (uint8_t*) strsrc, LCD_IMAGE);
//--------------------------------------------------------------------------------

    PrintNum(Int_Format_parameter[id + 3], pInsertIsInterger[3],
             &Pr_PacketCopy[id + 3 + Offset]);
    strcpy(strsrc, LCD_String_Page6[id + 3]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(8, 1, (uint8_t*) strsrc, LCD_IMAGE);
//-----------------------------------------------------------------------------------
// Display cursor
//
//  Page_Cursor_Display(2, pointer_pos);

}
void Page7_Display(void)
{
    Str_Display = "Calibration";
    Disp_Str_8x16_Image(2, 25, (uint8_t*) Str_Display);
    Str_Display = "Thoi gian: ";
    Disp_Str_5x8_Image(5, 20, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%.1f", *(float*) &Pr_PacketCopy[calibWeightObj]);
    Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);
    Str_Display = "s";
    Disp_Str_5x8_Image(7, 70, (uint8_t*) Str_Display, LCD_IMAGE);
}

void Page8_Display()
{
    uint8_t id, pointer_pos;
    id = Down_window_index;

//------------------------------------------------------------------------
    Str_Display = "Cum xay";
    Disp_Str_5x8_Image(1, 50, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = "Da xay";
    Disp_Str_5x8_Image(3, 27, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = "Canh bao";
    Disp_Str_5x8_Image(3, 80, (uint8_t*) Str_Display, LCD_IMAGE);
//------------------------------------------------------------------------
    Str_Display = "Cum A";
    Disp_Str_5x8_Image(5, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%d", *dataSentList[Blade1NofTimesUsed]);
    Disp_Str_5x8_Image(5, 40, (uint8_t*) Str_Temp, LCD_IMAGE);
    uint8_t objectIndexBladeA = 32;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexBladeA]);
    Disp_Str_5x8_Image(5, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
    Str_Display = "Cum B";
    Disp_Str_5x8_Image(7, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%d", *dataSentList[Blade2NofTimesUsed]);
    Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);
    uint8_t objectIndexBladeB = 33;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexBladeB]);
    Disp_Str_5x8_Image(7, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2 + 3;
    Page_Cursor_Display(4, pointer_pos);

}
void Page9_Display()
{
    uint8_t id, pointer_pos;
    id = Down_window_index;

//------------------------------------------------------------------------
    Str_Display = "Cum chiec xuat";
    Disp_Str_5x8_Image(1, 35, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = "Da chiec xuat";
    Disp_Str_5x8_Image(3, 15, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = "Canh bao";
    Disp_Str_5x8_Image(3, 80, (uint8_t*) Str_Display, LCD_IMAGE);
//------------------------------------------------------------------------
    Str_Display = "Cum A";
    Disp_Str_5x8_Image(5, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%d", *(uint32_t*) dataSentList[ExtractAtimes]);
    Disp_Str_5x8_Image(5, 40, (uint8_t*) Str_Temp, LCD_IMAGE);

    uint8_t objectIndexExtractA = 34;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexExtractA]);
    Disp_Str_5x8_Image(5, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
    Str_Display = "Cum B";
    Disp_Str_5x8_Image(7, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%d", *(uint32_t*) dataSentList[ExtractBtimes]);
    Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);
    uint8_t objectIndexExtractB = 35;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexExtractB]);
    Disp_Str_5x8_Image(7, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2 + 3;
    Page_Cursor_Display(4, pointer_pos);
}
void Page10_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;

//------------------------------------------------------------------------
    Str_Display = "Gioi han thoi gian";
    Disp_Str_5x8_Image(1, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    Str_Display = "Max: ";
    Disp_Str_5x8_Image(3, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexMaxtimeExtract = 36;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexMaxtimeExtract]);
    Disp_Str_5x8_Image(3, 30, (uint8_t*) Str_Temp, LCD_IMAGE);

    Str_Display = "Min: ";
    Disp_Str_5x8_Image(5, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexMintimeExtract = 37;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexMintimeExtract]);
    Disp_Str_5x8_Image(5, 30, (uint8_t*) Str_Temp, LCD_IMAGE);
//------------------------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2 + 1;
    Page_Cursor_Display(1, pointer_pos);

}
void Page11_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;

//------------------------------------------------------------------------
    Str_Display = "Cai dat Nhiet do";
    Disp_Str_5x8_Image(1, 15, (uint8_t*) Str_Display, LCD_IMAGE);
//------------------------------------------------------------------------
    Str_Display = "nuoc: ";
    Disp_Str_5x8_Image(2, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%.0f", *(float*) dataSentList[HotWaterTemp]);
    Disp_Str_5x8_Image(2, 30, (uint8_t*) Str_Temp, LCD_IMAGE);

    Str_Display = "Set: ";
    Disp_Str_5x8_Image(2, 60, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexHotWaterSet = 38;
    sprintf(Str_Temp, "%.1f",
            *((float*) &Pr_PacketCopy[objectIndexHotWaterSet]));
    Disp_Str_5x8_Image(2, 90, (uint8_t*) Str_Temp, LCD_IMAGE);
//------------------------------------------------------------------------
    Str_Display = "Hoi: ";
    Disp_Str_5x8_Image(4, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%.0f", *(float*) dataSentList[BoilerTemp]);
    Disp_Str_5x8_Image(4, 30, (uint8_t*) Str_Temp, LCD_IMAGE);

    Str_Display = "Set: ";
    Disp_Str_5x8_Image(4, 60, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexSteamSet = 39;
    sprintf(Str_Temp, "%.1f", *((float*) &Pr_PacketCopy[objectIndexSteamSet]));
    Disp_Str_5x8_Image(4, 90, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
    Str_Display = "Coi:";
    Disp_Str_5x8_Image(6, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%.0f", *(float*) dataSentList[GroupTemp]);
    Disp_Str_5x8_Image(6, 30, (uint8_t*) Str_Temp, LCD_IMAGE);

    Str_Display = "Set:";
    Disp_Str_5x8_Image(6, 60, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexGroupTempSet = 40;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexGroupTempSet]);
    Disp_Str_5x8_Image(6, 90, (uint8_t*) Str_Temp, LCD_IMAGE);

    Str_Display = "Run: ";
    Disp_Str_5x8_Image(8, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexGroupDuty = 41;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexGroupDuty]);
    Disp_Str_5x8_Image(8, 30, (uint8_t*) Str_Temp, LCD_IMAGE);

    Str_Display = "Off: ";
    Disp_Str_5x8_Image(8, 60, (uint8_t*) Str_Display, LCD_IMAGE);
    uint8_t objectIndexGroupShutdowDuty = 42;
    sprintf(Str_Temp, "%d", Pr_PacketCopy[objectIndexGroupShutdowDuty]);
    Disp_Str_5x8_Image(8, 90, (uint8_t*) Str_Temp, LCD_IMAGE);
//------------------------------------------------------------------------
// Display cursor
    uint8_t temp;
    if (Pr_Index == 3)
    {
        temp = 9;
        id = 0;
    }
    else
    {
        temp = 6;
    }
    pointer_pos = (Pr_Index - id + 1) * 2;

    Page_Cursor_Display(temp, pointer_pos);

}
void PageManual_Display(void)
{
    Str_Display = "Manual Control";
    Disp_Str_5x8_Image(2, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    char MotorStrDisplay[10], strsrc[32];
    bool haveDirControl;
    uint8_t id, pointer_pos;
    id = Down_window_index;
    switch (maMotorSlected)
    {
    case GrindMotorSelect:
        sprintf(MotorStrDisplay, "%s", "xay");
        haveDirControl = 1;
        break;
    case PressMotorSelect:
        sprintf(MotorStrDisplay, "%s", "ep");
        haveDirControl = 1;
        break;
    case PumpMotorSelect:
        sprintf(MotorStrDisplay, "%s", "bom");
        haveDirControl = 0;
        break;
    }

    if (id == 0)
    {
        strcpy(strsrc, LCD_String_PageManual[id]);
        strcat(strsrc, MotorStrDisplay);
        Disp_Str_5x8_Image(3, 1, (uint8_t*) strsrc, LCD_IMAGE);
    }
    else
    {
        Str_Display = LCD_String_PageManual[id];
        Disp_Str_5x8_Image(3, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    }
    Str_Display = LCD_String_PageManual[id + 1];
    Disp_Str_5x8_Image(4, 1, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = LCD_String_PageManual[id + 2];
    Disp_Str_5x8_Image(5, 1, (uint8_t*) Str_Display, LCD_IMAGE);

    Str_Display = LCD_String_PageManual[id + 3];
    Disp_Str_5x8_Image(6, 1, (uint8_t*) Str_Display, LCD_IMAGE);

    uint8_t noteMsg = Pr_Index;
    switch (noteMsg)
    {
    case 0:
        if (haveDirControl)
            Str_Display = "Push Re to CCW or Cl to CW";
        else
            Str_Display = "Push Re to run";
        break;
    case 1:
    case 2:
    case 3:
        Str_Display = "Push Re to open valve";

        break;
    case 4:
        Str_Display = "Push Re to test volumemeter";
        break;
    }
    Disp_Str_5x8_Image(8, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    pointer_pos = (Pr_Index - id + 3);
    Page_Cursor_Display(1, pointer_pos);
}
void ManualControlSelect(void)
{
    if (current_page == 13)
    {
        InManualTest = true;
        if (Pr_Index == 0)
            ManualObjSelected = maMotorSlected;
        else
        {
            uint8_t offsetId_Manualtest = 2;
            ManualObjSelected = Pr_Index + offsetId_Manualtest;
        }
    }
    else
        InManualTest = 0;
}
void pageWelcom_Display()

{

    static bool i = 0;
    if (i == 0)
    {
        Copy_bitExImage((void*) bitmap);
        i = 1;
    }
    uint32_t ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if (ui32Mode == UDMA_MODE_STOP && i == 1)
    {
        i = 0;
        Str_Display = "XIN CHAO";
        Disp_Str_5x8_Image(7, 45, (uint8_t*) Str_Display, LCD_IMAGE);
    }
}
static inline void DisplayInfoInProcess(void)
{
// Display grind time
    sprintf(Str_Temp, "%.1f", *(float*) dataSentList[GrindTime]);
    Disp_Str_5x8_Image(6, 25, (uint8_t*) Str_Temp, LCD_IMAGE);
// Display preinfusion time
    sprintf(Str_Temp, "%.1f", *(float*) dataSentList[PreinfusionTime]);
    Disp_Str_5x8_Image(6, 65, (uint8_t*) Str_Temp, LCD_IMAGE);
// Display extraction time
    sprintf(Str_Temp, "%.0f", *(float*) dataSentList[ExtractionTime]);
    Disp_Str_5x8_Image(6, 105, (uint8_t*) Str_Temp, LCD_IMAGE);

}
void DisplayError(void)
{
    uint8_t i;
    static uint8_t imask = 0;
    if (Error)
    {
        if (VrTimer1[displayError] > 2)
        {
            for (i = imask; i < 16; i++)
            {
                if (ErrorMachine[i].ErrorFlag == NULL)
                    continue;
                if (*ErrorMachine[i].ErrorFlag == true)
                {
                    sprintf(Str_ErrorNum, "%d", i);
                    imask = i;
                    break;
                }

            }
            (i < 16) ? (imask++, VrTimer1[displayError] = 0) : (imask = 0);
        }
        else
            VrTimer1[displayError]++;

        char eStr[5] = "E";
        strcat(eStr, Str_ErrorNum);
        Disp_Str_5x8_Image(7, 45, (uint8_t*) eStr, LCD_IMAGE);
        return;
    }
    imask = 0;
}
void DisplayWarning(void)
{
    uint8_t i;
    static uint8_t imask = 0;
    if (WarningFlag)
    {
        if (VrTimer1[displayWarning] > 2)
        {
            for (i = imask; i < 5; i++)
            {
                if (WarningMachine[i].ErrorFlag == NULL)
                    continue;
                if (*WarningMachine[i].ErrorFlag == true)
                {
                    sprintf(Str_WarningNum, "%d", i);
                    imask = i;
                    break;
                }

            }
            (i < 5) ? (imask++, VrTimer1[displayWarning] = 0) : (imask = 0);
        }
        else
            VrTimer1[displayWarning]++;
        char wStr[5] = "W";
        strcat(wStr, Str_WarningNum);
        Disp_Str_5x8_Image(7, 115, (uint8_t*) wStr, LCD_IMAGE);
        return;
    }
    imask = 0;
}

void Page_Cursor_Display(uint8_t page, uint8_t pos)
{
    uint8_t n, col;
    col = CursorPosCol[page];
    n = col + 3;
    Str_Display = "<";
    Disp_Str_5x8_Image(pos, col, (uint8_t*) Str_Display, LCD_IMAGE);
    LCD_IMAGE[128 * (pos - 2) + n] = 0x80;

}
inline void PrintNum(MathType typePar, _Bool cur, void *Num)
{
    switch (typePar)
    {
    case intMath:
        sprintf(Str_Temp, "%d", *(uint32_t*) Num);
        break;
    case floatMath:
        sprintf(Str_Temp, "%.2f", *(float*) Num);
        break;
    case boolMath:
        if (*(uint8_t*) Num)
            sprintf(Str_Temp, "Co");
        else
            sprintf(Str_Temp, "Khong");
        break;
    }
    if (cur && blinkCur)
        strcat(Str_Temp, CurStr);
}
