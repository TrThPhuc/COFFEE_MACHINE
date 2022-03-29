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
#define NumOfPage       9
#define InitPageDisplay 3
#define AddDataEeprom   0x00

//------------------------------- Status machine --------------------------------
extern _Bool Error;

extern Error_t ErrorMachine[16];

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

void SerialCommsInit(void);
void SerialHostComms(void);

void ButtonCmd_Init(void);  // Initialize GPIO for command button
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

void defaultcmd(void);
void pageWelcom_Display(void);
void Page_Cursor_Display(uint8_t page, uint8_t pos);   // Display cursor
void DisplayError(void);
static void PrintNum(_Bool intP, _Bool cur, void *Num);
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
uint8_t ObjSelectFlag = 0;
bool floatmath = 0;
uint8_t current_page = 0, page_change = 1, layer = 0;
uint8_t CursorPosCol[NumOfPage] = { 124, 90, 80, 95, 124, 124, 124, 124, 124 };

// Acknowledge Button event
int8_t Ack_BSet = 0, Ack_Bdown = 0, Ack_Bup = 0, Ack_Enter = 0, Ack_Return = 0,
        Ack_reset = 0; // Used to avoid duplicate problem
volatile uint8_t BsetFlag, BdownFlag, BupFlag;     // Flag used for scan process
// Acknowledge Flag
uint8_t Ack_PrAdj = 0;
uint8_t Up_window_index = 3, Down_window_index = 0; // Windown display - 4 line
extern bool calibWeightFlag, calibWeightStr;
extern uint8_t calibWeightObj;
extern uint16_t *dataSentList[]; //  Kernel terminal connect to monitor variable
extern bool En, idleMachine, InCleanning, InProcess;
#define NumberOfParameter 32

extern uint32_t *Pr_Packet[NumberOfParameter]; // Kernel terminal connect to parameter
union NumConvert_u
{
    float floatNum;
    uint32_t unintNum;
};
union NumConvert_u NumConvert;

uint32_t Pr_PacketCopy[NumberOfParameter];         // Array copy pratameter
_Bool Pr_PacketCopyMask[NumberOfParameter] = { };
float Pr_Gui_Packet[10];            // Array for Gui display parameter
uint8_t save_pr, cpy_pr;

// --------------------------User variable used for display -----------------------------------------
_Bool boostFlag = 1, HomePage = 0;
char *Str_Display;
char Str_Temp[16];
char Str_ErrorNum[10];
char CurStr[3] = " <";
uint8_t blinkCurVr;
_Bool blinkCur;
char *LCD_String_Page6[8] =
        { "Pre-infusion(s): ", "Nuoc ra(xung): ", "Tgian xay(s): ",
          "Nhiet do(oC): ", "Do ep(mm): " }; //

char *LCD_String_Page2[10] = { "Espresso 1 ly", "Espresso 2 ly",
                               "Special 1 ly ", "Special 2 ly" };
char *LCD_String_Page0[16] = { "Da san sang", "Dang ve sinh",
                               "Tha thuoc ve sinh", "Dang ve sinh", "thuoc",
                               "Espresso", "Espresso", "Special", "Special",
                               "Dang khoi dong", "Ve sinh", "hoc dung ba",
                               "Ma Loi:" };
uint8_t id_Page0 = 0xFF, idModeRunning, idPage0Display[8];
extern float ppi;
char LCD_PosStr_page0[16] =
        { 34, 32, 15, 32, 55, 16, 8, 20, 10, 24, 46, 35, 10 };
bool Int_Format_parameter[5] = { 1, 1, 0, 0, 1 };
//char *LCD_Format_Parameter[5] = { "%d", "%d", "%d", "%.1f", "%d" };
union NumConvert_u LCD_Step[5] = { { .unintNum = 1 }, { .unintNum = 1 }, {
        .floatNum = 0.1 },
                                   { .floatNum = 0.5 }, { .unintNum = 1 } };
//uint32_t LCD_Step_Parameter[6] = { 1, 1, 1, 1, 1, 1 };

// preInfusion, time, Weightofpowder, grinding dur, Volume of water,
union NumConvert_u LCD_Max_Parameter[5] = { { .unintNum = 7 },
                                            { .unintNum = 250 }, { .floatNum =
                                                    18 },
                                            { .floatNum = 102 }, { .unintNum =
                                                    19 } };
////////////////////////////////////////////////////////////////////////////////////
union NumConvert_u LCD_Min_Parameter[5] = { { .unintNum = 4 },
                                            { .unintNum = 20 },
                                            { .floatNum = 5 },
                                            { .floatNum = 90 },
                                            { .unintNum = 11 } };
uint32_t MaxTimesBlade = 65000, MinTimesBlade = 2000, stepTimesBalde = 100;
uint32_t MaxTimesExtract = 90000, MinTimesExtract = 2000,
        stepTimesExtract = 100;
float LCD_Step_Calib = 0.1f;
char str[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x7C, 0x38,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
extern uint8_t bitmap[];
typedef struct Node Node_t;
typedef struct
{
    uint8_t page;
    Node_t *item[5];
    Node_t *returnN;
} Node;

Node Menu;
Node Mode, Warning, InfoMachine, ErrorList;
Node Espresso_1, Espresso_2, Decatt_1, Decatt_2;
Node GrindModule, ExtractionModule;
Node *NodeSelected;
Node *MenuList[] = { &Mode, &Warning, &InfoMachine, &ErrorList, NULL };

Node *nullnode[] = { NULL, NULL, NULL, NULL, NULL };
Node *ModeList[] = { &Espresso_1, &Espresso_2, &Decatt_1, &Decatt_2 };
Node *WarningList[] = { &GrindModule, &ExtractionModule };

void AddNode(Node *Node_pr, uint8_t _page, Node **Node_ch, Node *_returnN)
{
    uint8_t i;
    Node_pr->page = _page;

    for (i = 0; i < 5; i++)
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
    AddNode(&Warning, 3, (Node**) &WarningList, &Menu);
    AddNode(&GrindModule, 11, NULL, &Warning);
    AddNode(&ExtractionModule, 12, NULL, &Warning);

    AddNode(&InfoMachine, 4, NULL, &Menu);
    AddNode(&ErrorList, 5, NULL, &Menu);

}
void EnterNode()
{

    if (BupFlag == 0 && Ack_Enter == 0)
    {
        Ack_reset = 1;
        Ack_Enter = 1;
        Node *ptr = (Node*) NodeSelected->item[Pr_Index];
        if (ptr == NULL)
            return;
        NodeSelected = ptr;
        current_page = NodeSelected->page;
        page_change = 1;
        ObjSelectFlag = 0;
    }

}
void ReturnNode()
{

    if (BdownFlag == 0 && Ack_Return == 0)
    {
        Ack_Return = 1;
        Ack_reset = 1;
        Node *ptr = (Node*) NodeSelected->returnN;
        if (ptr == NULL)
            return;
        NodeSelected = (Node*) NodeSelected->returnN;
        current_page = NodeSelected->page;
        page_change = 1;
        ObjSelectFlag = 0;
    }
}
void SerialCommsInit(void)
{

    LCD_TaskPointer = &GetButtonCmd;
    PagePointer = &pageWelcom_Display;
    MenuInitialize();
    NodeSelected = &Menu;
}
void ButtonCmd_Init(void)
{
    // GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0); // unlock pin PF0
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN); // PF0 UP Button
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN); // PF1 Down Button
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_DIR_MODE_IN); // PF4 Set Button
    // Weak pull-up
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_12MA,
    GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_STRENGTH_12MA,
    GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_STRENGTH_12MA,
    GPIO_PIN_TYPE_STD_WPU);
    //GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0);

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
        VrTimer1[1] = 0;
        Ack_BSet = 1;
        if (Ack_PrAdj == 1)
        {
            if (ObjSelectFlag == 0)
            {
                ObjSelect = &Pr_PacketCopy[Pr_Index + Offset];
                switch (current_page)
                {
                case 6:
                case 7:
                case 8:
                case 9:
                    if (Int_Format_parameter[Pr_Index] == 0)
                        floatmath = 1;
                    ObjSelectStep = (uint32_t*) &LCD_Step[Pr_Index];
                    ObjSelectMax = (uint32_t*) &LCD_Max_Parameter[Pr_Index];
                    ObjSelectMin = (uint32_t*) &LCD_Min_Parameter[Pr_Index];
                    break;
                case 11:
                    ObjSelectStep = (uint32_t*) &stepTimesBalde;
                    ObjSelectMax = (uint32_t*) &MaxTimesBlade;
                    ObjSelectMin = (uint32_t*) &MinTimesBlade;
                    break;
                case 12:

                    ObjSelectStep = (uint32_t*) &stepTimesExtract;
                    ObjSelectMax = (uint32_t*) &MaxTimesExtract;
                    ObjSelectMin = (uint32_t*) &MinTimesExtract;
                    break;
                }
                ObjSelectFlag = 1;

            }
            else if (ObjSelectFlag == 1)
            {

                ObjSelectFlag = floatmath = 0;
                ObjSelect = 0x00;
            }
        }
    }
    else // Ack_BSet == 1
    {
        if (VrTimer1[1] > 150 && Ack_reset == 0) // hold button set for change to page 0
        {
            Ack_reset = 1;
            VrTimer1[1] = ObjSelectFlag = 0;
            if (current_page == 0)
            {
                current_page = 1;
                HomePage = 0;
            }
            else
            {
                current_page = 0;
                // Save parameter to eeprom
                VrTimer1[2] = 150;  // VrTimer2 for display boot page
                boostFlag = 1;
                save_pr = 1;
                NodeSelected = &Menu;

            }
            page_change = 1;

        }
        else
            VrTimer1[1]++;
    }
    if (current_page != 0)
    {
        EnterNode();
        ReturnNode();
    }
    if (ReadMenuB) // Release Set button //ReadMenuB
    {
        Ack_BSet = 0;
        Ack_Return = 0;
        Ack_Enter = Ack_reset = 0;
        CmdDispatcher = &defaultcmd;
    }
}
void ButtonDown_cmd()
{
    if ((Ack_Bdown == 0) && (current_page != 0))
    {
        Ack_Bdown = 1;
        if (ObjSelectFlag == 1 && (*ObjSelect != 0))
        {
            if (floatmath && (*(float*) ObjSelect > *(float*) ObjSelectMin))
            {
                NumConvert.floatNum = (*(float*) ObjSelect
                        - *(float*) ObjSelectStep);
                *ObjSelect = NumConvert.unintNum;
            }
            else if (*ObjSelect > *ObjSelectMin)
                *ObjSelect = *ObjSelect - *ObjSelectStep;

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
        }
    }
    if (ReadDownB) // Release Down button //ReadDownB
    {
        CmdDispatcher = &defaultcmd;
        Ack_Bdown = 0;
    }

}
void ButtonUp_cmd()
{
    if ((Ack_Bup == 0) && (current_page != 0))
    {
        Ack_Bup = 1;
        if (ObjSelectFlag == 1)
        {
            if (floatmath && (*(float*) ObjSelect < *(float*) ObjSelectMax))
            {
                NumConvert.floatNum = (*(float*) ObjSelect
                        + *(float*) ObjSelectStep);
                *ObjSelect = NumConvert.unintNum;
            }
            else if (*ObjSelect < *ObjSelectMax)
                *ObjSelect = *ObjSelect + *ObjSelectStep;

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

        }
    }
    if (ReadUpB) // Release Up button //ReadUpB
    {
        CmdDispatcher = &defaultcmd;
        Ack_Bup = 0;
    }

}
void PageDisplay()
{
    uint16_t i;

    if (calibWeightStr)
    {
        ObjSelect = (uint32_t*) &Pr_PacketCopy[calibWeightObj];
        ObjSelectStep = (uint32_t*) &LCD_Step_Calib;
        ObjSelectMax = (uint32_t*) &LCD_Max_Parameter[3];
        ObjSelectMin = (uint32_t*) &LCD_Min_Parameter[3];
        ObjSelectFlag = 1;
        calibWeightStr = HomePage = 0;
        current_page = 10;
        page_change = floatmath = 1;
    }
    if (page_change == 1)
    {

        Pr_Index = 0;
        Down_window_index = 0;
        Up_window_index = 3;
        if (save_pr)
        {
            save_pr = 0;
            // Rewrite parameter to system when return page 0
            for (i = 0; i <= 4; i++)
            {
                if (i != idMask)
                    Pr_PacketCopyMask[3 + 8 * i] = 1;
                else
                    Pr_PacketCopyMask[3 + 8 * i] = 0;

            }
            for (i = 0; i < NumberOfParameter; i++)
            {
                if (!Pr_PacketCopyMask[i])
                    *Pr_Packet[i] = Pr_PacketCopy[i];
                Pr_PacketCopy[i] = *Pr_Packet[i];
            }
#ifdef SaveEeprom
            uint32_t *ui32Ptr = Pr_PacketCopy;
            EEPROMProgram(ui32Ptr, AddDataEeprom, NumberOfParameter * 4); // Polling method

#endif

        }
        if (current_page == 1 || current_page == 10)
        {                     // Read & modify parameter
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
            NumOfPr = 4;
            break;
        case 2:
            PagePointer = &Page2_Display;
            NumOfPr = 4;
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
            Offset = 29;
            Ack_PrAdj = 1;
            NumOfPr = 2;
            break;
        case 12:
            PagePointer = &Page9_Display;
            Offset = 21;
            Ack_PrAdj = 1;
            NumOfPr = 2;
            break;

        case 6:
            Offset = 0;
            strMode = "Espresso 1";
            idMask = 0;
            goto Skip;
        case 7:
            Offset = 8;
            strMode = "Espresso 2";
            idMask = 1;
            goto Skip;
        case 8:
            Offset = 16;
            strMode = "Special 1";
            idMask = 2;
            goto Skip;
        case 9:
            Offset = 24;
            strMode = "Special 2";
            idMask = 3;
            Skip: Ack_PrAdj = 1;

            NumOfPr = 5;
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
        (VrTimer1[2] > 250) ? (boostFlag = 0, page_change = 1) : VrTimer1[2]++;
//---------------------Refesh page--------------------------------------
    if (VrTimer1[0] > 12)   // VrTimer for refresh page
    {
        VrTimer1[0] = 0;
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
        VrTimer1[0]++;
    }
    LCD_TaskPointer = &GetButtonCmd;
}
void Vertical_Display(void)
{
//  LCD_Address_Set(8, 34);
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
        if (!InProcess)
            Copy_bitExImage((void*) bitmap);
        upload = 1;

    }
    uint32_t ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SW);
    if (ui32Mode == UDMA_MODE_STOP && upload == 1)
    {
        upload = 0;
        HomePage = 1;
        //  static float tempGui;
        if (VrTimer1[3] >= 15)
        {
            VrTimer1[3] = 0;
            // tempGui = *(float*) dataSentList[HotWaterTemp];
        }
        else
            VrTimer1[3]++;
#ifdef debuglcd

    sprintf(Str_Temp, "%.1f", tempGui);     // hot water temperature
    Disp_Str_5x8_Image(7, 72, (uint8_t*) Str_Temp, LCD_IMAGE);

    sprintf(Str_Temp, "%.0f", *(float*) dataSentList[ExtractionTime]);  // extraction time
    Disp_Str_5x8_Image(7, 15, (uint8_t*) Str_Temp, LCD_IMAGE);

    sprintf(Str_Temp, "%.0f", *(float*) dataSentList[tempExtrude]); // pulse volumeter
    Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);
#endif
        if (calibWeightFlag)
            Disp_Str_5x8_Image(5, 10, "Weight calibration", LCD_IMAGE);
// Copy_bitExImage((void*) bitmap);
        for (i = 0; i < 4; i++)
        {
            id_Page0 = idPage0Display[i];
            switch (id_Page0)
            {
            case 3: // Dang ve sinh thuoc (2 id)
            case 10: // ve sinh hoc ba (2 id)
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                Str_Display = LCD_String_Page0[id_Page0 + 1];
                Disp_Str_5x8_Image(7, LCD_PosStr_page0[id_Page0 + 1],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;

                // Display image make cofffee process
            case 5:
                Disp_20x20_Image(3, 82, (uint8_t*) bitmapCoffeeE1,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 6:
                Disp_20x20_Image(3, 78, (uint8_t*) bitmapCoffeeE1,
                                 (uint8_t*) LCD_IMAGE);
                Disp_20x20_Image(3, 103, (uint8_t*) bitmapCoffeeE1,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 7:
                Disp_20x20_Image(3, 90, (uint8_t*) bitmapCoffeeE2,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;
            case 8:
                Disp_20x20_Image(3, 78, (uint8_t*) bitmapCoffeeE2,
                                 (uint8_t*) LCD_IMAGE);

                Disp_20x20_Image(3, 103, (uint8_t*) bitmapCoffeeE2,
                                 (uint8_t*) LCD_IMAGE);
                goto skip;

            case 0:
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 255:
                break;
                skip: Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_8x16_Image(3, LCD_PosStr_page0[id_Page0],
                                    (uint8_t*) Str_Display);
                sprintf(Str_Temp, "%.1f", ppi);         // hot water temperature
                Disp_Str_5x8_Image(7, 82, (uint8_t*) Str_Temp, LCD_IMAGE);

                sprintf(Str_Temp, "%.0f",
                        *(float*) dataSentList[ExtractionTime]); // extraction time
                Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);

                break;
            case 1:
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(7, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 2:     // tha thuoc ve sinh
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(7, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
                //case 4:
            case 9:     // dang khoi dong
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(6, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                break;
            case 12:    // ma loi
                Str_Display = LCD_String_Page0[id_Page0];
                Disp_Str_5x8_Image(8, LCD_PosStr_page0[id_Page0],
                                   (uint8_t*) Str_Display, LCD_IMAGE);
                DisplayError();
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

    Str_Display = "Thong so: ";
    Disp_Str_5x8_Image(2, 2, (uint8_t*) Str_Display, LCD_IMAGE);
// disp_bitmap(2,50,(uint8_t*)str);
    Str_Display = "Canh bao: ";
    Disp_Str_5x8_Image(4, 2, (uint8_t*) Str_Display, LCD_IMAGE);
    Str_Display = "Thong tin may: ";
    Disp_Str_5x8_Image(6, 2, (uint8_t*) Str_Display, LCD_IMAGE);
    Str_Display = "Thong tin loi: ";
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

    Str_Display = LCD_String_Page2[0];
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
    uint8_t pInsert[4];
    id = Down_window_index;
//pointer_pos = (Pr_Index - id + 1) * 2;
    pointer_pos = (Pr_Index - id);
    for (i = 0; i < 4; i++)
        pInsert[i] = (i == pointer_pos) ? true : false;
    char strsrc[32];

//--------------------------------------------------------------------------------
    Disp_Str_5x8_Image(1, 35, (uint8_t*) strMode, LCD_IMAGE);
    PrintNum(Int_Format_parameter[id], pInsert[0], &Pr_PacketCopy[id + Offset]);
    strcpy(strsrc, LCD_String_Page6[id]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(2, 1, (uint8_t*) strsrc, LCD_IMAGE);
//--------------------------------------------------------------------------------
    PrintNum(Int_Format_parameter[id + 1], pInsert[1],
             &Pr_PacketCopy[id + 1 + Offset]);
    strcpy(strsrc, LCD_String_Page6[id + 1]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(4, 1, (uint8_t*) strsrc, LCD_IMAGE);
//--------------------------------------------------------------------------------

    PrintNum(Int_Format_parameter[id + 2], pInsert[2],
             &Pr_PacketCopy[id + 2 + Offset]);
    strcpy(strsrc, LCD_String_Page6[id + 2]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(6, 1, (uint8_t*) strsrc, LCD_IMAGE);
//--------------------------------------------------------------------------------

    PrintNum(Int_Format_parameter[id + 3], pInsert[3],
             &Pr_PacketCopy[id + 3 + Offset]);
    strcpy(strsrc, LCD_String_Page6[id + 3]);
    strcat(strsrc, Str_Temp);
    Disp_Str_5x8_Image(8, 1, (uint8_t*) strsrc, LCD_IMAGE);
//-----------------------------------------------------------------------------------
// Display cursor
//
//  Page_Cursor_Display(2, pointer_pos);

}
void Page7_Display()
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

    sprintf(Str_Temp, "%d", Pr_PacketCopy[29]);
    Disp_Str_5x8_Image(5, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
    Str_Display = "Cum B";
    Disp_Str_5x8_Image(7, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%d", *dataSentList[Blade2NofTimesUsed]);
    Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);

    sprintf(Str_Temp, "%d", Pr_PacketCopy[30]);
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

    sprintf(Str_Temp, "%d", Pr_PacketCopy[21]);
    Disp_Str_5x8_Image(5, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
    Str_Display = "Cum B";
    Disp_Str_5x8_Image(7, 1, (uint8_t*) Str_Display, LCD_IMAGE);
    sprintf(Str_Temp, "%d", *(uint32_t*) dataSentList[ExtractBtimes]);
    Disp_Str_5x8_Image(7, 40, (uint8_t*) Str_Temp, LCD_IMAGE);

    sprintf(Str_Temp, "%d", Pr_PacketCopy[22]);
    Disp_Str_5x8_Image(7, 85, (uint8_t*) Str_Temp, LCD_IMAGE);

//------------------------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2 + 3;
    Page_Cursor_Display(4, pointer_pos);
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
void DisplayError(void)
{
    uint8_t i;
    static uint8_t imask = 0;

    if (Error)
    {
        if (VrTimer1[5] > 2)
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
            (i < 16) ? (imask++, VrTimer1[5] = 0) : (imask = 0);
        }
        else
            VrTimer1[5]++;

        char eStr[5] = "E";
        strcat(eStr, Str_ErrorNum);
        Disp_Str_5x8_Image(8, 50, (uint8_t*) eStr, LCD_IMAGE);
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
inline void PrintNum(_Bool intP, _Bool cur, void *Num)
{

    if (intP)
        sprintf(Str_Temp, "%d", *(uint32_t*) Num);
    else
        sprintf(Str_Temp, "%.1f", *(float*) Num);
    if (cur && blinkCur)
        strcat(Str_Temp, CurStr);
}
