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

#include "TCA9539.h"
extern TCA9539Regs TCA9539_IC1;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// System Defines
//---------------------------------------------------------------
#define Kernel_LCD_pr
#define NumOfPage       9
#define InitPageDisplay 3
// ---------------------Shared function prototypes of Interface LCD -------------------------------
extern void Disp_Str_5x8(volatile uint8_t page, volatile uint8_t column,
                         uint8_t *text);
extern void LCD_Disp_Clr(uint8_t dat);
extern void LCD_Write_Cmd(uint8_t cmd);
extern void LCD_Write_Data(uint8_t cmd);

extern uint8_t coeff_change;
extern uint8_t *LCD_IMAGE_Send, *LCD_IMAGE_Write;
extern uint8_t LCD_IMAGE_Pri[1024];
extern uint8_t LCD_IMAGE_Sec[1024];
extern volatile uint8_t currentBuffer;
// -------------------------------- KERNEL -------------------------------------
extern void SerialCommsInit(void);
extern void SerialHostComms(void);

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
void defaultcmd(void);
void Page_Cursor_Display(uint8_t page, uint8_t pos);   // Display cursor

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
extern void WriteImageToDriverLCD(void *bufferPtr);
extern void LCD_Write_Dat(uint8_t cmd);
extern int16_t VrTimer1[4];
extern void LCD_Address_Set(uint8_t page, uint8_t column);
uint8_t Pr_Index = 0, NumOfPr, Offset = 0;    // Indicator  parameter
uint8_t countList = 0;    // Count variable used to change page fuction
uint16_t *ObjSelect, *ObjSelectStep;  // Object selected and step Object pointer
uint8_t ObjSelectFlag = 0;
uint8_t current_page = 0, page_change = 1, layer = 0;
uint8_t CursorPosCol[NumOfPage] = { 100, 100, 100, 60, 60, 60, 100, 60, 60 };

// Acknowledge Button event
int8_t Ack_BSet = 0, Ack_Bdown = 0, Ack_Bup = 0, Ack_Enter = 0, Ack_Return = 0,
        Ack_reset = 0; // Used to avoid duplicate problem
volatile uint8_t BsetFlag, BdownFlag, BupFlag;     // Flag used for scan process
// Acknowledge Flag
uint8_t Ack_PrAdj = 0;
uint8_t Up_window_index = 3, Down_window_index = 0; // Windown display - 4 line

extern uint16_t *dataSentList[10]; //  Kernel terminal connect to monitor variable
extern uint16_t *Pr_Packet[16];     // Kernel terminal connect to parameter
uint16_t Pr_PacketCopy[16];         // Array copy pratameter
float Pr_Gui_Packet[10];            // Array for Gui display parameter
uint8_t save_pr, cpy_pr;

// --------------------------User variable used for display -----------------------------------------
char *Str_Display;
char Str_Temp[10];

char *LCD_String_Page6[5] =
        { "Nuoc tong", "thosi gian xay", "U/Nuoc T1", "U/T2" };    //

char *LCD_String_Page2[] = { "Epresso 1", "Epresso 2", "Decatt 1 ", "Decatt 2 ",
                             "Times    " };

char *LCD_Format_Parameter[4] = { "%d", "%d", "%d", "%d" };
uint16_t LCD_Step_Parameter[4] = { 1, 1, 1, 1 };
typedef struct Node Node_t;
typedef struct
{
    uint8_t page;
    Node_t *item[5];
    Node_t *returnN;
} Node;

Node Menu;
Node Mode, Warning, Specification, Reset;
Node Espresso_1, Espresso_2, Decatt_1, Decatt_2, Time;
Node *NodeSelected;
Node *MenuList[] = { &Mode, &Warning, &Specification, &Reset, NULL };
Node *nullnode[] = { NULL, NULL, NULL, NULL, NULL };
Node *ModeList[] = { &Espresso_1, &Espresso_2, &Decatt_1, &Decatt_2, &Time };
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
    AddNode(&Time, 10, NULL, &Mode);
    AddNode(&Warning, 3, NULL, &Menu);
    AddNode(&Specification, 4, NULL, &Menu);
    AddNode(&Reset, 5, NULL, &Menu);

}
void EnterNode()
{

    if (BupFlag == 0 && Ack_Enter == 0)
    {
        Ack_Enter = 1;
        Node *ptr = (Node*) NodeSelected->item[Pr_Index];
        if (ptr == NULL)
            return;
        NodeSelected = ptr;
        current_page = NodeSelected->page;
        page_change = 1;
    }

}
void ReturnNode()
{

    if (BdownFlag == 0 && Ack_Return == 0)
    {
        Ack_Return = 1;
        Node *ptr = (Node*) NodeSelected->returnN;
        if (ptr == NULL)
            return;
        NodeSelected = (Node*) NodeSelected->returnN;
        current_page = NodeSelected->page;
        page_change = 1;
    }
}
void SerialCommsInit(void)
{
    ButtonCmd_Init();
    LCD_TaskPointer = &GetButtonCmd;
    PagePointer = &Page0_Display;
    MenuInitialize();
    NodeSelected = &Menu;
}
void ButtonCmd_Init(void)
{
    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0); // unlock pin PF0
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN); // UP Button
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN); // Down Button
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN); // Set Button
// Weak pull-up
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_12MA,
    GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_12MA,
    GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_12MA,
    GPIO_PIN_TYPE_STD_WPU);
    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0);
}
void SerialHostComms(void)
{
    (*LCD_TaskPointer)();
}
void GetButtonCmd(void)
{

    BsetFlag = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);    // sw1
    BupFlag = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);     // sw2
    BdownFlag = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);

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
                ObjSelectStep = &LCD_Step_Parameter[Pr_Index];
                ObjSelectFlag = 1;

            }
            else if (ObjSelectFlag == 1)
            {
                ObjSelectFlag = 0;
                ObjSelect = 0x00;
            }
        }
    }

    else
    {
        if (VrTimer1[1] > 100 && Ack_reset == 0) // hold button set for change to page 0
        {
            Ack_reset = 1;
            VrTimer1[1] = ObjSelectFlag = 0;
            if (current_page == 0)
                current_page = 1;
            else
            {
                current_page = 0;
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

    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) != 0) // Release Set button
    {
        Ack_BSet = 0;
        Ack_Return = 0;
        Ack_Enter = Ack_reset = 0;
        CmdDispatcher = &defaultcmd;
        // if (Ack_PrAdj != 1)
        //page_change = 1;
    }
}
void ButtonDown_cmd()
{
    if ((Ack_Bdown == 0) && (current_page != 0))
    {
        Ack_Bdown = 1;
        if (ObjSelectFlag == 1)
        {
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
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1) == 1)   // Release Down button
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
    if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0) == 1)  // Release Up button
    {
        CmdDispatcher = &defaultcmd;
        Ack_Bup = 0;
    }

}
void PageDisplay()
{
    uint16_t i;
    if (page_change == 1)
    {
        Pr_Index = 0;
        LCD_Disp_Clr(0x00);
        Down_window_index = 0;
        Up_window_index = 3;
        if (save_pr)
        {                    // Write parameter
            for (i = 0; i < 16; i++)
            {
                *Pr_Packet[i] = Pr_PacketCopy[i];
            }
        }
        if (cpy_pr && current_page == 1)
        {                     // Read & modify parameter
            for (i = 0; i < 16; i++)
            {
                Pr_PacketCopy[i] = *Pr_Packet[i];
            }
        }
        switch (current_page)
        {
        case 0:
            PagePointer = &Page0_Display;
            Ack_PrAdj = 0;
            break;
        case 1:
            PagePointer = &Page1_Display;
            Ack_PrAdj = 0;
            NumOfPr = 4;
            break;
        case 2:
            PagePointer = &Page2_Display;
            Ack_PrAdj = 0;
            break;
        case 3:
            PagePointer = &Page3_Display;
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

        case 6:
            Offset = 0;
            goto Skip;
        case 7:
            Offset = 4;
            goto Skip;
        case 8:
            Offset = 8;
            goto Skip;
        case 9:
            Offset = 16;
            Skip: Ack_PrAdj = 1;
            NumOfPr = 4;
            PagePointer = &Page6_Display;
            break;

        }
        page_change = 0;

    }
#ifdef uDma_SSI0
    if (VrTimer1[0] > 10) // refesh page
    {
        VrTimer1[0] = 0;
        if (currentBuffer == 0)
        {
            LCD_IMAGE_Send = LCD_IMAGE_Pri;
            LCD_IMAGE_Write = LCD_IMAGE_Sec;
            currentBuffer = 1;

        }

        else
        {
            currentBuffer = 0;
            LCD_IMAGE_Send = LCD_IMAGE_Sec;
            LCD_IMAGE_Write = LCD_IMAGE_Pri;

        }
        clearBuffer((void*) LCD_IMAGE_Write);
        LCD_Write_Dat(0);
        (*PagePointer)();

    }
    else
    {
        VrTimer1[0]++;
    }
#endif
      (*PagePointer)();
    LCD_TaskPointer = &GetButtonCmd;

}

void Page0_Display(void)
{
    Str_Display = "Time";
    Disp_Str_5x8(1, 80, (uint8_t*) Str_Display);

    Str_Display = "Ten nha san xuat: ";
    Disp_Str_5x8(3, 10, (uint8_t*) Str_Display);
    Str_Display = "Trang thai";
    Disp_Str_5x8(5, 10, (uint8_t*) Str_Display);

    Str_Display = "Tong ly(ngay):";
    Disp_Str_5x8(7, 2, (uint8_t*) Str_Display);
    fflush(stdout);
    sprintf(Str_Temp, "%d", *dataSentList[0]);
    Disp_Str_5x8(8, 30, (uint8_t*) Str_Temp);

    Str_Display = "Tong ly:";
    Disp_Str_5x8(7, 90, (uint8_t*) Str_Display);
    fflush(stdout);
    sprintf(Str_Temp, "%d", *dataSentList[1]);
    Disp_Str_5x8(8, 90, (uint8_t*) Str_Temp);

}
void Page1_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;
    Str_Display = "Thong so: ";
    Disp_Str_5x8(2, 2, (uint8_t*) Str_Display);
    Str_Display = "Thong so canh bao: ";
    Disp_Str_5x8(4, 2, (uint8_t*) Str_Display);
    Str_Display = "Thong tin may: ";
    Disp_Str_5x8(6, 2, (uint8_t*) Str_Display);
    Str_Display = "Factory reset: ";
    Disp_Str_5x8(8, 2, (uint8_t*) Str_Display);
//-----------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(1, pointer_pos);
}
void Page2_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;
    Str_Display = LCD_String_Page2[0];
    Disp_Str_5x8(2, 2, (uint8_t*) Str_Display);

    Str_Display = LCD_String_Page2[id + 1];
    Disp_Str_5x8(4, 2, (uint8_t*) Str_Display);

    Str_Display = LCD_String_Page2[id + 2];
    Disp_Str_5x8(6, 2, (uint8_t*) Str_Display);

    Str_Display = LCD_String_Page2[id + 3];
    Disp_Str_5x8(8, 2, (uint8_t*) Str_Display);
//-----------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(2, pointer_pos);
}
void Page3_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;
    Str_Display = "Luoi xay: ";
    Disp_Str_5x8(2, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", *dataSentList[2]);
    Disp_Str_5x8(2, 50, (uint8_t*) Str_Temp);

    Str_Display = "Ron: ";
    Disp_Str_5x8(4, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", *dataSentList[3]);
    Disp_Str_5x8(4, 50, (uint8_t*) Str_Temp);
//-----------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(3, pointer_pos);

}
void Page4_Display(void)
{
    Str_Display = "Ten nha sx: ";
    Disp_Str_5x8(2, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", *dataSentList[4]);
    Disp_Str_5x8(2, 60, (uint8_t*) Str_Temp);

    Str_Display = "Serial: ";
    Disp_Str_5x8(4, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", *dataSentList[5]);
    Disp_Str_5x8(4, 60, (uint8_t*) Str_Temp);

    Str_Display = "Model: ";
    Disp_Str_5x8(6, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", *dataSentList[6]);
    Disp_Str_5x8(6, 60, (uint8_t*) Str_Temp);

    Str_Display = "Ngay san xuat: ";
    Disp_Str_5x8(8, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", *dataSentList[7]);
    Disp_Str_5x8(8, 60, (uint8_t*) Str_Temp);

}
void Page5_Display(void)
{
    Str_Display = "Reset all: ";
    Disp_Str_5x8(2, 2, (uint8_t*) Str_Display);
    Str_Display = "Reset: ";
}
void Page6_Display(void)
{
    uint8_t id, pointer_pos;
    id = Down_window_index;

    Str_Display = LCD_String_Page6[id];
    Disp_Str_5x8(2, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", Pr_PacketCopy[id + Offset]);
    Disp_Str_5x8(2, 80, (uint8_t*) Str_Temp);

    Str_Display = LCD_String_Page6[id + 1];
    Disp_Str_5x8(4, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", Pr_PacketCopy[id + 1 + Offset]);
    Disp_Str_5x8(4, 80, (uint8_t*) Str_Temp);

    Str_Display = LCD_String_Page6[id + 2];
    Disp_Str_5x8(6, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", Pr_PacketCopy[id + 2 + Offset]);
    Disp_Str_5x8(6, 80, (uint8_t*) Str_Temp);

    Str_Display = LCD_String_Page6[id + 3];
    Disp_Str_5x8(8, 2, (uint8_t*) Str_Display);
    sprintf(Str_Temp, "%d", Pr_PacketCopy[id + 3 + Offset]);
    Disp_Str_5x8(8, 80, (uint8_t*) Str_Temp);
//-----------------------------------------------------------
// Display cursor
    pointer_pos = (Pr_Index - id + 1) * 2;
    Page_Cursor_Display(2, pointer_pos);

}

void Page_Cursor_Display(uint8_t page, uint8_t pos)
{
    uint8_t i, n, col;
    n = (uint8_t) pos / 2;
    col = CursorPosCol[page];
    for (i = 1; i <= 4; i++)
    {
        if (n == i)
        {
            Str_Display = "<";
            Disp_Str_5x8(2 * i, col, (uint8_t*) Str_Display);
        }
        else
        {
            Str_Display = " ";
        }
        Disp_Str_5x8(2 * i, col, (uint8_t*) Str_Display);
    }

}
