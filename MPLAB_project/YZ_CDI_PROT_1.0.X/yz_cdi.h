
/****************************************************
 TITLE: YZ_CDI config
 PIC: 16F15245
 DATE: 2023.12.06
 CODED BY: SHUKO-SHA
 OTHER:

****************************************************/
#ifndef INCLUDED_YZ_CDI_CONFIG_H
#define INCLUDED_YZ_CDI_CONFIG_H

//CONFIG1
#pragma config FEXTOSC = ECH     // External Oscillator Mode Selection bits->EC (external clock) 16 MHz and above
#pragma config RSTOSC = HFINTOSC_32MHZ     // Power-up Default Value for COSC bits->HFINTOSC (32 MHz)
#pragma config CLKOUTEN = OFF     // Clock Out Enable bit->CLKOUT function is disabled; I/O function on RA4
#pragma config VDDAR = HI     // VDD Range Analog Calibration Selection bit->Internal analog systems are calibrated for operation between VDD = 2.3V - 5.5V

//CONFIG2
#pragma config MCLRE = EXTMCLR     // Master Clear Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR
#pragma config PWRTS = PWRT_1     // Power-up Timer Selection bits->PWRT set at 1 ms
//#pragma config WDTE = SWDTEN     // WDT Operating Mode bits->WDT enabled/disabled by SEN bit
#pragma config WDTE = OFF     // WDT Operating Mode bits->WDT disabled; SEN is ignored
#pragma config BOREN = ON     // Brown-out Reset Enable bits->Brown-out Reset Enabled, SBOREN bit is ignored
#pragma config BORV = LO     // Brown-out Reset Voltage Selection bit->Brown-out Reset Voltage (VBOR) set to 1.9V
#pragma config PPS1WAY = ON     // PPSLOCKED One-Way Set Enable bit->The PPSLOCKED bit can be set once after an unlocking sequence is executed; once PPSLOCKED is set, all future changes to PPS registers are prevented
#pragma config STVREN = ON     // Stack Overflow/Underflow Reset Enable bit->Stack Overflow or Underflow will cause a reset

//CONFIG4
#pragma config BBSIZE = BB512     // Boot Block Size Selection bits->512 words boot block size
#pragma config BBEN = OFF     // Boot Block Enable bit->Boot Block is disabled
#pragma config SAFEN = OFF     // SAF Enable bit->SAF is disabled
#pragma config WRTAPP = OFF     // Application Block Write Protection bit->Application Block is not write-protected
#pragma config WRTB = OFF     // Boot Block Write Protection bit->Boot Block is not write-protected
#pragma config WRTC = OFF     // Configuration Registers Write Protection bit->Configuration Registers are not write-protected
#pragma config WRTSAF = OFF     // Storage Area Flash (SAF) Write Protection bit->SAF is not write-protected
#pragma config LVP = ON     // Low Voltage Programming Enable bit->Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored.

//CONFIG5
#pragma config CP = OFF     // User Program Flash Memory Code Protection bit->User Program Flash Memory code protection is disabled


#endif

