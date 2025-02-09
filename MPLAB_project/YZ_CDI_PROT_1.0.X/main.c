/*--------------------------------------------------------------------------
 YZ125/250 CDI system
------------------------------------------------------------------------- */

/*
 Updates
 DATE           VERSION  UPDATE
 09/DEC/2024    1.00     DEVEROPMENT START
 08/FEB/2025    1.02     FIRST TEST FOR YZ250
 08/FEB/2025    1.03     RS232C TEST V1
 
 Version    a.b.c
            | | + Minor version up with only software change
            | + - Minor version up with hardware change
            + - - Major version up 
 */

#include<xc.h>
#include<stdint.h>
#include<stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "yz_cdi.h"
#include "constant.h"

#define _XTAL_FREQ 32000000

//-------------------------------
// PIN I/O
//-------------------------------
//output
#define PWJOUT  LATA0   //Solenoid drive signal for PWK38(with power jet)
#define IGOUT   LATC1   //Digital iginition output
#define IGEN    LATC2   //0:PU2 ignition is ENABLED 1:PU2 Ignition is DISABLED
//input
#define PU1IN   RC0     //1st Pick up signal detect (falling edge of 1to 0)
#define PU2IN   RA2     //2nd Pick up signal detect (rising edge of 0to 1)
#define PWJ_SEL RC5     //pwj enable or disable select input
#define REV_SEL RA4     //Rev limitter enable or disable select input 
#define ADST_1  PORTCbits.RC4     //Advance start rpm setting input 1
#define ADST_2  PORTCbits.RC3     //Advance start rpm setting input 2
#define MAXAD_1  PORTCbits.RC6    //Maximum advance setting input 1
#define MAXAD_2  PORTCbits.RC7    //Maximum advance setting input 2
#define GRAD_1  PORTBbits.RB7     //Grad. of advanve setting input 1
#define GRAD_2  PORTBbits.RB6     //Grad. of advanve setting input 2
#define ADRV_1  PORTBbits.RB5     //Minimun retard @Hi speed setting input 1
#define ADRV_2  PORTBbits.RB4     //Minimun retard @Hi speed setting input 2

//-------------------------------
// Proto type
//-------------------------------
void main(void);
void initialize_system(void);
void __interrupt() InterruptManager(void);
void check_sw_state(void);
void calc_map(void);
void ignition_disable(void);
void ccp1_enable(void);
void ccp1_disable(void);
void ccp2_enable(void);
void ccp2_disable(void);
void Write_Byte(char chr);
void WriteString(const char *str);
void tx_data_pc(void);
void Write_table(void);
void UART_print(char moji[]);

//-------------------------------
// Engine state
//-------------------------------

typedef enum {
    EG_LOW,
    EG_RUN,
} EG_STATE;

typedef enum {
    PWJ_ENABLE,
    PWJ_DISABLE,
} PWJ_STATE;

typedef enum {
    REVLIMIT_ENABLE,
    REVLIMIT_DISABLE,
} REVLIMIT_STATE;
//

//-------------------------------
// CDI definition
//-------------------------------
#define IG_GATE_OFF         (0)     //IGBT gate driver input OFF
#define IG_GATE_ON          (1)     //IGBT gate driver input ON
#define IG_DISABLE          (1)     //IGBT gate driber enable pin OFF
#define IG_ENABLE           (0)     //IGBT gate driber enable pin ON
#define FIXED_IG_RPM        (15)    //Fixed ignition timing RPM
#define MAX_MAP_RPM         (130)   //Max RPM of ignition map
#define REVLIMIT_L          (97)    //Rev limitter enable Low RPM. Ignition once every 2 revolutions
#define REVLIMIT_M          (98)    //Rev limitter enable Mid RPM. Ignition once every 3 revolutions
#define REVLIMIT_H          (99)    //Rev limitter enable Hi RPM. Ignition is disabled
#define PWJ_CUT_RPMH        (85)    //Power jet cut rpm @Power jet Enable
#define PWJ_CUT_RPML        (83)    //For hysteresis
#define PWJ_DISABLE_RPMH    (30)    //Power jet cut rpm @Power jet Disaable
#define PWJ_DISABLE_RPML    (28)    //For hysteresis

//-------------------------------
// Ignition map setting
//-------------------------------

#define PU1_deg                 (3500)  //*100deg
#define PU2_deg                 (500)   //*100deg PU2 timing=Fixed ignition timing @low RPM
#define Ret_start_rpm           (55)    //*1/100rpm
#define Ret_end_rpm             (80)    //*1/100rpm
#define deg2time_coefficient    (1667)  //For calculate ignition deg to waiting time from PU1 (600,000/360)=1667

const uint8_t adv_start_rpm_table[4] = {45, 35, 25, 15}; //*100rpm
const uint16_t max_adv_table[4] = {PU2_deg + 2000, PU2_deg + 1600, PU2_deg + 1200, PU2_deg + 800}; //deg
const uint8_t max_adv_grad_table[4] = {40, 30, 20, 10}; //*100rpm
const uint16_t min_ret_table[4] = {PU2_deg + 1000, PU2_deg + 800, PU2_deg + 600, PU2_deg + 400};

//-------------------------------
// Ignition map
// map No. 0  1   2 ... 15   16   17 ... 130 
// rpm     0 100 200...1500 1600 1700...13000(max)
// Ig_table is shown in BTDC angle. At PU1 input, "rpm" is calculated during interruput sub.
// And ignition timing(angle) is read from IG_table based on that rpm.
// Ignition timing angle is then converted to the waiting time from PU1.
//-------------------------------
uint16_t IG_table[131] = {0x0000};
uint24_t deg2time_coeff[131] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2276, 2133, 2008, 1896, 1796,
    1707, 1625, 1552, 1484, 1422, 1365, 1313, 1264, 1219, 1177, 1138, 1101, 1067, 1034, 1004, 975, 948, 923, 898, 875,
    853, 833, 813, 794, 776, 759, 742, 726, 711, 697, 683, 669, 656, 644, 632, 621, 610, 599, 589, 579,
    569, 560, 551, 542, 533, 525, 517, 509, 502, 495, 488, 481, 474, 468, 461, 455, 449, 443, 438, 432,
    427, 421, 416, 411, 406, 402, 397, 392, 388, 384, 379, 375, 371, 367, 363, 359, 356, 352, 348, 345,
    341, 338, 335, 331, 328, 325, 322, 319, 316, 313, 310, 308, 305, 302, 299, 297, 294, 292, 289, 287,
    284, 282, 280, 278, 275, 273, 271, 269, 267, 265, 263
};

//-------------------------------
// global variables
//-------------------------------
uint8_t rpm = 0;
uint8_t orev_counter = 0;
uint16_t ig_counter = 0;
uint8_t map_sel = 0;
uint8_t EG_state = 0;
uint8_t revlimit_state = 0;
uint8_t pwj_state = 0;
uint8_t test = 0;
const uint16_t numerator_rpm = 37500;
uint8_t sw1_pos = 2;
uint8_t sw2_pos = 3;
uint8_t sw3_pos = 3;
uint8_t sw4_pos = 3;

uint8_t a;
uint8_t str3[2] = {
    40, 151
};

//-------------------------------
// main
//-------------------------------

void main() {
    initialize_system();
    IGEN = IG_ENABLE;
    check_sw_state();
    calc_map();
    ccp1_enable();

    while (1) {
        check_sw_state();
        CLRWDT();
        //tx_data_pc();
        Write_table();
    }
}
//-------------------------------
// UART Transmit engine data for PC
//-------------------------------

void tx_data_pc(void) {
    char bc = 15;
    WriteString("11,50");
    WriteString("\r\n");
    a++;
    if (a == 1) a = 0;
}

//-------------------------------
// UART wirte test table
//-------------------------------

void Write_table() { //1バイト送信関数
    char a[10];
    //while (!TRMT); //送信バッファーが空になるまで待つ
    //TX1REG = str3[1]; ////送信バッファーに1バイト書込み・送信
    sprintf(a,"%d",str3[1]);     //mojiにcounter:countを代入
    WriteString(a);
    WriteString("\r\n");
}

//-------------------------------
// UART wirte 1byte
//-------------------------------

void Write_Byte(char chr) { //1バイト送信関数
    while (!TRMT); //送信バッファーが空になるまで待つ
    TX1REG = chr; ////送信バッファーに1バイト書込み・送信
}

//-------------------------------
// UART wirte string
//-------------------------------

void WriteString(const char *str) { //文字列送信関数
    while (*str) {
        Write_Byte(*str); //データ送信
        str++;
    }
}

//-------------------------------
// Calculate ignition map
//-------------------------------

void calc_map() {
    uint8_t p1x, p2x, p3x, p4x;
    uint16_t p1y, p2y, p3y, p4y;
    uint8_t coeff_p1_p2, coeff_p3_p4;
    uint8_t a;
    uint24_t temp;
    uint24_t temp1;

    p1x = adv_start_rpm_table[sw1_pos];
    p2x = adv_start_rpm_table[sw1_pos] + max_adv_grad_table[sw3_pos];
    p3x = Ret_start_rpm;
    p4x = Ret_end_rpm;
    p1y = PU2_deg;
    p2y = max_adv_table[sw2_pos];
    p3y = p2y;
    p4y = min_ret_table[sw4_pos];
    coeff_p1_p2 = (uint8_t) ((p2y - p1y) / (p2x - p1x));
    coeff_p3_p4 = (uint8_t) ((p3y - p4y) / (p4x - p3x));

    //calc iginition timing (deg)
    for (a = 15; a <= p1x; a++) {
        IG_table[a] = p1y;
    }
    for (a = p1x + 1; a <= p2x; a++) {
        IG_table[a] = coeff_p1_p2 + IG_table[a - 1];
    }
    for (a = p2x + 1; a <= p3x; a++) {
        IG_table[a] = p3y;
    }
    for (a = p3x + 1; a <= p4x; a++) {
        IG_table[a] = IG_table[a - 1] - coeff_p3_p4;
    }
    for (a = p4x + 1; a <= 130; a++) {
        IG_table[a] = p4y;
    }
    for (a = 15; a <= 130; a++) {
        temp1 = ((PU1_deg - IG_table[a]) >> 1);
        temp = ((deg2time_coeff[a] * temp1) >> 10);
        IG_table[a] = temp;
    }
}

//-------------------------------
// Check switch state
//-------------------------------

void check_sw_state() {
    switch (PWJ_SEL) {
    case 0:
        pwj_state = PWJ_DISABLE;
        break;
    case 1:
        pwj_state = PWJ_ENABLE;
        break;
    }
    switch (REV_SEL) {
    case 0:
        revlimit_state = REVLIMIT_DISABLE;
        break;
    case 1:
        revlimit_state = REVLIMIT_ENABLE;
        break;
    }

    sw1_pos = (ADST_1 << 1) + ADST_2;
    sw2_pos = (MAXAD_1 << 1) + MAXAD_2;
    //sw3_pos = (GRAD_1 << 1) + GRAD_2;
    sw3_pos = 3; //disable sw3 select for uart 
    sw4_pos = (ADRV_1 << 1) + ADRV_2;
}

//-------------------------------
// Interrupt
//-------------------------------

void __interrupt() InterruptManager() {
    uint16_t t1_count;
    //PU1 input change detect
    if (CCP1IF) {
        if (EG_state == EG_RUN) {
            TMR1ON = 0;
            TMR1H = 0x00;
            TMR1L = 0x00;
            TMR1ON = 1;
            ccp1_disable();

            rpm = (uint8_t) (numerator_rpm / (CCPR1 >> 4));

            if ((rpm > FIXED_IG_RPM)&&(rpm <= MAX_MAP_RPM)) {
                ig_counter = IG_table[rpm];
                IGEN = IG_ENABLE;
                if ((ig_counter - 15) > TMR1) {
                    CCPR2 = ig_counter;
                    ccp2_enable();
                } else {
                    ccp2_disable();
                    IGOUT = IG_GATE_ON;
                    __delay_us(60);
                    IGOUT = IG_GATE_OFF;
                }
            }//Disable ditital map ignition under 1500rpm or over 13000rpm
            else {
                ccp2_disable();
                IGEN = IG_ENABLE;
            }

            //Rev limit controll
            if (revlimit_state == REVLIMIT_ENABLE) {
                if (rpm > REVLIMIT_L) {
                    orev_counter++;
                    if (orev_counter == 1) ignition_disable();
                } else if (rpm > REVLIMIT_M) {
                    orev_counter++;
                    if (orev_counter == 2) ignition_disable();
                } else if (rpm > REVLIMIT_H) ignition_disable();
            }

            //Power jet controll
            if (pwj_state == PWJ_ENABLE) {
                if (rpm > PWJ_CUT_RPMH) PWJOUT = 1;
                else if (rpm < PWJ_CUT_RPML) PWJOUT = 0;
            } else if (pwj_state == PWJ_DISABLE) {
                if (rpm > PWJ_DISABLE_RPMH) PWJOUT = 1;
                else if (rpm < PWJ_DISABLE_RPML) PWJOUT = 0;
            }
        }

        if (EG_state == EG_LOW) {
            TMR1 = 0;
            TMR1ON = 1;
            EG_state = EG_RUN;
        }
        CCP1IF = 0;
    }
    //ignition by CCP2 compare mode.ignition is done automaticaly by CCP2
    if (CCP2IF) {
        __delay_us(60);
        ccp2_disable();
        IGOUT = 0;
        ccp1_enable();
        if (rpm < 40) calc_map();
    }

    //Prevent reverse rotation  ex)stop at hill climbe
    if (IOCAF2) {
        uint16_t pu1_2_period_count;
        if (EG_state == EG_RUN) {
            pu1_2_period_count = TMR1;
            if ((rpm < 25)&&((t1_count - pu1_2_period_count)<(pu1_2_period_count << 2))) {
                IGEN = IG_DISABLE;
            }
            ccp1_enable();
            if (rpm < 40) calc_map();
        }
        IOCAF2 = 0;
    }
    //If low rpm or stop
    if (TMR1IF) {
        EG_state = EG_LOW;
        TMR1ON = 0;
        TMR1 = 0;
        CCPR1 = 0;
        CCPR2 = 0;
        TMR1IF = 0;
        PWJOUT = 0;
        ccp2_disable();
        IGOUT = 0;
    }
    CLRWDT();
}

//-------------------------------
// Disaable ignition sub
//-------------------------------

void ignition_disable(void) {
    ccp2_disable();
    IGEN = IG_DISABLE;
    orev_counter = 0;
    ccp1_enable();
}

//-------------------------------
// CCP1 enable sub
//-------------------------------

void ccp1_enable(void) {
    CCP1IE = 0;
    CCP1IF = 0;
    CCP1CON = 0x84;
    CCP1IE = 1;
}

//-------------------------------
// CCP1 disable sub
//-------------------------------

void ccp1_disable(void) {
    CCP1IE = 0;
    CCP1CON = 0;
}

//-------------------------------
// CCP2 enable sub
//-------------------------------

void ccp2_enable(void) {
    CCP2IE = 0;
    CCP2IF = 0;
    CCP2CON = 0x88;
    CCP2IE = 1;
}

//-------------------------------
// CCP2 disable sub
//-------------------------------

void ccp2_disable(void) {
    CCP2IE = 0;
    CCP2CON = 0;
}

//-------------------------------
// system initialize
//-------------------------------

void initialize_system(void) {
    //clock setting
    OSCEN = 0x40; //HFINTOSC ENABLE
    OSCFRQ = 0x05; //32MHz
    OSCTUNE = 0X00;

    //PORT rest
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    LATA = 0x00;
    LATB = 0x00;
    LATC = 0x00;
    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    //PORT setting
    TRISA = 0b00110100; //IN:RA2/4/5 
    TRISB = 0b10110000; //IN:RB4-5,7
    TRISC = 0b11111001; //IN:RC0/3-7 OUT:RC1/2
    INLVLA = 0b00110100; //RA2/4/5 is Schmitt triger
    INLVLB = 0b10110000; //RB4-5,7 is Schmitt triger
    INLVLC = 0b11111001; //RC0/3-7 is Schmitt triger

    //Timer1 setting for PU1 detection and Ignition
    T1CLK = 0b00000001; //Clock source is Fosc/4 = 500ns
    T1CON = 0b00110000; //1:8 Prescaler
    TMR1 = 0x0000;

    //AD setting for throttle position sensor

    //CCP setting
    CCP1CAP = 0x0; //CCP1 Pin is RC0 (Selected by CCP1PPS)
    CCP2CAP = 0x0; //CCP2 Pin is RC1 (Selected by CCP2PPS)
    CCPR1 = 0x0000;
    CCPR2 = 0x0000;

    //IOC setting
    IOCAN2 = 1; //RA2 negative edge detection

    //PPS setting
    PPSLOCK = 0x55; //Unlock PPS
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    CCP1PPS = 0x10; //input capture pin1 = RC0
    CCP2PPS = 0b00010001;
    RC1PPS = 0x02; //Output compare pin2 = RC1
    RX1PPS = 0x0F; //RX1 = RB7
    RB6PPS = 0x05; //TX1 = RB6
    PPSLOCK = 0x55; //lock PPS
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;

    //UART setting
    BAUD1CON = 0x00;
    RC1STA = 0x00;
    TX1STA = 0x00;
    SP1BRGL = 0x00;
    SP1BRGH = 0x00;
    //ABDEN disabled; WUE disabled; BRG16 8bit_generator; SCKP Non-Inverted; 
    BAUD1CON = 0x40;
    //ADDEN disabled; CREN disabled; SREN disabled; RX9 8-bit; SPEN enabled; 
    RC1STA = 0x80;
    //TX9D 0x0; BRGH lo_speed; SENDB sync_break_complete; SYNC asynchronous; TXEN enabled; TX9 8-bit; CSRC client; 
    TX1STA = 0x22;
    //SPBRGL 51; 
    SP1BRGL = 0x33;
    //SPBRGH 0; 
    SP1BRGH = 0x0;


    //Watch dog timer setting
    WDTCON = 0x13; //512ms interval

    //Clear all interrupt flag
    PIR0 = 0x0;
    PIR1 = 0x0;
    PIR2 = 0x0;

    //interrupt inable
    GIE = 1;
    PEIE = 1;
    CCP1IE = 1;
    CCP2IE = 1;
    TMR1IE = 1;
    IOCIE = 1;
    //TMR2IE =1;
    ccp2_disable();
    IGOUT = 0;
}