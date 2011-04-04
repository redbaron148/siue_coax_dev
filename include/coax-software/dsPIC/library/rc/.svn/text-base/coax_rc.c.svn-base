/*************************************************************
 *
 * Low-level sensor and control library for the Skybotix AG
 * helicopters, and particularly COAX
 *
 * Developed by:
 *  * Samir Bouabdallah samir.bouabdallah@skybotix.ch
 *  * Cedric Pradalier: cedric.pradalier@skybotix.ch
 *  * Gilles Caprari: gilles.capraru@skybotix.ch
 *  * Thomas Baumgartner
 * Send modification or corrections as patches (diff -Naur)
 * Copyright: Skybotix AG, 2009-2012
 * 
 * All rights reserved.
 * 
 * Skybotix API is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Skybotix API is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
 * 
 * 
 * Description:This software manages the RC receiver of the CoaX
 *************************************************************/

#include <libpic30.h>
#include <p33FJ256GP506.h>
#include <configs/coax_config.h>

#include "spi/spi.h"
#include "configs/coax_ports.h"
#include "rc/coax_rc.h"

/****** RC SIGNAL ******/
#define RSSI_THRESHOLD   0x80
#define ERROR_COUNTER_THRESHOLD_Level1 200    // channel seems disturbed -> 100ms without signal (@1.0ms Timer-Interrupt)
#define ERROR_COUNTER_THRESHOLD_Level2 2000   // link to RC lost -> 1s without signal (@1.0ms Timer-Interrupt)
#define MAX_SOP_WAIT_TIME 20 // Waiting for answers for 15ms
#define RESET_WAIT_TIME 1 // Waiting time after reset


unsigned char rcRawData[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned int rcData[8]={0,0,0,0,0,0,0,0};
float rcValue[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
int activeChannelIndex=0;
unsigned char channelList[3]={0xff,0xff,0xff};
RC_TYPE rcType = 0;
RC_LOWLEVEL_STATE rcState = RC_LOWLEVEL_INIT;
// unsigned char global_irq_status = 0;

unsigned int ERROR_Level1_COUNTER;
unsigned int ERROR_Level2_COUNTER;

float RC_PITCH        = 0.0;
float RC_ROLL        = 0.0;
float RC_THROTTLE   = 0.0;
float RC_YAW        = 0.0;

float RC_PITCH_TRIM      = 0.0;
float RC_ROLL_TRIM      = 0.0;
float RC_THROTTLE_TRIM   = 0.0;
float RC_YAW_TRIM      = 0.0;


void RCScaleChannels(void)
{
#define LIN(CHANNEL,RCTYPE) ((CHANNEL##_SLOPE*RCTYPE##_RAW_##CHANNEL)-CHANNEL##_SHIFT)
    switch (rcType) {
        case RC_WK2402:
            RC_PITCH        = LIN(PITCH,WK2402);
            RC_ROLL        = LIN(ROLL,WK2402);
            RC_THROTTLE     = LIN(THROTTLE,WK2402);
            RC_YAW           = LIN(YAW,WK2402);
            RC_PITCH_TRIM    = LIN(PITCH_TRIM,WK2402);
            RC_ROLL_TRIM    = LIN(ROLL_TRIM,WK2402);
            RC_THROTTLE_TRIM = LIN(THROTTLE_TRIM,WK2402);
            RC_YAW_TRIM       = LIN(YAW_TRIM,WK2402);
            break;
        case RC_WK2401:
        default:
            RC_PITCH        = LIN(PITCH,WK2401);
            RC_ROLL        = LIN(ROLL,WK2401);
            RC_THROTTLE     = LIN(THROTTLE,WK2401);
            RC_YAW           = LIN(YAW,WK2401);
            RC_PITCH_TRIM    = LIN(PITCH_TRIM,WK2401);
            RC_ROLL_TRIM    = LIN(ROLL_TRIM,WK2401);
            RC_THROTTLE_TRIM = LIN(THROTTLE_TRIM,WK2401);
            RC_YAW_TRIM       = LIN(YAW_TRIM,WK2401);
            break;
    }
#undef LIN

    if (RC_PITCH > 1.0)
        RC_PITCH = 1.0;
    else if (RC_PITCH < -1.0)
        RC_PITCH = -1.0;

    if (RC_ROLL > 1.0)
        RC_ROLL = 1.0;
    else if (RC_ROLL < -1.0)
        RC_ROLL = -1.0;

    if (RC_THROTTLE > 1.0)
        RC_THROTTLE = 1.0;
    else if (RC_THROTTLE < 0.0)
        RC_THROTTLE = 0.0;

    if (RC_YAW > 1.0)
        RC_YAW = 1.0;
    else if (RC_YAW < -1.0)
        RC_YAW = -1.0;

    if (RC_PITCH_TRIM > 1.0)
        RC_PITCH_TRIM = 1.0;
    else if (RC_PITCH_TRIM < -1.0)
        RC_PITCH_TRIM = -1.0;

    if (RC_ROLL_TRIM > 1.0)
        RC_ROLL_TRIM = 1.0;
    else if (RC_ROLL_TRIM < -1.0)
        RC_ROLL_TRIM = -1.0;

    if (RC_THROTTLE_TRIM > 1.0)
        RC_THROTTLE_TRIM = 1.0;
    else if (RC_THROTTLE_TRIM < 0.0)
        RC_THROTTLE_TRIM = 0.0;

    if (RC_YAW_TRIM > 1.0)
        RC_YAW_TRIM = 1.0;
    else if (RC_YAW_TRIM < -1.0)
        RC_YAW_TRIM = -1.0;
}



void RCInitCypressCYRF6936(void)
{
    unsigned char Address;                     //Low Order Address Byte
    unsigned char Data=0;                       //Data Byte
    unsigned char Length; 
    unsigned char PageString[128];

#if 0
    RC_reset = 1;               // Reset the Cypress
    __delay32(MILLISEC);  // 1 milisec delay
    RC_reset = 0;               // Reset the Cypress
    __delay32(MILLISEC);  // 1 milisec delay   
#endif

    Address = 0x81;                              // TX_LENGTH_ADR
    Data = 0x10;
    ByteWriteSPI2(Address, Data);

    Address = 0x82;                              // TX_CTRL_ADR -> TX disabled
    Data = 0x00;
    ByteWriteSPI2(Address, Data);

    Address = 0x83;                              // TX_CFG_ADR
    Data = 0x2F;
    ByteWriteSPI2(Address, Data);

    Address = 0x86;                              // RX_CFG_ADR
    Data = 0x4A;
    ByteWriteSPI2(Address, Data);

    Address = 0x87;                              // RX_IRQ_STATUS_ADR
    Data = 0x00;
    ByteWriteSPI2(Address, Data);

    Address = 0x8B;                              // PWR_CTRL_ADR
    Data = 0x00;
    ByteWriteSPI2(Address, Data);

    Address = 0x8C;                              // XTAL_CTRL_ADR
    Data = 0x04;
    ByteWriteSPI2(Address, Data);

    Address = 0x8D;                              // IO_CFG_ADR
    Data = 0x41;
    ByteWriteSPI2(Address, Data);

    Address = 0x8E;                              // GPIO_CTRL_ADR
    Data = 0x00;
    ByteWriteSPI2(Address, Data);

    Address = 0x90;                              // FRAMING_CFG_ADR
    Data = 0xEE;
    ByteWriteSPI2(Address, Data);

    Address = 0x9B;                              // TX_OFFSET_LSB_ADR
    Data = 0x55;
    ByteWriteSPI2(Address, Data);

    Address = 0x9C;                              // TX_OFFSET_MSB_ADR
    Data = 0x05;
    ByteWriteSPI2(Address, Data);

    Address = 0x9D;                              // MODE_OVERRIDE_ADR
    Data = 0x18;
    ByteWriteSPI2(Address, Data);

    Address = 0xB2;                              // AUTO_CAL_TIME
    Data = 0x3C;
    ByteWriteSPI2(Address, Data);

    Address = 0xB5;                              // AUTO_CAL_OFFSET
    Data = 0x14;
    ByteWriteSPI2(Address, Data);

    Address = 0x9E;                              // RX_OVERRIDE_ADR
    Data = 0x90;
    ByteWriteSPI2(Address, Data);

    Address = 0x9F;                              // TX_OVERRIDE_ADR
    Data = 0x00;
    ByteWriteSPI2(Address, Data);

    Address = 0x8F;                              // XACT_CFG_ADR
    Data = 0x2C;
    ByteWriteSPI2(Address, Data);

    Address = 0xA8;                              // CLK_EN_ADR
    Data = 0x02;
    ByteWriteSPI2(Address, Data);

    Address = 0xA7;                              // CLK_OVERRIDE_ADR
    Data = 0x02;
    ByteWriteSPI2(Address, Data);

    Address = 0xA2;                              // SOP_CODE_ADR
    BurstWriteSPI2(Address, 0xDF, 0xB1, 0xC0, 0x49, 0x62, 0xDF, 0xC1, 0x49);

    Address = 0x8F;                              // XACT_CFG_ADR -> clr FRC END
    Data = 0x0C;
    ByteWriteSPI2(Address, Data);

    Address = 0x41;                              // TX_LENGTH_ADR
    Length = 16;
    ByteReadSPI2(Address, PageString, Length );                          //Low Density Byte Read

    Address = 0x5D;                              
    Length = 3;
    ByteReadSPI2(Address, PageString, Length );                          //Low Density Byte Read

    Address = 0x66;                           
    Length = 3;
    ByteReadSPI2(Address, PageString, Length );                          //Low Density Byte Read

    Address = 0x72;                           
    Length = 1;
    ByteReadSPI2(Address, PageString, Length );                          //Low Density Byte Read

    Address = 0x75;                           
    Length = 1;
    ByteReadSPI2(Address, PageString, Length );                          //Low Density Byte Read

    ERROR_Level1_COUNTER=0;
    ERROR_Level2_COUNTER=0;
}

static void select3BestChannels(unsigned char list[3], unsigned char rssi[], unsigned int numChannels)
{
    unsigned int i;
    unsigned char bestrssi[3] = {0,0,0};
    for (i=0;i<numChannels;i++){
        if (rssi[i] >= bestrssi[0]) {
            bestrssi[2] = bestrssi[1];
            bestrssi[1] = bestrssi[0];
            bestrssi[0] = rssi[i];
            list[2] = list[1];
            list[1] = list[0];
            list[0] = i;
        } else if (rssi[i] >= bestrssi[1]) {
            bestrssi[2] = bestrssi[1];
            bestrssi[1] = rssi[i];
            list[2] = list[1];
            list[1] = i;
        } else if (rssi[i] >= bestrssi[2]) {
            bestrssi[2] = rssi[i];
            list[2] = i;
        }
    }
}


// unsigned char Channel = 0;
// unsigned char NumberOfFoundChannels = 0;
RCSC_STATE rcSearchState = RCSCPrepare;
//void searchChannels(unsigned char *channelList)
RCSC_STATE RCSearchChannels()
{
    static unsigned char time=0;
    static unsigned char NumberOfFoundChannels;
    static unsigned char RSSIvalues[0x4F];
    static unsigned char NumberOfChannels=0x4F;   
    static RCSC_STATE state = RCSCPrepare; 
    static unsigned char Channel = 0;
    static unsigned char RX_IRQ_STATUS = 0;

    unsigned char Data = 0;
    unsigned char RX_COUNT = 0;
    unsigned char temp = 0;
    unsigned char trash[256];
    unsigned char needToWait = 0;

    while (!needToWait) {
        switch (state) {
            case RCSCPrepare:
                NumberOfFoundChannels=0;
                Channel = 0x00;
                ByteWriteSPI2(0x86, 0x0A);                  // disable LNA
                // only strong signals shall be detectable
                state = RCSCTune;
                break;
            case RCSCTune:
                ByteWriteSPI2(0x80, Channel);                  // tune to CHANNEL   
                //abort pending reception
                ByteWriteSPI2(0xA9, 0x20);                     // RX_ABORT_ADR -> enable
                ByteWriteSPI2(0x8F, 0x2C);                     // XACT_CFG_ADR -> force END STATE
                RX_COUNT = 0;
                ByteReadSPI2(0x09, &RX_COUNT, 1 );               // dummy read
                ByteReadSPI2(0x21, trash , RX_COUNT );
                ByteWriteSPI2(0x8F, 0x0C);                     // XACT_CFG_ADR -> release   
                ByteWriteSPI2(0xA9, 0x00);                     // RX_ABORT_ADR -> disable
                ByteReadSPI2(0x07, &temp, 1 );             // to clear RX_IRQ_STATUS_ADR

                ByteWriteSPI2(0x8D, 0x40);                     // set IRQ active HIGH
                ByteWriteSPI2(0x85, 0x80);                     // RX_CTRL_ADR -> RX_GO

                RX_IRQ_STATUS = 0;
                time=0;
                state = RCSCWaitResponse;
                break;
            case RCSCWaitResponse:
                RX_IRQ_STATUS = 0;
                ByteReadSPI2(0x07, &RX_IRQ_STATUS, 1 );                     // RX_IRQ_STATUS_ADR
                if ((RX_IRQ_STATUS&0b01000000)!=0) {
                    state = RCSCReceivedResponse;
                } else {
                    time ++;
                    if (time > MAX_SOP_WAIT_TIME) { // 15 ms
                        Channel++;
                        if (Channel >= NumberOfChannels) {
                            if (NumberOfFoundChannels > 2) {
                                state = RCSCCompleted; // Got enough channels
                            } else {
                                state = RCSCPrepare;
                            }
                        } else {
                            state = RCSCTune;
                        }
                    }
                }
                needToWait = 1;
                break;
            case RCSCReceivedResponse:
                Data=0;
                ByteReadSPI2(0x13, &Data, 1 );                     // RSSI_ADR
                RSSIvalues[Channel]=Data; // store RSSI-Value
                if(Data>=RSSI_THRESHOLD) {
                    NumberOfFoundChannels++;
                }
                Channel ++;
                if (Channel >= NumberOfChannels) {
                    Channel = 0;
                    if (NumberOfFoundChannels > 2) {
                        state = RCSCCompleted; // Got enough channels
                    } else {
                        state = RCSCPrepare; // Not enough channel found, restart scanning
                        needToWait = 1;
                    }
                } else {
                    state = RCSCTune;
                }
                break;
#if 1
            case RCSCCompleted:
                ByteWriteSPI2(0x86, 0x4A);                  // enable LNA
                select3BestChannels(channelList,RSSIvalues,NumberOfChannels);

                ByteWriteSPI2(0x80, channelList[0]);   // tune to first channel
                //abort pending reception:
                ByteWriteSPI2(0xA9, 0x20);                     // RX_ABORT_ADR -> enable
                ByteWriteSPI2(0x8F, 0x2C);                     // XACT_CFG_ADR -> force END STATE
                RX_COUNT = 0;
                ByteReadSPI2(0x09, &RX_COUNT, 1 );               // dummy read
                ByteReadSPI2(0x21, trash , RX_COUNT );
                ByteWriteSPI2(0x8F, 0x0C);                     // XACT_CFG_ADR -> release   
                ByteWriteSPI2(0xA9, 0x00);                     // RX_ABORT_ADR -> disable
                ByteReadSPI2(0x07, &temp, 1 );             // to clear RX_IRQ_STATUS_ADR

                ByteWriteSPI2(0x8D, 0x40);                     // set IRQ active HIGH
                ByteWriteSPI2(0x85, 0x80);                     // RX_CTRL_ADR -> RX_GO
                state = RCSCReady;
                needToWait = 1;
                break;
            case RCSCReady:
                // If we're called again, go back to prepare
                state = RCSCPrepare;
                break;
#endif
            default:
                needToWait = 1;
                break;
        }
        rcSearchState = state;
        // global_irq_status = RX_IRQ_STATUS;
    }
    return state;
}

//void convertRaw(unsigned char *rawData, unsigned int *Data)
void RCConvertRaw()
{
    //   unsigned int section;
    unsigned int offset;
    unsigned int position;
    unsigned int pos_sec;
    unsigned int off_sec;


    // right stick up/down
    pos_sec = (int)rcRawData[4];
    off_sec = pos_sec;
    position = (int)rcRawData[0];
    offset = (int)rcRawData[1];

    pos_sec = (pos_sec&0x00C0)<<2;
    off_sec = (off_sec&0x0030)<<4;

    rcData[0]=pos_sec+position;
    rcData[1]=off_sec + offset;

    // right stick left/right
    pos_sec = (int)rcRawData[4];
    off_sec = pos_sec;
    position = (int)rcRawData[2];
    offset = (int)rcRawData[3];

    pos_sec = (pos_sec&0x000C)<<6;
    off_sec = (off_sec&0x0003)<<8;

    rcData[2]=pos_sec+position;
    rcData[3]=off_sec + offset;

    // left stick up/down
    pos_sec = (int)rcRawData[9];
    off_sec = pos_sec;
    position = (int)rcRawData[5];
    offset = (int)rcRawData[6];

    pos_sec = (pos_sec&0x00C0)<<2;
    off_sec = (off_sec&0x0030)<<4;

    rcData[4]=pos_sec+position;
    rcData[5]=off_sec + offset;

    // left stick left/right
    pos_sec = (int)rcRawData[9];
    off_sec = pos_sec;
    position = (int)rcRawData[7];
    offset = (int)rcRawData[8];

    pos_sec = (pos_sec&0x000C)<<6;
    off_sec = (off_sec&0x0003)<<8;

    rcData[6]=pos_sec+position;
    rcData[7]=off_sec + offset;
}

void RCScale_0to1(void)
{

    unsigned char i;

    for(i=0; i<8; i++){
        rcValue[i]=(((float)rcData[i])-200)/600;
        if (rcValue[i]<0.0)   //limit to 0..1
            rcValue[i]=0.0;
        if (rcValue[i]>1.0)
            rcValue[i]=1.0;
    }

}

int RCInitReception(int blocking)
{
    InitTimer1();               // Initialize and start timer 1
    StopTimer1();               // Stop timer
    RC_reset = 1;               // Reset the Cypress
    DS_CS_2 = 1;               // Deselect the Cypress
    activeChannelIndex=0;         // 
    rcState = RC_LOWLEVEL_RESET1;
    StartTimer1();               // Start timer
    if (blocking) {
        while (rcState != RC_LOWLEVEL_RUNNING) {
            // Nothing to do but wait...
        }
    }
    return 0;
}

int RCIsReady() {
    return rcState == RC_LOWLEVEL_RUNNING;
}


//====================================
//===   This is the RC receive isr  ===
//====================================

void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    static unsigned int resetCounter = 0;
    unsigned char RX_IRQ_STATUS=0;
    unsigned char RX_COUNT=0;
    unsigned char trash[256];
    // unsigned char RXOW=0;
    // unsigned char RSSI=0;


    IEC0bits.T1IE = 0; //disable Timer Interrupt

    // Throttle and trim are not reset, because the consequence are too hard to
    // predict, the controller will have to take care of them
    switch (rcState) {
        case RC_LOWLEVEL_INIT:
            RCInitCypressCYRF6936();
            rcState = RC_LOWLEVEL_SEARCHING;
            break;
        case RC_LOWLEVEL_SEARCHING:
            if (RCSearchChannels() == RCSCReady) {
                rcState = RC_LOWLEVEL_RUNNING;
            }
            break;
        case RC_LOWLEVEL_RUNNING:
            RX_COUNT=0;
            RX_IRQ_STATUS=0;
            ERROR_Level1_COUNTER++;
            ERROR_Level2_COUNTER++;

            ByteReadSPI2(0x07, &RX_IRQ_STATUS, 1 );                     // RX_IRQ_STATUS_ADR

            if (RX_IRQ_STATUS & 0b10000101){                   // error occured
                ByteWriteSPI2(0xA9, 0x20);                     // RX_ABORT_ADR -> enable
                ByteWriteSPI2(0x8F, 0x2C);                     // XACT_CFG_ADR -> force END STATE
                ByteReadSPI2(0x09, &RX_COUNT, 1 );             // dummy read
                ByteReadSPI2(0x21, trash, RX_COUNT );
                ByteWriteSPI2(0x8F, 0x0C);                     // XACT_CFG_ADR -> release   
                ByteWriteSPI2(0xA9, 0x00);
                ByteWriteSPI2(0x87, 0x80);                     //write 1 to clear RXOW-bit
                ByteWriteSPI2(0x85, 0x82);                     // RX_CTRL_ADR -> RX_GO, IRQ enabled
            }

            if (RX_IRQ_STATUS==0x3A){               //no error occured
                ERROR_Level1_COUNTER=0;
                ERROR_Level2_COUNTER=0;
                ByteReadSPI2(0x09, &RX_COUNT, 1 );
                ByteReadSPI2(0x21, rcRawData, RX_COUNT );

                if ((rcRawData[12]%4)==3){
                    activeChannelIndex++;
                    if (activeChannelIndex == 3) {
                        activeChannelIndex = 0;
                    }
                    ByteWriteSPI2(0x80, channelList[activeChannelIndex]);                  // tune to channel
                }
                ByteWriteSPI2(0x85, 0x82);                     // RX_CTRL_ADR -> RX_GO, IRQ enabled   
                RCConvertRaw();         // RC channels conversion
                RCScaleChannels();      // RC channels conversion to 0-1
            }


            if (ERROR_Level1_COUNTER >= ERROR_COUNTER_THRESHOLD_Level1){   // this channel seems disturbed, switch to last channel    
                activeChannelIndex--;
                if (activeChannelIndex < 0)
                    activeChannelIndex = 2;
                ByteWriteSPI2(0x80, channelList[activeChannelIndex]);                  // tune to last channel
                ByteWriteSPI2(0x85, 0x82);                     // RX_CTRL_ADR -> RX_GO, IRQ enabled   
                ERROR_Level1_COUNTER=0;
            }

            if (ERROR_Level2_COUNTER >= ERROR_COUNTER_THRESHOLD_Level2){   // we lost connection to RC
                activeChannelIndex=0;         // 
                rcState = RC_LOWLEVEL_RESET1;
                ERROR_Level1_COUNTER=0;
                ERROR_Level2_COUNTER=0;
            }


            break;

        /*** Reset state, to avoid blocking wait ***/
        case RC_LOWLEVEL_RESET1:
            // In case we don't read data, at least have safe values
            RC_PITCH      = 0.0;
            RC_ROLL       = 0.0;
            RC_YAW        = 0.0;
            RC_reset = 1;
            resetCounter = 0;
            rcState = RC_LOWLEVEL_RESET1WAIT;
            break;
        case RC_LOWLEVEL_RESET1WAIT:
            resetCounter ++;
            if (resetCounter > RESET_WAIT_TIME) {
                rcState = RC_LOWLEVEL_RESET2;
            }
            break;
        case RC_LOWLEVEL_RESET2:
            RC_reset = 1;
            resetCounter = 0;
            rcState = RC_LOWLEVEL_RESET2WAIT;
            break;
        case RC_LOWLEVEL_RESET2WAIT:
            resetCounter ++;
            if (resetCounter > RESET_WAIT_TIME) {
                rcState = RC_LOWLEVEL_INIT;
            }
            break;
        default:
            break;
    }
    // global_irq_status = RX_IRQ_STATUS;

    IFS0bits.T1IF = 0;      // Reset Timer1 interrupt flag
    IEC0bits.T1IE = 1; //Enable Interrupt
}


void RCSetType(RC_TYPE type) {
    if (type < RC_NUMTYPES) {
        rcType = type;
    }
}
 
RC_TYPE RCGetType() {
    return rcType;
}

RC_LOWLEVEL_STATE RCGetState() {
    return rcState;
}


