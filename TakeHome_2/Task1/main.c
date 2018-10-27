/**
 * Take-Home Lab2 for ECE4550 Fall 2018
 *
 * BiDirectional Serial Communication
 *
 * Author: Marion Anderson
 * Date: 2018-11-02
 */

#include "F2802x_Device.h"

char cmd_char;
interrupt void serialISR(void);


int main(void)
{
    // ==============================Begin Setup==============================
    DINT; EALLOW; SysCtrlRegs.SCSR = 0b10; SysCtrlRegs.WDCR = 0x68;

    // System clock setup (10MHz ref -> 30MHz for easy serial clock setting)
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;            // PLL bypass
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;           // ignore failures for now
    SysCtrlRegs.PLLCR.bit.DIV = 3;                // multiplier
    while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1);  // wait for clock to latch
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;           // detect failures
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;            // divider

    // LED setup
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;
    GpioDataRegs.GPASET.bit.GPIO0 = 1;  // force all leds off
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioDataRegs.GPASET.bit.GPIO3 = 1;

    // Serial setup
    SysCtrlRegs.LOSPCP.bit.LSPCLK = 1;      // 15MHz sciclk (1/2 sysclk)
    SciaRegs.SCIHBAUD = 0b11;               // BRR=780 for 2400baud
    SciaRegs.SCILBAUD = 0b00001100;
    SciaRegs.SCICCR.bit.SCICHAR = 0b111;    // 8-bit serial packet (ASCII)
    SciaRegs.SCICTL1.bit.SWRESET = 1;       // re-enable serial
    SciaRegs.SCICTL1.bit.TXENA = 1;         // transmit enable
    SciaRegs.SCICTL1.bit.RXENA = 1;         // recieve enable
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;    // enable rx interrupt
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;  // enable serial clock

    // Interrupt setup
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.SCIRXINTA = &serialISR;
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;  // serial recieving interrupt
    IER = 1;

    SysCtrlRegs.WDCR = 0x28; EDIS; EINT;
    // ===============================End Setup================================
    while (1) {
        SysCtrlRegs.WDKEY = 0x55;
        SysCtrlRegs.WDKEY = 0xAA;
    }
}



interrupt void serialISR(void)
{
    // Read Serial Data
    cmd_char = SciaRegs.SCIRXBUF.bit.RXDT;  // store character
    SciaRegs.SCITXBUF = cmd_char;  // retransmit character

    // Change Active LED
    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;  // LEDs off 1st for code brevity
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    switch (cmd_char) {                   // re-activate selected LED
    case '0':
        GpioDataRegs.GPASET.bit.GPIO0 = 1;
        break;
    case '1':
        GpioDataRegs.GPASET.bit.GPIO1 = 1;
        break;
    case '2':
        GpioDataRegs.GPASET.bit.GPIO2 = 1;
        break;
    case '3':
        GpioDataRegs.GPASET.bit.GPIO3 = 1;
        break;
    }
}
