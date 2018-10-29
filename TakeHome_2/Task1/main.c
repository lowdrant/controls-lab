/**
 * Take-Home Lab2 for ECE4550 Fall 2018
 *
 * BiDirectional Serial Communication
 *
 * Author: Marion Anderson
 * Date: 2018-11-02
 */

#include "F2802x_Device.h"

char cmd_char = '\0';
Uint32 isr_count = 0;  // for debugging
interrupt void serialISR(void);


int main(void)
{
    // ==============================Begin Setup==============================
    DINT; EALLOW; SysCtrlRegs.SCSR = 0b10; SysCtrlRegs.WDCR = 0x68;

    // Clock setup
    // System (10MHz ref -> 50MHz for accurate 9600baud)
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;            // PLL bypass
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;           // ignore failures for now
    SysCtrlRegs.PLLCR.bit.DIV = 5;                // multiplier (5)
    while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1);  // wait for clock to latch
    SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;           // detect failures
    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;            // divider (1)
    // Peripheral/SCI
    SysCtrlRegs.LOSPCP.bit.LSPCLK = 0;            // 50MHz (1x sysclk)
    asm(" NOP"); asm(" NOP");
    SysCtrlRegs.PCLKCR0.bit.SCIAENCLK = 1;        // enable serial clock
    asm(" NOP"); asm(" NOP");

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
    GpioDataRegs.GPASET.bit.GPIO0 = 1;  // Canary: force all leds off
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioDataRegs.GPASET.bit.GPIO3 = 1;

    // Serial GPIO (just mux to serial)
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;

    // Serial configuration (9600 baud, 8-bit packet, 1 stop bit, no parity)
    SciaRegs.SCIHBAUD = 0x2;                // 9600 baud (BRR=650)
    SciaRegs.SCILBAUD = 0x8a;
    SciaRegs.SCICCR.bit.SCICHAR = 0b111;    // 8-bit packet (ASCII)
    SciaRegs.SCICCR.bit.STOPBITS = 0;       // 1 stop bit
    SciaRegs.SCICCR.bit.PARITYENA = 0;      // disable parity
    SciaRegs.SCICTL1.bit.SWRESET = 1;       // re-enable SCI
    SciaRegs.SCICTL1.bit.TXENA = 1;         // transmit enable
    SciaRegs.SCICTL1.bit.RXENA = 1;         // recieve enable

    // Interrupt setup
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieVectTable.SCIRXINTA = &serialISR;  // serial receive ISR
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;    // 9.1 interrupt (serial)
    IER = 0x100;                          // activate group 9 path
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;  // enable rx interrupt

    SysCtrlRegs.WDCR = 0x28; EDIS; EINT;
    // ===============================End Setup================================
    
    // Feed watchdog
    while (1) {
        SysCtrlRegs.WDKEY = 0x55;
        SysCtrlRegs.WDKEY = 0xAA;
    }
}



interrupt void serialISR(void)
{
    isr_count++;  // for debugging

    // Read Serial Data
    cmd_char = SciaRegs.SCIRXBUF.bit.RXDT;  // store character
    SciaRegs.SCITXBUF = cmd_char;  // retransmit character

    // Change Active LED
    GpioDataRegs.GPASET.bit.GPIO0 = 1;  // LEDs off 1st for code brevity
    GpioDataRegs.GPASET.bit.GPIO1 = 1;
    GpioDataRegs.GPASET.bit.GPIO2 = 1;
    GpioDataRegs.GPASET.bit.GPIO3 = 1;
    switch (cmd_char) {                 // re-activate selected LED
    case '0':
        GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
        break;
    case '1':
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
        break;
    case '2':
        GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
        break;
    case '3':
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
        break;
    }

    PieCtrlRegs.PIEACK.all = M_INT9;
}
