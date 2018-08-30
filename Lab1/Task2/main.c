/**
 * Task 2 for ECE4550 Fall 2018
 *
 * Author: Marion Anderson
 * Date: 2018-08-27
 * File: main.c
 */
#include "F2837xD_device.h"
#include "math.h"
#ifndef PI
#define PI 3.14159
#endif

unsigned long slow = 0, fast = 0;  // counters to view program execution
float x[5000], y[5000];            // math coordinate arrays


int main(void)
{
    // Turn Watchdog off & back on, per lab instructions
    // 'off' bits: 00000000 | 0 | 1 | 101 | 000
    // 'on' bits:  00000000 | 0 | 0 | 101 | 000
    //
    // Bits 6 thru 3 are the important ones.
    //     bit 6 turns watchdog on (0) and off (1)
    //     bits 5-3 are an error check. Must be '101' every time WDCR is written. Else reset
    WdRegs.WDCR.all = 0x68;
    WdRegs.WDCR.all = 0x28;

    while (1) {
        // Compute x & y arrays
        // x in range [0, 1]
        // use fast as index to save memory
        if (fast < 5000) {                  // prevent out-of-bounds indexing
            x[fast] = (float) fast / 5000;  // down convert from [0, 5000] to [0, 1]
            y[fast] = cos(2 * PI * x[fast]);
        }

        // Counter incrementation
        // not taking advantage of operator precedence for readability
        fast++;
        if (fast % 100000 == 0) {
            slow++;
        }

        // Reset watchdog constantly
        WdRegs.WDKEY.all = 0x55;
        WdRegs.WDKEY.all = 0xAA;
    }
}
