//送信用
#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515/mcp2515.h"

MCP2515 can0;
struct can_frame rx;

int main() {
    stdio_init_all();

    //Initialize interface
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    can0.setNormalMode();
struct can_frame frame;
frame.can_id = 0x000;
frame.can_dlc = 4;
frame.data[0] = 0xFF;
frame.data[1] = 0xFF;
frame.data[2] = 0xFF;
frame.data[3] = 0xFF;

/* send out the message to the bus and
tell other devices this is a standard frame from 0x00. */
    //Listen loop
    while(true) {
        can0.sendMessage(&frame);
        printf("send\n");
    }

    return 0;
}
