#include <iostream>
#include <stdint.h>
#include <string.h>

#include "uart_struct.h"
#include "uart_driver.h"

// execute: ./uart_serial_example /dev/ttyUSB0 115200
int main(int argc, char** argv) {
    // defaults: pointer is not const, the thing that it is pointing to is const
    const char *uart_port = "/dev/ttyUSB0";
    int uart_speed = 115200;
    // if non default
    if (argc > 2) {
        uart_port = argv[1];
        uart_speed = std::stoi(argv[2]);
    } else {
        printf("[err] Enter port name and baud! \n");
        printf("Opening port: %s at baud: %d\n", uart_port, uart_speed);
    }
    
    SerialPort *obj = new SerialPort(uart_port, uart_speed);
    divergence_packet_t uart_rx_buffer;

    while(1) {

        if (obj->valid_uart_message_received) {
            // for (int i=0; i<FRAMELEN; i++) {
            //     printf("0x%02x,", obj->uart_msg_bytes[i]);
            // }
            // printf("\n");
            // memcpy(&uart_rx_buffer, &(obj->uart_msg_bytes[1]), FRAMELEN);
            // std::cout << "divergence: " << uart_rx_buffer.data.divergence << std::endl;
            // std::cout << "divergence_dot: " << uart_rx_buffer.data.divergence_dot << std::endl;
            // std::cout << "-----------" << std::endl;
        } 
        // TODO: is the else needed? spsc or deep copy decide,
        // exiting ctrl+c difficult while serial is high speed
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return 0;
}

// sudo gtkterm -b 8 -t 1 -s 115200 -p /dev/ttyTHS2
