/*
Threaded UART receiving buffer for any linux computer
Deep copy from:
        MIT License Copyright (c) 2018 Matthias Faessler
	(Robotics and Perception Group, University of Zurich, Switzerland)
*/

#include <thread>
#include <atomic>
#include "uart_struct.h"

class SerialPort {
        public:
                SerialPort(const std::string& port, int uart_speed);
                ~SerialPort();
                void serialPortReceiveThread();
                void serialPortTransmitThread();
                uint8_t uart_msg_bytes[sizeof(divergence_packet_t) + 3];
                bool valid_uart_message_received = false;

        private:
                bool createRXSharedMemory();
                bool createTXSharedMemory();
                bool connectSerialPort(const std::string& port, int uart_speed);
                bool configSerialPort(int uart_speed) const;
                bool disconnectSerialPort();
                bool startReceiverThread();
                bool startTransmitThread();
                // serial port int handle
                int serial_port_fd_;
                std::thread receiver_thread_;
                std::thread transmit_thread_;
                std::atomic_bool receiver_thread_should_exit_;

                static constexpr uint8_t HeaderByte_ = 0x24;
                static constexpr uint8_t FooterByte_ = 0x2A;
                static constexpr int kPollTimeoutMilliSeconds_ = 500;
                static constexpr int uartFrameLength_ = sizeof(divergence_packet_t);

};
