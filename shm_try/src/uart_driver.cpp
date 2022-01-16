#include "uart_driver.h"
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <deque>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <uart_struct.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
using namespace boost::interprocess;

#define LOIHI_RX_MEMNAME "loihi_rx"
#define LOIHI_TX_MEMNAME "loihi_tx"
loihi_rx_shm *loihi_rx_shm_data;
loihi_tx_shm *loihi_tx_shm_data;

// #define DBG

// rx buffer deep copied between receive thread and uart_driver
divergence_packet_t uart_packet;

SerialPort::SerialPort(const std::string &port, int uart_speed)
	: receiver_thread_(),
	  receiver_thread_should_exit_(false),
	  serial_port_fd_(-1) {

	valid_uart_message_received = false;
	if (!createRXSharedMemory()) {
		std::cout << "[Serial] Error. Can't create RX shared memory" << std::endl;
	}

	if (!createTXSharedMemory()) {
		std::cout << "[Serial] Error. Can't create TX shared memory" << std::endl;
	}

	if (!connectSerialPort(port, uart_speed)) {
		std::cout << "[Serial] Error. Can't connect to port" << std::endl;
	}

	if (!startReceiverThread()) {
		std::cout << "[Serial] Error. Can't start receiver thread" << std::endl;
	}
	
	if (!startTransmitThread()) {
		std::cout << "[Serial] Error. Can't start transmit thread" << std::endl;
	}
}

SerialPort::~SerialPort() { disconnectSerialPort(); }

bool SerialPort::createRXSharedMemory() {
	try {

		// Erase previous shared memory
		shared_memory_object::remove(LOIHI_RX_MEMNAME);

		// Create a shared memory object.
		boost::interprocess::shared_memory_object shm(create_only, LOIHI_RX_MEMNAME, read_write);

		// Set size
		shm.truncate(sizeof(loihi_rx_shm));

		// Map the whole shared memory in this process
		mapped_region region(shm, read_write);

		// Get the address of the mapped region
		void *addr = region.get_address();

		// Construct the shared structure in memory
		loihi_rx_shm_data = new (addr) loihi_rx_shm;

		// Initialize everything to zero
		loihi_rx_shm_data->cnt = 0;
		loihi_rx_shm_data->divergence = 0.f;
		loihi_rx_shm_data->divergence_dot = 0.f;
		loihi_rx_shm_data->flag = false;

		std::cout << "[SHM] Loihi RX shared memory created" << std::endl;

    } catch(interprocess_exception &ex){
      std::cout << "[SHM] RX: Boost interprocess exception: " << ex.what() << std::endl;
      return false;
    }
	
	return true;
}

bool SerialPort::createTXSharedMemory() {
	try {

		// Erase previous shared memory
		shared_memory_object::remove(LOIHI_TX_MEMNAME);

		// Create a shared memory object.
		boost::interprocess::shared_memory_object shm(create_only, LOIHI_TX_MEMNAME, read_write);

		// Set size
		shm.truncate(sizeof(loihi_tx_shm));

		// Map the whole shared memory in this process
		mapped_region region(shm, read_write);

		// Get the address of the mapped region
		void *addr = region.get_address();

		// Construct the shared structure in memory
		loihi_tx_shm_data = new (addr) loihi_tx_shm;

		// Initialize everything to zero
		loihi_tx_shm_data->cnt = 0;
		loihi_tx_shm_data->thrust = 0.f;
		loihi_tx_shm_data->flag = false;

		std::cout << "[SHM] Loihi TX shared memory created" << std::endl;

	} catch(interprocess_exception &ex){
      std::cout << "[SHM] TX: Boost interprocess exception: " << ex.what() << std::endl;
      return false;
    }
	
	return true;
}

bool SerialPort::startReceiverThread() {
	// Start watchdog thread
	try {
		receiver_thread_ = std::thread(&SerialPort::serialPortReceiveThread, this);
	} catch (...) {
		return false;
	}
	return true;
}

bool SerialPort::startTransmitThread() {
	// Start watchdog thread
	try {
		transmit_thread_ = std::thread(&SerialPort::serialPortTransmitThread, this);
	} catch (...) {
		return false;
	}
	return true;
}

void SerialPort::serialPortReceiveThread() {

	struct pollfd fds[1];
	fds[0].fd = serial_port_fd_;
	fds[0].events = POLLIN;
	std::cout << "[Serial] ReceiveThread spawned" << std::endl;

	// while atomic lock of thread
	while (!receiver_thread_should_exit_) {
		if (valid_uart_message_received) {
			boost::interprocess::shared_memory_object shm_rx(open_only, LOIHI_RX_MEMNAME, read_write);
			mapped_region region_rx(shm_rx, read_write);
			void *addr_rx = region_rx.get_address();
			loihi_rx_shm_data = static_cast<loihi_rx_shm*>(addr_rx);
			if (!loihi_rx_shm_data->flag) {
				loihi_rx_shm_data->cnt = uart_packet.data.cnt;
				loihi_rx_shm_data->divergence = uart_packet.data.divergence;
				loihi_rx_shm_data->divergence_dot = uart_packet.data.divergence_dot;
				loihi_rx_shm_data->flag = true;
			}
		}
	}
}

void SerialPort::serialPortTransmitThread() {
	std::cout << "[Serial] TransmitThread spawned" << std::endl;

	// Shared memory opening
	boost::interprocess::shared_memory_object shm_tx(open_only, LOIHI_TX_MEMNAME, read_write);
    mapped_region region_tx(shm_tx, read_write);
    void *addr_tx = region_tx.get_address();
    loihi_tx_shm_data = static_cast<loihi_tx_shm*>(addr_tx);

	// Initialize buffer with start and end bytes 
	uint8_t buffer[3 + sizeof(thrust_packet_t)] = {0};
	buffer[0] = {0x24};
	buffer[3 + sizeof(thrust_packet_t) - 1] = {0x2a};

	// While loop that transmits information to bebop
	while (1) {

		// Wait for valid Loihi thrust measurement
		if (!loihi_tx_shm_data->flag) {
			continue;
		}

		// Packet initialization
		thrust_packet_t uart_packet_tx = {0};
		uart_packet_tx.info.packet_type = DATA_FRAME;
		uart_packet_tx.info.packet_length = sizeof(thrust_packet_t);

		// Get data from shared memory
		uart_packet_tx.data.cnt = loihi_tx_shm_data->cnt;
		uart_packet_tx.data.thrust = loihi_tx_shm_data->thrust;
		loihi_tx_shm_data->flag = false;
		#ifdef DBG
		printf("[TX] cnt: %i, thrust: %f\n", uart_packet_tx.data.cnt, uart_packet_tx.data.thrust);
		#endif

		// Checksum
		uint8_t checksum = 0;
		uint8_t *p = (uint8_t *) &uart_packet_tx;
		for (int i = 0; i < sizeof(thrust_packet_t); i++) {
    		checksum += p[i];
		}

		// Copy message and checksum to buffer
		memcpy(&buffer[1], &uart_packet_tx, sizeof(thrust_packet_t));
		memcpy(&buffer[3 + sizeof(thrust_packet_t) - 2], &checksum, sizeof(uint8_t));

		const int written = write(serial_port_fd_, buffer, uartFrameLength_ + 3);
  		if (written != uartFrameLength_ + 3) {
			std::cout << "[Serial] written wrong bytes" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

