// maybe use deque only when multiple consumers
// #include <deque>

/*  decide what is more important: either getting current deep copy of data or 
   	missed push to the queue, but if your own thread runs slow, it could overflow the queue, 
	leading to unsuccessful spsc push... */

#include <iostream>
#include <stdint.h>
#include <thread>
#include <chrono>

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/queue.hpp>

struct uart_buffer {
	float val1;
	uint8_t val2;
};

// global queue
boost::lockfree::spsc_queue<uart_buffer, boost::lockfree::capacity<8192>> _buffer_queue;

void producer() {
	uart_buffer buffer_data;
	float val1_ = 0.1;
	while(1) {
		val1_ = val1_ + 0.1;
		buffer_data.val1 = val1_;
		
		if (!_buffer_queue.push(buffer_data)) {
			std::cout << "[err] producer failed to push buffer data" << std::endl;
			_buffer_queue.reset();
		}

		std::cout << "producer: write available: " << _buffer_queue.write_available() << std::endl;
		
		// also print the queue?
		std::this_thread::sleep_for(std::chrono::milliseconds(4));
	}
}

void consumer() {
	uart_buffer buffer_data;
	while(1) {
		if (!_buffer_queue.pop(buffer_data)) {
			std::cout << "[err] consumer failed to get buffer data" << std::endl;
		}

		std::cout << "consumer: read available: " << _buffer_queue.read_available() << std::endl;

		std::cout << "val1: " << buffer_data.val1 << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
}



int main(int argv, char** argc) {
	// std::deque<uint8_t> buffer;
	// for (int i=65; i<(65+26); i++) {
	// 	buffer.push_back(i);
	// }

	// uint8_t val = 0;

	// for (int j=0; j<26; j++) {
	// 	val = buffer.front();
	// 	buffer.pop_front();
	// 	std::cout<< "val: " << val << std::endl;
	// }
	
	std::thread thread1(producer);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	std::thread thread2(consumer);

	std::cout << "threads created!" << std::endl;

	thread1.join();
	thread2.join();

	return 0;
}
