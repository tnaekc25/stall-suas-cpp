#include "telemetry_thread.h"

SerialHandler::SerialHandler(const char *str, int _sbaud, int _rbaud, size_t _bfsize) : 
	bfsize(_bfsize), addr_str(str), sbaud(_sbaud), rbaud(_rbaud), fdes(-1) {
		
		/* Create the buffer */
		buffer = new char[bfsize];

		/* Trigger initial connection */
		serialConnect();
}

bool SerialHandler::serialConnect() { 
	
	/* Acquire mutexes so no read/write at reconnect */
	std::lock_guard<std::mutex> read_lock(read_mtx);
	std::lock_guard<std::mutex> write_lock(write_mtx);

	/* Close if already open */
	if (fdes >= 0) close(fdes);
	fdes = -1;

	fdes = open(addr_str, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fdes < 0) {
		std::cout << "SERIAL INIT ERROR" << std::endl;
	}

	struct termios tty;
	tcgetattr(fdes, &tty);

	cfsetospeed(&tty, sbaud);
	cfsetispeed(&tty, rbaud);
	tty.c_cflag = CS8 | CLOCAL | CREAD;
	tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
	tty.c_lflag = 0;
	tty.c_iflag = 0;
	tty.c_oflag = 0;

	tcflush(fdes, TCOFLUSH);

	return (tcsetattr(fdes, TCSANOW, &tty) == 0);
}	

bool SerialHandler::checkConnection() {
    return true;
}

SerialHandler::~SerialHandler() {
	delete [] buffer;
	close(fdes);
}

ssize_t SerialHandler::readData(int64_t *time = 0) {

	/* Acquire read mutex */
	std::lock_guard<std::mutex> lock(read_mtx);

	/* Invalid fdes */
	if (fdes < -1) return -1;
	
	ssize_t len = -1;
	if ((len = read(fdes, buffer, sizeof(bfsize)))) {
		if (time) (*time) = get_kernel_ns(); /* Set precise(ish) recv time */
	}

	return len;
}

ssize_t SerialHandler::writeData(uint8_t *data, int len) {

	/* Acquire write mutex */
	std::lock_guard<std::mutex> lock(write_mtx);

	/* Invalid fdes */
	if (fdes < -1) return -1;
	return write(fdes, data, len);
}

const char *SerialHandler::getBuffer() {
	return buffer;
}