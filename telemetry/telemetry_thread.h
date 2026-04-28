#ifndef SUAS_TELEM_H_
#define SUAS_TELEM_H_


#include "../common.h"

#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include "mavlink/common/mavlink.h"
#include "mavlink/standard/mavlink.h"

#define MESSAGE_SIZE 2048

#define HB_TIMEOUT 2000 /* In ms */
#define TELEM_RECV_PERIOD 1 /* In ms */
#define TELEM_SEND_PERIOD 1 /* In ms */
#define TELEM_SYNC_PERIOD 50 /* In ms */

#define SEND_BAUD B57600
#define RECV_BAUD B57600

#define EXP_ALPHA 0.1


class SerialHandler {
	private:
		size_t bfsize;
		const char *addr_str;
		int sbaud;
		int rbaud;

		int fdes;
		char *buffer;

		std::mutex read_mtx;
		std::mutex write_mtx;

	public:
		SerialHandler(const char *str, int sbaud, int rbaud, size_t bfsize);
		~SerialHandler();

		bool serialConnect();
		bool checkConnection();
		
		ssize_t readData();
		ssize_t writeData(uint8_t *data, int len);
		const char *getBuffer();
};


int telemRead(TelemBuffer *telem_buffer, SerialHandler *serial);
int telemWrite(SerialHandler *serial);


#endif