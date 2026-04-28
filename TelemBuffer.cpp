#include "common.h"

/*
#include <iostream>
#include <cstring>
#include <algorithm>
#include <mutex>
#include <cstdint>
#include <utility>

struct UavData {
	uint32_t time;

	double lat, lon, alt; 

	float vx, vy, vz;
	float roll, pitch, yaw;
};

class TelemBuffer {
	private:
		bool filled, available;
		int size, head;
		uint32_t div_time, max_time, min_time;

		UavData *arr;
		std::mutex mtx;

	public:
		TelemBuffer(int _size);
		~TelemBuffer();
		
		void putData(UavData data);
		UavData getData(uint32_t image_time);

		void printBuffer() {
			for (int i = 0;i < size;i++)
				std::cout << arr[i].time << ", ";
		}
};

*/



#define INTERP(v1, v2) (v1 + (v2 - v1) / 2.0f)


TelemBuffer::TelemBuffer(int _size) : filled(false), size(_size), head(0), max_time(-1), min_time(0) {
	arr = new UavData[size]();
}

TelemBuffer::~TelemBuffer() {
	delete [] arr;
}

void TelemBuffer::putData(UavData data) {
	std::lock_guard<std::mutex> lock(mtx);

	//std::cout << "<ws";
	arr[head] = data;
	
	max_time = data.time;

	if (!head) div_time = data.time;
	head = (head + 1 < size) ? (head + 1) : 0;
	
	if (!filled && !head) min_time = data.time;
	else if (filled) min_time = arr[head].time;
	
	if (!filled && head == size-1) filled = true;

	//std::cout << " we>" << std::endl;
}


UavData TelemBuffer::getData(uint32_t image_time) {
	std::lock_guard<std::mutex> lock(mtx);
	//std::cout << "<rs";

	int s, e, mid;
	
	if (image_time > max_time) return {UINT32_MAX}; /* wait value if not recvd yet*/
	if (image_time < min_time) return {UINT32_MAX-1}; /* discard value if smaller */

	if (!head || image_time < div_time) {s = head; e = size - 1;}
	else {s = 0; e = head - 1;}		

	while(s <= e) {
		mid = (s + e) / 2;

		UavData *p1 = arr + mid;
		UavData *p2 = (mid + 1 < size) ? (arr + mid + 1) : arr;

		if (image_time < p1 -> time) e = mid - 1;
		else if (image_time > p2 -> time) s = mid + 1;

		else if (image_time > p1 -> time && image_time < p2 -> time) {
			return {
				image_time,

				INTERP(p1 -> lat, p2 -> lat),
				INTERP(p1 -> lon, p2 -> lon),
				INTERP(p1 -> alt, p2 -> alt),

				INTERP(p1 -> vx, p2 -> vx),
				INTERP(p1 -> vy, p2 -> vy),
				INTERP(p1 -> vz, p2 -> vz),

				INTERP(p1 -> roll, p2 -> roll),
				INTERP(p1 -> pitch, p2 -> pitch),
				INTERP(p1 -> yaw, p2 -> yaw)
			};
		}

		else if (image_time == p1 -> time) return *p1;

		else return *p2;
	}

	//std::cout << " re>" << std::endl;
	return {UINT32_MAX};
}

/*
int main() {

	TelemBuffer buf(20);

	for (int i = 0;i < 63;i+=3)
		buf.putData({i});
	buf.printBuffer();

	for (int i = 0;i < 60;i++)
		std::cout << "<" << i << "-" <<  buf.getData(i).time << ">";

	return 0;
}
*/