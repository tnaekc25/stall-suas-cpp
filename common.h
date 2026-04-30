#ifndef SUAS_COMMON_H_
#define SUAS_COMMON_H_


#include "image.h"

#include <iostream>
#include <cstring>
#include <algorithm>
#include <mutex>
#include <cstdint>
#include <vector>
#include <chrono>
#include <utility>
#include <cmath>
#include <thread>
#include <atomic>
#include <condition_variable>



struct Point {double x, y;};
struct Point3 {double x, y, z;};
struct Quad {Point p1, p2, p3, p4;};

#define EARTH_RADIUS_M 6371000.0
#define PI_C 3.1415926


/* 3D rotation of a point */
#define ROTATE_POINT(X, Y) { \
	cosZ*cosY*X + (cosZ*sinY*sinX - sinZ*cosX)*Y + (cosZ*sinY*cosX + sinZ*sinX), \
	sinZ*cosY*X + (sinZ*sinY*sinX + cosZ*cosX)*Y + (sinZ*sinY*cosX - cosZ*sinX), \
	-sinY*X + cosY*sinX*Y + cosY*cosX \
}


/* The shared flight mode holder, initially 0 (A) */
extern std::atomic<int> flight_mode;

/* Time offset (pixhawk - jetson) to sync frames */
extern std::atomic<int64_t> sync_time_offset;

/* CV for triggering start of threads after first HB */


extern struct HBLatch {

	std::mutex mtx;
	std::condition_variable cv;
	bool signaled = false;

	void wait() {
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [this] {return signaled;});
	}

	void notify() {
		{
			std::lock_guard<std::mutex> lock(mtx);
			signaled = true;
		}

		cv.notify_all();
	}
} global_hb_latch;


struct UavData {
	uint32_t time;

	double lat, lon, alt; 

	float vx, vy, vz;
	float roll, pitch, yaw;
};

struct MatchedImage {
	cv::Mat frame;
	UavData data;

	MatchedImage(cv::Mat &_frame, UavData _data) : frame(_frame), data(_data) {}
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

		uint32_t getMax() {
			return max_time;
		}
};


class FrameQueue {
	private:
		MatchedImage **holder;
		int size;
		int head, tail;

		std::mutex mtx;

	public:
		FrameQueue(int size);
		~FrameQueue();

		void push(MatchedImage *image);
		MatchedImage *pop();


		void printHolder();

};


#endif