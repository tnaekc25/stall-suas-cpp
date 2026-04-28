#include "common.h"

FrameQueue::FrameQueue(int _size) : size(_size), head(0), tail(0) {
	holder = new MatchedImage*[_size]();
}

FrameQueue::~FrameQueue() {
	delete [] holder;
}

void FrameQueue::push(MatchedImage *image) {
	std::lock_guard<std::mutex> lock(mtx);

	MatchedImage *&ref = holder[head];
	
	/* Discard old data */
	if (ref) {
		tail = (tail < size-1) ? (tail + 1) : 0;
		delete ref; 
	}

	holder[head] = image;
	head = (head < size-1) ? (head + 1) : 0;
}

MatchedImage *FrameQueue::pop() {
	std::lock_guard<std::mutex> lock(mtx);

	MatchedImage *val = holder[tail];
	if (!val) return 0;

	holder[tail] = 0;
	tail = (tail < size-1) ? (tail + 1) : 0;

	return val;
}

void FrameQueue::printHolder() {
	for (int i = 0;i < size;i++)
		std::cout << (int64_t)holder[i] << ", ";
	std::cout << std::endl;
}