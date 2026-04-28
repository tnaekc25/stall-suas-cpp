#include "detect_thread.h"

int detectAndCommand(FrameQueue *frame_queue, WaypointMap *waypoint_map, SerialHandler *serial) {
	while (true) {
		MatchedImage *image = frame_queue -> pop();
		if (!image) {
			continue;
		} 

		cv::Mat &frame = image -> frame;
		UavData &uav_data = image -> data;

		/* TODO apply detection on frame */

		/* Code for mode B */
		if (flight_mode.load()) {
		}

		/* Code for mode A */
		else {
			/* TODO iterate all the boxes */
			for (int i = 0;i < 5;i++) {
				/* check if the detection worths adding */
				if (conf > WAYPOINT_CONF_TRESH) {
					waypoint_map -> addWaypoint(uav_data, px, py);
				}
			}
		}

		/* !!!VERY IMPORTANT!!! */
		delete image;
	}

	return 0;
}