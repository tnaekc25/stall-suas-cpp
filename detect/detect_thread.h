#ifndef SUAS_DETECT_H_
#define SUAS_DETECT_H_

#include "../common.h"
#include "../telemetry/telemetry_thread.h"

#define CONF_TRESH 0.8
#define WAYPOINT_CONF_TRESH 0.8


class WaypointMap {
	private:
		struct waypoint {double lat, lon;};

		std::vector<waypoint> lst;
		int *matrix;

		double clat, clon; /* center lat lon in rads */
		double szlat, szlon; /* In meters */

		double wrat, hrat; /* grid per meters */

		int width, height; /* n x m */

		int center_width;
		int center_height;

		int size;

	public:
		WaypointMap(int _width, int _height, double _clat, double _clon, double _szlat, double _szlon);
		~WaypointMap();

		void addWaypoint(UavData &uav_data, int px, int py);

		void clearMatrix();
		void printMatrix();
};


int detectAndCommand(FrameQueue *frame_queue, WaypointMap *waypoint_map, SerialHandler *serial);


#endif