#include "detect_thread.h"


WaypointMap::WaypointMap(int _width, int _height, double _clat, double _clon, double _szlat, double _szlon)
 : clat(_clat), clon(_clon), szlat(_szlat), szlon(_szlon), width(_width), height(_height) {
	size = _width*_height;
	matrix = new int[size]();

	wrat = _width / (double)_szlat;
	hrat = _height / (double)_szlon;

	center_width = _width / 2;
	center_height = _height / 2;
}

WaypointMap::~WaypointMap() {
	delete [] matrix;
}

void WaypointMap::addWaypoint(UavData &uav_data, int px, int py) {
	/* Get current lat-lon in radians */
	double radLon = uav_data.lon * PI_C / 180;
	double radLat = uav_data.lat * PI_C / 180;


	/* Calculate trig funcs to be used in rotation */
	double cosX = std::cos(uav_data.pitch + CAM_PITCH), sinX = std::sin(uav_data.pitch + CAM_PITCH);
	double cosY = std::cos(uav_data.roll), sinY = std::sin(uav_data.roll);
	double cosZ = std::cos(uav_data.yaw), sinZ = std::sin(uav_data.yaw);

	double cam_x = (px - center_width) / (double)CAM_FX;
	double cam_y = (py - center_width) / (double)CAM_FY;

	/* Apply rotation to get real pos */
	Point3 pr = ROTATE_POINT(cam_x, cam_y);
	double coef = uav_data.alt / pr.z;

	/* Convert position to gps pos and add as waypoints */ 
					
	/* TEST THIS PART */
	double diff_north = EARTH_RADIUS_M * (radLat - clon);
	double diff_east = EARTH_RADIUS_M * std::cos((radLat + clat) * 0.5) * (radLon - clon);

	double dst_north = diff_north + pr.y * coef;
	double dst_east = diff_east + pr.x * coef;

	int north_grid = dst_north*hrat + center_height;
	int east_grid = dst_east*wrat + center_width;
		


   	/* Calculate delta */
   	double dLat = pr.y * coef / EARTH_RADIUS_M;
   	double dLon = pr.x * coef / (EARTH_RADIUS_M * std::cos(radLat));
		
   	/* Add back to start point */
   	double new_lat = (radLat + dLat) / PI_C * 180;
   	double new_lon = (radLon + dLon) / PI_C * 180;    		

   	int grid_pos = north_grid*width + east_grid;

	/* Occupy a part of it */
	if (grid_pos < size && !matrix[grid_pos]) {
		std::cout << uav_data.lon << " " << uav_data.lat << "/" << grid_pos << std::endl;


		/* Assume a target is almost a grid in size (may need fix) */
		matrix[north_grid*width + east_grid] = 1;
		lst.push_back({new_lat, new_lon});
	}
}

void WaypointMap::clearMatrix() {
	std::memset(matrix, 0, size*sizeof(int));
}

void WaypointMap::printMatrix() {
	std::cout << "\033[2J"; 
    std::cout << "\033[H";

	for (int i = 0;i < height;i++) {
		for (int j = 0;j < width;j++) 
			std::cout << matrix[width*i + j] << " ";
		std::cout << std::endl;
	}
}