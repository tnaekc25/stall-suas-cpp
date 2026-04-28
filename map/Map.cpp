#include "map.h"


static int node_id = 0;


Map::Map() {
	int size = REAL_WIDTH*REAL_HEIGHT;
	list_tail = &list_head;

	matrix = new MapImage*[size]();
	scoreMatrix = new double[size];

	//for (int i = 0;i < size;i++) matrix[i] = nullptr;
}

Map::~Map() {
	delete [] matrix;
	delete [] scoreMatrix;
}

void Map::freeFrames() {
	MapImage *current = list_head.next;
	while (current) {
		MapImage *tmp = current -> next;
		delete current;

		current = tmp;
	}
}

/* This is used to scale the image into matrix */
Quad Map::scaleImage(double lat, double lon, double alt,
 double roll, double pitch, double yaw) {

	double cosX = std::cos(pitch + CAM_PITCH), sinX = std::sin(pitch + CAM_PITCH);
	double cosY = std::cos(roll), sinY = std::sin(roll);
	double cosZ = std::cos(yaw), sinZ = std::sin(yaw);

	double cam_x = 0.5 * IMAGE_WIDTH / (double)CAM_FX;
	double cam_y = 0.5 * IMAGE_HEIGHT / (double)CAM_FY;

	/* rotate corners to put on real world frame */
	Point3 p1r = ROTATE_POINT(cam_x, cam_y);
	Point3 p2r = ROTATE_POINT(-cam_x, cam_y);
	Point3 p3r = ROTATE_POINT(-cam_x, -cam_y);
	Point3 p4r = ROTATE_POINT(cam_x, -cam_y);

	if (p1r.z <= 0 || p2r.z <= 0 || p3r.z <= 0 || p4r.z <= 0)
		return Quad();

	double s1 = alt / p1r.z;
	double s2 = alt / p2r.z;
	double s3 = alt / p3r.z;
	double s4 = alt / p4r.z;

	double lat_rad = PI_C / 180 * lat;
	double lon_rad = PI_C / 180 * lon;

	/* calculate distance from map center */
	double north = EARTH_RADIUS_M * (lat_rad - MAP_LAT);
	double east = EARTH_RADIUS_M * std::cos((lat_rad + MAP_LAT) * 0.5) * (lon_rad - MAP_LON);

	/* points scaled onto the matrix */
	return {{(east + p1r.x*s1)*W_RATIO + CENTER_WIDTH, (north + p1r.y*s1)*H_RATIO + CENTER_HEIGHT},
		{(east + p2r.x*s2)*W_RATIO + CENTER_WIDTH, (north + p2r.y*s2)*H_RATIO + CENTER_HEIGHT},
		{(east + p3r.x*s3)*W_RATIO + CENTER_WIDTH, (north + p3r.y*s3)*H_RATIO + CENTER_HEIGHT},
		{(east + p4r.x*s4)*W_RATIO + CENTER_WIDTH, (north + p4r.y*s4)*H_RATIO + CENTER_HEIGHT}};
}



/* Check if the point is inside the quad */
#define POINT_INSIDE ( \
	(p2.x - p1.x) * ((i + 0.5) - p1.y) - (p2.y - p1.y) * ((j + 0.5) - p1.x) >= 0 && \
	(p3.x - p2.x) * ((i + 0.5) - p2.y) - (p3.y - p2.y) * ((j + 0.5) - p2.x) >= 0 && \
	(p4.x - p3.x) * ((i + 0.5) - p3.y) - (p4.y - p3.y) * ((j + 0.5) - p3.x) >= 0 && \
	(p1.x - p4.x) * ((i + 0.5) - p4.y) - (p1.y - p4.y) * ((j + 0.5) - p4.x) >= 0) \


/* Update the matrix with the image's quad */
void Map::updateMatrix(cv::Mat *frame, UavData &data, double image_score, Quad rect) {
	MapImage *image = new MapImage(frame, data, node_id++); /* Create image wrapper */
	bool image_inserted = false; /* if the image is referenced */

	/* get points of the quad */
	auto& [p1, p2, p3, p4] = rect;

	/* check if ccw */
	double area = (p2.x - p1.x) * (p2.y + p1.y) +
		(p3.x - p2.x) * (p3.y + p2.y) +
		(p4.x - p3.x) * (p4.y + p3.y) +
		(p1.x - p4.x) * (p1.y + p4.y);

	/* if not swap p2 and p4 so it is ccw */
	if (area > 0) {Point tmp = p2; p2 = p4; p4 = tmp;}

	/* find the bounding-box of the quad */
	int min_x = std::max((int)std::floor(std::min({p1.x, p2.x, p3.x, p4.x})), 0);
	int max_x = std::min((int)std::ceil (std::max({p1.x, p2.x, p3.x, p4.x})), REAL_WIDTH - 1);
	
	int min_y = std::max((int)std::floor(std::min({p1.y, p2.y, p3.y, p4.y})), 0);
	int max_y = std::min((int)std::ceil (std::max({p1.y, p2.y, p3.y, p4.y})), REAL_HEIGHT - 1);


	int total_cells = 0, new_cells = 0, old_cells = 0;

	/* Find gain */
	for (int i = min_y;i <= max_y;i++) {
		for (int j = min_x;j <= max_x;j++) {
			if (POINT_INSIDE){
				if (!matrix[i*REAL_WIDTH + j]) new_cells++;
				else if (scoreMatrix[i*REAL_WIDTH + j]*SCORE_BIAS < image_score) old_cells++;
				total_cells++;
			}
		}
	}

	/* Dont put the frame if the gain is not enough */
	double inter_gain = new_cells / (double)total_cells;
	if (inter_gain < MIN_INTER_GAIN || (inter_gain > MAX_INTER_GAIN && inter_gain < 1))
		if (old_cells / (double)total_cells < SCORE_GAIN) {delete image; return;}

	/* iterate scanline */
	for (int i = min_y;i <= max_y;i++) {
		for (int j = min_x;j <= max_x;j++) {
			if (POINT_INSIDE) {
				bool is_insert = false;

				/* if already occupied and score is bigger */
				if (matrix[i*REAL_WIDTH + j]) {
					if (scoreMatrix[i*REAL_WIDTH + j] < image_score) {
						MapImage &ref = *matrix[i*REAL_WIDTH + j];
						if (!(--ref.ref_count)) {
							if (ref.next) ref.next -> prev = ref.prev;
							if (ref.prev) ref.prev -> next = ref.next;
		
							if (list_tail == &ref) {list_tail = ref.prev; list_tail -> next = nullptr;} 
							ref.next = ref.prev = nullptr;
							
							/* delete the image if referenced by no tile */ 
							delete &ref;
						}
	
						/* add pointer reference to image */
						is_insert = true;
					}
				}

				else is_insert = true;

				if (is_insert) {
					scoreMatrix[i*REAL_WIDTH + j] = image_score;
					matrix[i*REAL_WIDTH + j] = image;
					image -> ref_count += 1;
	
					if (!image_inserted) {
						image -> prev = list_tail;
						list_tail -> next = image;
						list_tail = image;
						image_inserted = true;
					}
				}
			}
		}
	}

	/* delete the image if referenced by no tile */ 
	if (!image -> ref_count) delete image;
}



#define CLEAR_OVERFLOW { \
	if (!matrix[i*REAL_WIDTH + j]) continue; \
	MapImage &ref = *matrix[i*REAL_WIDTH + j]; \
	\
	if (!(--ref.ref_count)) { \
		if (ref.next) ref.next -> prev = ref.prev; \
		if (ref.prev) ref.prev -> next = ref.next; \
		\
		if (list_tail == &ref) {list_tail = ref.prev; list_tail -> next = NULL;} \
		ref.next = ref.prev = NULL; delete &ref; \
	} \
} \

/* This function is to clear the frames that are completely in the overflow region */
void Map::clearOverflow() {

	/* check out of bound entries */

	for (int i = 0;i < MARGIN_HEIGHT;i++) {
		for (int j = 0;j < MARGIN_WIDTH;j++) CLEAR_OVERFLOW
		for (int j = MATRIX_WIDTH + MARGIN_WIDTH;j < REAL_WIDTH;j++) CLEAR_OVERFLOW
	}

	for (int i = MATRIX_HEIGHT + MARGIN_HEIGHT;i < REAL_HEIGHT;i++) {
		for (int j = 0;j <= MARGIN_WIDTH;j++) CLEAR_OVERFLOW
		for (int j = MATRIX_WIDTH + MARGIN_WIDTH;j < REAL_WIDTH;j++) CLEAR_OVERFLOW
	}
}


std::vector<MapImage*> Map::getReferenceList() {
	std::vector<MapImage*> ret;

	MapImage *current = list_head.next;
	while (current) {
		ret.push_back(current);
		current = current -> next;
	}

	return ret;
}


void Map::saveImages(std::string dir) {

	std::ofstream data_file(dir + "/geo.txt");
	data_file << std::fixed << std::setprecision(8);
	
	data_file << "EPSG:32616" << std::endl;

	int img_i = 0;
	MapImage *current = list_head.next;
	
	while (current) {
		UavData &data = current -> uav_data;

		cv::imwrite(dir + "/output" + std::to_string(img_i) + ".jpg", *current -> frame);
		data_file << "output" + std::to_string(img_i) + ".jpg " <<
			data.lat << " " << data.lon << " " << data.alt << std::endl;

		current = current -> next;
		img_i++;
	}
}




