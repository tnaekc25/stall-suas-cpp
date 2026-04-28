#ifndef SUAS_MAP_H_
#define SUAS_MAP_H_

#include "../common.h"
#include "../image.h"

#include <fstream>
#include <iomanip>



/* Below are constants. Each cell represent an area of [(LAT_SIZE * LON_SIZE) / (MATRIX_WIDTH * MATRIX_HEIGHT)] */

#define MATRIX_WIDTH 100
#define MATRIX_HEIGHT 70
#define MARGIN_WIDTH 50
#define MARGIN_HEIGHT 35

#define MIN_INTER_GAIN 0.2
#define MAX_INTER_GAIN 1

#define SCORE_GAIN 0.5 /* set high to make score comparison local */
#define SCORE_BIAS 1.5

#define MAP_LAT 0 /* In radians */
#define MAP_LON 0 /* In radians */

#define LON_SIZE 260 /* In meters */
#define LAT_SIZE 180 /* In meters */


constexpr double W_RATIO = MATRIX_WIDTH / (double)LON_SIZE;
constexpr double H_RATIO = MATRIX_HEIGHT / (double)LAT_SIZE;

constexpr int REAL_WIDTH = MATRIX_WIDTH + MARGIN_WIDTH*2;
constexpr int REAL_HEIGHT = MATRIX_HEIGHT + MARGIN_HEIGHT*2;

constexpr int CENTER_WIDTH = REAL_WIDTH / 2;
constexpr int CENTER_HEIGHT = REAL_HEIGHT / 2;


struct MapImage {
	UavData uav_data;

	MapImage *next, *prev;
	cv::Mat *frame;

	int id;
	int ref_count;

	MapImage() : next(NULL), prev(NULL), frame(0), ref_count(0) {}
	MapImage(cv::Mat *_frame, UavData _uav_data, int _id) : uav_data(_uav_data), next(NULL), 
	prev(NULL), frame(_frame),  id(_id), ref_count(0) {}
	~MapImage() {delete frame;}
};



class Map {
	private:
		MapImage list_head, *list_tail;
		MapImage **matrix;
		double *scoreMatrix;

	public:
		Map();
		~Map();

		Quad scaleImage(double lat, double lon, double alt, double roll, double pitch, double yaw);
		void updateMatrix(cv::Mat *frame, UavData &data, double image_score, Quad rect);
	
		void clearOverflow();		
		std::vector<MapImage*> getReferenceList(); 

		void saveImages(std::string dir); 
		void freeFrames();


		void printMatrix() {
			for (int i = 0; i < MATRIX_HEIGHT;i++) {
				for (int j = 0; j < MATRIX_WIDTH;j++) {
					if (matrix[(i + MARGIN_HEIGHT)*REAL_WIDTH + j + MARGIN_WIDTH]) {

						unsigned int hash = matrix[(i + MARGIN_HEIGHT)*REAL_WIDTH + j + MARGIN_WIDTH]->id * 2654435761u; 
						
						int r = (hash & 0xFF) % 156 + 100;
						int g = (hash >> 8 & 0xFF) % 156 + 100;
						int b = (hash >> 16 & 0xFF) % 156 + 100;


						std::cout << "\033[38;2;" << r << ";" << g << ";" << b << "m"
              				<< "██" << "\033[0m";
					}

					else std::cout << "* ";
				}
				std::cout << "\n";
			}


		}

};

#endif