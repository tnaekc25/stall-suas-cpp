#include "image_thread.h"


float scoreImage(cv::Mat &frame, float roll, float pitch, cv::Mat &lap, cv::Mat &mean, cv::Mat &stddev) {
	float tilt_sq = roll*roll + pitch*pitch;
	float tilt_score = std::exp(-tilt_sq / SIGMA_SD);

	cv::Laplacian(roi, lap, CV_16S, 1); 
	cv::meanStdDev(lap, mean, stddev);
	double quality_score = stddev.at<double>(0) * stddev.at<double>(0);

	float score = quality_score*tilt_score;
	return (score > SCORE_TRESH) ? score : 0;
}


int matchAndMap(TelemBuffer *telem_buffer, FrameQueue *frame_queue) {
	cv::Mat lap, mean, stddev;

	bool first_mode_change = true;

	Map *matrix_map = new Map;
	
	cv::VideoCapture cap(0);
	if(!cap.isOpened()) {
	    std::cout << "CAM FAILED TO INIT" << std::endl;
	}

	cap.set(cv::CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
	cap.set(cv::CAP_PROP_FPS, CAM_FPS); 

	auto next_time = std::chrono::steady_clock::now();
	const int frame_time_ms = 1000 / CAM_FPS;
	auto frame_period = std::chrono::milliseconds(frame_time_ms);


	while (true) {
		cv::Mat *frame = new cv::Mat;

		cap.read(*frame);
		if(frame -> empty()) {
			std::cout << "FRAME EMPTY" << std::endl;
		}

		/* TODO CALCULATE FRAME TIME */
		int64_t frame_time = ESTIMATED_FRAME_DELAY; 
		uint32_t sync_frame_time = frame_time + sync_time_offset.load();
		UavData uav_data = telem_buffer -> getData(sync_frame_time);

		/* enqueue matched image */
		MatchedImage *matched_image = new MatchedImage(*frame, uav_data);
		frame_queue -> push(matched_image);
		/* deallocation will occur after detection */

		/* Do mapping if and only if in mode A */
		if (!flight_mode.load()) {
			float img_score = scoreImage(*frame, uav_data.roll, uav_data.pitch);
			Quad img_scale = matrix_map -> scaleImage(uav_data.lat, uav_data.lon, uav_data.alt, uav_data.roll, uav_data.pitch, uav_data.yaw);
			
			matrix_map -> updateMatrix(frame, uav_data, img_score, img_scale);
		}

		/* Save images and free the matrix if the mode changed */
		else if (first_mode_change) {
			first_mode_change = false;
			matrix_map -> saveImages("images");
			
			delete matrix_map;
		}

		/* Synchronize with the camera fps */
		while (next_time <= std::chrono::steady_clock::now())
			next_time += frame_period;
		std::this_thread::sleep_until(next_time);
	}
}