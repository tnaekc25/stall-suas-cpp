#ifndef SUAS_IMAGE_THREAD_H_
#define SUAS_IMAGE_THREAD_H_

#include "../common.h"
#include "../image.h"

#include "../map/map.h"

#define SIGMA_SD 0.02
#define SCORE_SAMPLE 150
#define SCORE_TRESH 0.1

#define ESTIMATED_FRAME_DELAY 5 /* In ms */

int matchAndMap(TelemBuffer *telem_buffer, FrameQueue *frame_queue);

#endif