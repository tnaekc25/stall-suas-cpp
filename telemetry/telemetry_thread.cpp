#include "telemetry_thread.h"


int64_t get_kernel_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}


/* Thread to read data from pixhawk over serial and parse */
int telemRead(TelemBuffer *telem_buffer, SerialHandler *serial) {
	bool hb_recvd = 0;

	bool sync_init = 0;
	double sync_time_ema = 0;

	auto recv_period = std::chrono::milliseconds(TELEM_RECV_PERIOD);

	auto hb_recv_time = std::chrono::steady_clock::now();
	auto next_time = std::chrono::steady_clock::now();

	int64_t last_recv_time;

	/* The snapshot ready flag (ready if 111) */
	unsigned char snap_cc = 0;
	UavData snapshot;

	/* Mavlink types */
	mavlink_message_t message;
	mavlink_status_t status;

	mavlink_timesync_t timesync_st;
	mavlink_global_position_int_t gps_st;
	mavlink_local_position_ned_t ned_st;
	mavlink_attitude_t attitude_st;
	mavlink_named_value_int_t named_int_st;


	while(true) {

		auto current_time = std::chrono::steady_clock::now();

		/* HB Timeout or Serial fd issue */
		if (!serial -> checkConnection() ||
		 (current_time - hb_recv_time) > std::chrono::milliseconds(HB_TIMEOUT)) {
			std::cout << "TELEM DISCONNECT/TIMEOUT..." << std::endl;

			/* Try reconnect */
			if (serial -> serialConnect())
				/* Reset snapshot */
				snap_cc = 0;

			/* Reset hb time */
			hb_recv_time = current_time;
		}

		/* No timeout */
		else {
			const ssize_t retsize = serial -> readData(&last_recv_time);
	
			for (int i = 0;i < retsize;i++) {
				if (mavlink_parse_char(MAVLINK_COMM_0, (serial -> getBuffer())[i], &message, &status)) {
					//std::cout << (int)snap_cc << std::flush;
	
					switch (message.msgid) {
						case MAVLINK_MSG_ID_HEARTBEAT:
							{
								hb_recv_time = std::chrono::steady_clock::now();
								if (hb_recvd); else {hb_recvd = 1; global_hb_latch.notify();}
							}
	
							break;

						case MAVLINK_MSG_ID_TIMESYNC:
							if (hb_recvd) {
								mavlink_msg_timesync_decode(&message, &timesync_st);
								if (timesync_st.tc1 == 0) break;

								int64_t offset_ns = timesync_st.tc1 - (last_recv_time + timesync_st.ts1) / 2;
								if (sync_init) sync_time_ema += EXP_ALPHA * ((double)offset_ns - sync_time_ema);
								else sync_time_ema = ((double)offset_ns);

								sync_time_offset.store((int64_t)(sync_time_ema) / 1000000);
								//std::cout << "SYNC: " << sync_time_offset.load() << std::endl;
							}
	
							break;
	
						case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
							if (hb_recvd) {
								mavlink_msg_global_position_int_decode(&message, &gps_st);
		
								snapshot.lat = gps_st.lat / 1000000.0;
								snapshot.lon = gps_st.lon / 1000000.0;
								snapshot.alt = gps_st.relative_alt / 1000.0;
			
								/* Set time based on lat lon alt for precise mapping */
								snapshot.time = gps_st.time_boot_ms;
		
								snap_cc |= 1u;
							}

							break;
	
						
						case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
							if (hb_recvd) {
								mavlink_msg_local_position_ned_decode(&message, &ned_st);
		
								snapshot.vx = ned_st.vx;
								snapshot.vy = ned_st.vy;
								snapshot.vz = ned_st.vz;
		
								snap_cc |= 2u;
							}

							break;
	
						case MAVLINK_MSG_ID_ATTITUDE:
							if (hb_recvd) {
								mavlink_msg_attitude_decode(&message, &attitude_st);
		
								snapshot.roll = attitude_st.roll;
								snapshot.pitch = attitude_st.pitch;
								snapshot.yaw = attitude_st.yaw;
		
								snap_cc |= 4u;
							}

							break;
	
						case MAVLINK_MSG_ID_NAMED_VALUE_INT:
							if (hb_recvd) {
								mavlink_msg_named_value_int_decode(&message, &named_int_st);
								
								/* Handle messages from gcs */
								if (!std::strcmp(named_int_st.name, "GCSD")) {
									flight_mode.store(named_int_st.value | 0b1); /* first bit is flight mode selector */
								}
							}
	
							break; 
	
						default:
							break;
					}
	
					/* If all areas are set put into buffer */
					if (snap_cc == 0b111) {
						//std::cout << ">> TELEM at " << snapshot.time << "ms ADD" << std::endl;
						telem_buffer->putData(snapshot); snap_cc = 0;
					}
				} 
	
				else {
					/* HANDLE PARSE ERROR */
				}
			}
		}

		/* Synchronize with a constant period */
		while (next_time <= std::chrono::steady_clock::now())
			next_time += recv_period;
		std::this_thread::sleep_until(next_time);
	}

	return 0;
}


int telemWrite(SerialHandler *serial) {
	auto send_period = std::chrono::milliseconds(TELEM_SEND_PERIOD);
	
	const int sync_count = TELEM_SYNC_PERIOD / TELEM_SEND_PERIOD;
	const int telem_period = 1000000 / TELEM_FREQ;

	int sync_timer_counter = sync_count;

	/* Prepare the heartbeat message for jetson */
	mavlink_message_t hb_msg;
	mavlink_msg_heartbeat_pack(1, 
		MAV_COMP_ID_ONBOARD_COMPUTER,
		&hb_msg,
		MAV_TYPE_ONBOARD_CONTROLLER,
		MAV_AUTOPILOT_INVALID,
		0,
		0,
		MAV_STATE_ACTIVE);

	uint8_t hb_buf[MAVLINK_MAX_PACKET_LEN];
	int hb_len = mavlink_msg_to_send_buffer(hb_buf, &hb_msg);

	uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN];


	/* Wait until first HB is get */
	global_hb_latch.wait();

	auto sendMessageInterval = [&](uint32_t msg_id, int32_t interval_us) {
		mavlink_message_t msg;
		mavlink_msg_command_long_pack(
			1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
			1, 1,
			MAV_CMD_SET_MESSAGE_INTERVAL,
			0,
			(float)msg_id,
			(float)interval_us,
			0, 0, 0, 0, 0
		);
		uint8_t buf[MAVLINK_MAX_PACKET_LEN];
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		serial->writeData(buf, len);
	};

	sendMessageInterval(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, telem_period);
	sendMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, telem_period);
	sendMessageInterval(MAVLINK_MSG_ID_ATTITUDE, telem_period);


	/* Hold time to set deadline */
	auto next_time = std::chrono::steady_clock::now();

	while (true) {
		auto current_time = std::chrono::steady_clock::now();

		/* Send heartbeat by default from onboard computer */
		if (serial -> writeData(hb_buf, hb_len) < 0); /* HANDLE WRITE ERROR */


		if (sync_timer_counter >= sync_count) {
			sync_timer_counter = 0;

			/* Prepare the timesync message */
			int64_t now_ns = get_kernel_ns();
			
			mavlink_message_t sync_msg;
			mavlink_msg_timesync_pack(1,
				MAV_COMP_ID_ONBOARD_COMPUTER,
				&sync_msg,
				0,
				now_ns,
				1,
				1
			);
	
			int sync_len = mavlink_msg_to_send_buffer(msg_buf, &sync_msg);
	
			/* Send timesync request */
			if (serial -> writeData(msg_buf, sync_len) < 0); /* HANDLE WRITE ERROR */	
		}

		sync_timer_counter = (sync_timer_counter < sync_count) ? (sync_timer_counter + 1) : sync_timer_counter;

		/* Synchronize with a constant period */
		while (next_time <= current_time)
			next_time += send_period;
		std::this_thread::sleep_until(next_time);
	}

	return 0;
}