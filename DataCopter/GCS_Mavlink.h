/*
 * GCS_Mavlink_Copter.h
 *
 *  Created on: 2017-9-22
 *      Author: wangbo
 */

#ifndef GCS_MAVLINK_COPTER_H_
#define GCS_MAVLINK_COPTER_H_

#include "GCS.h"

///
/// @class	GCS_MAVLINK
/// @brief	The mavlink protocol for qgroundcontrol
///
class GCS_MAVLINK : public GCS_Class
{
public:
	//GCS_MAVLINK(){}
    GCS_MAVLINK();
	void    update(void);
//    void    data_stream_send(uint16_t freqMin, uint16_t freqMax);
        
    void    gcs_init();
	void    queued_param_send();
	void    queued_waypoint_send();

	void data_stream_send(void);
	void send_message(enum ap_message id);
    void mavlink_send_message(enum ap_message id);
  
public:
    //void 	handleMessage(mavlink_message_t * msg);

   

	
    // NOTE! The streams enum below and the
	// set of stream rates _must_ be
	// kept in the same order
	enum streams 
    {
        STREAM_RAW_SENSORS,
        STREAM_EXTRA1,
        STREAM_EXTRA2,
        STREAM_EXTRA3,
        
        STREAM_ANO_STATUS,
        STREAM_ANO_SENSOR,
        STREAM_ANO_RCDATA,
        STREAM_ANO_GPSDATA,
        STREAM_ANO_PID_RPY,
        STREAM_ANO_PID_RPY_RATE,
        NUM_STREAMS
    };
    // see if we should send a stream now. Called at 50Hz
	bool stream_trigger(enum streams stream_num);
        
	/// this is set of stream rates
    /// data stream rates 实时数据发送的频率
	int16_t streamRateRawSensors;
	int16_t streamRateExtra1;
	int16_t streamRateExtra2;
	int16_t streamRateExtra3;
    
    int16_t streamRateANO_STATUS;
    int16_t streamRateANO_SENSOR;
    int16_t streamRateANO_RCDATA;
    int16_t streamRateANO_GPSDATA;
    int16_t streamRateANO_PID_RPY;
    int16_t streamRateANO_PID_RPY_RATE;

    // number of 50Hz ticks until we next send this stream
    uint8_t stream_ticks[NUM_STREAMS];

    // saveable rate of each stream
   int16_t  streamRates[NUM_STREAMS];

};


//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
//    MSG_LOCATION,
//    MSG_EXTENDED_STATUS1,
//    MSG_EXTENDED_STATUS2,
//    MSG_NAV_CONTROLLER_OUTPUT,
//    MSG_CURRENT_WAYPOINT,
//    MSG_VFR_HUD,
//    MSG_RADIO_OUT,
//    MSG_RADIO_IN,
//    MSG_RAW_IMU1,
//    MSG_RAW_IMU2,
//    MSG_RAW_IMU3,
//    MSG_GPS_STATUS,
//    MSG_GPS_RAW,
//    MSG_SERVO_OUT,
//    MSG_NEXT_WAYPOINT,
//    MSG_NEXT_PARAM,
//    MSG_STATUSTEXT,
//    MSG_AHRS,
//    MSG_SIMSTATE,
//    MSG_HWSTATUS,
    
    MSG_ANO_STATUS,
    MSG_ANO_SENSOR,
    MSG_ANO_RCDATA,
    MSG_ANO_GPSDATA,
    MSG_ANO_PID_RPY,
    MSG_ANO_PID_RPY_RATE,
    
    MSG_RETRY_DEFERRED // this must be last
};




#endif /* GCS_MAVLINK_COPTER_H_ */


