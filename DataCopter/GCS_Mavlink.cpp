#include "Copter.h"
#include "GCS_Mavlink.h"

GCS_MAVLINK::GCS_MAVLINK()
{   
    streamRateRawSensors       = 0;
    streamRateExtra1           = 0;
	streamRateExtra2           = 0;
	streamRateExtra3           = 0;
    
    streamRateANO_STATUS       = 0;
    streamRateANO_SENSOR       = 0;
    streamRateANO_RCDATA       = 0;
    streamRateANO_GPSDATA      = 0;
    streamRateANO_PID_RPY      = 10;
    streamRateANO_PID_RPY_RATE = 0;

}
    
void GCS_MAVLINK::gcs_init()
{
    
//    streamRateRawSensors       = 0;
//    streamRateExtra1           = 0;
//	streamRateExtra2           = 0;
//	streamRateExtra3           = 0;
//    
//    streamRateANO_STATUS       = 0;
//    streamRateANO_SENSOR       = 0;
//    streamRateANO_RCDATA       = 1;
//    streamRateANO_GPSDATA      = 0;
//    streamRateANO_PID_RPY      = 0;
//    streamRateANO_PID_RPY_RATE = 0;
//	
    
}


// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
	int16_t *stream_rates = &streamRateRawSensors;
	uint8_t rate = (uint8_t)stream_rates[stream_num];
    
    //DEBUG_PRINTF("GCS_MAVLINK::stream_trigger    :    rate = %d, stream_num = %d \n", rate, (int)stream_num);

	if (rate == 0) {
		return false;
	}

	if (stream_ticks[stream_num] == 0) {
		// we're triggering now, setup the next trigger point
		if (rate > 50) {
			rate = 50;
		}
		stream_ticks[stream_num] = (50 / rate);
		return true;
	}

	// count down at 50Hz
	stream_ticks[stream_num]--;
	return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
   

//    if (stream_trigger(STREAM_RAW_SENSORS)) 
//    {
//        send_message(MSG_RAW_IMU1);//2hz
//        send_message(MSG_RAW_IMU2);
//        send_message(MSG_RAW_IMU3);
//    }
    
    if (stream_trigger(STREAM_ANO_STATUS)) 
    {
        send_message(MSG_ANO_STATUS);
    }
    
    if (stream_trigger(STREAM_ANO_SENSOR)) 
    {
        send_message(MSG_ANO_SENSOR);
    }
    
    
    if (stream_trigger(STREAM_ANO_RCDATA)) 
    {
        send_message(MSG_ANO_RCDATA);
    }
    
    
    if (stream_trigger(STREAM_ANO_GPSDATA)) 
    {
        send_message(MSG_ANO_GPSDATA);
    }
    
    
    if (stream_trigger(STREAM_ANO_PID_RPY)) 
    {
        //printf("wangbo\n");
        send_message(MSG_ANO_PID_RPY);
    }
    
    
    if (stream_trigger(STREAM_ANO_PID_RPY_RATE)) 
    {
        send_message(MSG_ANO_PID_RPY_RATE);
    }

    

}

void GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message( id );
    
    //send_message 也可以是别的通信协议，比如
    //wangbo_link_send_message( id );
}

void GCS_MAVLINK::mavlink_send_message(enum ap_message id)
{
    //printf("send id = %d \n", (int)id);
    switch(id)
	{
	case MSG_HEARTBEAT:
		//copter.send_heartbeat();
		break;
	case MSG_ATTITUDE:
		//copter.send_attitude();
		break;
    
    default:
        break; // just here to prevent a warning
	}
}
