/*
 * GCS.h
 *
 *  Created on: 2017-9-18
 *      Author: wangbo
 */

#ifndef GCS_H_
#define GCS_H_

#include <stdint.h>

///
/// @class	GCS
/// @brief	Class describing the interface between the APM code
///			proper and the GCS implementation.
///
/// GCS' are currently implemented inside the sketch and as such have
/// access to all global state.  The sketch should not, however, call GCS
/// internal functions - all calls to the GCS should be routed through
/// this interface (or functions explicitly exposed by a subclass).
///
class GCS_Class
{
public:

	

	/// Update GCS state.
	///
	/// This may involve checking for received bytes on the stream,
	/// or sending additional periodic messages.
	//void		update(void) {}

	

    // send streams which match frequency range
    //void data_stream_send(uint16_t freqMin, uint16_t freqMax);

   

protected:
	
};

//
// GCS class definitions.
//
// These are here so that we can declare the GCS object early in the sketch
// and then reference it statically rather than via a pointer.
//


void GCS_SendString(const char *str);




#endif /* GCS_H_ */
