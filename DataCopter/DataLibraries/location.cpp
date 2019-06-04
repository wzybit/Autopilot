/*
 *@File     : location.cpp
 *@Author   : wangbo
 *@Date     : May 4, 2016
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : ?????????????????????????-??????????????
 */

#include <stdio.h>
#include <math.h>

#include "location.h"

//static int sgn(float x)
//{
//	if (x>0)
//	{
//		return 1;
//	}
//	else
//	{
//		return 0;
//	}
//}

static double haversin_equation(double theta)
{
	double v;

	v = sin(theta / 2);

	return v * v;
}

double convert_degree_to_radian(double angle_degree)
{
	return angle_degree * M_PI / 180;
}

double convert_radian_to_degree(double angle_radian)
{
	return angle_radian * 180.0 / M_PI;
}

/*
 * ??????????-PI~+PI ?????????+-180??
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
*/
double wrap_PI(double angle_radian)
{
	if (angle_radian > 10 * M_PI || angle_radian < -10 * M_PI)
	{
		// for very large numbers use modulus
	    angle_radian = fmod(angle_radian, 2 * M_PI);
	}
	while (angle_radian > M_PI) angle_radian -= 2 * M_PI;
	while (angle_radian < -M_PI) angle_radian += 2 * M_PI;

	return angle_radian;
}

/*
 * ??????0~2PI??
 * wrap an angle in radians to 0..2PI
*/
double wrap_2PI(double angle_radian)
{
	if (angle_radian > 10 * M_PI || angle_radian < -10 * M_PI)
	{
		// for very large numbers use modulus
	    angle_radian = fmod(angle_radian, 2 * M_PI);
	}
	while (angle_radian > 2 * M_PI) angle_radian -= 2 * M_PI;
	while (angle_radian < 0) angle_radian += 2 * M_PI;

	return angle_radian;
}

double get_distance_loc_to_loc(T_Location *start_loc,T_Location *end_loc)
{
    double longitude1=0.0, latitude1=0.0;
    double longitude2 = 0.0, latitude2 = 0.0;
    double delta_lng, delta_lat, h, distance_pt2pt;

	longitude1 = convert_degree_to_radian(start_loc->lng);
	latitude1 = convert_degree_to_radian(start_loc->lat);
	longitude2 = convert_degree_to_radian(end_loc->lng);
	latitude2 = convert_degree_to_radian(end_loc->lat);

	delta_lng = fabs(longitude1 - longitude2);
	delta_lat = fabs(latitude1 - latitude2);

	h = haversin_equation(delta_lat) + cos(latitude1) * cos(latitude2) * haversin_equation(delta_lng);

	distance_pt2pt = 2 * EARTH_RADIUS * sin(sqrt(h));

	return distance_pt2pt;
}

double get_mercator_x(T_Location *loc)
{
    double longitude_radian = 0.0;
    double lambda_0 = 0.0;

	longitude_radian = convert_degree_to_radian(loc->lng);
	lambda_0 = REFERENCE_LONGITUDE_MERCATOR;

	return EARTH_CIRCUMFERENCE*(longitude_radian - lambda_0);
}

double get_mercator_y(T_Location *loc)
{
    double phi_0 = 0.0;

	phi_0 = convert_degree_to_radian(loc->lat);

	return EARTH_CIRCUMFERENCE * log(tan(M_PI / 4 + phi_0 / 2));
}

unsigned char arrive_specific_location_radius(T_Location *current_loc,T_Location *specific_loc,unsigned int arrive_radius)
{
	double distance_pt2pt = 0.0;

	distance_pt2pt = get_distance_loc_to_loc(current_loc, specific_loc);

	if (distance_pt2pt < (double)arrive_radius)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*****************************???????????????????guidance??***************/
/*****************************/

/*
 * Function:       get_bearing_point_2_point_NED
 * Description:  ???????????????????,
 *                        ????????????????NED????x????,-180-+180,???0,???,???
 *                        ??????????????,
 *                        ?????:2?????????,?????????????????????????
 */
double get_bearing_point_2_point_NED(T_Location *previous_target_loc, T_Location *target_loc)
{
    struct T_NED start_point;
    struct T_NED end_point;

    /*????????,??x?,??y?,???z?*/
    double vector_x = 0.0, vector_y = 0.0;

    start_point.x = get_mercator_y(previous_target_loc);
    start_point.y = get_mercator_x(previous_target_loc);
    end_point.x = get_mercator_y(target_loc);
    end_point.y = get_mercator_x(target_loc);

    vector_x = end_point.x - start_point.x;
    vector_y = end_point.y - start_point.y;

    return atan2(vector_y,vector_x);
}



