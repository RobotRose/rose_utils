/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/09/10
*       - File created.
*
* Description:
*   Point stamped
* 
***********************************************************************************/

#ifndef POINT_STAMPED_HPP
#define POINT_STAMPED_HPP

#include <iostream>

#include <std_msgs/Header.h>
#include <geometry_msgs/PointStamped.h>

namespace rose_geometry{

template <typename T>
class Stamped 
{
  public:
  	Stamped()
	{}

	Stamped(const std_msgs::Header& init_header, const T& new_data)
	{
		header 	= init_header;
		data  	= new_data;
	}

	~Stamped()
	{}

    std_msgs::Header 	header;
	T 					data;

	// Operators
	inline Stamped<T>& operator=(const Stamped<T>& rhs)
	{
		header 	= rhs.header;
		data 	= rhs.data;

		return *this;
	}
	//! @todo OH: Move operator etc.
	
};

template <typename T> 
inline bool operator==(const Stamped<T>& lhs, const Stamped<T>& rhs){return (	lhs.header.seq == rhs.header.seq && 
																				lhs.header.stamp == rhs.header.stamp &&
																				lhs.header.frame_id == rhs.header.frame_id &&
																				lhs.data == rhs.data); }
template <typename T> 
inline bool operator!=(const Stamped<T>& lhs, const Stamped<T>& rhs){return !operator==(lhs,rhs);}
template <typename T> 
inline bool operator< (const Stamped<T>& lhs, const Stamped<T>& rhs){return false; }
template <typename T> 
inline bool operator> (const Stamped<T>& lhs, const Stamped<T>& rhs){return  operator< (rhs,lhs);}
template <typename T> 
inline bool operator<=(const Stamped<T>& lhs, const Stamped<T>& rhs){return !operator> (lhs,rhs);}
template <typename T> 
inline bool operator>=(const Stamped<T>& lhs, const Stamped<T>& rhs){return !operator< (lhs,rhs);}

}; // rose_geometry


#endif // POINT_STAMPED_HPP 
