#pragma once

#include <conversions.hpp>
#include <basic_lowpass_filters.cpp>

namespace nrg_tools{
	/** 
	 * \class RosLowPassFilter
	 * A Low Pass filter for ROS message types. 
	 * Useful for filtering sensor feedback (e.g. IMU, Force/Torque sensors, etc)
	 * directly from subscribers
	 */
	template <class T, class U> class RosLowPassFilter
	{
	public:
		/**
		 * Constructor
		 * @param message_type			The message type you want to filter. If it is not empty, the values will be used as the initial values
		 * @param filter_coefficients	The coefficients of the filters. Does not have to be the same type as the message_type parameter, just the same length. Higher values = more smoothing but more lag
		 */
		RosLowPassFilter(T message_type, U filter_coefficients);

		/**
		 * Updates the filter with the new measurement and returns the filtered data
		 * @param new_measurement	The new data to be filtered, in ROS message form
		 * @return 					The filtered measurement as a ROS message
		 */
		T filter(const T new_measurement);

		/**
		 * Sets the filter to a desired value
		 * @param reset_value	Resets the filter to match this ROS message
		 */
		void reset(const T reset_value);

	private:
		BasicLowPassMultiFilter* multifilter_;
	};

} //end nrg_tools namespace