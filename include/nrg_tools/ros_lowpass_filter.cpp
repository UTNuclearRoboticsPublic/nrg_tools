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
template<typename T>
class RosLowPassFilter
{
public:
	/**
	 * Constructor
	 * @param filter_coefficients	The coefficients of the filters. Must be the message type you want to filter later. Higher values = more smoothing but more lag
	 */
	RosLowPassFilter<T>(T filter_coefficients);

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

template<typename T>
RosLowPassFilter<T>::RosLowPassFilter(T filter_coefficients)
{
	// Convert to std::vector's
	std::vector<double> coeff_vector, inital_vector;
	convert(filter_coefficients, coeff_vector);

	// Feed into a basic multi filter
	inital_vector.assign(coeff_vector.size(), 0.0);
	multifilter_ = new BasicLowPassMultiFilter(coeff_vector, inital_vector);
}

template<typename T>
T RosLowPassFilter<T>::filter(const T new_measurement)
{
	// Convert to a std::vector and feed into multi filter
	std::vector<double> measurement_data, filtered_data;
	convert(new_measurement, measurement_data);
	filtered_data = multifilter_->filter(measurement_data);

	// Convert to the output type and return
	T output;
	convert(filtered_data, output);
	return output;
}

template<typename T>
void RosLowPassFilter<T>::reset(const T reset_value)
{
	// Convert to a std::vector and feed into multi filter
	std::vector<double> reset_vector;
	convert(reset_value, reset_vector);
	multifilter_->reset(reset_vector);
}

} // end nrg_tools namespace