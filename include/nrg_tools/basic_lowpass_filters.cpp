#pragma once

#include <basic_lowpass_filters.h>
using namespace nrg_tools;

BasicLowPassFilter::BasicLowPassFilter(double filter_coefficient, double init_value)
{
	filter_coeff_ = filter_coefficient;
	previous_measurements_[0] = init_value;
	previous_measurements_[1] = init_value;
	previous_filtered_measurement_ = init_value;	
}

double BasicLowPassFilter::filter(const double new_measurement)
{
	// Push in the new measurement
	previous_measurements_[1] = previous_measurements_[0];
	previous_measurements_[0] = new_measurement;

	double new_filtered_msrmt = (1. / (1. + filter_coeff_)) * (previous_measurements_[1] + previous_measurements_[0] -
                                 (-filter_coeff_ + 1.) * previous_filtered_measurement_);

	// Store the new filtered measurement
	previous_filtered_measurement_ = new_filtered_msrmt;

	return new_filtered_msrmt;
}

void BasicLowPassFilter::reset(const double reset_value)
{
	previous_measurements_[0] = reset_value;
	previous_measurements_[1] = reset_value;

	previous_filtered_measurement_ = reset_value;	
}

// ~~~~~~~~~~~~~ Multi Filter ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
BasicLowPassMultiFilter::BasicLowPassMultiFilter(std::vector<double> filter_coefficients, std::vector<double> init_values)
{
	num_filters_ = init_values.size();
	for(size_t i = 0; i<num_filters_; ++i)
	{
		filters_.push_back(BasicLowPassFilter(filter_coefficients[i], init_values[i]));
	}
}

std::vector<double> BasicLowPassMultiFilter::filter(const std::vector<double>& new_measurements)
{
	if(new_measurements.size() != num_filters_)
	{
		throw std::out_of_range("New Measurement vector must be same size as the number of filters");
	}
	std::vector<double> output;
	for(size_t i = 0; i<num_filters_; ++i)
	{
		output.push_back(filters_[i].filter(new_measurements[i]));
	}
	return output;
}

void BasicLowPassMultiFilter::reset(const std::vector<double>& reset_values)
{
	if(reset_values.size() != num_filters_)
	{
		throw std::out_of_range("Reset Values vector must be same size as the number of filters");
	}
	for(size_t i = 0; i<num_filters_; ++i)
	{
		filters_[i].reset(reset_values[i]);
	}
}

void BasicLowPassMultiFilter::reset(const int index, const double reset_value)
{
	if(index >= num_filters_)
	{
		throw std::out_of_range("Given index is higher than the number of filters");
	}
	filters_[index].reset(reset_value);
}