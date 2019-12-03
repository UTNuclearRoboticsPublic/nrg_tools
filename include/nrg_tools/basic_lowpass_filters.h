#pragma once

namespace nrg_tools{
	/** 
	 * \class BasicLowPassFilter
	 * A Low Pass Filter for a single value, with no ROS capabilities
	 */
	class BasicLowPassFilter
	{
	public:
		/**
		 * Constructor
		 * @param filter_coefficient		Higher = more smoothing, but also more lag in the data. Reccomended default = 2
		 * @param init_value				The starting value of the filter
		 */
		BasicLowPassFilter(double filter_coefficient, double init_value=0);

		/**
		 * Updates the filter with the new measurement and returns the filtered data
		 * @param new_measurement	The new data to be filtered
		 * @return 					The filtered measurement after accounting for the newest data
		 */
		double filter(const double new_measurement);

		/**
		 * Sets the filter to a desired value
		 * @param reset_value	The value to set the filter to
		 */
		void reset(const double reset_value);

	private:
		double previous_measurements_[2] = {0.0, 0.0};
		double previous_filtered_measurement_ = 0.0;
		double filter_coeff_ = 1.0;
	};


	/** 
	 * \class BasicLowPassMultiFilter
	 * A Low Pass Filter for a vector of values, with no ROS capabilities
	 * Filters each value in the vector seperately
	 */
	class BasicLowPassMultiFilter
	{
	public:
		/**
		 * Constructor
		 * @param filter_coefficients		A std::vector<double> of filter coefficients. Higher = more smoothing, but also more lag in the data. Reccomended default = 2
		 * @param init_values				The starting values of the filter. Make sure the length matches what you want to filter later
		 */
		BasicLowPassMultiFilter(std::vector<double> filter_coefficients, std::vector<double> init_values);

		/**
		 * Updates the filters with the new measurements and returns the filtered data as a vector
		 * @param new_measurements	The new data to be filtered
		 * @return 					The filtered measurement after accounting for the newest data
		 */
		std::vector<double> filter(const std::vector<double>& new_measurements);

		/**
		 * Sets all of the filters to the desired values
		 * @param reset_values	The values to set the filters to
		 */
		void reset(const std::vector<double>& reset_values);

		/**
		 * Sets the filter at the desired index to a desired value
		 * Throws an error if the index is out of range
		 * @param index			The index of the filter to change
		 * @param reset_value	The value to set the filter to
		 */
		void reset(const int index, const double reset_value);

		/**
		 * Gets the number of filters this multi filter is tracking
		 * @return		The number of filters
		 */
		size_t getNumberFilters(){return num_filters_;};

	private:
		size_t num_filters_ = 0;
		std::vector<BasicLowPassFilter> filters_;
	};


} //end nrg_tools namespace