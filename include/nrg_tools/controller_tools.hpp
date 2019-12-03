#pragma once

/**
 * General tools useful for robot control such as joint limit enforcing, etc
 */

#include "conversions.hpp"

namespace nrg_tools{

	/**
	 * Restricts a certain element to within the given bounds
	 * @param n 		The number to bound
	 * @param lower 	The lower limit
	 * @param upper 	The upper limit
	 * @return 			The bounded number
	 */
	template <class T> T bound (const T& n, const T& lower, const T& upper)
	{
		return std::max(lower, std::min(n, upper));
	}

	/**
	 * Restricts an array/message/etc to the given bounds. The bounds are
	 * allowed to be a different type as long as they have the same
	 * number of elements
	 * @param input		The input object to bound, must have valid conversion functions
	 * @param lower 	The lower limits, element-wise on the input
	 * @param upper 	The upper limits, element-wise on the input
	 * @return 			The bounded object, of same type as the input
	 */
	template <class T, class U> T boundAll (const T& input, const U& lower, const U& upper)
	{
		std::vector<double> input_vector, upper_vector, lower_vector, output_vector;
		convert(input, input_vector);
		convert(lower, lower_vector);
		convert(upper, upper_vector);
		if(input_vector.size() != lower_vector.size() 
			|| input_vector.size() != upper_vector.size())
		{
			throw std::invalid_argument("Input and bound sizes do not match");
			return input;
		}
		for(size_t i=0; i<input_vector.size(); ++i)
		{
			output_vector.push_back(bound(input_vector[i], lower_vector[i], upper_vector[i]));
		}
		T output;
		nrg_conversions::fromVec(output_vector, output);
		return output;
	}

	/**
	 * Restricts an array/message/etc to the given bounds by uniformly scaling
	 * the object until all elements are within bounds. The bounds are
	 * allowed to be a different type as long as they have the same number of elements.
	 * Because it scales the array, the limits considered to be symmetric around 0
	 * @param input		The input object to bound, must have valid conversion functions
	 * @param limit 	The (plus and minus) limit, element-wise on the input
	 * @return 			The bounded object, of same type as the input
	 */
	template <class T, class U> T boundUniform (const T& input, const U& limit)
	{
		Eigen::VectorXd input_vector, limit_vector;
		convert(input, input_vector);
		convert(limit, limit_vector);
		if(input_vector.size() != limit_vector.size())
		{
			throw std::invalid_argument("Input and limit sizes do not match");
			return input;
		}
		double min_multiplier = 1;
		for(size_t i=0; i<input_vector.size(); ++i)
		{
			if(fabs(input_vector(i)) > fabs(limit_vector(i)))
			{
				// We need to scale the input down
				min_multiplier = std::min(min_multiplier, (fabs(limit_vector(i))/fabs(input_vector(i))));
			}
		}
		input_vector *= min_multiplier;
		T output;
		convert(input_vector, output);
		return output;
	}

} // end nrg_tools namespace