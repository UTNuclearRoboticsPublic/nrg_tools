# nrg_tools
This is a C++ library for a bunch of useful tools I have used enough to turn into a library. There are a few broad categories of tool in the toolkit here. Read about them below:

1. [Type Conversions](#type-conversions)
2. [Low Pass Filters](#low-pass-filters)
3. [Printing](#printing)

# Usage
To use this package, download it into your `src` directory.
Add the line `<depend>nrg_tools</depend>` to a package's `package.xml` file where you would like to use the library. Also add `nrg_tools` to the `find_package` call in the `CMakeLists.txt` file of your package.

Now you should be able to `#include "nrg_tools.h"` in your C++ file where you want to use the library. 

The `nrg_tools.h` file includes all of the tools in the library. To get only specific ones, you can include only those headers instead (e.g. `#include "conversions.hpp"`).

The documentation for the library can be easily accessed by opening the `index.html` file in the `docs/html` folder, [here](https://github.com/UTNuclearRoboticsPublic/nrg_tools/tree/master/doc/html).

## Type Conversions
The type conversions ([conversions.hpp](https://github.com/UTNuclearRoboticsPublic/nrg_tools/blob/master/include/nrg_tools/conversions.hpp)) allow you to easily convert between many ROS message types, `std::vector<double>`, and `Eigen::VectorXd`. To convert between any two types is simpily a call to `nrg_tools::convert(from_type, to_type)`. As an example:
```
std::vector<double> input{100, 200, 300, 400, 500, 600};
geometry_msgs::WrenchStamped output;
bool success = nrg_tools::convert(input, output);

geometry_msgs::Wrench other_output;
success = nrg_tools::convert(output, other_output);
```

Adding new possible conversions to the library is as simple as writing 2 functions: the first to convert your new type to a `std::vector<double>` and the secont to convert a `std::vector<double>` to your new type. After doing this, `nrg_tools::convert()` will work on any other types with your new addition. This process is further documented in the actual header file.

## Low Pass Filters
### Standard Filters
The `BasicLowPassFilter` and `BasicLowPassMultiFilter` implement low-pass filters with no ROS components. A filter coefficient must be given (for each filter in the Multi Filter case). This value should be on the order of `~1-10`, recommended starting value is `2`.  The Multi Filter is used for vector's that are all updated at the same time, such as joint states or velocity commands. Example usage is:
```
nrg_tools::BasicLowPassFilter basic_filter(2);
double output = basic_filter.filter(100.5);
output = basic_filter.filter(101.3);
output = basic_filter.filter(100.9);

std::vector<double> coefficients{2, 1.2, 8};
std::vector<double> init_values{0, -3.14, 42};
nrg_tools::BasicLowPassMultiFilter multi_filter(coefficients, init_values);
init_values[1] = -3.0;
std::vector<double> output = multi_filter.filter(init_values);
init_values[1] = -2.4;
output = multi_filter.filter(init_values);
```
### ROS Filters
The `RosLowPassFilter` is a ROS wrapper for a `BasicLowPassMultiFilter` that allows you to filter message types. This might be useful when subscribed to a sensor and you want to filter in the same message type as the sensor. Basic usage is pretty straightforward:
```
std::vector<double> filter_coeffs{2, 2, 2, 2, 2, 2};
geometry_msgs::Wrench coeffs;
nrg_tools::convert(filter_coeffs, coeffs);
nrg_tools::RosLowPassFilter<geometry_msgs::Wrench> ros_filter(coeffs);
geometry_msgs::Wrench filtered_result = ros_filter.filter(some_wrench);
```

## Printing
Some additional functionality is provided for printing certain types. This is probably most useful for debugging, and to clean up ROS_INFO outputs. Usage is simply:
```
std::vector<double> some_vector{2.2, 2.4, 2.6};
std::string pretty_string = nrg_tools::getStr(some_vector)

std::vector<std::string> some_other_vector{"Hello", "World", "!"};
std::string pretty_other_string = nrg_tools::getStr(some_other_vector)
```