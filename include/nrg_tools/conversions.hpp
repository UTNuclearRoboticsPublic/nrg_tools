#pragma once

/**
 * Handles conversions of ROS message types to other messages, std::vector's, or Eigen::Vector's
 */

#include "ros_msgs_includes.h"
#include <Eigen/Eigen>

// This file consists of 3 main sections:
// 1. A list of functions that converts messages to std::vector's
// 2. A list of functions that converts std::vector's to message types
// 3. A template function to convert any 2 types (thru a std::vector)

// To add a message type to the available conversions, ALL YOU NEED TO DO is:
// 1. Write a function (following the same form as the others in section 1) that
// 			converts your type to a std::vector.
// 2. Write a function (following the same form as the othersin section 2) that
// 			converts a std::vector into your type
// 3. If you copy the comments too, the documentation should be easily updated

namespace nrg_conversions{

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~ CONVERSIONS TO STD::VECTOR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
	 * Trivial case for std::vector to std::vector
	 * @param input		A std::vector<double> input
	 * @return 			Returns the same input
	 */
	const std::vector<double> toVec(const std::vector<double> input)
	{
		return input;
	}

	/**
	 * Converts geometry_msgs::Vector3 to std::vector<double>
	 * @param input		A geometry_msgs::Vector3 input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Vector3 input)
	{
		std::vector<double> output{input.x, input.y, input.z};
		return output;
	}

	/**
	 * Converts geometry_msgs::Quaternion to std::vector<double>
	 * @param input		A geometry_msgs::Quaternion input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Quaternion input)
	{
		std::vector<double> output{input.x, input.y, input.z, input.w};
		return output;
	}

	/**
	 * Converts geometry_msgs::Accel to std::vector<double>
	 * @param input		A geometry_msgs::Accel input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Accel input)
	{
		const geometry_msgs::Vector3 linear = input.linear;
		const geometry_msgs::Vector3 angular = input.angular;

		const std::vector<double> lin_std_vec = toVec(linear);
		const std::vector<double> ang_std_vec = toVec(angular);

		std::vector<double> output = lin_std_vec;
		output.insert(output.end(), ang_std_vec.begin(), ang_std_vec.end());
		return output;
	}

	/**
	 * Converts geometry_msgs::AccelStamped to std::vector<double>
	 * @param input		A geometry_msgs::AccelStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::AccelStamped input)
	{
		const geometry_msgs::Accel accel = input.accel;
		return toVec(accel);
	}

	/**
	 * Converts Eigen::VectorXd to std::vector<double>
	 * @param input		A Eigen::VectorXd input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const Eigen::VectorXd input)
	{
		std::vector<double> output;
		for(int i=0; i<input.size(); i++)
		{
			output.push_back(input[i]);
		}
		return output;
	}

	/**
	 * Converts geometry_msgs::Point to std::vector<double>
	 * @param input		A geometry_msgs::Point input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Point input)
	{
		std::vector<double> output{input.x, input.y, input.z};
		return output;
	}

	/**
	 * Converts geometry_msgs::Point32 to std::vector<double>
	 * @param input		A geometry_msgs::Point32 input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Point32 input)
	{
		std::vector<double> output{input.x, input.y, input.z};
		return output;
	}

	/**
	 * Converts geometry_msgs::PointStamped to std::vector<double>
	 * @param input		A geometry_msgs::PointStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::PointStamped input)
	{
		const geometry_msgs::Point point = input.point;
		return toVec(point);
	}

	/**
	 * Converts geometry_msgs::Polygon to std::vector<double> point-by-point
	 * such that the 4th element of the output is the second point's X (first) value
	 * @param input		A geometry_msgs::Polygon input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Polygon input)
	{
		size_t num_points = input.points.size();
		std::vector<double> output;
		for(size_t i=0; i<num_points; ++i)
		{
			output.push_back(input.points[i].x);
			output.push_back(input.points[i].y);
			output.push_back(input.points[i].z);
		}
		return output;
	}

	/**
	 * Converts geometry_msgs::PolygonStamped to std::vector<double> point-by-point
	 * such that the 4th element of the output is the second point's X (first) value
	 * @param input		A geometry_msgs::PolygonStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::PolygonStamped input)
	{
		const geometry_msgs::Polygon poly = input.polygon;
		return toVec(poly);
	}

	/**
	 * Converts geometry_msgs::Pose to std::vector<double>
	 * @param input		A geometry_msgs::Pose input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Pose input)
	{
		const geometry_msgs::Point point = input.position;
		const geometry_msgs::Quaternion quat = input.orientation;

		const std::vector<double> v1 = toVec(point);
		const std::vector<double> v2 = toVec(quat);
		std::vector<double> output = v1;
		output.insert(output.end(), v2.begin(), v2.end());
		return output;
	}

	/**
	 * Converts geometry_msgs::Pose2D to std::vector<double>
	 * @param input		A geometry_msgs::Pose2D input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Pose2D input)
	{
		std::vector<double> output{input.x, input.y, input.theta};
		return output;
	}

	/**
	 * Converts geometry_msgs::PoseStamped to std::vector<double>
	 * @param input		A geometry_msgs::PoseStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::PoseStamped input)
	{
		const geometry_msgs::Pose pose = input.pose;
		return toVec(pose);
	}

	/**
	 * Converts tf::Quaternion to std::vector<double>
	 * @param input		A tf::Quaternion input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const tf::Quaternion input)
	{
		std::vector<double> output{input[0], input[1], input[2], input[3]};
		return output;
	}

	/**
	 * Converts tf2::Quaternion to std::vector<double>
	 * @param input		A tf2::Quaternion input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const tf2::Quaternion input)
	{
		std::vector<double> output{input[0], input[1], input[2], input[3]};
		return output;
	}

	/**
	 * Converts geometry_msgs::QuaternionStamped to std::vector<double>
	 * @param input		A geometry_msgs::QuaternionStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::QuaternionStamped input)
	{
		const geometry_msgs::Quaternion quat = input.quaternion;
		return toVec(quat);
	}

	/**
	 * Converts geometry_msgs::Transform to std::vector<double>
	 * @param input		A geometry_msgs::Transform input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Transform input)
	{
		const geometry_msgs::Vector3 vec = input.translation;
		const geometry_msgs::Quaternion quat = input.rotation;

		const std::vector<double> v1 = toVec(vec);
		const std::vector<double> v2 = toVec(quat);
		std::vector<double> output = v1;
		output.insert(output.end(), v2.begin(), v2.end());
		return output;
	}

	/**
	 * Converts geometry_msgs::TransformStamped to std::vector<double>
	 * @param input		A geometry_msgs::TransformStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::TransformStamped input)
	{
		const geometry_msgs::Transform tran = input.transform;
		return toVec(tran);
	}

	/**
	 * Converts geometry_msgs::Twist to std::vector<double>
	 * @param input		A geometry_msgs::Twist input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Twist input)
	{
		const geometry_msgs::Vector3 vec1 = input.linear;
		const geometry_msgs::Vector3 vec2 = input.angular;

		const std::vector<double> v1 = toVec(vec1);
		const std::vector<double> v2 = toVec(vec2);
		std::vector<double> output = v1;
		output.insert(output.end(), v2.begin(), v2.end());
		return output;
	}

	/**
	 * Converts geometry_msgs::TwistStamped to std::vector<double>
	 * @param input		A geometry_msgs::TwistStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::TwistStamped input)
	{
		const geometry_msgs::Twist twist = input.twist;
		return toVec(twist);
	}

	/**
	 * Converts tf::Vector3 to std::vector<double>
	 * @param input		A tf::Vector3 input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const tf::Vector3 input)
	{
		std::vector<double> output{input.getX(), input.getY(), input.getZ()};
		return output;
	}

	/**
	 * Converts tf2::Vector3 to std::vector<double>
	 * @param input		A tf2::Vector3 input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const tf2::Vector3 input)
	{
		std::vector<double> output{input.getX(), input.getY(), input.getZ()};
		return output;
	}

	/**
	 * Converts geometry_msgs::Vector3Stamped to std::vector<double>
	 * @param input		A geometry_msgs::Vector3Stamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Vector3Stamped input)
	{
		const geometry_msgs::Vector3 vec = input.vector;
		return toVec(vec);
	}

	/**
	 * Converts geometry_msgs::Wrench to std::vector<double>
	 * @param input		A geometry_msgs::Wrench input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::Wrench input)
	{
		const geometry_msgs::Vector3 vec1 = input.force;
		const geometry_msgs::Vector3 vec2 = input.torque;

		const std::vector<double> v1 = toVec(vec1);
		const std::vector<double> v2 = toVec(vec2);
		std::vector<double> output = v1;
		output.insert(output.end(), v2.begin(), v2.end());
		return output;
	}

	/**
	 * Converts geometry_msgs::WrenchStamped to std::vector<double>
	 * @param input		A geometry_msgs::WrenchStamped input
	 * @return 			A std::vector<double> that matches the input
	 */
	const std::vector<double> toVec(const geometry_msgs::WrenchStamped input)
	{
		const geometry_msgs::Wrench wrench = input.wrench;
		return toVec(wrench);
	}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~ CONVERSIONS TO OTHER TYPE ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	/**
	 * Trivial case for converting std::vector to std::vector
	 * @param input		A std::vector<double> input
	 * @param output	The same std::vector<double> as output
	 * @return 			Always true for the trivial case
	 */
	const bool fromVec(const std::vector<double>& input, std::vector<double>& output)
	{
		output = input;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Vector3
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Vector3 that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Vector3& output)
	{
		if(input.size() != 3) return false;
		geometry_msgs::Vector3 vec3;
		vec3.x = input[0]; vec3.y = input[1]; vec3.z = input[2];
		output = vec3;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Quaternion
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Quaternion that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Quaternion& output)
	{
		if(input.size() != 4) return false;
		geometry_msgs::Quaternion quat;
		quat.x = input[0]; quat.y = input[1]; quat.z = input[2]; quat.w = input[2];
		output = quat;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Accel
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Accel that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Accel& output)
	{
		if(input.size() != 6) return false;
		geometry_msgs::Accel accel;
		accel.linear.x = input[0]; accel.linear.y = input[1]; accel.linear.z = input[2];
		accel.angular.x = input[3]; accel.angular.y = input[4]; accel.angular.z = input[5];
		output = accel;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::AccelStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::AccelStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::AccelStamped& output)
	{
		geometry_msgs::Accel accel;
		const bool val = fromVec(input, accel);
		output.accel = accel;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a Eigen::VectorXd
	 * @param input		A std::vector<double> input
	 * @param output	A Eigen::VectorXd that matches the input
	 * @return 			Returns 'true'
	 */
	const bool fromVec(const std::vector<double>& input, Eigen::VectorXd& output)
	{
		Eigen::VectorXd vec(input.size());
		for(int i=0; i<input.size(); i++)
		{
			vec[i] = input[i];
		}
		output = vec;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Point
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Point that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Point& output)
	{
		if(input.size() != 3) return false;
		geometry_msgs::Point point;
		point.x = input[0]; point.y = input[1]; point.z = input[2];
		output = point;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Point32
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Point32 that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Point32& output)
	{
		if(input.size() != 3) return false;
		geometry_msgs::Point32 point;
		point.x = input[0]; point.y = input[1]; point.z = input[2];
		output = point;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::PointStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::PointStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::PointStamped& output)
	{
		geometry_msgs::Point point;
		const bool val = fromVec(input, point);
		output.point = point;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Polygon point-by-point
	 * such that the first 3 elements of the input are the first Point,
	 * the next 3 elements are the second Point, etc
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Polygon that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Polygon& output)
	{
		if(input.size() % 3 != 0) return false;
		geometry_msgs::Polygon poly;
		geometry_msgs::Point32 point;

		size_t num_points = input.size() / 3;
		for(size_t i=0; i<num_points; ++i)
		{
			point.x = input[3*i];
			point.y = input[3*i+1];
			point.z = input[3*i+2];
			poly.points.push_back(point);
		}

		output = poly;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Polygon point-by-point
	 * such that the first 3 elements of the input are the first Point,
	 * the next 3 elements are the second Point, etc
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::PolygonStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::PolygonStamped& output)
	{
		geometry_msgs::Polygon poly;
		const bool val = fromVec(input, poly);
		output.polygon = poly;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Pose
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Pose that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Pose& output)
	{
		if(input.size() != 7) return false;
		geometry_msgs::Point point;
		geometry_msgs::Quaternion quat;
		
		point.x = input[0]; point.y = input[1]; point.z = input[2];
		quat.x = input[3]; quat.y = input[4]; quat.z = input[5]; quat.w = input[6];

		output.position = point;
		output.orientation = quat;
		
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Pose2D
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Pose2D that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Pose2D& output)
	{
		if(input.size() != 3) return false;
		geometry_msgs::Pose2D pose;
		pose.x = input[0]; pose.y = input[1]; pose.theta = input[2];
		output = pose;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::PoseStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::PoseStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::PoseStamped& output)
	{
		geometry_msgs::Pose pose;
		const bool val = fromVec(input, pose);
		output.pose = pose;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a tf::Quaternion
	 * @param input		A std::vector<double> input
	 * @param output	A tf::Quaternion that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, tf::Quaternion& output)
	{
		if(input.size() != 4) return false;
		tf::Quaternion quat(input[0], input[1], input[2], input[3]);
		output = quat;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a tf2::Quaternion
	 * @param input		A std::vector<double> input
	 * @param output	A tf2::Quaternion that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, tf2::Quaternion& output)
	{
		if(input.size() != 4) return false;
		tf2::Quaternion quat(input[0], input[1], input[2], input[3]);
		output = quat;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::QuaternionStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::QuaternionStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::QuaternionStamped& output)
	{
		geometry_msgs::Quaternion quat;
		const bool val = fromVec(input, quat);
		output.quaternion = quat;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Transform
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Transform that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Transform& output)
	{
		if(input.size() != 7) return false;
		geometry_msgs::Vector3 vec;
		geometry_msgs::Quaternion quat;
		
		vec.x = input[0]; vec.y = input[1]; vec.z = input[2];
		quat.x = input[3]; quat.y = input[4]; quat.z = input[5]; quat.w = input[6];

		output.translation = vec;
		output.rotation = quat;
		
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::TransformStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::TransformStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::TransformStamped& output)
	{
		geometry_msgs::Transform tran;
		const bool val = fromVec(input, tran);
		output.transform = tran;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Twist
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Twist that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Twist& output)
	{
		if(input.size() != 6) return false;
		geometry_msgs::Twist twist;
		twist.linear.x = input[0]; twist.linear.y = input[1]; twist.linear.z = input[2];
		twist.angular.x = input[3]; twist.angular.y = input[4]; twist.angular.z = input[5];

		output = twist;
		
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::TwistStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::TwistStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::TwistStamped& output)
	{
		geometry_msgs::Twist twist;
		const bool val = fromVec(input, twist);
		output.twist = twist;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a tf::Vector3
	 * @param input		A std::vector<double> input
	 * @param output	A tf::Vector3 that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, tf::Vector3& output)
	{
		if(input.size() != 3) return false;
		tf::Vector3 vec(input[0], input[1], input[2]);
		output = vec;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a tf2::Vector3
	 * @param input		A std::vector<double> input
	 * @param output	A tf2::Vector3 that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, tf2::Vector3& output)
	{
		if(input.size() != 3) return false;
		tf2::Vector3 vec(input[0], input[1], input[2]);
		output = vec;
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Vector3Stamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Vector3Stamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Vector3Stamped& output)
	{
		geometry_msgs::Vector3 vec;
		const bool val = fromVec(input, vec);
		output.vector = vec;
		
		return val;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::Wrench
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::Wrench that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::Wrench& output)
	{
		if(input.size() != 6) return false;
		geometry_msgs::Wrench wrench;
		wrench.force.x = input[0]; wrench.force.y = input[1]; wrench.force.z = input[2];
		wrench.torque.x = input[3]; wrench.torque.y = input[4]; wrench.torque.z = input[5];

		output = wrench;
		
		return true;
	}

	/**
	 * Converts a std::vector<double> into a geometry_msgs::WrenchStamped
	 * @param input		A std::vector<double> input
	 * @param output	A geometry_msgs::WrenchStamped that matches the input
	 * @return 			Returns 'true' if the input is adequately sized, 'false' otherwise
	 */
	const bool fromVec(const std::vector<double>& input, geometry_msgs::WrenchStamped& output)
	{
		geometry_msgs::Wrench wrench;
		const bool val = fromVec(input, wrench);
		output.wrench = wrench;
		
		return val;
	}
} // end nrg_conversions namespace
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~ TEMPLATE FOR CONVERT() ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
namespace nrg_tools{
	/**
	 * Converts ROS Messages, std::vector's, and Eigen::Vector's between each other
	 * by converting to and from a std::vector
	 * @param a		The onject to convert from
	 * @param b 	The converted object
	 * @return 		Returns 'true' if the conversion was successful, 'false' otherwise
	 */
	template <class T, class U> const bool convert (const T &a, U &b)
	{
		return nrg_conversions::fromVec(nrg_conversions::toVec(a), b);
	}

} // end nrg_tools namespace