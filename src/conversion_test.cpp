#include <nrg_tools.h>

int main(int argc, char **argv)
{
	geometry_msgs::Point32 p1, p2, p3;
	p1.x = 100;
	p1.y = 200;
	p1.z = 300;
	p2.x = 400;
	p2.y = 500;
	p2.z = 600;
	p3.x = 700;
	p3.y = 800;
	p3.z = 900;

	geometry_msgs::Polygon test1;
	test1.points.push_back(p1);
	test1.points.push_back(p2);
	test1.points.push_back(p3);

	geometry_msgs::PolygonStamped testSTAMP;
	testSTAMP.polygon = test1;

	std::vector<double> test2{9, 8, 7, 6, 5, 4, 3, 2, 1};

	geometry_msgs::PolygonStamped res2;
	Eigen::VectorXd res1;

	bool suc1 = nrg_tools::convert(testSTAMP, res1);
	bool suc2 = nrg_tools::convert(test2, res2);

	std::cout << "\nResult 1 (" << suc1 << "):\n" << res1 << "\n";
	std::cout << "\nResult 2 (" << suc2 << "): " << res2.polygon.points[0].x << " " << res2.polygon.points[0].y << " " << res2.polygon.points[0].z << " "
			<< res2.polygon.points[2].x << " " << res2.polygon.points[2].y << " " << res2.polygon.points[2].z << "\n";


	geometry_msgs::Twist res3;
	bool suc3 = nrg_tools::convert(test2, res3);
	std::cout << "\nResult 3 (" << suc3 << "): " << res3.linear.x << " " << res3.linear.y << " " << res3.linear.z << " "
			<< res3.angular.x << " " << res3.angular.y << " " << res3.angular.z << "\n";


	geometry_msgs::Twist res4;
	geometry_msgs::Wrench test4;
	test4.force.x = 111;
	test4.force.y = 222;
	test4.force.z = 333;
	test4.torque.x = 444;
	test4.torque.y = 555;
	test4.torque.z = 666;
	bool suc4 = nrg_tools::convert(test4, res4);
	std::cout << "\nResult 4 (" << suc4 << "): " << res4.linear.x << " " << res4.linear.y << " " << res4.linear.z << " "
			<< res4.angular.x << " " << res4.angular.y << " " << res4.angular.z << "\n";

	tf2::Vector3 tf_test1(30, 31, 32);
	tf2::Quaternion tf_test2(40,41,42,44);
	geometry_msgs::Vector3 tf_res1;
	geometry_msgs::Quaternion tf_res2;
	nrg_tools::convert(tf_test1, tf_res1);
	nrg_tools::convert(tf_test2, tf_res2);
	std::cout << "\nResult 5: " << tf_res1;
	std::cout << "\nResult 6: " << tf_res2;


	std::vector<double> print1{1.1, 2.2, 3.3};
	std::vector<int> print2{95, 96, 97, 98, 99};
	std::vector<std::string> print3{"Hello", "World", "!"};
	std::cout << "\nPrint Test 1: " << nrg_tools::getStr(print1) << ".\n";
	std::cout << "\nPrint Test 2: " << nrg_tools::getStr(print2) << ".\n";
	std::cout << "\nPrint Test 3: " << nrg_tools::getStr(print3) << ".\n";


	std::cout << "\nBounding Test 1: " << nrg_tools::bound(1000, -100, 100);
	std::cout << "\nBounding Test 2: " << nrg_tools::bound(-1000, -100, 100);
	std::cout << "\nBounding Test 3: " << nrg_tools::bound(42, -100, 100);

	std::vector<double> lower, upper;
	lower.assign(9, -5); upper.assign(9, 5);
	std::vector<double> bound_res1 = nrg_tools::boundAll(test2, lower, upper);
	std::cout << "\nBounding Test 4: " << nrg_tools::getStr(bound_res1) << ".\n";

	lower.assign(6, 200); upper.assign(6, 400);
	geometry_msgs::Wrench bound_res2 = nrg_tools::boundAll(test4, lower, upper);
	std::cout << "\nBounding Test 5: " << bound_res2;


	std::vector<double> bound_test1{-10, 10, 0, -5, -5};
	std::vector<double> bound_limit{-20, 5, -10, 1, 100};
	std::vector<double> bound_res3 = nrg_tools::boundUniform(bound_test1, bound_limit);
	std::cout << "\nBounding Test 6: " << nrg_tools::getStr(bound_res3) << ".\n";


	std::vector<double> filter_coeffs{2, 2, 2, 2, 2, 2};
	geometry_msgs::Wrench coeffs;
	nrg_tools::convert(filter_coeffs, coeffs);
	nrg_tools::RosLowPassFilter<geometry_msgs::Wrench> test_filter(coeffs);
	test_filter.reset(test4);
	test4.force.x = 200;
	geometry_msgs::Wrench filtered_result = test_filter.filter(test4);
	test4.force.x = 250;
	filtered_result = test_filter.filter(test4);
	std::cout << "\nFilter Test 1: " << filtered_result << std::endl;

	return 0;
}