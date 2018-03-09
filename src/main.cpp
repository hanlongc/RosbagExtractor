#include <iostream>

#include "BagExtractor.h"

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		std::cout << "Usage: rosrun RosbagExtractor extract <path_to_bag_file>" << std::endl;
		return -1;
	}//end if

	ros::init(argc, argv, "extract");

	BagExtractor be(argv[1]);
	be.Extract();

	return 0;
}
