#include "rtabmap_ros/OccupancyGridBuilderWrapper.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "occupancy_grid_builder");

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	// process "--params" argument
	for(int i = 1; i < argc; ++i)
	{
		if(strcmp(argv[i], "--params") == 0)
		{
			rtabmap::ParametersMap parameters;
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
			uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
			for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				std::string str = "Param: " + iter->first + " = \"" + iter->second + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						rtabmap::Parameters::getDescription(iter->first).c_str() <<
						"]" <<
						std::endl;
			}
			ROS_WARN("Node will now exit after showing default parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}
		else if(strcmp(argv[i], "--udebug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
		}
		else if(strcmp(argv[i], "--uinfo") == 0)
		{
			ULogger::setLevel(ULogger::kInfo);
		}
	}

	rtabmap_ros::OccupancyGridBuilder occupancy_grid_builder(argc, argv);
	ros::spin();
	return 0;
}
