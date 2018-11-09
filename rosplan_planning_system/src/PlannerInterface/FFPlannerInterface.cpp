/* 
 * Copyright [2018] <King's College London (KCL)>  
 * 
 * Author: Michael Cashmore (michael.cashmore at kcl.ac.uk)
 * Contributor: Oscar Lima (olima_84@yahoo.com)
 * 
 * Derived class from PlannerInterface to call the ff planner.
 * 
 */

#include "rosplan_planning_system/PlannerInterface/FFPlannerInterface.h"

namespace KCL_rosplan {

// no need for constructor

bool FFPlannerInterface::parsePlanOutput(std::string& plan) {

	bool contingentFF = true; // TODO: remove harcoded true here

	// check the planner solved the problem
	std::ifstream planfile;
	planfile.open((data_path_ + "plan.pddl").c_str());
	std::string line;

	bool solved = false;
	std::vector<std::string> filtered_plan_output;
	while (std::getline(planfile, line)) {
		// skip lines until we found evidence that a plan was found
		if (contingentFF) {
			if (line.compare("ff: found plan as follows") == 0)
				solved = true;
		}
		else {
			if (line.compare("ff: found legal plan as follows") == 0)
				solved = true;
		}

		// find string "---" inside line
		if (line.find(std::string("---")) != std::string::npos) {
			// do not add lines that have more than 3 hyphens (---)
			if (!((line.find(std::string("-------")) != std::string::npos)))
				filtered_plan_output.push_back(line);
		}

		// find end of plan
		if (line.compare("||-1") == 0)
			break;
	}

	// if solution was found parse the solved plan
	if (solved) {

		ROS_INFO("size = %lu", filtered_plan_output.size());
		for(std::string element : filtered_plan_output) {
			ROS_DEBUG("raw line: %s", element.c_str());

			// remove "0||0 --- " from element
			auto index = element.find(std::string("||0 --- "));

			// remove "--- SON:" and further
			auto index2 = element.find(std::string(" --- SON:"));

			element = std::string("(") + element.substr(index + 8, index2 - (index + 8)) + std::string(")  [0.001]\n");

			// convert to lowercase
			std::transform(element.begin(), element.end(), element.begin(), ::tolower);

			ROS_DEBUG("parsed line : %s", element.c_str() );

			planner_output_ += element;
		}
		
		// TODO: ff metric stuff was removed from here
	}
	planfile.close();

	return solved;

} } // close namespace


int main(int argc, char **argv) {
	ros::init(argc,argv,"rosplan_planner_interface");
	KCL_rosplan::FFPlannerInterface pi;

	// wait for callbacks
	ros::spin();

	return 0;
}
