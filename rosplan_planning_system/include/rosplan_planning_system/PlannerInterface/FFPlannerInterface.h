/* 
 * Copyright [2018] <King's College London (KCL)>  
 * 
 * Author: Michael Cashmore (michael.cashmore at kcl.ac.uk)
 * Contributor: Oscar Lima (olima_84@yahoo.com)
 * 
 * Derived class from PlannerInterface to call the ff planner.
 * 
 */

#include "PlannerInterface.h"
#include <string>

#ifndef KCL_FF_planner_interface
#define KCL_FF_planner_interface

namespace KCL_rosplan {

	class FFPlannerInterface: public PlannerInterface
	{

		// no need for constructor

	  private:

		/**
		 * @brief process planner output, identify if planner found solution and if so then filter out actions,
		 *        transform to lowercase and save in member variable "planner_output_"
		 * @param contingentFF, true = contingentFF, false = metricFF
		 * @return true if planner found solution, false if planner did not found solution
		 */
		bool parsePlanOutput(std::string& plan);
	};

} // close namespace

#endif
