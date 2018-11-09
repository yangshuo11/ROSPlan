/* 
 * Copyright [2018] <King's College London (KCL)>  
 * 
 * Author: Michael Cashmore (michael.cashmore at kcl.ac.uk)
 * Contributor: Oscar Lima (olima_84@yahoo.com)
 * 
 * Base class interface to call (task) planners.
 * 
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <rosplan_dispatch_msgs/PlanningService.h>
#include <rosplan_dispatch_msgs/PlanAction.h>

#include <fstream>

#ifndef KCL_planner_interface
#define KCL_planner_interface

namespace KCL_rosplan {

	class PlannerInterface // this is a base class, a derived class must be created
	{
	  private:
		// problem subscription ; publish output
		ros::NodeHandle nh_;
		ros::Subscriber problem_sub_;
		ros::Publisher plan_publisher_;

		// services to call the planner and set parameters
		ros::ServiceServer ps_srv_; // planning service
		ros::ServiceServer ps_param_srv_; // planning service params service
		actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction> plan_server_;

		/**
		 * @brief callback that gets executed upon receiving problem instance message
		 * saves received problem instance in member variable "problem_instance_"
		 * @param problemInstance the string that contains the problem instance
		 */
		void problemCallback(const std_msgs::String::ConstPtr& problemInstance);

		/**
		 * @brief planning system service method (1)
		 * callback that gets executed upon receiving (empty) service call to produce plan
		 * calls processRequest function using params from param server
		 * @param req empty request, no params
		 * @param res true always
		 */
		bool runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		/**
		 * @brief planning system service method (2)
		 * callback that gets executed upon receiving service call to produce plan
		 * planner parameters are fetched from req msg (dynamic)
		 * calls processRequest function using req params
		 * @param req planner params are expected to come in req msg
		 * @param res true always
		 */
		bool runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req,
									 rosplan_dispatch_msgs::PlanningService::Response &res);

		/**
		 * @brief planning system service method (3)
		 * callback that gets executed upon receiving action lib request to make plan
		 * calls processRequest function using goal params
		 * @param goal planner params are expected to come in goal msg
		 */
		void runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal);

		/**
		 * @brief runs external commands
		 * this is a helper method that can be used by derived class
		 * @param cmd the string containing the command to run in terminal
		 * @return string console output produced by the runned command
		 */
		std::string runCommand(std::string cmd);

		/**
		 * @brief save params in member variables; ensure problem instance was received; call runPlanner
		 * @param domainPath
		 * @param problemPath
		 * @param dataPath
		 * @param plannerCommand
		 * @param useProblemTopic
		 * @return true if planner found solution, false otherwise
		 */
		bool processRequest(std::string domainPath, std::string problemPath,
							std::string dataPath, std::string plannerCommand, bool useProblemTopic);

	  protected:
		// planner params
		bool use_problem_topic_;
		std::string planner_command_;
		std::string domain_path_;
		std::string problem_path_;
		std::string problem_name_;
		std::string data_path_; // location where plan.pddl will be saved

		// planner output saved as string, is expected to be overwritten by derived
		// class to trim and leave only the actions
		std::string planner_output_;

		// flag to indicate that problem instance was received in callback
		bool problem_instance_received_;
		// to store the complete problem instance as string
		std::string problem_instance_;
		
		// stores the time at which the plan instance was received in secs
		double problem_instance_time_;

		/**
		 * @brief make plan, call the planner and process the output, remove unnecesary
		 * lines and leave only the actions (the plan)
		 * @return true if planner found solution, false otherwise
		 */
		// virtual bool runPlanner() = 0; // pure virtual method

		virtual bool parsePlanOutput(std::string& plan) = 0; // pure virtual method

	  public:

		/**
		 * @brief base class constructor
		 */
		PlannerInterface();
	};

} // close namespace

#endif
