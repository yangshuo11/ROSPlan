/* 
 * Copyright [2018] <King's College London (KCL)>  
 * 
 * Author: Michael Cashmore (michael.cashmore at kcl.ac.uk)
 * Contributor: Oscar Lima (olima_84@yahoo.com)
 * 
 * Base class interface to call (task) planners.
 * 
 */

#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"

namespace KCL_rosplan {

PlannerInterface::PlannerInterface(): nh_("~"), use_problem_topic_(false),
	problem_instance_received_(false), problem_instance_time_(-1.0),
	plan_server_(nh_, "start_planning",
	boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false) {

	// publishing raw planner output
	std::string plannerTopic;
	nh_.param<std::string>("planner_topic", plannerTopic, "planner_output");
	plan_publisher_ = nh_.advertise<std_msgs::String>(plannerTopic, 1, true);

	// subscribe to problem instance
	std::string problemTopic;
	nh_.param<std::string>("problem_topic", problemTopic, "problem_instance");
	problem_sub_ = nh_.subscribe(problemTopic, 1, &PlannerInterface::problemCallback, this);

	// start the planning services
	ps_srv_ = nh_.advertiseService("planning_server",
				&KCL_rosplan::PlannerInterface::runPlanningServerDefault, this);

	ps_param_srv_ = nh_.advertiseService("planning_server_params",
				&KCL_rosplan::PlannerInterface::runPlanningServerParams, this);

	// start planning action server
	plan_server_.start();

	ROS_INFO("Ready to receive problem instance");
}

void PlannerInterface::problemCallback(const std_msgs::String::ConstPtr& problemInstance) {
	ROS_INFO("Problem received");
	problem_instance_received_ = true;
	problem_instance_time_ = ros::WallTime::now().toSec();
	problem_instance_ = problemInstance->data;
}

bool PlannerInterface::runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

	// load params from param server, use defaults if not found
	nh_.param<bool>("use_problem_topic", use_problem_topic_, false);
	nh_.param<std::string>("domain_path", domain_path_, "common/");
	nh_.param<std::string>("data_path", data_path_, "common/domain.pddl");
	nh_.param<std::string>("problem_path", problem_path_, "common/problem.pddl");
	nh_.param<std::string>("planner_command", planner_command_, "timeout 60 common/bin/popf -n DOMAIN PROBLEM");

	// call planning server
	processRequest(domain_path_, problem_path_, data_path_, planner_command_, use_problem_topic_);
	return true;
}

bool PlannerInterface::runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res) {
	// call planning server
	res.plan_found = processRequest(req.domain_path, req.problem_path, req.data_path, req.planner_command, req.use_problem_topic);
	return true;
}

void PlannerInterface::runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal) {
	// call planning server
	if(processRequest(goal->domain_path, goal->problem_path, goal->data_path, goal->planner_command, goal->use_problem_topic)) {
		plan_server_.setSucceeded();
	} else {
		plan_server_.setAborted();
	}
}

std::string PlannerInterface::runCommand(std::string cmd) {
	std::string data;
	FILE *stream;
	char buffer[1000];
	// run cmd, save output on stream
	stream = popen(cmd.c_str(), "r");
	// write stream in string data
	while ( fgets(buffer, 1000, stream) != NULL )
		data.append(buffer);
	pclose(stream);
	
	// return command terminal output as string
	return data;
}

bool PlannerInterface::processRequest(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand, bool useProblemTopic) {

	// save parameters
	data_path_ = dataPath;
	domain_path_ = domainPath;
	problem_path_ = problemPath;
	planner_command_ = plannerCommand;
	use_problem_topic_ = useProblemTopic;
	
	// set problem name for ROS_INFO, extract from problem_path_
	std::size_t lastDivide = problem_path_.find_last_of("/\\");
	if(lastDivide != std::string::npos) {
		problem_name_ = problem_path_.substr(lastDivide + 1);
	} else {
		problem_name_ = problem_path_;
	}

	if(use_problem_topic_ && !problem_instance_received_) {
		ROS_INFO("Problem (%s) was not published yet.", problem_name_.c_str());
		return false;
	}

	// lower flag
	problem_instance_received_ = false;

	// save problem to file for planner
	if(use_problem_topic_ && problem_instance_received_) {
		ROS_INFO("Writing problem (%s) to file.", problem_name_.c_str());
		std::ofstream dest;
		dest.open((problem_path_).c_str());
		dest << problem_instance_;
		dest.close();
	}

	// prepare the planner command line
	std::string str = planner_command_;
	std::size_t dit = str.find("DOMAIN");
	if(dit!=std::string::npos) str.replace(dit, 6, domain_path_); // 6 = length of string: DOMAIN
	std::size_t pit = str.find("PROBLEM");
	if(pit!=std::string::npos) str.replace(pit, 7, problem_path_); // 7 = length of string: PROBLEM
	std::string commandString = str + " > " + data_path_ + "plan.pddl";

	// call the planner
	ROS_INFO("(%s) Running command: %s", problem_name_.c_str(), commandString.c_str());
	std::string plan = runCommand(commandString.c_str()); // helper method from base class
	ROS_INFO("(%s) Planning complete", problem_name_.c_str());

	// prevent previous plan to interfere
	planner_output_.clear();

	// parse planner output, check if problem was solved and store plan steps as string in member variable "planner_output_"
	bool solved = parsePlanOutput(plan); // this method is virtual, needs to be implemented in derived class

	if(!solved)
		ROS_INFO("(%s) Planning problem was unsolvable.", problem_name_.c_str());
	else
		ROS_INFO("(%s) Planning problem was solved.", problem_name_.c_str());

	if (planner_output_.c_str() == "")
		ROS_WARN("planner responded with empty string");

	// publish clean planner output steps (still string at this point)
	if(solved) {
		std_msgs::String planMsg;
		planMsg.data = planner_output_;
		plan_publisher_.publish(planMsg);
	}

	return solved;

} } // close namespace
