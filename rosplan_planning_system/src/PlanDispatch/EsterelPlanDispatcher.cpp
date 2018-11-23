#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"


namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	EsterelPlanDispatcher::EsterelPlanDispatcher(ros::NodeHandle& nh): PlanDispatcher(nh)  {

		node_handle = &nh;

		std::string plan_graph_topic = "plan_graph";
		nh.getParam("plan_graph_topic", plan_graph_topic);
		plan_graph_publisher = node_handle->advertise<std_msgs::String>(plan_graph_topic, 1000, true);

		reset();
	}

	EsterelPlanDispatcher::~EsterelPlanDispatcher()
	{

	}

	void EsterelPlanDispatcher::reset() {
		PlanDispatcher::reset();
		finished_execution = true;
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void EsterelPlanDispatcher::planCallback(const rosplan_dispatch_msgs::EsterelPlan plan) {
		if(finished_execution) {
			ROS_INFO("KCL: (%s) Plan received.", ros::this_node::getName().c_str());
			plan_received = true;
			mission_start_time = ros::WallTime::now().toSec();
			current_plan = plan;
			printPlan();
		} else {
			ROS_INFO("KCL: (%s) Plan received, but current execution not yet finished.", ros::this_node::getName().c_str());
		}
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool EsterelPlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

		ROS_INFO("KCL: (%s) Dispatching plan.", ros::this_node::getName().c_str());

		ros::Rate loop_rate(10);
		replan_requested = false;
		plan_cancelled = false;
		
		// initialise machine
		initialise();

		// begin execution
		finished_execution = false;
		state_changed = false;
		bool plan_started = false;
		while (ros::ok() && !finished_execution) {

			// loop while dispatch is paused
			while (ros::ok() && dispatch_paused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// cancel plan
			if(plan_cancelled) {
				ROS_INFO("KCL: (%s) Plan cancelled.", ros::this_node::getName().c_str());
				break;
			}


			finished_execution = true;
			state_changed = false;

			// for each node check completion, conditions, and dispatch
			for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
				
				rosplan_dispatch_msgs::EsterelPlanNode node = *ci;

				// activate plan start edges
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START && !plan_started) {

					// activate new edges
					std::vector<int>::const_iterator ci = node.edges_in.begin();
					ci = node.edges_out.begin();
					for(; ci != node.edges_out.end(); ci++) {
						edge_active[*ci] = true;
					}

					finished_execution = false;
					state_changed = true;
					plan_started = true;
				}
				
				// do not check actions for nodes which are not action nodes
				if(node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)
					continue;

				// If at least one node is still executing we are not done yet
				if (action_dispatched[node.action.action_id] && !action_completed[node.action.action_id]) {
					finished_execution = false;
				}

				// check action edges
				bool edges_activate_action = true;
				std::vector<int>::iterator eit = node.edges_in.begin();
				for (; eit != node.edges_in.end(); ++eit) {
					if(!edge_active[(*eit)]) edges_activate_action = false;
				}
				if(!edges_activate_action) continue;

				// dispatch new action
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && !action_dispatched[node.action.action_id]) {

					finished_execution = false;

					// query KMS for condition edges
					bool condition_activate_action = false;
					if(edges_activate_action) {			
						condition_activate_action = checkPreconditions(node.action);
					}

					if(condition_activate_action) {

						// activate action
						action_dispatched[node.action.action_id] = true;
						action_received[node.action.action_id] = false;
						action_completed[node.action.action_id] = false;

						// dispatch action
						ROS_INFO("KCL: (%s) Dispatching action [%i, %s, %f, %f]",
								ros::this_node::getName().c_str(), 
								node.action.action_id,
								node.action.name.c_str(),
								(node.action.dispatch_time+planStartTime-missionStartTime),
								node.action.duration);

						action_dispatch_publisher.publish(node.action);
						state_changed = true;

						// deactivate incoming edges
						std::vector<int>::const_iterator ci = node.edges_in.begin();
						for(; ci != node.edges_in.end(); ci++) {
							edge_active[*ci] = false;
						}

						// activate new edges
						ci = node.edges_out.begin();
						for(; ci != node.edges_out.end(); ci++) {
							edge_active[*ci] = true;
						}
					}
				}

				// handle completion of an action
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END && action_completed[node.action.action_id]) {

					ROS_INFO("KCL: (%s) %i: action %s completed",
							ros::this_node::getName().c_str(),
							node.action.action_id,
							node.action.name.c_str());

					finished_execution = false;
					state_changed = true;

					// deactivate incoming edges
					std::vector<int>::const_iterator ci = node.edges_in.begin();
					for(; ci != node.edges_in.end(); ci++) {
						edge_active[*ci] = false;
					}

					// activate new edges
					ci = node.edges_out.begin();
					for(; ci != node.edges_out.end(); ci++) {
						edge_active[*ci] = true;
					}
				}

			} // end loop (action nodes)

			ros::spinOnce();
			loop_rate.sleep();

			if(state_changed) printPlan();

			// cancel dispatch on replan
			if(replan_requested) {
				ROS_INFO("KCL: (%s) Replan requested.", ros::this_node::getName().c_str());
				reset();
				return false;
			}
		}

		ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());
		
		reset();
		return true;
	}

	void EsterelPlanDispatcher::initialise() {

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
			action_dispatched[ci->action.action_id] = false;
			action_received[ci->action.action_id] = false;
			action_completed[ci->action.action_id] = false;
		}

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::const_iterator ci = current_plan.edges.begin(); ci != current_plan.edges.end(); ci++) {
			edge_active[ci->edge_id] = false;
		}
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void EsterelPlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());

		// action enabled
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled"))) {
			action_received[msg->action_id] = true;
			state_changed = true;
		}

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved")) {

			// check action is part of current plan
			if(!action_received[msg->action_id]) {
				ROS_INFO("KCL: (%s) Action not yet dispatched, ignoring feedback", ros::this_node::getName().c_str());
			}
			action_completed[msg->action_id] = true;
			state_changed = true;
		}

		// action completed (failed)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {
			replan_requested = true;
			action_completed[msg->action_id] = true;
		}
	}

	/*-------------------*/
	/* Produce DOT graph */
	/*-------------------*/

	bool EsterelPlanDispatcher::printPlan() {

		// output stream
		std::stringstream dest;

		dest << "digraph plan" << " {" << std::endl;

		// nodes
		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = current_plan.nodes.begin(); nit!=current_plan.nodes.end(); nit++) {
			dest <<  nit->node_id << "[ label=\"" << nit->name;

			switch(nit->node_type) {
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START:
				if(action_received[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];" << std::endl;
				} else if(action_dispatched[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkgoldenrod2];" << std::endl;
				} else {
					dest << "\"];" << std::endl;
				}
				break;
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END:
				if(action_completed[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];" << std::endl;
				} else if(action_dispatched[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkgoldenrod2];" << std::endl;
				} else {
					dest << "\"];" << std::endl;
				}
				break;
			case rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START:
				dest << "\",style=filled,fillcolor=black,fontcolor=white];" << std::endl;
				break;
			default:
				dest << "\"];" << std::endl;
				break;
			}
		}

		// edges
		for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::iterator eit = current_plan.edges.begin(); eit!=current_plan.edges.end(); eit++) {
			for(int j=0; j<eit->sink_ids.size(); j++) {
			for(int i=0; i<eit->source_ids.size(); i++) {

				dest << "\"" << eit->source_ids[i] << "\"" << " -> \"" << eit->sink_ids[j] << "\"";
				if(eit->duration_upper_bound == std::numeric_limits<double>::max()) {
					dest << " [ label=\"[" << eit->duration_lower_bound << ", " << "inf]\"";
				} else {
					dest << " [ label=\"[" << eit->duration_lower_bound << ", " << eit->duration_upper_bound << "]\"";
				}
				dest << " , penwidth=2"
						<< ((edge_active[eit->edge_id]) ? " , color=\"red\"" : " , color=\"black\"")
						<< "]" << std::endl;
			}};
		}

		dest << "}" << std::endl;

		// publish on topic
		std_msgs::String msg;
		msg.data = dest.str();
		plan_graph_publisher.publish(msg);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"rosplan_esterel_plan_dispatcher");
		ros::NodeHandle nh("~");

		KCL_rosplan::EsterelPlanDispatcher epd(nh);

		// subscribe to planner output
		std::string planTopic = "complete_plan";
		nh.getParam("plan_topic", planTopic);
		ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::EsterelPlanDispatcher::planCallback, &epd);

		std::string feedbackTopic = "action_feedback";
		nh.getParam("action_feedback_topic", feedbackTopic);
		ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1000, &KCL_rosplan::EsterelPlanDispatcher::feedbackCallback, &epd);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}