#include "rosplan_planning_system/PlanParsing/PDDLEsterelPlanParser.h"
// remove, to print bound variable for test cases purposes
#include <fstream>

namespace KCL_rosplan {

	// constructor
	PDDLEsterelPlanParser::PDDLEsterelPlanParser(ros::NodeHandle &nh)
	{
		node_handle = &nh;

		// fetching problem info for TILs
		std::string kb = "knowledge_base";
		node_handle->getParam("knowledge_base", kb);

		std::stringstream ss;

		ss << "/" << kb << "/domain/predicates";
		get_predicate_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(ss.str().c_str());
		ss.str("");

		ss << "/" << kb << "/state/propositions";
		get_propositions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str().c_str());
		ss.str("");

		ss << "/" << kb << "/state/functions";
		get_functions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str().c_str());
		ss.str("");

		ss << "/" << kb << "/domain/operator_details";
		get_operator_details_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str().c_str());
		ss.str("");

		// publishing parsed plan
		std::string planTopic = "complete_plan";
		node_handle->getParam("plan_topic", planTopic);
		plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::EsterelPlan>(planTopic, 1, true);
	}

	PDDLEsterelPlanParser::~PDDLEsterelPlanParser() {}

	void PDDLEsterelPlanParser::reset() {
		action_list.clear();
		til_list.clear();
		last_plan.nodes.clear();
		last_plan.edges.clear();
	}

	void PDDLEsterelPlanParser::publishPlan() {
		plan_publisher.publish(last_plan);
	}

	void PDDLEsterelPlanParser::preparePlan() {

		// create the plan start node
		rosplan_dispatch_msgs::EsterelPlanNode plan_start;
		plan_start.node_type = rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START;
		plan_start.node_id = last_plan.nodes.size();
		plan_start.name = "plan_start";
		last_plan.nodes.push_back(plan_start);

		int curr, next;
		std::string line;

		// planner_output has the content of the subscription to the plan
		std::istringstream planfile(planner_output);

		size_t planFreeActionID = 0;

		// parse plan, create nodes, store them in last_plan member variable
		while (std::getline(planfile, line)) {

			if (line.length() < 2)
				break;

			// check to see if the line looks like a planned action
			if (line.find("[", 0) == std::string::npos
					|| line.find("]", 0) == std::string::npos
					|| line.find("(", 0) == std::string::npos
					|| line.find(")", 0) == std::string::npos
					|| line.find(":", 0) == std::string::npos)
				continue;

			rosplan_dispatch_msgs::ActionDispatch msg;

			// action ID
			msg.action_id = planFreeActionID;
			planFreeActionID++;

			// dispatchTime
			curr = line.find(":");
			double dispatchTime = (double)atof(line.substr(0,curr).c_str());
			msg.dispatch_time = dispatchTime;

			// check for parameters
			curr = line.find("(") + 1;
			bool paramsExist = (line.find(" ", curr) < line.find(")", curr));

			if(paramsExist) {

				// fill msg action name
				next = line.find(" ", curr);
				std::string name = line.substr(curr, next - curr).c_str();
				msg.name = name;

				/**
				 * msg:
				 * ---
				 * name: goto_waypoint
				 * parameters[]
				 * duration: 0
				 * dispatch_time: 0
				 */

				// fill msg parameters, use KB info to fill key information
				std::vector<std::string> params;
				curr = next + 1;
				next = line.find(")", curr);
				int at = curr;
				while(at < next) {
					int cc = line.find(" ", curr);
					int cc1 = line.find(")", curr);
					curr = cc < cc1? cc : cc1;
					std::string param = line.substr(at, curr - at);
					params.push_back(param);
					++curr;
					at = curr;
				}
				processPDDLParameters(msg, params);

				/**
				 * msg:
				 * ---
				 * name: goto_waypoint
				 * parameters[]
				 *   parameters[0]:
				 *     key: v
				 *     value: kenny
				 *   parameters[1]:
				 *     key: from
				 *     value: wp0
				 *   parameters[2]:
				 *     key: to
				 *     value: wp0
				 * duration: 0
				 * dispatch_time: 0
				 */

			} else {

				// name
				next = line.find(")", curr);
				std::string name = line.substr(curr, next - curr).c_str();
				msg.name = name;
			}

			// duration
			curr = line.find("[", curr) + 1;
			next = line.find("]", curr);
			msg.duration = (double)atof(line.substr(curr, next - curr).c_str());

			// create action msg
			// action_list.push_back(msg); TODO is not used!, this was spotted by oscar

			// create action start node
			rosplan_dispatch_msgs::EsterelPlanNode node_start;
			node_start.node_type = rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START;
			node_start.node_id = last_plan.nodes.size();
			node_start.action = msg;
			std::stringstream ss;
			ss << msg.name << "_start";
			node_start.name = ss.str();
			last_plan.nodes.push_back(node_start);

			// create action end node
			rosplan_dispatch_msgs::EsterelPlanNode node_end;
			node_end.node_type = rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END;
			node_end.node_id = last_plan.nodes.size();
			node_end.action = msg;
			ss.str("");
			ss << msg.name << "_end";
			node_end.name = ss.str();
			last_plan.nodes.push_back(node_end);
		}

		// fetch TILs
		fetchTILs();

		// find and create causal edges
		createGraph();
	}

	void PDDLEsterelPlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		// TODO: no need to query operators (which are fixed) multiple times, this can speed up the node
		// fetch operator details, this is assuming that the bottleneck is in creating the nodes, which is unlikely
		// I think creating the edges is where the problem is
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!get_operator_details_client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
		} else {

			// remove
			// ROS_INFO("Calling knowledge base srv to get operator details (TODO: fix wip)");

			action_details[msg.action_id] = srv.response.op;

			// save parameters in action message
			std::vector<diagnostic_msgs::KeyValue> opParams = srv.response.op.formula.typed_parameters;
			for(size_t i = 0; i < opParams.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = opParams[i].key;
				pair.value = params[i];
				msg.parameters.push_back(pair);
			}

			// ground and save action details
			groundFormula(action_details[msg.action_id].at_start_add_effects, opParams, params);
			groundFormula(action_details[msg.action_id].at_start_del_effects, opParams, params);

			groundFormula(action_details[msg.action_id].at_end_add_effects, opParams, params);
			groundFormula(action_details[msg.action_id].at_end_del_effects, opParams, params);

			groundFormula(action_details[msg.action_id].at_start_simple_condition, opParams, params);
			groundFormula(action_details[msg.action_id].at_end_simple_condition, opParams, params);
			groundFormula(action_details[msg.action_id].over_all_simple_condition, opParams, params);

			groundFormula(action_details[msg.action_id].at_start_neg_condition, opParams, params);
			groundFormula(action_details[msg.action_id].at_end_neg_condition, opParams, params);
			groundFormula(action_details[msg.action_id].over_all_neg_condition, opParams, params);
		}
	}

	void PDDLEsterelPlanParser::fetchTILs() {

		// fetch all domain predicates from KB
		rosplan_knowledge_msgs::GetDomainAttributeService srv;
		if(!get_predicate_client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for predicate list", ros::this_node::getName().c_str());
			return;
		}

		// for each predicate, fetch facts and filter to TILs, TODO: improve this comment once I get feedback from Michael

		// iterate over all domain predicates
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = srv.response.items.begin();
		for( ; pit != srv.response.items.end(); pit++) {

			rosplan_knowledge_msgs::GetAttributeService attsrv;
			attsrv.request.predicate_name = pit->name;
			if(!get_propositions_client.call(attsrv)) {
				ROS_ERROR("KCL: (%s) could not call Knowledge Base for (%s)", ros::this_node::getName().c_str(), pit->name.c_str());
				continue;
			}

			// save current time
			ros::Time time = ros::Time::now();

			// iterate over all state propositons
			for(int i = 0; i < attsrv.response.attributes.size(); i++) {

				// not a TIL (Timed Initial Literal?)
				if(time > attsrv.response.attributes[i].initial_time)
					continue;

				// save TIL
				double key = (attsrv.response.attributes[i].initial_time - time).toSec();
				til_list.insert(std::make_pair(key, attsrv.response.attributes[i]));

				ROS_INFO_STREAM("TIL key:" << key << ", TIL value: " << attsrv.response.attributes[i]);
			}
		}
	}

	void PDDLEsterelPlanParser::createGraph() {

		// map of absolute plan dispatch time to node ID
		std::multimap<double, int> nodes;

		// iterate over all nodes in the (so far unconnected) graph and get their dispatch time
		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ait = last_plan.nodes.begin();
		for(; ait != last_plan.nodes.end(); ait++) {

			// get node dispatch time
			double time = 0;
			switch(ait->node_type) {

				case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START:
					time = ait->action.dispatch_time;
					break;
				case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END:
					time = ait->action.dispatch_time + ait->action.duration;
					break;
				case rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START:
				default:
					time = 0;
					break;
			}

			// construct an ordered list of the nodes based on dispatch time
			nodes.insert(std::pair<double, int>(time, ait->node_id));
		}

		// HACK: make first edge, this is to remove all edges that are coming from initial state
		// makeEdge(0, 1, rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE);

		// get the next node
		std::multimap<double,int>::iterator nit = nodes.begin();
		for(; nit != nodes.end(); nit++) {

			// node : a pointer to the next node in the ordered list
			rosplan_dispatch_msgs::EsterelPlanNode *node = &last_plan.nodes[nit->second];

			switch(node->node_type) {
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END:

				// make all edges from action_start to action_end
				makeEdge(node->node_id-1, node->node_id, node->action.duration, node->action.duration,
					rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE);

				// NOTE: there is no break here, so case ACTION_END will continue

			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START:

				// PLAN_START is filtered out but at this point nodes can be still of type ACTION_END or ACTION_START

				// for each precondition add possible causal support edge with latest preceding node
				rosplan_knowledge_msgs::DomainOperator op = action_details[node->action.action_id];
				std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

				bool edge_created = false;
				if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {

					for(cit = op.at_start_simple_condition.begin(); cit != op.at_start_simple_condition.end(); cit++)
						if(addConditionEdge(nodes, nit, *cit, false, false)) edge_created = true;
					for(cit = op.over_all_simple_condition.begin(); cit!=op.over_all_simple_condition.end(); cit++)
						if(addConditionEdge(nodes, nit, *cit, false, true)) edge_created = true;
					for(cit = op.at_start_neg_condition.begin(); cit!=op.at_start_neg_condition.end(); cit++)
						if(addConditionEdge(nodes, nit, *cit, true, false)) edge_created = true;
					for(cit = op.over_all_neg_condition.begin(); cit!=op.over_all_neg_condition.end(); cit++)
						if(addConditionEdge(nodes, nit, *cit, true, true)) edge_created = true;

				} else if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {

					for(cit = op.at_end_simple_condition.begin(); cit!=op.at_end_simple_condition.end(); cit++)
						if(addConditionEdge(nodes, nit, *cit, false, false)) edge_created = true;
					for(cit = op.at_end_neg_condition.begin(); cit!=op.at_end_neg_condition.end(); cit++)
						if(addConditionEdge(nodes, nit, *cit, true, false)) edge_created = true;
				}

				// interference edges
				if(addInterferenceEdges(nodes, nit)) edge_created = true;

				// create an edge from the plan start
				if(!edge_created) {
					// Conditional edges comming from initial state to nodes
					makeEdge(0, node->node_id, rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE);
				}

				break;
			}
		}

		// o: remove
		std::ofstream myfile;
		myfile.open ("/home/oscar/test/bounds/bounds.txt");

		// o: remove, graph is finished, lets print all interference edge bounds
		for(auto it = bounds_bkp_.begin(); it != bounds_bkp_.end(); it++) {
			ROS_INFO("pair %lu = %lf, %lf", it - bounds_bkp_.begin(), it->first, it->second);
			myfile << "pair " << it - bounds_bkp_.begin() << " = " << it->first << ", " << it->second << std::endl ;
		}
		myfile.close();
	}

	/**
	 * @brief Given a node and a condition, creates an edge from the node that supports the condition,
	 * including from the initial state for TILs. Also creates an upper bound on that edge should a
	 * TIL violate the condition.
	 * @returns True if an edge is created.
	 */
	bool PDDLEsterelPlanParser::addConditionEdge(
				std::multimap<double,int> &node_map,
				std::multimap<double,int>::iterator &current_node,
				rosplan_knowledge_msgs::DomainFormula &condition,
				bool negative_condition,
				bool overall_condition) {

		// reverse through TILs
		std::multimap<double, rosplan_knowledge_msgs::KnowledgeItem>::const_reverse_iterator tit;
		tit = til_list.rbegin();

		// find latest TIL before current action
		while(tit != til_list.rend() && tit->first > current_node->first) {

			// check TIL interference
			if(satisfiesPrecondition(condition, tit->second, !negative_condition)) {
				if(overall_condition) {
					// edge goes to end action node instead of start action
					makeEdge(0, current_node->second + 1, 0, tit->first,
						rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE);
				} else {
					makeEdge(0, current_node->second, 0, tit->first,
						rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE);
				}
			}

			tit++;
		}

		// for this condition check previous actions and TILs
		std::multimap<double, int>::const_reverse_iterator rit(current_node);
		for(; rit != node_map.rend(); rit++) {
			// get TILs that come after the previous actions
			while(tit != til_list.rend() && tit->first > rit->first) {

				// check TIL support
				if(satisfiesPrecondition(condition, tit->second, negative_condition)) {
					makeEdge(0, current_node->second, tit->first, std::numeric_limits<double>::max(),
						rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE);
					return true;
				}
				tit++;
			}
			// check action support
			if(satisfiesPrecondition(condition, last_plan.nodes[rit->second], negative_condition)) {
				makeEdge(rit->second, current_node->second,
						 rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE);
				return true;
			}
		}

		return false;
	}

	/**
	 * @brief Adds edges from all previous nodes with which there is interference.
	 * @param node_map ordered map of dispatch_time and nodes id
	 * @param current_node iterator to current multimap element being analyzed
	 *   (access via ->first and ->second : dispatch_time and node_id)
	 * @return True if an edge is created.
	 */
	bool PDDLEsterelPlanParser::addInterferenceEdges(std::multimap<double, int> &node_map,
									std::multimap<double, int>::iterator &current_node) {

		bool edge_added = false;

		rosplan_dispatch_msgs::EsterelPlanNode *node = &last_plan.nodes[current_node->second];

		// o: iterate over all previous (ordered) nodes in the master node map
		// m: for this node check previous nodes
		std::multimap<double, int>::const_reverse_iterator rit(current_node);
		for(; rit != node_map.rend(); rit++) {

			// o: substract from plan all previous nodes, one at a time
			rosplan_dispatch_msgs::EsterelPlanNode *prenode = &last_plan.nodes[rit->second];

			// o: remove, measure getBounds() time
			ros::Time begin = ros::Time::now();

			// o: compute bounds using function, send previous nodes and current node
			// o: in first iteration same node will be sent twice prenode = node but
			// o: they have added a check for that and will return 0,0
			// m: if already ordered, then skip
			std::pair<double, double> bounds = getBounds(*prenode, *node);

			// o: remove, make bkp of all bounds for tests cases purposes
			bounds_bkp_.push_back(bounds);

			// o: remove, print bounds
			//ROS_INFO("lower bound = %lf, upper bound = %lf", bounds.first, bounds.second);

			// o: remove, measure getBounds() time
			ros::Time end = ros::Time::now();
			total_time_ += ros::Duration(end - begin);
			//ROS_INFO("Took %lf secs to compute getBounds()", total_time_.toSec());

			// o: condition to proceed analysis based on bounds value
			if( (bounds.first >= 0) || (bounds.second <= 0) )
				continue;

			bool interferes = false;

			rosplan_knowledge_msgs::DomainOperator op = action_details[prenode->action.action_id];
			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

			// m: for this pair of nodes check if current node interferes with previous node
			if(prenode->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {

				// m: determine if current_node negates at start condition of action start node
				for(cit = op.at_start_simple_condition.begin(); cit != op.at_start_simple_condition.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, true);
				for(cit = op.at_start_neg_condition.begin(); cit != op.at_start_neg_condition.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, false);

				// m: determine if previous node start effect interferes with current node effects
				for(cit = op.at_start_add_effects.begin(); cit != op.at_start_add_effects.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, true);
				for(cit = op.at_start_del_effects.begin(); cit != op.at_start_del_effects.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, false);

			} else if(prenode->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {

				// m: determine if current_node negates at end condition of action end node
				for(cit = op.at_end_simple_condition.begin(); cit != op.at_end_simple_condition.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, true);
				for(cit = op.at_end_neg_condition.begin(); cit != op.at_end_neg_condition.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, false);

				// m: determine if current_node negates over all condition of action end node
				for(cit = op.over_all_simple_condition.begin(); cit != op.over_all_simple_condition.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, true);
				for(cit = op.over_all_neg_condition.begin(); cit != op.over_all_neg_condition.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, false);

				// m: determine if previous node end effect interferes with current node effects
				for(cit = op.at_end_add_effects.begin(); cit != op.at_end_add_effects.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, true);
				for(cit = op.at_end_del_effects.begin(); cit != op.at_end_del_effects.end(); cit++)
					interferes = interferes || satisfiesPrecondition(*cit, *node, false);
			}

			if(interferes) {
				makeEdge(prenode->node_id, node->node_id,
						 rosplan_dispatch_msgs::EsterelPlanEdge::INTERFERENCE_EDGE);
				edge_added = true;
			}
		}

		return edge_added;
	}

	// TODO: optimize, do not make a copy, receive as reference function variables
	void PDDLEsterelPlanParser::makeEdge(int source_node_id, int sink_node_id, double lower_bound,
						double upper_bound, int edge_type) {

		// see if there is already an existing edge, TODO: optimize, do this check faster
		std::vector<int>::iterator eit = last_plan.nodes[source_node_id].edges_out.begin();
		std::vector<int>::iterator nit = last_plan.nodes[source_node_id].edges_out.begin();
		for(; eit!=last_plan.nodes[source_node_id].edges_out.end(); eit++) {
			int edgeID = (*eit);
			if(edgeID < 0 && edgeID >= last_plan.edges.size()) continue;
			nit = std::find(last_plan.edges[edgeID].sink_ids.begin(), last_plan.edges[edgeID].sink_ids.end(), sink_node_id);
			if(nit != last_plan.edges[edgeID].sink_ids.end()) {

				// update the bounds and return
				if(lower_bound > last_plan.edges[edgeID].duration_lower_bound)
					last_plan.edges[edgeID].duration_lower_bound = lower_bound;
				if(upper_bound < last_plan.edges[edgeID].duration_upper_bound)
					last_plan.edges[edgeID].duration_upper_bound = upper_bound;
				return;
			}
		}

		// create and add edge
		rosplan_dispatch_msgs::EsterelPlanEdge newEdge;
		newEdge.edge_id = last_plan.edges.size();
		// conditional edge, interference edge, useful for debugging purposes
		newEdge.edge_type = edge_type;
		std::stringstream ss;
		ss << "edge" << "_" << newEdge.edge_id;
		newEdge.edge_name = ss.str();
		newEdge.signal_type = 0;
		newEdge.source_ids.push_back(source_node_id);
		newEdge.sink_ids.push_back(sink_node_id);

		newEdge.duration_lower_bound = lower_bound;
		newEdge.duration_upper_bound = upper_bound;

		last_plan.edges.push_back(newEdge);

		last_plan.nodes[source_node_id].edges_out.push_back(newEdge.edge_id);
		last_plan.nodes[sink_node_id].edges_in.push_back(newEdge.edge_id);
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"rosplan_plan_parser");
		ros::NodeHandle nh("~");

		KCL_rosplan::PDDLEsterelPlanParser pp(nh);

		// subscribe to planner output
		std::string planTopic = "planner_output";
		nh.getParam("planner_topic", planTopic);
		ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::PlanParser::plannerCallback, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));

		// start the plan parsing services
		ros::ServiceServer service1 = nh.advertiseService("parse_plan", &KCL_rosplan::PlanParser::parsePlan, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));
		ros::ServiceServer service2 = nh.advertiseService("parse_plan_from_file", &KCL_rosplan::PlanParser::parsePlanFromFile, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
