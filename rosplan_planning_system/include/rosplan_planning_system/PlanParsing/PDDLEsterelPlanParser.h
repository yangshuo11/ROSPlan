#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "diagnostic_msgs/KeyValue.h"

#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>

#ifndef KCL_pddl_esterel_plan_parser
#define KCL_pddl_esterel_plan_parser

namespace KCL_rosplan {

	class PDDLEsterelPlanParser: public PlanParser
	{
	private:

		/*---------------------------------*/
		/* ACTION SUPPORT AND INTERFERENCE */
		/*---------------------------------*/

		/**
		 * @returns True if the node satisfies the (possibly negative) condition
		 */
		bool satisfiesPrecondition(rosplan_knowledge_msgs::DomainFormula& condition, const rosplan_dispatch_msgs::EsterelPlanNode& node, bool negative_condition) {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

			if(!negative_condition) {

				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {
					cit = action_details[node.action.action_id].at_start_add_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_start_add_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				} else if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
					cit = action_details[node.action.action_id].at_end_add_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_end_add_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				}

			} else {

				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {
					cit = action_details[node.action.action_id].at_start_del_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_start_del_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				} else if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
					cit = action_details[node.action.action_id].at_end_del_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_end_del_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				}
			}

			return false;
		}

		/**
		 * @returns True if the effect satisfies the (possibly negative) condition
		 */
		bool satisfiesPrecondition(rosplan_knowledge_msgs::DomainFormula& condition, const rosplan_knowledge_msgs::KnowledgeItem& effect, bool negative_condition) {

			// convert KnowledgeItem to (grounded) DomainFormula
			rosplan_knowledge_msgs::DomainFormula eff;
			eff.name = effect.attribute_name;
			for(int i=0; i<effect.values.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = effect.values[i].key;
				pair.value = effect.values[i].value;
				eff.typed_parameters.push_back(pair);
			}

			return (negative_condition == effect.is_negative) && domainFormulaMatches(condition, eff);
		}

		/*---------------*/
		/* GRAPH METHODS */
		/*---------------*/

		void makeEdge(int source_node_id, int sink_node_id, int edge_type) {
			// create an ordering that only specifies after
			makeEdge(source_node_id, sink_node_id, 0, std::numeric_limits<double>::max(), edge_type);
		}

		// TODO: remove all o: and m: put here and in corresponding cpp file

		/**
		 * @brief o: compute bounds, which is a condition that decides if interference edge is worth proceeding
		 * @param a o: previous ordered nodes in the master map, one at a time (this function is called multiple times)
		 * @param b o: current node being analyzed for interference edges
		 * @returns m: a std::pair describing the upper and lower bounds on the ordering
		 */
		std::pair<double, double> getBounds(rosplan_dispatch_msgs::EsterelPlanNode &a,
										   rosplan_dispatch_msgs::EsterelPlanNode &b) {

			// o: std::set is a container (e.g. like std::vector)
			// o: std::set does not allow duplicate elements, all are unique
			// o: elements that contains are specified in template argument
			// o: internally elements are stored in balanced binary tree
			// o: will keep inserted elements in order
			// o: supports iterators
			// TODO: o: i see this variable not being used and must be remove, however
			// is not the main cause of the problem therefore i will ignore for now but needs
			// to be fixed later
			std::set<int> checked_flags;
			return getBounds(a, b, checked_flags);
		}

		std::pair<double, double> getBounds(rosplan_dispatch_msgs::EsterelPlanNode &a,
				rosplan_dispatch_msgs::EsterelPlanNode &b, std::set<int> &checked_flags) {

			// o: sanity check, if nodes are same return bound 0,0?
			// m: a node happens when it happens
			if(a.node_id == b.node_id)
				return std::make_pair<double, double>(0, 0);

			// o: initialize bound default values to -inf, +inf
			// m: potentially no bounds
			std::pair<double, double> bounds = std::make_pair<double, double>(
				-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

			// m: check all nodes ordered after a

			// m: iterate over edges_out of a (prenode)
			for(size_t i = 0; i < a.edges_out.size(); i++) {
				int edge_id = a.edges_out[i];
				for(size_t j = 0; j < last_plan.edges[edge_id].sink_ids.size(); j++) {
					// o: remove
					if(j >= 1)
						ROS_INFO("GREATER!!!!\n THAN!!!!!\n1 element!!!!!!!\n");

					int node_id = last_plan.edges[edge_id].sink_ids[j];
					// m: get bounds from child
					std::pair<double, double> newbounds = getBounds(last_plan.nodes[node_id], b, checked_flags);
					// m: update bounds
					if(bounds.first < newbounds.first + last_plan.edges[edge_id].duration_lower_bound)
						bounds.first = newbounds.first + last_plan.edges[edge_id].duration_lower_bound;
					if(bounds.second > newbounds.second + last_plan.edges[edge_id].duration_upper_bound)
						bounds.second = newbounds.second + last_plan.edges[edge_id].duration_upper_bound;
				}
			}

			return bounds;
		}

		/*------------------------*/
		/* DOMAIN FORMULA METHODS */
		/*------------------------*/

		/**
		 * @brief Grounds the given vector of domain formulae. Resulting in e.g., (robot_at ?wp - wp01)
		 * @param formula  The domain formula to be grounded, e.g., (robot_at ?wp - waypoint)
		 * @param opParams An ordered KeyValue vector of operator parameters, e.g., (?r - robot ?wp - waypoint)
		 * @param params   An ordered vector of parameter objects, e.g., [robot01, wp01]
		 */

		void groundFormula(std::vector<rosplan_knowledge_msgs::DomainFormula> &formula,
				std::vector<diagnostic_msgs::KeyValue> &opParams, std::vector<std::string> &params) {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;
			cit = formula.begin();
			for(; cit!=formula.end(); cit++) {
				// for each predicate parameter
				for(size_t j = 0; j < cit->typed_parameters.size(); j++) {
					bool found = false;
					// for each operator parameter
					for(size_t i = 0; i < opParams.size(); i++) {
						// operator and predicate parmater labels match
						if(opParams[i].key == cit->typed_parameters[j].key) {
							// set predicate parameter to be: label -> value
							cit->typed_parameters[j].value = params[i];
							found = true;
						}
					}
					// if not found, then label must be a constant
					if(!found) {
						// set predicate parameter to be: value -> value
						// key could be fetched by "get_predicate_details" service, but is not required
						cit->typed_parameters[j].value = cit->typed_parameters[j].key;
					}
				}
			}
		}

		/**
		 * @returns True if the two domain formula parameters match completely.
		 */
		bool domainFormulaMatches(rosplan_knowledge_msgs::DomainFormula& a, rosplan_knowledge_msgs::DomainFormula& b) {
			if(b.name != a.name) return false;
			if(b.typed_parameters.size() != a.typed_parameters.size()) return false;
			for(size_t i = 0; i < a.typed_parameters.size(); i++) {
				if(a.typed_parameters[i].value != b.typed_parameters[i].value) {
					return false;
				}
			}
			return true;
		}

		/* plan description in Esterel */
		rosplan_dispatch_msgs::EsterelPlan last_plan;
		std::map<int, rosplan_knowledge_msgs::DomainOperator> action_details;

		/* TILs */
		ros::ServiceClient get_predicate_client;
		ros::ServiceClient get_propositions_client;
		ros::ServiceClient get_functions_client;
		ros::ServiceClient get_operator_details_client;
		std::multimap<double, rosplan_knowledge_msgs::KnowledgeItem> til_list;

		// remove
		ros::Duration total_time_;

		// remove
		std::vector<std::pair<double, double> > bounds_bkp_;

	protected:

		/* post process plan */

		/**
		 * @brief to populate action msgs with action parameters
		 * @param msg
		 * @param params
		 */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);

		/**
		 * @brief Apparently TODO : confirm; TILS (Timed Invariant Literals? are facts that came after the planner
		 * has created solution, they are used in addConditionEdge() only. They are not used in turtlebot example
		 */
		void fetchTILs();

		/**
		 * @brief Add edges to (all) already created nodes, the core of the esterel is here
		 */
		void createGraph();

		/**
		 * @brief TODO
		 */
		bool addInterferenceEdges(std::multimap<double,int> &node_map, std::multimap<double,int>::iterator &current_node);

		/**
		 * @brief helper function to facilitate the creation of edges
		 * first check if edge is already part of the graph, then make the edge
		 * @param source_node_id the parent node from which the edge is going to be created
		 * @param sink_node_id the child node to which the edge is directed to
		 * @param lower_bound dispatch time at start
		 * @param upper_bound dispatch time at end
		 * @param edge_type used for debugging purposes, can be conditional edge, interference or the kind
		 * that connects action start with action end
		 */
		void makeEdge(int source_node_id, int sink_node_id, double lower_bound, double upper_bound, int edge_type);

		/**
		 * @brief add edge to graph based on a cause-effect basis TODO:revise
		 * @param node_map ordered map of dispatch_time and nodes id
		 * @param current_node current multimap element being analyzed (access via first - dispatch_time and second - node_id)
		 * @param condition grounded action (pre) condition
		 * @param negative_condition true is condition is of the form e.g.: (not (at robot kitchen))
		 * @param overall_condition groups of conditions? all true or false. Use false if doubt.
		 */
		bool addConditionEdge(
				std::multimap<double,int> &node_map, std::multimap<double,int>::iterator &current_node,
				rosplan_knowledge_msgs::DomainFormula &condition, bool negative_condition, bool overall_condition);

		/* pure virtual methods */

		/**
		 * @brief Clear all (relevant) member variables to get ready to receive the next plan
		 */
		void reset();

		/**
		 * @brief parses planner PDDL output, and generates all nodes of the graph, edges at this point
		 * are still pending, will be added later by createGraph() function
		 */
		void preparePlan();

		/**
		 * @brief publish last_plan member variable.
		 */
		void publishPlan();

	public:

		/**
		 * @brief constructor: prepare kb service clients to get info (predicates, propositions, etc)
		 * set up esterel graph plan publisher.
		 */
		PDDLEsterelPlanParser(ros::NodeHandle &nh);

		/**
		 * @brief destructor: empty
		 */
		~PDDLEsterelPlanParser();

		// to publish the esterel complete plan (stored in member variable "last_plan")
		ros::Publisher plan_publisher;
	};
} // close namespace

#endif
