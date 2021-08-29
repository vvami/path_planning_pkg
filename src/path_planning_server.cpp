#include "ros/ros.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include "path_planning_pkg/PlanPath.h"
#include <vector>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;


class Planner
{
    public:

        Planner()
        {
            //publisher to visualize the obstacles as markers on rviz
            marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

            //publisher that publishes the computed collision-free path
            path_pub = n.advertise<nav_msgs::Path>("path_topic", 10);

            generate_obstacles();
            visualize_obstacles();

            //provides a server that accepts the start and end positions and returns the time taken to compute the path
            service = n.advertiseService("plan_path", &Planner::service_callback,this);
            ROS_INFO("Ready to plan the path.");
        }

        void generate_obstacles()
        {
            ompl::RNG rng_;

            //generate randomly placed and randomly sized obstacles
            for(int idx = 0; idx < obstacles_number; ++idx)
            {
                obstacles_x.push_back(rng_.uniformReal(-40,40));
                obstacles_y.push_back(rng_.uniformReal(-40,40));
                obstacles_r.push_back(rng_.uniformReal(5,10));
            }
        }

        void visualize_obstacles()
        {
            //loop through each and every obstacle randomly generated previously and visualize it on rviz
            for (int idx = 0; idx < obstacles_number; ++idx)
            {
                draw_sphere(obstacles_x[idx], obstacles_y[idx], obstacles_r[idx], idx);
            }
        }

        bool service_callback(path_planning_pkg::PlanPath::Request &req, path_planning_pkg::PlanPath::Response &res)
        {
            //measure initial time
            double secs = ros::Time::now().toSec();
            
            if (plan(req.start.position.x, req.start.position.y, req.goal.position.x, req.goal.position.y))
            {
                //if a collision-free path is computed calculate the time taken to compute that path
                res.time = (ros::Time::now().toSec() - secs) * 1000.0;
                return true;
            }
            else
            {
                res.time = 0.0;
                return false;
            }
        }

        bool plan(double start_x, double start_y, double goal_x, double goal_y)
        {
            // construct the state space we are planning in
            auto space(std::make_shared<ob::SE2StateSpace>());

            //set the bounds in the x and y directions
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-50);
            bounds.setHigh(50);

            space->setBounds(bounds);

            //create space information for the state space
            auto si(std::make_shared<ob::SpaceInformation>(space));
            si->setStateValidityChecker(boost::bind(&Planner::isStateValid, this, _1));
            si->setStateValidityCheckingResolution(0.01);
            
            //define the starting state
            ob::ScopedState<ob::SE2StateSpace> start(space);
            start->setX(start_x);
            start->setY(start_y);
            start->setYaw(0);

            //define the goal state
            ob::ScopedState<ob::SE2StateSpace> goal(space);
            goal->setX(goal_x);
            goal->setY(goal_y);
            goal->setYaw(0);

            //problem definition
            auto pdef(std::make_shared<ob::ProblemDefinition>(si));
            pdef->setStartAndGoalStates(start, goal);

            //select RRT* as the planner
            auto planner(std::make_shared<og::RRTstar>(si));
            planner->setProblemDefinition(pdef);
            planner->setup();
            ob::PlannerStatus solved = planner->ob::Planner::solve(1.0); //define the time for planning

            if (solved)
            {
                // get the goal representation from the problem definition (not the same as the goal state)
                // and inquire about the found path
                ob::PathPtr path = pdef->getSolutionPath();
                std::cout << "Found solution:" << std::endl;

                og::PathGeometric *path_;
                path_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
                path_->print(std::cout);

                //checks whether a collision-free path to the goal state has actually been found
                return(visualize_path(path_, goal_x,goal_y));
                
            }
            else
            {
                
                return false;
            }
        }//end plan

    private:

        void draw_sphere(double x, double y, double r, int uid)
        {
            //create a marker message in order to visualize the obstacles on rviz
            visualization_msgs::Marker marker;

            //shape of the obstacle is cube
            uint32_t shape = visualization_msgs::Marker::SPHERE;
            marker.header.frame_id = "my_frame";
            marker.header.stamp = ros::Time::now();

            //each obstacle would have a unique id
            marker.ns = "basic_shapes";
            marker.id = uid;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;

            //each obstacle has a different random position
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            //each obstacle has a different random size
            marker.scale.x = r;
            marker.scale.y = r;
            marker.scale.z = r;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            //ensure a subscriber(rviz) exists before publishing the marker
            while (marker_pub.getNumSubscribers() < 1)
            {
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
            marker_pub.publish(marker);
        }//end draw_sphere

        bool isStateValid(const ob::State *state)
        {
            
            double x = state->as<ob::SE2StateSpace::StateType>()->getX();
            double y = state->as<ob::SE2StateSpace::StateType>()->getY();
            double yaw = state->as<ob::SE2StateSpace::StateType>()->getYaw();

            for (int idx = 0; idx < obstacles_number; ++idx)
            {
                //if the state is within any of the obstacles. 2D circle equation is used as if statement condition
                if (pow(x - obstacles_x[idx], 2) + pow(y - obstacles_y[idx], 2) <= pow(obstacles_r[idx], 2))
                {
                    return false;
                }
            }

            return true;
        }//end isStateValid

        bool visualize_path(const og::PathGeometric *path, double goal_x, double goal_y)
        {
            nav_msgs::Path path_msg;
            
            path_msg.header.frame_id = "my_frame";

            bool check;

            for (std::size_t idx = 0; idx < path->getStateCount(); idx++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "my_frame";

                //extract state
                const auto state = path->getState(idx);

                //extract x and y
                double x = state->as<ob::SE2StateSpace::StateType>()->getX();
                double y = state->as<ob::SE2StateSpace::StateType>()->getY();

                pose.pose.position.x = x;
                pose.pose.position.y = y;

                path_msg.poses.push_back(pose);

            
                //check whether the last state is actually the goal
                if(idx == path->getStateCount()-1) check = (x == goal_x) && (y == goal_y);

             
            }
            if (check)
            {
                //only publish the path when the last state is the goal
                path_pub.publish(path_msg);
                std::cout << "Published path." << std::endl;
            }
            return check;
        }//end visualize_path

        const int obstacles_number = 20;
        std::vector<double> obstacles_x; //stores the x-position of the obstacles
        std::vector<double> obstacles_y; //stores the y-position of the obstacles
        std::vector<double> obstacles_r; //stores the size (radius) of the obstacles
        ros::NodeHandle n;
        ros::Publisher marker_pub; //publisher to visualize the obstacles as markers on rviz
        ros::Publisher path_pub; //publisher that publishes the computed collision-free path
        ros::ServiceServer service; //provides a server that accepts the start and end positions and returns the time taken to compute the path
};//end class Planner


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_server");

    Planner myplanner;
    
    ros::spin();

    return 0;
}