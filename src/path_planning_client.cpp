#include "ros/ros.h"
#include "path_planning_pkg/PlanPath.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_client");
    if (argc != 5)
    {
        ROS_INFO("usage: req_goal start_X start_Y goal_X goal_Y");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<path_planning_pkg::PlanPath>("plan_path");

    path_planning_pkg::PlanPath srv;
    srv.request.start.position.x = atof(argv[1]); 
    srv.request.start.position.y = atof(argv[2]); 
    srv.request.goal.position.x = atof(argv[3]); 
    srv.request.goal.position.y = atof(argv[4]); 

    if (client.call(srv))
    {
        ROS_INFO("Time: %f ms", srv.response.time);
    }
    else
    {
        ROS_ERROR("No solution at the exact location.. Time: %f ms", srv.response.time);
        return 1;
    }

    return 0;
}