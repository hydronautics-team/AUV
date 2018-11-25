#include <ros/ros.h>
#include <auv_common/MoveByTimeAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <boost/bind.hpp>


class MoveByTimeActionServer
{

protected:

    ros::NodeHandle n;
    actionlib::SimpleActionServer<auv_common::MoveByTimeAction> server;
    std::string actionName;
    ros::Publisher publisher;

public:

    MoveByTimeActionServer(std::string name):
        server(n, name, boost::bind(&MoveByTimeActionServer::executeCB, this, _1), false),
        actionName(name)
    {
        server.start();
    }

    ~MoveByTimeActionServer(void)
    {
    }

    void executeCB(const auv_common::MoveByTimeGoalConstPtr &goal)
    {
        geometry_msgs::Twist startMsg;
        int direction = goal->direction;
        startMsg.linear.x = startMsg.linear.y = startMsg.linear.z = startMsg.angular.x = startMsg.angular.y = startMsg.angular.z = 0;
        if (direction == 1)
            startMsg.linear.x = -0.5;
        if (direction == 2)
            startMsg.linear.x = 0.5;
        if (direction == 3)
            startMsg.linear.y = -0.5;
        if (direction == 4)
            startMsg.linear.y = 0.5;

        ros::Rate pollRate(100);
        while (publisher.getNumSubscribers() == 0)
            pollRate.sleep();
        publisher.publish(startMsg);

        int time = goal->time;
        if (time != 0) {
            ros::Duration(time).sleep();
            geometry_msgs::Twist endMsg;
            endMsg.linear.x = endMsg.linear.y = endMsg.linear.z = endMsg.angular.x = endMsg.angular.y = endMsg.angular.z = 0;
            while (publisher.getNumSubscribers() == 0)
                pollRate.sleep();
            publisher.publish(endMsg);
        }
        server.setSucceeded();
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_by_time_server");
    MoveByTimeActionServer action("move_by_time_server");
    ros::spin();
    return 0;
}