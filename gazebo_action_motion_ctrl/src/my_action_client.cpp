// my_action_client: 
// qck



//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../example_action_server/action/demo.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (demo) and appended name (Action)
// If you write a new client of the server in this package, you will need to include example_action_server in your package.xml,
// and include the header file below
#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<gazebo_action_motion_ctrl/pathAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h> 
#include <std_msgs/Int32.h>

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
bool g_lidar_alarm=false; // global var for lidar alarm
geometry_msgs::Pose g_curr_pose;
std::string changedir="x";
bool g_is_done=false;
bool was_canceled=false;
bool was_blocked_y=false;
double g_strt_dist=0.0;
double blocked_x=0.0; //this holds the x value at which the path was last blocked

void doneCb(const actionlib::SimpleClientGoalState& state,
        const gazebo_action_motion_ctrl::pathResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_curr_pose=result->final_pose;
    g_is_done=true;
    was_canceled=false;
    if(g_curr_pose.position.x>blocked_x){
        blocked_x=g_curr_pose.position.x;
        was_blocked_y=false;
    }
}
void feedbackCb(const gazebo_action_motion_ctrl::pathFeedbackConstPtr& feedback) {
    ROS_INFO("Pose #:%d Completed!",feedback->goal_num);
    ROS_INFO("%d goals Remaining in Path",feedback->num_goals_rem);
    g_curr_pose=feedback->curr_pose;
    ROS_INFO("Current Pos. (%f,%f)",g_curr_pose.position.x,g_curr_pose.position.y);
}

void alarmCallback(const std_msgs::Bool& alarm_msg) { 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!");
  }
}
void activeCb()
{
  ROS_INFO("Goal is Active");
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "action_client_node"); // name this node 
	ros::NodeHandle n;
    ros::Subscriber alarm_sub = n.subscribe("/lidar_alarm", 1, alarmCallback); 
    actionlib::SimpleActionClient<gazebo_action_motion_ctrl::pathAction> action_client("my_action", true);
    gazebo_action_motion_ctrl::pathGoal goal; 
    ROS_INFO("waiting for server: ");
	bool server_exists = action_client.waitForServer(); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever
    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }
    ROS_INFO("connected to action server");  // if here, then we connected to the server;
	geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
	double curr_yaw=0.0;
    pose.position.x = 1.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    goal.nav_path.poses.push_back(pose_stamped);
    action_client.sendGoal(goal, &doneCb);
    pose.position.x = 2.0;
    pose_stamped.pose = pose;
    goal.nav_path.poses.push_back(pose_stamped);
    while(true) {
        if(g_lidar_alarm){
            action_client.cancelAllGoals();
            ROS_INFO("Goals Cancelled");
            pose=g_curr_pose;
            pose_stamped.pose = pose;
            goal.nav_path.poses.push_back(pose_stamped);
            action_client.sendGoal(goal, &doneCb);
            action_client.waitForResult();
            if(changedir=="y"){
                was_blocked_y=true;
            } else if(changedir=="-y"){
                ROS_WARN("No Path Found, Aborting");
                return 0;
            }   
            was_canceled=true;
        }
        if(g_is_done){  //checks if the current goal is done, so that it can wait and check lidar alarm
            if(was_canceled){
                if(changedir=="x"&&!was_blocked_y){
                    pose.position.y+=1;
                    changedir="y";
                    g_strt_dist+=1;
                }else {
                    pose.position.y=pose.position.y-g_strt_dist;
                    g_strt_dist=1;
                    changedir="-y";
                }
            }else{
                pose.position.x+=1;
                changedir="x";
            }
            pose_stamped.pose = pose;
    	    goal.nav_path.poses.push_back(pose_stamped);
            ROS_INFO("Order up: x = %f y = %f", pose.position.x, pose.position.y);  
            action_client.sendGoal(goal, &doneCb,&activeCb, &feedbackCb );
            g_is_done=false;
        }
        ros::spinOnce();
        goal.nav_path.poses.clear();
    }
    return 0;
}

