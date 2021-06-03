
// Simple demo program that calls the UR10 in Gazebo with a trajectory
//

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>


int main(int argc, char **argv) {
	ros::init(argc, argv, "test_move_gazebo_cpp"); // name this node 
        ros::NodeHandle nh; //standard ros node handle        

	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("pos_based_pos_traj_controller/follow_joint_trajectory", true);

	ROS_INFO("Waiting for action server...");
	ac.waitForServer();

	ROS_INFO("Action server started");

	// setup trajectory message
	control_msgs::FollowJointTrajectoryGoal msg;

	// some arbitrary points of a not too useful trajectory
	const int nPoints = 4;
	double Q1[6] = {2.2,0.0,-1.57,0.0,0.0,0.0};
        double Q2[6] = {1.5,0.0,-1.57,0.0,0.0,0.0};
        double Q3[6] = {1.5,-0.2,-1.57,0.0,0.0,0.0};

	// set values for all points of trajectory
	for (int p = 0; p < 6; p++) { // iterate over all points
		trajectory_msgs::JointTrajectoryPoint point;
		for (int i = 0; i < 5; i++) { // 5 DOF
			point.positions.push_back(values[p][i]);
			point.velocities.push_back(0);
			point.accelerations.push_back(0);
		}
		point.time_from_start = ros::Duration((p+1) / 2.0);
		msg.trajectory.points.push_back(point);
	}

	// set joint names
	for (int i = 0; i < 5; i++) {
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		msg.trajectory.joint_names.push_back(jointName.str());
	}
	
	// fill message header and sent it out
	msg.trajectory.header.frame_id = "arm_link_0";
	msg.trajectory.header.stamp = ros::Time::now();
	ac.sendGoal(msg);

	// wait for reply that action was completed (or cancel after 10 sec)
	ac.waitForResult(ros::Duration(10));
	
	return 0;
}









#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>

using namespace std;

#define VECTOR_DIM 6 // e.g., a 6-dof vector
const double dt_traj = 0.02; // time step for trajectory interpolation
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_ur_jnt_names;
//TEST: deliberately limit joint velocities to very small values
double g_qdot_max_vec[] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; //put real vel limits here

void set_ur_jnt_names() {
    g_ur_jnt_names.push_back("shoulder_pan_joint");
    g_ur_jnt_names.push_back("shoulder_lift_joint");
    g_ur_jnt_names.push_back("elbow_joint");
    g_ur_jnt_names.push_back("wrist_1_joint");
    g_ur_jnt_names.push_back("wrist_2_joint");
    g_ur_jnt_names.push_back("wrist_3_joint");
}

//need to reference realistic joint velocity limits to compute min transition times
double transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0]) / g_qdot_max_vec[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = fabs(dqvec[i]) / g_qdot_max_vec[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_ur_jnt_names
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_ur_jnt_names[i].c_str());
    }

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start, q_end, dqvec;
    double del_time;
    double net_time = 0.05;
    q_start = qvecs[0];
    q_end = qvecs[0];
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];

    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
  //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout<<"traj pt: ";
                for (int j=0;j<VECTOR_DIM;j++) {
                    cout<<new_trajectory.points[iq].positions[j]<<", ";
                }
        cout<<endl;
        cout<<"arrival time: "<<new_trajectory.points[iq].time_from_start.toSec()<<endl;
    }
}


//parse the names in joint_names vector; find the corresponding indices of arm joints
//provide joint_names, as specified in message

void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_ur_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    cout << "indices of arm joints: " << endl;
    for (int i = 0; i < VECTOR_DIM; i++) {
        cout << g_arm_joint_indices[i] << ", ";
    }
    cout << endl;
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size() < 1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
    }
        for (int i = 0; i < VECTOR_DIM; i++) {
            g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
        }
        cout << "CB: q_vec_right_arm: " << g_q_vec_arm_Xd.transpose() << endl;
}

//action server will respond to this callback when done
void armDoneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_INFO(" armDoneCb: server responded with state [%s]", state.toString().c_str());
    g_done_move = true;
    //ROS_INFO("got return val = %d", result->return_val);
    g_done_count++;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ur_traj_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        

    Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec_arm;
    g_q_vec_arm_Xd.resize(VECTOR_DIM);

    std::vector<Eigen::VectorXd> des_path;
    
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   

    set_ur_jnt_names(); //fill a vector of joint names in DH order, from base to tip
    //here are initial, hard-coded joint angles for arm pose
    cout << "setting pre-pose: " << endl;
    q_pre_pose.resize(VECTOR_DIM);
    q_pre_pose << 0, 0, 0, 0, 0, 0; //-0.907528, -0.111813, 2.06622, 1.8737, -1.295, 2.00164, 0;

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);


    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size() < 1) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    //get current pose of arm:  
    cout << "current pose:" << g_q_vec_arm_Xd.transpose() << endl;


    //instantiate client of the arm server:
    //create an action client of UR's follow_joint_trajectory action server 
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
            arm_action_client("pos_based_pos_traj_controller/follow_joint_trajectory", true);

    // attempt to connect to the server(s):
    ROS_INFO("waiting for arm-control server: ");
    bool server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = arm_action_client.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;  

    int nposes = 6;
    int ans;
    control_msgs::FollowJointTrajectoryGoal goal; //consistent goal message for UR action service

    double q234;
    Eigen::Vector3d bz61;
    for (int i = 0; i < nposes; i++) {
        cout << "enter 1: "; //poor-man's break point
        cin >> ans;
        q_pre_pose[i] = -2.0; //each time through, set successive joint cmd to -2 rad
        des_path.clear();
        des_path.push_back(g_q_vec_arm_Xd); //start from current pose
        des_path.push_back(q_pre_pose); //and go to new desired pose
        stuff_trajectory(des_path, des_trajectory); //convert path to traj
        goal.trajectory = des_trajectory; // and put in a goal message

        ROS_INFO("sending goal to  arm: ");
        arm_action_client.sendGoal(goal, &armDoneCb); //ship off goal to action server
        while (g_done_count < 1) { //wait for "done" flag
            ROS_INFO("waiting to finish move..");
            ros::spinOnce(); //print jnt angles while waiting
            cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
        //q234 = g_q_vec_arm_Xd[1]+g_q_vec_arm_Xd[2]+g_q_vec_arm_Xd[3];
        //ROS_INFO("q234 = %f",q234);        
    }
    int jnt=1;
    double qval;
    while(jnt>=0) {
        cout<<"enter jnt num, 0 through 6: ";
        cin>>jnt;
        cout<<"enter jnt angle: ";
        cin>>qval;
        q_pre_pose[jnt] = qval; //each time through, set successive joint cmd to -2 rad
        des_path.clear();
        des_path.push_back(g_q_vec_arm_Xd); //start from current pose
        des_path.push_back(q_pre_pose); //and go to new desired pose
        stuff_trajectory(des_path, des_trajectory); //convert path to traj
        goal.trajectory = des_trajectory; // and put in a goal message

        ROS_INFO("sending goal to  arm: ");
        arm_action_client.sendGoal(goal, &armDoneCb); //ship off goal to action server
        while (g_done_count < 1) { //wait for "done" flag
            ROS_INFO("waiting to finish move..");
            ros::spinOnce(); //print jnt angles while waiting
            cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        cout << "arm is at: " << g_q_vec_arm_Xd.transpose() << endl;        
    }

    return 0;
}













#include <ros/ros.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -0.3;
    goal.trajectory.points[ind].positions[1] = 0.2;
    goal.trajectory.points[ind].positions[2] = -0.1;
    goal.trajectory.points[ind].positions[3] = -1.2;
    goal.trajectory.points[ind].positions[4] = 1.5;
    goal.trajectory.points[ind].positions[5] = -0.3;
    goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "test_move_gazebo");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
  return 0;
}
