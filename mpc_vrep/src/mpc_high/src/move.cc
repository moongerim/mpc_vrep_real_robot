#include "gen3_mpc_joint/move.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "std_msgs/Int32.h"
#include <rosgraph_msgs/Clock.h>

using namespace std;

ofstream myfile;
ofstream goalfile;
ofstream myperffile;

double p_control = 1.0000;

double d_1 =  0.08920;           // Offset of link 1
double d_4 =  0.10900;           // Offset of link 4
double d_5 =  0.09300;           // Offset of link 5
double d_6 =  0.08200;           // Offset of link 6
double a_2 = -0.42500;           // Length of link 2
double a_3 = -0.39243;			 // Length of link 3

double vrep_time;
int gripper = 0;
int rti_num = 10;
MPC_solver myMpcSolver(rti_num);

float dist_v(Eigen::Vector3f v, Eigen::Vector3f w){
	return (v-w).norm();
}

double x_pos = 0;
double y_pos = 0;
double z_pos = 0;

Eigen::MatrixXf get_cpose(float theta_1, float theta_2, float theta_3, float theta_4, float theta_5, float theta_6){
  Eigen::MatrixXf mat(3,9);
  mat <<0, x_pos+0.11*sin(theta_1) + a_2*cos(theta_1)*cos(theta_2), x_pos+(cos(theta_1)*(a_3*cos(theta_2 + theta_3) + 4.0*a_2*cos(theta_2)))/4, x_pos+(cos(theta_1)*(a_3*cos(theta_2 + theta_3) + 2.0*a_2*cos(theta_2)))/2, x_pos+(cos(theta_1)*(3.0*a_3*cos(theta_2 + theta_3) + 4.0*a_2*cos(theta_2)))/4, x_pos+cos(theta_1)*(a_3*cos(theta_2 + theta_3) + a_2*cos(theta_2)), x_pos+d_4*sin(theta_1) + a_2*cos(theta_1)*cos(theta_2) + a_3*cos(theta_1)*cos(theta_2)*cos(theta_3) - 1.0*a_3*cos(theta_1)*sin(theta_2)*sin(theta_3),      x_pos+d_4*sin(theta_1) + d_5*(cos(theta_4)*(cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2)) - 1.0*sin(theta_4)*(cos(theta_1)*sin(theta_2)*sin(theta_3) - 1.0*cos(theta_1)*cos(theta_2)*cos(theta_3))) + a_2*cos(theta_1)*cos(theta_2) + a_3*cos(theta_1)*cos(theta_2)*cos(theta_3) - 1.0*a_3*cos(theta_1)*sin(theta_2)*sin(theta_3),     x_pos+0.11*cos(theta_5)*sin(theta_1) + d_6*(cos(theta_5)*sin(theta_1) - cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5)) + d_4*sin(theta_1) - 0.11*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + a_2*cos(theta_1)*cos(theta_2) + d_5*sin(theta_2 + theta_3 + theta_4)*cos(theta_1) + a_3*cos(theta_1)*cos(theta_2)*cos(theta_3) - 1.0*a_3*cos(theta_1)*sin(theta_2)*sin(theta_3),
        0, y_pos+a_2*cos(theta_2)*sin(theta_1) - 0.11*cos(theta_1), y_pos+(sin(theta_1)*(a_3*cos(theta_2 + theta_3) + 4.0*a_2*cos(theta_2)))/4, y_pos+(sin(theta_1)*(a_3*cos(theta_2 + theta_3) + 2.0*a_2*cos(theta_2)))/2, y_pos+(sin(theta_1)*(3.0*a_3*cos(theta_2 + theta_3) + 4.0*a_2*cos(theta_2)))/4, y_pos+sin(theta_1)*(a_3*cos(theta_2 + theta_3) + a_2*cos(theta_2)), y_pos+a_2*cos(theta_2)*sin(theta_1) - 1.0*d_4*cos(theta_1) + a_3*cos(theta_2)*cos(theta_3)*sin(theta_1) - 1.0*a_3*sin(theta_1)*sin(theta_2)*sin(theta_3),  y_pos+d_5*(cos(theta_4)*(cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2)) - 1.0*sin(theta_4)*(sin(theta_1)*sin(theta_2)*sin(theta_3) - 1.0*cos(theta_2)*cos(theta_3)*sin(theta_1))) - 1.0*d_4*cos(theta_1) + a_2*cos(theta_2)*sin(theta_1) + a_3*cos(theta_2)*cos(theta_3)*sin(theta_1) - 1.0*a_3*sin(theta_1)*sin(theta_2)*sin(theta_3), y_pos+a_2*cos(theta_2)*sin(theta_1) - 1.0*d_6*(cos(theta_1)*cos(theta_5) + cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5)) - 1.0*d_4*cos(theta_1) - 0.11*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) - 0.11*cos(theta_1)*cos(theta_5) + d_5*sin(theta_2 + theta_3 + theta_4)*sin(theta_1) + a_3*cos(theta_2)*cos(theta_3)*sin(theta_1) - 1.0*a_3*sin(theta_1)*sin(theta_2)*sin(theta_3),
        0, z_pos+d_1 + a_2*sin(theta_2),                            z_pos+d_1 + 0.25*a_3*sin(theta_2 + theta_3) + a_2*sin(theta_2),             z_pos+d_1 + 0.5*a_3*sin(theta_2 + theta_3) + a_2*sin(theta_2),              z_pos+d_1 + 0.75*a_3*sin(theta_2 + theta_3) + a_2*sin(theta_2),                 z_pos+d_1 + a_3*sin(theta_2 + theta_3) + a_2*sin(theta_2),          z_pos+d_1 + a_3*sin(theta_2 + theta_3) + a_2*sin(theta_2),                                                                                                 z_pos+d_1 + a_3*sin(theta_2 + theta_3) + a_2*sin(theta_2) - 1.0*d_5*cos(theta_2 + theta_3 + theta_4),                                                                                                                                                                                                                                                                     z_pos+d_1 - sin(theta_5)*(0.11*cos(theta_2 + theta_3)*sin(theta_4) + 0.11*sin(theta_2 + theta_3)*cos(theta_4)) + a_3*sin(theta_2 + theta_3) + 1.0*d_5*(1.0*sin(theta_2 + theta_3)*sin(theta_4) - 1.0*cos(theta_2 + theta_3)*cos(theta_4)) + a_2*sin(theta_2) - 1.0*d_6*sin(theta_5)*(1.0*cos(theta_2 + theta_3)*sin(theta_4) + sin(theta_2 + theta_3)*cos(theta_4));
  return mat;
}

// Introduce class to make safer goal change
class GoalFollower 
{ 
  // Access specifier 
  public: 

  // Data Members 
  ros::Publisher chatter_pub;
  ros::Publisher chatter_pub_test;
  ros::Publisher goal_state;
  
  double robot_spheres[8] = {0.25, 0.13, 0.13, 0.13, 0.13, 0.15, 0.12, 0.18};
  double human_sphere[56]= {10.0517,   0.5220,   1.0895,   0.1500,
                            10.0658,   0.4526,   0.8624,   0.6010,
                            10.0844,   0.7044,   0.9207,   0.5010,
                            10.2083,   0.3075,   1.0208,   0.5010,
                            10.0556,   0.6289,   0.7595,   0.5010,
                            10.2024,   0.2732,   0.8478,   0.5010,
                            10.0267,   0.5535,   0.5983,   0.5010,
                            10.1965,   0.2389,   0.6749,   0.5010,
                            -10.0208,   0.3964,   0.5857,   0.4510,
                            10.0546,   0.2951,   0.6132,   0.4510,
                            -10.1062,   0.2444,   0.5897,   0.4810,
                            -10.0998,   0.3062,   0.5387,   0.4810,
                            10.1908,   0.5290,   1.0016,   0.5510,
                            10.2106,   0.4602,   0.6915,   0.6010};

  double goal[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
  double comand_vel[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
  double joint_position[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};

  // Member Functions() 

  void change_goal(double new_goal[],int n) 
  { 
    for (int i=0; i<n; i++) goal[i] = new_goal[i];
    ROS_INFO("Goal set to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
    goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]); 
  }

  void change_obstacles_msg(const std_msgs::Float64MultiArray obstacle_data) 
  { 
    for (int i=0; i<56; i++) human_sphere[i] = obstacle_data.data[i];
  }

  void change_states_msg(const sensor_msgs::JointState::ConstPtr& msg) 
  { 
    for (int i=0; i<6; i++) joint_position[i] = msg->position[i];
  }

  void SendVelocity(const std_msgs::Float64MultiArray joint_vel_values){
    chatter_pub.publish(joint_vel_values);
    return;
  }

  void SendVelocity_test(const std_msgs::Float64MultiArray joint_vel_values){
    chatter_pub_test.publish(joint_vel_values);
    return;
  }

}; 

void vrep_time_msg(const std_msgs::Float64& msg){
    vrep_time = msg.data;
    return;
}

int main(int argc, char **argv)
{
  myfile.open("data_high.csv", ios::out);
  goalfile.open("goal.csv", ios::out);
  myperffile.open("data_perf.csv", ios::out); 
  ros::init(argc, argv, "joint_controller_high");

  ros::NodeHandle n;
  ROS_INFO("Node Started");
  //--------------------------------
  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/my_ur5/mpc_high_positions", 1);
  my_follower.chatter_pub_test = n.advertise<std_msgs::Float64MultiArray>("/my_ur5/mpc_high_positions_test", 1);
  my_follower.goal_state = n.advertise<std_msgs::String>("/my_ur5/goal_status", 1);

  // ros::Publisher PauseHigh = n.advertise<std_msgs::Int32>("pauseHigh", 1);
  // while (PauseHigh.getNumSubscribers() < 1);
  // std_msgs::Int32 msg;
  // msg.data = 0;
  // PauseHigh.publish(msg);

  ROS_INFO("Goal default to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
	          my_follower.goal[0], my_follower.goal[1], my_follower.goal[2],
            my_follower.goal[3], my_follower.goal[4], my_follower.goal[5]);

  //--------------------------------

  // goal in joint space
  // define key trajectory points
  double read_goal[2][6] = {-0.619, -0.625, -0.591, -0.563, -0.531, -0.581,
                            -0.2084, -0.2847, -0.225, -0.267, -0.3357, -0.1715};
  double static_goal[6] = {read_goal[1][0], read_goal[1][1], read_goal[1][2], read_goal[1][3], read_goal[1][4], read_goal[1][5]};

  my_follower.change_goal(static_goal,6);

  //--------------------------------
  
  // turn off  
  // ros::Subscriber joint_goal = n.subscribe("/my_ur5/joint_goal", 1, &GoalFollower::change_goal_msg, &my_follower);
  //------
  ros::Subscriber human_status = n.subscribe("/vrep/my_ur5/mpc_high_spheres", 1, &GoalFollower::change_obstacles_msg, &my_follower);
  ros::Subscriber joint_status = n.subscribe("/high_joint_states", 1, &GoalFollower::change_states_msg, &my_follower);
  ros::Subscriber vrep_time_listener = n.subscribe("/clock", 1, vrep_time_msg);
  
  
  std_msgs::Float64MultiArray joint_vel_values;

  // Big loop
  double loop_duration = 85; // no pauses
  double start_motion_time = 15.00;
  double stop_human_time = (loop_duration*2);
  for (int big_loop_iteration=0; big_loop_iteration<10; big_loop_iteration++) {
    start_motion_time = big_loop_iteration*0.5 + 15.0;
    stop_human_time = big_loop_iteration*(loop_duration*2) + (loop_duration*2);
    //** Low level Loop      
    int row_index = 0;
    double loop_start_time = 0;
	  
	  ros::Rate goto_loop(20);
	  ros::Duration(0.50).sleep();
	  while (vrep_time < start_motion_time + stop_human_time - (loop_duration*2)){
      printf("vrep time=%f\n", vrep_time);
      joint_vel_values.data.clear();
      for (int i = 0; i < 12; i++) joint_vel_values.data.push_back(0.0);
      for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(static_goal[i]);
      my_follower.SendVelocity(joint_vel_values);
      ros::spinOnce();
      goto_loop.sleep();
      vrep_time=vrep_time+0.5;
	  };

	  int task = 0;
    int task_started = 0;
	  ros::Rate loop_rate(4); 

	  while (vrep_time < stop_human_time)
	  {
      // change to arrive check. add ~1.5s before next entrance
	    if (row_index==0) {
        if (task_started == 0) {
			    task = task + 1;
			    loop_start_time = vrep_time;
		    }
        task_started = 1;
      }

      // Goal reference position
      double currentState_targetValue[68];
      for (int i = 0; i < 6; ++i) currentState_targetValue[ i ] = my_follower.joint_position[ i ];
      for (int i = 0; i < 6; ++i) currentState_targetValue[ i+6 ] = read_goal[row_index][i];
      for (int i = 0; i < 56; ++i) currentState_targetValue[ i+14 ] = my_follower.human_sphere[ i ];

      ROS_INFO("Goal set to: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
	    currentState_targetValue[6], currentState_targetValue[7], currentState_targetValue[8],
      currentState_targetValue[9], currentState_targetValue[10], currentState_targetValue[11]); 
      
      double cgoal[3];
      Eigen::MatrixXf cgoal_mat = get_cpose(read_goal[row_index][0], read_goal[row_index][1], 
		                                        read_goal[row_index][2], read_goal[row_index][3], 
                                            read_goal[row_index][4], read_goal[row_index][5]);
      cgoal[0] = cgoal_mat.coeff(0, 8);
      cgoal[1] = cgoal_mat.coeff(1, 8);
      cgoal[2] = cgoal_mat.coeff(2, 8);
      ROS_INFO("cartesian Goal set to: %.3f, %.3f, %.3f", cgoal[0], cgoal[1], cgoal[2]); 
      // end Cartesian Goal
 
      // msg.data = 1;
      // PauseHigh.publish(msg);
      double* solutions=myMpcSolver.solve_mpc(currentState_targetValue, cgoal);
      // msg.data = 0;
      // printf("flag\n");
      // PauseHigh.publish(msg);
      
      //*********************** Apply control ********************************

      // Check if arrived
      std_msgs::String goal_state_msg;
      std::stringstream ss;
      float max_diff = 0;
      for (int i = 0; i < 6; ++i) {
        if (abs(currentState_targetValue[i] - currentState_targetValue[i+6]) > max_diff) {
          max_diff = abs(currentState_targetValue[i] - currentState_targetValue[i+6]); 
        }
      }
      printf("max value %f\n",max_diff);
      if (max_diff < 0.1) {
        ss << "Arrived";
        if (row_index==1 && task_started==1) {
          task_started = 0;
          double perf_record = vrep_time - loop_start_time;
          myperffile << "Performance " << big_loop_iteration <<" "<< perf_record <<" "<< task<< "\n";
        }
        row_index = (row_index+1)%2;
      }
      else ss << "Following";
      goal_state_msg.data = ss.str();
      my_follower.goal_state.publish(goal_state_msg);

      //******************* get_min_dist **********************
	    float local_val = 10000;
	    double smallest_dist = 10000;
      // printf("flag1\n");
	    double min_dist[] = {10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000};
		  Eigen::MatrixXf mat2 = get_cpose(my_follower.joint_position[0], my_follower.joint_position[1], 
                                       my_follower.joint_position[2], my_follower.joint_position[3], 
                                       my_follower.joint_position[4], my_follower.joint_position[5]);
      

	    for (int j = 0; j<8; j++) {
        Eigen::Vector3f w;
        w = mat2.col(j+1).transpose();
        for (int i = 0; i < 14; i++) {
		      Eigen::Vector3f p(my_follower.human_sphere[i*4+0],my_follower.human_sphere[i*4+1],my_follower.human_sphere[i*4+2]);
		      local_val = dist_v(w, p) - my_follower.robot_spheres[j] - my_follower.human_sphere[i*4+3];
		      if (min_dist[j] > local_val) min_dist[j] = local_val;
		    }
		    if (smallest_dist > min_dist[j]) smallest_dist = min_dist[j];
	    }
      
      joint_vel_values.data.clear();

      for (int i = 0; i < 12; i++) {
        joint_vel_values.data.push_back(solutions[i]);
      }
      for (int i = 0; i < 6; i++){
        joint_vel_values.data.push_back(currentState_targetValue[i]);
      } 
      my_follower.SendVelocity_test(joint_vel_values);
      std::stringstream save2file;
      if (myfile.is_open()){ 
        // mpc_time, KKT, is_infisible, smallest_dist
	      myfile << vrep_time << " " << solutions[12] << " " << solutions[13] << " " << solutions[14] << " " << smallest_dist << " " << row_index << endl;
	    }
      else cout << "Unable to open file";

      if (goalfile.is_open()){ 
        // q_goal
	      goalfile << vrep_time << " " <<read_goal[row_index][0] << " " <<read_goal[row_index][1]<< " " <<read_goal[row_index][2]<< " " <<read_goal[row_index][3]<< " " <<read_goal[row_index][4]<< " " <<read_goal[row_index][5]<< " "<<row_index << endl;
	    }
      else cout << "Unable to open file";

      ros::spinOnce();
      loop_rate.sleep();
	  }
  }
  myperffile.close();
  myfile.close();
  return 0;
}

