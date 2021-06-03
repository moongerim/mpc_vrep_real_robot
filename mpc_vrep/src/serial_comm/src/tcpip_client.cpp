#include <ros/ros.h>
#include <sys/socket.h>
#include<arpa/inet.h> 
#include<netinet/in.h>



int main(int argc, char **argv)
{
	ros::init(argc,argv,"client");
	ros::NodeHandle nh;

	ros::Rate loop_rate(50);
	
	in_addr_t rec_addr = inet_addr("10.1.71.192"); // first arg is '127.0.0.1'
   	in_port_t rec_port = 45678;
   	
   	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
	    ROS_ERROR_STREAM("error socket");
	}
	ROS_INFO_STREAM("socket created");
	
	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = rec_port;
	//inet_aton("10.1.71.192", &serv_addr.sin_addr.s_addr);
	serv_addr.sin_addr.s_addr = rec_addr;

	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(struct sockaddr_in)) == -1) {
	    ROS_ERROR_STREAM("error connect");
	}
	ROS_INFO_STREAM("connected");
	
	char buf[1024];
	//char msg[] = "OK";
	while(ros::ok())
	{
		if(recv(sockfd, buf, 1023, 0) <= 0) 
		{
			
			ROS_INFO_STREAM("nothing");
			ros::shutdown();
		}
		send(sockfd, "OK", 2, 0);
		ROS_INFO_STREAM(buf);
	}
	
}
