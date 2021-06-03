#include <ros/ros.h>
#include <sys/socket.h>
#include<arpa/inet.h> 
#include<netinet/in.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"server");
	ros::NodeHandle nh;

	ros::Rate loop_rate(50);
	
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
	struct sockaddr_in serv_addr, client_addr;

	//in_addr_t rec_addr = inet_addr(INADDR_ANY); // first arg is '127.0.0.1'
   	in_port_t rec_port = 45678;

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = rec_port;
	serv_addr.sin_addr.s_addr = INADDR_ANY;

	if( bind(sockfd,(struct sockaddr *)&serv_addr , sizeof(serv_addr)) < 0)
    	{
        //print the error message
        ROS_ERROR_STREAM("bind failed. Error");
        return 1;
    	}
	
	if (listen(sockfd, SOMAXCONN) < 0) ROS_ERROR_STREAM("Couldn't listen");
	int c = sizeof(struct sockaddr_in);
	
	int client_sock = accept(sockfd, (struct sockaddr *) &client_addr, (socklen_t *) &c);
	
	char msg[] = "Hello From Server!";
	char buf[1024];
	int someInt = 0;

	while(ros::ok())
	{
		char str[12];
		sprintf(str, "%d", someInt);
		send(client_sock, str, strlen(str), 0);
		read(client_sock, buf, 1023);
		ROS_INFO_STREAM(buf);
		loop_rate.sleep();
		someInt++;
	}
}
