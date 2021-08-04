#ifndef TCPCLIENT_H
#define TCPCLIENT_H

// TCP server
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAXPACKETSIZE 4096

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

class TCPClient
{
  public:
    TCPClient(){}
    ~TCPClient()
    {
      close(sockfd_);
    }
    void setup(std::string addr, int port);
    std::string receive();
    std::string getMessage(){return Message_;}

    void Send(std::string data){send(sockfd_, data.c_str(), data.length(), 0);}

    void detach()
    {
      shutdown(sockfd_, SHUT_RDWR);
    }

    void clean()
    {
      Message_ = "";
    }
  private:
    int sockfd_;
    struct sockaddr_in serv_addr_, cli_addr;
    std::string serv_addr_str_;
    std::string Message_;
    void error(const char *msg)
    {
      perror(msg);
      exit(1);
    }
};

class ROSBridge
{
  public:
    ROSBridge(ros::NodeHandle &n);
    ~ROSBridge(){}
    std::string ip_;
    int port_;
    static ros::Publisher pub_server_cmd_;

    static void *loop(void *m);
  private:
    ros::NodeHandle node_;
    ros::Subscriber sub_respond2server_;
    void respond2ServerCB(const std_msgs::String::ConstPtr &str);
};

// Publisher
ros::Publisher ROSBridge::pub_server_cmd_;

#endif // TCPCLIENT_H
