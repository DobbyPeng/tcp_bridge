#ifndef FACTORY_INTERFACE_H
#define FACTORY_INTERFACE_H

// TCP server
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

#define MAXPACKETSIZE 4096

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

class TCPServer
{
  public:
    TCPServer(){}
    ~TCPServer()
    {
      close(sockfd_);
      close(newsockfd_);
    }
    void setup(int port);
    std::string receive();
    std::string getMessage(){return Message_;}

    void Send(std::string data){send(newsockfd_, data.c_str(), data.length(), 0);}

    void detach()
    {
      shutdown(sockfd_, SHUT_RDWR);
      shutdown(newsockfd_, SHUT_RDWR);
    }

    static void clean();

  private:
    int sockfd_, newsockfd_, n_; //Socket file descriptors and port number
    struct sockaddr_in serv_addr_, cli_addr_; ///two objects to store client and server address
    static std::string cli_addr_str_;
    pthread_t serverThread_;
    static std::string Message_;

    static void * Task(void * arg);
    void error(const char *msg)
    {
      perror(msg);
      exit(1);
    }
};
std::string TCPServer::Message_;
std::string TCPServer::cli_addr_str_;
void TCPServer::clean()
{
  Message_ = "";
}

class ROSBridge
{
  public:
    ROSBridge(ros::NodeHandle &n);
    ~ROSBridge(){}
    int port_;
    static ros::Publisher pub_client_cmd_;

//    static void *loop(void *m);
  private:
    ros::NodeHandle node_;
    ros::Subscriber sub_respond2factory_;
    void respond2FactoryCB(const std_msgs::String::ConstPtr &str);
};

// Publisher
ros::Publisher ROSBridge::pub_client_cmd_;

#endif // FACTORY_INTERFACE_H
