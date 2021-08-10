#include "tcpServer.h"

/**
 * @brief TCPServer
 */
void TCPServer::setup(int port)
{
  // IPv4, TCP
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd_ < 0)
    error("ERROR opening socket");
  int enable = 1;
  if(setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    error("setsockopt(SO_REUSEADDR) failed");
  bzero((char *) &serv_addr_, sizeof(serv_addr_));
  serv_addr_.sin_family = PF_INET;
  serv_addr_.sin_addr.s_addr = INADDR_ANY;
  serv_addr_.sin_port = htons(port);
  if(bind(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_)) < 0)
    error("ERROR on binding");
  listen(sockfd_, 5);
}

/**
 * Wait for client connect
 */
std::string TCPServer::receive()
{
  ros::Rate rate_10hz(10);
  while(ros::ok())
  {
    rate_10hz.sleep();
    socklen_t clilen = sizeof(cli_addr_); //object clilen of type socklen_t
    newsockfd_ = accept(sockfd_, (struct sockaddr *) &cli_addr_, &clilen);
    if(newsockfd_ < 0)
      error("ERROR on accept");
    cli_addr_str_ = inet_ntoa(cli_addr_.sin_addr);
    pthread_create(&serverThread_, NULL, &Task, (void *)newsockfd_);
    ROS_INFO_STREAM(cli_addr_str_);
  }
  return cli_addr_str_;
}

/**
 * Receive message from client
 */
void * TCPServer::Task(void *arg)
{
  ssize_t n;
  ros::Rate rate_10hz(10);
  int newsockfd = (long)arg;
  pthread_detach(pthread_self());
  while(ros::ok())
  {
    rate_10hz.sleep();
    char buffer[MAXPACKETSIZE];
    n = recv(newsockfd, buffer, MAXPACKETSIZE, MSG_DONTWAIT);
    if(n == 0)
    {
      ROS_WARN_STREAM("TCP client: " << cli_addr_str_ << " disconnect.");
      break;
    }
    else if(n == -1)
      continue;
    buffer[n] = 0;
    Message_ = std::string(buffer);
    std_msgs::String factoryCommand;
    factoryCommand.data = Message_;
    ROSBridge::pub_client_cmd_.publish(factoryCommand);
    clean();
  }
  close(newsockfd);
  pthread_exit(NULL);
}

ROSBridge::ROSBridge(ros::NodeHandle &n):
  node_(n)
{
  node_.param("port", port_, 9939);

  pub_client_cmd_ = node_.advertise<std_msgs::String>("message_from_client", 1);
  sub_respond2factory_ = node_.subscribe("respond_to_client", 10, &ROSBridge::respond2FactoryCB, this);
  tcpser_.setup(port_);
}

void ROSBridge::respond2FactoryCB(const std_msgs::String::ConstPtr &str)
{
  tcpser_.Send(str->data);
}

void * ROSBridge::loop(void * m)
{
  pthread_detach(pthread_self());
  ros::waitForShutdown();
  tcpser_.detach();
  pthread_exit(NULL);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TCP_Server");
  ros::NodeHandle nh("~");
  ROSBridge RFI(nh);
  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();
  pthread_t shutdownThread;
  pthread_create(&shutdownThread, NULL, &RFI.loop, (void *)0);
  RFI.tcpser_.receive();
  return 0;
}
