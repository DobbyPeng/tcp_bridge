#include "tcpServer.h"

/**
 * @brief TCPServer
 */
void TCPServer::setup(int port)
{
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd_ < 0)
    error("ERROR opening socket");
  int enable = 1;
  if(setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    error("setsockopt(SO_REUSEADDR) failed");
  bzero((char *) &serv_addr_, sizeof(serv_addr_));
  serv_addr_.sin_family = AF_INET;
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
  while(ros::ok())
  {
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
  int n;
  int newsockfd = (long)arg;
  pthread_detach(pthread_self());
  while(ros::ok())
  {
    char buffer[MAXPACKETSIZE];
    n = recv(newsockfd, buffer, MAXPACKETSIZE, 0);
    if(n == 0)
    {
      ROS_WARN_STREAM("TCP client: " << cli_addr_str_ << " disconnect.");
      close(newsockfd);
      break;
    }
    buffer[n] = 0;
    Message_ = std::string(buffer);
    std_msgs::String factoryCommand;
    factoryCommand.data = Message_;
    ROSBridge::pub_client_cmd_.publish(factoryCommand);
    clean();
  }
  pthread_exit(NULL);
}

TCPServer tcpser;

ROSBridge::ROSBridge(ros::NodeHandle &n):
  node_(n)
{
  node_.param("port", port_, 9939);

  pub_client_cmd_ = node_.advertise<std_msgs::String>("message_from_client", 1);
  sub_respond2factory_ = node_.subscribe("respond_to_client", 10, &ROSBridge::respond2FactoryCB, this);
}

void ROSBridge::respond2FactoryCB(const std_msgs::String::ConstPtr &str)
{
  tcpser.Send(str->data);
}

//void * ROSBridge::loop(void * m)
//{
//  pthread_detach(pthread_self());
//  while(ros::ok())
//  {
//    std::string str = tcpser.getMessage();
//    if( str != "" )
//    {
//      std::cout << "Message:" << str << std::endl;
//      std_msgs::String factoryCommand;
//      factoryCommand.data = str;
//      pub_client_cmd_.publish(factoryCommand);
//      tcpser.clean();
//    }
//    usleep(1000);
//  }
//  tcpser.detach();
//  pthread_exit(NULL);
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TCP_Server");
  ros::NodeHandle nh("~");
  ROSBridge RFI(nh);
  tcpser.setup(RFI.port_);
//  pthread_t buffer;
  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();
//  if( pthread_create(&buffer, NULL, &RFI.loop, (void *)0) == 0)
//  {
//    tcpser.receive();
//  }
  tcpser.receive();
  return 0;
}
