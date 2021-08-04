#include "tcpClient.h"

void TCPClient::setup(std::string addr, int port)
{
  sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd_ < 0)
    error("ERROR opening socket");
  bzero(&serv_addr_, sizeof(serv_addr_));
  serv_addr_.sin_family = AF_INET;
  serv_addr_.sin_addr.s_addr = inet_addr(addr.c_str());
  serv_addr_.sin_port = htons(port);
  if(connect(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_)) < 0)
    error("ERROR on connecting");
  serv_addr_str_ = inet_ntoa(serv_addr_.sin_addr);
}

std::string TCPClient::receive()
{
  int n;
  int newsockfd = (long)sockfd_;
  while(ros::ok())
  {
    char buffer[MAXPACKETSIZE];
    n = recv(newsockfd, buffer, MAXPACKETSIZE, 0);
    if(n == 0)
    {
      ROS_WARN_STREAM("TCP server: " << serv_addr_str_ << " disconnect.");
      close(newsockfd);
      break;
    }
    buffer[n] = 0;
    Message_ = std::string(buffer);
    std_msgs::String serverCommand;
    serverCommand.data = Message_;
    ROSBridge::pub_server_cmd_.publish(serverCommand);
    clean();
  }
  return serv_addr_str_;
}

TCPClient tcpcli;

ROSBridge::ROSBridge(ros::NodeHandle &n):
  node_(n)
{
  XmlRpc::XmlRpcValue ip, port;
  if(!node_.getParam("IP", ip))
  {
    ROS_FATAL("No ip given. (namespace: %s)", node_.getNamespace().c_str());
    exit(1);
  }
  if(ip.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_FATAL("Malformed ip specification. (namespace: %s)", node_.getNamespace().c_str());
    exit(1);
  }
  ip_ = (std::string)ip;

  if(!node_.getParam("port", port))
  {
    ROS_FATAL("No port given. (namespace: %s)", node_.getNamespace().c_str());
    exit(1);
  }
  if(port.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_FATAL("Malformed port specification. (namespace: %s)", node_.getNamespace().c_str());
    exit(1);
  }
  port_ = port;

  pub_server_cmd_ = node_.advertise<std_msgs::String>("message_from_server", 1);
  sub_respond2server_ = node_.subscribe("respond_to_server", 10, &ROSBridge::respond2ServerCB, this);
}

void ROSBridge::respond2ServerCB(const std_msgs::String::ConstPtr &str)
{
  tcpcli.Send(str->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TCP_Client");
  ros::NodeHandle nh("~");
  ROSBridge RB(nh);
  tcpcli.setup(RB.ip_, RB.port_);
  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();
  tcpcli.receive();
  return 0;
}
