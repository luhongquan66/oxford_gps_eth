/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>


// ROS messages
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

// Tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// Packet structure
#include "dispatch.h"

// UTM conversion
#include <gps_common/conversions.h>

// Ethernet
#include <arpa/inet.h>
#include <sys/socket.h>

#include <iconv.h>

// UINT16_MAX is not defined by default in Ubuntu Saucy
#ifndef UINT16_MAX
#define UINT16_MAX (65535)
#endif

static size_t count = 0;
static size_t countSet = 100;
static double pubHz = 1.0;
static int client_sockfd;


static inline bool openTCPServerSocket(const std::string &interface, const std::string &ip_addr,
                                       uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr)
{
    int fd;
    fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(fd != -1){
        if (interface.length()) {
          if (!setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.length()) == 0) {
            close(fd);
            return false;
          }
        }
        memset(sock_ptr, 0, sizeof(sockaddr_in));
        sock_ptr->sin_family = AF_INET;
        sock_ptr->sin_port = htons(port);
        if (!inet_aton(ip_addr.c_str(), &sock_ptr->sin_addr)) {
          sock_ptr->sin_addr.s_addr = INADDR_ANY; // Invalid address, use ANY
        }
        if (bind(fd, (sockaddr*)sock_ptr, sizeof(sockaddr)) == 0) {
          *fd_ptr = fd;
          return true;
        }
    }
    return  false;
}

void ScanCallback(const sensor_msgs::NavSatFixConstPtr &gpsMessIN)
{
    count++;
    if(count >= countSet){
        double longtitude = gpsMessIN->longitude;
        double latitude = gpsMessIN->latitude;

        std::string longtitudeString = std::to_string(longtitude);
        std::string latitudeString = std::to_string(latitude);

        std::string result = longtitudeString + "," + latitudeString;

        const char* buffer = result.data();

        if(send(client_sockfd, buffer,strlen(buffer), 0) < 0)//(char*)&buffer, sizeof(buffer)
        {
             ROS_FATAL("Write error");
        }

        std::cout << result << std::endl;//"longtitude is " << longtitudeString << "latitude is " << latitudeString

        count = 0;
    }


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "oxford_gps");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  //Set TCP
  std::string interface_tcp_server = "";
  priv_nh.getParam("interface_tcp_server", interface_tcp_server);

  std::string ip_addr_tcp_server = "127.0.0.1";
  priv_nh.getParam("ip_addr_tcp_server", ip_addr_tcp_server);

  int port_tcp_server = 65500;
  priv_nh.getParam("port_tcp_server", port_tcp_server);

  std::string interface_tcp_client = "";
  priv_nh.getParam("interface_tcp_client", interface_tcp_client);

  std::string ip_addr_tcp_client = "127.0.0.1";
  priv_nh.getParam("ip_addr_tcp_client", ip_addr_tcp_client);

  int port_tcp_client = 65501;
  priv_nh.getParam("port_tcp_client", port_tcp_client);

  priv_nh.getParam("pubHz", pubHz);
  countSet = (size_t)(100.0 / pubHz);

    // Setup Publishers
  ros::Subscriber gps_sub = node.subscribe<sensor_msgs::NavSatFix>("gps/fix", 2,
                                                                   &ScanCallback);


  // Variables
  Packet packet;
  sockaddr source;
  bool first = true;

  int fd;
  sockaddr_in sock;
  //Set TCP
  int server_sockfd;
  sockaddr_in server_sock;
  sockaddr_in client_sock;
  bool sendTCPPacket = false;
  //  char buf[BUFSIZ]; //BUFSIZ system defalut cache size.


  if(openTCPServerSocket(interface_tcp_server,ip_addr_tcp_server,
                         port_tcp_server,&server_sockfd,&server_sock)){
      if(listen(server_sockfd,5) < 0){
          ROS_FATAL("Listen error");
      }else{
          socklen_t sin_size = sizeof (client_sock);

          if((client_sockfd = accept(server_sockfd,(struct sockaddr*)&client_sock, &sin_size)) < 0)
          {
              ROS_FATAL("Accept error");
          }else{
              sendTCPPacket= true;
          }
      }
  }else{
      ROS_FATAL("Failed to open socketTCP");
  }
  ROS_INFO("TCP link has been setup!");
  // Loop until shutdown
  while (ros::ok()) {
      ros::spin();
    }

  // Close socket
  close(fd);
  close(server_sockfd);
  close(client_sockfd);

  return 0;
}
