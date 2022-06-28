#ifndef L_UDP_HANDLER_H
#define L_UDP_HANDLER_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <arpa/inet.h>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>
class UDPServerHandler {
    public:
        UDPServerHandler(int port, bool need_to_connect, std::string ip_addr):
        portno_(port),
        need_to_connect_(need_to_connect),
        ip_addr_(ip_addr) {
            status_handler_ = "[udp@"+ip_addr+":"+std::to_string(port)+"]: ";
            addrlen_ = sizeof(serv_addr_);
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd_ < 0)
                std::cout << status_handler_ << "ERROR open socket" << std::endl;

            serv_addr_.sin_family    = AF_INET;
            serv_addr_.sin_port = htons(portno_); 
            serv_addr_.sin_addr.s_addr = inet_addr(ip_addr_.c_str());
            if ( bind(sockfd_, (struct sockaddr *)&serv_addr_, sizeof(sockaddr_in)) < 0)  {
                std::cout << status_handler_ << "Bind error" <<  std::endl;
            } else {
                std::cout << status_handler_ << "Bind success" <<  std::endl;
            }
            if (need_to_connect) {
                if ( connect(sockfd_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_)) < 0)  {
                    std::cout << status_handler_ << "connect error" << std::endl;
                } else {
                    std::cout << status_handler_ << " connect success" <<  std::endl;
                }
            }
        }
        ~UDPServerHandler() {
            close(sockfd_);
        }

        std::size_t udp_recvfrom(unsigned char* tmp_buffer, std::size_t len) {
            return recvfrom(sockfd_, tmp_buffer, len, MSG_WAITALL, (struct sockaddr *) &serv_addr_,  (socklen_t*) &addrlen_);
        }

        // std::size_t udp_sendto(unsigned char* tmp_buffer, std::size_t len) {
        //     return sendto(sockfd_, tmp_buffer, len, MSG_CONFIRM, (struct sockaddr *) &serv_addr_,  (socklen_t) addrlen_);
        // }

    private:
        int sockfd_, portno_;
        struct sockaddr_in serv_addr_;
        int addrlen_;
        bool need_to_connect_;
        std::string ip_addr_;
        std::string status_handler_;
};


class UDPClientHandler {
    public:
        UDPClientHandler(int port, bool need_to_connect, std::string ip_addr):
        portno_(port),
        need_to_connect_(need_to_connect),
        ip_addr_(ip_addr) {
            status_handler_ = "[udp@"+ip_addr+":"+std::to_string(port)+"]: ";
            addrlen_ = sizeof(serv_addr_);
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd_ < 0)
                std::cout << status_handler_ << "ERROR open socket" << std::endl;

            serv_addr_.sin_family    = AF_INET;
            serv_addr_.sin_port = htons(portno_); 
            serv_addr_.sin_addr.s_addr = inet_addr(ip_addr_.c_str());
            if (need_to_connect) {
                if ( connect(sockfd_, (struct sockaddr *)&serv_addr_, sizeof(serv_addr_)) < 0)  {
                    std::cout << status_handler_ << "connect error" << std::endl;
                } else {
                    std::cout << status_handler_ << " connect success" <<  std::endl;
                }
            }
        }
        ~UDPClientHandler() {
            close(sockfd_);
        }

        std::size_t udp_sendto(unsigned char* tmp_buffer, std::size_t len) {
            return sendto(sockfd_, tmp_buffer, len, MSG_CONFIRM, (struct sockaddr *) &serv_addr_,  (socklen_t) addrlen_);
        }

    private:
        int sockfd_, portno_;
        struct sockaddr_in serv_addr_;
        int addrlen_;
        bool need_to_connect_;
        std::string ip_addr_;
        std::string status_handler_;
};
#endif