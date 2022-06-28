#ifndef L_TCP_HANDLER_H
#define L_TCP_HANDLER_H
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
class TCPServerHandler {
    public:
        TCPServerHandler(int port, int buf_len, std::string ip_addr):
        portno_(port),
        buf_len_(buf_len),
        ip_addr_(ip_addr) {
            status_handler_ = "[tcp@"+ip_addr+":"+std::to_string(port)+"]: ";
            int addrlen_ = sizeof(serv_addr_);
            sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd_ < 0)
                std::cout << status_handler_ << "ERROR open socket" << std::endl;
            opt_ = 1;
            if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_, sizeof(opt_))) {
                std::cout << status_handler_ << "Socket set wrong" << std::endl;
            }
            int len = sizeof(buf_len_);
            if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &buf_len_, sizeof(buf_len_))) {
                std::cout << status_handler_ << "Socket set wrong" << std::endl;
            }
            if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &buf_len_, sizeof(buf_len_))) {
                std::cout << status_handler_ << "Socket set wrong" << std::endl;
            }
            getsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &buf_len_, (socklen_t*)&len);
            std::cout << status_handler_ << "the receive buffer size after setting is " << buf_len_ << std::endl;
            getsockopt(sockfd_, SOL_SOCKET, SO_SNDBUF, &buf_len_, (socklen_t*)&len);
            std::cout << status_handler_ << "the send buffer size after setting is " << buf_len_ << std::endl;
            serv_addr_.sin_family = AF_INET;
            serv_addr_.sin_port = htons(portno_);
            serv_addr_.sin_addr.s_addr = inet_addr(ip_addr_.c_str());

            if (bind(sockfd_, (struct sockaddr*)& serv_addr_, sizeof(serv_addr_))) {
                std::cout << status_handler_ << "Bind error" << std::endl;
            }
            if (listen(sockfd_, 3) < 0) {
                std::cout << status_handler_ << "listen error" << std::endl;
            } else {
                std::cout << status_handler_ << "listen sucess !" << std::endl;
            }
            if ((new_socket_ = accept(sockfd_, (struct sockaddr *)&serv_addr_, (socklen_t*)&addrlen_))<0) {
                std::cout << status_handler_ << "accept error" << std::endl;
            } else {
                std::cout << status_handler_ << "accept success !" << std::endl;
            }
        }

        ~TCPServerHandler() {
            close(new_socket_);
            close(sockfd_);
        }

        std::size_t tcp_read(unsigned char* read_buf_ptr, std::size_t read_len) {
            return read(new_socket_, read_buf_ptr, read_len);
        }

        void reconnect() {
            std::cout << status_handler_ << " Client Disconnect " << std::endl;
            close(new_socket_);
            std::cout << status_handler_ << " wait for reconnect " << std::endl;
            if ((new_socket_ = accept(sockfd_, (struct sockaddr *)&serv_addr_, (socklen_t*)&addrlen_))<0) {
                std::cout << status_handler_ << "accept error" << std::endl;
            }
        }
    private:
        int sockfd_, new_socket_, portno_, opt_, buf_len_;
        struct sockaddr_in serv_addr_;
        int addrlen_;
        std::string ip_addr_;
        std::string status_handler_;
};
#endif