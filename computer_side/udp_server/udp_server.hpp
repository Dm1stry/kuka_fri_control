#ifndef UDP_SERVER
#define UDP_SERVER

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#include <thread>
#include <mutex>
#include <functional>
#include <atomic>

namespace server
{
    class UDPServer
    {
    private:
        std::string server_ip_;
        unsigned long server_port_;

        std::string client_ip_;
        unsigned long client_port_;

        bool server_started_ = false;

        int sockfd_; 
        char buffer_[1024]; 
        struct sockaddr_in servaddr_;
        struct sockaddr_in cliaddr_;

        std::jthread reciev_; 

        socklen_t len_;
	    int n_; 
        
        std::queue<double> messageQueue_;
        std::mutex mtx_; 

        std::atomic<int> angle_;

        void closeSocket();
        void run();

    public: 
        UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip = INADDR_ANY, unsigned long client_port = 8080);
        ~UDPServer();

        void start();
        void stop();

        bool getNumber(double& number);
    };
}

#endif