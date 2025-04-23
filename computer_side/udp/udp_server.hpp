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

#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include "../lockfree/lockfree.hpp"

using json = nlohmann::json;

namespace server
{
    template <size_t receiv=7, size_t transmit=7>
    class UDPServer
    {
    private:
        std::string server_ip_;         // own server ip
        unsigned long server_port_;     // own server port

        std::string client_ip_;         // client ip
        unsigned long client_port_;     // client port

        std::atomic<bool> server_started_ = false;   // is receive started
        std::atomic<bool> msg_ready_ = false;

        int sockfd_; 
        char buffer_[1024]; 

        struct sockaddr_in servaddr_;
        struct sockaddr_in cliaddr_;
        struct sockaddr_in temp_addr_;

        std::jthread receiv_;           // thread for receiving 
        std::jthread transmit_;         // thread for transmit

        socklen_t len_;
        socklen_t temp_len_;
	    int n_; 
        
		// -----------------------------------------------------------------------
        // ---------------------------------------------------- type received data
		// -----------------------------------------------------------------------

        Eigen::Array<double,receiv,1> control_msg_;
        ring_buffer<Eigen::Array<double,receiv,1>> reciev_buffer_;

        // -----------------------------------------------------------------------
        // ------------------------------------------------- type transmitted data
		// -----------------------------------------------------------------------

        Eigen::Array<double,transmit,1> thetta_msg_;
        ring_buffer<Eigen::Array<double,transmit,1>> transmit_buffer_;

		// -----------------------------------------------------------------------
        
        void closeSocket();
        void run_receive();
        void run_transmit();

    public: 
        UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip = INADDR_ANY, unsigned long client_port = 8080);
        ~UDPServer();

        void start();
        void stop();

        bool getMsg(Eigen::Array<double,receiv,1> &command);
        bool setMsg(Eigen::Array<double,transmit,1> &thetta);
    };

    json eigenArrayToJson(const Eigen::ArrayXd& array);

    Eigen::ArrayXd jsonToEigenArray(const json& j);
    
    // ======================================================================
    // ======================================================================
    // ======================================================================

    template <size_t receiv, size_t transmit>
    UDPServer<receiv,transmit>::UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip, unsigned long client_port):
    server_ip_(server_ip),
    server_port_(server_port),
    client_ip_(client_ip),
    client_port_(client_port),
    transmit_buffer_(1024),
    reciev_buffer_(1024)
    {
        
    }

    template <size_t receiv, size_t transmit>
    UDPServer<receiv,transmit>::~UDPServer()
    {
        stop();
    }

    template <size_t receiv, size_t transmit>
    void UDPServer<receiv,transmit>::start()
    {	
        server_started_ = true;

        if ( (sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            std::cerr << "Ошибка при создании сокета" << std::endl; 
            return; 
        } 
        
        memset(&servaddr_, 0, sizeof(servaddr_)); 
        memset(&cliaddr_, 0, sizeof(cliaddr_)); 
        memset(&temp_addr_, 0, sizeof(temp_addr_)); 
        
        servaddr_.sin_family = AF_INET; // IPv4 
        // servaddr.sin_addr.s_addr = INADDR_ANY; 
        inet_pton(AF_INET, server_ip_.c_str(), &servaddr_.sin_addr);
        servaddr_.sin_port = htons(server_port_); 

        cliaddr_.sin_family = AF_INET; // IPv4 
        // cliaddr_.sin_addr.s_addr = INADDR_ANY; 
        inet_pton(AF_INET, client_ip_.c_str(), &cliaddr_.sin_addr);
        cliaddr_.sin_port = htons(client_port_); 
        
        // Bind the socket with the server address 
        if ( bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0 ) 
        { 
            std::cerr << "Ошибка привязки сокета" << std::endl;
            close(sockfd_);
            sockfd_ = -1; 
            return;
        } 

        len_ = sizeof(cliaddr_); //len is value/result
        temp_len_ = sizeof(temp_addr_); //len is value/result

        receiv_ = std::jthread(&UDPServer::run_receive, this);
        transmit_ = std::jthread(&UDPServer::run_transmit, this);
    }

    template <size_t receiv, size_t transmit>
    void UDPServer<receiv,transmit>::stop()
    {	
        if (server_started_) 
        {
            server_started_ = false;
            if ((receiv_.joinable())&&(transmit_.joinable())) {
                receiv_.join();
                transmit_.join();
            }
            closeSocket();
        }
    }

    template <size_t receiv, size_t transmit>
    void UDPServer<receiv,transmit>::run_receive()
    {
        while (server_started_)
        {
            n_ = recvfrom(sockfd_, (char *)buffer_, 1024, 
                        MSG_WAITALL, ( struct sockaddr *) &temp_addr_, 
                        &temp_len_); 
            buffer_[n_] = '\0';

            // std::cout << buffer_ << std::endl;

            // -----------------------------------------------------------------------
            // ---------------------------------------------- processing received data
            // -----------------------------------------------------------------------

            std::string str(buffer_);

            try {
                json j = json::parse(str);

                Eigen::Array<double,receiv,1> command = jsonToEigenArray(j);
                reciev_buffer_.push(command);

            } catch (const std::invalid_argument&) {
                std::cerr << "Некорректное сообщение: не число: " << str << std::endl;
            } catch (const std::out_of_range&) {
                std::cerr << "Число вне допустимого диапазона: " << str << std::endl;
            } 
        }
    }

    template <size_t receiv, size_t transmit>
    void UDPServer<receiv,transmit>::run_transmit()
    {
        const char* ch;
        while (server_started_)
        {
            if (msg_ready_)
            {
                // -----------------------------------------------------------------------
                // ------------------------------------------- processing transmitted data
                // -----------------------------------------------------------------------

                if (transmit_buffer_.pop(thetta_msg_))
                {
                    // std::cout << "AAAAAAAAAAAAAA" << thetta_msg_ << std::endl;
                    // ch = server::eigenArrayToJson(thetta_msg_).dump().c_str();
                    // std::cout << "AAAAAAAAAAAAAA" << server::eigenArrayToJson(thetta_msg_).dump().c_str() << std::endl;


                    // REWRITE

                    std::cout << eigenArrayToJson(thetta_msg_).dump().c_str() << std::endl;
                    
                    sendto(sockfd_, eigenArrayToJson(thetta_msg_).dump().c_str(), strlen(eigenArrayToJson(thetta_msg_).dump().c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr_, sizeof(cliaddr_)); 
                    msg_ready_ = false;
                }
            }
        }
    }

    template <size_t receiv, size_t transmit>
    void UDPServer<receiv,transmit>::closeSocket() 
    {
        if (sockfd_ >= 0) {
            close(sockfd_);
            sockfd_ = -1;
        }
    }

    template <size_t receiv, size_t transmit>
    bool UDPServer<receiv,transmit>::getMsg(Eigen::Array<double,receiv,1> &command)
    {
        return reciev_buffer_.pop(command);
    }

    template <size_t receiv, size_t transmit>
    bool UDPServer<receiv,transmit>::setMsg(Eigen::Array<double,transmit,1> &thetta)
    {

        msg_ready_ = transmit_buffer_.push(thetta);

        return true;
    }
}

#endif