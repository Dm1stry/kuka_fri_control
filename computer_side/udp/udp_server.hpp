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
#include <fcntl.h>

#include "json.hpp"
#include <Eigen/Dense>

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

        bool server_started_ = false;   // is receive started

        int sockfd_ = -1; 
        char buffer_[1024]; 

        struct sockaddr_in servaddr_;
        struct sockaddr_in cliaddr_;
        struct sockaddr_in temp_addr_;

        socklen_t len_;
        socklen_t temp_len_;
	    ssize_t n_; 
        
        void closeSocket();

    public: 
        UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip = "0.0.0.0", unsigned long client_port = 8080);
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
    client_port_(client_port)
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
        if (server_started_) {
            return;
        }

        if ( (sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
            std::cerr << "Ошибка при создании сокета" << std::endl; 
            return; 
        } 

        int flags = fcntl(sockfd_, F_GETFL, 0);
        if (flags < 0 || fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::cerr << "Ошибка настройки неблокирующего сокета" << std::endl;
            closeSocket();
            return;
        }

        int reuse = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            std::cerr << "Ошибка настройки SO_REUSEADDR" << std::endl;
            closeSocket();
            return;
        }

        timeval receive_timeout{};
        receive_timeout.tv_sec = 0;
        receive_timeout.tv_usec = 10000;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout)) < 0) {
            std::cerr << "Ошибка настройки таймаута приема" << std::endl;
            closeSocket();
            return;
        }
        
        memset(&servaddr_, 0, sizeof(servaddr_)); 
        memset(&cliaddr_, 0, sizeof(cliaddr_)); 
        memset(&temp_addr_, 0, sizeof(temp_addr_)); 
        
        servaddr_.sin_family = AF_INET; // IPv4 
        // servaddr.sin_addr.s_addr = INADDR_ANY; 
        if (inet_pton(AF_INET, server_ip_.c_str(), &servaddr_.sin_addr) != 1) {
            std::cerr << "Некорректный IP сервера: " << server_ip_ << std::endl;
            closeSocket();
            return;
        }
        servaddr_.sin_port = htons(server_port_); 

        cliaddr_.sin_family = AF_INET; // IPv4 
        // cliaddr_.sin_addr.s_addr = INADDR_ANY; 
        if (inet_pton(AF_INET, client_ip_.c_str(), &cliaddr_.sin_addr) != 1) {
            std::cerr << "Некорректный IP клиента: " << client_ip_ << std::endl;
            closeSocket();
            return;
        }
        cliaddr_.sin_port = htons(client_port_); 
        
        // Bind the socket with the server address 
        if ( bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0 ) 
        { 
            std::cerr << "Ошибка привязки сокета" << std::endl;
            closeSocket();
            return;
        } 

        len_ = sizeof(cliaddr_); //len is value/result
        temp_len_ = sizeof(temp_addr_); //len is value/result

        server_started_ = true;
    }

    template <size_t receiv, size_t transmit>
    void UDPServer<receiv,transmit>::stop()
    {	
        if (server_started_) 
        {
            server_started_ = false;
        }
        closeSocket();
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
        if (!server_started_) {
            return false;
        }

        bool received = false;
        while (true) {
            temp_len_ = sizeof(temp_addr_);
            n_ = recvfrom(sockfd_, (char *)buffer_, sizeof(buffer_) - 1,
                          0, (struct sockaddr *) &temp_addr_, &temp_len_);

            if (n_ < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    break;
                }

                perror("recvfrom");
                break;
            }

            if (n_ == 0) {
                continue;
            }

            buffer_[n_] = '\0';
            std::string str(buffer_);

            try {
                json j = json::parse(str);
                Eigen::ArrayXd parsed = jsonToEigenArray(j);

                if (parsed.size() != receiv) {
                    std::cerr << "Некорректный размер сообщения: ожидалось "
                              << receiv << ", получено " << parsed.size() << std::endl;
                    continue;
                }

                command = parsed;
                received = true;
            } catch (const std::invalid_argument&) {
                std::cerr << "Некорректное сообщение: не число: " << str << std::endl;
            } catch (const std::out_of_range&) {
                std::cerr << "Число вне допустимого диапазона: " << str << std::endl;
            } catch (const json::exception& err) {
                std::cerr << "Некорректный JSON: " << err.what() << std::endl;
            }
        }

        return received;
    }

    template <size_t receiv, size_t transmit>
    bool UDPServer<receiv,transmit>::setMsg(Eigen::Array<double,transmit,1> &thetta)
    {
        if (!server_started_) {
            return false;
        }

        std::string message = eigenArrayToJson(thetta).dump();
        if (sendto(sockfd_, message.c_str(), message.size(), MSG_CONFIRM,
                   (const struct sockaddr *) &cliaddr_, sizeof(cliaddr_)) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return false;
            }

            perror("sendto");
            return false;
        }

        return true;
    }
}

#endif
