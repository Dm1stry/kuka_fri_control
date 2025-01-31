#include "udp_server.hpp"

using namespace server;

UDPServer::UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip, unsigned long client_port):
server_ip_(server_ip),
server_port_(server_port),
client_ip_(client_ip),
client_port_(client_port)
{
    
}

UDPServer::~UDPServer()
{
	stop();
}

void UDPServer::start()
{	
	server_started_ = true;
    reciev_ = std::jthread(&UDPServer::run, this);
}

void UDPServer::stop()
{	
	if (server_started_) 
	{
		server_started_ = false;
		if (reciev_.joinable()) {
			reciev_.join();
		}
		closeSocket();
	}
}

void UDPServer::run()
{
	if ( (sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		std::cerr << "Ошибка при создании сокета" << std::endl; 
		return; 
	} 
	
	memset(&servaddr_, 0, sizeof(servaddr_)); 
	memset(&cliaddr_, 0, sizeof(cliaddr_)); 
	
	servaddr_.sin_family = AF_INET; // IPv4 
	// servaddr.sin_addr.s_addr = INADDR_ANY; 
	inet_pton(AF_INET, server_ip_.c_str(), &servaddr_.sin_addr);
	servaddr_.sin_port = htons(server_port_); 

	cliaddr_.sin_family = AF_INET; // IPv4 
	// servaddr.sin_addr.s_addr = INADDR_ANY; 
	inet_pton(AF_INET, client_ip_.c_str(), &cliaddr_.sin_addr);
	cliaddr_.sin_port = htons(client_port_); 
	
	// Bind the socket with the server address 
	if ( bind(sockfd_, (const struct sockaddr *)&servaddr_, 
			sizeof(servaddr_)) < 0 ) 
	{ 
		std::cerr << "Ошибка привязки сокета" << std::endl;
		close(sockfd_);
		sockfd_ = -1; 
		return;
	} 

	len_ = sizeof(cliaddr_); //len is value/result

    while (1)
	{
		n_ = recvfrom(sockfd_, (char *)buffer_, 1024, 
					MSG_WAITALL, ( struct sockaddr *) &cliaddr_, 
					&len_); 
		buffer_[n_] = '\0'; 

		// printf("Client : %s\n", buffer); 
		// sendto(sockfd, (const char *)(&num), 4, 
		// 	MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
		// 		len); 

		std::string str(buffer_);

		try 
		{
            double d =  std::stod(str);
			angle_ = int(d*10000);
			// mtx_.lock();
			// messageQueue_.push(d);
			// mtx_.unlock();

		} catch (const std::invalid_argument&) {
			std::cerr << "Некорректное сообщение: не число: " << str << std::endl;
		} catch (const std::out_of_range&) {
			std::cerr << "Число вне допустимого диапазона: " << str << std::endl;
        } 
	}
}

void UDPServer::closeSocket() 
{
	if (sockfd_ >= 0) {
		close(sockfd_);
		sockfd_ = -1;
	}
}

bool UDPServer::getNumber(double& number) {
	// mtx_.lock();
	// if (messageQueue_.empty()) {
	// 	mtx_.unlock();
	// 	return false;
	// }
	// number = messageQueue_.front();
	// messageQueue_.pop();
	// mtx_.unlock();
	number = double(angle_)/10000;

	return true;
}