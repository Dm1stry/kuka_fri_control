#include "apiserver.hpp"
#include <iostream>

using namespace ExtCon;
//using namespace APIServer

IJSONConvertible::~IJSONConvertible() {}

APIServer::APIServer(unsigned int server_port, unsigned int client_port, std::string client_ip)
  : endpoint_(ip::address::from_string(client_ip), client_port),
	send_socket_ptr_(new udp::socket(send_io_service_)),
	receive_socket_ptr_(new udp::socket(receive_io_service_)),
	stream_thread_state_(false),
	read_thread_state_(false)
{
	receive_buffer_.resize(1024);
	receive_socket_ptr_->open(endpoint_.protocol());
	receive_socket_ptr_->set_option(boost::asio::ip::udp::socket::reuse_address(true));
	receive_socket_ptr_->bind(udp::endpoint(ip::address::from_string("127.0.0.1"), server_port));
	send_socket_ptr_->open(endpoint_.protocol());
	send_socket_ptr_->set_option(boost::asio::ip::udp::socket::reuse_address(true));
}

APIServer::~APIServer()
{
	endStreamData();
}


//Synchronously sends data
bool APIServer::sendData(string string_data)
{
	if(send_socket_ptr_->is_open())
	{
		send_socket_ptr_->send_to(buffer(string_data), endpoint_);
		return true;
	}
	else
	{
		return false;
	}
	// TODO add meaningful return value
}

bool APIServer::sendData(json json_data)
{
	return sendData(json_data.dump());
}

bool APIServer::sendData(char* data, const unsigned int data_len)
{
	string str;
	str.assign(data, data + data_len);
	return sendData(str);
}



bool APIServer::receiveData(string& data)
{
	udp::endpoint sender_ep; 
	receive_socket_ptr_->receive_from(buffer(data, 1024), sender_ep);
	return true; // TODO add meaningful return value
}

bool APIServer::receiveData(json& data)
{
	string received_string;
	bool result = receiveData(received_string);
	data = json::parse(received_string);
	return result;
}

bool APIServer::receiveData(char * data, const unsigned int data_len)
{
	receive_socket_ptr_->receive_from(mutable_buffer(data, data_len), endpoint_);
	return true; // TODO add meaningful return value
}



void APIServer::on_send(const std::chrono::microseconds& dt, const boost::system::error_code & err, std::size_t read_bytes)
{
	using namespace std::placeholders;
	
	std::this_thread::sleep_for(dt);
	current_data_ = (stream_data_ptr_->toJSON()).dump();
	send_socket_ptr_->async_send_to(buffer(current_data_, current_data_.size()), endpoint_, std::bind(&APIServer::on_send, this, dt, _1, _2));
}

bool APIServer::streamData(shared_ptr<IJSONConvertible> data_ptr, const std::chrono::microseconds dt)
{
	using namespace std::placeholders;
	stream_data_ptr_ = data_ptr;
	stream_thread_state_ = true;
	current_data_ = (stream_data_ptr_->toJSON()).dump();
	send_socket_ptr_->async_send_to(buffer(current_data_, current_data_.size()), endpoint_, std::bind(&APIServer::on_send, this, dt, _1, _2));

	stream_thread_ = thread([&]{send_io_service_.run();});
	
	return true;
}

bool APIServer::streamData(shared_ptr<IJSONConvertible> data_ptr, unsigned int dt)
{
	return streamData(data_ptr, std::chrono::microseconds(dt));
}

bool APIServer::endStreamData()
{	
	stream_thread_.join();
	read_thread_.join();
	stream_data_ptr_.reset();
	return true;
}



void APIServer::on_read(const boost::system::error_code & err, std::size_t read_bytes)
{
	using namespace std::placeholders;
	json data = json::parse(receive_buffer_);
	callback_(data);
	receive_socket_ptr_->async_receive(buffer(receive_buffer_, 1024), std::bind(&APIServer::on_read, this, _1, _2));
}

bool APIServer::registerReceiveCallback(function<void(json jdata)> callback)
{
	using namespace std::placeholders;
	callback_ = callback;
	receive_socket_ptr_->async_receive(buffer(receive_buffer_, 1024), std::bind(&APIServer::on_read, this, _1, _2));
	read_thread_ = thread([&]{receive_io_service_.run();});
	return true;
}