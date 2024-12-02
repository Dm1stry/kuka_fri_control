#ifndef API_SERVER_HPP
#define API_SERVER_HPP

#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <functional>

#include <boost/asio.hpp>

#include <nlohmann/json.hpp>

namespace ExtCon
{
    using namespace std;
    using namespace boost::asio;
    using namespace boost::asio::ip;
    using json = nlohmann::json;

    class IJSONConvertible
    {
    public:
        virtual json toJSON() = 0;
        virtual bool fromJSON(json data) = 0;
        virtual ~IJSONConvertible();
    };

    /*
    There is two ways to use APIServer:
    1) As is, just create object and send or stream data through it as string, json or pure bytes
    2) Use it like parent of your object and register it's methods like receive callbacks
    */

    class APIServer
    {
    public:
        APIServer(unsigned int server_port=17576, unsigned int client_port=17575, std::string client_ip="127.0.0.1");
        ~APIServer();

        // Send single portion of the data
        bool sendData(string string_data);
        bool sendData(json json_data);
        bool sendData(char* data, const unsigned int data_len);

        // Receive what appeared in socket and save it like json, string or byta array
        bool receiveData(string& data);
        bool receiveData(char * data, const unsigned int data_len);
        bool receiveData(json& data);

        // Save pointer to some data and stream it like json, like bytes or like a string each dt 
        bool streamData(shared_ptr<IJSONConvertible> data_ptr, std::chrono::microseconds dt);
        bool streamData(shared_ptr<IJSONConvertible> data_ptr, unsigned int dt); //dt microseconds
        bool endStreamData();

        //Set function that handles received data
        //bool registerReceiveCallback(function<void(json data)> callback);
        bool registerReceiveCallback(function<void(json jdata)> callback);
        //void registerReceiveCallback(std::function<> callback)
    
        void on_read(const boost::system::error_code & err, std::size_t read_bytes);
        void on_send(const std::chrono::microseconds& dt, const boost::system::error_code & err, std::size_t read_bytes);
    private:
        //2 сокета и 2 потока, один на отправку данных, второй на прием (однажды это надо будет пофиксить)
        udp::endpoint endpoint_;
        io_service send_io_service_;
        unique_ptr<boost::asio::ip::udp::socket> send_socket_ptr_;

        io_service receive_io_service_;
        unique_ptr<boost::asio::ip::udp::socket> receive_socket_ptr_;
        //unique_ptr<boost::asio::ip::udp::socket> sending_socket_ptr_;

        shared_ptr<IJSONConvertible> stream_data_ptr_;
        string current_data_;

        thread stream_thread_;
        bool stream_thread_state_;
        thread read_thread_;
        bool read_thread_state_;
        
        mutex socket_lock_;

        string receive_buffer_;

        function<void(json data)> callback_;
        
    };

};
#endif