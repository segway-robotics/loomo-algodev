#include "SocketServer.h"
#include "ninebot_log.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

#ifdef WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#define close closesocket
#define SHUT_RDWR SD_BOTH
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

const int TIMEOUT = 3; // this unit of time is second.

namespace ninebot_algo
{
	/*! namespace of this algorithm  */
	namespace follow_algo
	{

		SocketServer::SocketServer(int port) {
			_reconnect_times = 0;
			_socket_connected = false;
			_socket_stopped = true;
			_init_success = -1;
			m_is_recv_timeout = false;
			m_is_send_timeout = false;
			init_socket(port);
			m_run_thread = std::thread(&SocketServer::run, this);
		}

		SocketServer::SocketServer(bool is_send_timeout, bool is_recv_timeout, int port) {
			_reconnect_times = 0;
			_socket_connected = false;
			_socket_stopped = true;
			_init_success = -1;
			m_is_send_timeout = is_send_timeout;
			m_is_recv_timeout = is_recv_timeout;
			init_socket(port);
			m_run_thread = std::thread(&SocketServer::run, this);
		}

		SocketServer::~SocketServer() {
			stopSocket();
		}

		int SocketServer::init_socket(int port) {
		#ifdef WIN32
		    WSADATA info;
		    if (WSAStartup(MAKEWORD(2, 0), &info))
				ALOGD("Could not start WSA");
		#endif

			_socket_handler = socket(AF_INET, SOCK_STREAM, 0);
			if (_socket_handler == -1) {
				ALOGD("could not create socket!");
				return -1;
			}
			int reuse = 1;
			if (setsockopt(_socket_handler, SOL_SOCKET, SO_REUSEADDR, (const char *) &reuse,
							sizeof(reuse)) < 0)
				ALOGE("setsockopt(SO_REUSEPORT) failed");

			// timeout of send and receive
		#ifdef WIN32
			int timeout = TIMEOUT*1000; // the unit is ms
		#else
			struct timeval timeout = {TIMEOUT, 0}; // the unit is s.
		#endif

			if(m_is_send_timeout) {
				int ret_send_timeo = setsockopt(_socket_handler, SOL_SOCKET,SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
				if(ret_send_timeo < 0) {
					ALOGE("setsockopt(SO_SNDTIMEO) failed");
				}
				else {
					ALOGD("setsockopt(SO_SNDTIMEO) success");
				}
			}

			if(m_is_recv_timeout) {
				int ret_recv_timeo = setsockopt(_socket_handler, SOL_SOCKET,SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
				if(ret_recv_timeo < 0) {
					ALOGE("setsockopt(SO_RCVTIMEO) failed");
				}
				else {
					ALOGD("setsockopt(SO_RCVTIMEO) success");
				}
			}

			_process_socket = -1;

			struct sockaddr_in server;
			server.sin_family = AF_INET;
			server.sin_addr.s_addr = INADDR_ANY;
			server.sin_port = htons(port);
			int ret = bind(_socket_handler, (struct sockaddr *) &server, sizeof(server));
			if (ret < 0) {
				ALOGE("bind failed. Error");
				return -1;
			}

			ret = listen(_socket_handler, 5);
			if (ret < 0) {
				ALOGE("error listening");
				return -1;
			}

			_socket_stopped = false;
			_init_success = 0;

			return 0;
		}

		int SocketServer::getReconnectTimes() {
			return _reconnect_times;
		}

		void SocketServer::re_connected() {
			ALOGD("reconnect %d times!\n", _reconnect_times);
			_reconnect_times++;
		}

		void SocketServer::reset_connect() {
			ALOGD("reset_connect!!");
			std::lock_guard<std::mutex> lock(_mutex_socket_connected);
			_socket_connected = false;

			return ;
		}

		bool SocketServer::stopSocket() {
			if (!_socket_stopped) {
				ALOGD("DEBUG >> ~stopSocket 1");
				{
					std::lock_guard<std::mutex> lock(_mutex_socket_stopped);
					_socket_stopped = true;
				}
				shutdown(_socket_handler, SHUT_RDWR);
				close(_socket_handler);

				m_run_thread.join();
				ALOGD("DEBUG >> ~stopSocket 2");
			}

			return true;
		}

		int SocketServer::run() {
			if (_init_success < 0) {
				_socket_connected = false;
				return -1;
			}

			while (true) {
				if (isStopped()) {
					if (_process_socket != -1) {
						close(_process_socket);
					}
					return -1;
				}

				//ALOGD("update _process_socket: %d\n", _process_socket);
				int sockaddr_size = sizeof(struct sockaddr_in);
				struct sockaddr_in client;
				_new_socket = accept(_socket_handler, (struct sockaddr *) &client,
					reinterpret_cast<socklen_t *>(&sockaddr_size));

				//ALOGD("update _new_socket: %d", _new_socket);

				if (_new_socket < 0) {
					//ALOGE("accept failed");
					continue;
				}

				if (_process_socket != -1) {
					close(_process_socket);
				}

				_process_socket = _new_socket;
				{
					std::lock_guard<std::mutex> lock(_mutex_socket_connected);
					_socket_connected = true;
				}

				re_connected();
			}
		}

		int SocketServer::sendLine(std::string line) {
			line += '\n';
			int send_info = send(_process_socket, line.c_str(), line.length(), 0);
			if (send_info < 0) {
				if (!m_is_send_timeout) {
					reset_connect();
				}
			}

			return send_info;
		}

		std::string SocketServer::receiveLine() {
			std::string ret;
			while (true) {
				char r;
				switch (recv(_process_socket, &r, 1, 0))
				{
				case 0:
					return "";
				case -1:
					if (!m_is_recv_timeout) {
						reset_connect();
					}
					return "";
				}

				ret += r;
				if (r == '\n')  return ret;
			}
		}

		int SocketServer::sendChars(const char* send_chars, const int length) {
			int send_info = send(_process_socket, send_chars, length, 0);
			if (send_info < 0) {
				ALOGE("send failed");
				if (!m_is_send_timeout) {
					reset_connect();
				}
			}

			return send_info;
		}

		// recv_chars already malloc memory.
		int SocketServer::recvChars(char* recv_chars, const int length) {
			if (length > 0 && recv_chars) {
				memset(recv_chars, 0, length);
				int i = 0;
				for (i = 0; i < length; ++i) {
					char r;
					switch (recv(_process_socket, &r, 1, 0)) {
					case 0:
						if (!m_is_recv_timeout) {
							reset_connect();
						}
						return 0;
					case -1:
						if (!m_is_recv_timeout) {
							reset_connect();
						}
						return -1;
					default:
						break;
					}

					recv_chars[i] = r;
				}

				return i;
			}

			return -1;
		}

		bool SocketServer::isConnected() {
			std::lock_guard<std::mutex> lock(_mutex_socket_connected);
			return _socket_connected;
		}

		bool SocketServer::isStopped() {
			std::lock_guard<std::mutex> lock(_mutex_socket_stopped);

			return _socket_stopped;
		}

		void SocketServer::sendMessage() {
			while (true) {
				if (_socket_connected == false) {
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
					continue;
				}

				if (_socket_stopped)
					return;

				// send to client
				std::string send_string = std::string("sample line");
				int send_info = sendLine(send_string);

				if (send_info < 0)
					ALOGD("Disconnect client!");

				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}

			return;
		}

		void SocketServer::receiveMessage() {
			int i = 0;
			while (true) {
				if (_process_socket == -1) {
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
					continue;
				}

				while (true) {
					if (_socket_stopped)
						return;

					std::string l = receiveLine();
					if (l.empty()) {
						ALOGD("client send empty");
						std::this_thread::sleep_for(std::chrono::milliseconds(500));
						break;
					}
				}
			}

			return;
		}

		void SocketServer::sendChars() {
			while (true) {
				if (isStopped())
					break;

				if (!isConnected()) {
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
					continue;
				}

				// send to client
				const char buffer[] = "sample chars";
				int send_info = sendChars(buffer, sizeof(buffer));
				if (send_info < 0)
					ALOGD("Disconnect server!");

				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}

			return;
		}

		void SocketServer::receiveChars() {
			char buffer[50];

			while (true) {
				if (isStopped())
					break;

				if (!isConnected()) {
					std::this_thread::sleep_for(std::chrono::milliseconds(500));
					continue;
				}

				int recv_size = recvChars(buffer, sizeof(buffer));

				if (recv_size <= 0) {
					ALOGD("client send empty");
					std::this_thread::sleep_for(std::chrono::milliseconds(30));
					continue;
				}
			}

			return;
		}

		int SocketServer::recvFloats(float* recv_floats, const int length) {
			int recv_info = recv(_process_socket, (char*)(recv_floats), length*sizeof(float), 0);
			if (recv_info < 0) {
				ALOGE("recvFloats failed");
				return -1;
			}
			return 1;
		}

		int SocketServer::sendFloats(const float* send_floats, const int length) {
			float* buffer = new float[length];
			memcpy(buffer, send_floats, length*sizeof(float));
			int send_info = send(_process_socket, (char *)(buffer), length*sizeof(float), 0);
			if (send_info < 0) {
				ALOGE("send failed");
				delete[] buffer;
				return -1;
			}
			delete[] buffer;
			return send_info;	
		}

		int SocketServer::sendImage(cv::Mat image, const int width, const int height, const int channels, const int bags) {
			
			if (image.empty()){
				ALOGE("send failed, empty image");
				return -1;
			}

			if(image.cols != width || image.rows != height || image.channels() != channels){
				ALOGE("send failed, format error");
				return -1; 
			}

			const int sz_image = height * width * channels* sizeof(uint8_t) / bags;
			char data[sz_image];
			memset(&data, 0, sizeof(data));	

			for (int k = 0; k < bags; ++k)   
			{
				// convert
				int num1 = height / bags * k;  
				for (int i = 0; i < height / bags; ++i)
				{  
					int num2 = i * width * channels;  
					uchar* ucdata = image.ptr<uchar>(i + num1);  
					for (int j = 0; j < width * channels; ++j)  
					{  
						data[num2 + j] = ucdata[j];  
					}  
				}  

				if (send(_process_socket, (char *)(&data), sizeof(data), 0) < 0)  
				{
					ALOGE("send failed, communication error");
					return -1;  
				}  
			}

			return 1;
		}

	}
}