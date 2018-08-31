/**
* SocketClient.h
* Version: 0.20
* Algo team, Ninebot Inc., 2017
*/

#ifndef NINEBOT_ALGO_SOCKETCLIENT_H_
#define NINEBOT_ALGO_SOCKETCLIENT_H_

#include <string>
#include <thread>
#include <mutex>
#include "RawDataUtil.h"

namespace ninebot_algo {
	class NINEBOT_EXPORT SocketClient {
	public:
		SocketClient(std::string host = "127.0.0.1", int port = 8081);
		~SocketClient();

		bool initSocket();
		bool disconnectSocket();
		bool isConnected();
		bool isStopped();
		bool stopSocket();
		int sendLine(std::string line);
		std::string receiveLine();
		int sendChars(const char* send_chars, const int length);
		int recvChars(char* recv_chars, const int length);

		// call example
		void sendMessage();
		void receiveMessage();
		void sendChars();
		void receiveChars();

	private:

		std::string _host;
		int _port;
		int _socket;
		int _init_success;

		std::mutex _mutex_socket_connected;
		std::mutex _mutex_socket_stopped;
		bool _socket_connected;
		bool _socket_stopped;
	};
}
#endif  // NINEBOT_ALGO_SOCKETCLIENT_H_
