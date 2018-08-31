/*! define the SocketServer on APR robot
* the class is the combination of
* Filename: SocketServer.h
* Version: 0.20
* Algo team, Ninebot Inc., 2017
*/

#ifndef NINEBOT_ALGO_SOCKET_SERVER_H_
#define NINEBOT_ALGO_SOCKET_SERVER_H_

#include <thread>
#include <mutex>
#include <string>
#include "RawDataUtil.h"

namespace ninebot_algo {
	class NINEBOT_EXPORT SocketServer {
	public :
		SocketServer(int port = 8081);
		SocketServer(bool is_send_timeout, bool is_recv_timeout, int port = 8081);
		virtual ~SocketServer();

		int recvChars(char* recv_chars, const int length);
		int sendChars(const char* send_chars, const int length);
		std::string receiveLine();
		int sendLine(std::string);
		bool isConnected();
		bool isStopped();
		bool stopSocket();
		int  getReconnectTimes();

		// call example
		void sendMessage();
		void receiveMessage();
		void sendChars();
		void receiveChars();

	private :
		int run();
		int init_socket(int port);
		void re_connected();
		void reset_connect();

	private:
		const static int RECEIVE_BUFFER_MAX = 200000;
		std::thread m_run_thread;
		std::mutex _mutex_socket_connected;
		std::mutex _mutex_socket_stopped;

		int _socket_handler;
		int _new_socket;
		int _process_socket;
		int _init_success;
		int _reconnect_times;
		bool _socket_connected;
		bool _socket_stopped;
		bool m_is_send_timeout;
		bool m_is_recv_timeout;
	};
}
#endif  // NINEBOT_ALGO_SOCKET_SERVER_H_
