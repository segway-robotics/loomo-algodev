/**
* SocketClient.cpp
* Version: 0.20
* Algo team, Ninebot Inc., 2017
*/

#include "SocketClient.h"

#include <stdio.h>
#include <iostream>
#include <cstring>

#ifdef WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#define close closesocket
#define SHUT_RDWR SD_BOTH
#else
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#endif

#include "ninebot_log.h"

const int RECEIVE_BUFFER_MAX = 10000;

using namespace ninebot_algo;

SocketClient::SocketClient(std::string host, int port) {
	_host = host;
	_port = port;
	_init_success = -1;
	_socket_connected = false;
	_socket_stopped = false;
}

SocketClient::~SocketClient() {
	close(_socket);
}

bool SocketClient::initSocket() {
#ifdef WIN32
	WSADATA info;
	if (WSAStartup(MAKEWORD(2, 0), &info)) {
		ALOGD("Could not start WSA");
		return false;
	}
#endif

	_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (_socket < 0) {
		ALOGE("error initializing socket");
		return false;
	}

	struct sockaddr_in server;
	server.sin_addr.s_addr = inet_addr(_host.c_str());
	server.sin_family = AF_INET;
	server.sin_port = htons(_port);
	int ret = connect(_socket, (struct sockaddr *) &server, sizeof(server));
	if (ret < 0) {
		ALOGE("connect failed. Error");
		return false;
	}

	std::lock_guard<std::mutex> lock(_mutex_socket_connected);
	_init_success = 0;
	_socket_connected = true;

	return true;
}

bool SocketClient::disconnectSocket() {
	if (_socket_connected) {
		std::lock_guard<std::mutex> lock(_mutex_socket_connected);
		_socket_connected = false;
		shutdown(_socket, SHUT_RDWR);
		close(_socket);
	}

	return true;
}

std::string SocketClient::receiveLine() {
	std::string ret;
	while (true) {
		char r;
		switch (recv(_socket, &r, 1, 0)) {
		case 0:
			return "";
		case -1:
			return "";
		}

		ret += r;
		if (r == '\n')
			return ret;
	}
}

int SocketClient::sendLine(std::string line) {
	line += '\n';
	int send_info = send(_socket, line.c_str(), line.length(), 0);
	if (send_info < 0)
		_socket_connected = false;

	return send_info;
}

int SocketClient::sendChars(const char* send_chars, const int length) {
	int send_info = send(_socket, send_chars, length, 0);
	if (send_info < 0) {
		ALOGE("send failed");
		_socket_connected = false;
	}

	return send_info;
}

// recv_chars already malloc memory.
int SocketClient::recvChars(char* recv_chars, const int length) {
	if (length > 0 && recv_chars) {
		memset(recv_chars, 0, length);
		int i = 0;
		for (i = 0; i < length; ++i) {
			char r;
			switch (recv(_socket, &r, 1, 0)) {
			case 0:
				return 0;
			case -1:
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

bool SocketClient::isConnected() {
	std::lock_guard<std::mutex> lock(_mutex_socket_connected);
	return _socket_connected;
}

bool SocketClient::isStopped() {
	std::lock_guard<std::mutex> lock(_mutex_socket_stopped);

	return _socket_stopped;
}

bool SocketClient::stopSocket() {
	if (!_socket_stopped) {
		{
			std::lock_guard<std::mutex> lock(_mutex_socket_stopped);
			_socket_stopped = true;
		}

		disconnectSocket();
	}

	return true;
}

void SocketClient::sendMessage() {
	while (true) {
		if (!isConnected())
			break;

		// send to client
		std::string send_string = std::string("sample line");
		int send_info = sendLine(send_string);

		if (send_info < 0)
			ALOGD("Disconnect server!");

		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		if (isStopped())
			break;
	}

	return;
}

void SocketClient::receiveMessage() {
	while (true) {
		if (!isConnected())
			break;
		std::string l = receiveLine();

		if (l.empty()) {
			ALOGD("server send empty");
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			continue;
		}

		if (isStopped())
			break;
	}

	return;
}

void SocketClient::sendChars() {
	while (true) {
		if (!isConnected())
			break;

		// send to client
		const char send_chars[] = "sample chars";
		int send_info = sendChars(send_chars, sizeof(send_chars));

		if (send_info < 0)
			ALOGD("Disconnect server!");

		std::this_thread::sleep_for(std::chrono::milliseconds(5));

		if (isStopped())
			break;
	}

	return;
}

void SocketClient::receiveChars() {
	char buffer[RECEIVE_BUFFER_MAX];

	while (true) {
		if (!isConnected())
			break;
		int recv_size = recvChars(buffer, sizeof(buffer));

		if (recv_size <= 0) {
			ALOGD("server send empty");
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			continue;
		}

		if (isStopped())
			break;
	}

	return;
}
