//
// Created by kccai.
//
#pragma once
#include "../bandit/macro_util.h"
#include "../bandit/bandit_util.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define CMPTS_PORT 9754

// client: get the measurements from the kernel socket
// 1. Establish a connection to the server.
// 2. Send data (the selected ids of the paths) to the server.
// 3. Receive data (the measurements of the paths) from the server.
// 4. Close the connection and exit.

namespace bandit {

class PathSelectionClient {
    int port;
    std::string serverIP;

public:
    PathSelectionClient(const std::string& serverIP_, const int port_ = CMPTS_PORT, )
            :serverIP(serverIP_), port(port_)
    {
    }
    // Fix: the size of the measurements
    int exchange(const std::vector<uint>& pathIDs, std::vector<Measurement>& recvMeasurements)
    {
        sockaddr_in serverAddress;
        socklen_t servLen = sizeof(serverAddress);
        int connfd = -1;
        if ((connfd = socket(AF_INET, SOCK_STREAM, 0))<0) {
            perror("socket");
            exit(1);
        }
        memset(&serverAddress, 0, sizeof(serverAddress));
        serverAddress.sin_family = AF_INET;
        // convert the port number representation
        serverAddress.sin_port = htons(port);
        //  convert the IP representation
        if (!inet_pton(AF_INET, serverIP.c_str(), &serverAddress.sin_addr)) {
            perror("inet_pton");
            exit(1);
        }
        // connect to the server.
        if (connect(connfd, (sockaddr*) &serverAddress, sizeof(sockaddr))<0) {
            perror("connect");
            exit(1);
        }
        // sending data (path ids) 
        if (write(connfd, &pathIDs[0], sizeof(uint)*pathIDs.size())<0) {
            perror("write");
            exit(1);
        }
        // receive msg from the server
        int nBytes = -1;
        if ((nBytes = read(connfd, &recvMeasurements[0], sizeof(Measurement)*recvMeasurements.size()))<0) {
            perror("read");
            exit(1);
        }
        // close the socket
        close(connfd);
        return 0;
    }

};

}// namespace
