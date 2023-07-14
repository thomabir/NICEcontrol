#include <iostream>
#include <fstream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define BUFFER_SIZE 1024

int main() {
    // Create a UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket." << std::endl;
        return 1;
    }

    // Set up the server address
    struct sockaddr_in serverAddr;
    std::memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    int PortNumber = 8;
    serverAddr.sin_port = htons(PortNumber);  // Replace with the desired port number
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);  // Listen on all network interfaces

    // Bind the socket to the server address
    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        return 1;
    }

    // Open the output file
    std::ofstream outputFile("output.csv");
    if (!outputFile) {
        std::cerr << "Failed to open output file." << std::endl;
        return 1;
    }

    int count = 0;
    int prev = 0;

    // Receive and write data to the file
    char buffer[BUFFER_SIZE];
    while (true) {

        count ++;

        // Receive data
        struct sockaddr_in clientAddr;
        socklen_t clientAddrLen = sizeof(clientAddr);
        int numBytes = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&clientAddr, &clientAddrLen);

        // Check for errors
        if (numBytes < 0) {
            std::cerr << "Error receiving data." << std::endl;
            break;
        }

        // Convert received data to vector of 10 ints
        int receivedData[10];
        std::memcpy(receivedData, buffer, sizeof(int)*10);

        // two arrays: counts, and data
        // in each element of receivedData, the 8 MSBs are the count, and the 24 LSBs are the data
        int counts[10];
        int data[10];
        for (int i = 0; i < 10; i++) {
            // print reveived data[i]
            // std::cout << receivedData[i] << std::endl;

            counts[i] = receivedData[i] >> 24;
            data[i] = receivedData[i] & 0x00FFFFFF;
        }


        // check that received data is exactly one more than prev
        // if not, print "skipped"
        // if (receivedData != prev + 1) {
        //     std::cout << "skipped: " << receivedData-prev-1 << std::endl;
        // }
        // else {
        //     std::cout << "correctly received: " << receivedData << std::endl;
        // }

        // print it
        // std::cout << receivedData << std::endl;

        // Write to the file, 1 line per received data
        for (int i = 0; i < 10; i++) {
            outputFile << counts[i] << "," << data[i] << std::endl;
        }

        // store the current value in prev
        // prev = receivedData;

        // every 64k counts, print a *
        if (count % 6400 == 0) {
            std::cout << count << std::endl;
        }
    }

    // Close the output file
    outputFile.close();

    // Close the socket
    // close(sockfd);

    return 0;
}