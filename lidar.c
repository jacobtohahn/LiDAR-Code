#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

// Define constants based on the LiDAR data protocol
#define START_CHARACTER 0x54
#define DATA_LENGTH 9

int serial_port;

// Function to initialize the LiDAR sensor
void initializeLidar() {
    // Open the serial port in read/write mode
    serial_port = open("/dev/ttyS0", O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(1);
    }
}

// Function to read a data packet from the LiDAR sensor
void readLidarDataPacket(unsigned char *dataPacket) {
    // Implement reading logic here
    // For example, read from the serial port into dataPacket
    unsigned char read_byte;
    read(serial_port, &read_byte, 1);
    printf("%x\n", read_byte);
    if (read_byte == 0x54) { 
        printf("START: " + read_byte);
    }
    else {
        printf(read_byte);
    }
}

// Function to process the LiDAR data and convert it into a top-down view
void processLidarData(unsigned char *data) {
    // Implement data processing logic here
    // Example: int distance = (data[2] << 8) | data[3];
}

int main() {
    initializeLidar();

    unsigned char dataPacket[9]; // Adjust size as necessary based on the data packet structure

    while (1) {
        readLidarDataPacket(dataPacket);
        processLidarData(dataPacket);

        // usleep(10); // Example: 10us delay
    }

    close(serial_port); // Close the serial port when done
    return 0;
}
