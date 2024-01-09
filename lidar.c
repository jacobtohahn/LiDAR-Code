#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

// Define constants based on the LiDAR data protocol
#define START_CHARACTER 0x54
#define DATA_LENGTH 47

int serial_port;

// Function to initialize the LiDAR sensor
void initializeLidar() {
    struct termios tty;

    // Open the serial port in read/write mode
    serial_port = open("/dev/ttyS0", O_RDWR);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(1);
    }

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(1);
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 230400
    cfsetispeed(&tty, B230400);
    cfsetospeed(&tty, B230400);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }
}

// Function to read a data packet from the LiDAR sensor
void readLidarDataPacket(unsigned char *dataPacket) {
    static int packetIndex = 0;
    static int isReadingPacket = 0;

    unsigned char read_byte;
    int bytesRead;

    while ((bytesRead = read(serial_port, &read_byte, 1)) > 0) {
        printf("Read byte: 0x%X\n", read_byte); // Print each byte read for debugging

        if (read_byte == START_CHARACTER) {
            printf("Start character detected.\n");
            printf("Starting new packet.\n");
            packetIndex = 0;
            isReadingPacket = 1;
        }

        if (isReadingPacket) {
            dataPacket[packetIndex] = read_byte;
            printf("Storing byte 0x%X at index %d\n", read_byte, packetIndex);
            packetIndex++;

            if (packetIndex == DATA_LENGTH) {
                printf("Packet complete. Packet size: %d\n", packetIndex);
                printf("End of current packet, processing data...\n");
                processLidarData(dataPacket);
                isReadingPacket = 0;
                packetIndex = 0;
            }
        }
    }

    if (bytesRead < 0) {
        printf("Error %i from read: %s\n", errno, strerror(errno));
    }
}

// Function to process the LiDAR data and convert it into a top-down view
void processLidarData(unsigned char *data) {
    // Implement data processing logic here
    int startAngle = (data[5] << 8) | data[4];
    // Map the startAngle value between 0 and 360
    startAngle = startAngle % 360;
    printf("Start angle: %d\n", startAngle);
}

int main() {
    initializeLidar();

    unsigned char dataPacket[DATA_LENGTH]; // Adjust size as necessary based on the data packet structure

    while (1) {
        readLidarDataPacket(dataPacket);
        processLidarData(dataPacket);

        usleep(1); // Example: 10us delay
    }

    close(serial_port); // Close the serial port when done
    return 0;
}
