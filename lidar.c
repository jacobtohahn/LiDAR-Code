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
    static int packetIndex = 0; // Index to keep track of the position in the data packet
    static int isReadingPacket = 0; // Flag to check if we are in the middle of reading a packet

    unsigned char read_byte;
    int bytesRead;

    // Read bytes one at a time and check for the start character
    while ((bytesRead = read(serial_port, &read_byte, 1)) > 0) {
        if (read_byte == START_CHARACTER) {
            if (isReadingPacket) {
                // We've reached the start of a new packet, so the current one is complete
                // Process the current packet before resetting
                processLidarData(dataPacket);
            }
            // Start a new packet
            packetIndex = 0;
            isReadingPacket = 1;
        }

        if (isReadingPacket) {
            // Store the byte in the data packet array
            dataPacket[packetIndex++] = read_byte;

            // If the packet is complete (reached the expected DATA_LENGTH), reset
            if (packetIndex == DATA_LENGTH) {
                isReadingPacket = 0; // Reset the flag
                packetIndex = 0; // Reset the index for the next packet
                // Process the packet if needed, or it can be processed outside this function
            }
        }
    }

    // Handle read error if bytesRead < 0
    if (bytesRead < 0) {
        printf("Error %i from read: %s\n", errno, strerror(errno));
        // Handle the error as appropriate
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

        usleep(10); // Example: 10us delay
    }

    close(serial_port); // Close the serial port when done
    return 0;
}
