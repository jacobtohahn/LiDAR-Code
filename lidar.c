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

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        exit(1);
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication
    tty.c_cflag |= CS8; // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10; // Wait for up to 1s, returning as soon as any data is received.
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
    // Implement reading logic here
    // For example, read from the serial port into dataPacket
    unsigned char read_byte;
    read(serial_port, &read_byte, 1);
    if (read_byte == START_CHARACTER) { 
        printf("Start character found\n");
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

        usleep(100000); // Example: 100ms delay
    }

    close(serial_port); // Close the serial port when done
    return 0;
}
