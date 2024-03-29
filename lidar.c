#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <SDL2/SDL.h>
#include <signal.h>

// Define constants based on the LiDAR data protocol
#define START_CHARACTER 0x54
#define DATA_LENGTH 47

int serial_port;

// Global variables for SDL window and renderer
SDL_Window *window = NULL;
SDL_Renderer *renderer = NULL;

// Global variable to control the main loop
volatile sig_atomic_t keep_running = 1;

// Signal handler for SIGINT
void handle_sigint(int sig) {
    keep_running = 0;
}

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

        if (read_byte == START_CHARACTER) {
            printf("Starting new packet.\n");
            packetIndex = 0;
            isReadingPacket = 1;
        }

        if (isReadingPacket) {
            dataPacket[packetIndex] = read_byte;
            packetIndex++;

            if (packetIndex == DATA_LENGTH) {
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
    startAngle = startAngle / 100;
    printf("Start angle: %d\n", startAngle);
    unsigned char groups[12][3];
    for (int i = 0; i < 12; i++) {
        groups[i][0] = data[6 + i*3];
        groups[i][1] = data[7 + i*3];
        groups[i][2] = data[8 + i*3];
    }
    for (int i = 0; i < 12; i++) {
        // printf("Group %d: %d, %d, %d\n", i, groups[i][0], groups[i][1], groups[i][2]);
        int distance = (groups[i][1] << 8) | groups[i][0];
        int quality = groups[i][2];
        printf("Distance: %d, Quality: %d\n", distance, quality);
    }

    // Check if we have completed a rotation
    static int lastEndAngle = 0;
    int endAngle = (data[43] << 8) | data[42];
    endAngle = endAngle / 100;

    // If the end angle is less than the last end angle, we assume a rotation has completed
    if (endAngle < lastEndAngle) {
        // Clear the renderer to remove old points
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Set draw color to black
        SDL_RenderClear(renderer); // Clear the renderer with the current draw color
    }

    lastEndAngle = endAngle;

    // After processing the data, call visualizeLidarData to update the window
    visualizeLidarData(data);
}

// Function to visualize the LiDAR data with the origin at the center of the GUI
void visualizeLidarData(unsigned char *data) {
    int startAngle = (data[5] << 8) | data[4];
    startAngle = startAngle / 100;
    // Get the end angle from the data
    int endAngle = (data[43] << 8) | data[42];
    endAngle = endAngle / 100;

    // Calculate the angle increment
    int angleIncrement = (endAngle - startAngle) / (12 - 1);

    unsigned char groups[12][3];
    for (int i = 0; i < 12; i++) {
        groups[i][0] = data[6 + i*3];
        groups[i][1] = data[7 + i*3];
        groups[i][2] = data[8 + i*3];
    }

    // Get the window size
    int window_width, window_height;
    SDL_GetWindowSize(window, &window_width, &window_height);

    // Calculate the center of the window
    int center_x = window_width / 2;
    int center_y = window_height / 2;

    // Draw the point on the window with the origin at the center
    for (int i = 0; i < 12; i++) {
        int distance = (groups[i][1] << 8) | groups[i][0];
        if (distance < 1000) {
            int angle = startAngle + i * angleIncrement; // Use the angle increment
            // Convert polar coordinates to Cartesian
            int x = center_x + (int)(distance * cos(angle * M_PI / 180.0));
            int y = center_y - (int)(distance * sin(angle * M_PI / 180.0)); // Invert y to account for SDL's top-left origin
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            SDL_RenderDrawPoint(renderer, x, y);
        }
    }

    // Event processing
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) {
            SDL_DestroyRenderer(renderer);
            SDL_DestroyWindow(window);
            SDL_Quit();
            exit(0);
        }
    }
    // Update your rendering if needed
    SDL_RenderPresent(renderer);
}

int main() {
    // Set up signal handler
    signal(SIGINT, handle_sigint);

    // Initialize LiDAR
    initializeLidar();

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }

    // Create window and renderer
    if (SDL_CreateWindowAndRenderer(500, 500, 0, &window, &renderer) != 0) {
        fprintf(stderr, "SDL_CreateWindowAndRenderer Error: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    unsigned char dataPacket[DATA_LENGTH]; // Adjust size as necessary based on the data packet structure

    while (keep_running) {
        readLidarDataPacket(dataPacket);
        processLidarData(dataPacket);
        visualizeLidarData(dataPacket);

        usleep(10); // Example: 10us delay
    }

    // Clean up SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    // Close the serial port
    close(serial_port);

    return 0;
}
