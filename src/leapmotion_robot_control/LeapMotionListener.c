/**
 * LeapMotionListener.c
 *
 * @date 14.06.2024
 * @author Joel
 *
 * This program connects to a Leap Motion device using the LeapC API. It tracks
 * hand movement data and sends that data via UDP to a specified address.
 */


#include <LeapC.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

/**
 * @struct LeapMotionListener
 * A struct holding:
 * - A pointer to a LeapC connection.
 * - Network address information for sending UDP data.
 * - A UDP socket descriptor.
 * - The ID of the currently "focused" hand.
 */

typedef struct {
    LEAP_CONNECTION* connection;    // Pointer to a LeapC connection object
    struct sockaddr_in address;     // Socket address structure for UDP
    int udp_socket;                 // File descriptor for the UDP socket
    int current_hand_id;            // Track the current hand ID
} LeapMotionListener;

/* Forward declarations of callback functions */

void on_init(LeapMotionListener* listener);
void on_connect(LeapMotionListener* listener);
void on_disconnect(LeapMotionListener* listener);
void onExit(LeapMotionListener* listener);
void on_frame(LeapMotionListener* listener, const LEAP_TRACKING_EVENT* frame);
void main_loop(LeapMotionListener* listener);

/**
 * @brief Called when the Leap Motion device is initialized.
 *
 * @param listener Pointer to the LeapMotionListener struct.
 */

void on_init(LeapMotionListener* listener) {
    printf("Initialized\n");
}

/**
 * @brief Called when the Leap Motion device successfully connects.
 *
 * @param listener Pointer to the LeapMotionListener struct.
 */

void on_connect(LeapMotionListener* listener) {
    printf("Motion Sensor Connected!\n");
}

/**
 * @brief Called when the Leap Motion device disconnects.
 *
 * @param listener Pointer to the LeapMotionListener struct.
 */

void on_disconnect(LeapMotionListener* listener) {
    printf("Motion sensor disconnected!\n");
}

/**
 * @brief Called before the application exits.
 *
 * @param listener Pointer to the LeapMotionListener struct.
 */

void onExit(LeapMotionListener* listener) {
    printf("Exited\n");
}

/**
 * @brief Processes each frame of tracking data from the Leap Motion device.
 *
 * This function loops through all hands detected in the current frame. It checks
 * if the focused hand (identified by listener->current_hand_id) is present. If so,
 * it sends the tracking data of that hand. If not, it attempts to set the first
 * detected hand as the new focus. If no hands are detected, it sends a zeroed-out
 * data packet.
 *
 * @param listener Pointer to the LeapMotionListener struct.
 * @param frame Pointer to the LEAP_TRACKING_EVENT containing hand data.
 */

void on_frame(LeapMotionListener* listener, const LEAP_TRACKING_EVENT* frame) {
    int handnummer = frame->nHands;            // Number of hands in the current frame
    int found_focused_hand = 0;                // Flag to track if the focused hand is found

    // Loop through all detected hands
    for (uint32_t i = 0; i < frame->nHands; ++i) {
        LEAP_HAND* hand = &frame->pHands[i];   // Pointer to the current hand data

        // Check if the current hand is the focused hand
        if (hand->id == listener->current_hand_id) {
            found_focused_hand = 1;

            // Determine if it's a left or right hand (for logging)
            const char* handType = hand->type == eLeapHandType_Left ? "Left Hand " : "Right Hand ";
            printf("%sHand ID: %d Palm Position: (%f, %f, %f)\n", handType, hand->id, hand->palm.position.x, hand->palm.position.y, hand->palm.position.z);

            // Prepare a float array with select hand data to send via UDP
            float bytes[15] = {
                (float)frame->nHands,       // Total number of hands
                hand->grab_strength,        // Grab strength of this hand
                (float)hand->id,            // Hand ID
                hand->palm.position.x,      // Palm in X direction
                hand->palm.position.y,      // Palm in Y direction
                hand->palm.position.z,      // Palm in Z direction
                (float)hand->visible_time,  // Visible time (how long the hand has been visible)
                hand->palm.normal.y,        // Palm normal's Y value
                hand->palm.velocity.x,      // Palm velocity in X direction
                hand->palm.velocity.y,      // Palm velocity in Y diection
                hand->palm.velocity.z,      // Palm velocity in Z drection
                hand->palm.orientation.x,   // Palm orientation X
                hand->palm.orientation.y,   // Palm orientation Y
                hand->palm.orientation.z,   // Palm orientation Z
                hand->palm.orientation.w    // Palm orientation W (quaternion)
            };

            // Send the data for the new focused hand
            sendto(listener->udp_socket, bytes, sizeof(bytes), 0, (struct sockaddr*)&listener->address, sizeof(listener->address));
            usleep(5000);

            break;  // Only process the focused hand
        }
    }

    // If the focused hand was not found in the frame, switch focus to another hand if available
    if (!found_focused_hand && handnummer > 0) {
        
        // Set the first hand in the list as the new focused hand
        LEAP_HAND* new_focus_hand = &frame->pHands[0];
        listener->current_hand_id = new_focus_hand->id;

        const char* handType = new_focus_hand->type == eLeapHandType_Left ? "Left Hand " : "Right Hand ";
        printf("%sHand ID: %d Palm Position: (%f, %f, %f)\n", handType, new_focus_hand->id, new_focus_hand->palm.position.x, new_focus_hand->palm.position.y, new_focus_hand->palm.position.z);

        float bytes[15] = {
            (float)frame->nHands,
            new_focus_hand->grab_strength,
            (float)new_focus_hand->id,
            new_focus_hand->palm.position.x,
            new_focus_hand->palm.position.y,
            new_focus_hand->palm.position.z,
            (float)new_focus_hand->visible_time,
            new_focus_hand->palm.normal.y,
            new_focus_hand->palm.velocity.x,
            new_focus_hand->palm.velocity.y,
            new_focus_hand->palm.velocity.z,
            new_focus_hand->palm.orientation.x,
            new_focus_hand->palm.orientation.y,
            new_focus_hand->palm.orientation.z,
            new_focus_hand->palm.orientation.w,
        };

        sendto(listener->udp_socket, bytes, sizeof(bytes), 0, (struct sockaddr*)&listener->address, sizeof(listener->address));
        usleep(5000);
    } 
    
    if (handnummer == 0) {

        float bytes[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

        sendto(listener->udp_socket, bytes, sizeof(bytes), 0, (struct sockaddr*)&listener->address, sizeof(listener->address));
        usleep(1000);
    }
}

/**
 * @brief Main event loop: polls for Leap Motion messages and handles them.
 *
 * @param listener Pointer to the LeapMotionListener struct.
 */

 void main_loop(LeapMotionListener* listener) {
    while (1) {
        LEAP_CONNECTION_MESSAGE msg;
        // Poll the Leap Motion connection. The second parameter is a timeout (ms).
        if (LeapPollConnection(*listener->connection, 10, &msg) == eLeapRS_Success) {
            switch (msg.type) {
                // If it's a tracking event, call on_frame() to process hand data
                case eLeapEventType_Tracking:
                    on_frame(listener, msg.tracking_event);
                    break;
                default:
                    // Handle any other event types here if needed
                    break;
            }
        }
    }
}

/**
 * @brief Main function. Sets up the LeapMotionListener and starts the loop.
 */
 int main() {
    LeapMotionListener listener;

    // Allocate memory for the LEAP_CONNECTION object
    listener.connection = malloc(sizeof(LEAP_CONNECTION));

    // Create and open a LeapC connection
    LeapCreateConnection(NULL, listener.connection);
    LeapOpenConnection(*listener.connection);

    // Configure the UDP address (IPv4, port, and IP)
    listener.address.sin_family = AF_INET;
    listener.address.sin_port = htons(57410);          // Port 57410
    listener.address.sin_addr.s_addr = inet_addr("127.0.0.1"); // Loopback IP address

    // Create a UDP socket
    listener.udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

    // Initialize the current hand ID to an invalid value (-1 indicates no focus yet)
    listener.current_hand_id = -1;

    // Call initialization callback
    on_init(&listener);
    
    // Call connect callback
    on_connect(&listener);

    // Enter the main polling loop
    main_loop(&listener);

    // Clean up before exiting
    onExit(&listener);

    // Close the UDP socket and destroy the Leap connection
    close(listener.udp_socket);
    LeapDestroyConnection(*listener.connection);

    // Free memory allocated for the connection
    free(listener.connection);

    return 0;
}
