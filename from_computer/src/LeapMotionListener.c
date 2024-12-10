/**
*       LeapMotionListener.c
*
*       @date 14.06.2024
*       @author Joel Santos
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

// Define the LeapMotionListener struct
typedef struct {
    LEAP_CONNECTION* connection;
    struct sockaddr_in address;
    int udp_socket;
    int current_hand_id;  // Track the current hand ID
} LeapMotionListener;

// Function declarations
void on_init(LeapMotionListener* listener);
void on_connect(LeapMotionListener* listener);
void on_disconnect(LeapMotionListener* listener);
void onExit(LeapMotionListener* listener);
void on_frame(LeapMotionListener* listener, const LEAP_TRACKING_EVENT* frame);
void main_loop(LeapMotionListener* listener);

// Function definitions
void on_init(LeapMotionListener* listener) {
    printf("Initialized\n");
}

void on_connect(LeapMotionListener* listener) {
    printf("Motion Sensor Connected!\n");
}

void on_disconnect(LeapMotionListener* listener) {
    printf("Motion sensor disconnected!\n");
}

void onExit(LeapMotionListener* listener) {
    printf("Exited\n");
}

void on_frame(LeapMotionListener* listener, const LEAP_TRACKING_EVENT* frame) {
    int handnummer = frame->nHands;
    int found_focused_hand = 0;

    for (uint32_t i = 0; i < frame->nHands; ++i) {
        LEAP_HAND* hand = &frame->pHands[i];

        // Check if the current hand is the focused hand
        if (hand->id == listener->current_hand_id) {
            found_focused_hand = 1;

            const char* handType = hand->type == eLeapHandType_Left ? "Left Hand " : "Right Hand ";
            printf("%sHand ID: %d Palm Position: (%f, %f, %f)\n", handType, hand->id, hand->palm.position.x, hand->palm.position.y, hand->palm.position.z);

            float bytes[15] = {
                (float)frame->nHands,
                hand->grab_strength,
                (float)hand->id,
                hand->palm.position.x,
                hand->palm.position.y,
                hand->palm.position.z,
                (float)hand->visible_time,
                hand->palm.normal.y,
                hand->palm.velocity.x,
                hand->palm.velocity.y,
                hand->palm.velocity.z,
                hand->palm.orientation.x,
                hand->palm.orientation.y,
                hand->palm.orientation.z,
                hand->palm.orientation.w,
            };

            sendto(listener->udp_socket, bytes, sizeof(bytes), 0, (struct sockaddr*)&listener->address, sizeof(listener->address));
            usleep(5000);

            break;  // Only process the focused hand
        }
    }

    // If the focused hand was not found in the frame, switch focus to another hand if available
    if (!found_focused_hand && handnummer > 0) {
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

void main_loop(LeapMotionListener* listener) {
    while (1) {
        LEAP_CONNECTION_MESSAGE msg;
        if (LeapPollConnection(*listener->connection, 10, &msg) == eLeapRS_Success) {
            switch (msg.type) {
                case eLeapEventType_Tracking:
                    on_frame(listener, msg.tracking_event);
                    break;
                default:
                    break;
            }
        }
    }
}

int main() {
    LeapMotionListener listener;
    listener.connection = malloc(sizeof(LEAP_CONNECTION));
    LeapCreateConnection(NULL, listener.connection);
    LeapOpenConnection(*listener.connection);

    listener.address.sin_family = AF_INET;
    listener.address.sin_port = htons(57410);
    listener.address.sin_addr.s_addr = inet_addr("127.0.0.1");

    listener.udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
   
    // Initialize current hand ID
    listener.current_hand_id = -1;

    on_init(&listener);
    on_connect(&listener);

    main_loop(&listener);

    onExit(&listener);

    close(listener.udp_socket);
    LeapDestroyConnection(*listener.connection);
    free(listener.connection);

    return 0;
}

