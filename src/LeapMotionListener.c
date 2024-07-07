#include <LeapC.h> //
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

    if (handnummer < 1) {
        //float bytes[21] = {0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
        //sendto(listener->udp_socket, bytes, sizeof(bytes), 0, (struct sockaddr*)&listener->address, sizeof(listener->address));
        //usleep(5000);
    } else if(handnummer == 1){
        for (uint32_t i = 0; i < frame->nHands; ++i) {
            LEAP_HAND* hand = &frame->pHands[i];

            const char* handType = hand->type == eLeapHandType_Left ? "Left Hand " : "Right Hand ";
            printf("%sHand ID: %d Palm Position: (%f, %f, %f)\n", handType, hand->id, hand->palm.position.x, hand->palm.position.y, hand->palm.position.z);

            float bytes[21] = {
                (float)frame->nHands,
                hand->grab_strength,
                (float)hand->id,
                hand->palm.position.x,
                hand->palm.position.y,
                hand->palm.position.z,
                hand->visible_time,
                hand->palm.normal.y,
                hand->palm.velocity.x,
                hand->palm.velocity.y,
                hand->palm.velocity.z,
                //hand->palm.basis.x_basis.x,
                //hand->palm.basis.x_basis.y,
                //hand->palm.basis.x_basis.z,
                //hand->palm.basis.y_basis.x,
                //hand->palm.basis.y_basis.y,
                //hand->palm.basis.y_basis.z,
                //hand->palm.basis.z_basis.x,
                //hand->palm.basis.z_basis.y,
                //hand->palm.basis.z_basis.z,
            };

            sendto(listener->udp_socket, bytes, sizeof(bytes), 0, (struct sockaddr*)&listener->address, sizeof(listener->address));
            usleep(5000);
        }
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

    on_init(&listener);
    on_connect(&listener);

    main_loop(&listener);

    onExit(&listener);

    close(listener.udp_socket);
    LeapDestroyConnection(*listener.connection);
    free(listener.connection);

    return 0;
}
