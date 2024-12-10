#ifndef LEAP_MOTION_LISTENER_H
#define LEAP_MOTION_LISTENER_H

#include <LeapC.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

typedef struct {
    LEAP_CONNECTION* connection;
    LEAP_TRACKING_EVENT* tracking_event;
    struct sockaddr_in address;
    int udp_socket;
} LeapMotionListener;

void on_init(LeapMotionListener* listener);
void on_connect(LeapMotionListener* listener);
void on_disconnect(LeapMotionListener* listener);
void on_exit(LeapMotionListener* listener);
void on_frame(LeapMotionListener* listener, const LEAP_TRACKING_EVENT* frame);

#endif // LEAP_MOTION_LISTENER_H
