#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <X11/Xlib.h>
#include <X11/extensions/XInput2.h>
#include <iostream>
#include <map>
#include <cmath>

std::map<int, std::pair<float, float>> touches; // 用于跟踪触摸点

int main(int argc, char **argv) {

    ros::init(argc, argv, "zoom_factor_publisher");
    ros::NodeHandle nh;
    ros::Publisher zoom_pub = nh.advertise<std_msgs::Float64>("zoom_factor", 10);
        Display *display = XOpenDisplay(NULL);
    if (!display) {
        std::cerr << "Cannot open display\n";
        return -1;
    }


    int opcode, event, error;
    if (!XQueryExtension(display, "XInputExtension", &opcode, &event, &error)) {
        std::cerr << "X Input extension not available.\n";
        return -1;
    }


    unsigned char mask[(XI_LASTEVENT + 7)/8] = { 0 };
    XISetMask(mask, XI_TouchBegin);
    XISetMask(mask, XI_TouchUpdate);
    XISetMask(mask, XI_TouchEnd);


    XIEventMask evmask;
    evmask.deviceid = XIAllDevices;
    evmask.mask_len = sizeof(mask);
    evmask.mask = mask;
    XISelectEvents(display, DefaultRootWindow(display), &evmask, 1);
        float initialDistance = 0.0;
    bool calculatingZoom = false;

    while (ros::ok()) {
        XEvent event;
        XGenericEventCookie *cookie = &event.xcookie;

        XNextEvent(display, &event);

        if (cookie->type == GenericEvent && cookie->extension == opcode && XGetEventData(display, cookie)) {
            switch (cookie->evtype) {
                case XI_TouchBegin: {
                    XIDeviceEvent* ev = (XIDeviceEvent*)cookie->data;
                    touches[ev->detail] = std::make_pair(ev->event_x, ev->event_y);

                    if (touches.size() == 2 && !calculatingZoom) {
                        auto it = touches.begin();
                        float x1 = it->second.first;
                        float y1 = it->second.second;
                        ++it;
                        float x2 = it->second.first;
                        float y2 = it->second.second;

                        initialDistance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
                        calculatingZoom = true;
                    }
                    break;
                }
                case XI_TouchUpdate: {
                    if (touches.size() == 2 && calculatingZoom) {
                        auto it = touches.begin();
                        float x1 = it->second.first;
                        float y1 = it->second.second;
                        ++it;
                        float x2 = it->second.first;
                        float y2 = it->second.second;

                        float currentDistance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
                        float scaleFactor = currentDistance / initialDistance;

                        std_msgs::Float64 zoom_msg;
                        zoom_msg.data = scaleFactor;
                        zoom_pub.publish(zoom_msg);
                        std::cout << "zoom_msg.data" << zoom_msg.data << std::endl;
                    }
                    break;
                }
                case XI_TouchEnd: {
                    XIDeviceEvent* ev = (XIDeviceEvent*)cookie->data;
                    touches.erase(ev->detail);
                    if (touches.size() < 2) {
                        calculatingZoom = false;
                    }
                    break;
                }
            }
            XFreeEventData(display, cookie);
        }

        ros::spinOnce();
    }

    XCloseDisplay(display);
    return 0;
}