// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp> // Include RealSense Cross Platform API
// v1 doesn't have an advanced mode. 
//#include <librealsense/rs_advanced_mode.hpp>
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <signal.h>
#include <iomanip>
#include <stdio.h>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>


//------------------- TCP Server Code -------------------
//-------------------------------------------------------

typedef void * (*THREADFUNCPTR)(void *);

class Server {

    public:
        Server(int port);
        void * listener_thread();
        void init_listener_thread();
        void update_buffer(const unsigned char * data, int offset, unsigned long numbytes);

    private:
        int init_sock, conn_sock;
        char * send_buffer;
        int buffer_size = 1024;
        char receive_buffer[1024];
        struct sockaddr_in serv_addr;
        struct sockaddr_storage serv_storage;
        socklen_t addr_size;
        pthread_mutex_t buffer_access_mutex;
        pthread_t listener_thread_id;
        unsigned long frame_size;
};

Server::Server(int port) {
    init_sock = socket(PF_INET, SOCK_STREAM, 0);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons (port);
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
    bind(init_sock, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    send_buffer = new char[buffer_size];
}

void Server::init_listener_thread() {
    pthread_create(&listener_thread_id, NULL, (THREADFUNCPTR) &Server::listener_thread, this);
    pthread_mutex_init(&buffer_access_mutex, NULL);
}

void * Server::listener_thread() {
    while(true) {
        if (listen(init_sock, 5) == 0)
            printf ("Listening...\n");
        else
            printf ("Error.\n");

        // Creates new socket for incoming connection
        addr_size = sizeof(serv_storage);
        conn_sock = accept (init_sock, (struct sockaddr *) &serv_storage, &addr_size);
        printf ("Connected to client.\n");

        while(true) {

            // Parse ping from client
            memset(receive_buffer, 0, sizeof(receive_buffer));
            int resp_msg_size = recv(conn_sock, receive_buffer, 64, 0);
            if (resp_msg_size <= 0) break;

            // Send buffer data
            pthread_mutex_lock(&buffer_access_mutex);
            int msg_size = send(conn_sock, send_buffer, buffer_size, MSG_MORE);
            if (msg_size == 0 ) printf("Warning: No data was sent to client.\n");
            int tmp = errno;
            if (msg_size < 0) printf ("Errno %d\n", tmp);
            pthread_mutex_unlock(&buffer_access_mutex);
        }
    }
}

void Server::update_buffer(const unsigned char * data, int offset, unsigned long numbytes) {
    pthread_mutex_lock(&buffer_access_mutex);

    // Update buffer size
    unsigned long new_buffer_size = numbytes + offset;
    if (new_buffer_size > buffer_size) {
        delete [] send_buffer;
        buffer_size = new_buffer_size;
        send_buffer = new char[buffer_size];
    }

    // Copy data
    memcpy(send_buffer + offset, data, numbytes);
    pthread_mutex_unlock(&buffer_access_mutex);
}

//-------------------------------------------------------
//-------------------------------------------------------

// Configure all streams to run at 640x480 resolution at 30 frames per second
const int stream_width = 640;
const int stream_height = 480;
const int stream_fps = 30;
const int depth_disparity_shift = 50;

// Capture color and depth video streams, render them to the screen, send them through TCP
int main(int argc, char * argv[]) try {

    Server realsense_server(50000);
    realsense_server.init_listener_thread();

    // Check if RealSense device is connected
    rs::context ctx;
    if (ctx.get_device_count() == 0) {
      std::cerr << "No device connected, please connect a RealSense device" << std::endl;
      return EXIT_FAILURE;
    }

    // Configure streams
    rs::device &dev = *ctx.get_device(0);
    dev.enable_stream(rs::stream::depth, stream_width, stream_height, rs::format::z16, stream_fps);
    dev.enable_stream(rs::stream::color, stream_width, stream_height, rs::format::rgb8, stream_fps);

    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image;

    // Declare depth colorizer for pretty visualization of depth data
    // Note(brycew): used for visualization, useful but not in v1 of lib.
    // rs::colorizer color_map;

    // Start streaming
    dev.start();

    // Print active device information
    std::cout << "Device information: " << std::endl;
    for (int i = 0; i < static_cast<int>(RS_CAMERA_INFO_COUNT); i++) {
      rs_camera_info info_type = static_cast<rs_camera_info>(i);
      std::cout << "  " << std::left << std::setw(20) << info_type << " : ";
      if (dev.supports((rs::camera_info)info_type))
          std::cout << dev.get_info((rs::camera_info)info_type) << std::endl;
      else
          std::cout << "N/A" << std::endl;
    }
    
    // Looks like v1 doesn't have this advanced mode. Ignoring for now.
    // // Create advanced mode abstraction for RS400 device and set disparity shift
    // rs400::advanced_mode advanced(dev);
    // STDepthTableControl depth_table_control;
    // if (advanced.is_enabled()) {
    //     depth_table_control = advanced.get_depth_table(); // Initialize depth table control group
    //     depth_table_control.depthUnits = 100; // Interpret RMS error at 100 micrometers
    //     depth_table_control.disparityShift = depth_disparity_shift; // Set disparity shift
    //     advanced.set_depth_table(depth_table_control);
    // }

    // Get active device sensors
    //std::vector<rs::sensor> sensors = dev.query_sensors();
    //rs::sensor depth_sensor = sensors[0];
    //rs::sensor color_sensor = sensors[1];

    // Enable auto white balancing for color sensor
    rs_option wb_option_type = static_cast<rs_option>(11);
    if (dev.supports_option((rs::option)wb_option_type))
        dev.set_option((rs::option)wb_option_type, 1);

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (int i = 0; i < 30; ++i) dev.wait_for_frames();

    // Print camera intrinsics of color sensor
    //rs::video_stream_profile color_stream_profile = active_pipe_profile.get_stream(rs_stream::RS_STREAM_COLOR).as<rs::video_stream_profile>();
    rs_intrinsics color_intrinsics = dev.get_stream_intrinsics(rs::stream::color);
    float color_intrinsics_arr[9] = {color_intrinsics.fx, 0.0f, color_intrinsics.ppx,
                                     0.0f, color_intrinsics.fy, color_intrinsics.ppy,
                                     0.0f, 0.0f, 1.0f};

    // Get depth scale for converting depth pixel values into distances in meters
    float depth_scale = dev.get_depth_scale(); //depth_sensor.as<rs::depth_sensor>().get_depth_scale();

    // Create alignment object (for aligning depth frame to color frame)
    //rs::align align(rs_stream::RS_STREAM_COLOR);

    if (argc == 2 && std::string(argv[1]) == "window")
    {
        window app(640, 480, "RealSense Stream");
        while(app) // Application still alive?
        {
            // Wait for next set of frames from the camera
            if(dev.is_streaming())
                dev.wait_for_frames();
    
            //rs::frame color = data.get_color_frame(); 
            // rs2::depth_frame raw_depth = data.get_depth_frame();       
    
            auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
            auto aligned_depth = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_color));
    
            // Find and colorize the depth data
            //rs::frame depth_colorized = color_map(aligned_depth);  
    
            // Since we initialize with z16 format type, there should be 16 bits per frame.
            int depth_size = dev.get_stream_width(rs::stream::depth_aligned_to_color)*dev.get_stream_height(rs::stream::depth_aligned_to_color)* 16; // dev.get_stream_bbp();
            realsense_server.update_buffer((unsigned char*)aligned_depth, 10*4, depth_size);
    
            // Since we initialize with rgb8, there should be 8 bits per frame.
            int color_size = dev.get_stream_width(rs::stream::color)*dev.get_stream_height(rs::stream::color)* 8; //dev.get_bytes_per_pixel();
            realsense_server.update_buffer((unsigned char*)points, 10*4 + depth_size, color_size);
    
            // Send camera intrinsics and depth scale
            realsense_server.update_buffer((unsigned char*)color_intrinsics_arr, 0, 9*4);
            realsense_server.update_buffer((unsigned char*)&depth_scale, 9*4, 4);
    
            const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
            // Render depth on to the first half of the screen and color on to the second
            //depth_image.render(depth_colorized, { 0, 0, app->width() / 2, app->height() });
            color_image.render(points, depth_intrin, { app.width() / 2, 0, app.width() / 2, app.height() });
        }
        return EXIT_SUCCESS;
    }

    // Otherwise, ignore this.
    while(true) // Run until ctrl-c 
    {
        // Wait for next set of frames from the camera
        if(dev.is_streaming())
            dev.wait_for_frames();

        //rs::frame color = data.get_color_frame(); 
        // rs2::depth_frame raw_depth = data.get_depth_frame();       

        auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
        auto aligned_depth = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth_aligned_to_color));

        // Find and colorize the depth data
        //rs::frame depth_colorized = color_map(aligned_depth);  

        // Since we initialize with z16 format type, there should be 16 bits per frame.
        int depth_size = dev.get_stream_width(rs::stream::depth_aligned_to_color)*dev.get_stream_height(rs::stream::depth_aligned_to_color)* 16; // dev.get_stream_bbp();
        realsense_server.update_buffer((unsigned char*)aligned_depth, 10*4, depth_size);

        // Since we initialize with rgb8, there should be 8 bits per frame.
        int color_size = dev.get_stream_width(rs::stream::color)*dev.get_stream_height(rs::stream::color)* 8; //dev.get_bytes_per_pixel();
        realsense_server.update_buffer((unsigned char*)points, 10*4 + depth_size, color_size);

        // Send camera intrinsics and depth scale
        realsense_server.update_buffer((unsigned char*)color_intrinsics_arr, 0, 9*4);
        realsense_server.update_buffer((unsigned char*)&depth_scale, 9*4, 4);
    }

    return EXIT_SUCCESS;
}
catch (const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



