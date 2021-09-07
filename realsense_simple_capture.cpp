#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <unistd.h>
#include <exception>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

constexpr auto STEREO_FRAME_WIDTH_PX = 848;
constexpr auto STEREO_FRAME_HEIGHT_PX = 480;
constexpr auto STEREO_FRAME_RATE_HZ = 30;

int main(){

    rs2::config cfg;
    rs2::context depth_device_context_;
    rs2::pipeline pipe_;

    cfg.enable_stream(
        RS2_STREAM_DEPTH,
        STEREO_FRAME_WIDTH_PX,
        STEREO_FRAME_HEIGHT_PX,
        RS2_FORMAT_Z16,
        STEREO_FRAME_RATE_HZ);
    cfg.enable_stream(RS2_STREAM_INFRARED);
    

    auto device = depth_device_context_.query_devices()[0];
    cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    auto depth_sensor = device.first<rs2::depth_sensor>();


    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        // depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.F);  // Disable emitter
    }
   
    if (depth_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
        depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, true); 
    }

        rs2::pipeline_profile pipeline_profile = pipe_.start(cfg);

        int counter = 0;
        double current_time_stamp = 0.0F;
        double previous_time_stamp = 0.0F; 
       while(1){
           try{
                rs2::frameset data = pipe_.wait_for_frames();
          
       
                current_time_stamp =  data.get_timestamp();
                std::cout<< " Frame received counter = "<< counter++ << "\t interval = "<< current_time_stamp - previous_time_stamp<<std::endl;
                previous_time_stamp = current_time_stamp;

                auto ir_image_left = data.get_infrared_frame(1);
                cv::Mat ir_mat_image(
                cv::Size(ir_image_left.get_width(), ir_image_left.get_height()),
                CV_8UC1,
                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast)
                (void *)ir_image_left.get_data(),
                cv::Mat::AUTO_STEP);

                cv::imshow("ir frame", ir_mat_image);
                cv::waitKey(1);
            }catch(std::exception e){
               std::cout<<"Could not retrieve frames, stereo cam may have timed out, e = "<<e.what()<< std::endl;
           }
       }
    return 0;
}