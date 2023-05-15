// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include <turbojpeg.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "iostream"
#include "fstream"
using namespace cv;

static bool point_cloud_color_to_depth(k4a_transformation_t transformation_handle,
    const k4a_image_t depth_image,
    const k4a_image_t color_image,
    std::string file_name)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        depth_image_width_pixels,
        depth_image_height_pixels,
        depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
        &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        depth_image_width_pixels,
        depth_image_height_pixels,
        depth_image_width_pixels * 3 * (int)sizeof(int16_t),
        &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
        depth_image,
        color_image,
        transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
        depth_image,
        K4A_CALIBRATION_TYPE_DEPTH,
        point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, transformed_color_image, file_name.c_str());

    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
    const k4a_image_t depth_image,
    const k4a_image_t color_image,
    std::string file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t),
        &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
        transformed_depth_image,
        K4A_CALIBRATION_TYPE_COLOR,
        point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());

    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}


// Timestamp in milliseconds. Defaults to 1 sec as the first couple frames don't contain color
static int playback(char* input_path, int timestamp = 20000, std::string output_filename = "output.ply")
{
    int returncode = 1;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t uncompressed_color_image = NULL;
    k4a_image_t color_image_test = NULL;

    k4a_image_t transformed_depth_image = NULL;
    int color_width_pixels = 0;
    int color_height_pixels = 0;
    uint8_t* buffer = NULL;
    uint8_t* colorBuffer = NULL;
    int rows = NULL;
    int cols = NULL;


    k4a_result_t result;
    k4a_stream_result_t stream_result;

    std::string dir = "c:\\users\\tommas\\kinect_transformations\\clinic_pain_2_depth\\\\";
    std::string ply_dir = "c:\\users\\tommas\\kinect_transformations\\ply_sad\\\\";
    std::string color_dir = "c:\\users\\tommas\\kinect_transformations\\noexp_test_color\\\\";

    std::string filename = "output2.png";
    std::string fileN = "clinic_pain_2_depth_";
    std::string color_fileN = "color_output_";
    std::string ply_fileN = "100CM_ply_output_";
    std::string fileExt = ".png";
    //std::string out_file = dir + filename;
    std::string out_file = "";
    std::string color_out_file = "";
    std::string out_file_ply = "";
    int color_width = 1920;
    int color_height = 1080;
    int depth_image_height_pixels = 576;
    int depth_image_width_pixels = 640;
    tjhandle tjhandle;



    // open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("failed to open recording %s\n", input_path);
        //goto exit;
    }
    //
    // 
    // 
    // 
    // 
    // 
    // 
    //get frame from video
    //grab first frame of video
    int time = 91600;
    result = k4a_playback_seek_timestamp(playback, time, K4A_PLAYBACK_SEEK_BEGIN);
    stream_result = k4a_playback_get_next_capture(playback, &capture);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("failed to seek timestamp %d\n", time);
        //goto exit;
    }
    /* printf("seeking to timestamp: %d/%d (ms)\n",
         time,
         (int)(k4a_playback_get_recording_length_usec(playback) / 1000));*/

    int i = 0;
    while (stream_result != K4A_STREAM_RESULT_EOF) {
        i++;
        printf("%d", i);
        out_file = dir + fileN + "" + std::to_string(i) + "" + fileExt;
        color_out_file = color_dir + color_fileN + "" + std::to_string(i) + "" + ".jpg";
        out_file_ply = ply_dir + ply_fileN + "" + std::to_string(i) + "" + ".ply";
        //result = k4a_playback_get_next_capture(playback, result);

        if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
        {
            printf("failed to fetch frame\n");
            //goto exit;
        }

        if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
        {
            printf("failed to get calibration\n");
            //goto exit;
        }

        transformation = k4a_transformation_create(&calibration);

        // fetch frame
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0)
        {
            printf("failed to get depth image from capture\n");
            //goto exit;
        }

        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0)
        {
            printf("failed to get color image from capture\n");
            //goto exit;
        }

        // convert color frame from mjpeg to bgra
        k4a_image_format_t format;
        format = k4a_image_get_format(color_image);
        if (format != K4A_IMAGE_FORMAT_COLOR_MJPG)
        {
            printf("color format not supported. please use mjpeg\n");
            //goto exit;
        }

        //int color_width, color_height;
        //color_width = k4a_image_get_width_pixels(color_image);
        //color_height = k4a_image_get_height_pixels(color_image);
        float color_stride = k4a_image_get_stride_bytes(color_image);

        /*if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            color_width,
            color_height,
            color_stride,
            &color_image_test))
        {
            printf("Failed to create transformed color image\n");
            return false;
        }*/


        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            color_width,
            color_height,
            color_width * 4 * (int)sizeof(uint8_t),
            &uncompressed_color_image))
        {
            printf("failed to create image buffer\n");
            //goto exit;
        }

        //tjhandle = tjInitDecompress();
        //if (tjDecompress2(tjhandle,
        //    k4a_image_get_buffer(color_image),
        //    static_cast<unsigned long>(k4a_image_get_size(color_image)),
        //    k4a_image_get_buffer(uncompressed_color_image),
        //    color_width,
        //    0, // pitch
        //    color_height,
        //    TJPF_BGRA,
        //    TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
        //{
        //    printf("failed to decompress color frame\n");
        //    if (tjDestroy(tjhandle))
        //    {
        //        printf("failed to destroy turbojpeg handle\n");
        //    }
        //    //goto exit;
        //}
        //if (tjDestroy(tjhandle))
        //{
        //    printf("failed to destroy turbojpeg handle\n");
        //}

        // compute color point cloud by warping depth image into color camera geometry
        //works
        /*if (point_cloud_depth_to_color(transformation, depth_image, uncompressed_color_image, out_file_ply) == false)
        {
            printf("failed to transform depth to color\n");
            goto exit;
        }
        else
        {
            printf("it worked\n");

        }*/



        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
            color_width,
            color_height,
            color_width * (int)sizeof(uint16_t),
            &transformed_depth_image))
        {
            printf("Failed to create transformed depth image\n");
            return false;
        }




        //int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
        //int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
        k4a_image_t transformed_color_image = NULL;
        if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            depth_image_width_pixels,
            depth_image_height_pixels,
            depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
            &transformed_color_image))
        {
            printf("Failed to create transformed color image\n");
            return false;
        }



        k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image);
        buffer = k4a_image_get_buffer(transformed_depth_image);
        // convert the raw buffer to cv::Mat
        rows = k4a_image_get_height_pixels(transformed_depth_image);
        cols = k4a_image_get_width_pixels(transformed_depth_image);
        //LinearStretching(rows, cols);
        cv::Mat depthMat(rows, cols, CV_16UC1, (void*)buffer, cv::Mat::AUTO_STEP);


        imwrite(out_file, depthMat);

        uint8_t* image_buffer = k4a_image_get_buffer(color_image);
        size_t image_buffer_size = k4a_image_get_size(color_image);
        String ext = ".png";
        color_out_file = "c:\\users\\tommas\\kinect_transformations\\clinic_pain_2\\\\clinic_pain_2_" + std::to_string(i) + ext;
        std::ofstream file_object(color_out_file, std::ios::out | std::ios::binary);
        file_object.write(reinterpret_cast<char*>(image_buffer), image_buffer_size);
        file_object.close();


        k4a_image_release(color_image);
        k4a_image_release(depth_image);
        k4a_image_release(transformed_color_image);
        k4a_image_release(transformed_depth_image);
        k4a_transformation_destroy(transformation);
        k4a_image_release(uncompressed_color_image);


        time += 10;
        stream_result = k4a_playback_get_next_capture(playback, &capture);

    }

    returncode = 0;

exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (uncompressed_color_image != NULL)
    {
        k4a_image_release(uncompressed_color_image);
    }
    if (capture != NULL)
    {
        k4a_capture_release(capture);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    return returncode;
}

static void print_usage()
{
    printf("Usage: transformation_example capture <output_directory> [device_id]\n");
    printf("Usage: transformation_example playback <filename.mkv> [timestamp (ms)] [output_file]\n");
}


//Driver Code
//Executes the main transofrmation functions and passes the video as an input argument
int main(int argc, char** argv)
{
    int returnCode = 0;

    if (argc < 2)
    {
        print_usage();
    }
    else
    {
        std::string mode = std::string(argv[1]);
        if (mode == "playback")
        {
            if (argc == 3)
            {
                returnCode = playback(argv[2]);
            }
            else if (argc == 4)
            {
                returnCode = playback(argv[2], atoi(argv[3]));
            }
            else if (argc == 5)
            {
                returnCode = playback(argv[2], atoi(argv[3]), argv[4]);
            }
            else
            {
                print_usage();
            }
        }
        else
        {
            print_usage();
        }
    }

    return returnCode;
}