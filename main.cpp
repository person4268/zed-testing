///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2024, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

#include "slCamera.hpp"
#include <fmt/format.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "utils.hpp"

using namespace sl;

Camera zed;

bool stop;
int stopCounter = 0;


void close_camera_and_exit(int s) {
    stop = true;
    if(stopCounter == 1) {
        std::cout << "Ctrl+C hit twice, exiting slightly more forcefully" << std::endl;
        zed.disableRecording();
        zed.close();
        exit(1);
    } else if (stopCounter > 1) { // honestly not sure if this can ever trigger
        std::cout << "Ctrl+C hit thrice, stopping extremely forcefully" << std::endl;
        exit(1);
    }
    stopCounter++;
}

int main(int argc, char **argv) {
    signal(SIGINT, close_camera_and_exit);

    // Set configuration parameters
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD2K;
    init_parameters.depth_mode = DEPTH_MODE::ULTRA; // Use ULTRA depth mode
    init_parameters.coordinate_units = UNIT::MILLIMETER; // Use millimeter units (for depth measurements)

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        std::cout << "Error " << returned_state << ", exit program." << std::endl;
        return EXIT_FAILURE;
    }

    String outputPath = "C:\\Users\\michael\\Documents\\ZED\\output.svo";
    RecordingParameters rec;
    rec.compression_mode = SVO_COMPRESSION_MODE::H264;
    rec.video_filename = outputPath;
    returned_state = zed.enableRecording(rec);
    if (returned_state != ERROR_CODE::SUCCESS) {
        fmt::println("Error {} while recording", (int)returned_state);
        return EXIT_FAILURE;
    }

    sl::Mat image, depth, point_cloud;

    while (!stop) {
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            // Retrieve left image
            zed.retrieveImage(image, VIEW::LEFT);
            // Retrieve depth map. Depth is aligned on the left image
            zed.retrieveMeasure(depth, MEASURE::DEPTH);
            // Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

            // Get and print distance value in mm at the center of the image
            // We measure the distance camera - object using Euclidean distance
            int x = image.getWidth() / 2;
            int y = image.getHeight() / 2;
            sl::float4 point_cloud_value;
            point_cloud.getValue(x, y, &point_cloud_value);

            cv::Mat cv_mat = slMat2cvMat(image);
            cv::circle(cv_mat, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), 1);

            if (std::isfinite(point_cloud_value.z)) {
                float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                cv::putText(cv_mat, fmt::format("{:.2f}cm", distance / 10), cv::Point(x+10, y+10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
                // std::cout <<"Distance to Camera at {" << x << ";" << y << "}: "<< distance << "mm" << std::endl;
                // fmt::print("Distance to Camera at {{{};{}}}: {}mm\n", x, y, distance);
            } else {
                // std:: cout << "The Distance can not be computed at {" << x << ";" << y << "}" << std::endl;
                fmt::print("The Distance can not be computed at {{{};{}}}\n", x, y);

            }

            cv::imshow("Image", cv_mat);
            cv::waitKey(1);
        }
    }
    // Close the camera
    zed.disableRecording();
    zed.close();
    return EXIT_SUCCESS;
}