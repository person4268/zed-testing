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
#include <tuple>
#include "utils.hpp"

extern "C" {
// time_util.cpp uses timeval which needs this include (it tries to include windows.h but i guess it doesn't include it?)
#ifdef WIN32
#include <winsock.h>
#endif
#pragma warning(push, 0)
#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag36h11.h>
#pragma warning(pop)
}

using namespace sl;

Camera zed;

bool stop;
int stopCounter = 0;

void do_cool_things_init();
void do_cool_things();
void do_cool_things_exit();

void close_camera_and_exit(int s) {
    stop = true;
    if(stopCounter == 1) {
        std::cout << "Ctrl+C hit twice, exiting slightly more forcefully" << std::endl;
        do_cool_things_exit();
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
    init_parameters.coordinate_units = UNIT::METER; // Use meter units (for depth measurements)

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

    do_cool_things_init();

    sl::Mat combined_img;

    while (!stop) {
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            do_cool_things();
            zed.retrieveImage(combined_img, sl::VIEW::SIDE_BY_SIDE);
            cv::Mat cv_mat = slMat2cvMat(combined_img);
            cv::imshow("Image", cv_mat);
            cv::waitKey(1);
        }
    }
    // Close the camera
    do_cool_things_exit();
    zed.disableRecording();
    zed.close();
    return EXIT_SUCCESS;
}

apriltag_detector_t* detector;
apriltag_family_t* tag_f;
sl::Mat image_left, depth_left, image_right;

sl::CalibrationParameters params;

void do_cool_things_init() {
    detector = apriltag_detector_create();
    tag_f = tag36h11_create();
    apriltag_detector_add_family(detector, tag_f);
    params = zed.getCameraInformation().camera_configuration.calibration_parameters;

    detector->quad_decimate = 1;
    detector->quad_sigma = 0.0f;
    fmt::println("Using {} threads for atags", get_n_threads());
    detector->nthreads = get_n_threads();
    detector->debug = 0;
    detector->refine_edges = true;
}


/* Returns center x, center y, dist_x, disy_y, dist_z */
std::vector<double> run_detections_on_mat(cv::Mat& mat) {
    image_u8_t img_left_atag = { .width = mat.cols, .height = mat.rows, .stride = mat.cols, .buf = mat.data };
    zarray_t* detections = apriltag_detector_detect(detector, &img_left_atag);

    // fmt::println("There are {} detections", zarray_size(detections));
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = (6 * 0.0254);
        info.fx = params.left_cam.fx;
        info.fy = params.left_cam.fy;
        info.cx = params.left_cam.cx;
        info.cy = params.left_cam.cy;

        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);

        double dist_x = pose.t->data[0];
        double dist_y = pose.t->data[1];
        double dist_z = pose.t->data[2];

        double cx = det->c[0];
        double cy = det->c[1];

        matd_destroy(pose.R);
        matd_destroy(pose.t);

        apriltag_detections_destroy(detections);

        return { cx, cy, dist_x, dist_y, dist_z };
    }
    return {};
}

double calc_dist(double x, double y, double z) {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
}

void do_cool_things() {
    zed.retrieveImage(image_left, sl::VIEW::LEFT_GRAY);
    zed.retrieveImage(image_right, sl::VIEW::RIGHT_GRAY);
    zed.retrieveMeasure(depth_left, sl::MEASURE::DEPTH);
    cv::Mat image_left_cv = slMat2cvMat(image_left);
    cv::Mat image_right_cv = slMat2cvMat(image_right);

    auto res_left = run_detections_on_mat(image_left_cv);
    auto res_right = run_detections_on_mat(image_right_cv);

    if(res_left.size() == 0 || res_right.size() == 0) return;

    double cam_dist_x = params.stereo_transform.tx;
    // fmt::println("stereo transform: tx={}, ty={}, tz={}", params.stereo_transform.tx, params.stereo_transform.ty, params.stereo_transform.tz);

    res_right[2] -= cam_dist_x; // align the right detection with the left?? assuming coordinate systems are what i think they are

    double left_dist = calc_dist(res_left[2], res_left[3], res_left[4]);
    double right_dist = calc_dist(res_right[2], res_right[3], res_right[4]);

    float zed_dist;
    depth_left.getValue(res_left[0], res_left[1], &zed_dist);

    if(!std::isfinite(zed_dist)) return;

    fmt::println("L-R: {}, L-Z: {} L: {}, R: {}, Z: {}", left_dist-right_dist, left_dist-zed_dist, left_dist, right_dist, zed_dist);
}

void do_cool_things_exit() {
    apriltag_detector_destroy(detector);
    tag36h11_destroy(tag_f);
}