#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <memory>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <AprilTags/apriltag.h>
#include <AprilTags/common/image_u8.h>
#include <AprilTags/tag16h5.h>
#include <AprilTags/common/zarray.h>
#include <AprilTags/common/getopt.h>

#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/crazyflie_t/webcam_pos_t.hpp>
#include <lcmtypes/crazytags/rigid_transform_t.hpp>

#include <Eigen/Dense>

#define IMAGE_U8_DEFAULT_ALIGNMENT 96

typedef Eigen::Matrix<double, 5, 1> Vector5d;

Eigen::Vector3d quat_to_rpy(const Eigen::Quaterniond &q) {
    Eigen::Vector3d rpy;
    rpy(0) = atan2(2*(q.w()*q.x() + q.y()*q.z()),
                   q.w()*q.w() + q.z()*q.z() 
                   - (q.x()*q.x() + q.y()*q.y()));
    rpy(1) = asin(2*(q.w()*q.y() - q.z()*q.x()));
    rpy(2) = atan2(2*(q.w()*q.z() + q.x()*q.y()),
                   q.w()*q.w() + q.x()*q.x()
                   - (q.y()*q.y() + q.z()*q.z()));
    return rpy;
}

int64_t timestamp_now()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

struct TagMatch {
    int id; 
    cv::Point2d p0, p1, p2, p3;
    Eigen::Matrix3d H;
};

Eigen::Isometry3d getRelativeTransform(TagMatch const& match, Eigen::Matrix3d const & camera_matrix, const Eigen::Vector4d &distortion_coefficients, double tag_size) 
{
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));


  imgPts.push_back(match.p0);
  imgPts.push_back(match.p1);
  imgPts.push_back(match.p2);
  imgPts.push_back(match.p3);

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
                           camera_matrix(0,0), 0, camera_matrix(0,2),
                           0, camera_matrix(1,1), camera_matrix(1,2),
                           0,  0,  1);

  cv::Vec4f distParam(distortion_coefficients(0), distortion_coefficients(1), distortion_coefficients(2), distortion_coefficients(3)); 
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Isometry3d T; 
  T.linear() = wRo;
  T.translation() << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  return T;
}


crazyflie_t::webcam_pos_t encodeWebcamPos(Eigen::Isometry3d const & frame) 
{
    Eigen::Vector3d t(frame.translation());
    Eigen::Quaterniond r(frame.rotation());
    Eigen::Vector3d rpy = quat_to_rpy(r);

    crazyflie_t::webcam_pos_t msg;
    msg.x = frame.translation().x();
    msg.y = frame.translation().y();
    msg.z = frame.translation().z();

    msg.roll = rpy(0);
    msg.pitch = rpy(1);
    msg.yaw = rpy(2);

    return msg;
}

crazytags::rigid_transform_t encodeLCMFrame(Eigen::Isometry3d const & frame) 
{
    Eigen::Vector3d t(frame.translation());
    Eigen::Quaterniond r(frame.rotation());

    crazytags::rigid_transform_t msg;
    msg.quat[0] = r.w();
    msg.quat[1] = r.x();
    msg.quat[2] = r.y();
    msg.quat[3] = r.z();

    msg.trans[0] = t[0];
    msg.trans[1] = t[1];
    msg.trans[2] = t[2];

    return msg;
}

class AprilTagDetector {
    public:
    AprilTagDetector(const std::unique_ptr<getopt_t, void(*)(getopt_t*)> &options){
        tf = tag16h5_create();
        tf->black_border = getopt_get_int(options.get(), "border");
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
        show_window = getopt_get_bool(options.get(), "window");

        td->quad_decimate = getopt_get_double(options.get(), "decimate");
        td->quad_sigma = getopt_get_double(options.get(), "blur");
        td->nthreads = getopt_get_int(options.get(), "threads");
        td->debug = getopt_get_bool(options.get(), "debug");
        td->refine_edges = getopt_get_bool(options.get(), "refine-edges");
        td->refine_decode = getopt_get_bool(options.get(), "refine-decode");
        td->refine_pose = getopt_get_bool(options.get(), "refine-pose");

        quiet = getopt_get_bool(options.get(), "quiet");
        tag_size = getopt_get_double(options.get(), "size");
        tag_id = getopt_get_int(options.get(), "tag_id");
    }

    ~AprilTagDetector() {

        apriltag_detector_destroy(td);
        tag16h5_destroy(tf);
    }

    std::vector<TagMatch> detectTags(image_u8_t *im) {

        const int hamm_hist_max = 10;

        int hamm_hist[hamm_hist_max];
        memset(hamm_hist, 0, sizeof(hamm_hist));
        zarray_t *detections = apriltag_detector_detect(td, im);

        std::vector<TagMatch> tag_matches;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (!quiet) {
                printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
                       i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);
                // image_u8_draw_line(im, det->p[x][0], det->p[x][1], det->p[x+1][0], det->p[x+1][1], 255, 10);
            }

            if (tag_id == -1 || det->id == tag_id) {
                TagMatch tag_match;
                tag_match.id = det->family->d*det->family->d;
                tag_match.p0 = cv::Point2d(det->p[0][0], det->p[0][1]);
                tag_match.p1 = cv::Point2d(det->p[1][0], det->p[1][1]);
                tag_match.p2 = cv::Point2d(det->p[2][0], det->p[2][1]);
                tag_match.p3 = cv::Point2d(det->p[3][0], det->p[3][1]);

                Eigen::Map<Eigen::Matrix3d> H_map(det->H->data);
                tag_match.H = H_map.transpose();
                tag_matches.push_back(tag_match);
            }

            hamm_hist[det->hamming]++;
        }

        apriltag_detections_destroy(detections);

        if (!quiet) {
            timeprofile_display(td->tp);
            printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);
            printf("Hamming histogram: ");
            for (int i = 0; i < hamm_hist_max; i++)
                printf("%5d", hamm_hist[i]);
            printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
            printf("\n");
        }
        
        return tag_matches;
    }

    std::vector<TagMatch> detectTags(const cv::Mat& image) {

        int residual = image.cols % IMAGE_U8_DEFAULT_ALIGNMENT;
        cv::Mat img_aligned;
        if (residual != 0) {
            cv::copyMakeBorder(image, img_aligned, 0, 0, (IMAGE_U8_DEFAULT_ALIGNMENT - residual) / 2, (IMAGE_U8_DEFAULT_ALIGNMENT - residual) / 2, cv::BORDER_CONSTANT, 0);
        } else {
            img_aligned = image;
        }

        cv::cvtColor(img_aligned, img_aligned, CV_RGB2GRAY);
        image_u8_t *image_u8 = fromCvMat(img_aligned);
        
        std::vector<TagMatch> tags = detectTags(image_u8);


        if (show_window) {
            cv::cvtColor(img_aligned, img_aligned, CV_GRAY2RGB);
            for (int i = 0; i < tags.size(); i++) { 

                cv::line(img_aligned, tags[i].p0, tags[i].p1, cv::Scalar(255,0,0), 2, CV_AA);
                cv::line(img_aligned, tags[i].p1, tags[i].p2, cv::Scalar(0,255,0), 2, CV_AA);
                cv::line(img_aligned, tags[i].p2, tags[i].p3, cv::Scalar(0,0,255), 2, CV_AA);
                cv::line(img_aligned, tags[i].p3, tags[i].p0, cv::Scalar(0,0,255), 2, CV_AA);

                Eigen::Vector3d x_axis(2,0,1);
                Eigen::Vector3d y_axis(0,2,1);
                Eigen::Vector3d origin(0,0,1);

                Eigen::Vector3d px = tags[i].H * x_axis;
                Eigen::Vector3d py = tags[i].H * y_axis;
                Eigen::Vector3d o  = tags[i].H * origin;

                px/= px[2];
                py/= py[2];
                o/= o[2];

                cv::line(img_aligned, cv::Point2d(o[0], o[1]), cv::Point2d(px[0], px[1]), cv::Scalar(255,0,255), 1, CV_AA);
                cv::line(img_aligned, cv::Point2d(o[0], o[1]), cv::Point2d(py[0], py[1]), cv::Scalar(255,255,0), 1, CV_AA);
            }
            cv::imshow("detections", img_aligned);
            cv::waitKey(1);
        }

        image_u8_destroy(image_u8);
        return tags;

    }
    
    image_u8_t *fromCvMat(const cv::Mat & img) { 
        image_u8_t *image_u8 = image_u8_create_alignment(img.cols, img.rows, img.step);
        // image_u8_t *image_u8 = image_u8_create(img.cols, img.rows);
        int size = img.total() * img.elemSize();
        memcpy(image_u8->buf, img.data, size * sizeof(uint8_t));
        return image_u8;
    }
    
    double getTagSize() const {
        return tag_size;
    }

    private:
    bool show_window;
    int quiet;
    int tag_id;
    double tag_size;
    apriltag_family_t *tf;
    apriltag_detector_t *td;
};

int main(int argc, char *argv[])
{
    auto options = std::unique_ptr<getopt_t, void(*)(getopt_t*)> (getopt_create(), getopt_destroy);
    // getopt_t *options = getopt_create();

    getopt_add_bool(options.get(), 'h', "help", 0, "Show this help");
    getopt_add_bool(options.get(), 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(options.get(), 'w', "window", 1, "Show the detected tags in a window");
    getopt_add_bool(options.get(), 'q', "quiet", 0, "Reduce output");
    getopt_add_int(options.get(), '\0', "border", "1", "Set tag family border size");
    getopt_add_int(options.get(), 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(options.get(), 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(options.get(), 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(options.get(), '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(options.get(), '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(options.get(), '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");
    getopt_add_double(options.get(), 's', "size", "0.05367", "Physical side-length of the tag (meters)");
    getopt_add_int(options.get(), 'c', "camera", "0", "Camera ID");
    getopt_add_int(options.get(), 'i', "tag_id", "-1", "Tag ID (-1 for all tags in family)");

    

    if (!getopt_parse(options.get(), argc, argv, 1) || getopt_get_bool(options.get(), "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(options.get());
        exit(0);
    }  
    AprilTagDetector tag_detector(options);
    auto lcm = std::make_shared<lcm::LCM>();

    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    // camera_matrix(0,0) = bot_camtrans_get_focal_length_x(mCamTransLeft);
    // camera_matrix(1,1) = bot_camtrans_get_focal_length_y(mCamTransLeft);
    // camera_matrix(0,2) = bot_camtrans_get_principal_x(mCamTransLeft);
    // camera_matrix(1,2) = bot_camtrans_get_principal_y(mCamTransLeft);
    camera_matrix(0,0) = 535.04778754;
    camera_matrix(1,1) = 533.37100256;
    camera_matrix(0,2) = 302.83654976;
    camera_matrix(1,2) = 237.69023961;

    Eigen::Vector4d distortion_coefficients(-7.74010810e-02, -1.97835565e-01, -4.47956948e-03, -5.42361499e-04);

    // camera matrix:
    // [[ 535.04778754    0.          302.83654976]
    //  [   0.          533.37100256  237.69023961]
    //  [   0.            0.            1.        ]]
    // distortion coefficients:  [ -7.74010810e-02  -1.97835565e-01  -4.47956948e-03  -5.42361499e-04
    //    9.30985112e-01]


    cv::VideoCapture capture(getopt_get_int(options.get(), "camera"));
    if (!capture.isOpened()) {
        std::cout << "Cannot open the video cam" << std::endl;
        return -1;
    }

    cv::Mat frame;
    Eigen::Isometry3d tag_to_camera = Eigen::Isometry3d::Identity();
    crazyflie_t::webcam_pos_t tag_to_camera_msg;
    while (capture.read(frame)) {
        std::vector<TagMatch> tags = tag_detector.detectTags(frame);
        if (tags.size() > 0) {
            tag_to_camera = getRelativeTransform(tags[0], camera_matrix, distortion_coefficients, tag_detector.getTagSize());
            tag_to_camera_msg = encodeWebcamPos(tag_to_camera);
            tag_to_camera_msg.frame_id = 1;
        } else {
            tag_to_camera_msg = encodeWebcamPos(tag_to_camera);
            tag_to_camera_msg.frame_id = -1;
        }
        tag_to_camera_msg.timestamp = timestamp_now();
        lcm->publish("WEBCAM_POS", &tag_to_camera_msg);
    }

    return 0;
}
