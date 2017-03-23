#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector() : image_transport(nh), gate_line_buffer(30), running(true) {
    image_sub = image_transport.subscribeCamera("stereo/right/image_raw", 1,
                &Sub8StartGateDetector::image_callback, this);
    service_2d = nh.advertiseService("/vision/start_gate/pose", &Sub8StartGateDetector::request_start_gate_position_2d, this);
    service_enable = nh.advertiseService("/vision/start_gate/enable", &Sub8StartGateDetector::request_start_gate_enable, this);
}
Sub8StartGateDetector::~Sub8StartGateDetector() {}

void Sub8StartGateDetector::image_callback(const sensor_msgs::ImageConstPtr &image_msg,
        const sensor_msgs::CameraInfoConstPtr &info_msg) {

    if (running) {

        cam_model.fromCameraInfo(info_msg);
        image_time = image_msg->header.stamp;

        if (gate_line_buffer.size() > 29) {

            for (size_t i = 0; i < gate_line_buffer.size(); i++) {
                for (size_t j = 0; j < gate_line_buffer[i].size(); j++) {
                    // std::cout << "tl " << gate_line_buffer[i][j][0] << std::endl;
                    // std::cout << "br " << gate_line_buffer[i][j][1] << std::endl;

                    cv::Point2i s = gate_line_buffer[i][j][1] - gate_line_buffer[i][j][0];
                    cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);
                    // std::cout << center << std::endl;
                    accX(center.x);
                    accY(center.y);
                    accSizeX(s.x);
                    accSizeY(s.y);
                }
            }
            // std::cout << mean(acc) << std::endl;
            // for (int i = 0; i < gate_line_buffer.size(); i++) {
            //     for (int i = 0; i < gate_line_buffer[0].size(); i++) {
            // cv::Point2i s = gate_line_buffer[0][i][1] - gate_line_buffer[0][i][0];
            // cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);
            // std::cout << center << std::endl;

            //         //Make bigger buffer. After like 30 calculate mean and average. Outlier stuff.
            //     }
            // }
            for (size_t i = 0; i < gate_line_buffer[29].size(); i++) {
                cv::Point2i s = gate_line_buffer[29][i][1] - gate_line_buffer[29][i][0];
                cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);
                // std::cout << center << std::endl;
                if (abs(center.x - mean(accX)) > sqrt(variance(accX)) ||
                        abs(center.y - mean(accY) > sqrt(variance(accY)))) {
                    gate_line_buffer[29].erase(gate_line_buffer[29].begin() + i);
                    i--;
                    std::cout << "Deleted: " << center << std::endl;
                    continue;
                }
                if (abs(s.x - mean(accSizeX)) > sqrt(variance(accSizeX)) ||
                        abs(s.y - mean(accSizeY)) > sqrt(variance(accSizeY))) {
                    gate_line_buffer[29].erase(gate_line_buffer[29].begin() + i);
                    i--;
                    std::cout << "Deleted size: " << s << std::endl;
                    continue;
                }
            }
            cv::Mat show = cv::Mat::zeros(rows, cols, CV_8UC3);

            for (size_t i = 0; i < gate_line_buffer[29].size(); i++) {
                // for (int j = 0; j < gate_line_buffer[0][i].size(); j++) {
                // cv::line(show, gate_line_buffer[0][i][j], gate_line_buffer[0][i][(j + 1) % 4], cv::Scalar(0, 0, 255), 1, 8 );
                // }
                cv::rectangle(show, gate_line_buffer[29][i][0], gate_line_buffer[29][i][1], cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            cv::imshow("ay", show);
            cv::waitKey(30);
        }
        findGate(image_msg);

    }
}


void Sub8StartGateDetector::findGate(const sensor_msgs::ImageConstPtr &image_msg) {

    cv::Mat cvImageMat = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    cv::imshow("show", cvImageMat);
    cols = cvImageMat.cols;
    rows = cvImageMat.rows;
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(cvImageMat, mean, stddev);
    // cv::imshow("view", cvImageMat);

    cv::Mat output;
    cv::inRange(cvImageMat, mean - 2 * stddev, mean + 2 * stddev, output);
    // cv::imshow("d", output);
    cv::bitwise_not(output, output);
    cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

    cv::Mat skel(output.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(output.size(), CV_8UC1);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do {
        cv::morphologyEx(output, temp, cv::MORPH_OPEN, element);
        cv::bitwise_not(temp, temp);
        cv::bitwise_and(output, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        cv::erode(output, output, element);

        double max;
        cv::minMaxLoc(output, 0, &max);
        done = (max == 0);
    } while (!done);

    cv::Mat cdst = cv::Mat::zeros(cvImageMat.size(), cvImageMat.type());
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skel, lines, 1, CV_PI / 180, 70, 50, 20 );
    for (size_t i = 0; i < lines.size(); i++ ) {
        cv::Vec4i l = lines[i];
        cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
    }
    cv::dilate(cdst, cdst, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    // cv::imshow("Skeleton", cdst);
    // cv::imshow("cd", cdst);
    // cv::Mat canny_output;
    cv::Mat canny_output = cv::Mat::zeros(cvImageMat.size(), cvImageMat.type());
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::cvtColor(cdst, canny_output, CV_BGR2GRAY );
    cv::Canny(canny_output, canny_output, 100, 100 * 2, 3 );
    // cv::imshow("canny", canny_output);

    cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // std::vector<cv::RotatedRect> minRect(contours.size());
    // std::vector<cv::RotatedRect> minRect;
    std::vector<cv::Rect> minRect;
    for (size_t i = 0; i < contours.size(); i++) {
        // cv::RotatedRect rect = cv::minAreaRect(cv::Mat(contours[i]));
        cv::Rect rect = cv::boundingRect(cv::Mat(contours[i]));
        if (rect.area() > 10000) {
            // std::cout << rect.area() << std::endl;
            minRect.push_back(rect);
        }
    }
    std::vector<std::vector<cv::Point2f>> rectanglePoints;
    for (size_t i = 0; i < minRect.size(); i++) {
        // cv::Point2f rect_points[4]; minRect[i].points(rect_points);
        // rectanglePoints.push_back(std::vector<cv::Point2f>(rect_points, rect_points + sizeof rect_points / sizeof rect_points[0]));
        rectanglePoints.push_back(std::vector<cv::Point2f>({minRect[i].tl(), minRect[i].br()}));
    }

    gate_line_buffer.push_back(rectanglePoints);
}



double Sub8StartGateDetector::distance() {
    double x = (START_GATE_WIDTH * cam_model.fx()) / mean(accSizeX);
    double y = (START_GATE_WIDTH * cam_model.fy()) / mean(accSizeY);
    return (x+y)/2;
}


bool Sub8StartGateDetector::request_start_gate_position_2d(sub8_msgs::VisionRequest2D::Request &req,
        sub8_msgs::VisionRequest2D::Response &resp) {
    if (gate_line_buffer.size() < 15) {
        resp.found = false;
        return true;
    }

    resp.pose.x = mean(accX);
    resp.pose.y = mean(accY);
    resp.pose.theta = 0;


    resp.camera_info = cam_model.cameraInfo();
    resp.found = true;
    return true;
}

bool Sub8StartGateDetector::request_start_gate_enable(std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &resp) {
    running = req.data;
    resp.success = true;
    resp.message = "Set start_gate running boolean";
    if (!req.data) { //If false we should also clear the buffer
        gate_line_buffer.clear();
    }
    return true;
}