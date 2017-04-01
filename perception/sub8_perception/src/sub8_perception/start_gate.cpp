#include <sub8_perception/start_gate.hpp>
Sub8StartGateDetector::Sub8StartGateDetector() : image_transport(nh), gate_line_buffer(30), running(true) {
    image_sub = image_transport.subscribeCamera("stereo/left/image_color", 1,
                &Sub8StartGateDetector::image_callback, this);
    service_2d = nh.advertiseService("/vision/start_gate/pose", &Sub8StartGateDetector::request_start_gate_position_2d, this);
    service_enable = nh.advertiseService("/vision/start_gate/enable", &Sub8StartGateDetector::request_start_gate_enable, this);
}
Sub8StartGateDetector::~Sub8StartGateDetector() {}

void Sub8StartGateDetector::image_callback(const sensor_msgs::ImageConstPtr &image_msg,
        const sensor_msgs::CameraInfoConstPtr &info_msg) {

    if (running) { 
        //Get some image/camera info
        cam_model.fromCameraInfo(info_msg);
        image_time = image_msg->header.stamp;

        //If we have a large enough sample of gate identifications:
        if (gate_line_buffer.size() > gate_line_buffer.capacity()-1) {

            //Loop through the buffer and save mean and deviations of both size and position
            for(auto &gate1 : gate_line_buffer) {
                for (auto &gate2 : gate1) {
                    cv::Point2i s = gate2[1] -gate2[0];
                    cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);
                    accX(center.x);
                    accY(center.y);
                    accSizeX(s.x);
                    accSizeY(s.y);
                }
            }

            //Look at one of the elements (gate_line_buffer.capacity()-1) in the circular buffer and check it's statistics and delete if necessary
            for (size_t i = 0; i < gate_line_buffer[gate_line_buffer.capacity()-1].size(); i++) {
                cv::Point2i s = gate_line_buffer[gate_line_buffer.capacity()-1][i][1] - gate_line_buffer[gate_line_buffer.capacity()-1][i][0];
                cv::Point2i center = cv::Point2i(s.x / 2 + s.x, s.y / 2 + s.y);

                //If position of the element is greater than one deviation from the mean, then delete
                if (abs(center.x - mean(accX)) > sqrt(variance(accX)) ||
                        abs(center.y - mean(accY) > sqrt(variance(accY)))) {
                    gate_line_buffer[gate_line_buffer.capacity()-1].erase(gate_line_buffer[gate_line_buffer.capacity()-1].begin() + i);
                    i--;
                    std::cout << "Deleted: " << center << std::endl;
                    continue;
                }

                //If the size of the element is greater than one deviation from the mean, then delete
                if (abs(s.x - mean(accSizeX)) > sqrt(variance(accSizeX)) ||
                        abs(s.y - mean(accSizeY)) > sqrt(variance(accSizeY))) {
                    gate_line_buffer[gate_line_buffer.capacity()-1].erase(gate_line_buffer[gate_line_buffer.capacity()-1].begin() + i);
                    i--;
                    std::cout << "Deleted size: " << s << std::endl;
                    continue;
                }
            }

            //Visualize on of the saved gates in the buffer
            
// #ifdef VISUALIZE
            cv::Mat show = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            for(auto &gate : gate_line_buffer[29]) {
                cv::rectangle(show, gate[0], gate[1], cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            cv::imshow("ay", show);
            cv::waitKey(30);
// #endif
        }
        findGate(image_msg);

    }
}


void Sub8StartGateDetector::findGate(const sensor_msgs::ImageConstPtr &image_msg) {

    cv::Mat cvImageMat = cv_bridge::toCvShare(image_msg, "bgr8")->image;

    cols = cvImageMat.cols;
    rows = cvImageMat.rows;

    //Get mean and deviation of the color in the image
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(cvImageMat, mean, stddev);

    //Threshold the image using mean and 2 standard deviations
    cv::Mat output;
    cv::inRange(cvImageMat, mean - 2 * stddev, mean + 2 * stddev, output);
    cv::bitwise_not(output, output);
    cv::dilate(output, output, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

    //Get a skeleton of things in the thresholded image
    cv::Mat skel(output.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp(output.size(), CV_8UC1);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    bool done = false;
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

    //Use HoughLine to find lines from the skeleton image
    cv::Mat cdst = cv::Mat::zeros(cvImageMat.size(), cvImageMat.type());
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(skel, lines, 1, CV_PI / 180, 70, 50, 20 );
    for (size_t i = 0; i < lines.size(); i++ ) {
        cv::Vec4i l = lines[i];
        cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);
    }

    //Dilate the image a little and find contours
    cv::dilate(cdst, cdst, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
    cv::Mat canny_output = cv::Mat::zeros(cvImageMat.size(), cvImageMat.type());
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::cvtColor(cdst, canny_output, CV_BGR2GRAY );
    cv::Canny(canny_output, canny_output, 100, 100 * 2, 3 );
    cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //Find occupying rectangles and save the ones that are within the tolerance size 
    std::vector<cv::Rect> minRect;
    for (auto &contour : contours) {
        cv::Rect rect = cv::boundingRect(cv::Mat(contour));
        if (rect.area() > START_GATE_SIZE_TOLERANCE) {
            minRect.push_back(rect);
        }
    }

    //Add the points to the buffer
    std::vector<std::vector<cv::Point2f>> rectanglePoints;
    for(auto &rect : minRect) {
        rectanglePoints.push_back(std::vector<cv::Point2f>({rect.tl(), rect.br()}));
    }

    gate_line_buffer.push_back(rectanglePoints);
}



double Sub8StartGateDetector::getGateDistance() {
    double x = (START_GATE_WIDTH * cam_model.fx()) / mean(accSizeX);
    double y = (START_GATE_HEIGHT * cam_model.fy()) / mean(accSizeY);
    return (x+y)/2;
}


bool Sub8StartGateDetector::request_start_gate_position_2d(sub8_msgs::VisionRequest2D::Request &req,
        sub8_msgs::VisionRequest2D::Response &resp) {
    //This was called too soon and doesn't have enough in the buffer
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