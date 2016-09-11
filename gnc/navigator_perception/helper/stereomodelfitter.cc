#include <navigator_vision_lib/stereomodelfitter.h>

StereoModelFitter::StereoModelFitter(PerceptionModel model, image_transport::Publisher debug_publisher):
    model(model),
    debug_publisher(debug_publisher)
{

//    ros::NodeHandle nh;
//    image_transport::ImageTransport it(nh);
//    pub = it.advertise("scan_the_code/debug1", 1);

}


using namespace std;
using namespace cv;

void StereoModelFitter::denoise_images(Mat& l_diffused, Mat& r_diffused,
                                       int diffusion_time, Mat current_image_left,
                                       Mat current_image_right)
{
    Mat diffusion_size_left, diffusion_size_right;
    resize(current_image_left, diffusion_size_left, Size(0, 0), image_proc_scale, image_proc_scale);
    resize(current_image_right, diffusion_size_right, Size(0, 0), image_proc_scale, image_proc_scale);
    cvtColor(diffusion_size_left, diffusion_size_left, CV_BGR2GRAY);
    cvtColor(diffusion_size_right, diffusion_size_right, CV_BGR2GRAY);
    boost::thread diffusion_L(anisotropic_diffusion, boost::cref(diffusion_size_left),
                              boost::ref(l_diffused), diffusion_time);
    boost::thread diffusion_R(anisotropic_diffusion, boost::cref(diffusion_size_right),
                              boost::ref(r_diffused), diffusion_time);
    diffusion_L.join();
    diffusion_R.join();
}

void StereoModelFitter::extract_features(vector<Point> & features,
                                         Mat& image,
                                         int max_corners,
                                         int block_size,
                                         double quality_level,
                                         double min_distance)
{
  goodFeaturesToTrack(image, features, max_corners, quality_level, min_distance, Mat(), block_size, true, .001);
}

void StereoModelFitter::get_corresponding_pairs(
        vector<Point> features_l,
        vector<Point> features_r,
        vector<Point>& features_l_out,
        vector<Point>& features_r_out,
        int picture_width)
{
    double curr_min_dist, xdiff, ydiff, dist;
    int curr_min_dist_idx;
    // MAYBE FIX THIS
    int y_diff_thresh = picture_width * 0.02;
    int x_diff_thresh = picture_width * 0.15;
    //cout << "y_diff_thresh: " << y_diff_thresh << endl;
    for (size_t i = 0; i < features_l.size(); i++)
    {
        curr_min_dist_idx = -1;
        curr_min_dist = 1E6;
        //cout << "\x1b[31m" << i << " \x1b[0mCurrent pt: "  << features_l[i] << endl;
        for(size_t j = 0; j < features_r.size(); j++)
            {
                //cout << "\t\x1b[31m" << j << " \x1b[0mCandidate pt: "  << features_r[j] << endl;
                ydiff = features_l[i].y - features_r[j].y;
                xdiff = features_l[i].x - features_r[j].x;
                //cout << "\t   ydiff: " << ydiff << endl;
                if(abs(ydiff) > y_diff_thresh) continue;
                if(abs(xdiff) > x_diff_thresh) continue;
                if(xdiff == 0 && ydiff == 0) continue;

                dist = sqrt(xdiff * xdiff + ydiff * ydiff);
                //cout << "\t   dist: " << dist << endl;
                if(dist < curr_min_dist)
                    {
                        curr_min_dist = dist;
                        curr_min_dist_idx = j;
                    }
            }
        if(curr_min_dist_idx != -1){
          features_l_out.push_back(features_l[i]);
          features_r_out.push_back(features_r[curr_min_dist_idx]);
        }
        //cout << "Match: " << curr_min_dist_idx << endl;
    }

}


void StereoModelFitter::calculate_3D_reconstruction(vector<Eigen::Vector3d>& feature_pts_3d,
        vector<Point> features_l,
        vector<Point> features_r)
{

    Eigen::Vector3d pt_3D;
    double reset_scaling = 1 / image_proc_scale;
    for (size_t i = 0; i < features_l.size(); i++)
        {
            Point2d pt_L = features_l[ i ];
            Point2d pt_R = features_r[ i ];

            // Undo the effects of working with coordinates from scaled images
            pt_L = pt_L * reset_scaling;
            pt_R = pt_R * reset_scaling;

            // Print points in image coordinates
            //cout << "L: " << pt_L << "R: " << pt_R << endl;

            Matx31d pt_L_hom(pt_L.x, pt_L.y, 1);
            Matx31d pt_R_hom(pt_R.x, pt_R.y, 1);
            Mat X_hom = nav::triangulate_Linear_LS(Mat(*left_cam_mat),
                                                   Mat(*right_cam_mat),
                                                   Mat(pt_L_hom), Mat(pt_R_hom));
            X_hom = X_hom / X_hom.at<double>(3, 0);
            pt_3D <<
                  X_hom.at<double>(0, 0), X_hom.at<double>(1, 0), X_hom.at<double>(2, 0);
            // cout << "[ " << pt_3D(0) << ", "  << pt_3D(1) << ", " << pt_3D(2) << "]" << endl;
            feature_pts_3d.push_back(pt_3D);
        }
    //cout << "num 3D features: " << feature_pts_3d.size() << endl;

}

bool StereoModelFitter::check_for_model(vector<Eigen::Vector3d>  feature_pts_3d, vector<Eigen::Vector3d>& correct_model)
{
    vector<int> debug_vec;
//    string bin;
//    cin >> bin;
//    if(bin != "done"){
//      debug_vec  = split(bin);
//    }

    decision_tree(feature_pts_3d, 0, model.min_points, debug_vec, false);
    model.get_model(correct_model, *current_image_left, *left_cam_mat);
    model.clear();
    return true;
}

void StereoModelFitter::decision_tree(vector<Eigen::Vector3d>  feature_pts_3d, int curr, int remaining, vector<int> debug_points, bool debug)
{
    int total = feature_pts_3d.size();

    // If the current model has the right amount of points or we have no more combinations left to check (the second is condition is unecessary, but for safety)
    if(!model.complete() || remaining > 0){
        for(int i = curr + 1; i <= total - remaining; ++i){
            Eigen::Vector3d point = feature_pts_3d[i];
            bool val = false;
            int a = model.current_points.size();
            if(((std::find(debug_points.begin(), debug_points.end(), i) != debug_points.end()) && a == 0) || debug){
              debug = true;
              val = model.check_point(point, *current_image_left, *left_cam_mat, true);
            }else{
              val = model.check_point(point, *current_image_left, *left_cam_mat, false);
              debug = false;
            }

            // If that point was correct, keep checking the rest of the points
            if(val)
                {
                    decision_tree(feature_pts_3d, i, remaining - 1, debug_points, debug);
                }
            // Remove that point from the model so that other points can be checked
            model.remove_point(point);
        }
    }
}

bool StereoModelFitter::determine_model_position(vector<Eigen::Vector3d>& model_position,
                                                 int max_corners,
                                                 int block_size,
                                                 double min_distance,
                                                 double image_proc_scale,
                                                 int diffusion_time,
                                                 Mat current_image_left,
                                                 Mat current_image_right,
                                                 Matx34d left_cam_mat,
                                                 Matx34d right_cam_mat)
{
    this->image_proc_scale = image_proc_scale;
    this->left_cam_mat = &left_cam_mat;
    this->right_cam_mat = &right_cam_mat;
    this->current_image_left = &current_image_left;
    this->current_image_right = &current_image_right;

    Mat l_diffused, r_diffused;
    denoise_images(l_diffused, r_diffused, diffusion_time, current_image_left, current_image_right);
    vector< Point > features_l, features_r;
    Mat l_diffused_draw = l_diffused.clone();
    Mat r_diffused_draw = r_diffused.clone();
    double quality_level = 0.05;
    extract_features(features_r, r_diffused_draw, max_corners, block_size,
                     quality_level, min_distance);
    extract_features(features_l, l_diffused_draw, max_corners, block_size,
                     quality_level, min_distance);

    vector<Point> points_l, points_r;
    get_corresponding_pairs(features_l, features_r, points_l, points_r, l_diffused.rows);

    vector<Eigen::Vector3d> feature_pts_3d;
    calculate_3D_reconstruction(feature_pts_3d, points_l, points_r);

    vector<Eigen::Vector3d> correct_model;

    visualize_points(feature_pts_3d, current_image_left);
    check_for_model(feature_pts_3d, correct_model);
    model_position = correct_model;
    return true;
}

void StereoModelFitter::visualize_points(
        vector<Eigen::Vector3d>  feature_pts_3d,
        Mat& img_left)
{
    cv::Mat current_image_left = img_left.clone();
    for(size_t i = 0; i < feature_pts_3d.size(); i++)
    {
        Eigen::Vector3d pt = feature_pts_3d[i];
        Matx41d position_hom(pt(0), pt(1), pt(2), 1);
        Matx31d pt_L_2d_hom = * left_cam_mat * position_hom;
        Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
        Scalar color(255, 0, 255);
        stringstream label;
        label << i;
        circle(current_image_left, L_center2d, 5, color, -1);
        putText(current_image_left, label.str(), L_center2d, FONT_HERSHEY_SIMPLEX,
                0.0015 * current_image_left.rows, Scalar(0, 0, 0), 2);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image_left).toImageMsg();
    debug_publisher.publish(msg);
    ros::spinOnce();
}


std::vector<int> split(string str){
      std::vector<int> vect;
      std::stringstream ss(str);
      int i;
      while (ss >> i)
      {
          vect.push_back(i);
          if (ss.peek() == ',')
              ss.ignore();
      }
      return vect;
}


void anisotropic_diffusion(const Mat &src, Mat &dest, int t_max)
{

    Mat x = src;
    Mat x0;
    x.convertTo(x0, CV_32FC1);

    double t=0;
    double lambda=0.25; // Defined in equation (7)
    double K=10,K2=(1/K/K); // defined after equation(13) in text

    Mat    dI00 = Mat::zeros(x0.size(),CV_32F);

    Mat x1, xc;

    while (t < t_max)
        {

            Mat D; // defined just before equation (5) in text
            Mat gradxX,gradyX; // Image Gradient t time
            Sobel(x0,gradxX,CV_32F,1,0,3);
            Sobel(x0,gradyX,CV_32F,0,1,3);
            D = Mat::zeros(x0.size(),CV_32F);

            for (int i=0; i<x0.rows; i++)
                for (int j = 0; j < x0.cols; j++)
                    {
                        float gx = gradxX.at<float>(i, j), gy = gradyX.at<float>(i,j);
                        float d;
                        if (i==0 || i== x0.rows-1 || j==0 || j==x0.cols-1) // conduction coefficient set to
                            d=1;                                           // 1 p633 after equation 13
                        else
                            d =1.0/(1+(gx*gx+0*gy*gy)*K2); // expression of g(gradient(I))
                        //d =-exp(-(gx*gx+gy*gy)*K2); // expression of g(gradient(I))
                        D.at<float>(i, j) = d;
                    }

            x1 = Mat::zeros(x0.size(),CV_32F);
            double maxD=0,intxx=0;
            {
                int i=0;
                float *u1 = (float*)x1.ptr(i);
                u1++;
                for (int j = 1; j < x0.cols-1; j++,u1++)
                    {
                        // Value of I at (i+1,j),(i,j+1)...(i,j)
                        float ip10=x0.at<float>(i+1, j),i0p1=x0.at<float>(i, j+1);
                        float i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);

                        // Value of D at at (i+1,j),(i,j+1)...(i,j)
                        float cp10=D.at<float>(i+1, j),c0p1=D.at<float>(i, j+1);
                        float c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);

                        // Equation (7) p632
                        double xx=(cp10+c00)*(ip10-i00) + (c0p1+c00)*(i0p1-i00) + (c0m1+c00)*(i0m1-i00);
                        dI00.at<float>(i, j) = xx;
                        if (maxD<fabs(xx))
                            maxD=fabs(xx);
                        intxx+=fabs(xx);
                        // equation (9)
                    }
            }

            for (int i = 1; i < x0.rows-1; i++)
                {

                    float *u1 = (float*)x1.ptr(i);
                    int j=0;
                    if (j==0)
                        {
                            // Value of I at (i+1,j),(i,j+1)...(i,j)
                            float ip10=x0.at<float>(i+1, j),i0p1=x0.at<float>(i, j+1);
                            float im10=x0.at<float>(i-1, j),i00=x0.at<float>(i, j);
                            // Value of D at at (i+1,j),(i,j+1)...(i,j)
                            float cp10=D.at<float>(i+1, j),c0p1=D.at<float>(i, j+1);
                            float cm10=D.at<float>(i-1, j),c00=D.at<float>(i, j);
                            // Equation (7) p632
                            double xx=(cp10+c00)*(ip10-i00) + (c0p1+c00)*(i0p1-i00) + (cm10+c00)*(im10-i00);
                            dI00.at<float>(i, j) = xx;
                            if (maxD<fabs(xx))
                                maxD=fabs(xx);
                            intxx+=fabs(xx);
                            // equation (9)
                        }

                    u1++;
                    j++;
                    for (int j = 1; j < x0.cols-1; j++,u1++)
                        {
                            // Value of I at (i+1,j),(i,j+1)...(i,j)
                            float ip10=x0.at<float>(i+1, j),i0p1=x0.at<float>(i, j+1);
                            float im10=x0.at<float>(i-1, j),i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);
                            // Value of D at at (i+1,j),(i,j+1)...(i,j)
                            float cp10=D.at<float>(i+1, j),c0p1=D.at<float>(i, j+1);
                            float cm10=D.at<float>(i-1, j),c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);
                            // Equation (7) p632
                            double xx=(cp10+c00)*(ip10-i00) + (c0p1+c00)*(i0p1-i00) + (cm10+c00)*(im10-i00)+ (c0m1+c00)*(i0m1-i00);
                            dI00.at<float>(i, j) = xx;
                            if (maxD<fabs(xx))
                                maxD=fabs(xx);
                            intxx+=fabs(xx);
                            // equation (9)
                        }

                    j++;
                    if (j==x0.cols-1)
                        {
                            // Value of I at (i+1,j),(i,j+1)...(i,j)
                            float ip10=x0.at<float>(i+1, j);
                            float im10=x0.at<float>(i-1, j),i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);

                            // Value of D at at (i+1,j),(i,j+1)...(i,j)
                            float cp10=D.at<float>(i+1, j);
                            float cm10=D.at<float>(i-1, j),c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);

                            // Equation (7) p632
                            double xx=(cp10+c00)*(ip10-i00)  + (cm10+c00)*(im10-i00)+ (c0m1+c00)*(i0m1-i00);
                            dI00.at<float>(i, j) = xx;
                            if (maxD<fabs(xx))
                                maxD=fabs(xx);
                            intxx+=fabs(xx);
                            // equation (9)
                        }
                }
            {
                int i=x0.rows-1;
                float *u1 = (float*)x1.ptr(i);
                u1++;
                for (int j = 1; j < x0.cols-1; j++,u1++)
                    {
                        // Value of I at (i+1,j),(i,j+1)...(i,j)
                        float i0p1=x0.at<float>(i, j+1);
                        float im10=x0.at<float>(i-1, j),i0m1=x0.at<float>(i, j-1),i00=x0.at<float>(i, j);

                        // Value of D at at (i+1,j),(i,j+1)...(i,j)
                        float c0p1=D.at<float>(i, j+1);
                        float cm10=D.at<float>(i-1, j),c0m1=D.at<float>(i, j-1),c00=D.at<float>(i, j);

                        // Equation (7) p632
                        double xx= (c0p1+c00)*(i0p1-i00) + (cm10+c00)*(im10-i00)+ (c0m1+c00)*(i0m1-i00);
                        dI00.at<float>(i, j) = xx;
                        if (maxD<fabs(xx))
                            maxD=fabs(xx);
                        intxx+=fabs(xx);
                        // equation (9)
                    }
            }
            lambda=100/maxD;
            // cout <<" lambda = "<< lambda<<"\t Maxd"<<maxD << "\t"<<intxx<<"\n";
            for (int i = 0; i < x0.rows; i++)
                {
                    float *u1 = (float*)x1.ptr(i);
                    for (int j = 0; j < x0.cols; j++,u1++)
                        {
                            *u1 = x0.at<float>(i, j) + lambda/4*dI00.at<float>(i, j);
                            // equation (9)
                        }
                }

            x1.copyTo(x0);
            x0.convertTo(xc,CV_8U);
            t=t+lambda;
        }

    dest = xc.clone();

}



