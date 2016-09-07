#include <navigator_vision_lib/stereomodelfitter.h>

StereoModelFitter::StereoModelFitter(PerceptionModel model, image_transport::Publisher debug_publisher):
    model(model), debug_publisher(debug_publisher)
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

void StereoModelFitter::extract_features(vector<Point> & features, Mat& image, int max_corners, int block_size, double quality_level, double min_distance)
{
//  vector<KeyPoint> kp;
//  FAST(image, kp, .00005, true);
//  for(KeyPoint k : kp){
//    features.push_back(k.pt);
//  }
  goodFeaturesToTrack(image, features, max_corners, quality_level, min_distance, Mat(), block_size, true, .001);

}


void StereoModelFitter::extract_features_1( Mat& image_left, Mat& image_right,  int max_corners, int block_size, double quality_level, double min_distance)
{
//  cv::OrbFeatureDetector detector(max_corners);
//  vector<KeyPoint> keypoints_left, keypoints_right;
//  detector.detect(image_left, keypoints_left);
//  detector.detect(image_right, keypoints_right);

  vector<Point> features_left, features_right;
  goodFeaturesToTrack(image_left, features_left, max_corners, quality_level, min_distance, Mat(), block_size, true, .001);
   goodFeaturesToTrack(image_right, features_right, max_corners, quality_level, min_distance, Mat(), block_size, true, .001);

  vector<KeyPoint> keypoints_left, keypoints_right;
  for(Point p : features_left){
      KeyPoint k = KeyPoint(p, 4);
      keypoints_left.push_back(k);
  }
  for(Point p : features_right){
      KeyPoint k = KeyPoint(p, 4);
      keypoints_right.push_back(k);
  }


  cv::Mat il1 = image_left.clone();
  cv::Mat ir1 = image_right.clone();

  for(size_t i = 0; i < keypoints_left.size(); i++)
  {
      Scalar color(255, 0, 255);
      stringstream label;
      label << i;
      circle(il1, keypoints_left[i].pt, 5, color, -1);
      putText(il1, label.str(), keypoints_left[i].pt, FONT_HERSHEY_SIMPLEX,
              0.0025 * il1.rows, Scalar(0, 0, 0), 2);

  }

  for(size_t i = 0; i < keypoints_left.size(); i++)
  {
      Scalar color(255, 0, 255);
      stringstream label;
      label << i;
      circle(ir1, keypoints_right[i].pt, 5, color, -1);
      putText(ir1, label.str(), keypoints_right[i].pt, FONT_HERSHEY_SIMPLEX,
              0.0025 * ir1.rows, Scalar(0, 0, 0), 2);

  }

  cv::imshow("knnimgleftpts",il1);
  cv::imshow("knnimgrightpts", ir1);


  cv::BriefDescriptorExtractor extractor;
  Mat descriptors_left, descriptors_right;
  extractor.compute(image_left, keypoints_left, descriptors_left);
  extractor.compute(image_right, keypoints_right, descriptors_right);

  BFMatcher matcher(NORM_L2);
  std::vector<vector<DMatch>> matches;
  matcher.knnMatch(descriptors_left, descriptors_right, matches,2);

  std::vector<DMatch> match_left;
  std::vector<DMatch> match_right;

  for(int i=0; i<matches.size(); i++)
  {
      match_left.push_back(matches[i][0]);
      match_right.push_back(matches[i][1]);
  }

    //= cv::DescriptorExtractor.create("ORB");


  cv::Mat il = image_left.clone();
  cv::Mat ir = image_right.clone();

  for(size_t i = 0; i < match_left.size(); i++)
  {
      Scalar color(255, 0, 255);
      stringstream label;
      label << i;
      circle(il, keypoints_left[match_left[i].queryIdx].pt, 5, color, -1);
      putText(il, label.str(), keypoints_left[match_left[i].queryIdx].pt, FONT_HERSHEY_SIMPLEX,
              0.0025 * il.rows, Scalar(0, 0, 0), 2);

      circle(ir, keypoints_right[match_right[i].trainIdx].pt, 5, color, -1);
      putText(ir, label.str(), keypoints_right[match_right[i].trainIdx].pt, FONT_HERSHEY_SIMPLEX,
              0.0025 * ir.rows, Scalar(0, 0, 0), 2);
  }
  cv::imshow("matchesl",il);
  cv::imshow("matchedr", ir);
  cv::waitKey();

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

void StereoModelFitter::get_corresponding_pairs_1(vector<int>& correspondence_pair_idxs,
        vector<Point> features_l,
        vector<Point> features_r,
        vector<Point>& points_l,
        vector<Point>& points_r,
        Mat r_diffused_draw_1,
        Mat l_diffused_draw_1)
{

  vector<Point> feature_right_new;
  for(int a : correspondence_pair_idxs){
      feature_right_new.push_back(features_r[a]);
  }

  cv::Mat ml = Mat(features_l.size(), 2, CV_32F);
  cv::Mat mr = Mat(features_l.size(), 2, CV_32F);
  for(int i = 0; i != features_l.size(); ++i){
    ml.at<float>(i,0) = features_l[i].x;
    ml.at<float>(i,1) = features_l[i].y;
  }
  for(int i = 0; i != feature_right_new.size(); ++i){
    mr.at<float>(i,0) = feature_right_new[i].x;
    mr.at<float>(i,1) = feature_right_new[i].y;
  }


  cv::Mat F = findFundamentalMat(ml, mr, cv::FM_RANSAC);
  vector<Point3d> fl, fr;
  for(int i = 0; i != features_l.size(); ++i){
      fl.push_back(Point3d(features_l[i].x, features_l[i].y, 1));
      fr.push_back(Point3d(features_r[i].x, features_r[i].y, 1));
  }

  for(int i = 0; i != features_l.size(); ++i){
       Mat x1 = Mat(fr[i]);
       x1 = x1.t();
       Mat x2 = Mat(fl[i]);
       cv::Mat result = x1 * F * x2;
       std::cout<<result<<std::endl;
       if(fabs(result.at<float>(0,0))<1.0){
          points_l.push_back(features_l[i]);
          points_r.push_back(feature_right_new[i]);
       }
  }

  for(size_t i = 0; i < fr.size(); i++)
  {
      Scalar color(255, 0, 255);
      stringstream label;
      label << i;
      cv::Mat point_left = F * Mat(fr[i]);
      float a = point_left.at<float>(0,0)/point_left.at<float>(2,0);
      float b = point_left.at<float>(1,0)/point_left.at<float>(2,0);
      Point p = Point(a,b);
      circle(l_diffused_draw_1, p, 5, color, -1);
      putText(l_diffused_draw_1, label.str(), p, FONT_HERSHEY_SIMPLEX,
              0.0025 * l_diffused_draw_1.rows, Scalar(0, 0, 0), 2);
  }

  imshow("F matrix project left", l_diffused_draw_1);
  waitKey();




//  cv::Mat correct_left, correct_right;
//  correctMatches(F, features_l, feature_right_new, correct_left, correct_right);

//  std::cout<<F<<std::endl;
//  std::cout<<ml<<std::endl;
//  std::cout<<mr<<std::endl;

//  correct_left.copyTo(points_ l);
//  correct_right.copyTo(points_r);
//  for(int i = 0; i != correct_left.rows; ++i){
//    points_l.push_back(Point(correct_left.at<float>(i,0), correct_left.at<float>(i,1)));
//  }

//  for(int i = 0; i != correct_right.rows; ++i){
//     points_r.push_back(Point(correct_right.at<float>(i,0), correct_right.at<float>(i,1)));
//  }
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

bool StereoModelFitter::check_for_model(vector<Eigen::Vector3d>  feature_pts_3d, vector<Eigen::Vector3d>& correct_model, vector<Point> correct_image_points)
{
//    std::string input = "";
//    cin >> input;
//    if(input != "skip"){
//        decision_tree(feature_pts_3d, 0, model.min_points, true);
//    }else{


  vector<int> boop;

  //  string bin;
  //  cin >> bin;
//  if(bin != "done"){
//    boop  = split(bin);

//  }

        decision_tree(feature_pts_3d, 0, model.min_points, boop, false);
    //}
    model.get_model(correct_model, correct_image_points, *current_image_right, *left_cam_mat);
    model.clear();
    return true;
}

void StereoModelFitter::decision_tree(vector<Eigen::Vector3d>  feature_pts_3d, int curr, int left, vector<int> fuck1, bool fuck2)
{
    int total = feature_pts_3d.size();

    // If the current model has the right amount of points or we have no more combinations left to check (the second is condition is unecessary, but for safety)
    if(!model.complete() || left > 0)
        {
            for(int i = curr + 1; i <= total - left; ++i)
                {
                    Eigen::Vector3d point = feature_pts_3d[i];
                    bool val = false;
                    int a = model.current_points.size();
                    if(((std::find(fuck1.begin(), fuck1.end(), i) != fuck1.end()) && a == 0) || fuck2){
                      fuck2 = true;
                      val = model.check_point(point, *current_image_left, *left_cam_mat, true);
                    }else{
                      val = model.check_point(point, *current_image_left, *left_cam_mat, false);
                      fuck2 = false;
                    }

                    // If that point was correct, keep checking the rest of the points
                    if(val)
                        {
                            decision_tree(feature_pts_3d, i, left - 1, fuck1, fuck2);
                        }
                    // Remove that point from the model so that other points can be checked
                    model.remove_point(point);
                }
        }
}

std::vector<int> StereoModelFitter::split(string str){
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

void StereoModelFitter::visualize_points(vector<Eigen::Vector3d>  feature_pts_3d,
        Mat& img)
{
    // visualize reconstructions
    //cout<<feature_pts_3d.size()<<endl;
    cv::Mat current_image_left = img.clone();
    for(size_t i = 0; i < feature_pts_3d.size(); i++)
    {
        Eigen::Vector3d pt = feature_pts_3d[i];
        //cout<<pt[0]<<","<<pt[1]<<","<<pt[2]<<endl;
        Matx41d position_hom(pt(0), pt(1), pt(2), 1);
        Matx31d pt_L_2d_hom = *left_cam_mat * position_hom;
        Point2d L_center2d(pt_L_2d_hom(0) / pt_L_2d_hom(2), pt_L_2d_hom(1) / pt_L_2d_hom(2));
        Scalar color(255, 0, 255);
        stringstream label;
        label << i;
        //std::cout<<i<<": " <<pt(0)<<","<<pt(1)<<","<<pt(2)<<std::endl;
        circle(current_image_left, L_center2d, 5, color, -1);
        putText(current_image_left, label.str(), L_center2d, FONT_HERSHEY_SIMPLEX,
                0.0015 * current_image_left.rows, Scalar(0, 0, 0), 2);
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image_left).toImageMsg();
    debug_publisher.publish(msg);
    ros::spinOnce();

//    string input;
//    cin >> input;
//    while(input != "done"){
//      int a, b;
//      std::size_t pos = input.find(",");
//      string t = input.substr(0, pos);
//      string s = input.substr(pos+1);
//      istringstream (t ) >> a;
//      istringstream (s) >> b;
//      std::cout<<std::endl;

//      Eigen::Vector3d diff = feature_pts_3d[a] - feature_pts_3d[b];
//      float dist_from_starting = sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));
//      std::cout<<dist_from_starting<<std::endl;
//      cin >> input;

//    }

}

bool StereoModelFitter::determine_model_position(Eigen::Vector3d& position,
        int max_corners, int block_size, double min_distance, double image_proc_scale,int diffusion_time,
        Mat current_image_left, Mat current_image_right,
        Matx34d left_cam_mat, Matx34d right_cam_mat)
{
    this->image_proc_scale = image_proc_scale;

    this->left_cam_mat = &left_cam_mat;
    this->right_cam_mat = &right_cam_mat;
    this->current_image_left = &current_image_left;
    this->current_image_right = &current_image_right;

    Mat l_diffused, r_diffused;
    denoise_images(l_diffused, r_diffused, diffusion_time, current_image_right, current_image_left);

    // Extract Features
    //l_diffused = current_image_left;
    //r_diffused = current_image_right;
    vector< Point > features_l, features_r;
    Mat l_diffused_draw = l_diffused.clone();
    Mat r_diffused_draw = r_diffused.clone();
    double quality_level = 0.05;
    extract_features(features_r, r_diffused_draw, 30, block_size,
                     quality_level, min_distance);
    extract_features(features_l, l_diffused_draw, 30, block_size,
                     quality_level, min_distance);

    Mat l_diffused_draw_2 = l_diffused.clone();
    Mat r_diffused_draw_2 = r_diffused.clone();

    Mat l_diffused_draw_4 = l_diffused.clone();
    Mat r_diffused_draw_4 = r_diffused.clone();
    Scalar color(255, 0, 255);


    for(size_t i = 0; i < features_r.size(); i++)
        {
            Point pt = features_r[i];
            stringstream label;
            label << i;
            circle(r_diffused_draw_2, pt, 5, color, -1);
            putText(r_diffused_draw_2, label.str(), pt, FONT_HERSHEY_SIMPLEX,
                    0.0015 * r_diffused_draw_2.rows, Scalar(0, 0, 0), 2);
        }
     cv::imshow("FeaturesToTrackRight", r_diffused_draw_2);
     for(size_t i = 0; i < features_l.size(); i++)
         {
             Point pt = features_l[i];
             stringstream label;
             label << i;
             circle(l_diffused_draw_2, pt, 5, color, -1);
             putText(l_diffused_draw_2, label.str(), pt, FONT_HERSHEY_SIMPLEX,
                     0.0015 * l_diffused_draw_2.rows, Scalar(0, 0, 0), 2);
         }
      cv::imshow("FeaturesToTrackLeft", l_diffused_draw_2);
      cv::waitKey(33);



     Mat l_diffused_draw_1 = l_diffused.clone();
    Mat r_diffused_draw_1 = r_diffused.clone();

//     extract_features_1(l_diffused_draw_1, r_diffused_draw_1, max_corners, block_size,
//                       quality_level, min_distance);

    // Stereo Matching
   // vector<int> correspondence_pair_idxs;
vector<Point> points_l, points_r;
get_corresponding_pairs(features_l, features_r, points_l, points_r, l_diffused.rows);
//    std::cout<<"Right"<<std::endl;
//    for(size_t i = 0; i < points_r.size(); i++)
//        {
//            Point pt = points_r[i];
//            stringstream label;
//            label << i;

//            std::cout<<i<<": "<<pt.x<<","<<pt.y<<std::endl;
//            circle(r_diffused_draw_4, pt, 5, color, -1);

//            putText(r_diffused_draw_4, label.str(), pt, FONT_HERSHEY_SIMPLEX,
//                    0.0015 * r_diffused_draw_4.rows, Scalar(0, 0, 0), 2);
//        }

//    std::cout<<"LEFT"<<std::endl;
//     cv::imshow("CorrespondingPairsRight", r_diffused_draw_4);
//     for(size_t i = 0; i < points_l.size(); i++)
//         {
//             Point pt = points_l[i];
//             stringstream label;
//             label << i;
//             std::cout<<i<<": "<<pt.x<<","<<pt.y<<std::endl;
//             circle(l_diffused_draw_4, pt, 5, color, -1);
//             putText(l_diffused_draw_4, label.str(), pt, FONT_HERSHEY_SIMPLEX,
//                     0.0015 * l_diffused_draw_4.rows, Scalar(0, 0, 0), 2);
//         }
//      cv::imshow("CorrespondingPairLeft", l_diffused_draw_4);
//      cv::waitKey(0);

    //get_corresponding_pairs_1(correspondence_pair_idxs, features_l, features_r, points_l, points_r, r_diffused_draw_4, l_diffused_draw_4);


//    for(size_t i = 0; i < correspondence_pair_idxs.size(); i++)
//    {
//        Scalar color(255, 0, 255);
//        stringstream label;
//        label << i;
//        circle(l_diffused_draw, features_l[i], 5, color, -1);
//        putText(l_diffused_draw, label.str(), features_l[i], FONT_HERSHEY_SIMPLEX,
//                0.0025 * l_diffused_draw.rows, Scalar(0, 0, 0), 2);

//        circle(r_diffused_draw, features_r[correspondence_pair_idxs[i]], 5, color, -1);
//        putText(r_diffused_draw, label.str(), features_r[correspondence_pair_idxs[i]], FONT_HERSHEY_SIMPLEX,
//                0.0025 * r_diffused_draw.rows, Scalar(0, 0, 0), 2);
//    }
//    cv::imshow("imgleft",l_diffused_draw);
//    cv::imshow("imgright", r_diffused_draw);

//    for(size_t i = 0; i < correspondence_pair_idxs.size(); i++)
//    {
//        Scalar color(255, 0, 255);
//        stringstream label;
//        label << i;
//        circle(l_diffused_draw_1, points_l[i], 5, color, -1);
//        putText(l_diffused_draw_1, label.str(), points_l[i], FONT_HERSHEY_SIMPLEX,
//                0.0025 * l_diffused_draw_1.rows, Scalar(0, 0, 0), 2);

//        circle(r_diffused_draw_1, points_r[i], 5, color, -1);
//        putText(r_diffused_draw_1, label.str(), points_r[i], FONT_HERSHEY_SIMPLEX,
//                0.0025 * r_diffused_draw_1.rows, Scalar(0, 0, 0), 2);
//    }

//    cv::imshow("imgleftbetter",l_diffused_draw_1);
//    cv::imshow("imgrightbetter", r_diffused_draw_1);
//    cv::waitKey();

    // Calculate 3D stereo reconstructions
    vector<Eigen::Vector3d> feature_pts_3d;
    calculate_3D_reconstruction(feature_pts_3d, points_l, points_r);


    vector<Eigen::Vector3d> correct_model;
    vector<Point> correct_image_points;

    visualize_points(feature_pts_3d, current_image_right);
    check_for_model(feature_pts_3d, correct_model, correct_image_points);


    return true;
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



