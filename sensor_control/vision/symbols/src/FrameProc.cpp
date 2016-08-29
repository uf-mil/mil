#include "FrameProc.h"


const int FrameProc::blur_kernel_size = 3;
const int FrameProc::erode_kernel_size = 3;
const int FrameProc::dilate_kernel_size = 3;

FrameProc::FrameProc()
{

	erode_element = getStructuringElement(MORPH_RECT,Size(2*erode_kernel_size + 1,2*erode_kernel_size+1), Point(erode_kernel_size,erode_kernel_size)) ;

	dilate_element = getStructuringElement(MORPH_RECT,Size( 2* dilate_kernel_size + 1, 2* dilate_kernel_size+1 ), Point(dilate_kernel_size, dilate_kernel_size));
	
	//OpenCV does 0-180 instead of 0-360, so just divide by 2
	red = ColorThresh{Scalar(0, 10, 100), Scalar(30, 255, 255)}; //0 to 30/2 (includes orange)
	red2 = ColorThresh{Scalar(155, 10, 100), Scalar(180, 255, 255)}; // 330/2 to 180
	blue = ColorThresh{Scalar(90,100,100),Scalar(150,255,255)}; // 210/2 to 270/2
	green = ColorThresh{Scalar(30, 85, 25),Scalar(85,255,255)}; // 70/2 to 170/2
	//Green now works... But the crests in the shapes are the same shades as the grass...
	
	//http://graphicdesign.stackexchange.com/questions/16166/what-is-the-relationship-between-hue-saturation-and-value
	
	rgb_frame = Mat();
	hsv_frame = Mat();
	binary_blue_frame = Mat();
	binary_red_frame = Mat();
	binary_green_frame = Mat();
	
	//namedWindow("blured",CV_WINDOW_AUTOSIZE);
	//namedWindow("hsv",CV_WINDOW_AUTOSIZE);
  //namedWindow("blue",CV_WINDOW_AUTOSIZE);
  //namedWindow("green",CV_WINDOW_AUTOSIZE);
  //namedWindow("red",CV_WINDOW_AUTOSIZE);
}
void FrameProc::ErodeDilate()
{
	erode(rgb_frame,rgb_frame,erode_element);
	dilate(rgb_frame,rgb_frame,dilate_element);
}
void FrameProc::ConvertHSV()
{
	cvtColor(rgb_frame,hsv_frame,CV_BGR2HSV);

}
void FrameProc::ThresholdColors()
{
	Mat rtemp, rtemp2;
	inRange(hsv_frame,red.low,red.high,rtemp);
	inRange(hsv_frame,red2.low,red2.high,rtemp2);
	binary_red_frame = rtemp | rtemp2;
	inRange(hsv_frame,blue.low,blue.high,binary_blue_frame);
	inRange(hsv_frame,green.low,green.high,binary_green_frame);
  
  //imshow("blue",binary_blue_frame);
  //imshow("green",binary_green_frame);
  //imshow("red",binary_red_frame);
}
void FrameProc::Prepare(Mat &frame)
{
  rgb_frame = frame;
  
  #ifdef DO_DEBUG
	DebugWindow::UpdateColor(rgb_frame);
  #endif
	ErodeDilate();
	ConvertHSV();
	ThresholdColors();
	
	//imshow("blured",rgb_frame);
	//imshow("hsv",hsv_frame);
}

Mat FrameProc::GetRed()
{
	return binary_red_frame;
}
Mat FrameProc::GetBlue()
{
	return binary_blue_frame;
}
Mat FrameProc::GetGreen()
{
	return binary_green_frame;
}
