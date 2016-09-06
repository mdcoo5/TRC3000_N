#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
  VideoCapture cap(0); // captures from camera 0
  if(!cap.isOpened())
    {
      cout << "Cannot open the camera" << endl;
      return -1;
    }

  //namedWindow("Control", CV_WINDOW_AUTOSIZE);

  // Image processing
  int iLowH = 168;
  int iHighH = 179;
  
  int iLowS = 40;
  int iHighS = 255;
  
  int iLowV = 0;
  int iHighV = 255;
  
  //cvCreateTrackbar("LowH","Control",&iLowH,179);
  //cvCreateTrackbar("HighH","Control",&iHighH,179);
  //cvCreateTrackbar("LowS","Control",&iLowS,255);
  //cvCreateTrackbar("HighS","Control",&iHighS,255);
  //cvCreateTrackbar("LowV","Control",&iLowV,255);
  //cvCreateTrackbar("HighV","Control",&iHighV,255);

  int flag = 0;
  int posX, posY;
  
  while(true)
    {
      Mat imgOriginal;

      bool bSuccess = cap.read(imgOriginal);

      if(flag == 0)
	{
	  int rows = imgOriginal.rows;
	  int cols = imgOriginal.cols;

	  cout << "the image is " << cols << " x " << rows << " pixels" << endl;
	  flag = 1;
	}

      if(!bSuccess)
	{
	  cout << "Cannot read from camera" << endl;
	  break;
	}

      Mat imgHSV;

      cvtColor(imgOriginal, imgHSV, CV_BGR2HSV);

      Mat imgThreshold;
      Mat imgFinal;

      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThreshold);
      
      erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
      dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

      dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
      erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

      Moments oMoments = moments(imgThreshold);

      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;

      cvtColor(imgThreshold, imgFinal, CV_GRAY2BGR);

      if(dArea > 10000)
	{
	  posX = dM10 / dArea;
	  posY = dM01 / dArea;
	  circle(imgFinal, Point(posX, posY), 5, Scalar(0, 0, 255), -1, 8);
	}

      line(imgFinal, Point(319, 0), Point(319, 479), Scalar(0, 255, 0), 2, 8);

      if( posX < 319)
	{
	  cout << "Left" << endl;
	}
      else
	{
	  cout << "Right" << endl;
	}
      
      imshow("Thresholded Image",imgFinal);
      //imshow("Original", imgOriginal);
  
      if(waitKey(30) == 27)
	{
	  cout << "esc key pressed" << endl;
	  break;
	}
    }
  return 0;
}

    
