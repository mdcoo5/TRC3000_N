#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

bool ctrl_win = 1; // 1 for control box window, 0 otherwise
bool out_thrs = 1; // 1 outputs threshold window
bool out_orig = 0; // 1 outputs original window
bool output = 0;   // 1 turns on left/right output

int iLowH, iHighH;
int iLowS, iHighS;
int iLowV, iHighV;

int main(int argc, char* argv[])
{
  VideoCapture cap(0); // captures from camera 0
  if(!cap.isOpened())
    {
      cout << "Cannot open the camera" << endl;
      return -1;
    }
  
  // Initial opening of threshold value file
  std::fstream fs;
  fs.open("thresh_vals.txt", ios::in);
  if(fs.is_open())
    {
      fs >> iLowH;
      fs >> iHighH;
      fs >> iLowS;
      fs >> iHighS;
      fs >> iLowV;
      fs >> iHighV;
      fs.close();
    }
  else cout << "cannot open file to read threshold vals" << endl;

  // Console output of read threshold values
  cout << "H: " << iLowH;
  cout << " - " << iHighH << endl;
  cout << "S: " << iLowS;
  cout << " - " << iHighS << endl;
  cout << "V: " <<  iLowV;
  cout << " - " << iHighV << endl;

  cout << "TRC3000 Group N OpenCV Checkpoint 3." << endl << endl;
  

  //Prompting of user for mode control
  cout << "1 - Set threshold values" << endl;
  cout << "2 - use existing threshold values" << endl << endl;;
  int choice;
  cout << "choice :";
  cin >> choice;
  switch(choice)
    {
    case 1:
      ctrl_win = 1; out_thrs = 1; out_orig = 0; output = 0;
      cout << "Set final Threshold values in thresh_vals.txt" << endl;
      break;
    case 2:
      ctrl_win = 0; out_thrs = 1; out_orig = 0; output = 1;
      break;
    case 3:
      ctrl_win = 1; out_thrs = 1; out_orig = 1; output = 1;
      break;
    default:
      cout << "Wrong choice" << endl;
      return -1;
    }
  
  if(ctrl_win == 1)
    {
      namedWindow("Control", CV_WINDOW_AUTOSIZE);
  
      cvCreateTrackbar("LowH","Control",&iLowH,179);
      cvCreateTrackbar("HighH","Control",&iHighH,179);
      cvCreateTrackbar("LowS","Control",&iLowS,255);
      cvCreateTrackbar("HighS","Control",&iHighS,255);
      cvCreateTrackbar("LowV","Control",&iLowV,255);
      cvCreateTrackbar("HighV","Control",&iHighV,255);
    }

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


      if(out_thrs == 1) imshow("Thresholded Image",imgFinal);
      if(out_orig == 1) imshow("Original", imgOriginal);

  
      if(waitKey(30) == 27)
	{
	  cout << "esc key pressed" << endl;
	  break;
	}

      if( posX < 319 && output)
	{
	  cout << "Left" << endl;
	}
      else if(output)
	{
	  cout << "Right" << endl;
	}

    }
  return 0;
}
