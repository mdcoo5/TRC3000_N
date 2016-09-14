/*
TRC3000 Group N OpenCV checkpoint 3 code.
Author: MC
Written: 09/2106
Purpose: To capture an image from a connected webcam and perform colour thresholding,
noise reduction and location analysis to determine where in frame coloured object is.
Outputs: outputs left or right values to the Raspberry Pi's bluetooth serial port to be
read and processed by BBB.
*/
//TODO: Convert file read process to separate function/header.
//TODO: Change output from console to Serial Port.

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BAUDRATE B9600
#define DEVICE "/dev/rfcomm0"
#define FALSE 0
#define TRUE 1

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
  int fd;
  struct termios oldtio, newtio;
  if(!cap.isOpened())
    {
      cout << "Cannot open the camera" << endl;
      return -1;
    }

  fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd < 0) { cout << "Error opening device" << endl; return -1; }
  else cout << "Device opened successfully" << endl;

  unsigned char left[] = "left\r";
  unsigned char right[] = "right\r";

  tcgetattr(fd, &oldtio); // save current serial port settings
  bzero(&newtio, sizeof(newtio)); // clear struct to recieve new settings

  newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag = ICANON;

  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  // Initial opening and read of threshold value file
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

  cout << "TRC3000 Group N OpenCV Checkpoint 3." << endl << endl;

  // Console output of read threshold values
  cout << "H: " << iLowH;
  cout << " - " << iHighH << endl;
  cout << "S: " << iLowS;
  cout << " - " << iHighS << endl;
  cout << "V: " <<  iLowV;
  cout << " - " << iHighV << endl << endl;

  //Prompting of user for mode control
  cout << "Options:" << endl;
  cout << "1 - Set threshold values" << endl;
  cout << "2 - use existing threshold values" << endl << endl;;
  int choice;
  cout << "choice :";
  cin >> choice;
  switch(choice)
    {
    case 1: //Threshold control
      ctrl_win = 1; out_thrs = 1; out_orig = 0; output = 0;
      cout << "Set final Threshold values in thresh_vals.txt" << endl;
      break;
    case 2: //Run with set thresholds
      ctrl_win = 0; out_thrs = 1; out_orig = 0; output = 1;
      break;
    case 3: //debug
      ctrl_win = 1; out_thrs = 1; out_orig = 1; output = 1;
      break;
    default: //default errors out
      cout << "Wrong choice" << endl;
      return -1;
    }

  if(ctrl_win == 1) //Opens threshold control window
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
      // imgOriginal is img captured from camera
      Mat imgOriginal;
      bool bSuccess = cap.read(imgOriginal);

      // Get size of image
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

      // Colour converted image
      Mat imgHSV;

      cvtColor(imgOriginal, imgHSV, CV_BGR2HSV);

      // binary threshold image and BGR final image
      Mat imgThreshold;
      Mat imgFinal;

      // Treshold image based on values read from file
      inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThreshold);

      // Noise reduction via open/close
      erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
      dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

      dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));
      erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(3,3)));

      // Moments function to find centre of blob
      Moments oMoments = moments(imgThreshold);

      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;

      cvtColor(imgThreshold, imgFinal, CV_GRAY2BGR);

      if(dArea > 10000)
	     {
	        posX = dM10 / dArea;
	         posY = dM01 / dArea;
	          // Draw red circle on outputted image at centre of 'mass'
	           circle(imgFinal, Point(posX, posY), 5, Scalar(0, 0, 255), -1, 8);
	          }

      // Draw line down centre of image to denote left/right side
      line(imgFinal, Point(319, 0), Point(319, 479), Scalar(0, 255, 0), 2, 8);

      if(out_thrs == 1) imshow("Thresholded Image",imgFinal);
      if(out_orig == 1) imshow("Original", imgOriginal);

      if(waitKey(30) == 27)
	{
	  cout << "esc key pressed" << endl;
	  break;
	}

int num = 0;
      if( posX < 319 && output)
	{
    num = write(fd,left,sizeof(left)-1);
    cout << "L Data Sent: " << num << " bytes" << endl;
	}
      else if(output)
	{
    num = write(fd,right,sizeof(right)-1);
    cout << "R Data Sent: " << num << " bytes" << endl;
	}
    }
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
  return 0;
}
