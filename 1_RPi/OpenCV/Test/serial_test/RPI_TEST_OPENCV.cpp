/*
TRC3000 Group N OpenCV checkpoint 3 code.
Author: MC
Written: 09/2106
Purpose: To capture an image from a connected webcam and perform colour thresholding,
noise reduction and location analysis to determine where in frame coloured object is.
Outputs: outputs left or right values to the Raspberry Pi's bluetooth serial port to be
read and processed by BBB.
*/
/*
 * TODO: Estimate angle of robot based on 2 markers on top of robot
 * TODO: Compute directions for robot to take t get to final marker
 */

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
#include <math.h>

#define BAUDRATE B9600
#define DEVICE "/dev/rfcomm0"
#define FALSE 0
#define TRUE 1
#define START_BYTE 0x7F
#define ANGLE_SMA 1
#define ANG_SMA_PERIOD 5

float sma[ANG_SMA_PERIOD];

using namespace cv;
using namespace std;

bool ctrl_win = 1; // 1 for control box window, 0 otherwise
bool out_thrs = 1; // 1 outputs threshold window
bool out_orig = 0; // 1 outputs original window
bool output = 0;   // 1 turns on left/right output
bool green = 1;
bool orange = 1;

int iLowH, iHighH;
int iLowS, iHighS;
int iLowV, iHighV;


//Location of cones
int cones[20][2];
//Location of objects
int objects[20][2]; // [no.][x,y]


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
  cout << "Marker Threshold values: " << endl;
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
    case 4:
      ctrl_win = 0; out_thrs = 0; out_orig = 0; output = 1;
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

  Mat imgOriginal;
  bool bSuccess;

  // Initialiation
  for(int k=0; k< 10; k++){
    bSuccess = cap.read(imgOriginal);
    waitKey(30);
  }
  
  // Read Image
  bSuccess = cap.read(imgOriginal);

  // Get size of image
  int rows = imgOriginal.rows;
  int cols = imgOriginal.cols;
  cout << "The image is " << cols << " x " << rows << " pixels" << endl;

  Mat imgHSV;

  // Colour converted image
  cvtColor(imgOriginal, imgHSV, CV_BGR2HSV);

  // Binary Thresholded Image & Final image
  Mat imgThreshold;
  Mat imgFinal;
  Mat dst = Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC3);  

  if(green){
    cout << "Green" << endl;
    // Processing to find GREEN cones
    inRange( imgHSV, Scalar(0, 111, 162), Scalar(179, 255, 255), imgThreshold);

    // Noise Reduction
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    // Find Contours of Image
    vector<vector<Point> > contours;
    vector<Vec4i> heirarchy;
    
    findContours( imgThreshold, contours, heirarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    
    cout << "processing" << endl;
    
    //iterate through top_level components
    int idx = 0;
    int count = 0;
    for( ; idx >= 0; idx = heirarchy[idx][0])
      {
	Scalar color( rand()&255, rand()&255, rand()&255);
	drawContours( dst, contours, idx, color, 2, 8, heirarchy);
	Moments cMoments = moments(contours[idx]);
	cones[idx][0] = (int)(cMoments.m10/cMoments.m00);
	cones[idx][1] = (int)(cMoments.m01/cMoments.m00);
	circle(dst, Point(cones[idx][0],cones[idx][1]), 3, Scalar(0, 255, 0), -1);
	count ++;
      }
    cout << "contours: " << count << endl;
    for(int k=0;k <3; k++)
      {
	cout << "x: " << cones[k][0] << " y: " << cones[k][1] << endl;
      }
  }
  if(orange){
    cout << "Orange" << endl;
    //Processing to find ORANGE Objects
    inRange(imgHSV, Scalar(0, 0 , 216), Scalar(69, 255, 255), imgThreshold);
    
    // Noise Reduction
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    
    // Find Contours of Image
    vector<vector<Point> > contours;
    vector<Vec4i> heirarchy;
    findContours( imgThreshold, contours, heirarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    
    //iterate through top_level components
    int idx = 0;
    int count = 0;
    for( ; idx >= 0; idx = heirarchy[idx][0])
      {
	Scalar color( rand()&255, rand()&255, rand()&255);
	drawContours( dst, contours, idx, color, 2, 8, heirarchy);
	Moments cMoments = moments(contours[idx]);
	objects[idx][0] = (int)(cMoments.m10/cMoments.m00);
	objects[idx][1] = (int)(cMoments.m01/cMoments.m00);
	circle(dst, Point(objects[idx][0],objects[idx][1]), 3, Scalar(0, 255, 0), -1);
	count ++;
      }
    cout << "orange contours: " << count << endl;
    for(int k=0;k <3; k++)
      {
	cout << "x: " << objects[k][0] << " y: " << objects[k][1] << endl;
      }
  }
  namedWindow( "Contours" , 1);
  imshow("Contours", dst);
  
  waitKey(30);
  
  while(true)
    {
      // imgOriginal is captured from camera
      bSuccess = cap.read(imgOriginal);

      if(!bSuccess)
	{
	  cout << "Cannot read from camera" << endl;
	  break;
	}

      // Colour converted image
      cvtColor(imgOriginal, imgHSV, CV_BGR2HSV);

      // Threshold image based on values read from file
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
      double dM20 = oMoments.m20;
      double dM02 = oMoments.m02;
      double dM11 = oMoments.m11;
      double dArea = oMoments.m00;

      cvtColor(imgThreshold, imgFinal, CV_GRAY2BGR);

      // Draw Green Circles at centre of Objects
      circle(imgFinal, Point(cones[0][0], cones[0][1]), 20, Scalar(0,255,0), -1, 8);
      circle(imgFinal, Point(cones[1][0], cones[1][1]), 20, Scalar(0,255,0), -1, 8);
      circle(imgFinal, Point(cones[2][0], cones[2][1]), 20, Scalar(0,255,0), -1, 8);

      circle(imgFinal, Point(objects[0][0], objects[0][1]), 20, Scalar(0,255,0), -1, 8);
      circle(imgFinal, Point(objects[1][0], objects[1][1]), 20, Scalar(0,255,0), -1, 8);
      circle(imgFinal, Point(objects[2][0], objects[2][1]), 20, Scalar(0,255,0), -1, 8);

      
      if(dArea > 10000)
	     {
	        posX = dM10 / dArea;
	         posY = dM01 / dArea;
	          // Draw red circle on outputted image at centre of 'mass'
	           circle(imgFinal, Point(posX, posY), 5, Scalar(0, 0, 255), -1, 8);
	          }

      // Draw line from centre of robot to final marker
      line(imgFinal, Point(posX, posY), Point(objects[0][0], objects[0][1]), Scalar(0, 255, 0), 1, 8);
      float heading = atan2(objects[0][0] - posX, objects[0][1] - posY);
      
      // Tilt Calculation
      float tilt = -0.5 * atan2(2.0*((dM11/dArea)-posX*posY),((dM20/dArea)-(posX*posX))-((dM02/dArea)-(posY*posY)));
      int length = 150;
      line(imgFinal, Point(posX, posY), Point((int) posX + length*cos(tilt), (int)posY - length*sin(tilt)), Scalar(0, 0, 255), 1, 8);

      heading = heading - tilt - (M_PI/2);
      heading = (heading*180.0)/M_PI;
      tilt = (tilt*180.0)/M_PI;

      cout << "tilt: " << tilt;
      cout << " difference: " << heading << endl;

      if(out_thrs == 1) imshow("Thresholded Image",imgFinal);
      if(out_orig == 1) imshow("Original", imgOriginal);

      if(waitKey(30) == 27)
	{
	  cout << "esc key pressed" << endl;
	  break;
	}

int num = 0;
/* RPI ->  BBB DATA STRUCTURE
 *
 * 9 bytes
 * data[0] - START BYTE
 * data[1] - MSG Length
 * data[2] - CHECKSUM
 * ------------------
 * data[3] - MSG TYPE
 * data[4] -
 * data[5] - 
 * data[6] - 
 * data[7] - 
 * data[8] = '\r' - return for bbb comm termination
 */
 
 unsigned char data[9];
 data[0] = START_BYTE;
 data[1] = 1; //MSG length
 
 	if( heading > 5 )
   	{
     	//num = write(fd,left,sizeof(left)-1);
     	cout << "Turn Left" << endl;
     	data[3] = 0x04; // Left data type
   	}
 	else if(heading < 5 && heading > -5)
   	{
     	//num = write(fd,right,sizeof(right)-1);
     	cout << "Go Straight" << endl;
     	data[3] = 0x01; //forward data type
   	}
 	else
   	{
     	//num = write(fd, ....
     	cout << "Turn Right" << endl;
     	data[3] = 0x08; // Right data type
  	 }
  	 
  	if( sqrt((objects[0][0] - posX)^2 + (objects[0][1] - posY)^2) < 10){
   		cout << "Stop" << endl;
   		data[3] = 0xFF; // stop data type
 	}
  	 data[4] = 0x00;
  	 data[5] = 0x00;
  	 data[6] = 0x00;
  	 data[7] = 0x00;
  	 
  	 data[2] = ~(data[3] + data[4] + data[5] + data[6] + data[7]);
  	 
  	 data[9] = '\r'; //return byte for BBB handling
  	 num = write(fd, data, sizeof(data));
} // end while
    
  	tcsetattr(fd, TCSANOW, &oldtio);
  	close(fd);
  	
  return 0;
} // end main
