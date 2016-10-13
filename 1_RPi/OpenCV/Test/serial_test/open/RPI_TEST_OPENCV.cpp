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
#define ANG_SMA_PERIOD 2

float sma[ANG_SMA_PERIOD];
int sma_ptr = 0;

int pt1[2];
int pt2[2];
int pt3[2];
int pt4[2];
int pt5[2];
int ptStart[2];
int ptStop[2];

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

float tilt;

//Location of cones
int cones[20][2];
//Location of objects
int objects[20][2]; // [no.][x,y]

int state = 1;
int nextpt = 0;
int sharp = 0;

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
  cout << "2 - use existing threshold values" << endl;
  cout << "3 - All windows" << endl;
  cout << "4 - Competition Settings, only initial contours" << endl << endl;
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
  for(int k=0; k< 30; k++){
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
    inRange( imgHSV, Scalar(40, 111, 162), Scalar(80, 255, 255), imgThreshold);

    // Noise Reduction
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));

    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
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
	cout << "idx: " << idx << " x: " << cones[idx][0] << " y: " << cones[idx][1] << endl;
      }
    cout << "contours: " << count << endl;
  }
  if(orange){
    cout << "Orange" << endl;
    //Processing to find ORANGE Objects
    inRange(imgHSV, Scalar(0, 0 , 200), Scalar(40, 255, 255), imgThreshold);
    
    // Noise Reduction
    erode(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
    
    dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(10,10)));
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
	int orange_x = (int)(cMoments.m10/cMoments.m00);
	int orange_y = (int)(cMoments.m01/cMoments.m00);	
	objects[idx][0] = orange_x;
	objects[idx][1] = orange_y;
	circle(dst, Point(objects[idx][0],objects[idx][1]), 3, Scalar(0, 255, 0), -1);
	count ++;
	cout << "idx: " << idx << " x: " << objects[idx][0] << " y: " << objects[idx][1] << endl;
      }
    cout << "orange contours: " << count << endl;
  }
  namedWindow( "Contours" , 1);
  imshow("Contours", dst);
  
  waitKey(30);

  pt1[0] = (cones[0][0] + cones[1][0] + cones[2][0])/3 - 50;
  pt1[1] = (cones[1][1] + cones[2][1])/2;
  pt2[0] = (cones[0][0] + cones[1][0] + cones[2][0])/3;
  pt2[1] = (cones[1][1] + cones[2][1])/2;
  pt3[0] = (cones[0][0] + cones[1][0] + cones[2][0])/3 + 50;
  pt3[1] = (cones[1][1] + cones[2][1])/2;
  
  int ramp[2];
  int Start[2];
  int Stop[2];
  
  if(objects[0][0] < 200){Start[0] = objects[0][0]; Start[1] = objects[0][1];}
  if(objects[1][0] < 200){Start[0] = objects[1][0]; Start[1] = objects[1][1];}
  if(objects[2][0] < 200){Start[0] = objects[2][0]; Start[1] = objects[2][1];}

  ptStart[0] = Start[0];
  ptStart[1] = Start[1];

  if(objects[0][0] > 500){Stop[0] = objects[0][0]; Stop[1] = objects[0][1];}
  if(objects[1][0] > 500){Stop[0] = objects[1][0]; Stop[1] = objects[1][1];}
  if(objects[2][0] > 500){Stop[0] = objects[2][0]; Stop[1] = objects[2][1];}

  ptStop[0] = Stop[0];
  ptStop[1] = Stop[1];
  
  if(objects[0][0] > 200 && objects[0][0] <500) {ramp[0] = objects[0][0]; ramp[1] = objects[0][1];}
  else if(objects[1][0] > 200 && objects[1][0] < 500) {ramp[0] = objects[1][0]; ramp[1] = objects[1][1];}
  else if(objects[2][0] > 200 && objects[2][0] < 500) {ramp[0] = objects[2][0]; ramp[1] = objects[2][1];}
  
  pt4[0] = ramp[0] - 60;
  pt4[1] = ramp[1];
  pt5[0] = ramp[0] + 60;
  pt5[1] = ramp[1];  
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
      dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(6,6)));

      dilate(imgThreshold, imgThreshold, getStructuringElement(MORPH_ELLIPSE, Size(6,6)));
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

      // Draw line from centre of robot to point
      int point[2];
	switch(state){
	case 1:
	  point[0] = pt1[0];
	  point[1] = pt1[1];
	  break;
	case 2:
	  point[0] = ramp[0];
	  point[1] = ramp[1];
	case 500:
	  point[0] = ramp[0];
	  point[1] = ramp[1];
	  break;
	case 501:
	  point[0] = ramp[0];
	  point[1] = ramp[1];
	  break;
	case 3:
	  point[0] = ptStop[0];
	  point[1] = ptStop[1];
	  break;
	case 6:
	  point[0] = ptStop[0];
	  point[1] = ptStop[1];
	  break;
	case 7:
	  point[0] = pt5[0];
	  point[1] = pt5[1];
 	  break;
	case 8:
	  point[0] = pt4[0];
	  point[1] = pt4[1];
	  break;
 	case 9:
	  point[0] = pt3[0];
 	  point[1] = pt3[1];
	  break;
	case 10:
	  point[0] = pt1[0];
	  point[1] = pt1[1];
	  break;
	case 11:
	  point[0] = ptStart[0];
	  point[1] = ptStart[1];
	  break;
	}
	line(imgFinal, Point(posX, posY), Point(point[0], point[1]), Scalar(0, 255, 0), 1, 8);
	float heading = atan2(point[0] - posX, point[1] - posY);
      
      // Tilt Calculation
      //float tilt = -0.5 * atan2(2.0*((dM11/dArea)-posX*posY),((dM20/dArea)-(posX*posX))-((dM02/dArea)-(posY*posY)));
      // find blue contours
      vector<vector<Point> > bcontours;
      vector<Vec4i> bheirarchy;
      int bcentres[2][2];
      findContours(imgThreshold, bcontours, bheirarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

      for(int l=0; l < bcontours.size(); l++){
	Moments bmoments = moments(bcontours[l]);
	bcentres[l][0] = (int) bmoments.m10/bmoments.m00;
	bcentres[l][1] = (int) bmoments.m01/bmoments.m00;
	//drawContours(imgFinal, bcontours, 0, Scalar(0,255,0), 2, 8, bheirarchy);
	//drawContours(imgFinal, bcontours, 1, Scalar(255, 0,0), 2, 8, bheirarchy);
      }
      if(bcentres[1][0] > bcentres[0][0]){
	tilt = -atan2(bcentres[1][1] - bcentres[0][1], bcentres[1][0] - bcentres[0][0]);
      } else {
	tilt = atan2(bcentres[1][1] - bcentres[0][1], bcentres[0][0] - bcentres[1][0]);
      }
      cout << "0: x: " << bcentres[0][0] << " y: " << bcentres[0][1];
      cout << " 1: x: " << bcentres[1][0] << " y: " << bcentres[1][1];

      sma[sma_ptr] = tilt;
      sma_ptr++;
      if(sma_ptr > (ANG_SMA_PERIOD - 1)) sma_ptr = 0;

      tilt = (sma[0] + sma[1])/ANG_SMA_PERIOD; //change for different sma values
	 
      int length = 150;
	if(out_thrs == 1){
      		line(imgFinal, Point(posX, posY), Point((int) posX + length*cos(tilt), (int)posY - length*sin(tilt)), Scalar(0, 0, 255), 1, 8);
	}

      // if state greater than turning point, add Pi
      heading = heading - tilt - (M_PI/2);
      if(state > 4) heading += M_PI;
      heading = (heading*180.0)/M_PI;
      tilt = (tilt*180.0)/M_PI;

      cout << " tilt: " << tilt;
      cout << " difference: " << heading;
      
      if(out_thrs == 1){
      circle(imgFinal, Point(pt1[0], pt1[1]), 5, Scalar(255, 0, 0), -1, 8); // 1st point
      circle(imgFinal, Point(pt2[0], pt2[1]), 5, Scalar(255, 0, 0), -1, 8); // 0.5st point
      circle(imgFinal, Point(pt3[0], pt3[1]), 5, Scalar(255, 0, 0), -1, 8); // 1.5st point

      circle(imgFinal,Point(200,219), 5, Scalar(255, 0, 255), -1, 8);
      circle(imgFinal, Point(500, 219), 5,Scalar(255, 0, 255), -1, 8);
      
      circle(imgFinal, Point(pt4[0], pt4[1]), 5, Scalar(0,0,255), -1, 8); // 2nd point
      circle(imgFinal, Point(pt5[0], pt5[1]), 5, Scalar(0,0,255), -1, 8); // 3rd Point

      circle(imgFinal, Point(ptStart[0], ptStart[1]), 5, Scalar(0, 0,255), -1, 8);
      circle(imgFinal, Point(ptStop[0], ptStop[1]), 5, Scalar(0,0,255), -1, 8);
      }

      if(out_thrs == 1) imshow("Thresholded Image",imgFinal);
      if(out_orig == 1) imshow("Original", imgOriginal);
      cout << "  State: " << state << "\t";

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
 
 	if( heading > 10 )
   	{
     	//num = write(fd,left,sizeof(left)-1);
     	cout << " Turn Left" << endl;
     	data[3] = 0x04; // Left data type
   	}
 	else if(heading < 10 && heading > -10)
   	{
     	//num = write(fd,right,sizeof(right)-1);
     	cout << " Go Straight" << endl;
     	data[3] = 0x01; //forward data type
   	}
 	else
   	{
     	//num = write(fd, ....
     	cout << " Turn Right" << endl;
     	data[3] = 0x08; // Right data type
  	 }
  	 
  	if( sqrt((point[0] - posX)^2 + (point[1] - posY)^2) < 10){
   		cout << "Stop" << endl;
   		data[3] = 0xFF; // stop data type
		if(nextpt == 3){
		  state++;
		  nextpt = 0;
		}
		else nextpt++;
 	}
  	 data[4] = 0x00;
  	 data[5] = 0x00;
  	 data[6] = 0x00;
  	 data[7] = 0x00;
  	 
  	 data[2] = (unsigned char) ~(data[3] + data[4] + data[5] + data[6] + data[7]);
  	 
  	 data[8] = '\r'; //return byte for BBB handling
	 num = write(fd, data, sizeof(data));
} // end while
    
  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
  	
  return 0;
} // end main
