#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    VideoCapture cap(0); // open the video camera no. 0

    if (!cap.isOpened())  // if not success, exit program
    {
        cout << "Cannot open the video cam" << endl;
        return -1;
    }

    //double fps = cap.get(CV_CAP_PROP_FPS); // get fps of video
    //cout << "Frames per Second: " << fps << endl;

    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);

    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);//create a window called "MyVideo"

    while (1)
    {
        Mat frame;

        bool bSuccess = cap.read(frame); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

        imshow("MyVideo", frame); //show the frame in "MyVideo" window

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
	    //cout << "esc key is pressed by user" << endl;
            //break;
       }
    }
    return 0;

}
