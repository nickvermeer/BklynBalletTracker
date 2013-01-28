#include <stdio.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void help(char ** av){
  cout << "Please provide two video devices\n";
}

int process(VideoCapture capture1, VideoCapture capture2)
{
  int n=0;
  char filename1[200];
  char filename2[200];
  string window_name1 = "Cam1";
  string window_name2 = "Cam2";
  cout << "press space to save a picture. q or esc to quit" << endl;
  namedWindow(window_name1, CV_WINDOW_KEEPRATIO); //resizable window;
  namedWindow(window_name2, CV_WINDOW_KEEPRATIO); //resizable window;
  Mat frame1;
  Mat frame2;
  capture1.set(CV_CAP_PROP_FRAME_WIDTH,800);
  capture1.set(CV_CAP_PROP_FRAME_HEIGHT,600);
  capture2.set(CV_CAP_PROP_FRAME_WIDTH,800);
  capture2.set(CV_CAP_PROP_FRAME_HEIGHT,600);         
  for (;;)
  {
    capture1 >> frame1;
    capture2 >> frame2;
    
    if (frame1.empty()||frame2.empty())
        break;
    imshow(window_name1, frame1);
    imshow(window_name2, frame2);
    char key = (char) waitKey(5); //delay N millis, usually long enough to display and capture input
    switch (key)
    {
      case 'q':
      case 'Q':
      case 27: //escape key
        return 0;
      case ' ': //Save an image
        sprintf(filename1, "cam1_%.3d.jpg", n);
        imwrite(filename1, frame1);
        sprintf(filename2, "cam2_%.3d.jpg", n++);
        imwrite(filename2, frame2);
        cout << "Saved " << filename1 << " and " << filename2 << endl;
        break;
      default:
          break;
    }
  }
  return 0;
}


int main(int ac, char ** av)
{

  VideoCapture capture1;
  VideoCapture capture2;
  
  
  if (ac != 3)
    {
        help(av);
        return 1;
    }
    std::string arg = av[1];
    capture1.open(arg); //try to open string, this will attempt to open
    if (!capture1.isOpened()) //if this fails, try to open as a video camera
        capture1.open(atoi(arg.c_str()));
    if (!capture1.isOpened())
    {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(av);
        return 1;
    }
    arg = av[2];
    capture2.open(arg); //try to open string, this will attempt to open
    if (!capture2.isOpened()) //if this fails, try to open as a video camera
        capture2.open(atoi(arg.c_str()));
    if (!capture2.isOpened())
    {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(av);
        return 1;
    }
    return process(capture1,capture2);
}