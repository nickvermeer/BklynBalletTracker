/*
* starter_video.cpp
*
*  Created on: Nov 23, 2010
*      Author: Ethan Rublee
*
* A starter sample for using opencv, get a video stream and display the images
* easy as CV_PI right?
*/
#include "opencv2/highgui/highgui.hpp"
#include "MovementFilteredTracker.hpp"
#include "Warper.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;



//hide the local functions in an anon namespace
namespace {
    void help(char** av) {
        cout << "\nThis program justs gets you started reading images from video\n"
            "Usage:\n./" << av[0] << " <video device number>\n"
            << "q,Q,esc -- quit\n"
            << "space   -- save frame\n\n"
            << "\tThis is a starter sample, to get you up and going in a copy pasta fashion\n"
            << "\tThe program captures frames from a camera connected to your computer.\n"
            << "\tTo find the video device number, try ls /dev/video* \n"
            << "\tYou may also pass a video file, like my_vide.avi instead of a device number"
            << endl;
    }

    int process(VideoCapture& capture1,VideoCapture& capture2) {
        int n = 0;
        char filename[200];
        string window_name = "video | q or esc to quit";
        cout << "press space to save a picture. q or esc to quit" << endl;
        namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
        Mat frame1;
        Mat frame2;
        Mat frame;
        Mat H1 = Mat::eye(3, 3, CV_64FC1);
        Mat H2 = Mat::eye(3, 3, CV_64FC1);

        H1.at<double>(0,0)=0.21760649;
        H1.at<double>(0,1)=-0.42936176;
        H1.at<double>(0,2)=809.98088241;
        H1.at<double>(1,0)=0.59922021;
        H1.at<double>(1,1)=0.70164611;
        H1.at<double>(1,2)=-31.94378487;
        H1.at<double>(2,0)=-0.00037348;
        H1.at<double>(2,1)=0.00046245;
        H1.at<double>(2,2)=0.75107234;
        H2.at<double>(0,0)=0.85272659;
        H2.at<double>(0,1)=0.52235750;
        H2.at<double>(0,2)=0.00000000;
        H2.at<double>(1,0)=-0.52235750;
        H2.at<double>(1,1)=0.85272659;
        H2.at<double>(1,2)=417.88599745;
        H2.at<double>(2,0)=0.00000000;
        H2.at<double>(2,1)=0.00000000;
        H2.at<double>(2,2)=1.00000000;
        Warper CorrectCamera(Size(800,600),Size(2000,1000),H1,H2);
        MovementFilteredTracker kpt_t1;
        MovementFilteredTracker kpt_t2;
        capture1.set(CV_CAP_PROP_FRAME_WIDTH,800);
        capture1.set(CV_CAP_PROP_FRAME_HEIGHT,600);
        capture2.set(CV_CAP_PROP_FRAME_WIDTH,800);
        capture2.set(CV_CAP_PROP_FRAME_HEIGHT,600);
                                        
        for (;;) {
            capture1 >> frame1;
            capture2 >> frame2;
            //frame1.copyTo(frame);
            kpt_t1.loadNewFrame(frame1);
            kpt_t1.drawTracked(&frame1);
            //kpt_t2.loadNewFrame(frame2);
            //kpt_t2.drawTracked(&frame2);
            //CorrectCamera.Warp(frame1,frame2,&frame);
            frame1.copyTo(frame);
            if (frame.empty())
                break;
            imshow(window_name, frame);
            char key = (char)waitKey(5); //delay N millis, usually long enough to display and capture input
            switch (key) {
        case 'q':
        case 'Q':
        case 27: //escape key
            return 0;
        case ' ': //Save an image
            sprintf(filename,"filename%.3d.jpg",n++);
            imwrite(filename,frame);
            cout << "Saved " << filename << endl;
            break;
        default:
            break;
            }
        }
        return 0;
    }

}

int main(int ac, char** av) {

    if (ac != 3) {
        help(av);
        return 1;
    }
    std::string arg = av[1];
    VideoCapture capture1(arg); //try to open string, this will attempt to open it as a video file
    if (!capture1.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
        capture1.open(atoi(arg.c_str()));
    if (!capture1.isOpened()) {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(av);
        return 1;
    }
    arg = av[2];
    VideoCapture capture2(arg); //try to open string, this will attempt to open it as a video file
    if (!capture2.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
        capture2.open(atoi(arg.c_str()));
    if (!capture2.isOpened()) {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(av);
        return 1;
    }
    
    return process(capture1,capture2);
}
