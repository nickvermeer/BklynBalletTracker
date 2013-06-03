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
#include "opencv2/highgui/highgui_c.h"
#include "MovementFilteredTracker.hpp"
#include "TuioSender.hpp"
#include "WarpPts.hpp"
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

    int process(VideoCapture& capture1, VideoCapture& capture2, char * host, int port){//,VideoCapture& capture2) {
        int n = 0;
        char filename[200];
        string window_name_1 = "cam1";
        string window_name_2 = "cam2";
        
        cout << "press space to save a picture. q or esc to quit" << endl;
        namedWindow(window_name_1, CV_WINDOW_KEEPRATIO); //resizable window;
        namedWindow(window_name_2, CV_WINDOW_KEEPRATIO); //resizable window;
        Mat frame1;
        Mat frame2;
        Mat gray1;
        Mat gray2;
        
        Mat frame;
        Mat H1 = Mat::eye(3, 3, CV_64FC1);
        Mat H2 = Mat::eye(3, 3, CV_64FC1);
        TuioSender output_1(host,port);
        TuioSender output_2(host,port);

//        TuioSender output;
//        output.setSize(Size(800,600));
        map<long int,Point2f> pts_1,pts_2,warped_pts_1,warped_pts_2;
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
        WarpPts warped_pts_t1(Size(800,600),H1);
        WarpPts warped_pts_t2(Size(800,600),H2);
        Size out_size=warped_pts_t1.getOutputSize();
        output_1.setSize(out_size);
        output_2.setSize(out_size);
        output_1.invert_x=true;
        output_2.invert_x=true;
                
        MovementFilteredTracker kpt_t1;
        MovementFilteredTracker kpt_t2;
        capture1.set(CV_CAP_PROP_FRAME_WIDTH,800);
        capture1.set(CV_CAP_PROP_FRAME_HEIGHT,600);
        capture2.set(CV_CAP_PROP_FRAME_WIDTH,800);
        capture2.set(CV_CAP_PROP_FRAME_HEIGHT,600);
        int tmp=0;                                
        for (;;) {
            capture1 >> frame1;
            if (frame1.empty())
                break;
            capture2 >> frame2;
            if (frame2.empty())
                break;
//            resize(frame1,frame1,Size(),0.75,0.75);
//            resize(frame2,frame2,Size(),0.75,0.75);

            cvtColor(frame1, gray1, COLOR_BGR2GRAY);
            cvtColor(frame2, gray2, COLOR_BGR2GRAY);
                            
            kpt_t1.loadNewFrame(gray1);
            kpt_t1.drawTracked(&frame1);
            kpt_t1.getTrackedPoints(&pts_1);
            warped_pts_t1.transformLabeled(pts_1,&warped_pts_1);
            output_1.sendPoints(warped_pts_1);
            kpt_t2.loadNewFrame(gray2);
            kpt_t2.drawTracked(&frame2);
            kpt_t2.getTrackedPoints(&pts_2);
            warped_pts_t2.transformLabeled(pts_2,&warped_pts_2);
            output_2.sendPoints(warped_pts_2);
            tmp++;
            //frame1.copyTo(frame);
            //if ((tmp % 2) == 0) { 
                imshow(window_name_1, frame1);
                imshow(window_name_2, frame2);
            //}
            char key = (char)waitKey(1); //delay N millis, usually long enough to display and capture input
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

    if (ac != 5) {
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
    
    return process(capture1,capture2,av[3],atoi(av[4]));//,capture2);
}
