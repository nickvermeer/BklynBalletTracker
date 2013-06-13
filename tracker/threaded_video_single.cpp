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
#include "TuioSender.hpp"
#include "WarpPts.hpp"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <pthread.h>

using namespace cv;
using namespace std;



//hide the local functions in an anon namespace

namespace {
    struct imgtrack_data{
        Mat *frame;
        MovementFilteredTracker *tracker;
    };
    void *imgtrack(void *t_args){
        Mat *my_frame;
        MovementFilteredTracker *my_tracker;
        imgtrack_data *my_data;
        my_data= (imgtrack_data *)t_args;
        my_frame=my_data->frame;
        my_tracker=my_data->tracker;
        my_tracker -> loadNewFrame(*my_frame);
        pthread_exit(NULL);
    }
    struct camera_t_args{
        VideoCapture *vc1;
        Mat *frame1;
        int *frame_num;
        pthread_mutex_t *output_lock;
    };
    
    void *camera_thread(void *c_args){
        camera_t_args *cam_args;
        Mat my_frame1;
        VideoCapture *my_vc1;
        int *out_fn;
        int frame_num=0;
        
        cam_args=(camera_t_args*) c_args;
        my_vc1 = cam_args->vc1;
        out_fn = cam_args->frame_num;
        int old_state=0;
        int old_type=0;
                
        //pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,&old_state);
        //pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,&old_type);
        
        my_vc1->grab();
        sleep(1);
        my_vc1->grab();
        
        while(true){
            my_vc1->grab();
            my_vc1->retrieve(my_frame1);
            if(my_frame1.empty()){
                break;
            }
            frame_num++;
            
            if(pthread_mutex_trylock(cam_args->output_lock) == 0){
                my_frame1.copyTo(*(cam_args->frame1));
                *out_fn = frame_num;
                pthread_mutex_unlock(cam_args->output_lock);
            }
        }
        pthread_exit(NULL);
        return NULL;
    }
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

    int process(VideoCapture& capture1, char * host, int port){//,VideoCapture& capture2) {
        int n = 0;
        char filename[200];
        string window_name_1 = "cam1";
        
        cout << "press space to save a picture. q or esc to quit" << endl;
        namedWindow(window_name_1, WINDOW_KEEPRATIO); //resizable window;
        Mat frame1;
        Mat t_frame1;
        Mat gray1;
        Mat frame;
        Mat H1 = Mat::eye(3, 3, CV_64FC1);
        TuioSender output_1(host,port);

//        TuioSender output;
//        output.setSize(Size(800,600));
        map<long int,Point2f> pts_1,pts_2,warped_pts_1,warped_pts_2;
/*
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
*/
        WarpPts warped_pts_t1(Size(800,600),H1);
        Size out_size=warped_pts_t1.getOutputSize();
        output_1.setSize(out_size);
        output_1.invert_x=true;
                
        MovementFilteredTracker kpt_t1;
        imgtrack_data trk_data1;
        trk_data1.tracker=&kpt_t1;
                
        capture1.set(CAP_PROP_FRAME_WIDTH,800);
        capture1.set(CAP_PROP_FRAME_HEIGHT,600);
        
        int cam_frame_num=0,local_frame_num=0;        
        pthread_t cam_thread;
        pthread_mutex_t cam_lock = PTHREAD_MUTEX_INITIALIZER;


        camera_t_args cam;
        cam.vc1=&capture1;
        cam.frame1=&t_frame1;
        cam.frame_num=&cam_frame_num;
        cam.output_lock=&cam_lock;
        kpt_t1.setThresholds(10,15,300);        
        pthread_create(&cam_thread,NULL,camera_thread,(void *)&cam);
        
        pthread_t threads[2];
        void *status;
        int tmp=0;                                
        for (;;) {
            pthread_mutex_lock(&cam_lock);
            if(*(cam.frame_num) != local_frame_num){
                t_frame1.copyTo(frame1);
                if(local_frame_num +1 != *(cam.frame_num)){
                    cout << local_frame_num << endl;
                }
                local_frame_num = *(cam.frame_num);
                pthread_mutex_unlock(&cam_lock);
                
                cvtColor(frame1, gray1, COLOR_BGR2GRAY);
//            resize(frame1,frame1,Size(),0.75,0.75);
//            resize(frame2,frame2,Size(),0.75,0.75);
                trk_data1.frame=&gray1;
                int rc[2];
            
                rc[0]=pthread_create(&threads[0], NULL, imgtrack, (void *)&trk_data1);
                pthread_join(threads[0], &status);
                kpt_t1.drawTracked(&frame1);
                kpt_t1.getTrackedPoints(&pts_1);
                warped_pts_t1.transformLabeled(pts_1,&warped_pts_1);
                output_1.sendPoints(warped_pts_1);
                tmp++;
            //frame1.copyTo(frame);
            //if ((tmp % 2) == 0) { 
                imshow(window_name_1, frame1);
            //}
            }else{
                pthread_mutex_unlock(&cam_lock);
            }
            char key = (char)waitKey(1); //delay N millis, usually long enough to display and capture input
            switch (key) {
        case 'q':
        case 'Q':
        case 27: //escape key
            pthread_cancel(cam_thread);
            //pthread_exit(NULL);
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

    if (ac != 4) {
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
    /*arg = av[2];
    VideoCapture capture2(arg); //try to open string, this will attempt to open it as a video file
    if (!capture2.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
        capture2.open(atoi(arg.c_str()));
    if (!capture2.isOpened()) {
        cerr << "Failed to open a video device or video file!\n" << endl;
        help(av);
        return 1;
    }*/
    
    return process(capture1,av[2],atoi(av[3]));//,capture2);
}
