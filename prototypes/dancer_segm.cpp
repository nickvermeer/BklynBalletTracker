#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "TuioSVR.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

static void help()
{
 printf("\nDo background segmentation, especially demonstrating the use of cvUpdateBGStatModel().\n"
"Learns the background at the start and then segments.\n"
"Learning is togged by the space key. Will read from file or camera\n"
"Usage: \n"
"			./dancer_segm (-c <camera number>|-fn <filename> \n\n");
}

const char* keys =
{
    "{c  camera   |         | use camera or not}"
    "{fn file_name|tree.avi | movie file        }"
};

//this is a sample for foreground detection functions


int main(int argc, const char** argv)
{
    //Tuio Parameters
    TuioServer *tuioServer;
    TuioTime currentTime;
    vector<TuioCursor*> cursors;
    help();
    
    //CommandLineParser parser(argc, argv, keys);
    bool useCamera=true;
    int camera=0;
    string file;
    if(argc == 3){
        if (strcmp(argv[1],"-c") == 0){
            camera=atoi(argv[2]);
        }else if (strcmp(argv[1],"-fn") == 0){
            useCamera=false;
            file=argv[2];
        }
    }
    VideoCapture cap;
    bool update_bg_model = true;
    int frames=0;
    
    if( useCamera )
        cap.open(camera);
    else
        cap.open(file.c_str());

    if( !cap.isOpened() )
    {
        printf("can not open camera or video file\n");
        return -1;
    }

    namedWindow("image", CV_WINDOW_KEEPRATIO);
    namedWindow("foreground mask", CV_WINDOW_KEEPRATIO);
    namedWindow("foreground image", CV_WINDOW_KEEPRATIO);
    namedWindow("outlines", CV_WINDOW_KEEPRATIO);

    BackgroundSubtractorMOG bg_model(100,25,0.5,10);
    //bg_model.set("noiseSigma", 10);
    
/*    SimpleBlobDetector::Params blobParams;
    blobParams.minDistBetweenBlobs = 10.0f;
    blobParams.filterByInertia = false;
    blobParams.filterByConvexity = false;
    blobParams.filterByColor = false;
    blobParams.filterByCircularity = false;
    blobParams.filterByArea = true;
    blobParams.minArea = 200.0f;
    blobParams.maxArea = 10000.0f;
    
    SimpleBlobDetector blobs(blobParams);
    vector<KeyPoint> blobKeypoints;
  */  
    vector<vector<Point> > contours,contoursAll;
    vector<Moments> contourMoments;    
    vector<Point2d> centers;
    

    Mat img, fgmask, fgmask_old, fgimg, temp, outline_img;
    tuioServer = new TuioServer("rb-mbp.local",3333);    
    //tuioServer = new TuioServer();    
    int niters=1;
    for(;;)
    {
        cap >> img;
        contours.clear();
        contoursAll.clear();
        contourMoments.clear();
        centers.clear();
        
        if( img.empty() )
            break;

        //cvtColor(_img, img, COLOR_BGR2GRAY);

        if( fgimg.empty() )
          fgimg.create(img.size(), img.type());

        if( outline_img.empty() )
          outline_img.create(img.size(), img.type());

        //update the model
        bg_model(img, fgmask, update_bg_model ? -1 : 0);


        //Fill in gaps in the mask using morphology functions        
        dilate(fgmask, temp, Mat(), Point(-1,-1), niters); 
        erode(temp, temp, Mat(), Point(-1,-1), niters*2);
        dilate(temp, temp, Mat(), Point(-1,-1), niters);
        threshold(temp,temp,200,255,CV_THRESH_BINARY);     
        temp.copyTo(fgmask);
        
        //detect movement.  Update bgimage when we have movement or every n frames
        /*
        if (fgmask_old.empty()){
            fgmask_old.create(fgmask.size(),fgmask.type());
        }
        if (sum(fgmask-fgmask_old)[0] > 1e6 || frames > 5){
            update_bg_model = true;
            frames = 0;
        }else {
            update_bg_model = false;
            frames++;
        }
        fgmask.copyTo(fgmask_old);
        */
    //    blobs.detect(fgmask,blobKeypoints);
    //    drawKeypoints(img,blobKeypoints,img);        
        /*
        **Find blobs using Contours
        **
        */
        fgimg = Scalar::all(0);        
        outline_img = Scalar::all(0); 
        findContours(temp,contoursAll,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);                
        
        for (int contour=0;contour < contoursAll.size(); contour++){
            Moments m = moments(contoursAll[contour]);
            if (m.m00 < 100)
                continue;
            contours.push_back(contoursAll[contour]);
            centers.push_back(Point2d(m.m10 / m.m00, m.m01 / m.m00));
            contourMoments.push_back(m);
        }
        drawContours(outline_img,contours,-1,Scalar(255,255,255));
        currentTime = TuioTime::getSessionTime();
        tuioServer->initFrame(currentTime);
        
        list<TuioCursor*> cursorList = tuioServer->getTuioCursors();
        for(list<TuioCursor *>::iterator cursor=cursorList.begin();cursor != cursorList.end();++cursor){
            tuioServer->removeTuioCursor(*cursor);
        }
            tuioServer->stopUntouchedMovingCursors();
            tuioServer->commitFrame();

        
        currentTime = TuioTime::getSessionTime();
        tuioServer->initFrame(currentTime);        
        for (int contour=0;contour < contours.size(); contour++){
            circle(outline_img,centers[contour],4,Scalar(0,0,255),-1);
            cursors.push_back(tuioServer->addTuioCursor((float)(centers[contour].x)/800.0,(float)(centers[contour]).y/600.0));
        }
        tuioServer->commitFrame();        
        
        
        img.copyTo(fgimg, fgmask);
        
        //bg_model.getBackgroundImage(bgimg);
        imshow("image", img);
        imshow("foreground mask", fgmask);
        
        imshow("foreground image", fgimg);
        imshow("outlines",outline_img);
        
        //if(!bgimg.empty())
        //  imshow("mean background image", bgimg );
        
        char k = (char)waitKey(30);
        if( k == 27 ) break;
        if( k == ' ' )
        {
            update_bg_model = !update_bg_model;
            if(update_bg_model)
                printf("Background update is on\n");
            else
                printf("Background update is off\n");
        }
    }

    return 0;
}
