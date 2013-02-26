#include <stdio.h>
#include <iostream>
#include "Tracking.hpp"
#include "Keypoints.hpp"

using namespace cv;
using namespace std;

void help(char ** av){
  cout << "Please provide two video devices\n";
}

static void onMouse( int event, int x, int y, int, void* )
{
    if( event != CV_EVENT_LBUTTONDOWN && event != CV_EVENT_RBUTTONDOWN)
        return;
    if(event == CV_EVENT_LBUTTONDOWN)
      cout << "(" << x << "," << y << ")" << ",";
    else
      cout << endl;
}

int process(VideoCapture capture1, VideoCapture capture2)
{
  Mat rawframe1,rawframe2,undistort1,undistort2;
  Mat gray1,gray2;
  Mat camera1_matrix,distortion_coefficients1,optimal_matrix1;
  Mat camera2_matrix,distortion_coefficients2,optimal_matrix2;
  Mat Mask;
  
  bool recording=false;
  bool paused=false;
  VideoWriter * recorder1;
  VideoWriter * recorder2;


  //Keypoint detector, descriptor and matcher(s)
  //GridAdaptedFeatureDetector detector(new SurfFeatureDetector(),DESIRED_FTRS,3,3);
  SurfFeatureDetector detector(2000,4);  
  FREAK descExtract;
  BFMatcher desc_matcher(NORM_HAMMING,true);
  vector<vector<DMatch> >matches_vect1; //Matches frames 1->2
  vector<vector<DMatch> >matches_vect2; //Matches frames 2->1
  vector<DMatch> matches;               //Matches that are symmetric 
  vector<unsigned char> match_mask;    
  vector<KeyPoint> cam1_kpts, cam2_kpts,cam1_new_kpts,cam2_new_kpts;   
  vector<Point2f>  kpt_cam1_pt, kpt_cam2_pt;
  vector<Point2f> detected_pts;
  Mat cam1_desc, cam2_desc;
    
  //Subpixel refinement controls
  TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
  Size subPixWinSize(10,10), winSize(10,10);

  Mat H_out = Mat::eye(3, 3, CV_64FC1);
  Mat H1 = Mat::eye(3, 3, CV_64FC1);
  Mat H2 = Mat::eye(3, 3, CV_64FC1);
    

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
/*
  //read camera Properties
  FileStorage cam1_storage("cam2.yml",FileStorage::READ);
  FileNode fn = cam1_storage["camera_matrix"];
  camera1_matrix = Mat((CvMat*)fn.readObj(),true);
  fn = cam1_storage["distortion_coefficients"];
  distortion_coefficients1 = Mat((CvMat*)fn.readObj(),true);
  optimal_matrix1=getOptimalNewCameraMatrix(camera1_matrix,distortion_coefficients1,Size(800,600),0);
    
  FileStorage cam2_storage("cam1.yml",FileStorage::READ);
  fn = cam2_storage["camera_matrix"];
  camera2_matrix = Mat((CvMat*)fn.readObj(),true);
  fn = cam2_storage["distortion_coefficients"];
  distortion_coefficients2 = Mat((CvMat*)fn.readObj(),true);
  optimal_matrix2=getOptimalNewCameraMatrix(camera2_matrix,distortion_coefficients2,Size(800,600),0);
*/
/*
  H1.at<double>(0,0)=0.97641872;
  H1.at<double>(0,1)=-0.21588537;
  H1.at<double>(0,2)=129.53122059;
  H1.at<double>(1,0)=0.21588537;
  H1.at<double>(1,1)=0.97641872;
  H1.at<double>(1,2)=0.00000000;
  H1.at<double>(2,0)=0.00000000;
  H1.at<double>(2,1)=0.00000000;
  H1.at<double>(2,2)=1.00000000;
  H2.at<double>(0,0)=0.96525232;
  H2.at<double>(0,1)=0.26131965;
  H2.at<double>(0,2)=0.00000000;
  H2.at<double>(1,0)=-0.26131965;
  H2.at<double>(1,1)=0.96525232;
  H2.at<double>(1,2)=209.05571760;
  H2.at<double>(2,0)=0.00000000;
  H2.at<double>(2,1)=0.00000000;
  H2.at<double>(2,2)=1.00000000;
 */
  H1.at<double>(0,0)=0.90852748;
  H1.at<double>(0,1)=-0.53276288;
  H1.at<double>(0,2)=662.24814528;
  H1.at<double>(1,0)=0.40674024;
  H1.at<double>(1,1)=0.82816794;
  H1.at<double>(1,2)=-33.72253292;
  H1.at<double>(2,0)=0.00000300;
  H1.at<double>(2,1)=-0.00030510;
  H1.at<double>(2,2)=1.00891098;
  H2.at<double>(0,0)=0.96525232;
  H2.at<double>(0,1)=0.26131965;
  H2.at<double>(0,2)=0.00000000;
  H2.at<double>(1,0)=-0.26131965;
  H2.at<double>(1,1)=0.96525232;
  H2.at<double>(1,2)=209.05571760;
  H2.at<double>(2,0)=0.00000000;
  H2.at<double>(2,1)=0.00000000;
  H2.at<double>(2,2)=1.00000000;
   
  Mask.create(Size(2000,900),CV_8UC1);
  Mask=Scalar::all(0);     
  Mask(Rect(0,0,800,600))=Scalar::all(1);
  warpPerspective(Mask,Mask,H1,Size(2000,900),INTER_LINEAR);  
  for (;;)
  {
    if (paused != true){
      capture1 >> rawframe1;
      capture2 >> rawframe2;
      if (rawframe1.empty()||rawframe2.empty())
        break;      
      rawframe1.copyTo(frame1);
      rawframe2.copyTo(frame2);
      warpPerspective(frame1,frame1,H1,Size(2000,900),INTER_LINEAR);  
      warpPerspective(frame2,frame2,H2,Size(2000,900),INTER_LINEAR);  
      frame1.copyTo(frame2,Mask);
    }        
    if (recording){
      //recorder1->write(frame1);
      recorder2->write(frame2);
    }
    imshow(window_name1, frame1);
    imshow(window_name2, frame2);
    setMouseCallback( window_name2, onMouse, 0 );
    setMouseCallback( window_name1, onMouse, 0 );
    
    char key = (char) waitKey(5); //delay N millis, usually long enough to display and capture input
    switch (key)
    {
      case 'q':
      case 'Q':
      case 27: //escape key
        return 0;
      case 'p':
        paused ^= 1;
        break;
      case ' ': //Save an image
        sprintf(filename1, "cam1_%.3d.jpg", n);
        imwrite(filename1, frame1);
        sprintf(filename2, "cam2_%.3d.jpg", n++);
        imwrite(filename2, frame2);
        cout << "Saved " << filename1 << " and " << filename2 << endl;
        break;
      case 'v':
        if (!recording){
          sprintf(filename1, "cam1_%.3d.avi", n);
//          recorder1 = new VideoWriter(filename1,CV_FOURCC('X','V','I','D'),15.0,Size(800,600));
          sprintf(filename2, "cam2_%.3d.avi", n++);
          recorder2 = new VideoWriter(filename2,CV_FOURCC('X','V','I','D'),15.0,Size(2000,900));
          cout << "Capturing to " << filename1 << " and " << filename2 << endl;
          recording=true;
        }else{
          delete recorder1;
          delete recorder2;
          cout << "Finished recording" << endl;
          recording=false;
        }
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