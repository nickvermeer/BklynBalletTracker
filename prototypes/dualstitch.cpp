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
    if( event != CV_EVENT_LBUTTONDOWN )
        return;
    cout << "(" << x << "," << y << ")" << ",";
}

int process(VideoCapture capture1, VideoCapture capture2)
{
  Mat rawframe1,rawframe2,undistort1,undistort2;
  Mat gray1,gray2;
  Mat camera1_matrix,distortion_coefficients1,optimal_matrix1;
  Mat camera2_matrix,distortion_coefficients2,optimal_matrix2;
  Mat Mask;


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
/*
  H1.at<double>(0,0)=0.63404614;
  H1.at<double>(0,1)=-0.77329522;
  H1.at<double>(0,2)=463.97713253;
  H1.at<double>(1,0)=0.77329522;
  H1.at<double>(1,1)=0.63404614;
  H1.at<double>(1,2)=0.00000000;
  H1.at<double>(2,0)=0.00000000;
  H1.at<double>(2,1)=0.00000000;
  H1.at<double>(2,2)=1.00000000;
  H2.at<double>(0,0)=0.75602128;
  H2.at<double>(0,1)=0.65454704;
  H2.at<double>(0,2)=0.00000000;
  H2.at<double>(1,0)=-0.65454704;
  H2.at<double>(1,1)=0.75602128;
  H2.at<double>(1,2)=523.63763113;
  H2.at<double>(2,0)=0.00000000;
  H2.at<double>(2,1)=0.00000000;
  H2.at<double>(2,2)=1.00000000;
*/

  H1.at<double>(0,0)=0.31050863;
  H1.at<double>(0,1)=-0.28211566;
  H1.at<double>(0,2)=698.76391944;
  H1.at<double>(1,0)=0.48271557;
  H1.at<double>(1,1)=0.80243315;
  H1.at<double>(1,2)=-79.59671562;
  H1.at<double>(2,0)=-0.00038568;
  H1.at<double>(2,1)=0.00049709;
  H1.at<double>(2,2)=0.70818923;
  H2.at<double>(0,0)=0.75602128;
  H2.at<double>(0,1)=0.65454704;
  H2.at<double>(0,2)=0.00000000;
  H2.at<double>(1,0)=-0.65454704;
  H2.at<double>(1,1)=0.75602128;
  H2.at<double>(1,2)=523.63763113;
  H2.at<double>(2,0)=0.00000000;
  H2.at<double>(2,1)=0.00000000;
  H2.at<double>(2,2)=1.00000000;

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
  //Tranform Matrix
  //Rotation Matrix by 0.22 from the Day1 videos
  H2.at<double>(0,0)=0.9758974493306055;
  H2.at<double>(1,1)=0.9758974493306055;
  H2.at<double>(0,1)=-0.21822962308086932;
  H2.at<double>(1,0)=0.21822962308086932;
  //X Translation
  //H2.at<double>(0,2)=131.0;
  H2.at<double>(0,2)=131.0;
  //Rotation Matrix by -0.271 from the Day1 videos
  H1.at<double>(0,0)=0.9635036830882489;
  H1.at<double>(1,1)=0.9635036830882489;
  H1.at<double>(0,1)=0.26769507405886134;
  H1.at<double>(1,0)=-0.26769507405886134;
  //Y translation
  H1.at<double>(1,2)=249.0;    
*/

  Mask.create(Size(2000,900),CV_8UC1);
  Mask=Scalar::all(0);     
  Mask(Rect(0,0,800,600))=Scalar::all(1);
  warpPerspective(Mask,Mask,H2,Size(2000,900),INTER_LINEAR);  
  for (;;)
  {
    capture1 >> rawframe1;
    capture2 >> rawframe2;
    
    if (rawframe1.empty()||rawframe2.empty())
        break;
        
    /*Uncomment the following two lines to undistort the frame and reduce the size by half*/
    //undistort(rawframe1,undistort1,camera1_matrix,distortion_coefficients1,optimal_matrix1);
    //resize(undistort1,frame1,Size(),1,1,INTER_AREA);
    //undistort(rawframe2,undistort2,camera2_matrix,distortion_coefficients2,optimal_matrix2);
    //resize(undistort2,frame2,Size(),1,1,INTER_AREA);

    /*Uncomment the following line to use the raw captured frame*/
    rawframe1.copyTo(frame1);
    rawframe2.copyTo(frame2);
    //frame1=rawframe1.colRange(400,800);
    //frame2=rawframe2.colRange(0,400);
    
    /*Uncomment this line to ignore distortion, but reduce the size*/
    //resize(rawframe1,frame1,Size(),0.75,0.75,INTER_AREA);
    //resize(rawframe2,frame2,Size(),0.75,0.75,INTER_AREA);

//    cvtColor(frame1, gray1, CV_BGR2GRAY);    
//    cvtColor(frame2, gray2, CV_BGR2GRAY);    
//    detector.detect(gray1,cam1_new_kpts);
//    detector.detect(gray2,cam2_new_kpts);
    
    
    if (false && cam1_new_kpts.size() > 2 && cam2_new_kpts.size() > 2){        
            /*refine keypoint locations cam1 using cornersubpix*/
            
            keypoints2points(cam1_new_kpts, detected_pts);
            cornerSubPix(gray1, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
            points2keypoints(detected_pts,cam1_new_kpts);
            //cam1_kpts.insert(cam1_kpts.end(),cam1_new_kpts.begin(),cam1_new_kpts.end());
            cam1_kpts=cam1_new_kpts;
            descExtract.compute(gray1, cam1_kpts, cam1_desc); //Compute brief descriptors at each keypoint location
            
            /*refine keypoint locations cam2 using cornersubpix*/
            keypoints2points(cam2_new_kpts, detected_pts);
            cornerSubPix(gray2, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
            points2keypoints(detected_pts,cam2_new_kpts);
            //cam2_kpts.insert(cam2_kpts.end(),cam2_new_kpts.begin(),cam2_new_kpts.end());
            cam2_kpts=cam2_new_kpts;
            descExtract.compute(gray2, cam2_kpts, cam2_desc); //Compute brief descriptors at each keypoint location
    
    
            /*desc_matcher.knnMatch(cam1_desc, cam2_desc, matches_vect1, 2);
            ratioTest(matches_vect1,0.75f);
            desc_matcher.knnMatch(cam2_desc, cam1_desc, matches_vect2, 2);
            ratioTest(matches_vect2,0.75f);
            symmetryTest(matches_vect1,matches_vect2,matches);
            */
            desc_matcher.match(cam1_desc, cam2_desc, matches);
            matches2points(cam1_kpts, cam2_kpts, matches,kpt_cam1_pt,kpt_cam2_pt);
          
            //Mat imgMatch;
            //drawMatches(frame1,cam1_kpts , frame2, cam2_kpts, matches, imgMatch);    
            //imshow(window_name1,imgMatch);
            //cout << matches.size() << endl;
            //if (matches.size() > 5){
              //Mat F = findFundamentalMat(kpt_cam1_pt,kpt_cam2_pt,CV_FM_RANSAC,3,0.99,match_mask);
              //correctMatches(F,kpt_cam1_pt,kpt_cam2_pt,kpt_cam1_pt,kpt_cam2_pt);
              //points2keypoints(kpt_cam1_pt,cam1_kpts);
              //points2keypoints(kpt_cam2_pt,cam2_kpts);
              //stereoRectifyUncalibrated(kpt_cam1_pt,kpt_cam2_pt,F,Size(1600,900),H1,H2,5);
              //Mat H=findHomography(kpt_cam1_pt,kpt_cam2_pt,CV_RANSAC,3,match_mask);
              //if (countNonZero(Mat(match_mask)) > 15)
              //H_out=H;
              //cout << "Inital Matches:" << matches.size() << " Final Matches:" << countNonZero(Mat(match_mask)) << endl;
  
              			
            //}



//            drawKeypoints(frame1, cam1_kpts, frame1, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
//            drawKeypoints(frame2, cam2_kpts, frame2, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
            
            //drawMatchesRelative(cam1_kpts, cam2_kpts, matches, frame1, match_mask);
    
    }

    
    warpPerspective(frame1,frame1,H1,Size(2000,900),INTER_LINEAR);  
    warpPerspective(frame2,frame2,H2,Size(2000,900),INTER_LINEAR);  
    frame2.copyTo(frame1,Mask);
          
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