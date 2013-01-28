#include <stdio.h>
#include <iostream>
#include "Tracking.hpp"
#include "Keypoints.hpp"

using namespace cv;
using namespace std;

void help(char ** av){
  cout << "Please provide two video devices\n";
}

int process(VideoCapture capture1, VideoCapture capture2)
{
  Mat rawframe1,rawframe2,undistort1,undistort2;
  Mat gray1,gray2;
  Mat camera1_matrix,distortion_coefficients1,optimal_matrix1;
  Mat camera2_matrix,distortion_coefficients2,optimal_matrix2;
     


  //Keypoint detector, descriptor and matcher(s)
  GridAdaptedFeatureDetector detector(new OrbFeatureDetector(),DESIRED_FTRS,3,3);
  OrbDescriptorExtractor descExtract;
  BFMatcher desc_matcher(NORM_HAMMING,false);
    
  vector<vector<DMatch> >matches_vect1; //Matches frames 1->2
  vector<vector<DMatch> >matches_vect2; //Matches frames 2->1
  vector<DMatch> matches;               //Matches that are symmetric 
  vector<unsigned char> match_mask;    
  vector<KeyPoint> cam1_kpts, cam2_kpts;   
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

  //read camera Properties
  FileStorage cam1_storage("cam1.yml",FileStorage::READ);
  FileNode fn = cam1_storage["camera_matrix"];
  camera1_matrix = Mat((CvMat*)fn.readObj(),true);
  fn = cam1_storage["distortion_coefficients"];
  distortion_coefficients1 = Mat((CvMat*)fn.readObj(),true);
  optimal_matrix1=getOptimalNewCameraMatrix(camera1_matrix,distortion_coefficients1,Size(800,600),0);
    
  FileStorage cam2_storage("cam2.yml",FileStorage::READ);
  fn = cam2_storage["camera_matrix"];
  camera2_matrix = Mat((CvMat*)fn.readObj(),true);
  fn = cam2_storage["distortion_coefficients"];
  distortion_coefficients2 = Mat((CvMat*)fn.readObj(),true);
  optimal_matrix2=getOptimalNewCameraMatrix(camera2_matrix,distortion_coefficients2,Size(800,600),0);
    

  for (;;)
  {
    capture1 >> rawframe1;
    capture2 >> rawframe2;
    
    if (rawframe1.empty()||rawframe2.empty())
        break;
        
    /*Uncomment the following two lines to undistort the frame and reduce the size by half*/
    undistort(rawframe1,undistort1,camera1_matrix,distortion_coefficients1,optimal_matrix1);
    resize(undistort1,frame1,Size(),0.75,0.75,INTER_AREA);
    undistort(rawframe2,undistort2,camera2_matrix,distortion_coefficients2,optimal_matrix2);
    resize(undistort2,frame2,Size(),0.75,0.75,INTER_AREA);

    /*Uncomment the following line to use the raw captured frame*/
    //rawframe1.copyTo(frame1);
    //rawframe2.copyTo(frame2);

    /*Uncomment this line to ignore distortion, but reduce the size*/
    //resize(rawframe1,frame1,Size(),0.75,0.75,INTER_AREA);
    //resize(rawframe2,frame2,Size(),0.75,0.75,INTER_AREA);

    cvtColor(frame1, gray1, CV_BGR2GRAY);    
    cvtColor(frame2, gray2, CV_BGR2GRAY);    
    detector.detect(gray1,cam1_kpts);
    detector.detect(gray2,cam2_kpts);

    if (cam1_kpts.size() > 2 && cam2_kpts.size() > 2){        
            /*refine keypoint locations cam1 using cornersubpix*/
            
            keypoints2points(cam1_kpts, detected_pts);
            cornerSubPix(gray1, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
            points2keypoints(detected_pts,cam1_kpts);
            descExtract.compute(gray1, cam1_kpts, cam1_desc); //Compute brief descriptors at each keypoint location
            
            /*refine keypoint locations cam2 using cornersubpix*/
            keypoints2points(cam2_kpts, detected_pts);
            cornerSubPix(gray2, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
            points2keypoints(detected_pts,cam2_kpts);
            descExtract.compute(gray2, cam2_kpts, cam2_desc); //Compute brief descriptors at each keypoint location
    
    
            desc_matcher.knnMatch(cam1_desc, cam2_desc, matches_vect1, 2);
            ratioTest(matches_vect1,0.75f);
            desc_matcher.knnMatch(cam2_desc, cam1_desc, matches_vect2, 2);
            ratioTest(matches_vect2,0.75f);
            symmetryTest(matches_vect1,matches_vect2,matches);
            matches2points(cam1_kpts, cam2_kpts, matches,kpt_cam1_pt,kpt_cam2_pt);
            cout << matches.size() << endl;
            if (matches.size() > 5){
              //Mat F = findFundamentalMat(kpt_cam1_pt,kpt_cam2_pt,CV_FM_RANSAC,3,0.99,match_mask);
              //correctMatches(F,kpt_cam1_pt,kpt_cam2_pt,kpt_cam1_pt,kpt_cam2_pt);
              points2keypoints(kpt_cam1_pt,cam1_kpts);
              points2keypoints(kpt_cam2_pt,cam2_kpts);
              //stereoRectifyUncalibrated(kpt_cam1_pt,kpt_cam2_pt,F,Size(1600,900),H1,H2,5);
              Mat H=findHomography(kpt_cam1_pt,kpt_cam2_pt,CV_RANSAC,3,match_mask);
              if (countNonZero(Mat(match_mask)) > 15)
                H_out=H;
              cout << "Inital Matches:" << matches.size() << " Final Matches:" << countNonZero(Mat(match_mask)) << endl;
            }
            warpPerspective(frame2,frame2,H_out,Size(1600,900),INTER_LINEAR|WARP_INVERSE_MAP);  
            drawKeypoints(frame1, cam1_kpts, frame1, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
            drawKeypoints(frame2, cam2_kpts, frame2, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
            
            //drawMatchesRelative(cam1_kpts, cam2_kpts, matches, frame1, match_mask);
    
    }

    

              
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