#ifndef KEYPOINTTRACKER_H
#define KEYPOINTTRACKER_H
//OpenCV Headers:
#include <opencv2/video/tracking.hpp>   
#include <opencv2/features2d/features2d.hpp>
#include "Tracker.hpp"
//STL Headers:
#include <vector>
#include <map>
#define DESIRED_FTRS 500

using namespace std;
using namespace cv;

class KeypointTracker: public TrackerInterface{
  public:
    KeypointTracker()
      :TrackerInterface()
    {
      detector = new GridAdaptedFeatureDetector(new FastFeatureDetector(),DESIRED_FTRS,3,3);
      descExtract=new OrbDescriptorExtractor();
      desc_matcher=new BFMatcher(NORM_HAMMING,true);
      termcrit=TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
      subPixWinSize=Size(10,10);
      winSize=Size(10,10);
    }
    
    KeypointTracker(FeatureDetector *fd,DescriptorExtractor *de, DescriptorMatcher *dm)
      :TrackerInterface()
    {
      detector = fd;
      descExtract=de;
      desc_matcher=dm;
      termcrit=TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
      subPixWinSize=Size(10,10);
      winSize=Size(10,10);
    }
    
    virtual ~KeypointTracker(){
      delete detector;
      delete descExtract;
      delete desc_matcher;
    }
  void loadNewFrame(const Mat& img);
  void getTrackedPoints(vector<long int> * labels,vector<Point2f> *pts);
  void getTrackedPoints(map<long int,Point2f> *pts);
  void drawTracked(Mat *img);
  
  private:
  //features2d Keypoint detection and matching
  FeatureDetector *detector;
  DescriptorExtractor *descExtract;
  DescriptorMatcher *desc_matcher;
  vector<KeyPoint> prev_kpts, curr_kpts;
  Mat prev_desc, curr_desc;
  vector<DMatch> matches;
  vector<Point2f> kpt_pts;
  //point refinement
  TermCriteria termcrit;
  Size subPixWinSize,winSize;
  //optical flow
  vector<Point2f> prev_pts,curr_pts;
  vector<long int> curr_labels,prev_labels;
  
  Mat prev_img;
  
  void findKeypoints(const Mat &img);
  void trackOpticalFlow(const Mat &img);
  void findPoints();
  void addTrackedPoints(const vector<Point2f> &pts);
  void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>* out);
  void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>* out);
};

#endif
