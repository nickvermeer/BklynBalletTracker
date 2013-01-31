#ifndef MOVEMENTFILTERED_H
#define MOVEMENTFILTERED_H
#include "Tracker.hpp"
#include "KeypointTracker.hpp"

//STL Headers:
#include <vector>
#include <map>
#include <iostream>

using namespace std;
using namespace cv;

class MovementFilteredTracker: public TrackerInterface{
  public:
    MovementFilteredTracker()
      :TrackerInterface()
    {
      tracker = new KeypointTracker();
      initalThreshold=5;
      staleMoveThreshold=5;
      staleFrameThreshold=15;
    }
    MovementFilteredTracker(TrackerInterface *trk)
      :TrackerInterface()
    {
      tracker = trk;
    }
    virtual ~MovementFilteredTracker(){
      delete tracker;
    }
  void setThresholds(int init,int staleMove,int staleFrame){
      initalThreshold=init;
      staleMoveThreshold=staleMove;
      staleFrameThreshold=staleFrame;    
  }
  void loadNewFrame(const Mat& img);
  void getTrackedPoints(vector<long int> * labels,vector<Point2f> *pts);
  void getTrackedPoints(map<long int,Point2f> *pts);
  void drawTracked(Mat *img);
  
  private:
  //features2d Keypoint detection and matching
  TrackerInterface *tracker;
  map<long int,int> staleCount;
  map<long int,Point2f> inital_pos,tkr_labeled_pts,prev_labeled_pts,curr_labeled_pts;
  map<long int,bool> active;
  
  int initalThreshold;
  int staleMoveThreshold;
  int staleFrameThreshold;
};

#endif
