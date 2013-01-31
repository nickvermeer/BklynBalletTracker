#ifndef TRACKER_H
#define TRACKER_H
#include <opencv2/core/core.hpp>
#include <vector>

using namespace cv;
using namespace std;

class TrackerInterface{
public:
  TrackerInterface(){
    nextLabel=0;
  }  
  virtual ~TrackerInterface(){
  }
  virtual void loadNewFrame(const Mat& img) = 0 ;
  virtual void getTrackedPoints(vector<long int> * labels,vector<Point2f> *pts) = 0;
protected:
  long int nextLabel;
};

#endif //TRACKER_H
