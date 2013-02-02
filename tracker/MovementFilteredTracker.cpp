#include "MovementFilteredTracker.hpp"

void MovementFilteredTracker::loadNewFrame(const Mat& img){
  tracker->loadNewFrame(img);
  if(!curr_labeled_pts.empty()){
    prev_labeled_pts=curr_labeled_pts;
  }
  tracker->getTrackedPoints(&tkr_labeled_pts);
  
  curr_labeled_pts.clear();
  
  for (map<long int, Point2f>::iterator tracked=tkr_labeled_pts.begin();tracked!=tkr_labeled_pts.end();++tracked){
    const long int label=tracked->first;
    const Point2f pt=tracked->second;
    
    if(inital_pos.count(label) == 0){
      inital_pos[label]=pt;      
      active[label]=false;
    } 
    
    else if (active[label] != false) {
      if(norm(prev_labeled_pts[label]-pt)<staleMoveThreshold){
        staleCount[label]=staleCount[label]+1;
       
        if(staleCount[label] > staleFrameThreshold){
          inital_pos[label]=pt;
          active[label]=false;
          staleCount.erase(label);
        }else{
          curr_labeled_pts.insert(*tracked);
        }
      } 
      else {
        curr_labeled_pts.insert(*tracked);
      }
    } 
    
    else if (norm(inital_pos[label]-pt) > initalThreshold){
      active[label]=true;
      staleCount[label]=0;
      curr_labeled_pts.insert(*tracked); 
    }
  }
}

void MovementFilteredTracker::getTrackedPoints(vector<long int> * labels,vector<Point2f> *pts){
  labels->clear();
  pts->clear();

  for (map<long int, Point2f>::iterator tracked=curr_labeled_pts.begin();tracked!=curr_labeled_pts.end();++tracked){
    labels->push_back(tracked->first);
    pts->push_back(tracked->second);
  }
}

void MovementFilteredTracker::getTrackedPoints(map<long int,Point2f> *pts){
  pts->clear();
  for (map<long int, Point2f>::iterator tracked=curr_labeled_pts.begin();tracked!=curr_labeled_pts.end();++tracked){
    pts->insert(*tracked);
  }
}

void MovementFilteredTracker::drawTracked(Mat *img){

  for (map<long int, Point2f>::iterator tracked=curr_labeled_pts.begin();tracked!=curr_labeled_pts.end();++tracked){
    const long int label=tracked->first;
    const Point2f pt=tracked->second;
    circle(*img, pt, 3, Scalar(((label/2)%255), (label%255), ((label/3)%255)), -1);
  }
}

