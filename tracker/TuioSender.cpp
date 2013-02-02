#include "TuioSender.hpp"
//debug
#include <iostream>

void TuioSender::sendPoints(const vector<long int> & labels,const vector<Point2f> &pts){

  if(!curr_labeled_pts.empty()){
    prev_labeled_pts=curr_labeled_pts;
    prev_labels=curr_labels;
  }
  curr_labeled_pts.clear();
  curr_labels.clear();
  for (size_t i=0; i < labels.size(); i++){
    curr_labeled_pts.insert(pair<long int,Point2f>(labels[i],pts[i]));
    curr_labels.push_back(labels[1]);
  }
  sort(curr_labels.begin(),curr_labels.end());
  commitPoints();
}

void TuioSender::sendPoints(const map<long int,Point2f> &pts){
  if(!curr_labeled_pts.empty()){
    prev_labeled_pts=curr_labeled_pts;
    prev_labels=curr_labels;
  }
  curr_labeled_pts.clear();
  curr_labels.clear();
  for(map<long int,Point2f>::const_iterator it = pts.begin(); it != pts.end(); ++it){
    curr_labeled_pts.insert(*it);
    curr_labels.push_back(it->first);
  }
  sort(curr_labels.begin(),curr_labels.end());     
  commitPoints();
}
void TuioSender::commitPoints(){
  vector<long int> new_labels(curr_labels.size());
  vector<long int> deleted_labels(prev_labels.size());
  vector<long int> updated_labels(curr_labels.size());
  vector<long int>::iterator endResults;
  
  endResults=set_difference(prev_labels.begin(),prev_labels.end(),curr_labels.begin(),curr_labels.end(),deleted_labels.begin());
  deleted_labels.resize(endResults-deleted_labels.begin()); 
  
  endResults=set_difference(curr_labels.begin(),curr_labels.end(),prev_labels.begin(),prev_labels.end(),new_labels.begin());
  new_labels.resize(endResults-new_labels.begin());
  
  endResults=set_intersection(curr_labels.begin(),curr_labels.end(),prev_labels.begin(),prev_labels.end(),updated_labels.begin());
  updated_labels.resize(endResults-updated_labels.begin());
  
  vector<long int>::iterator it;
  long int sentPoints=1;
  currentTime = TuioTime::getSessionTime();
  tuioServer->initFrame(currentTime);
  
  for (it=new_labels.begin();it!=new_labels.end();++it){
    long int s_id;
    TuioCursor* tcur;
    tcur=tuioServer->addTuioCursor((float)curr_labeled_pts[*it].x/tracked_size.width,(float)curr_labeled_pts[*it].y/tracked_size.height);
    s_id=tcur->getSessionID();
    ActiveCursors[*it]=s_id;
    sentPoints++;
    if ((sentPoints % 25)==0){
      tuioServer->commitFrame();
      currentTime = TuioTime::getSessionTime();
      tuioServer->initFrame(currentTime);
    }
  }

  for (it=updated_labels.begin();it!=updated_labels.end();++it){    
    long int s_id=ActiveCursors[*it];
    TuioCursor* tcur=tuioServer->getTuioCursor(s_id);
    tuioServer->updateTuioCursor(tcur,(float)curr_labeled_pts[*it].x/tracked_size.width,(float)curr_labeled_pts[*it].y/tracked_size.height);
    sentPoints++;
    if ((sentPoints % 25)==0){
      tuioServer->commitFrame();
      currentTime = TuioTime::getSessionTime();
      tuioServer->initFrame(currentTime);
    }
  }
  
  for (it=deleted_labels.begin();it!=deleted_labels.end();++it){    
    long int s_id=ActiveCursors[*it];
    TuioCursor* tcur=tuioServer->getTuioCursor(s_id);
    if (tcur != NULL)
      tuioServer->removeTuioCursor(tcur);
    ActiveCursors.erase(*it);
    sentPoints++;

    if ((sentPoints % 25)==0){
      tuioServer->commitFrame();
      currentTime = TuioTime::getSessionTime();
      tuioServer->initFrame(currentTime);
    }
  }
  tuioServer->stopUntouchedMovingCursors();
  tuioServer->commitFrame();
}
