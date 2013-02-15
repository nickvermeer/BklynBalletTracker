#ifndef TUIOSENDER_H
#define TUIOSENDER_H
#include "TuioServer.h"
#include "TuioCursor.h"

#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace cv;
using namespace TUIO;

class TuioSender {

public:
  bool invert_x;
  bool invert_y;

  TuioSender(){
    tuioServer = new TuioServer();  
    invert_x=false;
    invert_y=false;
  }
  TuioSender(char *serverAddress,int port){
    tuioServer = new TuioServer(serverAddress,port);
    invert_x=false;
    invert_y=false;
  }
  void setServer(char *serverAddress,int port){
    if(tuioServer!=NULL){
      delete tuioServer;
    }
    tuioServer = new TuioServer(serverAddress,port);
  }  
  void setSize(Size s_size){
    tracked_size=s_size;
  }
  virtual ~TuioSender(){
    delete tuioServer;
    ActiveCursors.clear();
    prev_labeled_pts.clear();
    curr_labeled_pts.clear();  
    curr_labels.clear();
    prev_labels.clear();
  }

  void sendPoints(const vector<long int> & labels,const vector<Point2f> &pts);
  void sendPoints(const map<long int,Point2f> &pts);

  

private:

  TuioServer *tuioServer;
  map<long int,long int> ActiveCursors;
  TuioTime currentTime; 
  map<long int,Point2f> prev_labeled_pts, curr_labeled_pts;  
  vector<long int> curr_labels,prev_labels;
  Size tracked_size;
  void commitPoints();
};

#endif //TUIOSERVER_H
