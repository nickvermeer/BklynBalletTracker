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
  TuioSender(){
    tuioServer = new TuioServer();  
  }
  TuioSender(char *serverAddress,int port){
    tuioServer = new TuioServer(serverAddress,port);
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
