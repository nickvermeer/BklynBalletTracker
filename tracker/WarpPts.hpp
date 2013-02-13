#ifndef WARPPTS_H
#define WARPPTS_H
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;

class WarpPts{
public:
  WarpPts(){
    in_size=Size(800,600);
    H=Mat::eye(3,3,CV_32F);
  }
  WarpPts(const Size &_in_size, const Mat &_H){
    setSize(_in_size);
    setH(_H);
  }  
  ~WarpPts(){
  }
  void setSize(const Size &_in_size){
    in_size=_in_size;
  }
  void setH(const Mat &_H){
    _H.copyTo(H);
  }
  Mat getH(){
    Mat ret_H;
    H.copyTo(ret_H);
    return ret_H;
  }
  
  const Size getOutputSize(){
    Size out_size;
    vector<Point2f>pts;  
    vector<Point2f>out_pts;  
    pts.push_back(Point2f(0,0));
    pts.push_back(Point2f(0,in_size.width));
    pts.push_back(Point2f(in_size.height,in_size.width));
    pts.push_back(Point2f(in_size.height,0));
    perspectiveTransform(pts,out_pts,H);
    for(vector<Point2f>::iterator i = out_pts.begin(); i != out_pts.end(); ++i){
      if(i->x > out_size.width)
        out_size.width=i->x;
      if(i->y > out_size.height)
        out_size.height=i->y;   
    }
    return out_size;
  }
  
  vector<Point2f> transform(const vector<Point2f> &pts){
    vector<Point2f>out_pts;  
    perspectiveTransform(pts,out_pts,H);
    return out_pts;
  }
    
private:
  Mat H;
  Size in_size;
};

#endif //WARPPTS_H
