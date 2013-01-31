#ifndef WARPER_H
#define WARPER_H
#include "Tracking.hpp"

class Warper{
public:
  Warper(Size in_size, Size out_size, Mat H1, Mat H2);  
  ~Warper();
  void setH1(Mat H1);
  void setH2(Mat H2);
  void prepareMask();
  Mat Warp(Mat img1,Mat img2);
    
private:
  Mat H1;
  Mat H2;
  Mat Mask;
  Size in_size;
  Size out_size;
};

#endif //WARPER_H
