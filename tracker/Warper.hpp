#ifndef WARPER_H
#define WARPER_H
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class Warper{
public:
  Warper(const Size &in_size, const Size &out_size, const Mat &H1, const Mat &H2);  
  ~Warper();
  void setH1(const Mat &H1);
  void setH2(const Mat &H2);
  void prepareMask();
  void Warp(const Mat &img1,const Mat &img2,Mat *imgout);
    
private:
  Mat H1;
  Mat H2;
  Mat Mask;
  Size in_size;
  Size out_size;
};

#endif //WARPER_H
