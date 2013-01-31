#include "Warper.hpp"
//private:
//  Mat H1;
//  Mat H2;
//  Mat Mask;
//  Size in_size;
//  Size out_size;

Warper::Warper(Size in_size, Size out_size, Mat H1, Mat H2){
  this->in_size=in_size;
  this->out_size=out_size;
  setH1(H1);
  setH2(H2);
  prepareMask();
}    
Warper::~Warper(){
  H1.release();
  H2.release();
  Mask.release();
}
void Warper::setH1(Mat H1){
  H1.copyTo(this->H1);
}
void Warper::setH2(Mat H2){
  H2.copyTo(this->H2);
}
void Warper::prepareMask(){
  Mask.create(out_size,CV_8UC1);
  Mask=Scalar::all(0);
  Mask(Rect(0,0,in_size.width,in_size.height))=Scalar::all(1);
  warpPerspective(Mask,Mask,H2,out_size,INTER_LINEAR); 
}
Mat Warper::Warp(Mat img1,Mat img2){
  Mat tmp1;
  Mat tmp2;
  warpPerspective(img1,tmp1,H1,out_size,INTER_LINEAR);
  warpPerspective(img2,tmp2,H2,out_size,INTER_LINEAR);
  tmp2.copyTo(tmp1,Mask);
  return tmp1;
}

  
//private:
//  Mat H1;
//  Mat H2;
//  Mat Mask;
//  Size in_size;
//  Size out_size;


