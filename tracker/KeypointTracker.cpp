#include "KeypointTracker.hpp"

void KeypointTracker::loadNewFrame(const Mat& img){
  if(!curr_kpts.empty()){
    prev_kpts=curr_kpts;
    curr_desc.copyTo(prev_desc);
  }
  findKeypoints(img);
  if (!prev_kpts.empty() && !curr_kpts.empty()){
    desc_matcher->match(curr_desc, prev_desc, matches);
    findPoints();
  }
  if(!curr_pts.empty()){
    prev_pts=curr_pts;
    prev_labels=curr_labels;
    trackOpticalFlow(img);
  }
  if (!kpt_pts.empty()){
    addTrackedPoints(kpt_pts);
  }
  img.copyTo(prev_img);
}

void KeypointTracker::getTrackedPoints(vector<long int> *labels,vector<Point2f> *pts){
  labels->clear();
  pts->clear();
  for (vector<Point2f>::iterator cpt=curr_pts.begin() ; cpt != curr_pts.end() ; ++cpt){
    pts->push_back(*cpt);
  }
  for (vector<long int>::iterator label=curr_labels.begin() ; label != curr_labels.end() ; ++label){
    labels->push_back(*label);
  }
}

void KeypointTracker::getTrackedPoints(map<long int,Point2f> *pts){
  pts->clear();
  for (size_t i=0; i < curr_labels.size(); i++){
    pts->insert(pair<long int,Point2f>(curr_labels[i],curr_pts[i]));
  }
}

void KeypointTracker::drawTracked(Mat *img){
  //if(!curr_kpts.empty()){
  //  drawKeypoints(*img, curr_kpts, *img, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
  //}
  for(size_t idx=0; idx<curr_pts.size(); idx++){
    circle(*img, curr_pts[idx], 3, Scalar(((curr_labels[idx]/2)%255), (curr_labels[idx]%255), ((curr_labels[idx]/3)%255)), -1);
  }
}

void KeypointTracker::findKeypoints(const Mat &img){
  vector<Point2f> detected_pts;
  detector->detect(img, curr_kpts);
//  The below only works if the img is 8bit single channel.
//  keypoints2points(curr_kpts, &detected_pts);
//  cornerSubPix(img, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
//  points2keypoints(detected_pts,&curr_kpts);
  descExtract->compute(img, curr_kpts, curr_desc);
}

void KeypointTracker::findPoints(){
  kpt_pts.clear();
  kpt_pts.reserve(matches.size());
  for (size_t i = 0; i < matches.size(); i++)
  {
    const DMatch & dmatch = matches[i];
    kpt_pts.push_back(curr_kpts[dmatch.queryIdx].pt);
  }
                                  
}

void KeypointTracker::trackOpticalFlow(const Mat &img){
  vector<uchar> status,status_backp;
  vector<float> err,err_backp;
  vector<Point2f> predict_pts,back_predict;
  vector<Mat> img_pyr;
  buildOpticalFlowPyramid(img,img_pyr,winSize,5,true);
  if(!prev_pyr.empty()){
    calcOpticalFlowPyrLK(prev_pyr, img_pyr, prev_pts, predict_pts, status, err, winSize, 5, termcrit, 0, 0.001);
    calcOpticalFlowPyrLK(img_pyr, prev_pyr,predict_pts, back_predict, status_backp, err_backp, winSize, 5, termcrit, 0, 0.001);  


    curr_pts.clear();
    curr_labels.clear();
  
    for (size_t pt_idx=0; pt_idx < back_predict.size() ; pt_idx++){
      if( norm(back_predict[pt_idx] - prev_pts[pt_idx]) < .5 ){
        if(status[pt_idx] == 1){
          curr_pts.push_back(predict_pts[pt_idx]);
          curr_labels.push_back(prev_labels[pt_idx]);
        }
      }
    }
  }
  prev_pyr=img_pyr;
}

void KeypointTracker::addTrackedPoints(const vector<Point2f> &pts){
  for (vector<Point2f>::const_iterator new_pt=pts.begin(); new_pt != pts.end(); new_pt++ ){
    int add_point=1;
    for (vector<Point2f>::iterator cpt=curr_pts.begin() ; cpt != curr_pts.end() ; ++cpt){
      Point2f diff = *new_pt - *cpt;
      if( ((double)(diff.x*diff.x)+(double)(diff.y*diff.y)) < 100.0 ){
        add_point=0;
        break;
      }
    }
    if(add_point==1){
      curr_pts.push_back(*new_pt);
      curr_labels.push_back(nextLabel);
      nextLabel++;                        
    }
  }  
}

void KeypointTracker::keypoints2points(const vector<KeyPoint>& in, vector<Point2f>* out)
{
    out->clear();
    out->reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out->push_back(in[i].pt);
    }
}

void KeypointTracker::points2keypoints(const vector<Point2f>& in, vector<KeyPoint>* out)
{
    out->clear();
    out->reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out->push_back(KeyPoint(in[i], 1));
    }
}
