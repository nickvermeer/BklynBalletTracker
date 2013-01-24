#ifndef KEYPOINTS_H
#define KEYPOINTS_H
#include "Tracking.hpp"

void drawMatchesRelative(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
      vector<cv::DMatch>& matches, Mat& img, const vector<unsigned char>& mask = vector<unsigned char> ());
      
void refineH(vector<Point2f>& train, vector<Point2f>& query,const vector<unsigned char>& mask);

void resetH(Mat&H);

void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out);

void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out);

void setkeypointpoints(const vector<Point2f>& in, const vector<KeyPoint>& orig, vector<KeyPoint>& out);

void warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out);

int pt2index(Point2f& pt,vector<KeyPoint>& kpts);

//Converts matching indices to xy points
void matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
  const vector<DMatch>& matches, vector<Point2f>& pts_train,vector<Point2f>& pts_query);

int ratioTest(std::vector<std::vector<cv::DMatch> >& matches,float ratio);

void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,const std::vector<std::vector<cv::DMatch> >& matches2,std::vector<cv::DMatch>& symMatches);

void knn2unique(vector<vector<DMatch> >& matches_vect,vector<DMatch> & matches);

#endif