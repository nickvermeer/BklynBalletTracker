#include "Keypoints.hpp"

void drawMatchesRelative(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
    std::vector<cv::DMatch>& matches, Mat& img, const vector<unsigned char>& mask )
{
    for (int i = 0; i < (int)matches.size(); i++)
    {
        if (mask.empty() || mask[i])
        {
            Point2f pt_new = query[matches[i].queryIdx].pt;
            Point2f pt_old = train[matches[i].trainIdx].pt;
            Point2f dist = pt_new - pt_old;

            cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
            cv::circle(img, pt_new, 2, Scalar(255, 0, 125), 1);

        }
    }
}

void refineH(vector<Point2f>& train, vector<Point2f>& query,const vector<unsigned char>& mask)
{
    vector<Point2f> newtrain,newquery;
    for (int i = 0; i < (int)mask.size(); i++)
    {
        if (mask[i])
        {
            newquery.push_back(query[i]);
            newtrain.push_back(train[i]);
        }
    }
    train=newtrain;
    query=newquery;
}

//Takes a descriptor and turns it into an xy point
void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}

//Takes an xy point and appends that to a keypoint structure
void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(KeyPoint(in[i], 1));
    }
}
void setkeypointpoints(const vector<Point2f>& in, const vector<KeyPoint>& orig, vector<KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(orig[i]);    
        out[i].pt=in[i];
    }
}

//Uses computed homography H to warp original input points to new planar position
void warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out)
{
    vector<Point2f> pts;
    keypoints2points(in, pts);
    vector<Point2f> pts_w(pts.size());
    Mat m_pts_w(pts_w);
    perspectiveTransform(Mat(pts), m_pts_w, H);
    setkeypointpoints(pts_w, in,out);
}
int pt2index(Point2f& pt,vector<KeyPoint>& kpts){
    for (int i=0;i<kpts.size();i++){
        if(kpts[i].pt == pt)
            return i;
    }
    return -1;
}
//Converts matching indices to xy points
void matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
    std::vector<Point2f>& pts_query)
{

    pts_train.clear();
    pts_query.clear();
    pts_train.reserve(matches.size());
    pts_query.reserve(matches.size());

    size_t i = 0;

    for (; i < matches.size(); i++)
    {
        const DMatch & dmatch = matches[i];

        pts_query.push_back(query[dmatch.queryIdx].pt);
        pts_train.push_back(train[dmatch.trainIdx].pt);

    }

}

void resetH(Mat&H)
{
    H = Mat::eye(3, 3, CV_64FC1);
}
// (corresponding entries being cleared, i.e. size will be 0)
int ratioTest(std::vector<std::vector<cv::DMatch> >& matches,float ratio) {
    int removed=0; 
    for (std::vector<std::vector<cv::DMatch> >::iterator matchIterator= matches.begin(); matchIterator!=matches.end(); ++matchIterator) {
        // if 2 NN has been identified 
        if (matchIterator->size() > 1) {

        // check distance ratio
         if((*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio) {
             matchIterator->clear(); 
             removed++;
        }

    } else { // does not have 2 neighbours

                                 matchIterator->clear();
                                 removed++;
    }
        }

    return removed;
}
// Insert symmetrical matches in symMatches vector
void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,const std::vector<std::vector<cv::DMatch> >& matches2,std::vector<cv::DMatch>& symMatches) {
    symMatches.clear();
    // for all matches image 1 -> image 2
    for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator1= matches1.begin();matchIterator1!= matches1.end(); ++matchIterator1) {

            if (matchIterator1->size() < 2) // ignore deleted matches 
                    continue;

        // for all matches image 2 -> image 1
        for (std::vector<std::vector<cv::DMatch> >::const_iterator matchIterator2= matches2.begin();matchIterator2!= matches2.end(); ++matchIterator2) {

            if (matchIterator2->size() < 2) // ignore deleted matches 
                    continue;

            // Match symmetry test
            if ((*matchIterator1)[0].queryIdx == (*matchIterator2)[0].trainIdx  && (*matchIterator2)[0].queryIdx == (*matchIterator1)[0].trainIdx) {

            // add symmetrical match
                symMatches.push_back((*matchIterator1)[0]);
                break; // next match in image 1 -> image 2
            }
        }
    }
}


void knn2unique(vector<vector<DMatch> >& matches_vect,vector<DMatch> & matches){
    matches.clear();
    for(vector<vector<DMatch> >::iterator matchIterator= matches_vect.begin(); matchIterator!=matches_vect.end(); ++matchIterator) {
        if (matchIterator->size() > 1) {
            matches.push_back((*matchIterator)[0]);
        }
    }
}
