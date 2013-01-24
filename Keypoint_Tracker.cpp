/*
* video_homography.cpp
*
*  Created on: Oct 18, 2010
*      Author: erublee
*/

#include "Tracking.hpp"
#include "Keypoints.hpp"
#define PI 3.14159265

void help(char **av)
{
    cout << "Please supply Camera #\n";
}


int main(int ac, char ** av)
{

    if (ac != 2)
    {
        help(av);
        return 1;
    }
    OrbDescriptorExtractor descExtract;
    
    VideoCapture capture;
    capture.open(atoi(av[1]));
    //capture.open(av[1]);
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if (!capture.isOpened())
    {
        help(av);
        cout << "capture device " << atoi(av[1]) << " failed to open!" << endl;
        return 1;
    }

    Mat rawframe,frame;

    vector<vector<DMatch> >matches_vect1;
    vector<vector<DMatch> >matches_vect2;
    vector<DMatch> matches;
    BFMatcher desc_matcher(NORM_HAMMING,false);
    


    vector<Point2f> train_pts, query_pts,train_pts2,query_pts2;
    vector<KeyPoint> train_kpts, query_kpts;
    vector<unsigned char> match_mask;
    Mat gray,gray2;
    Mat image_out=Mat::zeros(1024,768,CV_8UC3);
  
    Mat camera_matrix,distortion_coefficients,optimal_matrix;   
  
    bool ref_live = true;
    
    Mat train_desc, query_desc;
    const int DESIRED_FTRS = 800;
    GridAdaptedFeatureDetector detector(new FastFeatureDetector(),DESIRED_FTRS,3,3);
    FileStorage cam_storage("logi2_9000.yml",FileStorage::READ);
    
    FileNode fn = cam_storage["camera_matrix"];
    camera_matrix = Mat((CvMat*)fn.readObj(),true);
 
    fn = cam_storage["distortion_coefficients"];
    distortion_coefficients = Mat((CvMat*)fn.readObj(),true);
    optimal_matrix=getOptimalNewCameraMatrix(camera_matrix,distortion_coefficients,Size(640,480),0);
    for (;;)
    {
        capture >> rawframe;
        if (rawframe.empty())
            break;
        undistort(rawframe,frame,camera_matrix,distortion_coefficients,optimal_matrix);
        gray2=gray;
        cvtColor(frame, gray, CV_BGR2GRAY);
        
        detector.detect(gray, query_kpts); //Find interest points
        descExtract.compute(gray, query_kpts, query_desc); //Compute brief descriptors at each keypoint location
        
        if (!train_kpts.empty())
        {

            desc_matcher.knnMatch(query_desc, train_desc, matches_vect1, 2);
            ratioTest(matches_vect1,0.65f);
            desc_matcher.knnMatch(train_desc, query_desc, matches_vect2, 2);
            ratioTest(matches_vect2,0.65f);
            symmetryTest(matches_vect1,matches_vect2,matches);
            matches2points(train_kpts, query_kpts, matches, train_pts, query_pts);
            //drawKeypoints(frame, train_kpts, frame, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
            drawMatchesRelative(train_kpts, query_kpts, matches, frame, match_mask);

        }

        imshow("frame", frame);

        if (ref_live)
        {
            train_kpts = query_kpts;
            query_desc.copyTo(train_desc);
        }
        char key = (char)waitKey(2);
        switch (key)
        {
        case 'l':
            ref_live = true;
            break;
        case 't':
            ref_live = false;
            break;
        case 27:
        case 'q':
            return 0;
            break;
        }

    }
    return 0;
}
