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


    
    
    
    //Camera frames and related vars    
    VideoCapture capture;
    Mat rawframe,frame;
    Mat gray,prev_gray;
    Mat camera_matrix,distortion_coefficients,optimal_matrix;   


    //Keypoint detector, descriptor and matcher(s)
    GridAdaptedFeatureDetector detector(new FastFeatureDetector(),DESIRED_FTRS,3,3);
    OrbDescriptorExtractor descExtract;
    BFMatcher desc_matcher(NORM_HAMMING,false);
    
    vector<vector<DMatch> >matches_vect1; //Matches frames 1->2
    vector<vector<DMatch> >matches_vect2; //Matches frames 2->1
    vector<DMatch> matches;               //Matches that are symmetric 
    vector<unsigned char> match_mask;    
    vector<KeyPoint> prev_kpts, curr_kpts;   
    vector<Point2f>  kpt_prev_pt, kpt_curr_pt;
    Mat prev_desc, curr_desc;

    //Subpixel refinement controls
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);
      
    //Points for optical flow
    vector<Point2f> detected_pts;
    vector<Point2f> prev_pts,predict_pts,back_predict;    
    vector<Point2f> curr_pts;


    //Setup Camera
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
    //read camera Properties
    FileStorage cam_storage("logi2_9000.yml",FileStorage::READ);
    
    FileNode fn = cam_storage["camera_matrix"];
    camera_matrix = Mat((CvMat*)fn.readObj(),true);
 
    fn = cam_storage["distortion_coefficients"];
    distortion_coefficients = Mat((CvMat*)fn.readObj(),true);
    optimal_matrix=getOptimalNewCameraMatrix(camera_matrix,distortion_coefficients,Size(640,480),0);
    
    bool ref_live = true;
    for (;;)
    {
        capture >> rawframe;
        if (rawframe.empty())
            break;
        undistort(rawframe,frame,camera_matrix,distortion_coefficients,optimal_matrix);
        cvtColor(frame, gray, CV_BGR2GRAY);
        
        detector.detect(gray, curr_kpts); //Find interest points
        if (curr_kpts.size() > 2){
            keypoints2points(curr_kpts, detected_pts);
            cornerSubPix(gray, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
            points2keypoints(detected_pts,curr_kpts);
            descExtract.compute(gray, curr_kpts, curr_desc); //Compute brief descriptors at each keypoint location
            
            if (!prev_kpts.empty())
            {

                            
                desc_matcher.knnMatch(curr_desc, prev_desc, matches_vect1, 2);
                ratioTest(matches_vect1,0.65f);
                desc_matcher.knnMatch(prev_desc, curr_desc, matches_vect2, 2);
                ratioTest(matches_vect2,0.65f);
                symmetryTest(matches_vect1,matches_vect2,matches);
                matches2points(prev_kpts, curr_kpts, matches,kpt_prev_pt,kpt_curr_pt);
                //drawKeypoints(frame, curr_kpts, frame, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG);
                //drawMatchesRelative(prev_kpts, curr_kpts, matches, frame, match_mask);
                
            }
            
            if(!prev_pts.empty()){
                vector<uchar> status,status_backp;
                vector<float> err,err_backp;                        
                calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, predict_pts, status, err, winSize, 3, termcrit, 0, 0.001);
                calcOpticalFlowPyrLK(gray, prev_gray,predict_pts, back_predict, status_backp, err_backp, winSize, 3, termcrit, 0, 0.001);
                curr_pts.clear();
                for (size_t pt_idx=0; pt_idx < back_predict.size() ; pt_idx++){
                    if( norm(back_predict[pt_idx] - prev_pts[pt_idx]) < 1 ){
                        curr_pts.push_back(predict_pts[pt_idx]);
                    }
                }
                //detected_pts.clear();
                for (vector<Point2f>::iterator new_pt=kpt_curr_pt.begin(); new_pt != kpt_curr_pt.end(); new_pt++ ){
                    int add_point=1;
                    for (vector<Point2f>::iterator cpt=curr_pts.begin() ; cpt != curr_pts.end() ; ++cpt){
                        if( norm( *new_pt - *cpt ) < 15 ){
                            add_point=0;
                            break;
                        }
                    }
                    if(add_point==1){
                        curr_pts.push_back(*new_pt);
                    }
                }
               // for (vector<Point2f>::iterator pt=detected_pts.begin(); pt != detected_pts.end(); ++pt){
               //     curr_pts.push_back(*pt);
               // }
                //Draw Points!
                for(size_t idx=0; idx<curr_pts.size(); idx++){
                    cv::circle(frame, curr_pts[idx], 3, Scalar(255, 0, 225), 2);                         
                }
            }else{
                curr_pts=detected_pts;
            }
        }
        imshow("frame", frame);
        //imshow("gray", gray);
        
        if (ref_live)
        {
            prev_kpts = curr_kpts;
            prev_pts=curr_pts;
            gray.copyTo(prev_gray);
            curr_desc.copyTo(prev_desc);
        
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
