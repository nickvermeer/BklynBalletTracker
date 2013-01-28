/*
* Keypoint_Tracker
*  heavily hacked from video_homography
*  Created on: Oct 18, 2010
*      Author: erublee
*/

#include "Tracking.hpp"
#include "Keypoints.hpp"
#include "TuioSVR.hpp"
#define PI 3.14159265

void help(char **av)
{
    cout << "Please supply Camera # or video filename\n" ;
}


int main(int ac, char ** av)
{

    if (ac != 2)
    {
        help(av);
        return 1;
    }
    //Tuio Parameters
    TuioServer *tuioServer;
    map<long,TuioCursor*> ActiveCursors;
    TuioTime currentTime;
                            
    //Camera frames and related vars    
    VideoCapture capture;
    Mat rawframe,undistort_frame, frame;
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
    Size subPixWinSize(10,10), winSize(10,10);
      
    //Points for optical flow
    vector<Point2f> detected_pts;
    vector<Point2f> prev_pts,predict_pts,back_predict;    
    vector<Point2f> curr_pts;
    
    //Label variables
    vector<long> prev_labels,curr_labels;
    vector<long> new_labels;
    vector<long> old_labels;
    vector<long> active_labels;
    map<long,Point2f> labels_orig;
    map<long,double> labels_maxmove;
    long next_label=0;	

    //Setup Camera
    if(atoi(av[1])+'0'==av[1][0]){
        capture.open(atoi(av[1]));
        capture.set(CV_CAP_PROP_FRAME_WIDTH,800);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT,600);
    }else{
        capture.open(av[1]);
    }
    if (!capture.isOpened())
    {
        help(av);
        cout << "capture device " << atoi(av[1]) << " failed to open!" << endl;
        return 1;
    }
    //read camera Properties
    FileStorage cam_storage("cam1.yml",FileStorage::READ);
    FileNode fn = cam_storage["camera_matrix"];
    camera_matrix = Mat((CvMat*)fn.readObj(),true);
    fn = cam_storage["distortion_coefficients"];
    distortion_coefficients = Mat((CvMat*)fn.readObj(),true);
    optimal_matrix=getOptimalNewCameraMatrix(camera_matrix,distortion_coefficients,Size(800,600),0);
    
    bool ref_live = true;

    tuioServer = new TuioServer();

    for (;;)
    {
                                        
        capture >> rawframe;
        if (rawframe.empty())
            break;
        /*Uncomment the following two lines to undistort the frame and reduce the size by half*/
        undistort(rawframe,undistort_frame,camera_matrix,distortion_coefficients,optimal_matrix);
        resize(undistort_frame,frame,Size(),0.75,0.75,INTER_AREA);
        
        /*Uncomment the following line to use the raw captured frame*/
        //rawframe.copyTo(frame);
        
        /*Uncomment this line to ignore distortion, but reduce the size*/
        //resize(rawframe,frame,Size(),0.75,0.75,INTER_AREA);
        
        cvtColor(frame, gray, CV_BGR2GRAY);
        
        detector.detect(gray, curr_kpts); //Find interest points
        /*If there are enough detected points, process the points for tracking
        */
        if (curr_kpts.size() > 2){        
            /*refine keypoint locations using cornersubpix*/
            keypoints2points(curr_kpts, detected_pts);
            cornerSubPix(gray, detected_pts, subPixWinSize, Size(-1,-1), termcrit);
            points2keypoints(detected_pts,curr_kpts);
            descExtract.compute(gray, curr_kpts, curr_desc); //Compute brief descriptors at each keypoint location
            
            if (!prev_kpts.empty())
            {
                /*track descriptors from the previous frame*/
                            
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
                /*Use optical flow to track points from previous frame.
                **Then use those points, and track back to the previous frame again.
                **Finally, keep only the points that match well in both directions.
                */ 
                vector<uchar> status,status_backp;
                vector<float> err,err_backp;                        
                calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, predict_pts, status, err, winSize, 3, termcrit, 0, 0.001);
                calcOpticalFlowPyrLK(gray, prev_gray,predict_pts, back_predict, status_backp, err_backp, winSize, 3, termcrit, 0, 0.001);
                curr_pts.clear();
                curr_labels.clear();
                new_labels.clear();
                old_labels.clear();
                
                currentTime = TuioTime::getSessionTime();
                tuioServer->initFrame(currentTime);
                for (size_t pt_idx=0; pt_idx < back_predict.size() ; pt_idx++){
                    if( norm(back_predict[pt_idx] - prev_pts[pt_idx]) < .5 ){
                        if(status[pt_idx] == 1){
                            curr_pts.push_back(predict_pts[pt_idx]);
                            curr_labels.push_back(prev_labels[pt_idx]);
                            double distance=norm(predict_pts[pt_idx]-labels_orig[prev_labels[pt_idx]]);
                            if (distance > labels_maxmove[prev_labels[pt_idx]]){
                                if(labels_maxmove[prev_labels[pt_idx]] < 5.0 && distance >= 5.0)
                                    ActiveCursors[prev_labels[pt_idx]]=tuioServer->addTuioCursor((float)predict_pts[pt_idx].x/800.0,(float)predict_pts[pt_idx].y/600.0);    
                                labels_maxmove[prev_labels[pt_idx]]=distance;
                            }
                        }else{
                            old_labels.push_back(prev_labels[pt_idx]);
                        }
                    }else{
                        old_labels.push_back(prev_labels[pt_idx]);

                    }
                }
                tuioServer->commitFrame();
                //Add new points from the keypoint tracker
                for (vector<Point2f>::iterator new_pt=kpt_curr_pt.begin(); new_pt != kpt_curr_pt.end(); new_pt++ ){
                    int add_point=1;
                    for (vector<Point2f>::iterator cpt=curr_pts.begin() ; cpt != curr_pts.end() ; ++cpt){
                        if( norm( *new_pt - *cpt ) < 10 ){
                            add_point=0;
                            break;
                        }
                    }
                    if(add_point==1){
                        curr_pts.push_back(*new_pt);
                        curr_labels.push_back(next_label);
                        new_labels.push_back(next_label);
                        labels_orig[next_label]=*new_pt;
                        labels_maxmove[next_label]=0.0;
                        
                        next_label++;                        
                    }
                }
               //Draw Points
                int points_sent=0;
                currentTime = TuioTime::getSessionTime();
                tuioServer->initFrame(currentTime);
                for(size_t idx=0; idx<curr_pts.size(); idx++){
                    if(labels_maxmove[curr_labels[idx]] >= 5.0)
                        cv::circle(frame, curr_pts[idx], 3, Scalar(((curr_labels[idx]/2)%255), (curr_labels[idx]%255), ((curr_labels[idx]/3)%255)), -1);                         
                        tuioServer->updateTuioCursor(ActiveCursors[curr_labels[idx]],(float)curr_pts[idx].x/800.0,(float)curr_pts[idx].y/600.0);
                        points_sent++;
                        if (points_sent % 48 == 0){
                            tuioServer->commitFrame();
                            currentTime = TuioTime::getSessionTime();
                            tuioServer->initFrame(currentTime);
                        }
                }
                tuioServer->commitFrame();

            }else{
                curr_pts=detected_pts;
                for(vector<Point2f>::iterator pt = curr_pts.begin(); pt != curr_pts.end(); ++pt){
                    curr_labels.push_back(next_label);
                    new_labels.push_back(next_label);
                    labels_orig[next_label]=*pt;
                    labels_maxmove[next_label]=0.0;
                    next_label++;
                }
                
            }
        }
        
                        
        imshow("frame", frame);
        currentTime = TuioTime::getSessionTime();
        tuioServer->initFrame(currentTime);
        for(vector<long>::iterator label = old_labels.begin();  label != old_labels.end(); ++label){
            labels_orig.erase(*label);
            if(labels_maxmove[*label] >= 5.0){
                tuioServer->removeTuioCursor((ActiveCursors.find(*label))->second);
                ActiveCursors.erase(*label);
            }
            labels_maxmove.erase(*label);

        }
        tuioServer->stopUntouchedMovingCursors();
        tuioServer->commitFrame();
                
        if (ref_live)
        {
            prev_kpts = curr_kpts;
            prev_pts=curr_pts;
            prev_labels=curr_labels;
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
