/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;


/*
cv::Mat System::TrackMonoCamera(const double & timestamp) {
//#include "opencv2/opencv.hpp"
//using namespace cv;

    cv::VideoCapture cap;
    cv::Mat frame;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
    {
        cerr << "Cannot open Mono Camera." << endl;
        exit(-1);
    }
    for(;;)
    {

          cap >> frame;
          if( frame.empty() ) break; // end of video stream
          //imshow("this is you, smile! :)", frame);
          //if( waitKey(1) == 27 ) break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return TrackMonocular(frame,timestamp);
}
*/

using namespace std::chrono;


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./mono_cam path_to_vocabulary path_to_settings " << endl;
        return 1;
    }
    cv::VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
    {
        cerr << "Cannot open Mono Camera." << endl;
        exit(-1);
    }

    // Retrieve paths to images
    //vector<string> vstrImageFilenames;
    //vector<double> vTimestamps;
    //LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    //int nImages = vstrImageFilenames.size();
    int nImages = 1000; // random number just to keep the variable alive

    //if(nImages<=0)
    //{
    //    cerr << "ERROR: Failed to load images" << endl;
    //    return 1;
    //}

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);


    // Main loop
    cv::Mat im;
    double timestamp = 1;
    for(;;)
    {
        // Read image from file
        cap >> im;
        //im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        //double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to grab image" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        timestamp++;

        // Pass the image to the SLAM system
        //double timestamp = std::chrono::duration_cast<std::chrono::duration<double> >(t1).count();
        SLAM.TrackMonocular(im,timestamp);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //vTimesTrack[ni]=ttrack;

        /*
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
            */
        cout << "waiting 1" << endl;
        usleep(1);
    }

    // Stop all threads
    SLAM.Shutdown();
    /*
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    */
    return 0;
}


/*
cv::Mat System::TrackMonoCamera(const double & timestamp) {
//#include "opencv2/opencv.hpp"
//using namespace cv;

    cv::VideoCapture cap;
    cv::Mat frame;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
    {
        cerr << "Cannot open Mono Camera." << endl;
        exit(-1);
    }
    for(;;)
    {

          cap >> frame;
          if( frame.empty() ) break; // end of video stream
          //imshow("this is you, smile! :)", frame);
          //if( waitKey(1) == 27 ) break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return TrackMonocular(frame,timestamp);
}
*/
