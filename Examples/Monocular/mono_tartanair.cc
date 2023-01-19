/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequenceFullPath, vector<string> &vstrImages, vector<double> &vTimeStamps, const double dT, const double start_time_stamp);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_tartanair path_to_vocabulary path_to_settings path_to_images_full_path trajectory(proto)_file_name" << endl;
        return 1;
    }

    string file_name = string(argv[argc-1]);
    // Load all sequences:
    vector<string> vstrImageFilenames;
    vector<double> vTimestampsCam;

    int fps = 20;
    double dT = 1.f/fps;
    double start_time_stamp = 1403638538.927829504;

    cout << "Loading images for sequence ";
    //TODO: load images and generate timestamps for the images
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestampsCam, dT, start_time_stamp);
    cout << "LOADED!" << endl;
    const int nImages = vstrImageFilenames.size();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout.precision(17);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;
    // Main loop
    cv::Mat im;
    int proccIm = 0;
    for(int ni=0; ni<nImages; ni++, proccIm++)
    {

        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // cout << "tframe = " << tframe << endl;
        SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];

        //std::cout << "T: " << T << std::endl;
        //std::cout << "ttrack: " << ttrack << std::endl;

        if(ttrack<T) {
            //std::cout << "usleep: " << (dT-ttrack) << std::endl;
            usleep((T-ttrack)*1e6); // 1e6
        }
    }
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);

    const string proto_map_file = "sip_" + string(argv[argc-1]) + ".pb";
    SLAM.SaveProtoMap(proto_map_file);

    return 0;
}

void LoadImages(const string &strPathToSequenceFullPath, vector<string> &vstrImages, vector<double> &vTimeStamps, const double dT, const double start_time_stamp)
{
    ifstream fImagePaths;
    fImagePaths.open(strPathToSequenceFullPath.c_str());
    vstrImages.reserve(5000);
    double timestamp = start_time_stamp;
    while(!fImagePaths.eof())
    {
        string s;
        getline(fImagePaths,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(ss.str());
            vTimeStamps.push_back(timestamp);
            timestamp += dT;
        }
    }
}
