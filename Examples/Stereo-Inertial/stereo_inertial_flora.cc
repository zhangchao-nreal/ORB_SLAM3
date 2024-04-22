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
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>
// #include <yaml-cpp/yaml.h>


#include<System.h>
#include "ImuTypes.h"
#include "Optimizer.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

void LoadCalibParas(const string &setting_files, const string &calib_path);


int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_flora path_to_vocabulary path_to_settings path_to_sequence_folder_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }

    const int num_seq = 1;
    bool bFileName= false;
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        // string pathTimeStamps(argv[(2*seq) + 4]);

        // string pathCam0 = pathSeq + "/mav0/cam0/data";
        // string pathCam1 = pathSeq + "/mav0/cam1/data";
        // string pathImu = pathSeq + "/mav0/imu0/data.csv";
        string pathCam0 = pathSeq + "/ground_truth/data/cam0_path.txt";
        string pathCam1 = pathSeq + "/ground_truth/data/cam1_path.txt";
        string pathImu = pathSeq + "/ground_truth/data/imu_data.txt";
        string pathCalib = pathSeq  + "/ground_truth/calib/calib.yaml";

        LoadImages(pathCam0, pathCam1, pathSeq, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        // LoadCalibParas(argv[2], pathCalib);

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, true);

    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        double t_rect = 0.f;
        double t_resize = 0.f;
        double t_track = 0.f;
        int num_rect = 0;
        int proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            cout << "-----------------------------image " << ni << "------------------------" <<endl;
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }

            double tframe = vTimestampsCam[seq][ni];

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe,vImuMeas);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_rect + t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }


    }
    // Stop all threads
    SLAM.Shutdown();


    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file = string(argv[argc-1]);
        const string f_file = string(argv[argc-1]);
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &rootPath,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    std::vector<std::string> results_left;
    std::vector<std::string> results_right;
    std::vector<double> timestamps;

    std::ifstream fp_left(strPathLeft);

    std::string line;
    while (std::getline(fp_left, line)) {
      if (line[0] == '#') {
        continue;
      }
      double timestamp;
      std::string path;

      std::stringstream ss(line);

      ss >> timestamp;
      ss >> path;

      timestamps.push_back(timestamp);
      results_left.push_back(rootPath +"/"+ path);
    //   cout << "timestamp: " << timestamp << " path: " << rootPath +"/"+ path;
    }
    fp_left.close();

    std::ifstream fp_right(strPathRight);

    while (std::getline(fp_right, line)) {
      if (line[0] == '#') {
        continue;
      }
      double timestamp;
      std::string path;

      std::stringstream ss(line);

      ss >> timestamp;
      ss >> path;

      results_right.push_back(rootPath +"/"+ path);
    //   cout << "timestamp: " << timestamp << " path: " << rootPath +"/"+ path;
    }
    fp_right.close();

    vTimeStamps = timestamps;
    vstrImageLeft = results_left;
    vstrImageRight = results_right;
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    std::ifstream fp(strImuPath);
    // CHECK(fp.is_open()) << strImuPath << " is not valid!";
    std::string line;
    double current_timestamp, tmp1, tmp2, tmp3, tmp4;
    Eigen::Vector3d gyro, accel;
    while (std::getline(fp, line)) {
      if (line[0] == '#') {
        continue;
      }
      std::stringstream ss(line);
      ss >> current_timestamp >> gyro.x() >> gyro.y() >> gyro.z() >> accel.x() >> accel.y() >> accel.z() >> tmp1 >>
          tmp2 >> tmp3 >> tmp4;
      vTimeStamps.push_back(current_timestamp);
      vGyro.push_back(cv::Point3f(gyro.x(), gyro.y(), gyro.z()));
      vAcc.push_back(cv::Point3f(accel.x(), accel.y(), accel.z()));
    }
}

// void LoadCalibParas(const string &orb_setting_files, const string &calib_path) {
//     cout << "calib_path: " << calib_path << endl;
//     YAML::Node calib_fsSettings = YAML::LoadFile(calib_path);
//     cout << __LINE__ << endl;
//     std::vector<double> cc0, fc0, kc0;
//     std::vector<double> cc1, fc1, kc1;
//     std::vector<double> imu_noises;
//     std::vector<double> imu_q_cam0;
//     std::vector<double> imu_p_cam0;
//     std::vector<double> cam0_q_cam1;
//     std::vector<double> cam0_p_cam1;
//     const auto child_left = calib_fsSettings["camera"]["cam0"];
//     const auto child_right = calib_fsSettings["camera"]["cam1"];
//     const auto imu_noises_node = calib_fsSettings["imu"]["imu_noises"];
//     cout << __LINE__ << endl;
//     for (const auto &cc : child_left["cc"]) {
//         cc0.push_back(cc.as<double>());
//     }
//     cout << __LINE__ << endl;
//     for (const auto &fc : child_left["fc"]) {
//         fc0.push_back(fc.as<double>());
//     }
//     for (const auto &kc : child_left["kc"]) {
//         kc0.push_back(kc.as<double>());
//     }
//     for (const auto &cc : child_right["cc"]) {
//         cc1.push_back(cc.as<double>());
//     }
//     for (const auto &fc : child_right["fc"]) {
//         fc1.push_back(fc.as<double>());
//     }
//     for (const auto &kc : child_right["kc"]) {
//         kc1.push_back(kc.as<double>());
//     }
//     for (const auto &noise : imu_noises_node) {
//         imu_noises.push_back(noise.as<double>());
//     }
//     for (const auto &kk : child_left["imu_p_cam"]) {
//         imu_p_cam0.push_back(kk.as<double>());
//     }
//     for (const auto &kk : child_left["imu_q_cam"]) {
//         imu_q_cam0.push_back(kk.as<double>());
//     }
//     for (const auto &kk : calib_fsSettings["camera"]["cam0_q_cam1"]) {
//         cam0_q_cam1.push_back(kk.as<double>());
//     }
//     for (const auto &kk : calib_fsSettings["camera"]["cam0_p_cam1"]) {
//         cam0_p_cam1.push_back(kk.as<double>());
//     }
//     cout << "cc0" << cc0[0] << " " << cc0[1] << " fc0: " << fc0[0] << " " << fc0[1] << " kc0: " << kc0[0] << kc0[1] << kc0[2] << kc0[3] << endl;
//     cout << "cc1" << cc1[0] << " " << cc1[1] << " fc1: " << fc1[0] << " " << fc1[1] << " kc1: " << kc1[0] << kc1[1] << kc1[2] << kc1[3] << endl;
//     cout << " imu_noises:" << imu_noises[0] << imu_noises[1] << imu_noises[2] << imu_noises[3] << endl;
//     cout << " imu_p_cam0:" << imu_p_cam0[0] << imu_p_cam0[1] << imu_p_cam0[2] << endl;
//     cout << " cam0_p_cam1:" << cam0_p_cam1[0] << cam0_p_cam1[1] << cam0_p_cam1[2] << endl;
//     cout << " imu_q_cam0:" << imu_q_cam0[0] << imu_q_cam0[1] << imu_q_cam0[2] << imu_q_cam0[3] << endl;
//     cout << " cam0_q_cam1:" << cam0_q_cam1[0] << cam0_q_cam1[1] << cam0_q_cam1[2] << cam0_q_cam1[3] << endl;
//     cout << "orb_setting_files: " << orb_setting_files << endl;
//     cv::FileStorage orb_fsSettings_read(orb_setting_files.c_str(), cv::FileStorage::READ);
//     // cv::FileNode root = orb_fsSettings_read.root();
//     orb_fsSettings_read.release();
//     cv::FileStorage orb_fsSettings_write(orb_setting_files.c_str(), cv::FileStorage::WRITE);
//     // orb_fsSettings_read.root() >> orb_fsSettings_write.root();
//     // orb_fsSettings_read.root().write(orb_fsSettings_write);
//     orb_fsSettings_write << orb_fsSettings_read.root();

//     // orb_fsSettings_write << "Camera1.fx" << 220;
//     orb_fsSettings_write.release();
//     // ofstream fout(orb_setting_files);
//     // fout << orb_fsSettings;
//     // fout.close();
//     // cv::FileStorage calib_fsSettings(orb_setting_files.c_str(), cv::FileStorage::READ);
//     // if(!calib_fsSettings.isOpened())
//     // {
//     //    cerr << "Failed to open settings file at: " << orb_setting_files << endl;
//     //    exit(-1);
//     // }

//     // auto node = calib_fsSettings["File.version"];
// }

