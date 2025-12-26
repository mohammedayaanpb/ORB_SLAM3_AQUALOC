/**
* This file is part of ORB-SLAM3
* Modified for AQUALOC underwater dataset
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_aqualoc path_to_vocabulary path_to_settings path_to_associations" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strAssociationFile = string(argv[3]);
    LoadImages(strAssociationFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages <= 0)
    {
        cerr << "ERROR: No images found in association file." << endl;
        return 1;
    }

    cout << endl << "-------" << endl;
    cout << "AQUALOC Monocular SLAM" << endl;
    cout << "Images in sequence: " << nImages << endl;
    cout << "-------" << endl << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    // Main loop
    cv::Mat im;
    for(int ni = 0; ni < nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // Resize image if needed
        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame (simulate real-time)
        double T = 0;
        if(ni < nImages - 1)
            T = vTimestamps[ni + 1] - tframe;
        else if(ni > 0)
            T = tframe - vTimestamps[ni - 1];

        if(ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strAssociationFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFile.c_str());
    
    if(!fAssociation.is_open())
    {
        cerr << "ERROR: Cannot open association file: " << strAssociationFile << endl;
        return;
    }
    
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation, s);
        
        if(!s.empty() && s[0] != '#')
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            ss >> sRGB;
            
            if(!sRGB.empty())
            {
                vTimestamps.push_back(t);
                vstrImageFilenames.push_back(sRGB);
            }
        }
    }
    
    fAssociation.close();
}
