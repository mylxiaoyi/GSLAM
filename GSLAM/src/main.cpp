//
//  main.cpp
//  STAR
//
//  Created by Chaos on 4/25/16.
//  Copyright © 2016 Chaos. All rights reserved.
//

#include "opencv2/imgproc/imgproc.hpp"
#include "System.h"
#include "GlobalReconstruction.h"
#include <sys/time.h>
#include <sys/stat.h>

int main(int argc, char **argv){
    
    printf("version no global ptr\n");
    system("exec rm -r /Users/chaos/Desktop/debug/*");
    
    argv[1]="/Users/chaos/Downloads/sequences/robot";
    argv[2]="/Users/chaos/Downloads/sequences/ORBvoc.txt";
    std::cout<<"process "<<argv[1]<<std::endl;
    
    char name[200];
    sprintf(name,"%s/viewer",argv[1]);
    
    mkdir(name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    //argv[2]="/Users/chaos/Downloads/ORB_SLAM2-master/Vocabulary/ORBvoc.txt";
    
    char loadname[200];
    sprintf(loadname,"%s/framestamp.txt",argv[1]);
    ifstream timestamps(loadname);
    
    //load imu
    sprintf(loadname,"%s/shake.mov",argv[1]);
    
    sprintf(loadname,"%s/config.yaml",argv[1]);
    GSLAM::System slamSystem(argv[2],loadname);
    slamSystem.path=argv[1];
    
    sprintf(loadname,"%s/gyro.txt",argv[1]);
    slamSystem.imu.loadImuData(loadname);

    
    int frameCount=0;
    std::ofstream record("/Users/chaos/Desktop/debug/time.txt");
    cv::Mat curImage,nextImage;
    cv::Mat image, rgb;

    while ( true ){
        double timestamp;
        timestamps>>timestamp;
        
        if(frameCount==slamSystem.frameStart){
            
            slamSystem.colorImage=&rgb;
            image.copyTo(curImage);
            
        }else if(frameCount>slamSystem.frameStart){
            image.copyTo(nextImage);
            slamSystem.preloadImage=&nextImage;
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            GSLAM::Transform pose=slamSystem.Track(curImage,timestamp,frameCount);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            record<<ttrack<<std::endl;
        }

        frameCount++;
        
        if(frameCount>slamSystem.frameEnd){
            break;
        }
    }

    record.close();
    slamSystem.finish();
    cout<<"processed "<<frameCount<<" frames"<<endl;
    timestamps.close();
    return 0;
}
