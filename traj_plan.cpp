    #include <stdio.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include<cmath>
#include <Eigen/Core>


using namespace cv;
using namespace std;



float  deg2rad(float);

int circleRay = 1;
int row = 1000;
int col = 1000;
float k_1 = 0.5;
float k_2 = 2.5; 
float deltaTime = 0.01;
float R = 0.15;
float B = 0.3;
static int size = 0;
static float traj[3];


typedef cv::Mat_< cv::Vec3b > RGBImage;


typedef struct 
 {
    float x=-200;
    float y=-200;
    float theta= deg2rad(270);   //rad
} RobotPose;

typedef struct 
 {
    float linVel = 0;
    float angVel = 0;  
} CommandVelocity;




void computeCommandVel(const RobotPose&, CommandVelocity &   );
void moveRobot(RobotPose&, CommandVelocity);
void printStats(RobotPose& , CommandVelocity& );
void sleep(float);
void drawPoints(RGBImage& img,  int radius);


int main(int argc, char** argv )
{


    RobotPose robotPose;
    CommandVelocity commandVel;


    Mat M(row,col, CV_8UC3, Scalar(255,255,255));
    RGBImage img = M;
    while(abs(robotPose.x) > 10 || abs(robotPose.y) > 10)
    {

        computeCommandVel(robotPose,commandVel);
        moveRobot(robotPose, commandVel);

        //printStats(robotPose, commandVel);
        drawPoints(img,circleRay);
       // sleep(deltaTime);
        imshow("immagine", img);
         imwrite("traj.png", img);
    }


    //waitKey(0);

    return 0;
}




void computeCommandVel(const RobotPose& pose, CommandVelocity& vel){
  
    float orientation = pose.theta;

    vel.linVel = -k_1*(pose.x*cos(orientation) + pose.y*sin(orientation) );
    vel.angVel = k_2*(atan2(pose.y, pose.x) - orientation + M_PI);


}



void moveRobot(RobotPose& robotPose, CommandVelocity vel){

    //float omega_r, omega_l;

    float omega = vel.angVel;
    float velocity = vel.linVel;

    float orientation = robotPose.theta;
    //omega_r = 0.5*(omega*B + 2*velocity)/R; 
    //omega_l = 0.5*(-omega*B + 2*velocity)/R;

    robotPose.x +=  deltaTime*velocity*cos(orientation);
    robotPose.y +=  deltaTime*velocity*sin(orientation);
    robotPose.theta +=  deltaTime*omega;   
   // if(robotPose.theta>6.28){ 
    //    robotPose.theta -= 2*M_PI;
    //}

    traj[0] =   robotPose.x;
    traj[1] =   robotPose.y;
    traj[2] =   robotPose.theta;

    size++;

}


void printStats(RobotPose& robotPose, CommandVelocity& commandVel){

        printf("Lin Vel %f\n    ", commandVel.linVel);
        printf("Ang Vel %f\n    ", commandVel.angVel);
        printf("X %f\n    ", robotPose.x);
        printf("Y %f\n    ", robotPose.y);
        printf("TH %f\n    ", robotPose.theta*180.0/M_PI);
        printf("\n\n############\n\n    ");
}


void sleep(float seconds){
    clock_t startClock = clock();
    float secondsAhead = seconds * CLOCKS_PER_SEC;
    // do nothing until the elapsed time has passed.
    while(clock() < startClock+secondsAhead);
    return;
}


float deg2rad(float orientation){

   return orientation*M_PI /180.0;
}



void drawPoints(RGBImage& img,
		  int radius){

    int rows=img.rows;
    int cols=img.cols;
    Scalar color;
    color = 255;
    for (size_t i = 0; i < size; ++i ){

      int r = traj[0];
      int c = traj[1];
        
   //   if(it.first==-1||r>=rows){     
   //    i++;
   //       continue;
   //   }
     
      //if(c<0||c>=cols)
	//continue;
    if(size==0){
      cv::circle(img, cv::Point(0,0), 5, 100);
      
    }
      cv::circle(img, cv::Point(c+col/2,r+row/2), 5, color);
    //  i++;
    }
  } 

