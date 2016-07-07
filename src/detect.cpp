    #include <cv.h>  
    #include <highgui.h>  
    #include <iostream>  
    #include "ImageProcessClass.h"
    #include "ros/ros.h"
    #include "std_msgs/String.h"
    using namespace cv;  
    using namespace std;  


   int main(int argc,char **argv) 
{
    ros::init(argc, argv, "pose_publisher");
//    ros::NodeHandle n;
/*    ros::Publisher pose_pub = n.advertise<std_msgs::String>("pose_pub", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
  while (ros::ok())
  {
    
    // * This is a message object. You stuff it with data, and then publish it.
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    pose_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
*/
/*
//CvCapture* cap;

   //cap = cvCaptureFromFile("runway061301.mp4");//cvCreateCameraCapture(-1);//cvCaptureFromCAM(-1); //home/zhangjing//runner//r
   VideoCapture cap("runway061301.mp4");
   if(!cap.isOpened())
	{
		cout<<"fail to open!"<<endl;
	}

  Mat mat = imread("1.png");
  cout<<mat.rows<<endl;
  

   int numFrames = 0;//(int) cvGetCaptureProperty(cap,CV_CAP_PROP_FRAME_COUNT);//得到视频帧数，

//得到的帧数是148帧


IplImage* pImg = NULL;

int count = 0;
namedWindow("Extracted frame");
//while((pImg = cvQueryFrame(cap))!=NULL)
{

cvShowImage("Extracted frame", pImg);
count++;
cout<<count<<endl;

}//得到的帧数只有60帧*/

    CImageProcessClass image;
    image.testVisualNavigation();
   //image.testDistortion();
    // image.testCali("runway1133.bmp", 297, 210);
 	return -1;	
}

