#include <cv.h>  
    #include <highgui.h>  
    #include <iostream>  
    using namespace cv;  
    using namespace std;  
    int main(int argc,char **argv)  
    {  
        Mat image;  
        image = imread(argv[1],1);  
      
        if(argc != 2 || !image.data)  
        {  
            cout << "No image data\n";  
            return -1;  
        }  
      
        namedWindow("Display Image",CV_WINDOW_AUTOSIZE);  
        imshow("Display Image",image);  
        waitKey(0);  
        return 0;  
    }  

//HIGHGUI ERROR: V4L/V4L2: VIDIOC_S_CROP
//是因为设置图像分辨率失败，使用了默认分辨率

//  g++ detect.cpp ImageProcessClass.cpp CommonFunClass.cpp CameraProcess.cpp -o Detect `pkg-config opencv --cflags --libs`

