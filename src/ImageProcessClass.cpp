//#include "StdAfx.h"
#include <cv.h>  
#include <highgui.h>  
#include <iostream>  
#include "ImageProcessClass.h"
#include "CommonFunClass.h"
#include "CameraProcess.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"
#include "track_follower/Num.h"

CvCapture* g_capture = 0; 
double g_dR=0, g_dG=0, g_dB = 0;
double g_dRadius = 0;
bool g_isInverse = false;
bool g_isFirst = true;
#define MAX_PATH   1000

#define TEST 1
#define VEDIO 0
#define CAMERA  1
#define CALIBRATE 0

//CvMat* intrinsics = 0;
//CvMat* distortion_coeff = 0;
Mat intrinsics;
Mat distortion_coeff;
CImageProcessClass::CImageProcessClass(void)
{
    if (!g_capture)
    {
       cvReleaseCapture(&g_capture);
    }
    waitKey(10);
    g_capture = cvCreateCameraCapture(0); 
 //   intrinsics = cvCreateMat(3,3,CV_32FC1);
 //  distortion_coeff = cvCreateMat(1,4,CV_32FC1);
   intrinsics = Mat(3,3,CV_32FC1);
   distortion_coeff = Mat(1,4,CV_32FC1);
}


CImageProcessClass::~CImageProcessClass(void)
{
	cvReleaseCapture(&g_capture);

     //  cvReleaseMat(&intrinsics);
    //   cvReleaseMat(&distortion_coeff);
}


void CImageProcessClass::testVisualNavigation(void)
{
       double dCaliX = 0;
       double dCaliY = 0;
   
       ros::NodeHandle n;
       ros::Publisher pose_pub = n.advertise<track_follower::Num>("pose_pub", 1000);
       track_follower::Num msg;

       getCaliData("/opt/ros/hydro/catkin_ws/src/track_follower/src/cali.txt", dCaliX, dCaliY);    
       cout<<"cali X =  "<<dCaliX<<", y = "<<dCaliY<<endl;
#if TEST
system("rm -r /opt/ros/hydro/catkin_ws/src/track_follower/src/Result");
system("mkdir  /opt/ros/hydro/catkin_ws/src/track_follower/src/Result");

#endif

long frameToStart = 1;
 double rate = 1.0;
long frameToStop = 1; //400 //572;

#if VEDIO

	//打开视频文件：其实就是建立一个VideoCapture结构
	VideoCapture capture("/opt/ros/hydro/catkin_ws/src/track_follower/src/runway062202.avi");// runway061301.mp4 runway062202.avi 0530 061301拐弯

        capture.set(CV_CAP_PROP_CONVERT_RGB, 1);
	//检测是否正常打开:成功打开时，isOpened返回ture
	if(!capture.isOpened())
	{
	    cout<<"capture fail to open!"<<endl;
            return;//
	}
	//获取整个帧数
	long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	cout<<"all frame: "<<totalFrameNumber<<"帧"<<endl;

	//设置开始帧()
	capture.set( CV_CAP_PROP_POS_FRAMES,frameToStart);
	cout<<"start frame: "<<frameToStart<<endl;

	//设置结束帧
	int frameToStop = 270; //totalFrameNumber;
	if(frameToStop < frameToStart)
	{
	    cout<<"exit！"<<endl;
	    return;//
	}
	else
	{
	    cout<<"end frame: "<<frameToStop<<endl;
              
	}
  
	//获取帧率
	 rate = capture.get(CV_CAP_PROP_FPS);  
	//cout<<"frame rate: "<<rate<<endl;

    int currentFrame1 = capture.get(CV_CAP_PROP_POS_FRAMES);
    double currentTime = capture.get(CV_CAP_PROP_POS_MSEC);
    double fps = currentFrame1 / (currentTime / 1000);
    cout<<"frame rate: "<<fps<<endl;

#endif 
	//定义一个用来控制读取视频循环结束的变量
	bool IsStop = false;
	//承载每一帧的图像
	Mat frame;
	//显示每一帧的窗口
//	namedWindow("Extracted frame");
	//两帧间的间隔时间:
	int delay = 1000/rate;
        //cout<<delay<<endl;//return;

	//利用while循环读取帧
	//currentFrame是在循环体中控制读取到指定的帧后循环结束的变量
	long currentFrame = frameToStart;

	//滤波器的核
	int kernel_size = 3;
	Mat kernel = Mat::ones(kernel_size,kernel_size,CV_32F)/(float)(kernel_size*kernel_size);
	int nEdgeNum = 0; //检测到的边缘数目，根据不同情况做相应处理
	//存储之前检测帧的坐标及角度，目的是为了预测下一帧的坐标角度处理特殊情况
	CvPoint ptTemp[1000];
	memset(ptTemp, 0, sizeof(CvPoint)*1000);
	double dAngleTemp[1000] = {0};
	int nIndex = 0;

        Mat dst;
	CvRect rect;
	CvPoint pt = cvPoint(0,0);
	double dAngle = 0;
	CCommonFunClass fun;
	string s;
	char buffer[MAX_PATH] = {0};
	CvPoint ptStart, ptEnd;

        CCameraProcess proc;
	CvSize szChessBoard;
	szChessBoard.height = 6;
	szChessBoard.width = 9;
        
        string ss = "/opt/ros/hydro/catkin_ws/src/track_follower/src/distortion.txt";
        proc.distortionData(ss, intrinsics, distortion_coeff, true);
        
        bool isFirstAngle = true;
        double dAngleStart = 90;
        double dAngleEnd = 90;
        double dAngleDiff = 0;
        double dDis = 0;
#if VEDIO
	while(!IsStop)
	{
	   //读取下一帧
	   if(!capture.read(frame))
	   {
	      cout<<"fail to open!"<<endl;
	      return;	
	   }
	
#endif

#if CAMERA
         while(1)
	  {
          
if (g_capture)
{
		 frame = cvQueryFrame(g_capture);
cout<<frame.rows<<"   "<<frame.cols<<endl;
}
		 if (frame.empty())
		 {
			 continue;
		 }

                //////////// distortion 

#if CALIBRATE
	int ret = proc.calcDistortionCorrectionParam(g_capture, 20, szChessBoard, intrinsics, distortion_coeff); 
	if (ret < 0)
	{
		return;
	}
	
 Mat mat1;
                 proc.distortionCorrection(frame, intrinsics, distortion_coeff, mat1, true); 

#endif 
                 Mat mat;
		 proc.distortionCorrection(frame, intrinsics, distortion_coeff, mat, false); //true:write calibrate data, false: read calibrate data
	         imwrite("runway11.bmp", mat);	
                 frame = mat;	
#endif
                ///////////////
		//这里加滤波程序
//		imshow("Extracted frame", frame);
		filter2D(frame, frame, -1, kernel);
		imwrite("runway.bmp", frame);	

		//step1:处理，获得2边缘的中心点坐标、角度
		rect.x = 5;//frame.cols/10;
		rect.width = frame.cols-5;//8*frame.cols/10;
		rect.y = 1*frame.rows/3;
		rect.height = frame.rows/3;

		if (g_isFirst)
		{

		     int h = 20; 
			CvRect r;
			r.x = frame.cols/2 - h;
			r.width = 2*h;
			r.y = 2*frame.rows/3;
			r.height = frame.rows/3;
			getRegionAverageValue(frame, r, g_dRadius, g_dR, g_dG, g_dB);		
			nEdgeNum = runwayFunThree(frame, nIndex, rect, dst, ptStart, ptEnd, dAngle);//
			g_isFirst = false;
		}
		
		//flip(frame, frame, 1);
		//Sleep(100);
                
                double t1 = (double)getTickCount();
		//nEdgeNum = runwayFunThree(frame, nIndex, rect, dst, ptStart, ptEnd, dAngle);//	
	        nEdgeNum = runwayFunOne(frame, nIndex, rect, dst, ptStart, ptEnd, dAngle);

#if TEST
sprintf(buffer, "/opt/ros/hydro/catkin_ws/src/track_follower/src/Result//%d.bmp", nIndex);
imwrite(buffer, dst);
#endif              

nIndex++;
   double t2 = (double)getTickCount();
                double t = 1000*(t2-t1)/getTickFrequency();

                getAngleDiff(dAngle, dAngleStart, dAngleEnd, isFirstAngle, dAngleDiff);
//                cout<<"angle diff: "<<dAngleDiff<<endl;

                getDistance(ptStart, ptEnd, dCaliX, dCaliY, dDis);
//                cout<<"dis: "<<dDis<<endl;
                cout<<"time: "<<t<<" angle: "<<dAngle<<" angle diff: "<<dAngleDiff<<" dis: "<<dDis<<endl;
                msg.angdiff = dAngleDiff;
                msg.distance = dDis;
                msg.header.stamp = ros::Time::now();
                pose_pub.publish(msg);

//		imshow("Contours", dst);
	//	imshow("after filter", frame);
//		cout<<"frame Num: "<<currentFrame<<"帧"<<endl;
  //              cout<<endl;
		//waitKey(int delay=0)当delay ≤ 0时会永远等待；当delay>0时会等待delay毫秒
		//当时间结束前没有按键按下时，返回值为-1；否则返回按键

		int c = waitKey(100);//delay);
		//按下ESC或者到达指定的结束帧后退出读取视频
		if((char)c==27 || currentFrame>frameToStop)
		{
		     IsStop = true;
//                     return;
		}
		//按下按键后会停留在当前帧，等待下一次按键
		/*if (c==32)
		{
			continue;
		}
		else*///
               if( c >= 0)		
		{
		 waitKey(0);
#if VIDEO
                capture.release();
#endif
   //             cvReleaseMat(&intrinsics);
	//        cvReleaseMat(&distortion_coeff);
	        //waitKey(0);
             if (!g_capture)
             {
                cvReleaseCapture(&g_capture);
             }
             return;
	}              
	    currentFrame++;
//	    nIndex++;
	}
	//关闭视频文件
#if VIDEO
                capture.release();
#endif
	waitKey(0);
}

void CImageProcessClass::sharpenUSM(Mat src, Mat &_dst, double sigma, int threshold, float amount)
{	
	Mat imgBlurred;
	GaussianBlur(src, imgBlurred, Size(), sigma, sigma);  
	Mat lowContrastMask = abs(src-imgBlurred)<threshold;  
	_dst = src*(1+amount)+ imgBlurred*(-amount);  
	src.copyTo(_dst, lowContrastMask);  
#if TEST
	imwrite("sharpenUSM.bmp", _dst);  
#endif
}

void CImageProcessClass::getDistance(CvPoint ptStart, CvPoint ptEnd, double dCaliX, double dCaliY, double &_dDis)
{
    double dX = fabs(ptEnd.x - ptStart.x) * dCaliX;
    double dY = fabs(ptEnd.y - ptStart.y) * dCaliY;

    _dDis = dX * dX + dY * dY;
    _dDis = sqrt(_dDis);
}

void CImageProcessClass::getAngleDiff(const double dAngle, double &dAngleStart, double &dAngleEnd, bool &isFirst, double &_dAngle)
{
   if (isFirst)
   {
      dAngleStart = dAngle;
      isFirst = false;
   }
   else
   {
      dAngleEnd = dAngle;
      if (dAngleStart < 0)
      {
          dAngleStart = fabs(dAngleStart);
      }
      else
      {
          dAngleStart = 180 - dAngleStart; 
      }

      if (dAngleEnd < 0)
      {
          dAngleEnd = fabs(dAngleEnd);
      }
      else
      {
          dAngleEnd = 180 - dAngleEnd; 
      }
      _dAngle = dAngleEnd - dAngleStart;

      // 
      dAngleStart = dAngleEnd;
   }

}

int CImageProcessClass::getRectImage(const Mat src, CvRect rect, Mat &_dst)
{
	int nRow = src.rows;
	int nCol = src.cols;

	int nLeft = rect.x;
	int nRight = rect.x + rect.width;
	int nTop = rect.y;
	int nBottom = rect.y + rect.height;
	if (nLeft<0 || nRight>nCol || nTop<0 || nBottom>nRow)
	{
		cout<<"region is error!"<<endl;
		return -1;
	}

	Mat matRow = src.rowRange(nTop,nBottom).clone();  
	Mat mat = matRow.colRange(nLeft,nRight).clone(); 
	nRow = mat.rows;
	nCol = mat.cols;
	_dst = mat;
	imwrite("rect.bmp", mat);//matRow);	
}

void CImageProcessClass::getRegionAverageValue(const Mat src, CvRect rect, double &_dRadius, double &_dR, double &_dG, double &_dB)
{
	Mat mat;
	getRectImage(src, rect, mat);
	imwrite("RegionAverageValue.bmp", mat);

	if (3 == mat.channels())
	{
		vector<Mat> mat_channels; 
	//	cvtColor(mat, mat, CV_BGR2HSV);
		split(mat, mat_channels); //RGB 2、1、0

		Scalar  mean[3], stddev[3];  
		for (int i=0; i<3; i++)
		{
			cv::meanStdDev(mat_channels.at(i), mean[i], stddev[i]);
		}
		  
		_dR = mean[2].val[0];  
		_dG = mean[1].val[0];  
		_dB = mean[0].val[0]; 

		_dRadius = -1;
	}
	else
	{
		Mat matGray;
		cvtColor(mat, matGray, CV_BGR2GRAY);

		Scalar  mean, stddev;  
		cv::meanStdDev(matGray, mean, stddev);  
		double  dGray = mean.val[0]; 

		_dR = dGray;
		_dG = dGray;
		_dB = dGray;

		double dMin = 0.0, dMax = 0.0;  
		minMaxLoc(matGray, &dMin, &dMax);  

		_dRadius = max(abs(dGray-dMax),abs(dGray-dMin));
	}
	
}

void CImageProcessClass::enhanceRegion(Mat src, const double dRadius, const double dR, const double dG, const double dB, Mat &_dst)
{
	_dst = Mat::zeros (src.rows, src.cols, src.type());
	float fGray = 0;

	if (3 == src.channels()) //RGB彩色图像的处理方法
	{
		 double Row0 = sqrt(dR * dR + dG * dG + dB * dB);
	     double Row = 0;
		 float fR = 0, fG = 0, fB = 0;
		 Vec3b* p3; 
		 Vec3b* pDst3;
		 //cvtColor(src, src, CV_BGR2HSV);
		 for(int row=0; row<src.rows; row++)  
		 {
			 p3 = src.ptr<Vec3b>(row);  
			 pDst3 = _dst.ptr<Vec3b>(row); 
			 for(int col=0; col<src.cols; col++)  
			 {
				 fB = p3[col][0]; 
				 fG = p3[col][1];
				 fB = p3[col][2];  //Red 

				 Row = sqrt(fR * fR + fG * fG + fB * fB);
				 Row = (fR * dR + fG * dG + fB * dB)/(Row*Row0);
				 fGray = Row*Row*255.0;

				 pDst3[col][0] = fGray;
				 pDst3[col][1] = fGray;
				 pDst3[col][2] = fGray;
			 }
		 }

	}
	else //灰度图的处理方法
	{
		Scalar  mean, stddev;  
		cv::meanStdDev(src, mean, stddev);  
		double  dMean = mean.val[0]; 
	    
		float* p; 
		float* pDst;
		for (int row=0; row<src.rows; row++)
		{
			p = src.ptr<float>(row); 
			pDst = _dst.ptr<float>(row); 
			for (int col=0; col<src.cols; col++)
			{
				fGray = p[col]; 
				if (fabs(dMean-fGray) <= dRadius)
				{
					pDst[col] = fGray;
				}
			}
		}

	}
}

//直接检测跑道
int CImageProcessClass::runwayFunThree(const Mat src, int nIndex, CvRect rect, Mat &_dst, CvPoint &_ptStart, CvPoint &_ptEnd, double &_dAngle)
{
	Mat mat;
	getRectImage(src, rect, mat);
	int nRow = mat.rows;
	int nCol = mat.cols;
        
        sharpenUSM(mat, mat);
        Mat matGray;
	if (3 == mat.channels())
	{
		cvtColor(mat, mat, CV_BGR2HSV);//CV_BGR2YCrCb
        //cvtColor(mat, matGray, CV_BGR2GRAY);
	}

	vector<Mat> mat_channels;  
	split(mat, mat_channels);  //HSV

	matGray = mat_channels.at(1); //RGB 2、1、0
	GaussianBlur(matGray, matGray, Size(3,3), 0, 0); 	
/*	Mat matTemp;
	enhanceRegion(mat, g_dRadius, g_dR, g_dG, g_dB, matTemp);
	imwrite("enhanceRegion.bmp", matTemp);

	if (3 == matTemp.channels())
	{
        cvtColor(matTemp, matGray, CV_BGR2GRAY);
	}
*/
	threshold(matGray, matGray, 0, 255, CV_THRESH_OTSU);

	if (g_isInverse)
	{
		for(int i=0; i<matGray.rows; i++)
		{
			//matGray.row(i) = 255 - matGray.row(i) + 0;
		}

               // float* p; 
               Vec3b* p;
		for (int row=0; row<matGray.rows; row++)
		{
			//p = matGray.ptr<float>(row); 
                        p = matGray.ptr<Vec3b>(row);
			for (int col=0; col<matGray.cols; col++)
			{
				//p[col] = 255.0 - p[col];
                            //matGray.at<float>(row,col) = 255.0 - matGray.at<float>(row,col);
                            p[col][0] = 255 - p[col][0]; 
                            p[col][1] = 255 - p[col][1]; 
                            p[col][2] = 255 - p[col][2]; 
			}
		}

	}
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	//erode(matGray, matGray, element);
	//dilate(matGray, matGray, element);

	//flip(matGray, matGray, 1);//1:flip by y axis，0: flip by x axis, -1: flip by both axises

#if TEST
	imwrite("thresh.bmp", matGray);
#endif	
	//
	CvRect r = cvRect(0,0,0,0);
	bool isOK = false;
	CvBox2D boxMax;
	CvRect rcMax = cvRect(0,0,0,0);

	CvBox2D box;
	int nNum = 0;
	CvPoint2D32f pt[4];
	vector<vector<Point> > contours;
	vector<vector<Point> > contours_max;
	Point ptCenter = cvPoint(nCol/2,nRow/2);
	double dMaxArea = 0;
	double dArea = 0;

	 //findContours的输入是二值图像
	findContours(matGray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
#if TEST
cout<<"num: "<<contours.size()<<endl;
#endif
	for(int i=0; i<(int)contours.size(); i++)//将有符号的contours.size()强制转换位非符号的；
	{
		dArea = contourArea(Mat(contours[i]));
if (dArea<100)
{
  continue;
}
		r = boundingRect(Mat(contours[i]));
		box = minAreaRect(Mat(contours[i]));
		isOK = r.height>nRow*0.8 && abs(box.center.x-ptCenter.x)<nCol/4;
		if (isOK)
		{	
			if (dArea > dMaxArea)
			{
				dMaxArea = dArea;
				contours_max.clear();
				contours_max.push_back(contours[i]);
				//box.center.x += rect.x;
			    //box.center.y += rect.y;
				boxMax = box;//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
				rcMax = r; 
				
				//drawContours(mat,contours_max,-1, Scalar(0,0,0),10); //-1画所有轮廓	
			}        
		}
	} 
	if (g_isFirst)
	{
		dArea = matGray.rows*matGray.cols;
		if (dMaxArea < dArea/3)
		{
			g_isInverse = true;
		}
		return 0;
	}
 	_ptStart.x = rcMax.x + rcMax.width/2;
	_ptStart.y = rcMax.y + rcMax.height;
//	circle(mat,_pt,5,Scalar(0,0,0),2);

	vector<vector<Point> > contours_edge;
    if (contours_max.size() > 0)
	{
		getEdgeContours(mat, contours_max.at(0), rcMax, contours_edge);
		judgeContoursState(mat, nIndex, contours_edge, _dAngle);
#if TEST
		for (int j=0; j<contours_edge.size(); j++)
		{
			for (int i=0; i<contours_edge.at(j).size(); i++)
			{
				circle(mat,contours_edge.at(j).at(i),1,Scalar(0,0,0));
			}
		}
#endif
		if (0 == (int)_dAngle)//表示检测有误，就假设让直走
		{
			_dAngle = 90;
		}
        double dK = tan(CV_PI/180*_dAngle);
		{	
			_ptEnd.y = 0;
			_ptEnd.x = (_ptEnd.y - _ptStart.y)/dK + _ptStart.x;
#if TEST
			cv::line(mat,_ptStart,_ptEnd,cv::Scalar(0),4); 
#endif
		}
#if TEST
		cout<<"image leftup: "<<"(0,0)"<<","<<"image rightbottom: "<<"("<<mat.cols-1<<","<<mat.rows-1<<")"<<endl;
		cout<<"start Point: "<<"("<<_ptStart.x<<","<<_ptStart.y<<")"<<","<<"end Point: "<<"("<<_ptEnd.x<<","<<_ptEnd.y<<")"<<endl;
#endif
	}
	
	//cout<<"角度 = "<<boxMax.angle<<endl;
 	_dst = mat;


	return 0;
}

void CImageProcessClass::judgeContoursState(Mat mat, int nIndex, const vector<vector<Point> > contours, double &_dAngle)
{
	if (contours.size() < 2)
	{
		_dAngle = -1;
		return;
	}
	int nAngleFlag[2] = {0};
	double dAngle[2] = {0}; 
	double dK[2] = {0};
	double dRatio[2] = {0};
	double dMean = 0;
	int nNum = 0;
	int ret = 0;
	vector<double> vdAngle;
	bool isExistLeftWay = false;
	bool isExistRightWay = false;
	for (int i=0; i<2; i++)
	{
	   ret = getTangent(mat, contours.at(i), 5, dRatio[i], dAngle[i]);

	  // dAngle[i] = atan(dK[i])*180/CV_PI;//CV_PI/180*atan(dK[i]);
	  // dAngle[i] = -1*dAngle[i];
	   cout<< dAngle[i]<<", "<<tan(CV_PI/180*dAngle[i])<<endl;
	   if (ret > -1)
	   {
		   dMean += dAngle[i];
		   nNum++;
		   vdAngle.push_back(dAngle[i]);
	   }
	   if (ret>-1 && 0==i)
	   {
		   isExistLeftWay = true;
	   }
	   if (ret>-1 && 1==i)
	   {
		   isExistRightWay = true;
	   }
	}
	dMean = dMean/(nNum+0.00000001);
#if TEST
	cout<<"mean: "<<dMean<<endl;
#endif
	/*if (fabs(dMean) < 5)
	{
		cout<<"直走"<<endl;
		_dAngle = 0;
	}
	else*/
	{
		if (isExistLeftWay)
		{
			_dAngle = dAngle[0];
		}
		else if (!isExistLeftWay && isExistRightWay)
		{
			_dAngle = dAngle[1];
		}
		else if (!isExistLeftWay && !isExistRightWay)
		{
			_dAngle = 0;
		}

                double dLeft = 0; //负
		double dRight = 0; //正
		if (vdAngle.size() > 1)
		{
			double dAngle = 0;
			judgeMaxValueDire(vdAngle, _dAngle);
#if TEST
			cout<<"Angle: "<<_dAngle<<endl;
#endif                
			if ((int)vdAngle.at(0)*vdAngle.at(1) < 0)
			{
				if (vdAngle.at(0) < 0)
				{
					dLeft = vdAngle.at(0);
					dRight = vdAngle.at(1);
				}
				else
				{
					dLeft = vdAngle.at(1);
					dRight = vdAngle.at(0);
				}
				dAngle = (fabs(dLeft) + 180 - dRight)/2;
				cout<<"Angle: "<<dAngle<<endl;
				if ((int)dAngle > 90)
				{
					dAngle = 180 - dAngle;
				}
				_dAngle = _dAngle/fabs(_dAngle)*dAngle;
			}
		}
		else if (1 == vdAngle.size())
		{
			judgeMaxValueDire(vdAngle, _dAngle);
		}
		else if (vdAngle.size() < 1)
		{
			_dAngle = 90;
		}
		//_dAngle = -1*dAngle[1];
	}
#if TEST
	cout<<_dAngle<<endl;
	cout<<endl;
#endif
    return;
}


void CImageProcessClass::getEdgeContours(Mat mat, const vector<Point> contours, const CvRect rect, vector<vector<Point> > &_contours)
{
	vector<Point> contours_left;
	vector<Point> contours_right;
	int nLen = contours.size();

	bool isLeft = false;
	bool isRight = false;
	int nRight = rect.x + rect.width;
	int nLeft = rect.x;
	int nTop = rect.y;
	int nBottom = rect.y + rect.height;
	int nMid = rect.x + (rect.x+rect.width)/2;

	Point* ptLeft = new Point[rect.height];
	Point* ptRight = new Point[rect.height];
	for (int i=0; i<rect.height; i++)
	{
		ptLeft[i].x = nRight;
		ptLeft[i].y = -1; //-1代表这个点未更新

		ptRight[i].x = nLeft;
		ptRight[i].y = -1;
	}

	Point pt = cvPoint(0,0);
	int y = 0;
	for (int i=0; i<nLen; i++)
	{	
		pt = contours.at(i);
		isLeft = pt.x<nMid && pt.y>=nTop && pt.y<=nBottom;
		isRight = pt.x>nMid && pt.y>=nTop && pt.y<=nBottom;
		y = pt.y - nTop;
		if (isLeft)
		{
			ptLeft[y].x = pt.x<ptLeft[y].x? pt.x : ptLeft[y].x;
			ptLeft[y].y = pt.y;
		}
		if (isRight)
		{
			ptRight[y].x = pt.x>ptRight[y].x? pt.x : ptRight[y].x;
			ptRight[y].y = pt.y;
		}
	}

	CCommonFunClass fun;
	string s;
	char buffer[MAX_PATH] = {0};

	for (int i=0; i<rect.height; i++)
	{
		if (ptLeft[i].y>-1 && ptLeft[i].x>5)
		{
			contours_left.push_back(ptLeft[i]);
		}
		if (ptRight[i].y>-1 && ptRight[i].x<mat.cols-5)
		{
			contours_right.push_back(ptRight[i]);
		}
	}
	_contours.clear();
	_contours.push_back(contours_left);
	_contours.push_back(contours_right);
	
	if (ptLeft != NULL)
	{
		delete []ptLeft;
		ptLeft = NULL;
	}
	if (ptRight != NULL)
	{
		delete []ptRight;
		ptRight = NULL;
	}

}


//切线
int CImageProcessClass::getTangent(Mat mat, const vector<Point> contours, int n, double &_dRatio, double &_dAngle)
{
	_dAngle = -1;
	int nLen = contours.size();
	if (nLen < mat.rows/2)
	{
		return -1;
	}
	//画一个线段  

	//储存拟合直线的容器  
        Vec4f line;  
	vector<Point> vPoints;
        vPoints.clear();
	int h = nLen/10;
	n = nLen/2;
	double dK = 0;
	//double t1 = (double)cvGetTickCount(); 
	for (int j=n-h; j<n+h; j++)
	{		
		vPoints.push_back(contours.at(j));
#if TEST
		circle(mat,contours.at(j),5,Scalar(0,255,0));
#endif
	}
#if TEST
	circle(mat,contours.at(n),5,Scalar(0,255,0));
	circle(mat,contours.at(n-h),5,Scalar(0,255,0));
	circle(mat,contours.at(n+h),5,Scalar(0,255,0));
#endif
	fitLine(vPoints, line, CV_DIST_L1, 1, 0.001, 0.001);//(line[0],line[1])表示直线的方向向量，(line[2],line[3])表示直线上的一个点。 
	//double t2 = (double)cvGetTickCount();
	//cout<<"拟合时间："<<(t2-t1)/cvGetTickFrequency()/1000<<endl;
	if (line[0]<-0.00000001 || line[0]>0.0000001)
	{
		dK = line[1]/line[0];
	}
	if (dK<0.0000001 && dK>-0.0000001)
	{
		_dAngle = 90;
	}
	else
	{
		_dAngle = atan(dK)*180/CV_PI;
	}
#if TEST
	//cout<<dK<<", ";
	//画一个线段  
	int x0= line[2];  
	int y0= line[3];  
	//int x1= x0-10*line[0];  
	//int y1= y0-10*line[1];  
	int y1 = mat.rows-1;
	int x1 = (y1-y0)*line[0]/line[1] + x0;
	cv::line(mat,Point(x0,y0),cv::Point(x1,y1),cv::Scalar(0),4);  
	
	circle(mat,cvPoint(line[2],line[3]),5,Scalar(255,255,255));
	circle(mat,cvPoint(x1,y1),5,Scalar(255,255,255));

	   // ... and the long enough line to cross the whole image  
	double d = sqrt((double)line[0]*line[0] + (double)line[1]*line[1]);  //line[0 & 1]存储的是单位向量，所以d=1  
	//printf("\n %f\n", d);  
	line[0] /= d;  
	line[1] /= d;  

	//画出线段的两个端点(避免线太短，以线上一个随机点向两侧延伸line[0]*t )  
	float t = (float)(mat.cols + mat.rows) ;  
	CvPoint pt1, pt2;
	pt1.x = cvRound(line[2] - line[0]*t);  
	pt1.y = cvRound(line[3] - line[1]*t);  
	pt2.x = cvRound(line[2] + line[0]*t);  
	pt2.y = cvRound(line[3] + line[1]*t);  
	//cv::line(mat, pt1, pt2, CV_RGB(0,255,0), 3);  
#endif
/*	x0 = contours.at(n).x;
	y0 = contours.at(n).y;
	y1 = mat.rows-1;
	x1 = (y1-y0)*line[0]/line[1] + x0;
	cv::line(mat,Point(x0,y0),cv::Point(x1,y1),cv::Scalar(255,255,255),4);  
*/
	//dAngle = atan(dK);
	/*
	int x0= contours.at(nLen/2).x;  
	int y0= contours.at(nLen/2).y;   
	int y1=  200; 
	int x1= (y1-100+dMean*100)/dMean;  
	cv::line(mat,Point(x0,y0),cv::Point(x1,y1),cv::Scalar(0),4);  */
	return 0;
}

void CImageProcessClass::judgeMaxValueDire(const vector<double> vdValue, double &_dMax)
{
	int nLen = vdValue.size();
	if (nLen < 1)
	{
		_dMax = 0;
		return;
	}

	int nIndex = 0;
	double dMax = 0;
	double dMean = 0;
	double dMin = 1000;
	for (int i=0; i<nLen; i++)
	{
		/*if (dMax < fabs(vdValue.at(i)))
		{
			dMax = fabs(vdValue.at(i));
			nMaxIndex = i;
		}*/
		if (dMin > fabs(vdValue.at(i)))
		{
			dMin = fabs(vdValue.at(i));
			nIndex = i;
		}
		dMean += fabs(vdValue.at(i));
	}

	dMean /= nLen;
	if ((int)vdValue.at(nIndex) > 0)
	{
		_dMax = dMean;//dMax
	}
	else
	{
		_dMax = -1*dMean;//-1*dMax
	}	
}

void CImageProcessClass::testCali(string sPath, const double dWidth, const double dHeight)
{
    Mat frame = imread(sPath.c_str());
    CvRect rect = cvRect(0,0,0,0);
    rect.x = 5;//frame.cols/10;
    rect.width = frame.cols-5;//8*frame.cols/10;
    rect.y = 1*frame.rows/3;
    rect.height = frame.rows/3;
    
    double dPixelX = 0;
    double dPixelY = 0;
    calcCaliData(frame, rect, dPixelX, dPixelY);

    writeCaliData("cali.txt", dPixelX, dPixelY, dWidth, dHeight);

}

void CImageProcessClass::calcCaliData(const Mat src, CvRect rect, double &_dPixelX, double &_dPixelY)
{
        Mat mat;
	getRectImage(src, rect, mat);
	int nRow = mat.rows;
	int nCol = mat.cols;

        Mat matGray;
	if (3 == mat.channels())
	{
	    //cvtColor(mat, mat, CV_BGR2HSV);//CV_BGR2YCrCb
           cvtColor(mat, matGray, CV_BGR2GRAY);
	}

	vector<Mat> mat_channels;  
	//split(mat, mat_channels);  //HSV

	//matGray = mat_channels.at(1); //RGB 2、1、0
//	GaussianBlur(matGray, matGray, Size(3,3), 0, 0); 	
/*	Mat matTemp;
	enhanceRegion(mat, g_dRadius, g_dR, g_dG, g_dB, matTemp);
	imwrite("enhanceRegion.bmp", matTemp);

	if (3 == matTemp.channels())
	{
        cvtColor(matTemp, matGray, CV_BGR2GRAY);
	}
*/
	threshold(matGray, matGray, 0, 255, CV_THRESH_OTSU);
       
   
	if (false)
	{
		
               // float* p; 
               Vec3b* p;
		for (int row=0; row<matGray.rows; row++)
		{
			//p = matGray.ptr<float>(row); 
                        p = matGray.ptr<Vec3b>(row);
			for (int col=0; col<matGray.cols; col++)
			{
				//p[col] = 255.0 - p[col];
                            //matGray.at<float>(row,col) = 255.0 - matGray.at<float>(row,col);
                            p[col][0] = 255 - p[col][0]; 
                            p[col][1] = 255 - p[col][1]; 
                            p[col][2] = 255 - p[col][2]; 
			}
		}

	}
	//Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	//erode(matGray, matGray, element);
	//dilate(matGray, matGray, element);

	//flip(matGray, matGray, 1);//1:flip by y axis，0: flip by x axis, -1: flip by both axises

#if TEST
	imwrite("thresh.bmp", matGray);
#endif	
	//
	CvBox2D boxMax;
	CvBox2D box;
	int nNum = 0;
	double dMaxArea = 0;
	double dArea = 0;
        vector<vector<Point> > contours;
	 //findContours的输入是二值图像
	findContours(matGray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
#if TEST
        cout<<"num: "<<contours.size()<<endl;
#endif
	for(int i=0; i<(int)contours.size(); i++)//将有符号的contours.size()强制转换位非符号的；
	{
	    dArea = contourArea(Mat(contours[i]));
            if (dArea<100)
            {
                continue;
            }

	    box = minAreaRect(Mat(contours[i]));			
	    if (dArea > dMaxArea)
	    {
		dMaxArea = dArea;				
		boxMax = box;//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
		//drawContours(mat,contours_max,-1, Scalar(0,0,0),10); //-1画所有轮廓	
	    }	      
	}
	
	_dPixelX = min(boxMax.size.width, boxMax.size.height);
        _dPixelY = max(boxMax.size.width, boxMax.size.height);
}

void CImageProcessClass::writeCaliData(string sPath, const double dPixelX, const double dPixelY, const double dWidth, const double dHeight)
{
   double dCaliX = dWidth / (dPixelX + 0.000001);
   double dCaliY = dHeight / (dPixelY + 0.000001);

   char buffer[MAX_PATH] = {0};
   CCommonFunClass fun;

   if (!sPath.empty())
   {
	//system("rm );//DeleteFile(sFilePath.c_str());
        remove(sPath.c_str());
   }
		
   sprintf(buffer, "%.4f", dCaliX);
   fun.WriteTxt(sPath.c_str(), buffer);
			
   sprintf(buffer, "%.4f", dCaliY);
   fun.WriteTxt(sPath.c_str(), buffer);

}

void CImageProcessClass::getCaliData(string sFilePath, double &_dCaliX, double &_dCaliY)
{
    
      char sValue[256];
      fstream out;
      char filename[256];  ///////////////////
      int nLen = sFilePath.length();
      for (int i=0; i<nLen; i++)
      {
          filename[i] = sFilePath.at(i);  
      }
      filename[nLen] = '\0';
    //  cout<<filename;//<<endl;

      out.open(filename, ios::in); //sFilePath
      if(out.fail())//文件打开失败:返回0
      {
	 cout<<"Cali file open failed "<<endl;

	 return;
      }
        
      int i = 0;
      while(!out.eof())
      {
	 out.getline(sValue, 256,'\n'); //getline(char *,int,char) 表示该行字符达到256个或遇到换行就结束  
         if (0 == i)
         {  
            _dCaliX = atof(sValue);
cout<<_dCaliX<<endl;
         }
         if (1 == i)
         {  
            _dCaliY = atof(sValue);
cout<<_dCaliY<<endl;
         }
         i++;
      }

}

void CImageProcessClass::testDistortion(void)
{
	CCameraProcess proc;
	CvSize szChessBoard;
	szChessBoard.height = 6;
	szChessBoard.width = 9;
        Mat intrinsics = Mat(3,3,CV_32FC1);
	Mat distortion_coeff = Mat(1,4,CV_32FC1);

/*	int ret = proc.calcDistortionCorrectionParam(g_capture, 20, szChessBoard, intrinsics, distortion_coeff);
	if (ret < 0)
	{
		return;
	}*/
	string s = "distortion.txt";
	proc.distortionData(s, intrinsics, distortion_coeff, true);

	Mat dst;
	Mat src;
	cvNamedWindow("result", 0);
	while(1)
	{
		 src = cvQueryFrame(g_capture);
		 if (src.empty())
		 {
			continue;
		 }

		 proc.distortionCorrection(src, intrinsics, distortion_coeff, dst);

		 //imshow("result", dst);
		 imshow("result", dst);
		 cvWaitKey(10);
	}

//	cvReleaseMat(&intrinsics);
	//cvReleaseMat(&distortion_coeff);
}


int CImageProcessClass::runwayFunOne(const Mat src, int nIndex, CvRect rect, Mat &_dst, CvPoint &_ptStart, CvPoint &_ptEnd, double &_dAngle)
{	
	Mat mat;
	getRectImage(src, rect, mat);
	int nRow = mat.rows;
	int nCol = mat.cols;
	sharpenUSM(mat, mat);

        Mat matGray;
	if (3 == mat.channels())
	{
		cvtColor(mat, mat, CV_BGR2HSV);
        //cvtColor(mat, matGray, CV_BGR2GRAY);
	}
	vector<Mat> mat_channels;  
	split(mat, mat_channels);  //HSV

	matGray = mat_channels.at(1); //RGB 2、1、0
	threshold(matGray, matGray, 0, 255, CV_THRESH_OTSU);
	for(int i=0; i<nRow; i++)
	{
		//matGray.row(i) = 255 - matGray.row(i) + 0;
	}
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	erode(matGray, matGray, element);
	dilate(matGray, matGray, element); //open process
	dilate(matGray, matGray, element);
      //  dilate(matGray, matGray, element);

	imwrite("thresh.bmp", matGray);
	//_dst = matGray;
    
	//return;
	CvRect r = cvRect(0,0,0,0);
	bool isOK = false;
	vector<CvBox2D> vBoxContours;
	vector<CvRect> vRect;
	double dArea = 0;
	CvBox2D box;
	int nNum = 0;
	CvPoint2D32f pt[4];
	vector<vector<Point> > contours;
	vector<vector<Point> > contours_temp;
	int width = 0;
	int height = 0;
	 //findContours的输入是二值图像
	findContours(matGray, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
	for(int i=0; i<(int)contours.size(); i++)//将有符号的contours.size()强制转换位非符号的；
	{
		dArea = contourArea(Mat(contours[i]));
		if (dArea < 50)
		{
			continue;
		}
		r = boundingRect(Mat(contours[i]));	
		box = minAreaRect(Mat(contours[i]));
		width = min(box.size.width, box.size.height);
		height = max(box.size.width, box.size.height);
		isOK = height>nRow*0.5 && width>5 && width<nCol/6;
		if (isOK)
		{
			contours_temp.push_back(contours[i]);	
			vBoxContours.push_back(box);//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
			vRect.push_back(r);
			nNum++;
	            drawContours(mat,contours_temp,-1, Scalar(255,0,0), 2); //-1画所有轮廓	
		}
	} 
       _dst = mat;

	vector<vector<Point> > contours_edge1;
	vector<Point> contours_edge11;
	double dMidX = (nCol+0.0)/2;
	if (nNum >=2) //边缘多余2个边缘，取距离图片x轴最近的2个边
	{		
		distanceSort(contours_temp, vBoxContours, vRect, dMidX);
		int ret = (vBoxContours.at(0).center.x-dMidX)*(vBoxContours.at(1).center.x-dMidX);
		if (ret>0)//说明离图像中心最近两个边缘在同一侧，一般情况都是存在误检的情况
		{
			_dAngle = 90;
               
			nNum = 0;
		}	
	}

        _ptStart.x = matGray.cols/2;
        _ptStart.y = matGray.rows-1;
        _ptEnd.x = _ptStart.x;
        _ptEnd.y = 0;

	nNum = min(nNum,2);
	for (int i=0; i<nNum; i++)
	{
		getEdgeContours(contours_temp.at(i), vRect.at(i), contours_edge11);
		for (int j=0; j<contours_edge11.size(); j++)
		{			
			circle(mat,contours_edge11.at(j),2,Scalar(0,0,0));
		}
		contours_edge1.push_back(contours_edge11);
	}
		
	if (contours_edge1.size() > 0)
	{
  		judgeContoursState(mat, 0, contours_edge1, _dAngle);
                if (fabs(_dAngle) < 5)
                {
                    _dAngle = 90;
                }
	}

	_ptStart.y = mat.rows-1;
	if (nNum > 1)
	{
		_ptStart.x = (vBoxContours.at(0).center.x + vBoxContours.at(1).center.x)/2;
	}
	else if(nNum < 1)
	{
	    _ptStart.x = dMidX;
	}
	else if(1==nNum)
	{
		if (vRect.at(0).x-dMidX > 0) //右边
		{
			_ptStart.x = vBoxContours.at(0).center.x/2;
		}
		else
		{
			_ptStart.x = (mat.cols + vBoxContours.at(0).center.x)/2; 
		}
	}

	if (0 == (int)_dAngle)//表示检测有误，就假设让直走
	{
		_dAngle = 90;
	}
	double dK = tan(CV_PI/180*_dAngle);
	{	
		_ptEnd.y = 0;
		_ptEnd.x = (_ptEnd.y - _ptStart.y)/dK + _ptStart.x;
                _ptEnd.x = min(_ptEnd.x, matGray.cols-1);

		cv::line(mat,_ptStart,_ptEnd,cv::Scalar(0),4); 
	}
//	cout<<"leftUp："<<"(0,0)"<<","<<"rightBottom: "<<"("<<mat.cols-1<<","<<mat.rows-1<<")"<<endl;
//	cout<<"startPoint： "<<"("<<_ptStart.x<<","<<_ptStart.y<<")"<<","<<"endPoint: "<<"("<<_ptEnd.x<<","<<_ptEnd.y<<")"<<endl;
	_dst = mat;


  /*  CvPoint pt1 = cvPoint(vBoxContours.at(0).center.x,vBoxContours.at(0).center.y);
	CvPoint pt2 = cvPoint(vBoxContours.at(1).center.x,vBoxContours.at(1).center.y);
	line(mat, pt1, pt2, Scalar(0,255,0),2);  //跑道边缘中心点连线

	_pt.x = (vBoxContours.at(0).center.x + vBoxContours.at(1).center.x)/2;
	_pt.y = (vBoxContours.at(0).center.y + vBoxContours.at(1).center.y)/2;
	CvPoint pt00 = cvPoint(0,0);
	pt00.x = ((vRect.at(0).x + vRect.at(1).x)/2 + (vRect.at(0).x+vRect.at(0).width + vRect.at(1).x+vRect.at(1).width)/2)/2;
	pt00.y = (vRect.at(0).y + vRect.at(1).y)/2;
	line(mat, _pt, pt00, Scalar(255,255,0),2);  
	_dAngle = (pt00.y-_pt.y)/(pt00.x-_pt.x+0.000001);
	_dAngle = atan(_dAngle);

	cout<<nNum<<","<<_dAngle<<","<<vBoxContours.at(0).angle<<","<<vBoxContours.at(1).angle<<endl;
	_dst = mat;*/
	//imwrite("D:\\rect.bmp", mat);

	return nNum;
}


void CImageProcessClass::distanceSort(vector<vector<Point> > &vContours, vector<CvBox2D> &vBox, vector<CvRect> &vRect, double dMid)
{
	int nLen = min(vBox.size(), vRect.size());
	CvBox2D* pBoxTemp = new CvBox2D[nLen];
	CvRect* pRectTemp = new CvRect[nLen];
	int* nIndex = new int[nLen];
	for (int i=0; i<nLen; i++)
	{
		pBoxTemp[i] = vBox.at(i);
		pRectTemp[i] = vRect.at(i);
		nIndex[i] = i;
	}
	double dis = 0;
	double dMin = 0;
	bool isNegative = false;
	CvBox2D box;
	CvRect r;
	int n = 0;
	for (int i=0; i<nLen-1; i++)
	{
		for (int j=1; j<nLen; j++)
		{
			isNegative = fabs(pBoxTemp[i].center.x - dMid) - fabs(pBoxTemp[j].center.x - dMid);
			if (isNegative)
			{
               box = pBoxTemp[i];
			   pBoxTemp[i] = pBoxTemp[j];
			   pBoxTemp[j] = box;

			   r = pRectTemp[i];
			   pRectTemp[i] = pRectTemp[j];
			   pRectTemp[j] = r;

			   n = nIndex[i];
			   nIndex[i] = nIndex[j];
			   nIndex[j] = n;
			}
		}
	}

	vector<vector<Point> > contours;
	vBox.clear();
	vRect.clear();
	nLen = min(nLen, 2);
	for (int i=0; i<nLen; i++)
	{
		vBox.push_back(pBoxTemp[i]);
		vRect.push_back(pRectTemp[i]);
		contours.push_back(vContours.at(nIndex[i]));
	}

	vContours.clear();
	for (int i=0; i<nLen; i++)
	{
		vContours.push_back(contours.at(i));
	}

	if (pBoxTemp != NULL)
	{
		delete []pBoxTemp;
		pBoxTemp = NULL;
	}
	if (pRectTemp != NULL)
	{
		delete []pRectTemp;
		pRectTemp = NULL;
	}
	if (nIndex != NULL)
	{
		delete []nIndex;
		nIndex = NULL;
	}

}

void CImageProcessClass::getEdgeContours(const vector<Point> contours, const CvRect rect, vector<Point> &_contours)
{
    Point* pt = new Point[2*rect.height];
	int* pNum = new int[2*rect.height];
	for (int i=0; i<rect.height; i++)
	{
		pt[i].x = 0;
		pt[i].y = i; //-1代表这个点未更新
		pNum[i] = 0;
	}
	int nLen = contours.size();
	int row = 0;
	int col = 0;
	for (int i=0; i<nLen; i++)
	{
		row = contours.at(i).y-1;
		col = contours.at(i).x;
		if (row>-1 && row<rect.height)
		{
			pt[row].x += col;
			pNum[row]++;
		}
	}
	_contours.clear();
	for (int i=0; i<rect.height; i++)
	{
        col = 0==pNum[i]? 0 : pt[i].x/pNum[i];
		if (col != 0)
		{
			_contours.push_back(cvPoint(col,i+1));
		}
	}

	if (pt != NULL)
	{
		delete []pt;
		pt = NULL;
	}
	if (pNum != NULL)
	{
		delete []pNum;
		pNum = NULL;
	}
}

