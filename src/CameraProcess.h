#pragma once
#include <cv.h>  
#include <highgui.h>  
#include <iostream> 
#include <string> 
using namespace std;
using namespace cv;

class CCameraProcess
{
public:
	CCameraProcess(void);
	~CCameraProcess(void);

public:
	//CvCapture* m_capture;// = 0;//cvCreateCameraCapture(0); //cvReleaseCapture
	int calcDistortionCorrectionParam(CvCapture* capture, int nImageNum,CvSize szChessBoard, CvMat* intrinsics, CvMat* distortion_coeff);
	void distortionCorrection(Mat src, Mat &intrinsics, Mat &distortion_coeff, Mat &_dst, bool isShowResult=false);

	void distortionData(string sFilePath, Mat &intrinsics, Mat &distortion_coeff, bool isRead=false);
private:
	void _InitCorners3D(CvMat *Corners3D, CvSize szChessBoard, int nImages, float fSquareSize);
	void _makeChessBoard(IplImage *imgChessBoard, CvSize szChessBoard, CvSize szImage);

private:
	CvMat *rotation_vectors;
	CvMat *translation_vectors;
};

