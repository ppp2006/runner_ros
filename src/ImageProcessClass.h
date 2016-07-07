#pragma once
#include <vector>
//#include "opencv2/cv.h"  
//#include "opencv2/highgui.h"  
//#include "opencv2/cxcore.h"  

using namespace cv;  
using namespace std;  

const double MINVALUE = 999999999;
const double EP = 0.00001;

class CImageProcessClass
{
public:
	CImageProcessClass(void);
	~CImageProcessClass(void);
public:
	void testVisualNavigation(void);
	int runwayFunThree(const Mat src, int nIndex, CvRect rect, Mat &_dst, CvPoint &_ptStart, CvPoint &_ptEnd, double &_dAngle);
        int runwayFunOne(const Mat src, int nIndex, CvRect rect, Mat &_dst, CvPoint &_ptStart, CvPoint &_ptEnd, double &_dAngle);

	void oneEdge(vector<CvBox2D> &vBox, vector<CvRect> &vRect, double dMid, CvPoint &_pt, double &_dAngle);
	int getRectImage(const Mat src, CvRect rect, Mat &_dst);
	void getEdgeContours(Mat mat, const vector<Point> contours, const CvRect rect, vector<vector<Point> > &_contours);
	void judgeContoursState(Mat mat, int nIndex, const vector<vector<Point> > contours, double &_dAngle);
	int getTangent(Mat mat, const vector<Point> contours, int n, double &_dRatio, double &_dAngle);
	void judgeMaxValueDire(const vector<double> vdValue, double &_dMax);
        void getAngleDiff(const double dAngle, double &dAngleStart, double &dAngleEnd, bool &isFirst, double &_dAngle);
        void getDistance(const CvPoint ptStart, const CvPoint ptEnd, double dCaliX, double dCaliY, double &_dDis);
       
	void getRegionAverageValue(const Mat src, CvRect rect, double &_dRadius, double &_dR, double &_dG, double &_dB);
	void enhanceRegion(Mat src, const double dRadius, const double dR, const double dG, const double dB, Mat &_dst);

        void sharpenUSM(Mat src, Mat &_dst, double sigma=3, int threshold=0, float amount=1);

	void testDistortion(void);
        
        void calcCaliData(const Mat src, CvRect rect, double &_dPixelX, double &_dPixelY);     
        void writeCaliData(string sPath, const double dPixelX, const double dPixelY, const double dWidth, const double dHeight);
        void getCaliData(string sFilePath, double &_dCaliX, double &_dCaliY);
        void testCali(string sPath, const double dWidth, const double dHeight);

        void distanceSort(vector<vector<Point> > &vContours, vector<CvBox2D> &vBox, vector<CvRect> &vRect, double dMid);
        void getEdgeContours(const vector<Point> contours, const CvRect rect, vector<Point> &_contours);
        
};

