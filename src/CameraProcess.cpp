#include <cv.h>  
#include <highgui.h>  
#include <iostream> 
#include <string> 
#include <fstream>
#include "CameraProcess.h"
#include "CommonFunClass.h"
using namespace cv;
#define MAX_PATH   1000
const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w 4 -h 5 -s 0.025 -o camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w 4 -h 5 -s 0.025 -o camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";


const char* liveCaptureHelp =
    "When the live video from camera is used as input, the following hot-keys may be use\n"
        "  <ESC>, 'q' - quit the program\n"
        "  'g' - start capturing images\n"
        "  'u' - switch undistortion on/off\n";

CCameraProcess::CCameraProcess(void)
{
	 
}


CCameraProcess::~CCameraProcess(void)
{

}

int CCameraProcess::calcDistortionCorrectionParam(CvCapture* capture, int nImageNum,CvSize szChessBoard, CvMat* intrinsics, CvMat* distortion_coeff)
{
	if( !capture )
	{
		fprintf(stderr,"Could not initialize capturing...\n");
		return -1;
	}

	Mat mat, mat2;//IplImage *current_frame_rgb;
	Mat matGray;//IplImage *current_frame_gray;
	Mat matChessBoard;//IplImage *chessBoard_Img;
	
	cvWaitKey(100); 
	mat = cvQueryFrame( capture );
	// cvSaveImage("./src.bmp", current_frame_rgb);

	if (mat.empty())
	{
		fprintf(stderr,"The captured image is empty \n");
		return -1;
	}
	int Thresholdness = 120;
	CvSize szImage;
	szImage.height = mat.rows; //480
	szImage.width = mat.cols; //640

	//const int ChessBoardSize_w = 9;//7;
	//const int ChessBoardSize_h = 6;//7;
	// Calibration stuff
	float 	SquareWidth = 17; //投影实际距离 毫米单位  //21.6f;实际距离 毫米单位 在A4纸上为两厘米
	const   int nPoints = szChessBoard.width*szChessBoard.height;
	CvPoint2D32f* corners = new CvPoint2D32f [nPoints*nImageNum];//CvPoint2D32f corners[nPoints*nImageNum];//
	int* corner_count = new int [nImageNum];////int corner_count[nImageNum] = {0};
	memset(corner_count, 0, nImageNum*sizeof(int));
	int captured_frames = 0;

	CvMat *object_points;
	CvMat *point_counts;
	CvMat *image_points;
	int find_corners_result =0 ;

	//intrinsics = cvCreateMat(3,3,CV_32FC1);
	//distortion_coeff = cvCreateMat(1,4,CV_32FC1);
	rotation_vectors = cvCreateMat(nImageNum,3,CV_32FC1);
	translation_vectors = cvCreateMat(nImageNum,3,CV_32FC1);

	point_counts = cvCreateMat(nImageNum,1,CV_32SC1);

	object_points = cvCreateMat(nImageNum*nPoints,3,CV_32FC1);
	image_points = cvCreateMat(nImageNum*nPoints,2,CV_32FC1);

	// Function to fill in the real-world points of the checkerboard
	_InitCorners3D(object_points, szChessBoard, nImageNum, SquareWidth);

	cvNamedWindow( "Window 0", 0);
	cvCreateTrackbar("Thresholdness","Window 0",&Thresholdness, 255,0);

	while (captured_frames < nImageNum)
	{
		mat = cvQueryFrame( capture );
		//cvSaveImage("./src.bmp", current_frame_rgb);
		if (mat.empty())
		{
			continue;
		}

		mat2 = mat;
		cvtColor(mat, matGray, CV_BGR2GRAY);
                IplImage pImgTemp1 = IplImage(matGray);

		find_corners_result = cvFindChessboardCorners(&pImgTemp1,
			szChessBoard,
			&corners[captured_frames*nPoints],
			&corner_count[captured_frames],0);
                
                IplImage pImgTemp2 = IplImage(mat2);
		cvDrawChessboardCorners(&pImgTemp2, szChessBoard, &corners[captured_frames*nPoints], nPoints, find_corners_result);
		imshow("Window 0",mat2);
	
		if(1 == find_corners_result)
		{
			int c = waitKey(10);
			//按下按键后会停留在当前帧，等待下一次按键
			if (c==32)
			{
				continue;
			}
			else //*///if( c >= 0)		
			{
				waitKey(0);
			}
			//cvWaitKey(10);
			imwrite("./result.bmp", mat2);     
			captured_frames++;
			cout<<captured_frames<<endl;
		}
		
		intrinsics->data.fl[0] = 256.8093262;   //fx		
		intrinsics->data.fl[2] = 160.2826538;   //cx
		intrinsics->data.fl[4] = 254.7511139;   //fy
		intrinsics->data.fl[5] = 127.6264572;   //cy

		intrinsics->data.fl[1] = 0;   
		intrinsics->data.fl[3] = 0;   
		intrinsics->data.fl[6] = 0;   
		intrinsics->data.fl[7] = 0;   
		intrinsics->data.fl[8] = 1;   	

		distortion_coeff->data.fl[0] = -0.193740;  //k1
		distortion_coeff->data.fl[1] = -0.378588;  //k2
		distortion_coeff->data.fl[2] = 0.028980;   //p1
		distortion_coeff->data.fl[3] = 0.008136;   //p2

		cvWaitKey(40);
		find_corners_result = 0;
	}   

	cvSetData( image_points, corners, sizeof(CvPoint2D32f));
	cvSetData( point_counts, corner_count, sizeof(int));

	cout<<"corner_count = ";
	int npstep = point_counts->rows == 1 ? 1 : point_counts->step/CV_ELEM_SIZE(point_counts->type);
	bool isError = false;
	for (int i=0; i<nImageNum; i++)
	{
		cout<<" "<<point_counts->data.i[i*npstep];
		if (point_counts->data.i[i*npstep] < 4)
		{
			isError = true;
			i = nImageNum + 1;
			cout<<"point num is error"<<endl;
		}
	}
	
	cvCalibrateCamera2(object_points,
		image_points,
		point_counts,
		szImage,
		intrinsics,
		distortion_coeff,
		rotation_vectors,
		translation_vectors,
		0);

	if (corners != NULL)
	{
		delete []corners;
		corners = NULL;
	}
	if (corner_count != NULL)
	{
		delete []corner_count;
		corner_count = NULL;
	}

	cvReleaseMat(&rotation_vectors);
	cvReleaseMat(&translation_vectors);

	cvReleaseMat(&point_counts);
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);

	return 0;
}

void CCameraProcess::distortionCorrection(Mat src, Mat &intrinsics, Mat &distortion_coeff,
	                                      Mat &_dst, bool isShowResult)
{
	// [fx 0 cx; 0 fy cy; 0 0 1].
	undistort(src, _dst, intrinsics, distortion_coeff);
	
       
	if (isShowResult)
	{
		float intr[3][3] = {0.0};
		float dist[4] = {0.0};
		float tranv[3] = {0.0};
		float rotv[3] = {0.0};

		for (int i=0; i<3; i++)
		{
			for (int j=0; j<3; j++)
			{
				intr[i][j] = intrinsics.at<float>(j,i);
			}
			dist[i] = distortion_coeff.at<float>(0,i);
		//	tranv[i] = ((float*)(translation_vectors->data.ptr))[i];
		//	rotv[i] = ((float*)(rotation_vectors->data.ptr))[i];
		}
		dist[3] = distortion_coeff.at<float>(0,3);

		printf("-----------------------------------------\n");
		printf("INTRINSIC MATRIX: \n");
		printf("[ %6.4f %6.4f %6.4f ] \n", intr[0][0], intr[0][1], intr[0][2]);
		printf("[ %6.4f %6.4f %6.4f ] \n", intr[1][0], intr[1][1], intr[1][2]);
		printf("[ %6.4f %6.4f %6.4f ] \n", intr[2][0], intr[2][1], intr[2][2]);
		printf("-----------------------------------------\n");
		printf("DISTORTION VECTOR: \n");
		printf("[ %6.4f %6.4f %6.4f %6.4f ] \n", dist[0], dist[1], dist[2], dist[3]);
		printf("-----------------------------------------\n");
		printf("ROTATION VECTOR: \n");
		printf("[ %6.4f %6.4f %6.4f ] \n", rotv[0], rotv[1], rotv[2]);
		printf("TRANSLATION VECTOR: \n");
		printf("[ %6.4f %6.4f %6.4f ] \n", tranv[0], tranv[1], tranv[2]);
		printf("-----------------------------------------\n");

	}
}


void CCameraProcess::distortionData(string sFilePath, Mat &intrinsics, Mat &distortion_coeff, bool isRead)
{
	CCommonFunClass fun;
	int nW1 = intrinsics.cols;
	int nH1 = intrinsics.rows;
	int nW2 = distortion_coeff.cols;
	int nH2 = distortion_coeff.rows;

	char buffer[MAX_PATH] = {0};
	double dValue = 0;
	if (!isRead)
	{
	    if (!sFilePath.empty())
	    {
		//system("rm );//DeleteFile(sFilePath.c_str());
                remove(sFilePath.c_str());
	    }
	    for (int row=0; row<nH1; row++)
	    {
		for (int col=0; col<nW1; col++)
		{
		    dValue = intrinsics.at<float>(row,col);
		    sprintf(buffer, "%.4f", dValue);
		    fun.WriteTxt(sFilePath.c_str(), buffer);
		}
	   }
	   for (int row=0; row<nH2; row++)
	   {
		for (int col=0; col<nW2; col++)
		{
		    dValue = distortion_coeff.at<float>(row,col);
		    sprintf(buffer, "%.4f", dValue);
		    fun.WriteTxt(sFilePath.c_str(), buffer);
		}
	   }
	}
	else
	{
		char sValue[256];
		fstream out;
                char filename[256];  ///////////////////
                int nLen = sFilePath.length();
                for (int i=0; i<nLen; i++)
                {
                    filename[i] = sFilePath.at(i);
                }
                cout << "distortion name "<< filename <<endl;
		out.open("/opt/ros/hydro/catkin_ws/src/track_follower/src/distortion.txt", ios::in); //sFilePath
		if(out.fail())//文件打开失败:返回0
		{
			cout<<"disort File open failed "<<endl;
			return;
		}
        
		int i = 0;
		int col = 0;
		int row = 0;
		while(!out.eof())
		{
			out.getline(sValue, 256,'\n'); //getline(char *,int,char) 表示该行字符达到256个或遇到换行就结束
			if (i<nW1*nH1)
			{
				row = i/nW1;
				col = i%nW1;
				intrinsics.at<float>(row,col) = atof(sValue);
				dValue = intrinsics.at<float>(row,col);
				cout<<dValue<<endl;
			}
			else if (i>=nW1*nH1 && i<(nH1*nW1 + nH2*nW2))
			{
				row = (i - nH1*nW1)/nW2;
				col = (i - nH1*nW1)%nW2;
				distortion_coeff.at<float>(row,col) = atof(sValue);
				dValue = distortion_coeff.at<float>(row,col);
				cout<<dValue<<endl;
			}
			i++;
		}
	}
}

void CCameraProcess::_InitCorners3D(CvMat *Corners3D, CvSize szChessBoard, int nImages, float fSquareSize)
{
	int nCurrentImage = 0;
	int nCurrentRow = 0;
	int nCurrentCol = 0;
	int nPoints = szChessBoard.height * szChessBoard.width;
	float * temppoints = new float[nImages * nPoints * 3];

	// for now, assuming we're row-scanning
	for (nCurrentImage=0; nCurrentImage<nImages; nCurrentImage++)
	{
		for (nCurrentRow=0; nCurrentRow<szChessBoard.height; nCurrentRow++)
		{
			for (nCurrentCol=0; nCurrentCol<szChessBoard.width; nCurrentCol++)
			{
				temppoints[(nCurrentImage*nPoints*3)+(nCurrentRow*szChessBoard.width + nCurrentCol)*3]=(float)nCurrentRow*fSquareSize;
				temppoints[(nCurrentImage*nPoints*3)+(nCurrentRow*szChessBoard.width + nCurrentCol)*3+1]=(float)nCurrentCol*fSquareSize;
				temppoints[(nCurrentImage*nPoints*3)+(nCurrentRow*szChessBoard.width + nCurrentCol)*3+2]=0.f;
			}
		}
	}
	(*Corners3D) = cvMat(nImages*nPoints,3,CV_32FC1, temppoints);
}

void CCameraProcess::_makeChessBoard(IplImage *imgChessBoard, CvSize szChessBoard, CvSize szImage)
{
 
	CvScalar e; 
	e.val[0] =255;
	e.val[1] =255;
	e.val[2] =255;
	cvSet(imgChessBoard,e,0);
	for(int i=0; i<szChessBoard.width+1; i++)
	{
		for(int j=0; j<szChessBoard.height+1; j++)
		{
			int w =(szImage.width)/2/(szChessBoard.width);
			int h = w; //(image_height)/2/(ChessBoardSize.height);

			int ii = i+1;
			int iii = ii+1;
			int jj =j+1;
			int jjj =jj+1;
			int s_x = szImage.width/6;		  

			if((i+j)%2==1)
			{
				cvRectangle(imgChessBoard, cvPoint(w*i+s_x,h*j+s_x),cvPoint(w*ii-1+s_x,h*jj-1+s_x), CV_RGB(0,0,0),CV_FILLED, 8, 0);
			}
		}
	}

}
 
