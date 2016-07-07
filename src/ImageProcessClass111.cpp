#include "StdAfx.h"
#include "ImageProcessClass.h"
#include "CommonFunClass.h"
#include "CameraProcess.h"

#define	round(x)	((x-floor(x))>0.5 ? ceil(x) : floor(x))
#define eps 0.0001

#define TEST 1

//图像的像素直接提取
#define        _I(img,x,y) ((unsigned char*)((img)->imageData + (img)->widthStep*(y)))[(x)]
//亚像素级灰度值
#define        _IF(image,x,y)    ( ((int)(x+1)-(x))*((int)(y+1)-(y))*_I((image),(int)(x),(int)(y)) + ((int)(x+1)-(x))*((y)-(int)(y))*_I((image),(int)(x),(int)(y+1)) + ((x)-(int)(x))*((int)(y+1)-(y))*_I((image),(int)(x+1),(int)(y)) + ((x)-(int)(x))*((y)-(int)(y))*_I((image),(int)(x+1),(int)(y+1)) )//插值后的像素值(IN表示interpolation),x、y可以为小数

CvCapture* g_capture = 0; 
CImageProcessClass::CImageProcessClass(void)
{
	if (!mclInitializeApplication(NULL,0))
	{
		return;
	}
	g_capture = cvCreateCameraCapture(0);
}


CImageProcessClass::~CImageProcessClass(void)
{
    mclTerminateApplication();
	cvReleaseCapture(&g_capture);
}

//
// 二维离散小波变换（单通道浮点图像）
void CImageProcessClass::DWT(IplImage *pImage, int nLayer)
{
	// 执行条件
	if (pImage)
	{
		if (pImage->nChannels == 1 &&
			pImage->depth == IPL_DEPTH_8U &&
			((pImage->width >> nLayer) << nLayer) == pImage->width &&
			((pImage->height >> nLayer) << nLayer) == pImage->height)
		{
			int     i, x, y, n;
			float   fValue   = 0;
			float   fRadius  = sqrt(2.0f);
			int     nWidth   = pImage->width;
			int     nHeight  = pImage->height;
			int     nHalfW   = nWidth / 2;
			int     nHalfH   = nHeight / 2;
			float **pData    = new float*[pImage->height];
			float  *pRow     = new float[pImage->width];
			float  *pColumn  = new float[pImage->height];
			for (i = 0; i < pImage->height; i++)
			{
				pData[i] = (float*) (pImage->imageData + pImage->widthStep * i);
			}
			/* 多层小波变换*/
			for (n = 0; n < nLayer; n++, nWidth /= 2, nHeight /= 2, nHalfW /= 2, nHalfH /= 2)
			{
				// 水平变换
				for (y = 0; y < nHeight; y++)
				{
					/* 奇偶分离*/
					memcpy(pRow, pData[y], sizeof(float) * nWidth);
					for (i = 0; i < nHalfW; i++)
					{
						x = i * 2;
						pData[y][i] = pRow[x];
						pData[y][nHalfW + i] = pRow[x + 1];
					}
					/* 提升小波变换*/
					for (i = 0; i < nHalfW - 1; i++)
					{
						fValue = (pData[y][i] + pData[y][i + 1]) / 2;
						pData[y][nHalfW + i] -= fValue;
					}
					fValue = (pData[y][nHalfW - 1] + pData[y][nHalfW - 2]) / 2;
					pData[y][nWidth - 1] -= fValue;
					fValue = (pData[y][nHalfW] + pData[y][nHalfW + 1]) / 4;
					pData[y][0] += fValue;
					for (i = 1; i < nHalfW; i++)
					{
						fValue = (pData[y][nHalfW + i] + pData[y][nHalfW + i - 1]) / 4;
						pData[y][i] += fValue;
					}
					// 频带系数
					for (i = 0; i < nHalfW; i++)
					{
						pData[y][i] *= fRadius;
						pData[y][nHalfW + i] /= fRadius;
					}
				}
				// 垂直变换
				for (x = 0; x < nWidth; x++)
				{
					// 奇偶分离
					for (i = 0; i < nHalfH; i++)
					{
						y = i * 2;
						pColumn[i] = pData[y][x];
						pColumn[nHalfH + i] = pData[y + 1][x];
					}
					for (i = 0; i < nHeight; i++)
					{
						pData[i][x] = pColumn[i];
					}
					// 提升小波变换
					for (i = 0; i < nHalfH - 1; i++)
					{
						fValue = (pData[i][x] + pData[i + 1][x]) / 2;
						pData[nHalfH + i][x] -= fValue;
					}
					fValue = (pData[nHalfH - 1][x] + pData[nHalfH - 2][x]) / 2;
					pData[nHeight - 1][x] -= fValue;
					fValue = (pData[nHalfH][x] + pData[nHalfH + 1][x]) / 4;
					pData[0][x] += fValue;
					for (i = 1; i < nHalfH; i++)
					{
						fValue = (pData[nHalfH + i][x] + pData[nHalfH + i - 1][x]) / 4;
						pData[i][x] += fValue;
					}
					// 频带系数
					for (i = 0; i < nHalfH; i++)
					{
						pData[i][x] *= fRadius;
						pData[nHalfH + i][x] /= fRadius;
					}
				}
			}
		
			delete[] pData;	
			delete[] pRow;
			delete[] pColumn;
		}
	}
}

// 二维离散小波恢复（单通道浮点图像）
void CImageProcessClass::IDWT(IplImage *pImage, int nLayer)
{
   // 执行条件
   if (pImage)
   {
      if (pImage->nChannels == 1 &&
         pImage->depth == IPL_DEPTH_8U &&
         ((pImage->width >> nLayer) << nLayer) == pImage->width &&
         ((pImage->height >> nLayer) << nLayer) == pImage->height)
      {
         int     i, x, y, n;
         float   fValue   = 0;
         float   fRadius  = sqrt(2.0f);
         int     nWidth   = pImage->width >> (nLayer - 1);
         int     nHeight  = pImage->height >> (nLayer - 1);
         int     nHalfW   = nWidth / 2;
         int     nHalfH   = nHeight / 2;
         float **pData    = new float*[pImage->height];
         float  *pRow     = new float[pImage->width];
         float  *pColumn  = new float[pImage->height];
         for (i = 0; i < pImage->height; i++)
         {
            pData[i] = (float*) (pImage->imageData + pImage->widthStep * i);
         }
         // 多层小波恢复
         for (n = 0; n < nLayer; n++, nWidth *= 2, nHeight *= 2, nHalfW *= 2, nHalfH *= 2)
         {
            // 垂直恢复
            for (x = 0; x < nWidth; x++)
            {
               // 频带系数
               for (i = 0; i < nHalfH; i++)
               {
                  pData[i][x] /= fRadius;
                  pData[nHalfH + i][x] *= fRadius;
               }
               // 提升小波恢复
               fValue = (pData[nHalfH][x] + pData[nHalfH + 1][x]) / 4;
               pData[0][x] -= fValue;
               for (i = 1; i < nHalfH; i++)
               {
                  fValue = (pData[nHalfH + i][x] + pData[nHalfH + i - 1][x]) / 4;
                  pData[i][x] -= fValue;
               }
               for (i = 0; i < nHalfH - 1; i++)
               {
                  fValue = (pData[i][x] + pData[i + 1][x]) / 2;
                  pData[nHalfH + i][x] += fValue;
               }
               fValue = (pData[nHalfH - 1][x] + pData[nHalfH - 2][x]) / 2;
               pData[nHeight - 1][x] += fValue;
               // 奇偶合并
               for (i = 0; i < nHalfH; i++)
               {
                  y = i * 2;
                  pColumn[y] = pData[i][x];
                  pColumn[y + 1] = pData[nHalfH + i][x];
               }
               for (i = 0; i < nHeight; i++)
               {
                  pData[i][x] = pColumn[i];
               }
            }
            // 水平恢复
            for (y = 0; y < nHeight; y++)
            {
               // 频带系数
               for (i = 0; i < nHalfW; i++)
               {
                  pData[y][i] /= fRadius;
                  pData[y][nHalfW + i] *= fRadius;
               }
               // 提升小波恢复
               fValue = (pData[y][nHalfW] + pData[y][nHalfW + 1]) / 4;
               pData[y][0] -= fValue;
               for (i = 1; i < nHalfW; i++)
               {
                  fValue = (pData[y][nHalfW + i] + pData[y][nHalfW + i - 1]) / 4;
                  pData[y][i] -= fValue;
               }
               for (i = 0; i < nHalfW - 1; i++)
               {
                  fValue = (pData[y][i] + pData[y][i + 1]) / 2;
                  pData[y][nHalfW + i] += fValue;
               }
               fValue = (pData[y][nHalfW - 1] + pData[y][nHalfW - 2]) / 2;
               pData[y][nWidth - 1] += fValue;
               // 奇偶合并
               for (i = 0; i < nHalfW; i++)
               {
                  x = i * 2;
                  pRow[x] = pData[y][i];
                  pRow[x + 1] = pData[y][nHalfW + i];
               }
               memcpy(pData[y], pRow, sizeof(float) * nWidth);
            }
         }
         delete[] pData;
         delete[] pRow;
         delete[] pColumn;
      }
   }
}

/**********************************************************************
*函数功能：灰度图像腐蚀
*参数： R模板半径
*返回值：无
***********************************************************************/
void CImageProcessClass::GrayErode(IplImage *pImgSrc, int R, IplImage *pImgDst)
{
	if (pImgSrc->depth != 8 )
	{
		return;
	}

	IplImage* pImgTemp = cvCreateImage(cvGetSize(pImgSrc), 8, 1);
	cvCopy(pImgSrc, pImgDst);
	cvCopy(pImgSrc, pImgTemp);

	int nHeight = pImgSrc->height;
	int nWidth = pImgSrc->width;
	int i,j,n,m;
	double dResult,dValue;
	double dMean = 0;
	double dMin = 255;
	double dMax = 0;
	for (i=R; i<nHeight-R; i++)
    {
		for (j=R; j<nWidth-R; j++) 
		{
			
			dValue = pImgSrc->imageData[pImgSrc->widthStep * i + j];
			//
			dMean = 0;
			dMin = 255;
			dMax = 0;
			for (n=-R; n<=R; n++)
			{
				dResult = pImgSrc->imageData[pImgSrc->widthStep*(i + n) + j];
				dMean += dResult;	
				dMin = dResult<dMin? dResult : dMin;
				dMax = dResult>dMax? dResult : dMax;
			}
			dMean /= R*R;
			for (n=-R; n<=R; n++)
			{
				dResult = pImgSrc->imageData[pImgSrc->widthStep*(i + n) + j];
				dValue = dResult<dMean? dResult : dValue;
			//	dValue = dResult<dMean? dMin : dMax;	    
			}
			pImgTemp->imageData[pImgTemp->widthStep * i + j] = (char)dValue;
		}				
	}
	for (i=R; i<nHeight; i++)
    {
		for (j=R; j<nWidth; j++)
		{			
			dValue = pImgTemp->imageData[pImgTemp->widthStep * i + j];
			dMean = 0;
			dMax = 0;
			dMin = 255;
			for (m=-R; m<=R; m++)
			{
				dResult = pImgTemp->imageData[pImgTemp->widthStep * i + j + m];	
				dMean += dResult;	
				dMin = dResult<dMin? dResult : dMin;
				dMax = dResult>dMax? dResult : dMax;
			}
			dMean /= R*R;
			for (m=-R; m<=R; m++)
			{
				dResult = pImgTemp->imageData[pImgTemp->widthStep * i + j + m];	
				dValue = dResult<dMean? dResult : dValue;
				//dValue = dResult<dMean? dMin : dMax;

			}
			pImgDst->imageData[pImgDst->widthStep * i + j] = (char)dValue;
		}				
	}
	
	cvReleaseImage(&pImgTemp);
}

void CImageProcessClass::MeanFilter(IplImage* pImgSrc, int R, IplImage* _pImgDst)
{
	int nHeight = pImgSrc->height;
	int nWidth = pImgSrc->width;
	int i,j, m, n;
	double dValue = 0;

	//均值滤波
	for(i=R; i<nHeight-R; i++)
	{
		for(j=R; j<nWidth-R; j++)
		{
			dValue = 0;
			for(m=0; m<=R; m++)
			{
				for(n=0; n<=2; n++)
				{
					dValue += pImgSrc->imageData[pImgSrc->widthStep * (i+m) + (j+n)];
				}
			}
			_pImgDst->imageData[_pImgDst->widthStep * i + j] = (char)(dValue / 9);		
		}
	}

}

void filter(double**source,int m_nWidth,int m_nHeight)
{
	int i,j;
	double **temp;
	//申请一个临时二维数组
	temp = new  double*[m_nHeight+2];
	for(i=0;i <=m_nHeight+1; i++)
		temp[i] = new double[m_nWidth+2];
	
	//边界均设为0
	for(i=0; i<=m_nHeight+1; i++)
	{
		temp[i][0] = 0;
		temp[i][m_nWidth+1] = 0;
	}
	for(j=0; j<=m_nWidth+1; j++)
	{
		temp[0][j] = 0;
		temp[m_nHeight+1][j] = 0;
	}

	//将原数组的值赋予临时数组
	for(i=0; i<m_nHeight; i++)
		for(j=0; j<m_nWidth; j++)
			temp[i+1][j+1] = source[i][j];

	//均值滤波
	for(i=0; i<m_nHeight; i++)
	{
		for(j=0; j<m_nWidth; j++)
		{
			source[i][j] = 0;
			for(int k=0;k<=2;k++)
			for(int l=0;l<=2;l++)
			source[i][j] += temp[i+k][j+l];
		
			source[i][j] /= 9;
		}
	}

	if(temp!=NULL)
	{
		for(int i=0;i<=m_nHeight+1;i++)
			if(temp[i]!=NULL) delete temp[i];
		delete temp;
	}
}

bool CImageProcessClass::CalBinary(IplImage* pImgSrc, IplImage* _pImgDst)
{
	int i,j;
	int nHeight = pImgSrc->height;
	int nWidth = pImgSrc->width;
	BYTE **temp;
	temp = new  BYTE * [nHeight];
	for(i=0;i <nHeight; i++)
	{
		temp[i] = new BYTE[nWidth];	
		for(j=0; j<nWidth; j++) 
			temp[i][j] = 0;
	}

	double min = 10000000000000000.0;
	int index = -1;
	double dValue = 0.0;
	for(int k=5;k>=0;k--)
	{
		double sum = 0;
		for(i=0; i<nHeight; i++)
		for(j=0; j<nWidth; j++)
		{
			dValue = pImgSrc->imageData[pImgSrc->widthStep * i + j];
			dValue /= 255;
			if(dValue > k*0.1 + 0.05)
				 _pImgDst->imageData[_pImgDst->widthStep * i + j] = 1;
			else
				 _pImgDst->imageData[_pImgDst->widthStep * i + j] = 0;
			sum += ( _pImgDst->imageData[_pImgDst->widthStep * i + j] - temp[i][j]);
		}
		
		if(sum < min)
		{
			min = sum;
			index = 6-k;
		}

	    for(i=0; i<nHeight; i++)
	    for(j=0; j<nWidth; j++)
		temp[i][j] =  _pImgDst->imageData[_pImgDst->widthStep * i + j];
	}

	double optimalThreshold = (7-index)*0.1;
	for(i=0; i<nHeight; i++)
	for(j=0; j<nWidth; j++)
	{
		if(pImgSrc->imageData[pImgSrc->widthStep * i + j] > optimalThreshold)
			 _pImgDst->imageData[_pImgDst->widthStep * i + j] = 0;
		else
			 _pImgDst->imageData[_pImgDst->widthStep * i + j] = (char)255;
	}

	if(temp!=NULL)
	{
		for(int i=0;i<=nHeight-1;i++)
			if(temp[i]!=NULL) delete temp[i];
		delete temp;
	}


	return true;
}
void CImageProcessClass::CalCbCr(int R, int G, int B, double &_dCr, double &_dCb)
{	
	_dCb =( 128 - 37.797 * R/255 - 74.203 * G/255 +   112 * B/255);
	_dCr =( 128 + 112    * R/255 - 93.786 * G/255 -18.214 * B/255);
}
/**********************************************************************************
*函数功能：计算彩色图像的相似度
*参数： 
IplImage *pImgSrc： 输入，彩色图像
IplImage* _pImgDst: 输出，灰度图 
*返回值：无
***********************************************************************************/
void CImageProcessClass::CalcSimilarity(IplImage *pImgSrc, IplImage* _pImgDst)
{
	if (pImgSrc->nChannels != 3)
	{
		cvZero(_pImgDst);
		return;
	}
	int nHeight = pImgSrc->height;
	int nWidth = pImgSrc->width;

	int i = 0, j = 0;
	double dCr = 0, dCb = 0;
	double x1 = 0, x2 = 0;
	CvScalar value; //b,g,r 
	double t = 0;

	//赋值
	double dBMean = 117.4361;
 	double dRMean = 156.5599;
	double dBRCov[2][2] = {{160.1301,12.1430}, {12.1430, 299.4574}};
	
	double **temp;
	//申请一个临时二维数组
	temp = new  double*[nHeight];
	for(i=0;i <nHeight; i++)
	{
		temp[i] = new double[nWidth];
	}

	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{		
			temp[i][j] = 0;
		}
	}
//	cvCvtColor(pImgSrc, pImgSrc, CV_BGR2YCrCb);

	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{		
			value = cvGet2D(pImgSrc, i, j);
			CalCbCr((int)value.val[2], (int)value.val[1], (int)value.val[0], dCr, dCb);
		//	GetNewYCrCb(value.val[2], value.val[1], value.val[0], dCr, dCb);
			x1 = dCb - dBMean;
			x2 = dCr - dRMean;
		
			t = x1 * (x1*dBRCov[1][1] - x2*dBRCov[1][0]) + x2 * (-x1*dBRCov[0][1] + x2*dBRCov[0][0]);
			t /= (dBRCov[0][0] * dBRCov[1][1] - dBRCov[0][1] * dBRCov[1][0]);
			t /= (-2);
		
			temp[i][j] = exp(t);
		}
	}

	filter(temp, nWidth, nHeight);

	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{		
			 _pImgDst->imageData[_pImgDst->widthStep * i + j] = (char)temp[i][j] ;
		}
	}

	double max = 0.0;
	double dValue = 0;
	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{
			dValue = temp[i][j];//_pImgDst->imageData[_pImgDst->widthStep * i + j];
			if(dValue > max) 
			{
				max = dValue;
			}
		}
	}
	
	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{
			dValue = temp[i][j]; //_pImgDst->imageData[_pImgDst->widthStep * i + j];
			dValue /= max;
			dValue *= 255;
			_pImgDst->imageData[_pImgDst->widthStep * i + j] = (char)dValue;
		}
	}

	if(temp!=NULL)
	{
		for(int i=0;i<nHeight;i++)
			if(temp[i]!=NULL) delete temp[i];
		delete temp;
	}

}

int CImageProcessClass::_BasicGlobalThreshold(int *iHist, int start, int end, int &_iGlobalThres)
{ 
	int t = 0;
	double u = 0;
	int i = 0;
	for (int i=start; i<end; i++)
	{
		t += iHist[i];		
		u += i*iHist[i];
	}
	
	double k2 = (double)(u/t);                     
	double k1 = 0;
	double t1 = 0;
	double t2 = 0;
	double u1 = 0;
	double u2 = 0;
	
	do
	{
		k1 = k2;
		t1 = 0;    
		u1 = 0;
		for (i=start; i<=k1; i++)
		{
			t1 += iHist[i];	
			u1 += i*iHist[i];
		}
		t2 = t - t1;
		u2 = u - u1;
		u1 = t1>0 ? u1/t1 : 0;                
		u2 = t2>0 ? u2/t2 : 0;                
		k2 = (int) ((u1+u2)/2);               
	}
	while(fabs(k1-k2)>1);                          
	_iGlobalThres = (int)k1;
	
	return 0; 
}

int CImageProcessClass::GetBasicGlobalThreshold(IplImage *pImgGray, IplImage* _pImgBinary)
{
    int pg[256] = {0};
	int thre=0;
	for (int i=0; i<pImgGray->imageSize; i++)      //  直方图统计
	{
		pg[(BYTE)pImgGray->imageData[i]]++;
	}

	_BasicGlobalThreshold(pg, 0, 256, thre);   //确定阈值
	cvThreshold(pImgGray, _pImgBinary, thre, 255, CV_THRESH_BINARY);  //二值化

	return thre;
}
/**********************************************************************************
*函数功能：计算二值图像的人脸图像区域
*参数： 
IplImage *pImgSrc： 输入，二值图像
CvRect &_rect:      输出，人脸图像区域
*返回值：无
***********************************************************************************/
void CImageProcessClass::GetFaceRect(IplImage* pImgSrc, CvRect &_rect, int nMinSize, int nMaxSize)
{
	if (pImgSrc->nChannels != 1)
	{
		return;
	}
	
	IplImage* pImgSeq = cvCreateImage(cvGetSize(pImgSrc), 8, 1);
	cvCopy(pImgSrc, pImgSeq);
	cvSaveImage("D:\\test.bmp", pImgSeq);
	//cvErode(pImgSeq, pImgSeq);
	cvDilate(pImgSeq, pImgSeq);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours = NULL;
	double dArea = 0;
	double dMaxArea = 100;
	CvRect rcMax =  cvRect(0, 0, pImgSrc->width, pImgSrc->height);

	int n = cvFindContours(pImgSeq, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	for (int i=0; contours!=0; contours=contours->h_next,i++) 
	{
		dArea = fabs(cvContourArea(contours, CV_WHOLE_SEQ));
		if(dArea > dMaxArea )
		{
            dMaxArea = dArea;
			rcMax = cvBoundingRect(contours);
		}
	}

	int nMin = __min(rcMax.width, rcMax.height);
	int nMax = __max(rcMax.width, rcMax.height);
	if (nMin>nMinSize && nMax<nMaxSize)
	{
		_rect = rcMax;
	}

	cvReleaseMemStorage(&storage);
	cvReleaseImage(&pImgSeq);

}

#define KLow  125
#define KHigh 188
#define MinY  16
#define MaxY  235
#define WCb   46.97
#define WCr   38.76
#define WLCb  23
#define WLCr  20  
#define WHCb  14
#define WHCr  10

void CImageProcessClass::GetNewYCrCb(double dY, double dCr, double dCb, double &_dCr, double &_dCb)
{
	double dCrTemp = 0;
	double dCbTemp = 0;
	double dWCb = 0;
	double dWCr = 0;

	if (dY < KLow)
	{
		dCbTemp = 108 + 10*(KLow-dY) /(KLow-MinY);
		dCrTemp = 154 + 10*(KLow-dY) /(KLow-MinY);

		dWCb = WLCb + (dY-MinY)*(WCb-WLCb)/(KLow-MinY);
        dWCr = WLCr + (dY-MinY)*(WCb-WLCb)/(KLow-MinY);
	}
	else if (dY > KHigh)
	{
		dCbTemp = 108 + 10*(dY-KHigh)/(KHigh-MaxY);
		dCrTemp = 154 + 22*(dY-KHigh)/(KHigh-MaxY);

		dWCb = WHCb + (MaxY-dY)*(WCb-WHCb)/(KHigh-MaxY);
		dWCr = WHCr + (MaxY-dY)*(WCr-WHCr)/(KHigh-MaxY);
	}


	if (dY>=KLow && dY<=KHigh)
	{
		_dCb = dCb;
		_dCr = dCr;	
	}
	else if (dY < KLow)
	{
       _dCb = (dCb - dCbTemp)*WCb/dWCb + dCbTemp;
	   _dCr = (dCr - dCrTemp)*WCr/dWCr + dCrTemp;
	}

}

void CImageProcessClass::DoLOG(IplImage* pImgSrc, CvRect rect, IplImage* _pImgDst)
{
	if (pImgSrc->nChannels!=3 || _pImgDst->nChannels!=1)
	{
		return;
	}
	
	int i,j;
	int nHeight = pImgSrc->height;
	int nWidth = pImgSrc->width;
	int top = rect.y;
	int bottom = rect.y + rect.height;
	int left = rect.x;
	int right = rect.x + rect.width;
	CvScalar value;

    RGBQUAD **target;
	target = new RGBQUAD*[nHeight];
	double **result;					 
	result = new double*[nHeight];
	for(int i=0; i<nHeight; i++)
	{
		result[i] = new double[nWidth];
		target[i] = new RGBQUAD[nWidth];
		for(j=0; j<nWidth; j++)
		{
			value = cvGet2D(pImgSrc, i, j);
			result[i][j] = value.val[2];
		}
	}

	double r,g,temp;
	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{
			value = cvGet2D(pImgSrc, i, j);
			temp = value.val[0] + value.val[1] + value.val[2];
			r = (double)value.val[2]/temp;
			g = (double)value.val[1]/temp;
			if (g<0.398 && g>0.246 && r<0.664 && r>0.233 && r>g && g>=0.5*(1-r))
			{
				 target[i][j].rgbRed = 255;  //face
			}
			else 
			{
				target[i][j].rgbRed = 0;
			}
		}
	}
	
	for(i=top+2; i<bottom-2; i++)
	{
		for(j=left+2; j<right-2; j++)
		{
			result[i][j] =  
				(0-2.0/24.0)*((unsigned char)target[i-2][j-2].rgbRed) + 
				(0-4.0/24.0)*((unsigned char)target[i-2][j-1].rgbRed) + 
				(0-4.0/24.0)*((unsigned char)target[i-2][j].rgbRed)   +
				(0-4.0/24.0)*((unsigned char)target[i-2][j+1].rgbRed) +
				(0-2.0/24.0)*((unsigned char)target[i-2][j+2].rgbRed) +
				(0-4.0/24.0)*((unsigned char)target[i-1][j-2].rgbRed) + 
				(8.0/24.0)  *((unsigned char)target[i-1][j].rgbRed)   +
				(0-4.0/24.0)*((unsigned char)target[i-1][j+2].rgbRed) +
				(0-4.0/24.0)*((unsigned char)target[i][j-2].rgbRed)   + 
				(8.0/24.0)  *((unsigned char)target[i][j-1].rgbRed)   + 
				(1.0)       *((unsigned char)target[i][j].rgbRed)     +
				(8.0/24.0)  *((unsigned char)target[i][j+1].rgbRed)   +
				(0-4.0/24.0)*((unsigned char)target[i][j+2].rgbRed)   +
				(0-4.0/24.0)*((unsigned char)target[i+1][j-2].rgbRed) + 
				(8.0/24.0)  *((unsigned char)target[i+1][j].rgbRed)   +
				(0-4.0/24.0)*((unsigned char)target[i+1][j+2].rgbRed) +
				(0-2.0/24.0)*((unsigned char)target[i+2][j-2].rgbRed) + 
				(0-4.0/24.0)*((unsigned char)target[i+2][j-1].rgbRed) + 
				(0-4.0/24.0)*((unsigned char)target[i+2][j].rgbRed)   +
				(0-4.0/24.0)*((unsigned char)target[i+2][j+1].rgbRed) +
				(0-2.0/24.0)*((unsigned char)target[i+2][j+2].rgbRed);
		}
	}

	for(i=0; i<nHeight; i++)
	{
		for(j=0; j<nWidth; j++)
		{
			target[i][j].rgbRed =  target[i][j].rgbBlue = target[i][j].rgbGreen = 255;
		}
	}

    int positive = 0;   
	int negtive  = 0;
	for(i=top+1; i<bottom-1; i++)
	{
		for(j=left+1; j<right-1; j++)
		{
			positive = 0;   
	         negtive  = 0;
			for(int m=-1;m<=1;m++)
			{
				for(int n=-1;n<=1;n++)
				{
					if(m!=0 || n!=0)
					{
						if(result[i+m][j+n]<-5)
						{
							negtive++;
						}
						if(result[i+m][j+n]>=5)
						{
							positive++;
						}
					}
					if(positive>2 && negtive>2) 
					{
						target[i][j].rgbBlue = target[i][j].rgbGreen = target[i][j].rgbRed = 0;
					}
				}
			}
		}
	}

	for (i=0; i<nHeight; i++)
	{
		for (j=0; j<nWidth; j++)
		{
			_pImgDst->imageData[_pImgDst->widthStep * i + j] = target[i][j].rgbRed;
		}
	}

	if (result != NULL)
	{
		for (int i=0 ;i<nHeight;i++)
			if(result[i] !=NULL ) delete result[i];
		delete result;
	}
	 
	if (target != NULL)
	{
		for (int i=0 ;i<nHeight;i++)
			if(target[i] !=NULL ) delete target[i];
		delete target;
	}
}

/*************************************************************************
函数：判断某区域是否是肤色
参数：
输入：
IplImage* pImgSrc：彩色图
vector<Rect> vRect：检测区域
输出：
vector<bool> &_vIsSkinColour：区域是否为肤色
**************************************************************************/
void CImageProcessClass::IsSkinColour(IplImage* pImgSrc, vector<CvRect> vRect, vector<bool> &_vIsSkinColour)
{
	if (pImgSrc->nChannels != 3)
	{
		return;
	}

	int nWidth = pImgSrc->width;
	int nHeight = pImgSrc->height;

	int nNum = vRect.size();
	CvRect rc;
	CvRect rect = cvRect(0, 0, nWidth, nHeight);
	CCommonFunClass cFun;
	double dSum = 0;
	int i = 0, j = 0;
	double dValue = 0; 
	CvScalar value;
	double dCr, dCb;
	for (int k=0; k<nNum; k++)
	{
		//扣取rect的图像
		rc = cvRect(vRect.at(k).x, vRect.at(k).y, vRect.at(k).width, vRect.at(k).height);
		dSum = 0;
		for (i=rc.x; i<rc.x+rc.width; i++)
		{
			for (j=rc.y; j<rc.y+rc.height; j++)
			{
				value = cvGet2D(pImgSrc, j, i);
				CalCbCr((int)value.val[2], (int)value.val[1], (int)value.val[0], dCr, dCb);

				if ((dCr>=133 && dCr<=173) && (dCb>=77 && dCb<=127))
				{
					dSum++;
				}
	        }
		}
		if (dSum > rc.width*rc.height*0.4)
		{
			_vIsSkinColour.push_back(true);
		}
		else
		{
			_vIsSkinColour.push_back(false);
		}

	}
}

/*****************************************************************
函数：mxArray2IplImage8U3、IplImage2mxArray8U3
功能：mxArray 与 IplImage转换，在C++中多维矩阵是按行存放，而Matlab中是按列存放
备注：彩色图像
******************************************************************/
IplImage* CImageProcessClass::mxArray2IplImage8U3(const mxArray* mx)
{
	int h,w,sz;
	h = mxGetM(mx);
	w = mxGetN(mx);
	w = w/3;
	sz = h*w;

	double* img = (double*)mxGetData(mx);
	IplImage* im = cvCreateImage(cvSize(w,h),IPL_DEPTH_8U,3);
	int step = im->widthStep/sizeof(uchar);
	uchar* imdata = (uchar* )im->imageData;
	for(int i=0;i<h;i++)
	{
		for(int j=0;j<w;j++)
		{
			imdata[i*step+j*3+2] = (uchar)img[i+j*h];		
			imdata[i*step+j*3+1] = (uchar)img[i+j*h+sz];
			imdata[i*step+j*3+0] = (uchar)img[i+j*h+2*sz];
		}
	}

	return im; 	
}

mxArray* CImageProcessClass::IplImage2mxArray8U3(IplImage* pImgSrc)
{ 
	int h,w,sz;
	h = pImgSrc->height;
	w = pImgSrc->width;
	sz = h*w;
	mwSize dims[3] = {h, w, 3};

	mxArray* mx = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
	double* img = (double*)mxGetData(mx);

	int step = pImgSrc->widthStep/sizeof(uchar);
	uchar* imdata = (uchar* )pImgSrc->imageData;

	for(int i=0;i<h;i++)
	{
		for(int j=0;j<w;j++)
		{
			img[i+j*h] = (double)imdata[i*step+j*3+2];
			img[i+j*h+sz] = (double)imdata[i*step+j*3+1];
			img[i+j*h+2*sz] = (double)imdata[i*step+j*3+0];
		}
	}
	return mx;
}

static int nIndex = 0;
void CImageProcessClass::_ReadModelData(struModel &_model)
{
	CCommonFunClass fun;
	char sValue[1000] = {0};
	fstream file;
	file.open("D:\\face.txt", ios::in);
	if (file.fail()) //文件打开失败
	{
        return;
	}
	string s;
	string ss;
	_model.InitialParam();
	_model.strComponents[1].nLen = 39;
	int nPos = 0;
	int nLen = 0;
	int n = 0;
	int q = 0; //商
	int r = 0; //余数
	int i,j;
	vector<string> vs;
	while (!file.eof())
	{
		n++;
		file.getline(sValue, 1000, '\n'); //表示该行字符达到256个或遇到换行就结束
		ss = sValue;
		nPos = ss.find(":");
		nLen = ss.length();
		s.clear();
		fun.GetString(ss, nPos+1, nLen, s);
		//components
		if (n>8 && n<74)
		{   
			q = (n-8)/5;
			r = (n-8)%5;
			nIndex = q;
			switch (r)
			{
			case 0:
				fun.GetVectorString(s, " ", vs);
				nLen = vs.size();
				for (i=0; i<_model.strComponents[nIndex].nLen; i++)
				{
					_model.strComponents[nIndex].nParent[i] = atoi(vs.at(i).c_str());
				}
				break;
			case 2:	
				_model.strComponents[nIndex].nLen = atoi(s.c_str());
				cout<<nIndex<<" "<<s.c_str()<<endl;
				break;
			case 3:
				fun.GetVectorString(s, " ", vs);
				nLen = vs.size();
				for (i=0; i<_model.strComponents[nIndex].nLen; i++)
				{
					_model.strComponents[nIndex].nDefid[i] = atoi(vs.at(i).c_str());
				}
				break;
			case 4:
				fun.GetVectorString(s, " ", vs);
				nLen = vs.size();
				for (i=0; i<_model.strComponents[nIndex].nLen; i++)
				{
					_model.strComponents[nIndex].nFilterid[i] = atoi(vs.at(i).c_str());
				}
				break;
			default:
				break;
			}
		 }
		else if( n>75 && n<2916)
		{
			q = (n-75)/4;
			r = (n-75)%4;
			nIndex = q;
			switch (r)
			{
			case 0:
				fun.GetVectorString(s, " ", vs);
				nLen = vs.size();
				for (i=0; i<3; i++)
				{
					_model.strDefs[nIndex].nArchor[i] = atoi(vs.at(i).c_str());
				}
				break;
			case 3:	
				_model.strDefs[nIndex].nI = atoi(s.c_str());
				break;
			case 2:
				fun.GetVectorString(s, " ", vs);
				nLen = vs.size();
				for (i=0; i<4; i++)
				{
					_model.strDefs[nIndex].dW[i] = atof(vs.at(i).c_str());
				}
				break;
			default:
				break;
			}
		}
		else if (n > 2917)
		{
            q = (n-2916)/68;
			r = (n-2916)%68;
			nIndex = q;
		    if (r == 2)
			{
				_model.strFilters[nIndex].nI = atoi(s.c_str());
			}
			else if ( r>4 && r%2)
			{
				i = r/2 - 2;
				fun.GetVectorString(s, " ", vs);
				nLen = vs.size();
				for (j=0; j<25; j++)
				{
					_model.strFilters[nIndex].dW[i][j] = atof(vs.at(j).c_str());
				}
			}
		}
	 }
	file.close();
}

// copy src into dst using pre-computed interpolation values 插值
void CImageProcessClass::_Alphacopy(double *src, double *dst, struct alphainfo *ofs, int n) 
{
  struct alphainfo *end = ofs + n;
  while (ofs != end) 
  {
    dst[ofs->di] += ofs->alpha * src[ofs->si];
    ofs++;
  }
}

// resize along each column
// result is transposed(换位）, so we can apply it twice for a complete resize
void CImageProcessClass::_Resize1dtran(double *src, int sheight, double *dst, int dheight, int width, int chan) 
{
	double scale = (double)dheight/(double)sheight;
	double invscale = (double)sheight/(double)dheight;

	// we cache the interpolation values since they can be 
	// shared among different columns
	int len = (int)ceil(dheight*invscale) + 2*dheight;
	//alphainfo ofs[len];
	alphainfo *ofs = new alphainfo[len];
	int k = 0;
	for (int dy = 0; dy < dheight; dy++) 
	{
		double fsy1 = dy * invscale;
		double fsy2 = fsy1 + invscale;
		int sy1 = (int)ceil(fsy1);
		int sy2 = (int)floor(fsy2);       

		if (sy1 - fsy1 > 1e-3) 
		{
			assert(k < len);
			assert(sy1-1 >= 0);
			ofs[k].di = dy*width;
			ofs[k].si = sy1-1;
			ofs[k++].alpha = (sy1 - fsy1) * scale;
		}

		for (int sy = sy1; sy < sy2; sy++) 
		{
			assert(k < len);
			assert(sy < sheight);
			ofs[k].di = dy*width;
			ofs[k].si = sy;
			ofs[k++].alpha = scale;
		}

		if (fsy2 - sy2 > 1e-3) 
		{
			assert(k < len);
			assert(sy2 < sheight);
			ofs[k].di = dy*width;
			ofs[k].si = sy2;
			ofs[k++].alpha = (fsy2 - sy2) * scale;
		}
	}

	// resize each column of each color channel
	// bzero(dst, chan*width*dheight*sizeof(double));
	memset(dst, 0, chan*width*dheight*sizeof(double));
	for (int c = 0; c < chan; c++) 
	{
		for (int x = 0; x < width; x++) 
		{
			double *s = src + c*width*sheight + x*sheight;
			double *d = dst + c*width*dheight + x;
			_Alphacopy(s, d, ofs, k);
		}
	}
}

mxArray* CImageProcessClass::Resize(mxArray* mxsrc, double dScale)
{
	double *src = (double *)mxGetPr(mxsrc);
	const int *sdims = (int*)mxGetDimensions(mxsrc);
	if (mxGetNumberOfDimensions(mxsrc) != 3 || 
		mxGetClassID(mxsrc) != mxDOUBLE_CLASS)
	{
		//mexErrMsgTxt("Invalid input"); 
	}

	//double scale = mxGetScalar(mxscale);
	if (dScale > 1)
	{
		//mexErrMsgTxt("Invalid scaling factor"); 
	}

	const mwSize ddims[3] = {(int)round(sdims[0]*dScale),(int)round(sdims[1]*dScale),sdims[2]};
	mxArray *mxdst = mxCreateNumericArray(3, ddims, mxDOUBLE_CLASS, mxREAL);
	double *dst = (double *)mxGetPr(mxdst);

	double *tmp = (double *)mxCalloc(ddims[0]*sdims[1]*sdims[2], sizeof(double));
	_Resize1dtran(src, sdims[0], tmp, ddims[0], sdims[1], sdims[2]);
	_Resize1dtran(tmp, sdims[1], dst, ddims[1], ddims[0], sdims[2]);
	mxFree(tmp);

	return mxdst;
}

// unit vectors used to compute gradient orientation
double uu[9] = {1.0000, 0.9397, 0.7660, 0.500, 0.1736, -0.1736, -0.5000, -0.7660, -0.9397};
double vv[9] = {0.0000, 0.3420, 0.6428, 0.8660, 0.9848, 0.9848, 0.8660, 0.6428, 0.3420};
mxArray* CImageProcessClass::Features(const mxArray *mximage, const int nSbin) 
{
	double *im = (double *)mxGetPr(mximage);
	const int *dims = (int*)mxGetDimensions(mximage);
	if (mxGetNumberOfDimensions(mximage) != 3 ||
		dims[2] != 3 ||
		mxGetClassID(mximage) != mxDOUBLE_CLASS)
	{
		//mexErrMsgTxt("Invalid input");
	}

	//int sbin = (int)mxGetScalar(mxsbin);
	// memory for caching orientation histograms & their norms
	int blocks[2];
	blocks[0] = (int)round((double)dims[0]/(double)nSbin);
	blocks[1] = (int)round((double)dims[1]/(double)nSbin);
	double *hist = (double *)mxCalloc(blocks[0]*blocks[1]*18, sizeof(double));
	double *norm = (double *)mxCalloc(blocks[0]*blocks[1], sizeof(double));

	// memory for HOG features
	const mwSize out[3] = {max(blocks[0]-2,0), max(blocks[1]-2,0), 27+4+1};
	mxArray *mxfeat = mxCreateNumericArray(3, out, mxDOUBLE_CLASS, mxREAL);
	double *feat = (double *)mxGetPr(mxfeat);

	int visible[2];
	visible[0] = blocks[0]*nSbin;
	visible[1] = blocks[1]*nSbin;

	for (int x = 1; x < visible[1]-1; x++) 
	{
		for (int y = 1; y < visible[0]-1; y++) 
		{
			// first color channel
			double *s = im + min(x, dims[1]-2)*dims[0] + min(y, dims[0]-2);
			double dy = *(s+1) - *(s-1);
			double dx = *(s+dims[0]) - *(s-dims[0]);
			double v = dx*dx + dy*dy;

			// second color channel
			s += dims[0]*dims[1];
			double dy2 = *(s+1) - *(s-1);
			double dx2 = *(s+dims[0]) - *(s-dims[0]);
			double v2 = dx2*dx2 + dy2*dy2;

			// third color channel
			s += dims[0]*dims[1];
			double dy3 = *(s+1) - *(s-1);
			double dx3 = *(s+dims[0]) - *(s-dims[0]);
			double v3 = dx3*dx3 + dy3*dy3;

			// pick channel with strongest gradient
			if (v2 > v) 
			{
				v = v2;
				dx = dx2;
				dy = dy2;
			} 
			if (v3 > v) 
			{
				v = v3;
				dx = dx3;
				dy = dy3;
			}

			// snap to one of 18 orientations
			double best_dot = 0;
			int best_o = 0;
			for (int o = 0; o < 9; o++) 
			{
				double dot = uu[o]*dx + vv[o]*dy;
				if (dot > best_dot) 
				{
					best_dot = dot;
					best_o = o;
				} else if (-dot > best_dot) 
				{
					best_dot = -dot;
					best_o = o+9;
				}
			}

			// add to 4 histograms around pixel using linear interpolation
			double xp = ((double)x+0.5)/(double)nSbin - 0.5;
			double yp = ((double)y+0.5)/(double)nSbin - 0.5;
			int ixp = (int)floor(xp);
			int iyp = (int)floor(yp);
			double vx0 = xp-ixp;
			double vy0 = yp-iyp;
			double vx1 = 1.0-vx0;
			double vy1 = 1.0-vy0;
			v = sqrt(v);

			if (ixp >= 0 && iyp >= 0) 
			{
				*(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += 
					vx1*vy1*v;
			}

			if (ixp+1 < blocks[1] && iyp >= 0) 
			{
				*(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += 
					vx0*vy1*v;
			}

			if (ixp >= 0 && iyp+1 < blocks[0]) 
			{
				*(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += 
					vx1*vy0*v;
			}

			if (ixp+1 < blocks[1] && iyp+1 < blocks[0])
			{
				*(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += 
					vx0*vy0*v;
			}
		}
	}

	// compute energy in each block by summing over orientations
	for (int o = 0; o < 9; o++) 
	{
		double *src1 = hist + o*blocks[0]*blocks[1];
		double *src2 = hist + (o+9)*blocks[0]*blocks[1];
		double *dst = norm;
		double *end = norm + blocks[1]*blocks[0];
		while (dst < end) 
		{
			*(dst++) += (*src1 + *src2) * (*src1 + *src2);
			src1++;
			src2++;
		}
	}

	// compute features
	for (int x=0; x<(int)out[1]; x++) 
	{
		for (int y= 0; y<(int)out[0]; y++)
		{
			double *dst = feat + x*out[0] + y;      
			double *src, *p, n1, n2, n3, n4;

			p = norm + (x+1)*blocks[0] + y+1;
			n1 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
			p = norm + (x+1)*blocks[0] + y;
			n2 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
			p = norm + x*blocks[0] + y+1;
			n3 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
			p = norm + x*blocks[0] + y;      
			n4 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

			double t1 = 0;
			double t2 = 0;
			double t3 = 0;
			double t4 = 0;

			// contrast-sensitive features
			src = hist + (x+1)*blocks[0] + (y+1);
			for (int o = 0; o < 18; o++) 
			{
				double h1 = min(*src * n1, 0.2);
				double h2 = min(*src * n2, 0.2);
				double h3 = min(*src * n3, 0.2);
				double h4 = min(*src * n4, 0.2);
				*dst = 0.5 * (h1 + h2 + h3 + h4);
				t1 += h1;
				t2 += h2;
				t3 += h3;
				t4 += h4;
				dst += out[0]*out[1];
				src += blocks[0]*blocks[1];
			}

			// contrast-insensitive features
			src = hist + (x+1)*blocks[0] + (y+1);
			for (int o = 0; o < 9; o++) 
			{
				double sum = *src + *(src + 9*blocks[0]*blocks[1]);
				double h1 = min(sum * n1, 0.2);
				double h2 = min(sum * n2, 0.2);
				double h3 = min(sum * n3, 0.2);
				double h4 = min(sum * n4, 0.2);
				*dst = 0.5 * (h1 + h2 + h3 + h4);
				dst += out[0]*out[1];
				src += blocks[0]*blocks[1];
			}

			// texture features
			*dst = 0.2357 * t1;
			dst += out[0]*out[1];
			*dst = 0.2357 * t2;
			dst += out[0]*out[1];
			*dst = 0.2357 * t3;
			dst += out[0]*out[1];
			*dst = 0.2357 * t4;

			// truncation feature
			dst += out[0]*out[1];
			*dst = 0;
		}
	}
	mxFree(hist);
	mxFree(norm);

	return mxfeat;
}

/*************************************************************************************
reduce(im) resizes im to half its size, using a 5-tap binomial filter for anti-aliasing
(see Burt & Adelson's Laplacian Pyramid paper)
reduce each column
result is transposed, so we can apply it twice for a complete reduction
************************************************************************************/
void CImageProcessClass::_Reduce1dtran(double *src,int sheight,double *dst,int dheight,int width,int chan) 		                    
{
  // resize each column of each color channel
  //bzero(dst, chan*width*dheight*sizeof(double));
  memset(dst, 0, chan*width*dheight*sizeof(double));
  int y;
  double *s, *d;

  for (int c=0; c<chan; c++) 
  {
    for (int x=0; x<width; x++) 
	{
      s  = src + c*width*sheight + x*sheight;
      d  = dst + c*dheight*width + x;

      // First row
      *d = s[0]*.6875 + s[1]*.2500 + s[2]*.0625;      

	  for (y=1; y<dheight-2; y++) 
	  {	
		  s += 2;
		  d += width;
		  *d = s[-2]*0.0625 + s[-1]*.25 + s[0]*.375 + s[1]*.25 + s[2]*.0625;
	  }

      // Last two rows
      s += 2;
      d += width;
      if (dheight*2 <= sheight) 
	  {
		  *d = s[-2]*0.0625 + s[-1]*.25 + s[0]*.375 + s[1]*.25 + s[2]*.0625;
	  }
	  else 
	  {
		  *d = s[1]*.3125 + s[0]*.3750 + s[-1]*.2500 + s[-2]*.0625;
	  }
      s += 2;
      d += width;
      *d = s[0]*.6875 + s[-1]*.2500 + s[-2]*.0625;
    }
  }
}

//takes a double color image and a scaling factor,returns resized image
mxArray* CImageProcessClass::Reduce(const mxArray *mxsrc) 
{
	double *src = (double *)mxGetPr(mxsrc);
	const int *sdims = (int *)mxGetDimensions(mxsrc);
	if (mxGetNumberOfDimensions(mxsrc) != 3 || 
		mxGetClassID(mxsrc) != mxDOUBLE_CLASS)
	{
		//  mexErrMsgTxt("Invalid input");  
	}

	const mwSize ddims[3] = {(int)round(sdims[0]*.5), (int)round(sdims[1]*.5), sdims[2]};
	mxArray *mxdst = mxCreateNumericArray(3, ddims, mxDOUBLE_CLASS, mxREAL);
	double *dst = (double *)mxGetPr(mxdst);

	double *tmp = (double *)mxCalloc(ddims[0]*sdims[1]*sdims[2], sizeof(double));
	_Reduce1dtran(src, sdims[0], tmp, ddims[0], sdims[1], sdims[2]);
	_Reduce1dtran(tmp, sdims[1], dst, ddims[1], ddims[0], sdims[2]);
	mxFree(tmp);

	return mxdst;
}

void CImageProcessClass::_FeatPyramid(IplImage* pImgSrc, struModel model, struPyra &_strPyra)
{
	// pyra = featpyramid(im, model, padx, pady);
	// Compute feature pyramid.//计算特征金字塔
	//
	//pyra.feat{i} is the i-th level of the feature pyramid.//特征金字塔的第i级
	//pyra.scales{i} is the scaling factor used for the i-th level.//第i级的尺度因子
	//pyra.feat{i+interval} is computed at exactly half the resolution of feat{i}.//一半的分辨率
	//first octave halucinates higher resolution data.//第一个八度出现幻觉更高分辨率的数据

	int nInterval  = model.nInterval;
	int nSbin = model.nSbin;
	//Select padding(填充）, allowing for one cell in model to be visible
	//Even padding allows for consistent spatial relations across 2X scales//
	int nPadx = __max(model.strMaxSize.cx-1-1,0);
	int nPady = __max(model.strMaxSize.cy-1-1,0);

	double dSc = pow(2.0,(1/(nInterval+0.0000001)));
	struSize strImsize;
	strImsize.cx = pImgSrc->width;
	strImsize.cy = pImgSrc->height;

	int nMax_scale = 1 + (int)floor(log(__min(strImsize.cx,strImsize.cy)/(5*nSbin+0.000001))/log(dSc));
	int nLen = nMax_scale + nInterval;
	vector<struPyra> vstrPyra;
 
	// our resize function wants floating point values
	int nWidth = pImgSrc->width;
	int nHeight = pImgSrc->height;
	const mwSize dims[3] = {nHeight, nWidth, 3};
	mxArray *mxSrc = IplImage2mxArray8U3(pImgSrc);

	_strPyra.InitialParam();
	int nLenPyra = 0;
	for (int i=1; i<=nInterval; i++) //for i = 1:interval(包含interval)
	{
		mxArray* mxScale = Resize(mxSrc, 1/pow(dSc,(i-1)));
	
		// "first" 2x nInterval  
		DWORD dwCurrent = GetTickCount(); 
		mxArray* mxFeat1 = Features(mxScale, nSbin/2);//pyra.feat{i}/**/
		DWORD dwEnd = GetTickCount(); 
		cout<<dwEnd-dwCurrent<<endl;

		_strPyra.mxFeat[i-1] = mxFeat1;
		_strPyra.dScale[i-1] = 2/pow(dSc,(i-1));
		nLenPyra++;
		mxDestroyArray(mxFeat1);

		 // "second" 2x nInterval
        mxArray* mxFeat2 = Features(mxScale, nSbin);//pyra.feat{i}/**/
		_strPyra.mxFeat[i+nInterval-1] = mxFeat2;
		_strPyra.dScale[i+nInterval-1] = 1/pow(dSc,(i-1));
		nLenPyra++;
		mxDestroyArray(mxFeat2);

		for (int j=i+nInterval; j<=nMax_scale; j+=nInterval)
		{
			mxArray* mxScale1 = Reduce(mxScale);
			mxArray* mxFeat3 = Features(mxScale1, nSbin);//pyra.feat{i}/**/
			_strPyra.mxFeat[j+nInterval-1] = mxFeat3;
			_strPyra.dScale[j+nInterval-1] = 0.5 * _strPyra.dScale[j-1];
			nLenPyra++;
			mxDestroyArray(mxScale1);
			mxDestroyArray(mxFeat3);
		}
		mxDestroyArray(mxScale);
	}
/*
// 还没改好
	for ( int i=0; i<nLenPyra; i++)
	{
		//add 1 to padding(填充） because feature generation deletes a 1-cell
		//wide border around the feature map
		pyra.feat{i} = padarray(pyra.feat{i}, [pady+1 padx+1 0], 0);
		//write boundary occlusion feature
		pyra.feat{i}(1:pady+1, :, end) = 1;
		pyra.feat{i}(end-pady:end, :, end) = 1;
		pyra.feat{i}(:, 1:padx+1, end) = 1;
		pyra.feat{i}(:, end-padx:end, end) = 1;
	}*/

	for (int i=0; i<nLenPyra; i++)
	{
		_strPyra.dScale[i] = model.nSbin/_strPyra.dScale[i];
	}
	_strPyra.nInterval = nInterval;
	_strPyra.nImy = strImsize.cy;
	_strPyra.nImx = strImsize.cx;
	_strPyra.nPady = nPady;
	_strPyra.nPadx = nPadx;

    mxDestroyArray(mxSrc);
}

/***********************************************************************************
功能：通过姿态估计进行人脸检测，可检测+-90的人脸
输入：
IplImage* pImgSrc: 待检测图
输出：
vector<Rect> &_vRect：人脸所在的区域
备注：算法目前还没写完及测试
************************************************************************************/
void CImageProcessClass::DetectFacePoseEstimation(IplImage* pImgSrc, vector<CvRect> &_vRect)
{
	//step1:read model data
	struModel model;
	_ReadModelData(model);
	vector<RECT> vRect;
	model.nInterval = 5;

	//step2:detect
	if (pImgSrc->nChannels != 3)
	{
		return;
	}
	//Keep track of detected boxes and features
	struBox strboxes[BOXCACHESIZE];
	for (int i=0; i<BOXCACHESIZE; i++)
	{
	   strboxes[i].InitialParam();
	}
	
	int nCnt = 0;
	struPyra vStrPyra;
	DWORD dwCurrent = GetTickCount(); 
	_FeatPyramid(pImgSrc, model, vStrPyra);

	DWORD dwEnd = GetTickCount(); 
	cout<<dwEnd-dwCurrent<<endl;

	Sleep(2000);

	///////////////////////////////////////////////
	//dll
/*
	if (!mclInitializeApplication(NULL,0))
	{
		cout<<"Could not initialize the application !"<<endl;;
		return;
	}
	
	if (!testInitialize())
	{
		cout<<"Could not initialize the library !"<<endl;
		return;
	}


	if (!visualizemodelInitialize())
	{
		cout<<"Could not initialize the library !"<<endl;
		return;
	}
	
	mwArray y(1,1,mxDOUBLE_CLASS);
	//mwArray x;

	//给输入 mxArray 对象分配内存
	mwArray x(1,1,mxDOUBLE_CLASS);
	double x1[] = {1};
	// 给输入 mxArray 对象赋值
	x.SetData(x1,1);

	test(1, y, x);

	visualizemodelTerminate();
	*/
	//preprocess();
	/*MATFile *pmat;
	const char **dir;
	const char *file;
	const char *name;
	int         ndir;
	mxArray *pa;
	file = "D:\\smartvision\\project\\faceDetect\\Debug\\face_p146_small.mat";  //双反斜杠防止转义

	pmat = matOpen(file, "r");//打开文件，返回指向文件指针
	if (pmat == NULL)
	{
		printf("pmat==NULL");//cout<<"Error opening file:" <<file<<endl;
		return;
	}

	dir = (const char **)matGetDir(pmat, &ndir);
	//ndir 表示mat文件中含有矩阵数目
	if (dir == NULL)
	{
		printf("Error reading directory of file:");
		return;
	}
	else
	{
		printf("Directory of %s\n",file);
		for (int i=0; i < ndir; i++)
			printf("%s\n",dir[i]);//输出所含矩阵数目
	}
	 
	 */
/*	mxArray *pMxArray = NULL; 
	pMxArray = matGetVariable(pmat, "model"); 
	double* initA = (double*) mxGetData(pMxArray);
	int M = mxGetM(pMxArray);
	int N = mxGetN(pMxArray);
	IplImage *pImg = NULL;
	pImg = mxArray2IplImage8U3(pMxArray);

	cvSaveImage("D:\\1.bmp", pImg);
	
	mwArray in1(M, N, mxDOUBLE_CLASS, mxREAL); 
	double* img = mxGetPr(pMxArray);
	in1.SetData(img, M*N);
    
	mwArray compid;
	double dArray[13] = {0};
	for (int i=0; i<14; i++)
	{
		dArray[i] = i;
	}	
	compid.SetData(dArray, 13);*/
//	visualizemodel(in1, compid); 
	
//	matClose(pmat);
//	mxFree(pMxArray);

}
/*****************************************************************
函数：IplImage2mxArray8U3
功能：IplImage数据转换double的mxArray，在C++中多维矩阵是按行存放，而Matlab中是按列存放
备注：彩色图像
******************************************************************/

bool CImageProcessClass::testSVM(string* positiveTestPath, string* negativTestPath, string* svmPath)
{/*
#pragma region Initialization

	printf("Initialize\n");
	// Finding all images in both pathes
	std::vector<String> positiveFileNames, negativeFileNames, allFileNames;
	glob(*positiveTestPath, positiveFileNames);
	glob(*negativTestPath, negativeFileNames);

	// Testing if there are images in the pathes
	if (positiveFileNames.size() <= 0)
	{
		printf("There are no images in %s\n", *positiveTestPath);
		return false;
	}
	if (negativeFileNames.size() <= 0)
	{
		printf("There are no images in %s\n", *negativTestPath);
		return false;
	}
	allFileNames.insert(allFileNames.end(), positiveFileNames.begin(), positiveFileNames.end());
	allFileNames.insert(allFileNames.end(), negativeFileNames.begin(), negativeFileNames.end());

	//Mat testData = Mat_<float>(DESCRIPTOR_SIZE, allFileNames.size());
	Mat testData = Mat_<float>(allFileNames.size(), DESCRIPTOR_SIZE);
	int testCount = 0;

	Ptr<ml::SVM> svm = ml::SVM::create();
	svm = svm->load<ml::SVM>(*svmPath);
	if (!svm->isTrained())
	{
		printf("The SVM isn't trained through this path: %s\n", *svmPath);
		return false;
	}

	HOGDescriptor hogD;
	hogD.winSize = Size(WINDOW_SIZE, WINDOW_SIZE);
	std::vector<float> descriptorsValues;
	std::vector<Point> locations;

	clock_t beginTime = clock();

#pragma endregion

#pragma region HOG Descriptors

	// Converting the positve images and calculating the HOG
	std::cout << "Calculate all Images (" << (clock() - beginTime) / (float)CLOCKS_PER_SEC << ") ...";
	for (std::vector<String>::iterator fileName = allFileNames.begin(); fileName != allFileNames.end(); ++fileName)
	{

		Mat actualImage = imread(*fileName);

		// Testing if the file is an image
		if (actualImage.empty())
		{
			printf("Couldn't read the image %s\n", *fileName);
			return false;
		}
		cvtColor(actualImage, actualImage, CV_BGR2GRAY);
		resize(actualImage, actualImage, Size(WINDOW_SIZE, WINDOW_SIZE));

		// Calculating the HOG
		hogD.compute(actualImage, descriptorsValues, Size(0, 0), Size(0, 0), locations);

		// I need to transpose to get every sample in a column and not in a row
		Mat descriptorsVector = Mat_<float>(descriptorsValues, true);
		transpose(descriptorsVector, descriptorsVector);
		descriptorsVector.row(0).copyTo(testData.row(testCount));
		testCount++;

	}
	std::cout << " Finished (" << (clock() - beginTime) / (float)CLOCKS_PER_SEC << ")" << std::endl;
	std::cout << std::endl << std::endl;

#pragma endregion

#pragma region Testing the Data

	Mat results;
	svm->predict(testData, results);
	int fPos = 0, fNeg = 0;

	for (int c = 0; c < allFileNames.size(); c++)
	{
		float result = results.at<float>(c, 0);
		if (c < positiveFileNames.size() && result != 1)
			fNeg++;
		else if (c >= positiveFileNames.size() && result != -1)
			fPos++;
	}
	printf("False Positive: %d\n", fPos);
	printf("False Negative: %d\n", fNeg);

#pragma endregion
*/
	return true;
}

void CImageProcessClass::GetHOGFeatures(IplImage* pImgSrc, vector<float> &_vfFeatures)
{
	if (pImgSrc == NULL)
	{
		return;
	}

	IplImage *pImgScale = NULL;       
	double dScale = 1; //缩放倍数  
    CvSize czSize;              //目标图像尺寸 
	HOGDescriptor *hog = new HOGDescriptor(cvSize(64,64),cvSize(16,16),cvSize(8,8),cvSize(8,8),9);  
	_vfFeatures.clear();//结果数组 
	vector<float> vfFeatures;

	int nInterval = 1;
	double dSc = pow(2, 1.0/nInterval);
	for (int i=0; i<nInterval; i++)
	{
		dScale = 0.7;//1.0/pow(dSc, i);
		//计算目标图像大小  
		czSize.width = (int)(pImgSrc->width * dScale);  
		czSize.height = (int)(pImgSrc->height * dScale);  
		if (__min(czSize.width,czSize.height) < 64) 
		{
			return;
		}

		//创建图像并缩放  
		pImgScale = cvCreateImage(czSize, pImgSrc->depth, pImgSrc->nChannels);  
		cvResize(pImgSrc, pImgScale);       

		hog->compute(pImgScale, vfFeatures, Size(64,64), Size(0,0)); //调用计算函数开始计算    
		copy(vfFeatures.begin(), vfFeatures.end(), std::back_inserter(_vfFeatures)); 
		cout<<"HOG dims: "<<_vfFeatures.size()<<endl;    
	}

}

void CImageProcessClass::_SampleRelevance(vector<float> vfData[], int nNum)
{
	float fRelevance[500][500] = {0};
	float fMean[500] = {0};
	float fVar[500] = {0};
	double dSum = 0;	
	int nLen = 0;
	for (int i=0; i<nNum; i++)
	{
		dSum = 0.0;
		nLen = vfData[i].size();
		for (int j=0; j<nLen; j++)
		{
			dSum += (double)vfData[i].at(j);
		}
		fMean[i] = (float)dSum/nLen;

		dSum = 0;
		for (int j=0; j<nLen; j++)
		{
			dSum += (double)pow((vfData[i].at(j) - fMean[i]), 2);
		}
		fVar[i] = (float)dSum;
	}
	
	FILE* pFileCSV = NULL;
	pFileCSV = fopen("D:\\relevance.csv", "at");
	char buffer[MAX_PATH] = {0};
	for (int i=0; i<nNum; i++)
	{
		for (int j=i+1; j<nNum; j++)
		{
			nLen = __min(vfData[i].size(), vfData[j].size());
			dSum = 0;
			for (int k=0; k<nLen; k++)
			{
				dSum += fabs(vfData[i].at(k) -  vfData[j].at(k));
			}
			fRelevance[i][j] = dSum /sqrt(fVar[i]*fVar[j]);			
		}		
	}

	for (int i=0; i<nNum; i++)
	{
		for (int j=0; j<nNum; j++)
		{
			sprintf(buffer, "%.4f,", fRelevance[i][j]);
			fprintf(pFileCSV, "%s", buffer);
		}
		fprintf(pFileCSV, "\n");
	}
    fclose(pFileCSV);
}
 
void CImageProcessClass::TrainAndRecognitionHOGSVMFolder()
{
	//step1:train
	CCommonFunClass fun;
	vector<string> vsFolder, vsFileName[100];
	string sPath = "E:\\face_file\\sampleData\\face\\sample"; 
	vector<float> vfFeatures;
	IplImage* pImgSrc = NULL;
	CvMat *data_mat, *res_mat;    
    string s = "";
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	int nSum = 0;
	for (int i=0; i<(int)vsFolder.size(); i++)
	{
		nSum += vsFileName[i].size();
	}
    
	vector<float> vfData[300];
	int nImgNum = nSum;            //读入样本数量     
    ////样本矩阵，nImgNum：横坐标是样本数量， WIDTH * HEIGHT：样本特征向量，即图像大小     
    data_mat = cvCreateMat( nImgNum, 1764, CV_32FC1 );  //
    cvSetZero( data_mat );   

    //类型矩阵,存储每个样本的类型标志     
    res_mat = cvCreateMat( nImgNum, 1, CV_32FC1 );    
    cvSetZero( res_mat );  

	IplImage* pImg = cvCreateImage(cvSize(42,42), 8, 1);
	cvZero(pImg);

	CvSVM svm;      
    CvSVMParams param;      
    CvTermCriteria criteria;      
    criteria = cvTermCriteria( CV_TERMCRIT_EPS, 1000, FLT_EPSILON );      
    param = CvSVMParams( CvSVM::C_SVC, CvSVM::RBF, 10.0, 0.09, 1.0, 10.0, 0.5, 1.0, NULL, criteria );         

	FILE* pFileCSV = NULL;
	pFileCSV = fopen("D:\\feat2.csv", "at");
	char buffer[MAX_PATH] = {0};
	int row = 0;
	int col = 0;
	for (int i=0; i<(int)vsFolder.size(); i++)
	{
		for (int j=0; j<(int)vsFileName[i].size(); j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			pImgSrc = cvLoadImage(s.c_str());
			GetHOGFeatures(pImgSrc, vfFeatures);
	        
			row = i==0? j : j + i*vsFileName[i-1].size();
            //for(vector<float>::iterator iter=vfFeatures.begin();iter!=vfFeatures.end();iter++)    
			for (int k=0; k<(int)vfFeatures.size(); k++)
            {    
				cvmSet(data_mat, row, k, vfFeatures.at(k)); 		 
			/*	sprintf(buffer, "%.2f,", 100*vfFeatures.at(k));
				fprintf(pFileCSV, "%s", buffer);

				w = k%42;
				h = k/42;
				pImg->imageData[pImg->widthStep *h + w] = 1000*vfFeatures.at(k);
				vfData[row].push_back(vfFeatures.at(k));*/
            }
			//	fprintf(pFileCSV, "\n");
		//	sprintf(buffer, "D:\\1\\%d-%d.bmp", i,j);
		//	cvSaveImage(buffer, pImg);
			cvmSet(res_mat,row,0, atof(vsFolder.at(i).c_str()));//);  	(double)(i+1)
		}
	}
	cvReleaseImage(&pImg);
	fclose(pFileCSV);
	//_SampleRelevance(vfData, nSum);
                  
	svm.train( data_mat, res_mat, NULL, NULL, param );      
    //☆☆利用训练数据和确定的学习参数,进行SVM学习☆☆☆☆        
    svm.save( "D:\\SVM_DATA.txt" );  
	
	//svm.load( "D:\\SVM_DATA.txt" );

	// step2: recognition	
	CvMat *sample_mat = cvCreateMat( 1, 1764, CV_32FC1 );  //1764    
    cvSetZero( sample_mat );  
	CvMat* results = cvCreateMat( 1, 1, CV_32FC1 );
	cvSetZero( results );
	int ret = 0;

	sPath = "E:\\face_file\\sampleData\\face\\test";
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	int nOK = 0;
	int nNG = 0;
	for (int i=0; i<(int)vsFolder.size(); i++)
	{
		for (int j=0; j<(int)vsFileName[i].size(); j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			pImgSrc = cvLoadImage(s.c_str());
			GetHOGFeatures(pImgSrc, vfFeatures);
			
			row = i==0? j : j + i*vsFileName[i-1].size();
			cvSetZero( sample_mat );  
            //for(vector<float>::iterator iter=vfFeatures.begin();iter!=vfFeatures.end();iter++) 
			for (int k=0; k<(int)vfFeatures.size(); k++)
            {    
                cvmSet(sample_mat, 0, k, vfFeatures.at(k));//*iter);   
            }	
            ret = (int)svm.predict(sample_mat, results);
			cout<<atoi(vsFolder.at(i).c_str())<<", "<<ret<<endl;
			if (ret == atoi(vsFolder.at(i).c_str()))
			{
				nOK++;
			}
			else
			{
				nNG++;
			}
		}
	}

	cvReleaseMat( &data_mat );    
	cvReleaseMat( &res_mat );  
	cvReleaseMat( &sample_mat );
	cvReleaseMat( &results );

	cout<<"HOGSVM OK rate = "<< nOK/(nNG+nOK+0.0000001)<<endl;
}

//opencv 常用的人脸识别方法
void CImageProcessClass::TrainAndRecognitionCascade()
{
	//step1:train
	CCommonFunClass fun;
	vector<string> vsFolder, vsFileName[100];
	string sPath = "E:\\face_file\\sampleData\\face\\sample"; 
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);

	Ptr<FaceRecognizer> cFaceModel = createEigenFaceRecognizer(80);//createLBPHFaceRecognizer();////createFisherFaceRecognizer(NUM_COMPONENTS);//10 Principal components;
	vector<Mat> vMatImages;
	vector<int> vnLabels;
	IplImage* pImgSrc = NULL;
	Mat matSrc;
	string s = "";
	for (int i=0; i<(int)vsFolder.size(); i++)
	{
		for (int j=0; j<(int)vsFileName[i].size(); j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			pImgSrc = cvLoadImage(s.c_str(), 0);

			//matSrc = imread(s.c_str(), 0);
			FaceModel(pImgSrc, pImgSrc);

			vMatImages.push_back(pImgSrc);
			vnLabels.push_back(atoi(vsFolder.at(i).c_str()));
		}
	}

	cFaceModel->train(vMatImages, vnLabels);
	cFaceModel->save("D:\\FACE_DATA.txt");

	//step2:recognition
	sPath = "E:\\face_file\\sampleData\\face\\test";
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	int nOK = 0;
	int nNG = 0;
	int nLabel = 0;
	double dConfidence = 0;
	for (int i=0; i<(int)vsFolder.size(); i++)
	{
		for (int j=0; j<(int)vsFileName[i].size(); j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
		//	matSrc = imread(s.c_str(), 0);
			pImgSrc = cvLoadImage(s.c_str(), 0);
			FaceModel(pImgSrc, pImgSrc);

			cFaceModel->predict((Mat)pImgSrc, nLabel, dConfidence);
			cout<<"label= "<<nLabel<<"("<<atoi(vsFolder.at(i).c_str())<<")"<<"， confidence= "<<dConfidence<<endl;
			if (nLabel == atoi(vsFolder.at(i).c_str()))
			{
				nOK++;
			}
			else
			{
				nNG++;
			}
		}
	}

	cout<<"HOGSVM OK rate = "<< nOK/(nNG+nOK+0.0000001)<<endl;
}

void CImageProcessClass::GetSkinRect(IplImage* pImgSrc, CvRect rect, CvRect &_rect)
{
	if (pImgSrc->nChannels !=3 )
	{
		return;
	}

	cvSetImageROI(pImgSrc, rect);
	IplImage* pImgROI = cvCreateImage(cvSize(pImgSrc->roi->width, pImgSrc->roi->height), 8, 3);
	cvCopy(pImgSrc, pImgROI);
	cvResetImageROI(pImgSrc);

	cvtColor((Mat)pImgROI, (Mat)pImgROI, CV_BGR2YCrCb); //首先转换成到YCrCb空间
	imwrite("D:\\skin.bmp", (Mat)pImgROI);

	cvReleaseImage(&pImgROI);

}

void CImageProcessClass::CalcGaborFeatures(IplImage* pImgSrc, vector<float> &_vfFeatures, double dSigma, 
	                                       double dF, double dPhi, int iNu, int nBlockNum)
{
	IplImage* pImgDst1 = cvCreateImage(cvGetSize(pImgSrc), 8, 1);    //一定要先分配一个内存
	IplImage* pImgDst2 = cvCreateImage(cvGetSize(pImgSrc), 8, 1);
	IplImage* pImgDst3 = cvCreateImage(cvGetSize(pImgSrc), 8, 1);

	CvGabor *gabor1 = new CvGabor;      
	gabor1->Init(dPhi, iNu, dSigma, dF); 

	//获取载入图像的gabor滤波响应的实部并且显示 
	gabor1->conv_img(pImgSrc, pImgDst1, CV_GABOR_REAL);     
	//获取载入图像的gabor滤波响应的虚部并且显示        
    gabor1->conv_img(pImgSrc, pImgDst2, CV_GABOR_IMAG); 
	//获取载入图像的gabor滤波响应的模并且显示      
    gabor1->conv_img(pImgSrc, pImgDst3, CV_GABOR_MAG); 
	  
	int nWBlock = pImgSrc->width/nBlockNum;
	int nHBlock = pImgSrc->height/nBlockNum;

	vector<float> vfReal;
	vector<float> vfImag;
	vector<float> vfMag;
	float fSum1 = 0;
	float fSum2 = 0;
	float fSum3 = 0;
	int nSize = nBlockNum*nBlockNum;
	for (int m=0; m<nHBlock; m++)
	{
		for (int n=0; n<nWBlock; n++)
		{
			fSum1 = 0;
			fSum2 = 0;
			fSum3 = 0;
			for (int mm=0; mm<nBlockNum; mm++)
			{
				for (int nn=0; nn<nBlockNum; nn++)
				{
					fSum1 += pImgDst1->imageData[pImgDst1->widthStep*(m*nBlockNum+mm) + n*nBlockNum + nn];
					fSum2 += pImgDst2->imageData[pImgDst2->widthStep*(m*nBlockNum+mm) + n*nBlockNum + nn];
					fSum3 += pImgDst3->imageData[pImgDst3->widthStep*(m*nBlockNum+mm) + n*nBlockNum + nn];
				}
			}
			vfReal.push_back(fSum1/nSize);
			vfImag.push_back(fSum2/nSize);
			vfMag.push_back(fSum3/nSize);
		}
	}
	_vfFeatures.clear();
	//copy(vfReal.begin(), vfReal.end(), std::back_inserter(_vfFeatures)); 
	//copy(vfImag.begin(), vfImag.end(), std::back_inserter(_vfFeatures)); 
	copy(vfMag.begin(), vfMag.end(), std::back_inserter(_vfFeatures)); 

	cvReleaseImage(&pImgDst1);
	cvReleaseImage(&pImgDst2);
	cvReleaseImage(&pImgDst3);
}

void CImageProcessClass::testFaceFeatures()
{
	//创建一个方向是PI/4而尺度是3的gabor     
    double Sigma = 2*PI;      
    double F = sqrt(2.0);      
    CvGabor *gabor1 = new CvGabor;      
    gabor1->Init(PI/16, 3, Sigma, F);  

	//step1:train
	CCommonFunClass fun;
	vector<string> vsFolder, vsFileName[100];
	string sPath = "E:\\face_file\\sampleData\\face\\sample";//"D:\\Sample1"; 
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	string sPath1 = "E:\\face_file\\sampleData\\face\\sample_test";
	//IplImage* pImg = cvLoadImage("D:\\face2.bmp", 0);
    
	int nBlock = 2;
	IplImage* pImg = cvCreateImage(cvSize(100/nBlock, 100/nBlock), 8, 1);
	char buffer[MAX_PATH] = {0};
	IplImage* pImgSrc = NULL;
	string s = "";
	double dSum = 0;
	for (int i=0; i<(int)vsFolder.size(); i++)
	{
		for (int j=0; j<(int)vsFileName[i].size(); j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			pImgSrc = cvLoadImage(s.c_str(), 0);
			//获取载入图像的gabor滤波响应的模并且显示      
			gabor1->conv_img(pImgSrc, pImgSrc, CV_GABOR_MAG);  
			for (int m=0; m<100/nBlock; m++)
			{
				for (int n=0; n<100/nBlock; n++)
				{	
					dSum = 0;
					for (int m1=0; m1<nBlock; m1++)
					{
						for (int n1=0; n1<nBlock; n1++)
						{
							dSum += pImgSrc->imageData[pImgSrc->widthStep*(m*nBlock+m1) + n*nBlock + n1];
						}
					}
					pImg->imageData[pImg->widthStep*m + n] = (int)(dSum/(nBlock*nBlock));	
				}
			}
			s = sPath1 + "\\" + vsFolder.at(i).c_str() + "_" + vsFileName[i].at(j);  
			cvSaveImage(s.c_str(), pImg); 
		}
	}

	cvReleaseImage(&pImg);
}

void CImageProcessClass::FaceModel(IplImage* pImgSrc, IplImage* pImgDst)
{
	string sPath = "E:\\detect\\face.bmp";
	IplImage* pImg = cvLoadImage(sPath.c_str(), 0);
	cvCopy(pImgSrc, pImgDst);

	if (pImgSrc->width == pImg->width && pImgSrc->height==pImg->height)
	{
		for (int i=0; i<pImgSrc->height; i++)
		{
			for (int j=0; j<pImgSrc->width; j++)
			{
				if (pImg->imageData[pImg->widthStep*i + j] >= 0)
				{
					pImgSrc->imageData[pImgSrc->widthStep*i + j] = 0;
				}
			}
		}
	}

}

void CImageProcessClass::KeyFace()
{
	IplImage* pImgSrc = cvLoadImage("E:\\face_file\\sampleData\\face\\sample\\1\\1.bmp", 0);//
	String eyeCasName = "C:\\Program Files\\opencv2410\\sources\\data\\haarcascades\\haarcascade_mcs_mouth.xml";
	//haarcascade_eye_tree_eyeglasses.xml";//

	IplImage* pImgDst = cvCreateImage(cvSize(100,100), 8, 1);
	cvZero(pImgDst);
	cvResize(pImgSrc,pImgDst,CV_INTER_CUBIC);
	int nW = pImgDst->width;
	int nH = pImgDst->height;
    cvSaveImage("D:\\test.bmp", pImgDst);

	CvMat* A = cvCreateMat(nW,nH,CV_32FC1);
	CvMat* U = cvCreateMat(nW,nH,CV_32FC1); 
	CvMat* D = cvCreateMat(nW,nH,CV_32FC1); 
	CvMat* V = cvCreateMat(nW,nH,CV_32FC1); 
/*
	for (int i=0; i<nW; i++)
	{
		for (int j=0; j<nH; j++)
		{
			cvmSet(A,i,j, (double)pImgSrc->imageData[pImgSrc->widthStep*j + i]);
		}
	}
	*/
	cvConvert(pImgDst, A);
	cvSVD(A, D, U, V, CV_SVD_U_T|CV_SVD_V_T); // A = U D V^T 
    
	Mat img(100, 100, CV_8UC3);
	img = U;
    img.convertTo(img,CV_32FC3);  
	Mat dst = clustering(img, 3);

	//聚类
	Scalar colorTab[] =     //因为最多只有5类，所以最多也就给5个颜色
    {
        Scalar(0, 0, 255),
        Scalar(0,255,0),
        Scalar(255,100,100),
        Scalar(255,0,255),
        Scalar(0,255,255),
		Scalar(255,255,255)
    };

	
	RNG rng(12345); //随机数产生器
	int i, k, sampleCount = 10000;
    Mat points(sampleCount, 1, CV_32FC2), labels;   //产生的样本数，实际上为2通道的列向量，元素类型为Point2f
	img.convertTo(points, CV_32FC2);
	imwrite("D:\\test2.bmp", points);

	int clusterCount = 3;
    Mat centers(clusterCount, 1, points.type());    //用来存储聚类后的中心点

	/* generate random sample from multigaussian distribution */
	for(k=0; k<clusterCount; k++) //产生随机数
	{
		Point center;
		center.x = rng.uniform(0, img.cols);
		center.y = rng.uniform(0, img.rows);
		Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
			k == clusterCount - 1 ? sampleCount :
			(k+1)*sampleCount/clusterCount);   //最后一个类的样本数不一定是平分的，
		//剩下的一份都给最后一类
		//每一类都是同样的方差，只是均值不同而已
		rng.fill(pointChunk, CV_RAND_NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.05, img.rows*0.05));
	}

	randShuffle(points, 1, &rng);   //因为要聚类，所以先随机打乱points里面的点，注意points和pointChunk是共用数据的。
	
	kmeans(points, clusterCount, labels,
		TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
		3, KMEANS_PP_CENTERS, centers);  //聚类3次，取结果最好的那次，聚类的初始化采用PP特定的随机算法。

	img = Scalar::all(0);
	int clusterIdx = -1;
	Point ipt;

	cvZero(pImgDst);
	Mat img0(100, 100, CV_8UC1);
	Mat img1(100, 100, CV_8UC1);
	Mat img2(100, 100, CV_8UC1);
	img0 = Scalar::all(0);
	img1 = Scalar::all(0);
	img2 = Scalar::all(0);
	IplImage* pImgDst1 = cvCreateImage(cvSize(100,100), 8, 1);
	cvZero(pImgDst1);
	IplImage* pImgDst2 = cvCreateImage(cvSize(100,100), 8, 1);
	cvZero(pImgDst2);

	for(i=0; i<sampleCount; i++)
	{
		clusterIdx = labels.at<int>(i);
		ipt = points.at<Point2f>(i);
		circle(img, ipt, 2, colorTab[clusterIdx], CV_FILLED, CV_AA );
	
		if (0==clusterIdx)
		{
			circle(img0, ipt, 1, colorTab[5], CV_FILLED, CV_AA );
			pImgDst->imageData[pImgDst->widthStep*ipt.y + ipt.x] = 1;
		}
		if (1==clusterIdx)
		{
			circle(img1, ipt, 1, colorTab[5], CV_FILLED, CV_AA );
			pImgDst1->imageData[pImgDst1->widthStep*ipt.y + ipt.x] = 1;
		}
		if (2==clusterIdx)
		{
			circle(img2, ipt, 1, colorTab[5], CV_FILLED, CV_AA );
			pImgDst2->imageData[pImgDst2->widthStep*ipt.y + ipt.x] = 1;
		}
	}
	imwrite("D:\\test2.bmp",img);

	img0 = Scalar::all(0);
	img0 = pImgDst;
	img1 = Scalar::all(0);
	img1 = pImgDst1;
	img2 = Scalar::all(0);
	img2 = pImgDst2;

	IplImage* pImgSrc1 = cvLoadImage("E:\\face_file\\sampleData\\face\\sample\\1\\1.bmp", 0);//
	cvZero(pImgDst);
	cvResize(pImgSrc,pImgDst,CV_INTER_CUBIC);
    Mat img_new(100, 100, CV_8UC1);
	img_new = pImgDst;

	Mat img_new0(100, 100, CV_32FC1);
	Mat img_new1(100, 100, CV_32FC1);
	Mat img_new2(100, 100, CV_32FC1);
	img_new0 = Scalar::all(0);
	img_new1 = Scalar::all(0);
	img_new2 = Scalar::all(0);

	img_new0 = img_new.mul(img0);
	img_new1 = img_new.mul(img1);
	img_new2 = img_new.mul(img2);
	//cvMatMulAdd(&img_new, &img0, 0, &img_new0);
	//cvMatMulAdd(&img_new, &img1, 0, &img_new1);
	//cvMatMulAdd(&img_new, &img2, 0, &img_new2);
	imwrite("D:\\test2.bmp",img_new0);
	imwrite("D:\\test2.bmp",img_new1);
	imwrite("D:\\test2.bmp",img_new2);

	Scalar     mean;  
	Scalar     stddev;  
	cv::meanStdDev (img_new0, mean, stddev );  
	double  mean_pxl = mean.val[0];  
	double  stddev_pxl = stddev.val[0];  
	int j;
	double* p;
	for( i = 0; i <img_new0.rows; i++)
	{
		p = img_new0.ptr<double>(i);
		for ( j = 0; j < img_new0.cols; j++)
		{
			p[j] = (p[j]-mean_pxl)/stddev_pxl;
		}
	}
	imwrite("D:\\test1.bmp",img_new0);

	cv::meanStdDev (img_new1, mean, stddev );  
	mean_pxl = mean.val[0];  
	stddev_pxl = stddev.val[0];  
	for( i = 0; i <img_new1.rows; i++)
	{
		p = img_new1.ptr<double>(i);
		for ( j = 0; j < img_new1.cols; j++)
		{
			p[j] = (p[j]-mean_pxl)/stddev_pxl;
		}
	}
	imwrite("D:\\test1.bmp",img_new1);

	cv::meanStdDev (img_new2, mean, stddev );  
	mean_pxl = mean.val[0];  
	stddev_pxl = stddev.val[0];  
	for( i = 0; i <img_new2.rows; i++)
	{
		p = img_new2.ptr<double>(i);
		for ( j = 0; j < img_new2.cols; j++)
		{
			p[j] = (p[j]-mean_pxl)/stddev_pxl;
		}
	}
	imwrite("D:\\test1.bmp",img_new2);

	img_new0 += img_new1;
    img_new0 += img_new2;
	imwrite("D:\\test1.bmp",img_new0);
	//img_new0.convertTo(img0, CV_8UC1);
	//cvSaveImage("D:\\test1.bmp", pImgDst);



    /* usage: eyedetect <image> */

/*	int i;
	int j=1;
	char cstr[256];
	for(i =1;i<10;i++) 
	{
		sprintf(cstr,"OriginalImages/song/s%d.jpg",i);
		IplImage *img = cvLoadImage(cstr);
		IplImage* processedImage = detectFaceFeaturesAndProcessImage(img);
		if(processedImage!= NULL) 
		{
			sprintf(cstr,"TestImages/s11/%d.pgm",j);
			cvSaveImage(cstr,processedImage);
			j++;
		}
	}*/


    /* detect eyes and display image */
   // IplImage* processedImage = detectFaceFeaturesAndProcessImage(pImgSrc);
   // cvSaveImage("D:\\test.bmp", processedImage);
 


	//亮度变化
	/*CvScalar value = cvAvg(pImgSrc);
	double dMean = value.val[0];
	cvSubS(pImgSrc, value, pImgSrc);
	cvSaveImage("D:\\test.bmp", pImgSrc);
	for (int i=0; i<nW; i++)
	{
		for (int j=0; j<nH; j++)
		{
			pImgSrc->imageData[pImgSrc->widthStep*j + i] = pImgSrc->imageData[pImgSrc->widthStep*j + i]*2.0 + dMean*0.9;
		}
	}
	cvSaveImage("D:\\test.bmp", pImgSrc);
	*/

/*	double dMean = 0;
	double dVar = 0;
	int R = 1;
	int nMax = 0;
	int nValue = 0;
	for (int i=R; i<nW-R; i++)
	{
		for (int j=R; j<nH-R; j++)
		{
			nMax = 0;
			dMean = 0;
			dVar = 0;
			//mean 
			for (int ii=-R; ii<=R; ii++)
			{
				for (int jj=-R; jj<=R; jj++)
				{
					nValue = pImgSrc->imageData[pImgSrc->widthStep*(j+jj) + (i+ii)];
					//nMax = nMax>nValue? nMax : nValue;
					dMean += nValue;
				}
			}
			dMean /= pow((2*R+1),2.0);

			//var 
			for (int ii=-R; ii<=R; ii++)
			{
				for (int jj=-R; jj<=R; jj++)
				{
					nValue = pImgSrc->imageData[pImgSrc->widthStep*(j+jj) + (i+ii)];
					dVar += pow((nValue-dMean),2);
				}
			}
			nValue = pImgSrc->imageData[pImgSrc->widthStep*j + i];
			pImgDst->imageData[pImgDst->widthStep*j + i] = (char)(nValue-dMean)/sqrt(dVar);
		}
	}*/
	
	

/*	CascadeClassifier cFaceCascade;
	if (!cFaceCascade.load(eyeCasName))
	{ 
		printf("[error] 无法加载级联分类器文件！\n"); 
		return; 
	}  

	vector<Rect> vFaceRect;
	cFaceCascade.detectMultiScale(pImgSrc, vFaceRect, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30), Size(80, 80)); 

	for (int i=0; i<vFaceRect.size(); i++)
	{
		Rect r = vFaceRect.at(i);
		cvRectangle(pImgSrc, cvPoint(r.x, r.y), cvPoint(r.x+r.width, r.y+r.height), Scalar(0, 255, 0), 1);
	}
*/
	//sift 特征
/*	SIFT sift;    //实例化SIFT类  

	vector<KeyPoint> key_points;    //特征点  
	// descriptors为描述符，mascara为掩码矩阵  
	Mat descriptors, mascara;  
	Mat output_img;    //输出图像矩阵  

	//Mat img = imread("E:\\face_file\\sampleData\\face\\sample\\1\\1.bmp");  
	sift((Mat)pImgSrc, mascara, key_points,descriptors);    //执行SIFT运算  
	//在输出图像中绘制特征点  
	drawKeypoints(pImgSrc,     //输入图像  
		key_points,      //特征点矢量  
		output_img,      //输出图像  
		Scalar::all(-1),      //绘制特征点的颜色，为随机  
		//以特征点为中心画圆，圆的半径表示特征点的大小，直线表示特征点的方向  
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS);  


	imwrite("D:\\test.bmp", output_img); */
}

Mat CImageProcessClass::clustering(Mat src, int ClusterNum) 
{
	IplImage *pImgSrc = cvLoadImage( "E:\\face_file\\sampleData\\face\\sample\\1\\1.bmp", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
	IplImage* imgA = cvCreateImage(cvSize(100,100), 8, 1);
	cvZero(imgA);
	cvResize(pImgSrc, imgA,CV_INTER_CUBIC);
	int nW = imgA->width;
	int nH = imgA->height;

	CvMat* A = cvCreateMat(nW,nH,CV_32FC1);
	CvMat* U = cvCreateMat(nW,nH,CV_32FC1); 
	CvMat* D = cvCreateMat(nW,nH,CV_32FC1); 
	CvMat* V = cvCreateMat(nW,nH,CV_32FC1); 
	
	cvConvert(imgA, A);
	cvSVD(A, D, U, V, CV_SVD_U_T|CV_SVD_V_T); // A = U D V^T 
	cvConvert(U, imgA);

	int row = 100;//src.rows;  
    int col = 100;//src.cols;  
    unsigned long int size = row*col;  

	CvMat *clusters;//分类后的矩阵
	clusters = cvCreateMat (size, 1, CV_32SC1);//32位1通道的矩阵
	CvMat *points;//分类前的样例浮点矩阵
	points = cvCreateMat (size, 1, CV_32FC3); //32位3通道的矩阵

	unsigned long int i; 
	for (i = 0; i < size; i++) 
	{
		points->data.fl[i*3] = (unsigned char) imgA->imageData[i*3];
		points->data.fl[i*3 + 1] = (unsigned char) imgA->imageData[i*3 + 1];
		points->data.fl[i*3 + 2] = (unsigned char) imgA->imageData[i*3 + 2]; 
	} //得到三通道图像的数据

	cvKMeans2 (points, 3, clusters,
		cvTermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0));
	//拆分为8类聚合，最大迭代次数是10，精度是1.0

	CvMat *color = cvCreateMat (ClusterNum, 1, CV_32FC3);//8行1列的三通道浮点矩阵
	CvMat *count = cvCreateMat (ClusterNum, 1, CV_32SC1);//8行1列的单通道整数矩阵，用作计数
	cvSetZero (color);
	cvSetZero (count);

	for (i = 0; i < size; i++)
	{
		int idx = clusters->data.i[i];
		int j = ++count->data.i[idx];
		color->data.fl[idx * 3 ] = color->data.fl[idx * 3 ] * (j - 1) / j + points->data.fl[i * 3 ] / j;
		color->data.fl[idx * 3 + 1] = color->data.fl[idx * 3 + 1] * (j - 1) / j + points->data.fl[i * 3 + 1] / j;
		color->data.fl[idx * 3 + 2] = color->data.fl[idx * 3 + 2] * (j - 1) / j + points->data.fl[i * 3 + 2] / j;
	}

	//把处理过的数据打回imgA
	for (i = 0; i < size; i++)
	{
		int idx = clusters->data.i[i];
		imgA->imageData[i * 3 ] = (char) color->data.fl[idx * 3 ]*255;
		imgA->imageData[i * 3 + 1] = (char) color->data.fl[idx * 3 + 1]*255;
		imgA->imageData[i * 3 + 2] = (char) color->data.fl[idx * 3 + 2]*255;
	}

	//imwrite("D:\\test1.bmp",imgA);
	cvSaveImage("D:\\test1.bmp",imgA);
	Mat label_show;  
	return label_show;
 /*   Mat clusters(size, 1, CV_32SC1);    //clustering Mat, save class label at every location;  
  
    //convert src Mat to sample srcPoint.  
    Mat srcPoint(size, 1, CV_32FC3);   /////////
	Vec3f* srcPoint_p = (Vec3f*)srcPoint.data;//////////////////////////////////////////////  
    Vec3f* src_p = (Vec3f*)src.data;  
    unsigned long int i;  
   
    for(i = 0;i < size; i++)  
    {  
        *srcPoint_p = *src_p;  
        srcPoint_p++;  
        src_p++;  
    }  
    Mat center(ClusterNum,1,CV_32FC3);  
    double compactness;//compactness to measure the clustering center dist sum by different flag  
    compactness = kmeans(srcPoint, ClusterNum, clusters,  
        cvTermCriteria (CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.1),ClusterNum,  
        KMEANS_PP_CENTERS , center);  
    
    cout<<"center row:"<<center.rows<<" col:"<<center.cols<<endl;  
    for (int y = 0; y < center.rows; y++)   
    {  
        Vec3f* imgData = center.ptr<Vec3f>(y);  
        for (int x = 0; x < center.cols; x++)  
        {  
            cout<<imgData[x].val[0]<<" "<<imgData[x].val[1]<<" "<<imgData[x].val[2]<<endl;  
        }  
        cout<<endl;  
    }  

	double minH,maxH;  
    minMaxLoc(clusters, &minH, &maxH);          //remember must use "&"  
    cout<<"H-channel min:"<<minH<<" max:"<<maxH<<endl;  
  
    int* clusters_p = (int*)clusters.data;  
    //show label mat  
    Mat label(src.size(), CV_32SC1);  
    int* label_p = (int*)label.data;  
    //assign the clusters to Mat label  
    for(i = 0;i < size; i++)  
    {  
        *label_p = *clusters_p;  
        label_p++;  
        clusters_p++;  
    }  
  
    Mat label_show;  
    label.convertTo(label_show,CV_8UC1);  
    normalize(label_show,label_show,255,0,CV_MINMAX);  
    //imshow("label",label_show); 
	
	imwrite("D:\\test1.bmp",label_show);
	return label_show;*/
}
void CImageProcessClass::KMean_new(IplImage* pImgSrc, int cluster_count,
           IplImage* pImgLabel, CvTermCriteria termcrit)  
{  
	int nH = pImgSrc->height;
	int nW = pImgSrc->width;
	//step1:随机产生cluster_count个中心点
	Point pt_Center[10];
	RNG rng(12345);

	for(int k=0; k<cluster_count; k++)
	{
		Point center;
		center.x = rng.uniform(0, pImgSrc->width);
		center.y = rng.uniform(0, pImgSrc->height);
		pt_Center[k] = center;
	}
	
	IplImage* pImgLabel_best = cvCreateImage(cvGetSize(pImgSrc), 8, 1);
	cvZero(pImgLabel_best);
    cvZero(pImgLabel);

	double dValue = 0;
	double dValue_center = 0;
	double dDis_min = 1000, dDis = 0;
	int nLabel_min_dis = 0;
	//step2:计算每个对象与这些中心对象的距离；并根据最小距离重新对相应对象进行分类；
	for (int i=0; i<nW; i++)
	{
		for (int j=0; j<nH; j++)
		{
			dValue = pImgSrc->imageData[pImgSrc->widthStep*j + i];
			dDis_min = 1000;
			for(int k=0; k<cluster_count; k++)
			{
				dValue_center = pImgSrc->imageData[pImgSrc->widthStep*pt_Center[k].y + pt_Center[k].x];
				dDis = fabs(dValue - dValue_center);
				if (dDis_min > dDis)
				{
                    dDis_min = dDis;
					nLabel_min_dis = k;
				}
				pImgLabel_best->imageData[pImgLabel_best->widthStep*j + i] = nLabel_min_dis;
			}
		}
	}

	//step3:
}

void CImageProcessClass::ImgSurfaceNormalVectorMatrix(string sPath, int nH, int nW, Mat &mat_normal_vector)//表面法向量矩阵
{
	CCommonFunClass fun;
	vector<string> vsFolder, vsFileName[1000];
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	string s = "";
	int person_num = (int)vsFolder.size();
	int every_person_image_num = 0;
	int nSize = nW*nH;
	int N = 0;
	Mat mat;
	Mat mat_one_col(nSize, 1, CV_32FC1); //存放图像转换成一列
	mat_one_col = Mat::zeros(mat_one_col.rows, mat_one_col.cols, mat_one_col.type());
	Mat mat_standard(nH, nW, CV_32FC1); //存放图像大小归一化后的图
	mat_standard = Mat::zeros(mat_standard.rows, mat_standard.cols, mat_standard.type());
	mat_normal_vector = Mat::zeros(mat_normal_vector.rows, mat_normal_vector.cols, mat_normal_vector.type());

	double d = 0;
	for (int i=0; i<person_num; i++)
	{
		every_person_image_num = (int)vsFileName[i].size();
		Mat mat_person(nSize, every_person_image_num, CV_32FC1);
		mat_person = Mat::zeros(mat_person.rows, mat_person.cols, mat_person.type());
		
		for (int j=0; j<every_person_image_num; j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			mat = imread(s.c_str(), 0);
			resize(mat, mat_standard, Size(nW,nH), 0, 0, CV_INTER_LINEAR); //转到标准大小，如：100*100
			for (int row=0; row<mat_standard.rows; row++) //转换到一列
			{
				for (int col=0; col<mat_standard.cols; col++)
				{
					mat_one_col.at<float>(row*mat_standard.cols+col,0) = mat_standard.at<uchar>(row,col);		
				}
			}
	
			mat_one_col.col(0).copyTo(mat_person.col(j)) ; //整列赋值
		}

		CvMat* A = cvCreateMat(nSize, every_person_image_num, CV_32FC1);
		CvMat* U = cvCreateMat(nSize, every_person_image_num, CV_32FC1); 
		CvMat* D = cvCreateMat(every_person_image_num, every_person_image_num, CV_32FC1); 
		CvMat* V = cvCreateMat(every_person_image_num, every_person_image_num, CV_32FC1); 
		cvConvert(&IplImage(mat_person), A); // Mat->CvMat*
		cvSVD(A, D, U, V);//, CV_SVD_U_T|CV_SVD_V_T); // A = U D V^T 

		Mat tempMat = Mat(U, true); //CvMat->Mat
		//用前3维左奇异向量代表法向量，特征值是从大到小排序的，大的特征值对应的特征向量代表更对细节
		//求和
		for (int k=0; k<3; k++)
		{	
			mat_normal_vector.col(k) += tempMat.col(k);// + 0; 
		}
		cvReleaseMat(&A);
		cvReleaseMat(&U);
		cvReleaseMat(&D);
		cvReleaseMat(&V);
	}

	//求平均表面法向量矩阵
	for (int k=0; k<3; k++)
	{
		mat_normal_vector.col(k) /= person_num; 
	}
	///////


	//////////////////////////////////////////////////////
	//显示平均表面法向量矩阵分列，再转成一副图像
	int nNum = 0;
	int col = 0;
	float fValue = 0;
	Mat mat0(nH, nW, CV_32FC1);
	Mat mat1(nH, nW, CV_32FC1);
	Mat mat2(nH, nW, CV_32FC1);
	for (int row=0; row<nH*nW; row++)
	{
		mat0.at<float>(row/nW, row%nW) = mat_normal_vector.at<float>(row,0);
		mat1.at<float>(row/nW, row%nW) = mat_normal_vector.at<float>(row,1);
		mat2.at<float>(row/nW, row%nW) = mat_normal_vector.at<float>(row,2);
	}
	double dMin, dMax;
	double dDis = 1;
	Mat mat00(nH, nW, CV_8UC1);
	Mat mat11(nH, nW, CV_8UC1);
	Mat mat22(nH, nW, CV_8UC1);
	minMaxLoc(mat0, &dMin, &dMax);	
	dDis = dMax - dMin + 0.000000001;	
	for (int row=0; row<nH; row++)
	{
		for (int col=0; col<nW; col++)
		{
			if (dMin < 0)
			{
				mat0.at<float>(row, col) = mat0.at<float>(row, col) - dMin + 0;
			}
			mat00.at<uchar>(row, col) = (uchar)(mat0.at<float>(row, col)/dDis*255);
		}
	}
	imwrite("D:\\test00.bmp", mat00);

	minMaxLoc(mat1, &dMin, &dMax);
	dDis = dMax - dMin + 0.000000001;	
	for (int row=0; row<nH; row++)
	{
		for (int col=0; col<nW; col++)
		{
			if (dMin < 0)
			{
				mat1.at<float>(row, col) = mat1.at<float>(row, col) - dMin + 0;
			}
			mat11.at<uchar>(row, col) = (uchar)(mat1.at<float>(row, col)/dDis*255);
		}
	}
	imwrite("D:\\test11.bmp", mat11);

	minMaxLoc(mat2, &dMin, &dMax);
	dDis = dMax - dMin + 0.000000001;	
	for (int row=0; row<nH; row++)
	{
		for (int col=0; col<nW; col++)
		{
			if (dMin < 0)
			{
				mat2.at<float>(row, col) = mat2.at<float>(row, col) - dMin + 0;
			}
			mat22.at<uchar>(row, col) = (uchar)(mat2.at<float>(row, col)/dDis*255);
		}
	}
	imwrite("D:\\test22.bmp", mat22);

}

void CImageProcessClass::Clustering(const Mat &src, int K, Mat &_bestLabels,
                         TermCriteria criteria, Mat _pMat[])
{
	_bestLabels = Mat::zeros(_bestLabels.rows, _bestLabels.cols, _bestLabels.type());
	int nH = src.rows; //图像大小
	int nW = src.cols;

	Mat mat_center(K, nW, CV_32FC1);
	mat_center = Mat::zeros(mat_center.rows, mat_center.cols, mat_center.type());
	Mat mat_center_old(K, nW, CV_32FC1);
	mat_center_old = Mat::zeros(mat_center_old.rows, mat_center_old.cols, mat_center_old.type());

	int col = 0, row = 0;
	double d = 0;
	int n = 0;
	int nNum = 0;
	float fValue = 0;
	int k = 0;
	//给聚类中心赋值，任意的，不影响最后的聚类结果
	for (row=0; row<nH; row++)
	{
		for (col=0; col<nW; col++)
		{
			fValue = src.at<float>(row,col);
			if (fValue>0.0000001 || fValue<-0.0000001)
			{
				nNum++;
			}
		}
		if (nNum > 0)
		{
			mat_center.row(k) = src.row(row) + 0;
			mat_center_old.row(k) = mat_center.row(k) + 0;
			k++;
			nNum = 0;
		}
	    if (k >= K)
		{
			row = nH + 1;
		}
	}

	Mat mat_mean_num(K, 1, CV_32FC1);
	Mat mat_mean(K, 1, CV_32FC1);
	Mat mat_mean_old(K, 1, CV_32FC1);

	mat_mean_num = Mat::zeros(mat_mean_num.rows, mat_mean_num.cols, mat_mean_num.type());
	mat_mean = Mat::zeros(mat_mean.rows, mat_mean.cols, mat_mean.type());
	mat_mean_old = Mat::zeros(mat_mean_old.rows, mat_mean_old.cols, mat_mean_old.type());

	int nLabel = 0;
	double dDis = 0;
	double dMinDis = 0;
	double dMaxDis = 0;

	Mat mat_row(1, 3, CV_32FC1);
	Mat mat_row_center(1, 3, CV_32FC1);
	mat_row = Mat::zeros(mat_row.rows, mat_row.cols, mat_row.type());
	mat_row_center = Mat::zeros(mat_row_center.rows, mat_row_center.cols, mat_row_center.type());
    Mat mat_dis_label(nH, 2, CV_32FC1); //第一列存放距离，第二列存放簇号
	mat_dis_label = Mat::zeros(mat_dis_label.rows, mat_dis_label.cols, mat_dis_label.type());

	double dis[10] = {0};
	CCommonFunClass fun;
	string s;
	char buffer[MAX_PATH] = {0};
	int nIter = 0;
	double dEpsilon = 1000;
	while (nIter<=criteria.maxCount && dEpsilon>criteria.epsilon)
	{
		sprintf(buffer, "%s", "中心点");
		fun.WriteTxt("D:\\1.txt", buffer);

		for (int k=0; k<K; k++)
		{
			sprintf(buffer, "%d, %d, %.4f, %.4f, %.4f", nIter, k, mat_center.at<float>(k,0), mat_center.at<float>(k,1),mat_center.at<float>(k,2));
		    fun.WriteTxt("D:\\1.txt", buffer);
		}
		mat_mean_num = Mat::zeros(mat_mean_num.rows, mat_mean_num.cols, mat_mean_num.type());
		mat_mean = Mat::zeros(mat_mean.rows, mat_mean.cols, mat_mean.type());
		for (row=0; row<nH; row++)
		{
			dMinDis = 1000;
			mat_row.row(0) = src.row(row) + 0;

			for (int k=0; k<K; k++)
			{
				mat_row_center.row(0) = mat_center.row(k) + 0;
				dDis = getDistance(mat_row_center, mat_row);
				dis[k] = dDis;
				//最小距离分类
				if (dDis < dMinDis)
				{
					nLabel = k;
					dMinDis = dDis;
					_bestLabels.at<uchar>(row,0) = nLabel;
				}
			}
			sprintf(buffer, "%d, %.4f, %.4f, %.4f, %d", row, dis[0], dis[1], dis[2], _bestLabels.at<uchar>(row,0));
			fun.WriteTxt("D:\\2.txt", buffer);
			//统计均值
			mat_mean.at<float>(nLabel,0) += dMinDis + 0;
			mat_mean_num.at<float>(nLabel,0) += 1.0 + 0;
			mat_dis_label.at<float>(row,0) = (float)dMinDis; //存放每个点对应的最小聚类距离
			mat_dis_label.at<float>(row,1) = (float)nLabel;		
		}
		sprintf(buffer, "%s", "均值");
		fun.WriteTxt("D:\\1.txt", buffer);
		//求簇的均值
		for (row=0; row<K; row++)
		{
			mat_mean.at<float>(row,0) = mat_mean.at<float>(row,0)/(mat_mean_num.at<float>(row,0)+0.00000000001);
		}
	    for (row=0; row<nH; row++)
		{
			sprintf(buffer, "%d, %.4f, %4f", row, mat_dis_label.at<float>(row,0), mat_dis_label.at<float>(row,1));
			//fun.WriteTxt("D:\\1.txt", buffer);
		}
		sprintf(buffer, "%d, %.4f, %.4f, %.4f", row, mat_mean.at<float>(0,0), mat_mean.at<float>(1,0), mat_mean.at<float>(2,0));
		fun.WriteTxt("D:\\1.txt", buffer);

		//计算簇的中心点，在此以簇中所有对象的平均距离来计算中心点
		dMaxDis = 0;
		for (row=0; row<K; row++)
		{
			dDis = fabs(mat_mean.at<float>(row,0) - mat_mean_old.at<float>(row,0));
			dMaxDis = dDis>dMaxDis? dDis : dMaxDis;
		}

		dEpsilon = dMaxDis;
		mat_center = Mat::zeros(mat_center.rows, mat_center.cols, mat_center.type());
		//交换均值、中心点
		nLabel = 0;
		
		if (dMaxDis>criteria.epsilon)
		{
			mat_mean_old.col(0) = mat_mean.col(0) + 0;
			//中心点, 即最接近聚类均值的点
			for (col=0; col<K; col++)
			{
				dMinDis = 1000;
				for (row=0; row<nH; row++)
				{
					nNum = 0;
					if (col == mat_dis_label.at<float>(row,1))
					{
						dDis = fabs(mat_dis_label.at<float>(row,0) - mat_mean_old.at<float>(col,0));
						if (dDis < dMinDis)
						{
							dMinDis = dDis;
							nLabel = row;
						}
					}
				}
				mat_center.row(col) = src.row(nLabel) + 0;		
			}
		}
		nIter++;
	}

	nH = (int)sqrt(nH+0.0001);
	uchar c = 0;
	for (int row=0; row<nH; row++)
	{
		for (int col=0; col<nH; col++)
		{
            c = _bestLabels.at<uchar>(row*nH+col,0);
			_pMat[c].at<float>(row,col) = 255;
		}
	}

	string sTemp = "D:\\";
	for (int i=0; i<K; i++)
	{
		sprintf(buffer, "clustering_%d.bmp", i);
		s = sTemp + buffer;
		imwrite(s.c_str(), _pMat[i]);
	}
}

void CImageProcessClass::Clustering_zero(const Mat &src, int K, Mat &_bestLabels,
                         TermCriteria criteria, Mat _pMat[])
{
	_bestLabels = Mat::zeros(_bestLabels.rows, _bestLabels.cols, _bestLabels.type());
	int nH = src.rows; //图像大小
	int nW = src.cols;

	CCommonFunClass fun;
	string s;
	char buffer[MAX_PATH] = {0};
	int nLen = 0;
	for (int k=0; k<nH; k++)
	{
		sprintf(buffer, "%d, %.4f, %.4f, %.4f", k, src.at<float>(k,0), src.at<float>(k,1), src.at<float>(k,2));
		fun.WriteTxt("D:\\src.txt", buffer);
	}

	Mat mat_center(K, nW, CV_32FC1);
	mat_center = Mat::zeros(mat_center.rows, mat_center.cols, mat_center.type());
	Mat mat_center_old(K, nW, CV_32FC1);
	mat_center_old = Mat::zeros(mat_center_old.rows, mat_center_old.cols, mat_center_old.type());

	int col = 0, row = 0;
	double d = 0;
	int n = 0;
	int nNum = 0;
	float fValue = 0;
	int k = 0;
	//给聚类中心赋值，任意的，不影响最后的聚类结果
	for (k=0; k<K; k++)
	{
		row = nH/K*k;
		mat_center.row(k) = src.row(row) + 0;
		mat_center_old.row(k) = mat_center.row(k) + 0;
	}

/*	for (row=0; row<nH; row++)
	{
		for (col=0; col<nW; col++)
		{
			fValue = src.at<float>(row,col);
			if (fValue>0.0000001 || fValue<-0.0000001)
			{
				nNum++;
			}
		}
		if (nNum > 0)
		{
			mat_center.row(k) = src.row(row) + 0;
			mat_center_old.row(k) = mat_center.row(k) + 0;
			k++;
			nNum = 0;
		}
	    if (k >= K)
		{
			row = nH + 1;
		}
	}*/

	Mat mat_mean_num(K, 1, CV_32FC1);
	Mat mat_mean(K, 1, CV_32FC1);
	Mat mat_mean_old(K, 1, CV_32FC1);

	mat_mean_num = Mat::zeros(mat_mean_num.rows, mat_mean_num.cols, mat_mean_num.type());
	mat_mean = Mat::zeros(mat_mean.rows, mat_mean.cols, mat_mean.type());
	mat_mean_old = Mat::zeros(mat_mean_old.rows, mat_mean_old.cols, mat_mean_old.type());

	int nLabel = 0;
	double dDis = 0;
	double dMinDis = 0;
	double dMaxDis = 0;

	Mat mat_row(1, 3, CV_32FC1);
	Mat mat_row_center(1, 3, CV_32FC1);
	mat_row = Mat::zeros(mat_row.rows, mat_row.cols, mat_row.type());
	mat_row_center = Mat::zeros(mat_row_center.rows, mat_row_center.cols, mat_row_center.type());
    Mat mat_dis_label(nH, 2, CV_32FC1); //第一列存放距离，第二列存放簇号
	mat_dis_label = Mat::zeros(mat_dis_label.rows, mat_dis_label.cols, mat_dis_label.type());

	double dis[10] = {0};
	int nIter = 0;
	double dEpsilon = 1000;
	while (nIter<=criteria.maxCount && dEpsilon>criteria.epsilon)
	{
		sprintf(buffer, "%s", "中心点");
		fun.WriteTxt("D:\\1.txt", buffer);

		for (int k=0; k<K; k++)
		{
			sprintf(buffer, "%d, %d, %.4f, %.4f, %.4f", nIter, k, mat_center.at<float>(k,0), mat_center.at<float>(k,1),mat_center.at<float>(k,2));
		    fun.WriteTxt("D:\\1.txt", buffer);
		}
		mat_mean_num = Mat::zeros(mat_mean_num.rows, mat_mean_num.cols, mat_mean_num.type());
		mat_mean = Mat::zeros(mat_mean.rows, mat_mean.cols, mat_mean.type());
		for (row=0; row<nH; row++)
		{
			dMinDis = 1000;
			mat_row.row(0) = src.row(row) + 0;

			for (int k=0; k<K; k++)
			{
				mat_row_center.row(0) = mat_center.row(k) + 0;
				dDis = getDistance(mat_row_center, mat_row);
				dis[k] = dDis;
				//最小距离分类
				if (dDis < dMinDis)
				{
					nLabel = k;
					dMinDis = dDis;
					_bestLabels.at<uchar>(row,0) = nLabel;
				}
			}
			sprintf(buffer, "%d, %.4f, %.4f, %.4f, %d", row, dis[0], dis[1], dis[2], _bestLabels.at<uchar>(row,0));
			fun.WriteTxt("D:\\2.txt", buffer);
			//统计均值
			mat_mean.at<float>(nLabel,0) += dMinDis + 0;
			mat_mean_num.at<float>(nLabel,0) += 1.0 + 0;
			mat_dis_label.at<float>(row,0) = (float)dMinDis; //存放每个点对应的最小聚类距离
			mat_dis_label.at<float>(row,1) = (float)nLabel;		
		}
		sprintf(buffer, "%s", "均值");
		fun.WriteTxt("D:\\1.txt", buffer);
		//求簇的均值
		for (row=0; row<K; row++)
		{
			mat_mean.at<float>(row,0) = mat_mean.at<float>(row,0)/(mat_mean_num.at<float>(row,0)+0.00000000001);
		}
	    for (row=0; row<nH; row++)
		{
			sprintf(buffer, "%d, %.4f, %4f", row, mat_dis_label.at<float>(row,0), mat_dis_label.at<float>(row,1));
			//fun.WriteTxt("D:\\1.txt", buffer);
		}
		sprintf(buffer, "%d, %.4f, %.4f, %.4f", row, mat_mean.at<float>(0,0), mat_mean.at<float>(1,0), mat_mean.at<float>(2,0));
		fun.WriteTxt("D:\\1.txt", buffer);

		//计算簇的中心点，在此以簇中所有对象的平均距离来计算中心点
		dMaxDis = 0;
		for (row=0; row<K; row++)
		{
			dDis = fabs(mat_mean.at<float>(row,0) - mat_mean_old.at<float>(row,0));
			dMaxDis = dDis>dMaxDis? dDis : dMaxDis;
		}

		dEpsilon = dMaxDis;
		mat_center = Mat::zeros(mat_center.rows, mat_center.cols, mat_center.type());
		//交换均值、中心点
		nLabel = 0;
		
		if (dMaxDis>criteria.epsilon)
		{
			mat_mean_old.col(0) = mat_mean.col(0) + 0;
			//中心点, 即最接近聚类均值的点
			for (col=0; col<K; col++)
			{
				dMinDis = 1000;
				for (row=0; row<nH; row++)
				{
					nNum = 0;
					if (col == mat_dis_label.at<float>(row,1))
					{
						dDis = fabs(mat_dis_label.at<float>(row,0) - mat_mean_old.at<float>(col,0));
						if (dDis < dMinDis)
						{
							dMinDis = dDis;
							nLabel = row;
						}
					}
				}
				mat_center.row(col) = src.row(nLabel) + 0;		
			}
		}
		nIter++;
	}

	nH = (int)sqrt(nH+0.0001);
	uchar c = 0;
	for (int row=0; row<nH; row++)
	{
		for (int col=0; col<nH; col++)
		{
            c = _bestLabels.at<uchar>(row*nH+col,0);
			_pMat[c].at<float>(row,col) = 1;
		}
	}

	string sTemp = "D:\\";
	for (int i=0; i<K; i++)
	{
		sprintf(buffer, "clustering_%d.bmp", i);
		s = sTemp + buffer;
		imwrite(s.c_str(), _pMat[i]);
	}
}
void CImageProcessClass::getSLNImage(Mat &src, int K, Mat _pMat[], Mat &_dst)
{
	int nCol = src.cols;
	int nRow = src.rows;
	///////////
	Mat mat(nRow, nCol, CV_32FC1);
	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=0; row<10; row++)
	{
		for (int col=0; col<nCol; col++)
		{
			_pMat[0].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah0.bmp", _pMat[0]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=10; row<nRow/3; row++)
	{
		for (int col=0; col<nCol/2; col++)
		{
			_pMat[1].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah1.bmp", _pMat[1]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=10; row<nRow/3; row++)
	{
		for (int col=nCol/2; col<nCol; col++)
		{
			_pMat[2].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah2.bmp", _pMat[2]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=nRow/3; row<2*nRow/3; row++)
	{
		for (int col=1*nCol/4; col<3*nCol/4; col++)
		{
			_pMat[3].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah3.bmp", _pMat[3]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=nRow/3; row<2*nRow/3; row++)
	{
		for (int col=0; col<1*nCol/4; col++)
		{
			_pMat[4].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah4.bmp", _pMat[4]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=nRow/3; row<2*nRow/3; row++)
	{
		for (int col=3*nCol/4; col<nCol; col++)
		{
			_pMat[5].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah5.bmp", _pMat[5]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=2*nRow/3; row<nRow; row++)
	{
		for (int col=1*nCol/6; col<5*nCol/6; col++)
		{
			_pMat[6].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah6.bmp", _pMat[6]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=2*nRow/3; row<nRow; row++)
	{
		for (int col=0; col<1*nCol/6; col++)
		{
			_pMat[7].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah7.bmp", _pMat[7]);

	mat = Mat::zeros(mat.rows, mat.cols, mat.type());
	for (int row=2*nRow/3; row<nRow; row++)
	{
		for (int col=5*nCol/6; col<nCol; col++)
		{
			_pMat[8].at<float>(row,col) = 1;
		}
	}
	imwrite("D:\\hah8.bmp", _pMat[8]);

	
	//一个脸整体
/*	Mat mat(nRow, nCol, CV_32FC1);
	mat = Mat::ones(mat.rows, mat.cols, mat.type());
	_pMat[0] = mat;*/

	//y方向分成3个区域
	/*for (int k=0; k<K; k++)
	{
		Mat mat = _pMat[k];
		for (int row=0; row<nRow/K; row++)
		{
			mat.row(row+nRow/K*k) = 1;
		}
		_pMat[k] = mat;
	}*/
	//////////
	vector<Mat> vMat;
	Mat mean;
	Mat var;
	double dMean = 0;
	double dVar = 0;
	char buffer[MAX_PATH] = {0};
	string s = "";
	double d = 0;
	float* pSrc; 
	float* pDst;
	float* pTemp;
	for (int i=0; i<K; i++)
	{
		src.convertTo(src, CV_32FC1);
		imwrite("D:\\src.bmp", src);

		_pMat[i].convertTo(_pMat[i], CV_32FC1);
		Mat mat_temp = src.mul(_pMat[i]);
		/////
		sprintf(buffer, "D:\\mask_%d.bmp", i);
		s = buffer;
		imwrite(s.c_str(), mat_temp);
		/////	
		int nNum = 0;
		dMean = 0;
		dVar = 0;
	    getMaskMeanVar(mat_temp, _pMat[i], dMean, dVar);

		//
		if (dVar>-0.00000001 && dVar<0.00000001)
		{
			mat_temp = Mat::zeros(mat_temp.rows, mat_temp.cols, mat_temp.type());
		}
		else
		{
			for (int row=0; row<mat_temp.rows; row++)
			{
				pSrc = mat_temp.ptr<float>(row);
				for (int col=0; col<mat_temp.cols; col++)
				{				
					d = pSrc[col];
					if (d > 0.000001)
					{
						d = (d - dMean)/dVar;
						pSrc[col] = d;
						d = pSrc[col];
					}				
				}
			}
			//
		/*	double dMax = 0;
			double dMin = 0;
			minMaxLoc(mat_temp, &dMin, &dMax);
			double dDis = dMax - dMin + 0.0000001;
			for (int row=0; row<mat_temp.rows; row++)
			{
				pSrc = mat_temp.ptr<float>(row);
				for (int col=0; col<mat_temp.cols; col++)
				{				
					d = pSrc[col];
					if (d > 0.000001)
					{
						d = d*255/dDis;//(d - dMean)/dVar;
						pSrc[col] = d;
						d = pSrc[col];
					}				
				}
			}*/
			imwrite("D:\\temp.bmp", mat_temp);
		}
		vMat.push_back(mat_temp);
	}

	_dst = Mat::zeros(_dst.rows, _dst.cols, _dst.type());
	_dst.convertTo(_dst, CV_32FC1);
	
	for (int row=0; row<src.rows; row++)
	{	
		for (int i=0; i<K; i++)
		{
			Mat mat = vMat.at(i);
           _dst.row(row) += mat.row(row) + 0;
		}
	}

	double dMax = 0;
	double dMin = 0;
	minMaxLoc(_dst, &dMin, &dMax);
	double dDis = dMax - dMin + 0.0000001;
    for (int row=0; row<src.rows; row++)
	{
		if (dMin < 0)
		{
			_dst.row(row) = _dst.row(row) - dMin + 0;
		}
		_dst.row(row) = _dst.row(row)*255/dDis + 0;
	} 
	imwrite("D:\\dst.bmp", _dst);
}

void CImageProcessClass::testSLN(void)
{
	string sPath = "E:\\faceDetect\\Debug\\FaceSample\\Sample_";//E:\\face_file\\sampleData\\face\\test"; 
	string sStandardImage = "";
	sStandardImage = "E:\\faceDetect\\Debug\\FaceSample\\standard.bmp";
	IplImage* pImgStandard = cvLoadImage(sStandardImage.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	int nH = pImgStandard->height;
	int nW = pImgStandard->width;
	int nSize = nH * nW;
	int K = 9;
	Mat mat_normal_vector(nSize, 3, CV_32FC1); //存放表面法向量
	//ImgSurfaceNormalVectorMatrix(sPath, nH, nW, mat_normal_vector);

	Mat bestLabels(mat_normal_vector.rows, 1, CV_8UC1);
	Mat centers(K, 1, CV_32FC1);
	Mat pMat[9];
	for (int i=0; i<K; i++)
	{
	   pMat[i] = Mat::Mat(nH, nW, CV_32FC1);
	   pMat[i] = Mat::zeros(pMat[i].rows, pMat[i].cols, pMat[i].type());
	}
	
	//Clustering(mat_normal_vector, K, bestLabels,
    //                cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.01), pMat);
    //E:\\face_file\\sampleData\\face\\sample\\1\\1.bmp
	CCommonFunClass fun;
	vector<string> vsFolder, vsFileName[1000];
	sPath = "E:\\faceDetect\\Debug\\FaceSample\\Test_";//Sample_
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	string s = "";
	Mat mat;
	string sPath1 = "E:\\faceDetect\\Debug\\FaceSample\\Test";//";Sample
	for (int i=0; i<vsFolder.size(); i++)
	{
		s = sPath1 + "\\" + vsFolder.at(i);
		::CreateDirectory(s.c_str(), NULL);
		for (int j=0; j<vsFileName[i].size(); j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			mat = imread(s.c_str(), 0);
			Mat mat_standard(nH, nW, CV_32FC1); //存放图像大小归一化后的图
			resize(mat, mat_standard, Size(nH, nW), 0, 0, CV_INTER_LINEAR); //转到标准大小，如：100*100
			getSLNImage(mat_standard, K, pMat, mat_standard);
			s = sPath1 + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			imwrite(s.c_str(), mat_standard);
		}
	}
}

double CImageProcessClass::getDistance(const Mat mat1, const Mat mat2)
{
	//return 1 - mat1.dot(mat2)/(sqrt(mat1.dot(mat1))*sqrt(mat2.dot(mat2))+0.0000000001);
	return 1 - mat1.dot(mat2)/(getMod(mat1)*getMod(mat2) + 0.00000000001);
}

double CImageProcessClass::getMod(const Mat &src)//求模
{
	//int nRow = src.rows;
	//int nCol = src.cols;

	double dMod = 0;
	/*double dSum = 0;
	for (int i=0; i<nRow; i++)
	{
		for (int j=0; j<nCol; j++)
		{		
			dSum += pow(src.at<float>(i,j), 2);
		}
	}

	dMod = sqrt(dSum);*/

	dMod  = src.dot(src);
	dMod = sqrt(dMod);

	return dMod;
}

void CImageProcessClass::getMaskMeanVar(Mat src, Mat mask, double &_dMean, double &_dVar)
{
	src.convertTo(src, CV_32FC1);
	mask.convertTo(mask, CV_8UC1);
	float* pSrc;
	uchar* pMask;
	_dMean = 0;
	_dVar = 0;
	int nNum = 0;
	for (int row=0; row<src.rows; row++)
	{
		pSrc = src.ptr<float>(row);
		pMask = mask.ptr<uchar>(row);
		for (int col=0; col<src.cols; col++)
		{
			if (pMask[col] > 0)
			{
				nNum++;
				_dMean += pSrc[col];
			}
		}
	}
	_dMean /= nNum;

	for (int row=0; row<src.rows; row++)
	{
		pSrc = src.ptr<float>(row);
		pMask = mask.ptr<uchar>(row);
		for (int col=0; col<src.cols; col++)
		{
			if (pMask[col] > 0)
			{
				_dVar += pow((pSrc[col]-_dMean),2);
			}
		}
	}
	_dVar /= nNum;
	_dVar = sqrt(_dVar);

}

void CImageProcessClass::WhitePreprocess(string sPath, int nH, int nW, Mat &_dst)
{
	Mat mat = imread(sPath.c_str(), 0);
	Scalar  mean;
	Scalar stddev; 
	cv::meanStdDev(mat, mean, stddev);  
	mat.convertTo(mat, CV_32FC1);
	for (int row=0; row<mat.rows; row++)
	{
		for (int col=0; col<mat.cols; col++)
		{
			mat.at<float>(row,col) = mat.at<float>(row,col) - mean.val[0];
		}
	}
	Mat mat_cov = mat*mat.t();

	CvMat* A = cvCreateMat(mat_cov.rows, mat_cov.cols, CV_32FC1);
	CvMat* U = cvCreateMat(mat_cov.rows, mat_cov.cols, CV_32FC1); 
	CvMat* D = cvCreateMat(mat_cov.cols, mat_cov.cols, CV_32FC1); 
	CvMat* V = cvCreateMat(mat_cov.cols, mat_cov.cols, CV_32FC1); 
	cvConvert(&IplImage(mat_cov), A); // Mat->CvMat*
	cvSVD(A, D, U, V);//, CV_SVD_U_T|CV_SVD_V_T); // A = U D V^T 

	Mat mat_D = Mat(D, true); //CvMat->Mat
	for (int row=0; row<mat.rows; row++)
	{
		mat_D.at<float>(row, row) = sqrt(mat_D.at<float>(row, row));
	}
	mat_D = mat_D.inv();

    Mat mat_U = Mat(U, true); //CvMat->Mat
	Mat mat_PCAWhite = mat_D*(mat_U.t())*mat;
	Mat mat_ZCAWhite = mat_U*mat_PCAWhite;

	double dMin, dMax, dDis;
	minMaxLoc(mat_PCAWhite, &dMin, &dMax);
	dDis = dMax - dMin + 0.000000001;	
	for (int row=0; row<mat_PCAWhite.rows; row++)
	{
		for (int col=0; col<mat_PCAWhite.cols; col++)
		{
			if (dMin < 0)
			{
				mat_PCAWhite.at<float>(row, col) = mat_PCAWhite.at<float>(row, col) - dMin + 0;
			}
			mat_PCAWhite.at<float>(row, col) = mat_PCAWhite.at<float>(row, col)/dDis*255;
		}
	}
	imwrite("D:\\pca.bmp", mat_PCAWhite);

	minMaxLoc(mat_ZCAWhite, &dMin, &dMax);
	dDis = dMax - dMin + 0.000000001;	
	for (int row=0; row<mat_PCAWhite.rows; row++)
	{
		for (int col=0; col<mat_PCAWhite.cols; col++)
		{
			if (dMin < 0)
			{
				mat_ZCAWhite.at<float>(row, col) = mat_ZCAWhite.at<float>(row, col) - dMin + 0;
			}
			mat_ZCAWhite.at<float>(row, col) = mat_ZCAWhite.at<float>(row, col)/dDis*255;
		}
	}
	imwrite("D:\\zca.bmp", mat_ZCAWhite);
	_dst = mat_ZCAWhite;
}

void CImageProcessClass::testMatlabDll(void)
{
	if (!mclInitializeApplication(NULL, 0))
	{
		return;
	}

	if (!faceDetectInitialize())
	{
		return ;
	}

}

void CImageProcessClass::testLBP(void)
{
/*	string s = "E:\\faceDetect\\Debug\\FaceSample\\Sample\\1\\2.bmp";
	Mat src = imread(s.c_str(), 0);
	Mat dst = ownLBP(src, 3, 8);
    int numPatterns = static_cast<int>(std::pow(2.0, static_cast<double>(8)));
	Mat dst1 = SpatialHistogram(dst, numPatterns, 8, 8);    
	imwrite("D:\\lbp.bmp", dst);
	*/

	vector<string> vsFolder, vsFileName[1000];
	string sPath = "E:\\faceDetect\\Debug\\FaceSample\\Sample";
	CCommonFunClass fun;
	fun.GetDirAllFile(sPath, vsFolder, vsFileName);
	string s = "";
	int person_num = (int)vsFolder.size();
	int every_person_image_num = 0;
	Mat mat;
	
	int radius = 1;
	int neighbors = 8;
	int grid_x = 8;
	int grid_y = 8;
	string sPath1 = "E:\\faceDetect\\Debug\\FaceSample\\Test1";
	double d = 0;
	vector<Mat> histograms;
	for (int i=0; i<person_num; i++)
	{
		every_person_image_num = (int)vsFileName[i].size();	
		s = sPath1 + "\\" + vsFolder.at(i);
		::CreateDirectory(s.c_str(), NULL);
		for (int j=0; j<every_person_image_num; j++)
		{
			s = sPath + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			mat = imread(s.c_str(), 0);
			/////////////////
			Mat lbp_image = ownLBP(mat, radius, neighbors);
			// get spatial histogram from this lbp image
			Mat p = SpatialHistogram(lbp_image,
				static_cast<int>(std::pow(2.0, static_cast<double>(neighbors))), /* number of possible patterns */
				grid_x, grid_y);
			//////////////////////////////////////////////
			// add to templates
			histograms.push_back(p);
			//s = sPath1 + "\\" + vsFolder.at(i) + "\\" + vsFileName[i].at(j);
			//imwrite(s.c_str(), _dst);
		}
	}

}
/*************************************************************************
功能：改进LBP算法
输入：
src为输入图像，
radius为半径，
neighbor为计算当前点LBP所需的邻域像素点数，也就是样本点个数
输出：
dst为输出图像，
**************************************************************************/
Mat CImageProcessClass::ownLBP(Mat src, int radius, int neighbors)
{
	//圆形LBP算子
	src.convertTo(src, CV_32FC1);
    // allocate memory for result 结果分配内存
	Mat dst(src.rows-2*radius, src.cols-2*radius, CV_32SC1);
    // zero
    dst.setTo(0);
    for(int n=0; n<neighbors; n++) 
	{
        // sample points 获取当前采样点
        float x = static_cast<float>(radius * cos(2.0*CV_PI*n/static_cast<float>(neighbors)));
        float y = static_cast<float>(-radius * sin(2.0*CV_PI*n/static_cast<float>(neighbors)));
        // relative indices 相对指标
        int fx = static_cast<int>(floor(x)); //向下取整
        int fy = static_cast<int>(floor(y));
        int cx = static_cast<int>(ceil(x)); //向上取整
        int cy = static_cast<int>(ceil(y));
        // fractional part 小数部分
        float ty = y - fy;
        float tx = x - fx;
        // set interpolation weights 采样点权重
        float w1 = (1 - tx) * (1 - ty);
        float w2 =      tx  * (1 - ty);
        float w3 = (1 - tx) *      ty;
        float w4 =      tx  *      ty;
        // iterate through your data 迭代数据
        for(int i=radius; i<src.rows-radius; i++) 
		{
            for(int j=radius; j<src.cols-radius; j++) 
			{
                // calculate interpolated value 计算插值
                float t = static_cast<float>(w1*src.at<float>(i+fy,j+fx) + w2*src.at<float>(i+fy,j+cx) + w3*src.at<float>(i+cy,j+fx) + w4*src.at<float>(i+cy,j+cx));
                // floating point precision, so check some machine-dependent epsilon
                dst.at<int>(i-radius,j-radius) += ((t > src.at<float>(i,j)) || (std::abs(t-src.at<float>(i,j)) < std::numeric_limits<float>::epsilon())) << n;
            }
        }

    }

	//LBP旋转不变模式
	int nMin = 1000;
	int nValue = 0;
	for(int i=0; i<dst.rows; i++) 
	{
		for(int j=0; j<dst.cols; j++) 
		{
			nMin = 1000;
			for(int n=0; n<neighbors; n++) 
			{
				nValue  = dst.at<int>(i,j) << n;
				nMin = nValue<nMin? nValue : nMin;
			}
		}
	}

	return dst;
}

Mat CImageProcessClass::SpatialHistogram(Mat src, int numPatterns, int grid_x, int grid_y)                          
{
    // calculate LBP patch size 
    int width = src.cols/grid_x;
    int height = src.rows/grid_y;
    // allocate memory for the spatial histogram 为LBPH分配内存空间
    Mat result = Mat::zeros(grid_x * grid_y, numPatterns, CV_32FC1);
    // return matrix with zeros if no data was given 如果没有输入数据，返回的是0
	if(src.empty())
	{
		return result.reshape(1,1); //一行数据
	}

	imwrite("D:\\test.bmp", result);
	CCommonFunClass fun;
	string s;
	char buffer[MAX_PATH] = {0};
    // initial result_row 初始化结果行
    int resultRowIdx = 0;
	float fValue = 0;
    // iterate through grid
    for(int i=0; i<grid_y; i++) 
	{
        for(int j=0; j<grid_x; j++) 
		{
			//获取指定区域
            Mat src_cell = Mat(src, Range(i*height,(i+1)*height), Range(j*width,(j+1)*width));
			src_cell.convertTo(src_cell, CV_32FC1);
            Mat cell_hist = Histc(src_cell, 0, (numPatterns-1), true); // 计算指定区域的直方图

	/*		for (int col=0; col<cell_hist.cols; col++)
			{
				fValue = cell_hist.at<float>(0,col);
				sprintf(buffer, "%.4f ", fValue);
				fun.WriteTxt("D:\\1.txt", buffer);
			}
			*/
            //copy to the result matrix 将计算得到的结果拷贝到每一行
			result.row(resultRowIdx) = cell_hist.row(0) + 0;
		/*	for (int col=0; col<cell_hist.cols; col++)
			{
				fValue = result.at<float>(resultRowIdx,col);
				sprintf(buffer, "%.4f ", fValue);
				fun.WriteTxt("D:\\2.txt", buffer);
			}*/

    /*      Mat result_row = result.row(resultRowIdx);
            cell_hist.reshape(1,1).convertTo(result_row, CV_32FC1); */
            //increase row count in result matrix
            resultRowIdx++;
			imwrite("D:\\test.bmp", result);
        }
    }
    // return result as reshaped feature vector
    return result.reshape(1,1);
}

Mat CImageProcessClass::Histc(const Mat& src, int minVal, int maxVal, bool normed)
{
    Mat result;
    // Establish the number of bins.
    int histSize = maxVal-minVal+1;
    // Set the ranges.
    float range[] = { static_cast<float>(minVal), static_cast<float>(maxVal+1) };
    const float* histRange = { range };
    // calc histogram
    calcHist(&src, 1, 0, Mat(), result, 1, &histSize, &histRange, true, false);
    // normalize
    if(normed) 
	{
        result /= (int)src.total();
    }
    return result.reshape(1,1);
}

void CImageProcessClass::threshProcess(void)
{
	string sPath = "E:\\机器人视觉导航\\image_runway\\rect_red.bmp";//ren_runway_10.jpg";D:\\office.jpg";//
	Mat src = imread(sPath.c_str(), 0);//, 0);
	//imwrite("d:\\dst1.bmp", src);

	Mat dst = src;
/*	CvRect rect;
	rect.x = src.cols/10;
	rect.width = 8*src.cols/10;
	rect.y = 2*src.rows/3;
	rect.height = src.rows/3;
	getRectImage(src, rect, dst);*/
	imwrite("d:\\dst.bmp", dst);
	dst.convertTo(dst, CV_32FC1);

	Scalar mean, stddev;  
	cv::meanStdDev(dst, mean, stddev);  
	double  mean_pxl = mean.val[0];  
	double  stddev_pxl = stddev.val[0];  
	float fValue = 0;
	for (int row=0; row<dst.rows; row++)
	{
		for (int col=0; col<dst.cols; col++)
		{
			fValue = dst.at<float>(row, col);
			fValue = fValue>=mean_pxl? 255 : 0;
			dst.at<float>(row, col) = fValue;
		}
	}
	imwrite("d:\\thresh.bmp", dst);
}

double g_dR=0, g_dG=0, g_dB = 0;
double g_dRadius = 0;
bool g_isInverse = false;
bool g_isFirst = true;
void CImageProcessClass::testVisualNavigation(void)
{
	//打开视频文件：其实就是建立一个VideoCapture结构
	VideoCapture capture("E:/机器人视觉导航/image_runway/runway0622.mp4"); //runway061301 0530 061301拐弯
	//检测是否正常打开:成功打开时，isOpened返回ture
	if(!capture.isOpened())
	{
		cout<<"fail to open!"<<endl;
	}
	//获取整个帧数
	long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	cout<<"整个视频共"<<totalFrameNumber<<"帧"<<endl;

	//设置开始帧()
	long frameToStart = 150;//100; //400 //572;
	capture.set( CV_CAP_PROP_POS_FRAMES,frameToStart);
	cout<<"从第"<<frameToStart<<"帧开始读"<<endl;

	//设置结束帧
	int frameToStop = totalFrameNumber;

	if(frameToStop < frameToStart)
	{
		cout<<"结束帧小于开始帧，程序错误，即将退出！"<<endl;
		return;
	}
	else
	{
		cout<<"结束帧为：第"<<frameToStop<<"帧"<<endl;
	}

	//获取帧率
	double rate = capture.get(CV_CAP_PROP_FPS);
	cout<<"帧率为:"<<rate<<endl;

	//定义一个用来控制读取视频循环结束的变量
	bool IsStop = false;
	//承载每一帧的图像
	Mat frame;
	//显示每一帧的窗口
	namedWindow("Extracted frame");
	//两帧间的间隔时间:
	int delay = 1000/rate;

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
	while(!IsStop)
	{
		//读取下一帧
		if(!capture.read(frame))
		{
			cout<<"读取视频失败"<<endl;
			return;	
		}
		
		//这里加滤波程序
		imshow("Extracted frame", frame);
		filter2D(frame, frame, -1, kernel);
		imwrite("D:\\runway.bmp", frame);	

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
		sharpenUSM(frame, frame);
		//nEdgeNum = runwayFunThree(frame, nIndex, rect, dst, ptStart, ptEnd, dAngle);//
		nEdgeNum = runwayFunOne(frame, nIndex, rect, dst, ptStart, ptEnd, dAngle);
		             
	//	sprintf(buffer, "(%d,%d) %.4f", pt.x, pt.y, dAngle);
	//	fun.WriteTxt("D:\\1.txt", buffer);

		//step2: 存储当前帧获得坐标、角度，通过前几帧来预测获得坐标角度来校正或验证检测结果
	//	ptTemp[nIndex%1000] = pt;
	//	dAngleTemp[nIndex%1000] = dAngle;
	//	correctionResult(ptTemp, dAngleTemp, 1000, nIndex, 20);

		imshow("Contours", dst);
	//	imshow("after filter", frame);
		cout<<"正在读取第"<<currentFrame<<"帧"<<endl;
		//waitKey(int delay=0)当delay ≤ 0时会永远等待；当delay>0时会等待delay毫秒
		//当时间结束前没有按键按下时，返回值为-1；否则返回按键

		int c = waitKey(delay);
		//按下ESC或者到达指定的结束帧后退出读取视频
		if((char)c==27 || currentFrame>frameToStop)
		{
			IsStop = true;
		}
		//按下按键后会停留在当前帧，等待下一次按键
		if (c==32)
		{
			continue;
		}
		else/* *///if( c >= 0)		
		{
			waitKey(0);
		}

		currentFrame++;
		nIndex++;
	
	}
	//关闭视频文件
	capture.release();
	waitKey(0);

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
	imwrite("D:\\rect.bmp", mat);//matRow);	
}

void CImageProcessClass::getRegionAverageValue(const Mat src, CvRect rect, double &_dRadius, double &_dR, double &_dG, double &_dB)
{
	Mat mat;
	getRectImage(src, rect, mat);
	imwrite("D:\\RegionAverageValue.bmp", mat);

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

		_dRadius = __max(abs(dGray-dMax),abs(dGray-dMin));
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

	imwrite("D:\\rect1.bmp", mat);
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
	imwrite("D:\\enhanceRegion.bmp", matTemp);

	if (3 == matTemp.channels())
	{
        cvtColor(matTemp, matGray, CV_BGR2GRAY);
	}
*/
	threshold(matGray, matGray, 0, 255, CV_THRESH_OTSU);
	Mat mat_temp = matGray;
//	imwrite("D:\\thresh_temp.bmp", mat_temp);
	
	//if (g_isInverse)
	{
		for(int i=0; i<matGray.rows; i++)
		{
			matGray.row(i) = 255 - matGray.row(i) + 0;
		}
	}
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	erode(matGray, matGray, element);
	dilate(matGray, matGray, element); //open process

	//flip(matGray, matGray, 1);//1:flip by y axis，0: flip by x axis, -1: flip by both axises
	imwrite("D:\\thresh.bmp", matGray);
	
	//
	vector<vector<Point>> contours_max;
	double dMaxArea = 0;
	CvRect rcMax = cvRect(0,0,0,0);
	CvBox2D box;
	getMaxContours(matGray, contours_max, dMaxArea, rcMax, box);
/*	if (contours_max.size() > 0)
	{
		int len = contours_max.at(0).size();
		Mat mat1 = Mat(matGray.rows, matGray.cols, CV_32FC1);
		cv::Mat::zeros(mat1.rows, mat1.cols, mat1.type());
		CvPoint pt1;
		for (int i=0; i<len; i++)
		{
			pt1 = contours_max.at(0).at(i);
			mat1.at<float>(pt1.y,pt1.x) = 255;
		}
		imwrite("D:\\thresh_max.bmp", mat1);
	}*/
/*	vector<double> vdArea;
 	vector<CvRect> vrcMax;
	getEdgeContours(matGray, contours_max, vdArea, vrcMax);

	Mat mat1 = Mat(matGray.rows, matGray.cols, CV_32FC1);
	cv::Mat::zeros(mat1.rows, mat1.cols, mat1.type());
	for (int i=0; i<contours_max.size(); i++)
	{
		int len = contours_max.at(i).size();		
		CvPoint pt1;
		for (int j=0; j<len; j++)
		{
			pt1 = contours_max.at(i).at(j);
			mat1.at<float>(pt1.y,pt1.x) = 255;
		}		
	}
	imwrite("D:\\thresh_max.bmp", mat1);

	vector<vector<Point>> contours_edge1;
	vector<Point> contours_edge11;
	for (int i=0; i<contours_max.size(); i++)
	{
		//getEdgeContours(mat, contours_max.at(i), rcMax, contours_edge1);
		getEdgeContours(contours_max.at(i), vrcMax.at(i), contours_edge11);
		for (int j=0; j<contours_edge11.size(); j++)
		{			
			circle(mat,contours_edge11.at(j),2,Scalar(0,0,0));
		}
		contours_edge1.push_back(contours_edge11);
	}
	if (contours_edge1.size() > 0)
	{
		judgeContoursState(mat, nIndex, contours_edge1, _dAngle);
	}

//	imwrite("D:\\thresh_max.bmp", mat);
	_dst = mat;
	return 0;
	*/
	if (g_isFirst)
	{
		double dArea = matGray.rows*matGray.cols;
		if (dMaxArea < dArea/3)
		{
			g_isInverse = true;
		}
		return 0;
	}
 	_ptStart.x = rcMax.x + rcMax.width/2;
	_ptStart.y = rcMax.y + rcMax.height;
/*	CvPoint2D32f cvPt[4];
	CvPoint pt[4];
	cvBoxPoints(box, cvPt);
	pt[0].x = cvPt[0].x;
	pt[0].y = cvPt[0].y;
	pt[1].x = cvPt[1].x;
	pt[1].y = cvPt[1].y;
	pt[2].x = cvPt[2].x;
	pt[2].y = cvPt[2].y;
	pt[3].x = cvPt[3].x;
	pt[3].y = cvPt[3].y;
	cv::line(mat,pt[0],pt[1],cv::Scalar(0),4); 
	cv::line(mat,pt[1],pt[2],cv::Scalar(0),4); 
	cv::line(mat,pt[2],pt[3],cv::Scalar(0),4); 
	cv::line(mat,pt[3],pt[0],cv::Scalar(0),4); 

	circle(mat,pt[0],5,Scalar(0,0,0),2);
	circle(mat,pt[1],5,Scalar(0,0,0),2);
    circle(mat,pt[2],5,Scalar(0,0,0),2);
	circle(mat,pt[3],5,Scalar(0,0,0),2);*/
	
//	rectangle(mat, cvPoint(rcMax.x,rcMax.y), cvPoint(rcMax.x+rcMax.width,rcMax.y+rcMax.height), Scalar(255,0,0), 5);  

	vector<vector<Point>> contours_edge;
    for (int i=0; i<contours_max.size(); i++)
	{
		getEdgeContours(mat, contours_max.at(i), rcMax, contours_edge);
		judgeContoursState(mat, nIndex, contours_edge, _dAngle);
		for (int j=0; j<contours_edge.size(); j++)
		{
			for (int i=0; i<contours_edge.at(j).size(); i++)
			{
				circle(mat,contours_edge.at(j).at(i),1,Scalar(0,0,0));
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

			cv::line(mat,_ptStart,_ptEnd,cv::Scalar(0),4); 
		}
		cout<<"图像左上角："<<"(0,0)"<<","<<"图像右下角"<<"("<<mat.cols-1<<","<<mat.rows-1<<")"<<endl;
		cout<<"起始点："<<"("<<_ptStart.x<<","<<_ptStart.y<<")"<<","<<"终止点"<<"("<<_ptEnd.x<<","<<_ptEnd.y<<")"<<endl;
	}
	
	//cout<<"角度 = "<<boxMax.angle<<endl;
 	_dst = mat;

	return 0;
}

//边缘检测
int CImageProcessClass::runwayFunTwo(const Mat src, int nIndex, CvRect rect, Mat &_dst, CvPoint &_pt, double &_dAngle)
{
	Mat mat;
	getRectImage(src, rect, mat);
	int nRow = mat.rows;
	int nCol = mat.cols;

	Mat matGray;
	if (3 == mat.channels())
	{
        cvtColor(mat, matGray, CV_BGR2GRAY);
	}

	GaussianBlur(mat, mat, Size(3,3), 0, 0);
	Canny(mat, mat, 100, 200, 3);
	imwrite("D:\\thresh.bmp", matGray);
	_dst = mat;

	return 0;
}

//二值化
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

	imwrite("D:\\thresh.bmp", matGray);
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
	vector<vector<Point>> contours;
	vector<vector<Point>> contours_temp;
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
		isOK = height>nRow*0.6 && width>5 && width<nCol/6;
		if (isOK)
		{
			contours_temp.push_back(contours[i]);	
			vBoxContours.push_back(box);//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
			vRect.push_back(r);
			nNum++;
	        drawContours(mat,contours_temp,-1, Scalar(255,0,0), 2); //-1画所有轮廓	
		}
	} 

	vector<vector<Point>> contours_edge1;
	vector<Point> contours_edge11;
	double dMidX = (nCol+0.0)/2;
	if (nNum >=2) //边缘多余2个边缘，取距离图片x轴最近的2个边
	{		
		distanceSort(contours_temp, vBoxContours, vRect, dMidX);
		int ret = (vBoxContours.at(0).center.x-dMidX)*(vBoxContours.at(1).center.x-dMidX);
		if (ret>0)//说明离图像中心最近两个边缘在同一侧，一般情况都是存在误检的情况
		{
			_dAngle = 90;
			return nNum;
		}	
	}
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

		cv::line(mat,_ptStart,_ptEnd,cv::Scalar(0),4); 
	}
	cout<<"图像左上角："<<"(0,0)"<<","<<"图像右下角"<<"("<<mat.cols-1<<","<<mat.rows-1<<")"<<endl;
	cout<<"起始点："<<"("<<_ptStart.x<<","<<_ptStart.y<<")"<<","<<"终止点"<<"("<<_ptEnd.x<<","<<_ptEnd.y<<")"<<endl;
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

void CImageProcessClass::getEdgeContours(Mat src, vector<vector<Point>> &_contours, vector<double> &_vdArea, vector<CvRect> &_vrcMax)
{
	_contours.clear();
	_vdArea.clear();
	_vrcMax.clear();
	int nCol = src.cols;
	int nRow = src.rows;
	CvRect r = cvRect(0,0,0,0);
	bool isOK = false;
	CvBox2D boxMax;
	CvRect rcMax = cvRect(0,0,0,0);

	//CvBox2D box;
	RotatedRect box;
	vector<RotatedRect> vBox;
	vector<vector<Point>> contours;
	Point ptCenter = cvPoint(nCol/2,nRow/2);
	double dMaxArea = 0;
	double dArea = 0;
	int width = 0;
	int height = 0;
	//findContours的输入是二值图像
	findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
	for(int i=0; i<(int)contours.size(); i++) //将有符号的contours.size()强制转换位非符号的；
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

		//isOK = r.height>nRow*0.8 && abs(box.center.x-ptCenter.x)<nCol/4;
		isOK = height>nRow*0.6 && width>5 && width<nCol/3;
		if (isOK)
		{	
			if (dArea > dMaxArea)
			{
				dMaxArea = dArea;
			}
			//_contours.clear();
			_contours.push_back(contours[i]);
			_vdArea.push_back(dArea);
			_vrcMax.push_back(r);
			vBox.push_back(box);
			//box.center.x += rect.x;
			//box.center.y += rect.y;
			boxMax = box;//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
			rcMax = r; 			
			//drawContours(mat,contours_max,-1, Scalar(0,0,0),10); //-1画所有轮廓	       
		}
	} 
   /* 
	//排除干扰边缘
	int nLen = vBox.size();
	int nNumLeft = 0;
	int nNumRight = 0;
	bool isLeft = false;
	for (int i=0; i<nLen; i++)
	{
		isLeft = vBox.at(i).center.x-ptCenter.x>0? false : true;
		if (isLeft)
		{
			nNumLeft++;
		}
		else
		{
			nNumRight++;
		}
	}
	*/
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

void CImageProcessClass::getMaxContours(Mat src, vector<vector<Point>> &_contours, double &_dArea, CvRect &_rcMax, CvBox2D &_box)//)
{
	int nCol = src.cols;
	int nRow = src.rows;
	CvRect r = cvRect(0,0,0,0);
	bool isOK = false;
	CvBox2D boxMax;
	CvRect rcMax = cvRect(0,0,0,0);

	CvBox2D box;
	vector<vector<Point>> contours;
	Point ptCenter = cvPoint(nCol/2,nRow/2);
	double dMaxArea = 0;
	double dArea = 0;

	 //findContours的输入是二值图像
	findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
	for(int i=0; i<(int)contours.size(); i++)//将有符号的contours.size()强制转换位非符号的；
	{
		dArea = contourArea(Mat(contours[i]));
		r = boundingRect(Mat(contours[i]));
		box = minAreaRect(Mat(contours[i]));
		isOK = r.height>nRow*0.8 && abs(box.center.x-ptCenter.x)<nCol/4;
		if (isOK)
		{	
			if (dArea > dMaxArea)
			{
				dMaxArea = dArea;
				_contours.clear();
				_contours.push_back(contours[i]);
				//box.center.x += rect.x;
			    //box.center.y += rect.y;
				boxMax = box;//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
				rcMax = r; 			
				//drawContours(mat,contours_max,-1, Scalar(0,0,0),10); //-1画所有轮廓	
			}        
		}
	} 
	_dArea = dMaxArea;
	_rcMax = rcMax;
	_box = boxMax;

}

void CImageProcessClass::getRunwayandEdgeColor(Mat src, Mat binary, double &_dValue1, double &_dR1, 
	                                 double &_dValue2, double &_dR2)
{
	int nCol = src.cols;
	int nRow = src.rows;
	CvRect r = cvRect(0,0,0,0);
	bool isOK = false;
	CvBox2D boxMax;
	CvRect rcMax = cvRect(0,0,0,0);

	CvBox2D box;
	int nNum = 0;
	CvPoint2D32f pt[4];
	vector<vector<Point>> contours;
	vector<vector<Point>> contours_max;
	vector<vector<Point>> contours_temp;
	vector<double> vdArea;
	Point ptCenter = cvPoint(nCol/2,nRow/2);
	double dMaxArea = 0;
	double dArea = 0;

	 //findContours的输入是二值图像
	findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
	for(int i=0; i<(int)contours.size(); i++)//将有符号的contours.size()强制转换位非符号的；
	{
		dArea = contourArea(Mat(contours[i]));
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
				boxMax = box;//夹角 angle是水平轴逆时针旋转，与碰到的第一个边（不管是高还是宽）的夹角
				rcMax = r; 				
				//drawContours(mat,contours_max,-1, Scalar(0,0,0),10); //-1画所有轮廓	
			}        
		}
	} 

	dArea = src.rows * src.cols;
	if (dMaxArea < dArea/3)//大概说明检测到的不是跑道区域
	{
		contours_max.clear(); 
	}

}


void CImageProcessClass::distanceSort(vector<vector<Point>> &vContours, vector<CvBox2D> &vBox, vector<CvRect> &vRect, double dMid)
{
	int nLen = __min(vBox.size(), vRect.size());
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

	vector<vector<Point>> contours;
	vBox.clear();
	vRect.clear();
	nLen = __min(nLen, 2);
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

void CImageProcessClass::correctionResult(CvPoint pt[], double dAngle[], const int nLength, const int nIndex, const int nNum)
{
	int nIndexTemp = nIndex - 1;
	int m = nIndexTemp/nLength; //商
	int r = nIndexTemp%nLength; //余数
    int nLen = nNum;
	
	//step1: 获取数据
	CvPoint* ptTemp = new CvPoint[nNum];
	double* pdAngleTemp = new double[nNum];	
	if (r>=nNum)
	{
		for (int i=0; i<nNum; i++)
		{
			ptTemp[i] = pt[i];
			pdAngleTemp[i] = dAngle[i];
		}
	}
	else 
	{
		if (m < 1)
		{
			for (int i=0; i<r; i++)
			{
				ptTemp[i] = pt[i];
				pdAngleTemp[i] = dAngle[i];
			}
			nLen = r;
		}
		else
		{
			int t = nNum - r;
			for (int i=nLength-t; i<nLength; i++)
			{
				ptTemp[i-nLength+t] = pt[i];
				pdAngleTemp[i-nLength+t] = dAngle[i];
			}
			for (int i=0; i<r; i++)
			{
				ptTemp[t+i] = pt[i];
				pdAngleTemp[t+i] = dAngle[i];
			}
		}
	}
	//step2:预测、校正


	if (ptTemp != NULL)
	{
		delete []ptTemp;
		ptTemp = NULL;
	}
	if (pdAngleTemp != NULL)
	{
		delete []pdAngleTemp;
		pdAngleTemp = NULL;
	}

}

int yk = 0;
int xk = 0;
void CImageProcessClass::judgeContoursState(Mat mat, int nIndex, const vector<vector<Point>> contours, double &_dAngle)
{
	int nLen = contours.size();
	if (nLen<1 || nLen>2)
	{
		_dAngle = -1;
		return;
	}
/*
	vector<double> vdCur[2];
	vector<double> vdAngle[2];
	double dCurVar[2] = {-1};
	double dAngleVar[2] = {-1};
	double dCur[2] = {-1};
	int nDire[2] = {0};*/
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
	for (int i=0; i<nLen; i++)
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
	cout<<"均值："<<dMean<<endl;

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
			cout<<"最小值符号的均值："<<_dAngle<<endl;
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
				cout<<"异号均值："<<dAngle<<endl;
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
	cout<<_dAngle<<endl;
	cout<<endl;
    return;


	double dAngleMean = 0;
	judgeMaxValueDire(vdAngle, dAngleMean);
	_dAngle = dAngleMean;
	cout<<_dAngle<<endl;
	
	CCommonFunClass fun;
	int y = 0;
	int x = nIndex;

	if (fabs(dMean) < 5)
	{
		cout<<"直走"<<endl;
		y = 0;//yk;
		yk = 0;
		_dAngle = 0;
	}
	else if(dMean > 0)
	{
		cout<<"沿y轴左转"<<dMean<<endl;
		yk = yk - dMean;//y = -dMean;//y = tan(dMean+90)*(x-xk) + yk;
		//dMean = 
		_dAngle = dAngle[0];//dAngleMean;
		
	}
	else if(dMean < 0)
	{
		cout<<"沿y轴右转"<<fabs(dMean)<<endl;
		//y = -dMean;//tan(90+dMean)*(x-xk) + yk;
		yk = yk - dMean;
		dMean = 90 + dMean;
		_dAngle = dAngle[0];//dAngleMean;
	}
	else
	{
		cout<<"需要预测"<<endl;
	}

	cout<<_dAngle<<endl;
	//char buffer[MAX_PATH] = {0};
	//sprintf(buffer, "%d, %d", yk, x);
	//fun.writetoCSV("D:\\path3.csv", buffer);
	//cout<<"拐弯角度： "<<yk<<endl;

/*	for (int i=0; i<2; i++)
	{
		getCurvature(mat, contours.at(i), 5, dCurVar[i], dCur[i], nDire[i], vdCur[i]);
		getContoursAngle(mat, contours.at(i), 5, dAngleVar[i],vdAngle[i]);
	}

	cout<<"斜率方差 = "<<dAngleVar[0]<<" , "<<dAngleVar[1]<<endl;	
	cout<<"曲率方差 = "<<dCurVar[0]<<" , "<<dCurVar[1]<<endl;
	cout<<"曲率 = "<<dCur[0]<<" , "<<dCur[1]<<endl;
	cout<<"方向 = "<<nDire[0]<<" , "<<nDire[1]<<endl;

	//sprintf(buffer, "%.4f, %.4f", dCurLeft,dCurRight);
	//fun.WriteTxt("D:\\2.txt", buffer);
	
	if (-1==dAngleVar[0] || -1==dAngleVar[1])
	{
		cout<<"根据前几帧来预估计！"<<endl;
		return;
	}

	if (dAngleVar[0]<1 && dAngleVar[1]<1)
	{
		cout<<"直走"<<endl;
	}
	else 
	{
		cout<<"拐弯"<<endl;	
	}	
*/		
	cout<<endl;
}


void CImageProcessClass::getEdgeContours(Mat mat, const vector<Point> contours, const CvRect rect, vector<vector<Point>> &_contours)
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
	//	pt = ptLeft[i];
	//	sprintf(buffer, "%d, (%d,%d)", i, pt.x, pt.y);
	//	fun.WriteTxt("D:\\1.txt", buffer);
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

void CImageProcessClass::removeInterferencePoints(Mat mat, const vector<Point> contours, vector<Point> &_contours)
{
	_contours.clear();
	int nRow = mat.rows;
	int nCol = mat.cols;
	Point pt;

	int nLen = contours.size();
	for (int i=0; i<nLen; i++)
	{
		pt = contours.at(i);
		if(pt.x>10 && pt.x<nCol-10)
		{
			_contours.push_back(pt);
		}
	}

	nLen = _contours.size();
	if (nLen < nRow/2)
	{
		_contours.clear();
	}

	nLen = _contours.size();
	for (int i=0; i<nLen; i++)
	{
	   circle(mat,_contours.at(i),2,Scalar(0,255,0));
	}
}

//n表示间隔几个像素 nDirec -1左 1右
void CImageProcessClass::getCurvature(Mat mat, const vector<Point> contours, int n, double &_dVar, double &_dCurv, int &_nDirec, vector<double> &_vdCurv)
{
	_dVar = -1;
	_vdCurv.clear();
	//计算轮廓的曲率
	int nLen = contours.size();
	if (nLen < mat.rows/2)
	{
		return;
	}

	double dMean = 0;
	double dAngle = 0;
	double dVar = 0;
	vector<Point>* pvPoints = new vector<Point>[n];
	int nLeft = 0;
	int nRight = 0;
	int nNumLeft = 0;
	int nNumRight = 0;
	Point ptCenter;
	double r = 0;
	double dCurv = 0;
	double dRound = 0, dEccentric = 0;
	double dLinearity = 0;
	for (int j=0; j<nLen; j++)
	{		
		pvPoints[j%n].push_back(contours.at(j));
	}
	cout<<"曲率  ";
	for (int i=0; i<n; i++)
	{	
		LeastSquareFitCircle(pvPoints[i], ptCenter, r);
		//LeastSquareFitCircle(pvPoints[i], ptCenter, r, dRound, dEccentric);
		//getLinearity(pvPoints[i], dLinearity);
		//cout<<dCurv<<", "<<dRound<<", "<<dEccentric<<", "<<dLinearity<<endl;
		nLeft = 0;
		nRight = 0;
		for (int t=0; t<pvPoints[i].size(); t++)
		{
			if (pvPoints[i].at(t).x > ptCenter.x)
			{
				nLeft++;
			}
			else
			{
				nRight++;
			}
		}
		if (nLeft > nRight)
		{
			nNumLeft++;
		}
		else 
		{
			nNumRight++;
		}

		dCurv = 1/r;
		_vdCurv.push_back(dCurv);
		dMean += dCurv;
		cout<<dCurv<<", ";
		if (r<-0.0000001 || r>0.0000001)
		{
			circle(mat,ptCenter,(int)r,Scalar(0,0,0),4);	
		}
	}

	dMean /= n;
	for (int i=0; i<_vdCurv.size(); i++)
	{
		dCurv = _vdCurv.at(i);
		dVar += pow(dCurv-dMean, 2);
	}
	_dVar = sqrt(dVar);

	//求曲率
	_dCurv = _dVar<1.0? dMean : -1;
	
	//求弯曲方向
	_nDirec = nNumLeft>nNumRight? -1 : 1;
	cout<<endl;
}

void CImageProcessClass::getLinearity(const vector<Point> contours, double &_dLinearity)
{
	double x_bar = 0;
	double y_bar = 0;
	int n = contours.size();
	for (int i=0; i<n; i++)
	{
		x_bar += contours.at(i).x;
		y_bar += contours.at(i).y;
	}
	x_bar /= n;
	y_bar /= n;

	double u11 = 0;
	double u20 = 0;
	double u02 = 0;
	for (int i=0; i<n; i++)
	{
		u11 += (contours.at(i).x-x_bar) * (contours.at(i).y-y_bar);
		u20 += (contours.at(i).x-x_bar) * (contours.at(i).x-x_bar);
		u02 += (contours.at(i).y-y_bar) * (contours.at(i).y-y_bar);
	}

	u11 /= n;
	u20 /= n;
	u02 /= n;
	double linearity = 0;
	linearity = sqrt(4*u11*u11 + (u20-u02)*(u20-u02));
	linearity /= (u02+u20);
	_dLinearity = linearity;
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
/*	int fx = 0, fy = 0;
	double dK = 0; //Laplacian
	int nNumZero = 0;
	double dMean = 0;
	int nNum = 0;
	for (int i=n; i<nLen; i++)
	{
		fy = contours.at(i).y - contours.at(i-n).y;
		fx = contours.at(i).x - contours.at(i-n).x;
		if (0 == fx)
		{
			nNumZero++;
		}
		else
		{
			dK = (double)fy/fx;
			dMean += dK;
			nNum++;
		}		
		cout<<dK<<", ";
	}

	dMean /= nNum;	
	_dRatio = nNumZero/(nNumZero+nNum+0.0000001);
	_dAngle = atan(dMean); //-pi/2 ~ pi/2
	cout<<endl<<_dAngle<<endl;
	cout<<180/CV_PI*_dAngle<<endl;
	*/
	//画一个线段  

	//储存拟合直线的容器  
    Vec4f line;  
	vector<Point> vPoints;
	int h = nLen/4;
	n = nLen/2;
	double dK = 0;
	//double t1 = (double)cvGetTickCount(); 
	for (int j=n-h; j<n+h; j++)
	{		
		vPoints.push_back(contours.at(j));
		//circle(mat,contours.at(j),5,Scalar(0,255,0));
	}

	circle(mat,contours.at(n),10,Scalar(0,255,0));
	circle(mat,contours.at(n-h+1),10,Scalar(0,255,0));
	circle(mat,contours.at(n+h-1),10,Scalar(0,255,0));

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
	//cout<<dK<<", ";
	//画一个线段  
	int x0= line[2];  
	int y0= line[3];  
	//int x1= x0-10*line[0];  
	//int y1= y0-10*line[1];  
	int y1 = mat.rows-1;
	int x1 = (y1-y0)*line[0]/line[1] + x0;
	cv::line(mat,Point(x0,y0),cv::Point(x1,y1),cv::Scalar(255,255,255),4);  
	
//	circle(mat,cvPoint(line[2],line[3]),5,Scalar(255,255,255));
//	circle(mat,cvPoint(x1,y1),5,Scalar(255,255,255));

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

//轮廓均分成n等分
void CImageProcessClass::getContoursAngle(Mat mat, const vector<Point> contours, int n, double &_dVar, vector<double> &_vdAngle)
{
	_vdAngle.clear();
	_dVar = -1;
	int nLen = contours.size();
	if (nLen < mat.rows/2 )
	{
		return;
	}
	int h = nLen/n;
	double dK = 0;	
	double dMean = 0;
	double dAngle = 0;
	double dVar = 0;
	//储存拟合直线的容器  
    Vec4f line;  
	vector<Point> vPoints;
	cout<<"斜率  ";
	for (int i=1; i<=n; i++)
	{
		vPoints.clear();
		for (int j=0; j<h; j++)
		{		
			vPoints.push_back(contours.at((i-1)*h+j));
		}
		fitLine(vPoints, line, CV_DIST_L1, 1, 0.001, 0.001);//(line[0],line[1])表示直线的方向向量，(line[2],line[3])表示直线上的一个点。 
		if (line[0]<-0.00000001 || line[0]>0.0000001)
		{
			dK = line[1]/line[0];
		}

		//画一个线段  
		int x0= (int)line[2];  
		int y0= (int)line[3];  
		int x1= (int)(x0-200*line[0]);  
		int y1= (int)(y0-200*line[1]);  
		cv::line(mat,Point(x0,y0),cv::Point(x1,y1),cv::Scalar(0),1);  
		cout<<dK<<", ";
		dAngle = atan(dK);
		dMean += dAngle;
		_vdAngle.push_back(dAngle);
	}
	cout<<endl;
	dMean /= n;
	for (int i=0; i<_vdAngle.size(); i++)
	{
		dAngle = _vdAngle.at(i);
		dVar += pow(dAngle-dMean, 2);
	}
	_dVar = sqrt(dVar/n);

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

int CImageProcessClass::LeastSquareFitCircle(vector<Point> &vPts, Point &_p, double &_r)
{
	int n = vPts.size();
	double x1 = 0;
	double y1 = 0;
	double x2 = 0;
	double y2 = 0;
	double x3 = 0;
	double y3 = 0;
	double x1y1 = 0;
	double x1y2 = 0;
	double x2y1 = 0;
	for (int i=0; i<n; i++)
	{
		x1 += vPts.at(i).x;
		y1 += vPts.at(i).y;
		x2 += vPts.at(i).x * vPts.at(i).x;
		y2 += vPts.at(i).y * vPts.at(i).y;
		x3 += vPts.at(i).x * vPts.at(i).x * vPts.at(i).x;
		y3 += vPts.at(i).y * vPts.at(i).y * vPts.at(i).y;
		x1y1 += vPts.at(i).x * vPts.at(i).y;
		x1y2 += vPts.at(i).x * vPts.at(i).y * vPts.at(i).y;
		x2y1 += vPts.at(i).x * vPts.at(i).x * vPts.at(i).y;
	}
	
	double c = 0;
	double d = 0;
	double e = 0;
	double g = 0;
	double h = 0;
	double a = 0;
	double b = 0;
	double f = 0;
	c = n*x2 - x1*x1;
	d = n*x1y1 - x1*y1;
	e = n*x3 + n*x1y2 - (x2+y2)*x1;
	g = n*y2 - y1*y1;
	h = n*x2y1 + n*y3 - (x2+y2)*y1;
	a = (h*d - e*g) / (c*g - d*d);
	b = (h*c - e*d) / (d*d - g*c);
	f = -(a*x1 + b*y1 + x2 + y2) / n;
	
	Point ptCenter = Point((int)(-a/2), (int)(-b/2));
	_r = sqrt(a*a + b*b - 4*f) / 2;
	_p.x = ptCenter.x;
	_p.y = ptCenter.y;
	
	return 0;
}

/**********************************************************
Description（描述）
        用采集到的轮廓数据拟合圆
Parameters（参数）
输入参数：
        vpts: 轮廓数据
输出参数：
       _p: 圆心坐标,  _r: 半径，_dRound: 圆度, _dEccentric: 偏心度
***************************************************************/
int CImageProcessClass::LeastSquareFitCircle(vector<Point> &vPts, Point &_p, double &_r, double &_dRound, double &_dEccentric)
{
	int n = vPts.size();
	if (n < 3)
	{
		return -1;
	}

	Point ptCenter;
	double r = 0;
    LeastSquareFitCircle(vPts, ptCenter, r);
	Point p1;
	double r1 = 0;
	_FindMaxInCircle(vPts, ptCenter, p1, r1);
	Point p2;
	double r2 = 0;
	_FindMinOutCircle(vPts, p2, r2);
	
	_p.x = ptCenter.x;
	_p.y = ptCenter.y;
	_r = r;
	_dRound = r1 - r2;
	double offset = (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y);
	_dEccentric = sqrt(offset);

	return 0;
}

int CImageProcessClass::_FindMinOutCircle(vector<Point> &vPts, Point &_p, double &_r)
{
	double dist = 0;
	int n = vPts.size();
	double max = 0;
	Point pt1;
	Point pt2;	
	for(int i=0; i<n; i++)
	{
		for (int j=i+1; j<n; j++)
		{
			dist = (vPts.at(i).x-vPts.at(j).x)*(vPts.at(i).x-vPts.at(j).x) \
				+ (vPts.at(i).y-vPts.at(j).y)*(vPts.at(i).y-vPts.at(j).y);
			dist = fabs(dist);
			if (dist>max)
			{
				max = dist;
				pt1.x = vPts.at(i).x;
				pt1.y = vPts.at(i).y;
				pt2.x = vPts.at(j).x;
				pt2.y = vPts.at(j).y;
			}
		}
	}
	
	dist = 0;
	max = 0;
	Point pt0;
	for (int i=0; i<n; i++)
	{
        pt0.x = vPts.at(i).x;
		pt0.y = vPts.at(i).y;
		_DistOfPointToLine(pt0, pt1, pt2, dist);
		if (dist>max)
		{
			max = dist;
            break;
		}
	}

	_ThreePointsCoCircle(pt0, pt1, pt2, _p, _r);

	return 0;
}

int CImageProcessClass::_FindMaxInCircle(vector<Point> &vPts, Point c, Point &_p, double &_r)
{
	double dist = 0;
	int n = vPts.size();
	double min1 = MINVALUE;
	double min2 = MINVALUE;
	Point pt1;
	Point pt2;
	for(int i=0; i<n; i++)
	{
		dist = (vPts.at(i).x-c.x)*(vPts.at(i).x-c.x) + (vPts.at(i).y-c.y)*(vPts.at(i).y-c.y);
		dist = fabs(dist);
		if (dist < min1)
		{
			min1 = dist;
			pt1.x = vPts.at(i).x;
			pt1.y = vPts.at(i).y;
		}
		else if (dist < min2)
		{
			min2 = dist;
			pt2.x = vPts.at(i).x;
			pt2.y = vPts.at(i).y;
		}
		else
		{
			;
		}
	}

	dist = 0;
	double max = 0;
	Point pt0;
	for (int i=0; i<n; i++)
	{
        pt0.x = vPts.at(i).x;
		pt0.y = vPts.at(i).y;
		_DistOfPointToLine(pt0, pt1, pt2, dist);
		if (dist>max)
		{
			max = dist;
            break;
		}
	}
	
	_ThreePointsCoCircle(pt0, pt1, pt2, _p, _r);

	return 0;
}

// 求点p到线段l所在直线的距离
int CImageProcessClass::_DistOfPointToLine(Point p, Point lineStart, Point lineEnd, double &_dDist) 
{
	double mul = 0.;
	_TwoVectorsMultiply(p, lineEnd, lineStart, mul);
	double dist = 0;
	_DistOfTwoPoints(lineStart, lineEnd, dist);
	dist = mul / dist;
	_dDist = dist;
	
	return 0; 
}

// 求不共线的三点确定一个圆 
int CImageProcessClass::_ThreePointsCoCircle(Point p1, Point p2, Point p3, Point &_c, double &_r) 
{ 
	double x12 = p2.x - p1.x; 
	double y12 = p2.y - p1.y; 
	double x13 = p3.x - p1.x; 
	double y13 = p3.y - p1.y; 
	double z2 = x12 * (p1.x+p2.x) + y12 * (p1.y+p2.y); 
	double z3 = x13 * (p1.x+p3.x) + y13 * (p1.y+p3.y); 
	double d = 2.0 * (x12 * (p3.y-p2.y) - y12 * (p3.x-p2.x)); 
	if(fabs(d) < EP)
	{
		return -1;
	}

	_c.x = (int)((y13*z2 - y12*z3) / d); 
	_c.y = (int)((x12*z3 - x13*z2) / d); 
	_DistOfTwoPoints(p1, _c, _r); 

	return 0; 
}

// r=TwoPointsMultiply(sp,ep,op),得到(sp-op)和(ep-op)的叉积 
// r>0：ep在矢量opsp的逆时针方向； 
// r=0：opspep三点共线； 
// r<0：ep在矢量opsp的顺时针方向 
int CImageProcessClass::_TwoVectorsMultiply(Point sp, Point ep, Point op, double &_dRes) 
{
	double res = (sp.x-op.x) * (ep.y-op.y);
	res -= (ep.x-op.x) * (sp.y-op.y);
	_dRes = res;

	return 0; 
}

// 返回两点之间欧氏距离
int CImageProcessClass::_DistOfTwoPoints(Point p1, Point p2, double &_dDist)                
{
	double dist = 0;
	dist = (p1.x-p2.x) * (p1.x-p2.x);
	dist += (p1.y-p2.y) * (p1.y-p2.y);
	dist = sqrt(dist);
	_dDist = dist;

	return 0; 
}

void CImageProcessClass::testDistortion(void)
{
	CCameraProcess proc;


	CvSize szChessBoard;
	szChessBoard.height = 6;
	szChessBoard.width = 9;
    CvMat* intrinsics = cvCreateMat(3,3,CV_32FC1);
	CvMat* distortion_coeff = cvCreateMat(1,4,CV_32FC1);

/*	int ret = proc.calcDistortionCorrectionParam(g_capture, 20, szChessBoard, intrinsics, distortion_coeff);
	if (ret < 0)
	{
		return;
	}*/
	string s = "D:\\distortion.txt";
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

	cvReleaseMat(&intrinsics);
	cvReleaseMat(&distortion_coeff);

    cvWaitKey(0);
   
}

void CImageProcessClass::sharpenUSM(Mat src, Mat &_dst)
{
	double sigma = 3;  
	int threshold = 0;  
	float amount = 1;  
	
	Mat imgBlurred;
	GaussianBlur(src, imgBlurred, Size(), sigma, sigma);  
	Mat lowContrastMask = abs(src-imgBlurred)<threshold;  
	imwrite("D:\\res1.jpg", lowContrastMask);  
	_dst = src*(1+amount)+ imgBlurred*(-amount);  
	src.copyTo(_dst, lowContrastMask);  
	imwrite("D:\\res.jpg", _dst);  
}

void CImageProcessClass::errorDiffusion(Mat src, Mat &_dst)
{
	if (src.empty())
	{
		return;
	}
	
	src.convertTo(src, CV_32FC1);
	_dst = src;
	int nRows = src.rows;
	int nCols = src.cols;
	int r = 1;
	Vec3b* pSrc;
	Vec3b* pDst;
	Vec3b* pSrc1;
	float fOld,fNew;
	double err = 0;
	int nValue = 16;
    for (int row=r; row<nRows-r; row++)
    {
		pSrc = src.ptr<Vec3b>(row);
		pSrc1 = src.ptr<Vec3b>(row+1);
        for(int col=r; col<nCols-r; col++)
        {
            //计算输出像素的值
			for (int i=0; i<3; i++)
			{
				fOld = pSrc[col][i];
				fNew = floor(fOld/nValue)*nValue;
		        pSrc[col][i] = fNew;      
				//计算误差值
				err = fOld - fNew; 
				//扩散误差
				pSrc[col+1][i] += err * 7/nValue; //右
				pSrc1[col-1][i] += err * 3/nValue; //左下
				pSrc1[col][i] += err * 5/nValue; //下
				pSrc1[col+1][i] += err * 1/nValue; //右下
			}
        }
    }
	imwrite("D:\\res.bmp", src);

/*	Mat matGray;
	if (3 == src.channels())
	{
        cvtColor(src, matGray, CV_BGR2GRAY);
	}
	imwrite("D:\\res11.bmp", matGray);
	for (int row=r; row<nRows-r; row++)
    {	
        for(int col=r; col<nCols-r; col++)
        {
			//计算输出像素的值
			fOld = matGray.at<float>(row,col);
			fNew = floor(fOld/nValue)*nValue;
			matGray.at<float>(row,col) = fNew;      
			//计算误差值
			err = fOld - fNew;
			//扩散误差
			matGray.at<float>(row,col+1) += err * 7/nValue; //右
			matGray.at<float>(row+1,col-1) += err * 3/nValue; //左下
			matGray.at<float>(row+1,col) += err * 5/nValue; //下
			matGray.at<float>(row+1,col+1) += err * 1/nValue; //右下
        }
    }	
	imwrite("D:\\res.bmp", matGray);*/
}

