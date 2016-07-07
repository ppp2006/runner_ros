//#include "StdAfx.h"
#include <cv.h>  
    #include <highgui.h>  
    #include <iostream>  
#include <vector>  
#include "CommonFunClass.h"


CCommonFunClass::CCommonFunClass(void)
{
}


CCommonFunClass::~CCommonFunClass(void)
{
}


void CCommonFunClass::WriteTxt(const char* sFilePath, const char* sValue)
{
	if (strlen(sFilePath) < 1)
	{
		return;
	}

	FILE *pFile = NULL;
	pFile = fopen(sFilePath, "at");//fopen(s,"a+"/*"at"*/);
	if (NULL == pFile)
	{
		return;
	}

	fprintf(pFile,"%s", sValue);
	fprintf(pFile,"\n");
	fclose(pFile);
}



void CCommonFunClass::writetoCSV(const char* sPath, const char* sValue)
{
	FILE *pFileCSV = NULL;
	pFileCSV = fopen(sPath, "at");//fopen(s,"a+"/*"at"*/);
	if (NULL == pFileCSV)
	{
		return;
	}

	fprintf(pFileCSV,"%s,", sValue);
	fprintf(pFileCSV,"\n"); 

	if (pFileCSV != NULL)
	{
		fclose(pFileCSV);
	}
}
