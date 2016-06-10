#include "image_process.h"

using namespace cv;
using namespace std;

float Color_Detection(IplImage* src,IplImage* dst,double &xpositon,double& yposition)
{
	int sum=0,xsum=0,ysum=0;
	float percent=0;
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			//if(datafloat[i*step + j*channels + 2]>10&&(datafloat[i*step + j*channels + 1]>0.1)&&(datafloat[i*step + j*channels]>130&&datafloat[i*step + j*channels]<310))
			//if((datafloat[i*step + j*channels]>10&&datafloat[i*step + j*channels]<90))
			if((datafloat[i*step + j*channels]>30&&datafloat[i*step + j*channels]<90)&&(datafloat[i*step + j*channels + 1]>0.2))
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=255;
			}
			else
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;
				sum++;
				xsum+=(j+1);
				ysum+=(i+1);
			}
		}
	}

	percent=1.0*sum/(hsv_img->height*hsv_img->width);
	if(sum>0)
	{
		xpositon=1.0*xsum/sum;
		yposition=1.0*ysum/sum;
		//printf("%d\n", sum);
	}
	else
	{
		xpositon=0;
		yposition=0;
	}
	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
	return percent;
}

void Number_Detection(IplImage* src,IplImage* dst)
{
	cvSmooth(src,src,CV_MEDIAN,5,5);
	IplImage *src_float = cvCreateImage(cvGetSize(src),IPL_DEPTH_32F, 3);
	cvConvertScale(src, src_float, 1.0, 0.0);
	IplImage *hsv_img = cvCreateImage(cvGetSize(src), IPL_DEPTH_32F , 3);
	cvCvtColor(src_float, hsv_img, CV_BGR2HSV);
	int step = hsv_img->widthStep/sizeof(float);
	int channels = hsv_img->nChannels;
	float * datafloat = (float *)hsv_img->imageData;
	for(int i = 0; i < hsv_img->height; i++)
	{
		for(int j = 0; j < hsv_img->width; j++)
		{
			//if(datafloat[i*step + j*channels + 2]>10&&(datafloat[i*step + j*channels + 1]>0.1)&&(datafloat[i*step + j*channels]>130&&datafloat[i*step + j*channels]<310))
			if((datafloat[i*step + j*channels]>10&&datafloat[i*step + j*channels]<90))
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=255;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=255;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=255;
			}
			else
			{
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+1]=0;
				// src->imageData[i*(src->widthStep)+j*(src->nChannels)+2]=0;
				dst->imageData[i*(dst->widthStep)+j*(dst->nChannels)]=0;
			}
		}
	}

	cvReleaseImage(&hsv_img);
	cvReleaseImage(&src_float);
}

//Input: 8比特、单通道(二值)图像
void edge_extracting(IplImage* src, IplImage* dst)
{
	cvCanny(src,dst,200,220,3);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	lines = cvHoughLines2(dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, 30, 10);
	for (int i=0;i<lines->total;i++)  
    {  
        CvPoint *line = (CvPoint *)cvGetSeqElem(lines,i);  
        cvLine(dst,line[0],line[1],CV_RGB(255,255,255),3,CV_AA,0);  
    }  

	cvShowImage("line Detector",dst);
	cvReleaseMemStorage(&storage);
}

//Input: 8比特、单通道(二值)图像
float find_center(IplImage* src, double &x, double &y)
{
	int sum=0,xsum=0,ysum=0;
	float percent=0;
	unsigned char* data=(unsigned char *)src->imageData;  
	int step = src->widthStep/sizeof(unsigned char);  

	for(int i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(data[i*step + j] > 100)
			{
				sum++;
				xsum+=(j+1);
				ysum+=(i+1);
			}
		}
	}
	if(sum > 0)
	{
		x = xsum/sum;
		y = ysum/sum;
		percent = 1.0 * sum / (src->height * src->width);
	}
	
	return percent;
}
