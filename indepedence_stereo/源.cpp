#pragma warning( disable: 4996 )
/* *************** License:**************************

************************************************** */

#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
//#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
using namespace std;

//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
static void StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated)
{
	int displayCorners = 1;
	int showUndistorted = 1;
	bool isVerticalStereo = false;//OpenCV can handle left-right
	//or up-down camera arrangements
	const int maxScale = 1;
	const float squareSize = 40; //Set this to your actual square size
	FILE* f = fopen(imageList, "rt");
	int i, j, lr, nframes, n = nx*ny, N = 0;
	vector<string> imageNames[2];                      //��������ͼƬ��·����0Ϊ��1Ϊ��
	vector<CvPoint3D32f> objectPoints;                 //����nframes���ǵ����ꣻ
	vector<CvPoint2D32f> points[2];                    //��������ͼƬ���ܽǵ����ꣻ
	vector<int> npoints;                               //�ǵ������Ŀ��
	vector<uchar> active[2];                           //�������ҽǵ������ķ���ֵ
	vector<CvPoint2D32f> temp(n);                      //����ÿ�����̽ǵ�����ꣻ
	CvSize imageSize = { 0, 0 };
	

	double M1[3][3], M2[3][3], D1[5], D2[5];
	double R[3][3], T[3], E[3][3], F[3][3];
	CvMat _M1 = cvMat(3, 3, CV_64F, M1);               //��������ڲ�������
	CvMat _M2 = cvMat(3, 3, CV_64F, M2);               //��������ڲ�������
	CvMat _D1 = cvMat(1, 5, CV_64F, D1);               //��������������
	CvMat _D2 = cvMat(1, 5, CV_64F, D2);               //��������������
	CvMat _R = cvMat(3, 3, CV_64F, R);                 //��ת����
	CvMat _T = cvMat(3, 1, CV_64F, T);                 //ƽ�ƾ���
	CvMat _E = cvMat(3, 3, CV_64F, E);                 //��������
	CvMat _F = cvMat(3, 3, CV_64F, F);                 //��������
	
	if (displayCorners)
	{
		cvNamedWindow("corners", 1);
	}
	
	//�򿪺��б궨���·����txt�ļ�
	if (!f)                                           //�ж�f�Ƿ�Ϊ��
	{
		fprintf(stderr, "can not open file %s\n", imageList);
		return;
	}
	
	for (i = 0;; i++)
	{
		char buf[1024];
		int count = 0, result = 0;
		lr = i % 2;
		vector<CvPoint2D32f>& pts = points[lr];
		
		if (!fgets(buf, sizeof(buf)-3, f))             //�õ�txt�ļ��е�һ�У������ļ���ַ�ŵ�buf������
		{
			break;
		}

		size_t len = strlen(buf);                     //�õ�����ĳ���
		
		while (len > 0 && isspace(buf[len - 1]))      //�ж�ͼ��·����Ϊ��
		{
			buf[--len] = '\0';
		}

		if (buf[0] == '#')                            //·��ǰ���#���Զ��Թ�
		{
			continue;
		}

		IplImage* img = cvLoadImage(buf, 0);          //����ͼ��
		
		if (!img)                                     //�ж�ͼ���Ƿ�Ϊ��
		{
			break;
		}

		imageSize = cvGetSize(img);                  //�õ�ͼ��Ĵ�С
		imageNames[lr].push_back(buf);               //��ͼƬ��·������imageNames��
		

		for (int s = 1; s <= maxScale; s++)
		{
			IplImage* timg = img;
			if (s > 1)
			{
				timg = cvCreateImage(cvSize(img->width*s, img->height*s),img->depth, img->nChannels); //�õ�imgͼ����ͬ�Ĵ�С����Ⱥ�ͨ��
				cvResize(img, timg, CV_INTER_CUBIC);                                                  //�õ���ͬ�Ĵ�С
			}

			//Ѱ�ҽǵ�����
			result = cvFindChessboardCorners(timg, cvSize(nx, ny),&temp[0], &count,CV_CALIB_CB_ADAPTIVE_THRESH |CV_CALIB_CB_NORMALIZE_IMAGE);
			if (timg != img)
			{
				cvReleaseImage(&timg);
			}

			if (result || s == maxScale)    
			for (j = 0; j < count; j++)
			{
				temp[j].x /= s;
				temp[j].y /= s;
			}
			if (result)            
			{
				break;                 //��ʾѰ�ҽǵ�ʧ��
			}
		}

		if (displayCorners)           //�Ƿ���ʾ�궨�Ľǵ�
		{
			printf("%s\n", buf);      //���ͼ���·��
			IplImage* cimg = cvCreateImage(imageSize, 8, 3);
			cvCvtColor(img, cimg, CV_GRAY2BGR);
			cvDrawChessboardCorners(cimg, cvSize(nx, ny), &temp[0],
				count, result);
			cvShowImage("corners", cimg);
		
			cvReleaseImage(&cimg);
			if (cvWaitKey(0) == 27) //Allow ESC to quit
				exit(-1);
		}
		else
			putchar('.');
		N = pts.size();
		pts.resize(N + n, cvPoint2D32f(0, 0));
		active[lr].push_back((uchar)result);
		//assert( result != 0 );
		if (result)
		{
			//��ʼѰ�������ؽǵ�����
			cvFindCornerSubPix(img, &temp[0], count,cvSize(11, 11), cvSize(-1, -1),cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,30, 0.01));
			copy(temp.begin(), temp.end(), pts.begin() + N);            //û����ʲô��˼��Ӧ���ǰ�temp�Ľǵ���߷ŵ�point��ȥ�ˣ�
		}
		cvReleaseImage(&img);
	}

	fclose(f);
	printf("\n");
	// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
	nframes = active[0].size();//Number of good chessboads found
	objectPoints.resize(nframes*n);
	for (i = 0; i < ny; i++)
	for (j = 0; j < nx; j++)
		objectPoints[i*nx + j] =
		cvPoint3D32f(i*squareSize, j*squareSize, 0);
	for (i = 1; i < nframes; i++)
		copy(objectPoints.begin(), objectPoints.begin() + n,
		objectPoints.begin() + i*n);
	npoints.resize(nframes, n);
	N = nframes*n;
	CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0]);
	CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0]);       //��ߵ�0��ͼ��Ľǵ�����
	CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0]);       //�ұߵ�0��ͼ��Ľǵ�����
	CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0]);
	cvSetIdentity(&_M1);                                              //���к�����ȵ�����Ϊ1
	cvSetIdentity(&_M2);
	cvZero(&_D1);
	cvZero(&_D2);

	// CALIBRATE THE STEREO CAMERAS
	printf("Running stereo calibration ...");
	fflush(stdout);
	cvStereoCalibrate(&_objectPoints, &_imagePoints1,&_imagePoints2, &_npoints,&_M1, &_D1, &_M2, &_D2,imageSize, &_R, &_T, &_E, &_F,cvTermCriteria(CV_TERMCRIT_ITER +CV_TERMCRIT_EPS, 100, 1e-5),
                      CV_CALIB_FIX_ASPECT_RATIO +CV_CALIB_ZERO_TANGENT_DIST +CV_CALIB_SAME_FOCAL_LENGTH);
	
	cvSave("M1.xml", &_M1);
	cvSave("M2.xml", &_M2);
	cvSave("D1.xml", &_D1);
	cvSave("D2.xml", &_D2);
	cvSave("R.xml", &_R);
	cvSave("T.xml", &_T);
	cvSave("E.xml", &_E);
	cvSave("F.xml", &_F);
	cvSave("imagePoints1.xml", &_imagePoints1);
	cvSave("imagePoints2.xml", &_imagePoints2);

	printf(" done\n");
}


	int main(void)
	{
		StereoCalib("ch12_list.txt", 12, 9, 1);
		return 0;
	}