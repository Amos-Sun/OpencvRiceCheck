#include<cv.h>
#include<highgui.h>
#include<iostream>
#include<opencv2\opencv.hpp>
#include<vector>
#include<ctime>
using namespace cv;
using namespace std;

void getPhoto(IplImage* src);
IplImage* getGrayPhoto(IplImage* img);
IplImage* getFilterPhoto(IplImage* src, const int MODEL_VALUE);
int getThresholdValue(IplImage* src, int thresholdValue);
IplImage* getTwoPhoto(IplImage* src);
IplImage* getErodePhoto(IplImage* src);
IplImage* getDilatePhoto(IplImage* src);
int getThresholdValueTest(IplImage* src);
Mat iplImageToMat(IplImage* src);
IplImage* matToIplImage(Mat src);
IplImage* getSubPhoto(IplImage* src1, IplImage* src2);
IplImage* getEdgePhoto(IplImage* src);
void showImage(IplImage* img);
void lightOperatePhotoShow(IplImage* src);

//ͼ���еı�����
double scale;
const double STAFF_LENGTH = 3.48;

int main()
{
	
	IplImage* img = cvLoadImage("img\\rice.jpg");
	IplImage* grayPhoto = getGrayPhoto(img);
	IplImage* filterPhoto = getFilterPhoto(grayPhoto, 3);
	IplImage* filterPhoto1 = cvCloneImage(filterPhoto);
	cvMorphologyEx(filterPhoto, filterPhoto1, NULL, NULL, CV_MOP_OPEN, 2);
	IplImage* binaryImage = getTwoPhoto(filterPhoto1);
	IplImage* binaryImage1 = getTwoPhoto(filterPhoto);

	//��rice.png���д����ʱ��Ҫ�õ�
	/*IplImage* openOperatePhoto = cvCreateImage(cvGetSize(grayPhoto), grayPhoto->depth, grayPhoto->nChannels);
	cvMorphologyEx(grayPhoto, openOperatePhoto, NULL, NULL, CV_MOP_OPEN, 81);
	IplImage* subPhoto = getSubPhoto(grayPhoto, openOperatePhoto);
	IplImage* binaryImage = getTwoPhoto(grayPhoto);
	IplImage* binaryImage1 = getTwoPhoto(subPhoto);

	cvNamedWindow("ԭͼ", 1);
	cvShowImage("ԭͼ", img);

	cvNamedWindow("δ�������ͼ��", 1);
	cvShowImage("δ�������ͼ��", binaryImage);

	cvNamedWindow("�������ͼ��", 1);
	cvShowImage("�������ͼ��", binaryImage1);
	cvWaitKey(0);
	cvDestroyAllWindows();*/


	/*IplImage* openOperatePhoto1 = cvCreateImage(cvGetSize(filterPhoto), filterPhoto->depth, filterPhoto->nChannels);
	cvMorphologyEx(filterPhoto, openOperatePhoto, NULL, NULL, CV_MOP_OPEN, 81);
	IplImage* subPhoto1 = getSubPhoto(grayPhoto, openOperatePhoto);
	IplImage* binaryImage1 = getTwoPhoto(subPhoto1);*/

	//rice.jpg����ʾ
	//cvNamedWindow("ԭͼ", 0);
	//cvShowImage("ԭͼ", img);

	//cvNamedWindow("δ���п�����ͼ��", 0);
	//cvShowImage("δ���п�����ͼ��", binaryImage1);

	//cvNamedWindow("������֮��ͼ��", 0);
	//cvShowImage("������֮��ͼ��", binaryImage);
	//cvWaitKey(0);
	//cvDestroyAllWindows();


	//showImage(binaryImage);
	getPhoto(binaryImage);

	cvReleaseImage(&img);
	cvReleaseImage(&grayPhoto);
	cvReleaseImage(&filterPhoto);
	cvReleaseImage(&binaryImage);

	return 0;
}
//�ܳ����������Ϣ
void getPhoto(IplImage* src)
{
	vector<double> areas;
	//�����Ĵ洢����
	CvMemStorage* storage = cvCreateMemStorage(0);
	//������������������ĵ�һ��ָ��
	CvSeq* firstContour = 0;
	IplImage* dst = cvCloneImage(src);
	//�ڶ�ֵͼ����Ѱ�Ҷ�������������Cv ͼ����
	//ע�� dst �ᱻ�ı� 
	int contourNum = cvFindContours(dst, storage, &firstContour,
		sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//freeman ���� 
	//�����Ĵ洢���� 1(Cxcore ��̬�ṹ) 
	CvMemStorage* storage1 = cvCreateMemStorage(0);
	//���������������һ�����������ָ�� 
	CvSeq* firstContour1 = 0;
	IplImage* dst1 = cvCloneImage(src);
	//�ڶ�ֵͼ����Ѱ�� freeman ����������Cv ͼ����
	//ע�� dst1 �ᱻ�ı� 
	int contourNum1 = cvFindContours(dst1, storage1, &firstContour1,
		sizeof(CvChain), CV_RETR_EXTERNAL, CV_CHAIN_CODE);
	//����ͼ
	IplImage* image = cvCloneImage(src);
	cvSet(image, cvScalarAll(0), 0);
	//����������������Cxcore ��ͼ������
	cvDrawContours(image, firstContour, CV_RGB(255, 255, 255),
		CV_RGB(255, 255, 255), 2, 1, 0);
	cvNamedWindow("����ͼ", 0);
	cvShowImage("����ͼ", image);
	cvWaitKey(0);

	//��ȡ freeman ���� 
	CvSeqReader reader;
	//������ʾ��Ӿ��� 
	IplImage*image1 = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	for (CvSeq* contour = firstContour, *contour1 = firstContour1;
		contour != 0, contour1 != 0; contour = contour->h_next, contour1 = contour1->h_next)
	{
		cvSet(image, cvScalarAll(0), 0);
		cvDrawContours(image, contour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, CV_FILLED, 8);

		cvShowImage("����ͼ", image);
		cout << "����������" << contour->total << '\t';
		//(Cv �ṹ����) 
		cout << "�ܳ���" << cvArcLength(contour) << '\t';
		//�Լ������ܳ� ��Cxcore ��̬�ṹ��
		//cvStartReadSeq((CvSeq*)contour1, &reader, 0);
		//double perimeter = 0;
		//for (int cNum = 0; cNum < contour->total; cNum++)
		//{
		//	char code;
		//	//��ȡ���� 
		//	CV_READ_SEQ_ELEM(code, reader);
		//	//����Ϊż�����ܳ�+1 
		//	if (int(code) % 2 == 0)
		//	{
		//		perimeter += 1;
		//	}
		//	//����Ϊ���������ܳ�+sqrt(2) 
		//	if (int(code) % 2 != 0)
		//	{
		//		perimeter += sqrt(2);
		//	}
		//}
		//cout << "������ܳ���" << perimeter << '\t';
		//cout << "�����" << fabs(cvContourArea(contour)) << '\t';
		//cout << "�����ڷ� 0 ������" << cvCountNonZero(image) << '\t';
		// cvWaitKey(0);

		//��С��Ӿ���ͼ 
		cvSet(image1, cvScalarAll(0), 0);
		cvDrawContours(image1, contour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, CV_FILLED, 8);
		//�Ը��� 2D �㼯��Ѱ����С�������Χ���Σ�Cv �ṹ������
		CvBox2D box = cvMinAreaRect2(contour, NULL);
		cout << "�������꣺(" << box.center.x << "," << box.center.y << ")" << '\t';
		cvSet2D(image1, box.center.y, box.center.x, CV_RGB(0, 0, 255));

		double length = (box.size.height > box.size.width ? box.size.height : box.size.width);
		double width = (box.size.height > box.size.width ? box.size.width : box.size.height);

		//ת��ʵ�����
		if (length > 100)
		{
			scale = STAFF_LENGTH / length;
		}

		double area = length * width;
		double realArea = (length * scale)*(width * scale);
		double realLehgth = 2 * (length*scale + width*scale);
		areas.push_back(realArea);
		cout << "����" << length << '\t';
		cout << "��" << width << '\t';
		cout << "���������" << area << endl;
		cout << "��:��" << length / width << '\t';
		cout << "ʵ�������" << realArea << "cm*cm" << '\t';
		cout << "ʵ���ܳ���" << realLehgth << "cm";

		//���������С���� 
		CvPoint2D32f pt[4];
		//Ѱ�Һ��ӵĶ��㣨Cv �ṹ������ 
		cvBoxPoints(box, pt);
		for (int i = 0; i < 4; ++i)
		{
			//(Cxcore ��ͼ������Cxcore �����ṹ)
			cvLine(image1, cvPointFrom32f(pt[i]),
				cvPointFrom32f(pt[((i + 1) % 4) ? (i + 1) : 0]),
				CV_RGB(255, 0, 0),
				1);
		}
		cvShowImage("����ͼ", image1);
		cvWaitKey(0);
		cout << endl;
	}
	cout << "������Ŀ��" << contourNum << endl;
	int i;
	for (i = 1; i < areas.size(); i++)
	{
		cout << areas[i] / areas[1] << '\t';
	}
	cvWaitKey(0);
	
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&dst);
	cvReleaseMemStorage(&storage1);
	cvReleaseImage(&dst1);
	cvReleaseImage(&image);
	cvReleaseImage(&image1);
}
//�õ��Ҷ�ͼ
IplImage* getGrayPhoto(IplImage* img)
{
	IplImage* grayPhoto = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);//����Ŀ��ͼ��  
	cvCvtColor(img, grayPhoto, CV_BGR2GRAY);//cvCvtColor(src,des,CV_BGR2GRAY) 

	return grayPhoto;
}
//�õ��˲�ͼ��
IplImage* getFilterPhoto(IplImage* src, const int MODEL_VALUE)
{
	Mat inputSrc = cvarrToMat(src);
	Mat outputDst;

	blur(inputSrc, outputDst, Size(MODEL_VALUE, MODEL_VALUE), Point(0, 0));

	//����ת������ʹ�ã������ڴ�й©
	/*IplImage* output = &IplImage(outputDst);*/

	IplImage imgTemp = outputDst;
	IplImage* output = cvCloneImage(&imgTemp);

	inputSrc.release();
	outputDst.release();



	return output;
}
//�õ���ֵ������ֵ
int getThresholdValueTest(IplImage* src)
{
	//�õ�ƽ���Ҷ�
	CvScalar initThresholdValue = cvAvg(src);
	CvScalar thresholdValue;
	thresholdValue.val[0] = 0;
	CvScalar temp1;
	CvScalar temp2;
	temp1.val[0] = 0;
	temp2.val[0] = 0;

	while (abs(thresholdValue.val[0] - initThresholdValue.val[0]) > 0.1)
	{
		thresholdValue = initThresholdValue;

		IplImage* dst = cvCloneImage(src);
		//����ǰ��ƽ���Ҷ�
		cvThreshold(src, dst, thresholdValue.val[0], 255, CV_THRESH_TOZERO);
		temp1.val[0] = cvSum(dst).val[0] / cvCountNonZero(dst);
		//���㱳��ƽ���Ҷ�
		cvThreshold(src, dst, thresholdValue.val[0], 255, CV_THRESH_TOZERO_INV);
		temp2.val[0] = cvSum(dst).val[0] / cvCountNonZero(dst);

		initThresholdValue.val[0] = (temp1.val[0] + temp2.val[0]) / 2;

		cvReleaseImage(&dst);
	}
	return initThresholdValue.val[0];
}

int getThresholdValue(IplImage* src, int thresholdValue)
{
	vector<int>bigger;
	vector<int>smaller;
	unsigned char grayvalue;
	int i, j;
	////ͳ��ÿһ������ֵ�ж��ٸ���
	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++)
		{
			//�õ�����ֵ
			grayvalue = src->imageData[i*src->width + j];

			if (grayvalue > thresholdValue)
			{
				bigger.push_back(grayvalue);
			}
			else if (grayvalue <= thresholdValue)
			{
				smaller.push_back(grayvalue);
			}
		}
	}

	//������ڸ�����ֵ��ƽ��ֵ
	double bigAverage = 0;
	for (i = 0; i < bigger.size(); i++)
	{
		bigAverage += bigger[i];
	}
	if (bigger.size() != 0)
	{
		bigAverage /= bigger.size()*1.0;
	}
	//����С�ڸ�����ֵ��ƽ��ֵ
	double smaAverage = 0;
	for (i = 0; i < smaller.size(); i++)
	{
		smaAverage += smaller[i];
	}
	if (smaller.size() != 0)
	{
		smaAverage /= smaller.size()*1.0;
	}

	int thresholdValueTemp = (bigAverage + smaAverage) / 2;
	
	//
	if (thresholdValueTemp - thresholdValue <= 0.1)
	{
		return thresholdValueTemp;
	}
	else{
		bigger.clear();
		smaller.clear();
		thresholdValueTemp = getThresholdValue(src, thresholdValueTemp);
	}
	return thresholdValueTemp;

}
//�õ���ֵ��֮���ͼƬ
IplImage* getTwoPhoto(IplImage* src)
{
	IplImage* binaryImage = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);

	int timeStart = clock();
	//int thresholdValue = getThresholdValue(src, 0);
	int thresholdValue = getThresholdValueTest(src);
	cvThreshold(src, binaryImage, thresholdValue, 255, CV_THRESH_BINARY);
	int timeEnd = clock();

	cout << "����ʱ�䣺" << timeEnd - timeStart << endl;

	return binaryImage;
}
//ͼ��ʴ����
IplImage* getErodePhoto(IplImage* src)
{
	IplImage* dst = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
	cvErode(src, dst, NULL, 1);
	return dst;
}
//ͼƬ���ʹ���
IplImage* getDilatePhoto(IplImage* src)
{
	IplImage* dst = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
	cvDilate(src, dst, NULL, 1);
	return dst;
}
//�õ���Ե���ͼƬ
IplImage* getEdgePhoto(IplImage* src)
{

	Mat inputSrc = cvarrToMat(src);
	Mat ouputDst;

	//Laplacian(inputSrc, ouputDst, inputSrc.depth());
	Canny(inputSrc, ouputDst, 50, 150, 3);

	IplImage outputTemp = ouputDst;
	IplImage* output = cvCloneImage(&outputTemp);

	//Ҫ�ͷţ�������ڴ����
	inputSrc.release();
	ouputDst.release();

	return output;
}
//ͼ�����
//src1 Ϊԭͼ    src2  ΪҪ��ȥ��ͼƬ
IplImage* getSubPhoto(IplImage* src1, IplImage* src2)
{
	Mat srcTemp1 = iplImageToMat(src1);
	Mat srcTemp2 = iplImageToMat(src2);
	Mat subPic = srcTemp1 - srcTemp2;

	IplImage* output = matToIplImage(subPic);
	return output;
}
//MatתIplImage
IplImage* matToIplImage(Mat src)
{
	IplImage outputTemp = src;
	IplImage* output = cvCloneImage(&outputTemp);
	return output;
}
//IplImageתMat
Mat iplImageToMat(IplImage* src)
{
	Mat output = cvarrToMat(src);
	return output;
}
//��ʾͼƬ
void showImage(IplImage* img)
{
	cvNamedWindow("test", 1);
	cvShowImage("test", img);

	cvWaitKey(0);

	cvDestroyAllWindows();
}
//�Թ��ղ���ͼ����д���
void lightOperatePhotoShow(IplImage* src)
{
	IplImage* filterPhoto = getFilterPhoto(src, 9);
	IplImage* subPhoto = getSubPhoto(src, filterPhoto);
	
	IplImage* grayPhoto = getGrayPhoto(src);
	IplImage* filterPhoto1 = getFilterPhoto(grayPhoto, 5);
	IplImage* binaryImage = getTwoPhoto(filterPhoto1);
	
	showImage(binaryImage);

	return ;
}