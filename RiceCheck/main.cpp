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

//图像中的比例尺
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

	//对rice.png进行处理的时候要用到
	/*IplImage* openOperatePhoto = cvCreateImage(cvGetSize(grayPhoto), grayPhoto->depth, grayPhoto->nChannels);
	cvMorphologyEx(grayPhoto, openOperatePhoto, NULL, NULL, CV_MOP_OPEN, 81);
	IplImage* subPhoto = getSubPhoto(grayPhoto, openOperatePhoto);
	IplImage* binaryImage = getTwoPhoto(grayPhoto);
	IplImage* binaryImage1 = getTwoPhoto(subPhoto);

	cvNamedWindow("原图", 1);
	cvShowImage("原图", img);

	cvNamedWindow("未处理光照图像", 1);
	cvShowImage("未处理光照图像", binaryImage);

	cvNamedWindow("处理光照图像", 1);
	cvShowImage("处理光照图像", binaryImage1);
	cvWaitKey(0);
	cvDestroyAllWindows();*/


	/*IplImage* openOperatePhoto1 = cvCreateImage(cvGetSize(filterPhoto), filterPhoto->depth, filterPhoto->nChannels);
	cvMorphologyEx(filterPhoto, openOperatePhoto, NULL, NULL, CV_MOP_OPEN, 81);
	IplImage* subPhoto1 = getSubPhoto(grayPhoto, openOperatePhoto);
	IplImage* binaryImage1 = getTwoPhoto(subPhoto1);*/

	//rice.jpg的显示
	//cvNamedWindow("原图", 0);
	//cvShowImage("原图", img);

	//cvNamedWindow("未进行开运算图像", 0);
	//cvShowImage("未进行开运算图像", binaryImage1);

	//cvNamedWindow("开运算之后图像", 0);
	//cvShowImage("开运算之后图像", binaryImage);
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
//周长，面积等信息
void getPhoto(IplImage* src)
{
	vector<double> areas;
	//轮廓的存储容器
	CvMemStorage* storage = cvCreateMemStorage(0);
	//输出参数，包括轮廓的第一个指针
	CvSeq* firstContour = 0;
	IplImage* dst = cvCloneImage(src);
	//在二值图像中寻找定点序列轮廓（Cv 图像处理）
	//注意 dst 会被改变 
	int contourNum = cvFindContours(dst, storage, &firstContour,
		sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//freeman 链码 
	//轮廓的存储容器 1(Cxcore 动态结构) 
	CvMemStorage* storage1 = cvCreateMemStorage(0);
	//输出参数：包含第一个输出轮廓的指针 
	CvSeq* firstContour1 = 0;
	IplImage* dst1 = cvCloneImage(src);
	//在二值图像中寻找 freeman 链码轮廓（Cv 图像处理）
	//注意 dst1 会被改变 
	int contourNum1 = cvFindContours(dst1, storage1, &firstContour1,
		sizeof(CvChain), CV_RETR_EXTERNAL, CV_CHAIN_CODE);
	//轮廓图
	IplImage* image = cvCloneImage(src);
	cvSet(image, cvScalarAll(0), 0);
	//将所有轮廓画出（Cxcore 绘图函数）
	cvDrawContours(image, firstContour, CV_RGB(255, 255, 255),
		CV_RGB(255, 255, 255), 2, 1, 0);
	cvNamedWindow("轮廓图", 0);
	cvShowImage("轮廓图", image);
	cvWaitKey(0);

	//获取 freeman 链码 
	CvSeqReader reader;
	//用于显示外接矩形 
	IplImage*image1 = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
	for (CvSeq* contour = firstContour, *contour1 = firstContour1;
		contour != 0, contour1 != 0; contour = contour->h_next, contour1 = contour1->h_next)
	{
		cvSet(image, cvScalarAll(0), 0);
		cvDrawContours(image, contour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, CV_FILLED, 8);

		cvShowImage("轮廓图", image);
		cout << "轮廓点数：" << contour->total << '\t';
		//(Cv 结构分析) 
		cout << "周长：" << cvArcLength(contour) << '\t';
		//自己计算周长 （Cxcore 动态结构）
		//cvStartReadSeq((CvSeq*)contour1, &reader, 0);
		//double perimeter = 0;
		//for (int cNum = 0; cNum < contour->total; cNum++)
		//{
		//	char code;
		//	//获取链码 
		//	CV_READ_SEQ_ELEM(code, reader);
		//	//链码为偶数则周长+1 
		//	if (int(code) % 2 == 0)
		//	{
		//		perimeter += 1;
		//	}
		//	//链码为奇数，则周长+sqrt(2) 
		//	if (int(code) % 2 != 0)
		//	{
		//		perimeter += sqrt(2);
		//	}
		//}
		//cout << "计算的周长：" << perimeter << '\t';
		//cout << "面积：" << fabs(cvContourArea(contour)) << '\t';
		//cout << "轮廓内非 0 点数：" << cvCountNonZero(image) << '\t';
		// cvWaitKey(0);

		//最小外接矩形图 
		cvSet(image1, cvScalarAll(0), 0);
		cvDrawContours(image1, contour, CV_RGB(255, 255, 255), CV_RGB(255, 255, 255), 0, CV_FILLED, 8);
		//对给定 2D 点集，寻找最小面积的外围矩形（Cv 结构分析）
		CvBox2D box = cvMinAreaRect2(contour, NULL);
		cout << "中心坐标：(" << box.center.x << "," << box.center.y << ")" << '\t';
		cvSet2D(image1, box.center.y, box.center.x, CV_RGB(0, 0, 255));

		double length = (box.size.height > box.size.width ? box.size.height : box.size.width);
		double width = (box.size.height > box.size.width ? box.size.width : box.size.height);

		//转到实际面积
		if (length > 100)
		{
			scale = STAFF_LENGTH / length;
		}

		double area = length * width;
		double realArea = (length * scale)*(width * scale);
		double realLehgth = 2 * (length*scale + width*scale);
		areas.push_back(realArea);
		cout << "长：" << length << '\t';
		cout << "宽：" << width << '\t';
		cout << "像素面积：" << area << endl;
		cout << "长:宽：" << length / width << '\t';
		cout << "实际面积：" << realArea << "cm*cm" << '\t';
		cout << "实际周长：" << realLehgth << "cm";

		//绘制外接最小矩形 
		CvPoint2D32f pt[4];
		//寻找盒子的顶点（Cv 结构分析） 
		cvBoxPoints(box, pt);
		for (int i = 0; i < 4; ++i)
		{
			//(Cxcore 绘图函数、Cxcore 基础结构)
			cvLine(image1, cvPointFrom32f(pt[i]),
				cvPointFrom32f(pt[((i + 1) % 4) ? (i + 1) : 0]),
				CV_RGB(255, 0, 0),
				1);
		}
		cvShowImage("轮廓图", image1);
		cvWaitKey(0);
		cout << endl;
	}
	cout << "米粒数目：" << contourNum << endl;
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
//得到灰度图
IplImage* getGrayPhoto(IplImage* img)
{
	IplImage* grayPhoto = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);//创建目标图像  
	cvCvtColor(img, grayPhoto, CV_BGR2GRAY);//cvCvtColor(src,des,CV_BGR2GRAY) 

	return grayPhoto;
}
//得到滤波图像
IplImage* getFilterPhoto(IplImage* src, const int MODEL_VALUE)
{
	Mat inputSrc = cvarrToMat(src);
	Mat outputDst;

	blur(inputSrc, outputDst, Size(MODEL_VALUE, MODEL_VALUE), Point(0, 0));

	//这种转换方法使得，发生内存泄漏
	/*IplImage* output = &IplImage(outputDst);*/

	IplImage imgTemp = outputDst;
	IplImage* output = cvCloneImage(&imgTemp);

	inputSrc.release();
	outputDst.release();



	return output;
}
//得到二值化的阈值
int getThresholdValueTest(IplImage* src)
{
	//得到平均灰度
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
		//计算前景平均灰度
		cvThreshold(src, dst, thresholdValue.val[0], 255, CV_THRESH_TOZERO);
		temp1.val[0] = cvSum(dst).val[0] / cvCountNonZero(dst);
		//计算背景平均灰度
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
	////统计每一个像素值有多少个点
	for (i = 0; i < src->height; i++)
	{
		for (j = 0; j < src->width; j++)
		{
			//得到像素值
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

	//计算大于给定阈值的平均值
	double bigAverage = 0;
	for (i = 0; i < bigger.size(); i++)
	{
		bigAverage += bigger[i];
	}
	if (bigger.size() != 0)
	{
		bigAverage /= bigger.size()*1.0;
	}
	//计算小于给定阈值的平均值
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
//得到二值化之后的图片
IplImage* getTwoPhoto(IplImage* src)
{
	IplImage* binaryImage = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);

	int timeStart = clock();
	//int thresholdValue = getThresholdValue(src, 0);
	int thresholdValue = getThresholdValueTest(src);
	cvThreshold(src, binaryImage, thresholdValue, 255, CV_THRESH_BINARY);
	int timeEnd = clock();

	cout << "运行时间：" << timeEnd - timeStart << endl;

	return binaryImage;
}
//图像腐蚀处理
IplImage* getErodePhoto(IplImage* src)
{
	IplImage* dst = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
	cvErode(src, dst, NULL, 1);
	return dst;
}
//图片膨胀处理
IplImage* getDilatePhoto(IplImage* src)
{
	IplImage* dst = cvCreateImage(cvGetSize(src), src->depth, src->nChannels);
	cvDilate(src, dst, NULL, 1);
	return dst;
}
//得到边缘检测图片
IplImage* getEdgePhoto(IplImage* src)
{

	Mat inputSrc = cvarrToMat(src);
	Mat ouputDst;

	//Laplacian(inputSrc, ouputDst, inputSrc.depth());
	Canny(inputSrc, ouputDst, 50, 150, 3);

	IplImage outputTemp = ouputDst;
	IplImage* output = cvCloneImage(&outputTemp);

	//要释放，否则会内存溢出
	inputSrc.release();
	ouputDst.release();

	return output;
}
//图像相减
//src1 为原图    src2  为要减去的图片
IplImage* getSubPhoto(IplImage* src1, IplImage* src2)
{
	Mat srcTemp1 = iplImageToMat(src1);
	Mat srcTemp2 = iplImageToMat(src2);
	Mat subPic = srcTemp1 - srcTemp2;

	IplImage* output = matToIplImage(subPic);
	return output;
}
//Mat转IplImage
IplImage* matToIplImage(Mat src)
{
	IplImage outputTemp = src;
	IplImage* output = cvCloneImage(&outputTemp);
	return output;
}
//IplImage转Mat
Mat iplImageToMat(IplImage* src)
{
	Mat output = cvarrToMat(src);
	return output;
}
//显示图片
void showImage(IplImage* img)
{
	cvNamedWindow("test", 1);
	cvShowImage("test", img);

	cvWaitKey(0);

	cvDestroyAllWindows();
}
//对光照不均图像进行处理
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