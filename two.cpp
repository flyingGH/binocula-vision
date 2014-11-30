#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include "library.c"
#include "sift.c"
#define X 160           //640
#define Y 120           // 480     

using namespace std;

void MakeImg(IplImage* img, float* src, int h, int w)
{
	int i,j;
	uchar *ptr;
	for(i=0;i<h;i++)			//along Height
	{
        ptr = (uchar*)(img->imageData + i*img->widthStep);
		for(j=0;j<w;j++)		//along Width
		{
             *src=(0.299*ptr[3*j+2] + 0.587*ptr[3*j+1] + 0.114*ptr[3*j]);
             src++;
        }
    }    
}

void cent(matchingslist matchings, int num_match, float* ang, float* scale)
{
	if(num_match < 2 ){
		printf("Not enough Matches\n");
		return;
	}
	int gx1,gy1,gx2,gy2,  fx1,fy1,fx2,fy2;
	
	gx1 = matchings[0].first.x;
	gy1 = matchings[0].first.y;
	fx1 = matchings[0].second.x;
	fy1 = matchings[0].second.y;
	
	gx2 = matchings[1].first.x;
	gy2 = matchings[1].first.y;
	fx2 = matchings[1].second.x;
	fy2 = matchings[1].second.y;
	
	float thetha = atan2(gy2-gy1,gx2-gx1) - atan2(fy2-fy1,fx2-fx1);
	if(thetha > 2*PI)	thetha -= 2*PI;
	else if(thetha < -2*PI)	thetha += 2*PI;
	
	float d1 = (gy2-gy1)*(gy2-gy1) + (gx2-gx1)*(gx2-gx1);
	float d2 = (fy2-fy1)*(fy2-fy1) + (fx2-fx1)*(fx2-fx1);
	
	*scale = sqrt(d1/d2);
	*ang = (thetha*180)/PI;
}

int main(int argc, char **argv)
{	
    if(argc < 3){
		printf("Please provide Input image File Name.\n");
		system("pause");
		exit(1);
	}
	///////////////////
	int i;
	float * iarr1;		
    size_t w1, h1;		
    IplImage *img1;	
    img1=cvLoadImage(argv[1]);
	if( img1==NULL ){
		printf("Unable to open image file\n");
		exit(1);
	}
	printf("Computing keypoints on the two images...\n");
	h1=img1->height;
	w1=img1->width;
    iarr1=(float*)malloc(sizeof(float)*h1*w1);
    MakeImg(img1,iarr1,h1,w1);
/////////////////////////////
                             
    float * iarr2;		
    size_t w2, h2;		
    IplImage *img2;	
    img2=cvLoadImage(argv[2]);
	if( img2==NULL ){
		printf("Unable to open image file\n");
		exit(1);
	}
	printf("Computing keypoints on the two images...\n");
	h2=img2->height;
	w2=img2->width;
    iarr2=(float*)malloc(sizeof(float)*h2*w2);
    MakeImg(img2,iarr2,h2,w2);

//////////////////////////
    struct siftPar siftparameters;	
	default_sift_parameters(&siftparameters);
	
    keypointslist_short keys1;	
	int num_keys1=0;
	num_keys1 = compute_sift_keypoints(iarr1, &keys1, w1, h1, siftparameters);
	printf("%d SIFT keypoints are detected. \n", num_keys1);
	free(iarr1);

////////////////
    keypointslist_short keys2;	
	int num_keys2=0;
	num_keys2 = compute_sift_keypoints(iarr2, &keys2, w2, h2, siftparameters);
	printf("%d SIFT keypoints are detected. \n", num_keys2);
	free(iarr2);
	
	//////////
    matchingslist matchings;
	printf("Matching the keypoints...\n");
	int num_matchings = compute_sift_matches(keys1, keys2, num_keys1, num_keys2, &matchings, siftparameters);
		
//		tend = time(0);
	if ( num_matchings > 0 )
	  	printf("The number of final matches is %d \n", num_matchings);
	else {
		printf("The two images do not match.\n");
	}
////////////////
                
    
	int band_w = 10; 
	
	int woH, hoH;
	IplImage *mainImg;	

	woH =  w1+w2+band_w;
	hoH = MAX(h1,h2);
	
	mainImg = cvCreateImage(cvSize(woH,hoH),8,3);

    cvZero(mainImg);
	cvSetImageROI(mainImg, cvRect(0,0,w1,h1));
	cvCopy(img1, mainImg);
	cvResetImageROI(mainImg);	
	cvSetImageROI(mainImg, cvRect(w1+band_w,0,w2,h2));
	cvCopy(img2, mainImg);
	cvResetImageROI(mainImg);
	
     for(i=0; i< num_matchings; ++i)
	{	
            int pix = 320;
         	float a = matchings[i].first.x, b=matchings[i].second.x;
            if(a >=  pix/2 && b <= pix/2){
			cvLine(mainImg, cvPoint((int) (matchings[i].first.x), (int) (matchings[i].first.y)), 
					cvPoint((int) (matchings[i].second.x) + w1 + band_w,(int) (matchings[i].second.y)) , CV_RGB(255,255,255),
			       1);
			cout << matchings[i].first.x << "," << matchings[i].first.y << ") = ("  <<matchings[i].second.x  << "," << matchings[i].second.y ;
			float d = 15, pi = 3.1415926535;float half = (pi*28/180); // cm
    
    float a1 = atan(((a -(pix/2))/(pix/2))*tan(half));
    float a2 = atan((((pix/2 - b))/(pix/2))*tan(half));
    float k1 = tan(pi/2-a1);
    float k2 = tan(pi/2-a2);
    float ans = (k1*k2)*d/(k1+k2);
      cout << "\tMeasured distance = " << ans  << "cm" << endl;
          }
           else{
            cvLine(mainImg, cvPoint((int) (matchings[i].first.x), (int) (matchings[i].first.y)), 
					cvPoint((int) (matchings[i].second.x) + w1 + band_w,(int) (matchings[i].second.y)) , CV_RGB(255,255,255),
			       1);
			cout << matchings[i].first.x << "," << matchings[i].first.y << ") = ("  <<matchings[i].second.x  << "," << matchings[i].second.y ;
			float d = 15, pi =3.1415926535;float half = (pi*25/180); // cm
    
    float a1 = atan(((a -(pix/2))/(pix/2))*tan(half));
    float a2 = atan((((pix/2 - b))/(pix/2))*tan(half));
    float k1 = sin(pi/2-a1);
    float k2 = sin(pi/2-a2);
    float ans = (k1*k2)*d/sin(abs(a1-a2));
       cout << "\tMeasured distance = " << (float)ans*3/2  << "cm" << endl;
           }
     }
//		printf("\n--------------------------\n");
//		printf("Angle = %f, Scale = %f",ang,scale);
//		printf("\n--------------------------\n");
		
		cvShowImage("Output",mainImg);
		cvWaitKey(1);
		

    getchar();
    return 0;	
}
