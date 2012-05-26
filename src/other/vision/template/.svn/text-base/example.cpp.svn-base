// Template matching. 
//   Usage: matchTemplate image template 
// 
#include <cv.h> 
#include <cxcore.h> 
#include <highgui.h> 
#include <stdio.h> 
int main( int argc, char** argv ) { 
    IplImage *src, *templ,*ftmp[6]; //ftmp will hold results 
    int i; 
    if( argc == 3){ 
        //Read in the source image to be searched: 
        if((src=cvLoadImage(argv[1], 1))== 0) { 
            printf("Error on reading src image %s\n",argv[i]); 
            return(-1); 
        } 
        //Read in the template to be used for matching: 
        if((templ=cvLoadImage(argv[2], 1))== 0) { 
            printf("Error on reading template %s\n",argv[2]); 
                return(-1); 
        } 
        //ALLOCATE OUTPUT IMAGES: 
        int iwidth = src->width - templ->width + 1; 
        int iheight = src->height - templ->height + 1; 
        for(i=0; i<6; ++i){ 
            ftmp[i] = cvCreateImage( 
                              cvSize(iwidth,iheight),32,1); 
        } 
        //DO THE MATCHING OF THE TEMPLATE WITH THE IMAGE:
        for(i=0; i<6; ++i){ 
            cvMatchTemplate( src, templ, ftmp[i], i); 
            cvNormalize(ftmp[i],ftmp[i],1,0,CV_MINMAX); 
        } 
        //DISPLAY 
        cvNamedWindow( "Template", 0 ); 
        cvShowImage(   "Template", templ ); 
        cvNamedWindow( "Image", 0 ); 
        cvShowImage(   "Image", src ); 
        cvNamedWindow( "SQDIFF", 0 ); 
        cvShowImage(   "SQDIFF", ftmp[0] ); 
        cvNamedWindow( "SQDIFF_NORMED", 0 ); 
        cvShowImage(   "SQDIFF_NORMED", ftmp[1] ); 
        cvNamedWindow( "CCORR", 0 ); 
        cvShowImage(   "CCORR", ftmp[2] ); 
        cvNamedWindow( "CCORR_NORMED", 0 ); 
        cvShowImage(   "CCORR_NORMED", ftmp[3] ); 
        cvNamedWindow( "CCOEFF", 0 ); 
        cvShowImage(   "CCOEFF", ftmp[4] ); 
        cvNamedWindow( "CCOEFF_NORMED", 0 ); 
        cvShowImage(   "CCOEFF_NORMED", ftmp[5] ); 
        //LET USER VIEW RESULTS: 
        cvWaitKey(0); 
    } 
    else { printf("Call should be: matchTemplate image template \n");} 
} 