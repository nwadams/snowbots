
#include <stdio.h>
#include "cv.h"
#include "highgui.h"

#include <imageops.h>
#include "templateMatch.h"

using namespace std;
using namespace imageops;

int main( int argc, char** argv )
{
    IplImage    *img;
    IplImage    *tpl;
    IplImage    *res;
    int         width, height;
	
	 /* check for arguments */
    if( argc < 4 ) {
        fprintf( stderr, "Usage: icon_search <screenshot> <icon template> <max error>\n" );
        return 1;
    }

    /* load reference image */
    img = cvLoadImage( argv[1], CV_LOAD_IMAGE_COLOR );

    /* always check */
    if( img == 0 ) {
        fprintf( stderr, "Cannot load file %s!\n", argv[1] );
        return 1; 
    }

    /* load template image */
    tpl = cvLoadImage( argv[2], CV_LOAD_IMAGE_COLOR );

    /* always check */
    if( tpl == 0 ) {
        fprintf( stderr, "Cannot load file %s!\n", argv[2] );
        return 1;
    }

    /* get the width and height for the result image */
    width  = img->width  - tpl->width  + 1;
    height = img->height - tpl->height + 1;

    /* create a new image for comparison result */
    res = cvCreateImage( cvSize( width, height ), IPL_DEPTH_32F, 1 );

    /* always check */
    if( res == 0 ) {
        fprintf( stderr, "Cannot create new image for comparison result!\n" );
        return 1; 
    }
    
    vision::imagePyramid P(img,300,40,10);
    vision::imagePyramid R(P,cvGetSize(tpl));
    
    double maxerror = atof(argv[3]);
    
    for(int i = 0; i < P.levels(); i++){
    	cvMatchTemplate( P[i], tpl, R[i], CV_TM_SQDIFF_NORMED );
    	
    	/*std::vector<double> values;
    	std::vector<CvPoint> minima = R.findLocalExtrema(R[i], VISION_MINIMA, values);
        
        
        
        int validcount = 0;
        
        for(int e = 0; e < (int) minima.size(); e++){
        	if(values[e] < maxerror){
        		validcount++;
        		
        		cvRectangle( P[i], 
					 minima[e], 
					 cvPoint( minima[e].x + tpl->width, minima[e].y + tpl->height ),
					 CV_RGB(0,255,0), 1, 0, 0 );
        	}
        }
        printf("found %d minima, %d have error < %f\n",minima.size(),validcount,maxerror);*/
        
    }
	
	int min_level;
	CvPoint min_pos;
	double min_val;
	R.findMinimum(min_level,min_pos,min_val);
	
	cvRectangle( P[min_level], 
				 min_pos, 
				 cvPoint( min_pos.x + tpl->width, min_pos.y + tpl->height ),
				 CV_RGB(0,255,0), 1, 0, 0 );
	
	printf("minimum: level %d, position [%d,%d], value = %f\n",min_level,min_pos.x,min_pos.y,min_val);
	
	/* display image */
    cvNamedWindow( "image", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "pyramid", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );
    cvShowImage( "image", img );
    cvShowImage( "pyramid" , P[0]);
    cvShowImage( "result" , R[0]);
    
    int key = 0;
    int displayIndex = 0;
    
    while(key != 27){
    	
    	if(key == ' '){
    		displayIndex++;
    		if(displayIndex >= P.levels()){displayIndex = 0;}
   			cvShowImage( "pyramid" , P[displayIndex]);
   			cvShowImage( "result" , R[displayIndex]);
    	}
    	
    	key = cvWaitKey( );
    }

    /* free memory */
    cvDestroyWindow( "result" );
    cvDestroyWindow( "pyramid" );
    cvDestroyWindow( "image" );
    cvReleaseImage( &img );
    cvReleaseImage( &tpl );
    cvReleaseImage( &res );
    
}
