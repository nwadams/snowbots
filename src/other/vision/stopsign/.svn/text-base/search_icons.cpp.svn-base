/*
 * icon_search 1.5
 *
 * Search for a particular icon in a screenshot image
 *
 * @author    Nashruddin Amin <me@nashruddin.com>
 * @version   1.5
 * @license   GPL
 * @website   http://www.nashruddin.com
 */

#include <stdio.h>
#include "cv.h"
#include "highgui.h"

int main( int argc, char** argv )
{
    IplImage    *img;
    IplImage    *tpl;
    IplImage    *res;
    int         width, height;
    float       threshold = 0.12;
    int         i, j;

    /* check for arguments */
    if( argc < 3 ) {
        fprintf( stderr, "Usage: icon_search <screenshot> <icon template>\n" );
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
    
    /* perform template matching */
    cvMatchTemplate( img, tpl, res, CV_TM_SQDIFF_NORMED );

    /*
     * now res contains the comparison result from template matching operation.
     * from this result, we find each element which value is lower than threshold
     */

    CvScalar s;
    for( i = 0 ; i < height ; i++) {
        for( j = 0 ; j < width ; j++) {
            /* get an element */
            s = cvGet2D( res, i, j );
			
            /* if value below the threshold, similar object is found */
            if( s.val[0] <= threshold ) {
                /* draw a box to mark the object */
                cvRectangle( img, 
                             cvPoint( j, i ), 
                             cvPoint( j + tpl->width, i + tpl->height ),
                             cvScalar( 0, 0, 0, 0 ), 3, 0, 0 );
            }
        }
    }
    
    /* display image */
    cvNamedWindow( "screenshot", CV_WINDOW_AUTOSIZE );
    cvShowImage( "screenshot", img );
    
    cvWaitKey( 0 );

    /* free memory */
    cvDestroyWindow( "screenshot" );
    cvReleaseImage( &img );
    cvReleaseImage( &tpl );
    cvReleaseImage( &res );

    return 0;
}
