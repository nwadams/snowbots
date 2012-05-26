#include "camera.h"

namespace vision{
	
//----------------------------------------------------------------------------	
	
camera::camera(const CvSize& s){
	initialize(NULL,s,CV_CAP_ANY,0,NULL,NULL,NULL);
}

camera::camera(const CvSize& s, int index){
	initialize(NULL,s,index,horizon,NULL,NULL,NULL);
}

camera::camera(const CvSize& s, int index, int horizon, const char* intrinsics_file, const char* distortions_file, const char* homography_file){
	initialize(NULL,s,index,horizon,intrinsics_file,distortions_file,homography_file);
}

camera::camera(CvCapture* capture, const CvSize& s, int index, int horizon, const char* intrinsics_file, const char* distortions_file, const char* homography_file){
	initialize(capture,s,index,horizon,intrinsics_file,distortions_file,homography_file);
}

camera::~camera(){
	finalize();
}

bool camera::captureFrame(){
	if(capture == NULL){
		cout << "capture is NULL" << endl;
		return false;
	}
	frame = cvQueryFrame( capture );
	if(frame == NULL) {cout << "frame is NULL" << endl;}
	cvResize(frame,image_buffer);
	undistort_up_to_date = false;
	return true;
}

bool camera::getFrame(IplImage* dst){
	if(frame == NULL || dst == NULL){return false;}
	cvResize(image_buffer,dst);
	
	return true;
}

bool camera::getLowerFrame(IplImage* dst){
	cvSetImageROI(image_buffer, cvRect(0, horizon, image_buffer->width, (image_buffer->height-horizon)));
	cvResize(image_buffer,dst);
	cvResetImageROI(image_buffer);
	return true;
}

bool camera::getUpperFrame(IplImage* dst){
	cvSetImageROI(image_buffer, cvRect(0, 0, image_buffer->width, horizon));
	cvResize(image_buffer,dst);
	cvResetImageROI(image_buffer);
	return true;
}

bool camera::getUndistortedLowerFrame(IplImage* dst){
	if(!undistort_up_to_date){
		// Rectify our image 
		cvRemap(image_buffer, undistorted_image_buffer, mapx, mapy );
		undistort_up_to_date = true;
	}
	
	cvSetImageROI(undistorted_image_buffer, cvRect(0, horizon, undistorted_image_buffer->width, (undistorted_image_buffer->height-horizon)));
	cvResize(undistorted_image_buffer,dst);
	cvResetImageROI(undistorted_image_buffer);
	return true;
}

bool camera::getUndistortedUpperFrame(IplImage* dst){
	if(!undistort_up_to_date){
		// Rectify our image 
		cvRemap(image_buffer, undistorted_image_buffer, mapx, mapy );
		undistort_up_to_date = true;
	}
	
	cvSetImageROI(undistorted_image_buffer, cvRect(0, 0, undistorted_image_buffer->width, horizon));
	cvResize(undistorted_image_buffer,dst);
	cvResetImageROI(undistorted_image_buffer);
	return true;
}

bool camera::getUndistortedFrame(IplImage* dst){
	if(image_buffer == NULL){
		cout << "image_buffer == NULL" << endl;
		return false;
	}
	
	if(dst == NULL){
		cout << "dst == NULL" << endl;
		return false;
	}
	
	if(mapx == NULL){
		cout << "mapx == NULL" << endl;
		return false;
	}
	
	if(mapy == NULL){
		cout << "mapy == NULL" << endl;
		return false;
	}
	
	if(!undistort_up_to_date){
		// Rectify our image 
		cvRemap(image_buffer, undistorted_image_buffer, mapx, mapy );
		undistort_up_to_date = true;
	}
	
	cvResize(undistorted_image_buffer,dst);
	
	return true;
}

bool camera::getBirdsEyeFrame(IplImage* dst){
	
	IplImage* ground = cvCreateImage(cvSize(image_buffer->width,image_buffer->height-horizon),8,3);
	getUndistortedLowerFrame(ground);
	
	cvWarpPerspective( 
	  ground, 
	  dst, 
	  homography, 
	  CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS 
    );
    
    cvReleaseImage(&ground);
	return true;
}

bool camera::calibrated(){
	return intrinsics == NULL && distortions == NULL;
}

bool camera::hasHomography(){
	return homography != NULL;
}

bool camera::computeDistortionMaps(){
	
	//cout << "computing distortion maps" << endl;
	
	//This initializes rectification matrices 
	cvInitUndistortMap( 
		intrinsics, 
		distortions, 
		mapx, 
		mapy 
	); 
	
	return true;
}

bool camera::computeHomography(int board_w, int board_h, IplImage* image, CvPoint2D32f* objPts){
	
	int       board_n    = board_w * board_h; 
	CvSize    board_sz   = cvSize( board_w, board_h ); 
	 
	IplImage* gray_image = 0; 
	
	gray_image = cvCreateImage( cvGetSize(image), 8, 1 ); 
	cvCvtColor(image, gray_image, CV_BGR2GRAY ); 
	
	//IplImage *t = cvCloneImage(image); 
	
	// Rectify our image 
	// cvRemap( t, image, mapx, mapy ); 
	
	// GET THE CHESSBOARD ON THE PLANE 
	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ]; 
	int corner_count = 0; 
	int found = cvFindChessboardCorners( 
		image, 
		board_sz, 
		corners, 
		&corner_count, 
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS 
	); 
	
	cout << "Found " << corner_count << " of " << board_n << " corners." << endl;
	
	if(!found){ 
		printf("Couldn't acquire chessboard, " 
		  "only found %d of %d corners\n", corner_count,board_n 
		); 
		return false; 
	} 
	
	 
	
	//Get Subpixel accuracy on those corners: 
	cvFindCornerSubPix( 
		gray_image, 
		corners, 
		corner_count, 
		cvSize(11,11), 
		cvSize(-1,-1), 
		cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ) 
	); 
	
	//GET THE IMAGE AND OBJECT POINTS: 
	// We will choose chessboard object points as (r,c): 
	// (0,0), (board_w-1,0), (0,board_h-1), (board_w-1,board_h-1). 
	int x_offset = image->width/3;
	int y_offset = 0;
	
	//CvPoint2D32f objPts[4]; 
	//objPts[0].x = 0 + x_offset;         		objPts[0].y = 0 + y_offset; 
	//objPts[1].x = board_w-1 + x_offset; 		objPts[1].y = 0 + y_offset; 
	//objPts[2].x = 0 + x_offset;         		objPts[2].y = board_h-1 + y_offset; 
	//objPts[3].x = board_w-1 + x_offset; 		objPts[3].y = board_h-1 + y_offset; 
	
	CvPoint2D32f imgPts[4]; 
	imgPts[0]   = corners[0];
	imgPts[1]   = corners[board_w-1]; 
	imgPts[2]   = corners[(board_h-1)*board_w]; 
	imgPts[3]   = corners[(board_h-1)*board_w + board_w-1]; 
	
	cout << "Chessboard corners" << endl;
	cout << "[" << imgPts[0].x << "," << imgPts[0].y << "]" << endl;
	cout << "[" << imgPts[1].x << "," << imgPts[1].y << "]" << endl;
	cout << "[" << imgPts[2].x << "," << imgPts[2].y << "]" << endl;
	cout << "[" << imgPts[3].x << "," << imgPts[3].y << "]" << endl;
	
	// DRAW THE POINTS in order: B,G,R,YELLOW 
	//cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255),   3); 
	//cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0),   3); 
	//cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0),   3); 
	//cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0), 3); 
	
	// DRAW THE FOUND CHESSBOARD 
	//cvDrawChessboardCorners( 
		//image, 
		//board_sz, 
		//corners, 
		//corner_count, 
		//found 
	//); 
	
	//check the homography to see if it's already filled
	//if(homography != NULL){
	//	CvMat* temp = homography ;
	//	homography = NULL;
	//	cvReleaseMat(&temp);
	//}
	
	// FIND THE HOMOGRAPHY 
	//homography = cvCreateMat( 3, 3, CV_32F); 
	cvGetPerspectiveTransform( objPts, imgPts, homography);
	
	//cvReleaseImage(&image);
	cvReleaseImage(&gray_image);
	
	return true;
}

bool camera::loadIntrinsics(const char* filename){
	//cout << "loadIntrinsics" << endl;
	if(intrinsics != NULL){
		CvMat* temp = intrinsics;
		intrinsics = NULL;
		//cout << "releasing old memory" << endl;
		cvReleaseMat(&temp);
	}
	
	//cout << "loading intrinsics from " << filename << endl;
	intrinsics = (CvMat*) cvLoad(filename);
	
	if(intrinsics != NULL && distortions != NULL){
		computeDistortionMaps();
	}
	
	return intrinsics != NULL;
}

bool camera::loadDistortions(const char* filename){
	//cout << "loadDistortions" << endl;
	if(distortions != NULL){
		CvMat* temp = distortions;
		distortions = NULL;
		//cout << "releasing old memory" << endl;
		cvReleaseMat(&temp);
	}
	
	//cout << "loading distortions from " << filename << endl;
	distortions = (CvMat*) cvLoad(filename);
	
	if(intrinsics != NULL && distortions != NULL){
		computeDistortionMaps();
	}
	
	return distortions != NULL;
}

bool camera::loadHomography(const char* filename){
	if(homography != NULL){
		CvMat* temp = homography ;
		homography  = NULL;
		cvReleaseMat(&temp);
	}
	homography = (CvMat*) cvLoad(filename);
	return homography != NULL;
}

bool camera::saveIntrinsics(const char* filename){
	cvSave(filename,intrinsics);
	return true;
}

bool camera::saveDistortions(const char* filename){
	cvSave(filename,distortions);
	return true;
}

bool camera::saveHomography(const char* filename){
	cvSave(filename,homography);
	return true;
}

void camera::setHorizon(int horizon){
	this->horizon = horizon;
}

int camera::getHorizon(){
	return horizon;
}

bool camera::initialize(CvCapture* cap, const CvSize& s, int index, int horizon, const char* intrinsics_file, const char* distortions_file, const char* homography_file){
	
	capture = cap;
	
	frame = NULL;
	image_buffer = NULL;
	undistorted_image_buffer = NULL;
	
	intrinsics = NULL;
	distortions = NULL;
	homography = cvCreateMat( 3, 3, CV_32F); 
	
	undistort_up_to_date = false;
	
	image_buffer = cvCreateImage(s,8,3);
	undistorted_image_buffer = cvCreateImage(s,8,3);
	
	mapx = cvCreateImage(s,IPL_DEPTH_32F, 1);
	mapy = cvCreateImage(s,IPL_DEPTH_32F, 1);
	
	if(capture == NULL)
	{
		
		capture = cvCreateCameraCapture(index);
		if(capture == NULL){
			cout << " capture error: cannot capture on index " << index << endl;
			return false;
		}
	}
	
	
	this->horizon = horizon;

	if(intrinsics_file != NULL){
		loadIntrinsics(intrinsics_file);
		if(intrinsics == NULL){return false;}
	}
	
	if(distortions_file != NULL){
		loadDistortions(distortions_file);
		if(distortions == NULL){return false;}
	}
	
	if(homography_file != NULL){
		loadHomography(homography_file);
		if(homography == NULL){return false;}
	} 
	
	return true;
}

bool camera::finalize(){
	if(capture != NULL){
		cvReleaseCapture(&capture);
	}
	
	if(intrinsics != NULL){
		cvReleaseMat(&intrinsics);
	}
	
	if(distortions != NULL){
		cvReleaseMat(&distortions);
	}
	
	if(homography != NULL){
		cvReleaseMat(&homography);
	}
	
	cvReleaseImage(&image_buffer);
	cvReleaseImage(&undistorted_image_buffer);
	
	cvReleaseMat(&homography);
	
	return true;
}




//----------------------------------------------------------------------------

}