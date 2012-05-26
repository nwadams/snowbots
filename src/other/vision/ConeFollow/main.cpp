/*
 *
 *	Simple example of combining OpenCvs camera interface and OpenGL
 *
 *  original author:
 *	@author Gordon Wetzstein, now @ The University of British Columbia [wetzste1@cs.ubc.ca]
 *	@date 10/15/06
 *	
 *  modified by:
 *	@author Matthew Baumann, University of British Columbia [mabauman@cs.ubc.ca]
 */


//-----------------------------------------------------------------------------
// includes

#ifdef WIN32
#include <windows.h>
#endif

#include <stdio.h>
//#include <gl/glew.h>
//#include <gl/glut.h>
#include "GLUT/glut.h"

#include <highgui.h>

#include "imageOps.h"
#include "serialBridge.h"

#include "tbrclient.h"

//-----------------------------------------------------------------------------

#define STEER_HDR '"'	//33 this value will precede the steering byte
#define SPEED_HDR '!'	//34 this value will precede the throttle byte

#define MIN_CHAR '#'	//35 character representing the minimum value (max negative)
#define MID_CHAR 'Q'	//81 character representing the middle (zero value)
#define MAX_CHAR '~'	//126 character representing the max value (max positive)

#define CLAMP(val,min,max)	(MAX(MIN(val,max),min))

//-----------------------------------------------------------------------------
// GLUT callbacks and functions

void initGlut(int argc, char **argv);
void displayFunc(void);
void idleFunc(void);
void reshapeFunc(int width, int height);
void mouseFunc(int button, int state, int x, int y);
void mouseMotionFunc(int x, int y);
void keyboardFunc(unsigned char key, int x, int y);
void specialFunc(int key, int x, int y);
//-----------------------------------------------------------------------------

// other [OpenGL] functions
void countFrames(void);
void renderBitmapString(float x, float y, float z, void *font, char *string);

//-----------------------------------------------------------------------------

bool bFullsreen = false;
int nWindowID;

//-----------------------------------------------------------------------------

// parameters for the framecounter
char pixelstring[30];
char statusstring[1024];
int cframe = 0;
int time_v = 0;
int timebase = 0;

//-----------------------------------------------------------------------------

// OpenCV variables

CvCapture *cvCapture = 0;

GLuint cameraImageTextureID;

//-----------------------------------------------------------------------------

int steering_filtered = 0;

int throttle = 0;

//serial controller
rSerial::serialBridge ser;

tbrprobe07::tbrclient TC;

bool bInit = false;

//-----------------------------------------------------------------------------

int interpolate(int x, int x0, int x1, int y0, int y1)
{
	return y0 + (x - x0) * ((float) (y1-y0) / (float) (x1-x0));
}


char Percent2Char(int percent)
{
	int p = CLAMP(percent,-100,100);
	
	int charval = (int) MID_CHAR;
	if(p == 0)
	{
		charval = (int) MID_CHAR;
	}
	else if(p < 0)
	{
		charval = interpolate(p,-100,0,MIN_CHAR,MID_CHAR);
	}
	else
	{
		charval = interpolate(p,0,100,MID_CHAR,MAX_CHAR);
	}
	return (char) charval;
}

void displayFunc(void) {

	if(!bInit) {

		// initialize 1st camera on the bus
		cvCapture = cvCreateCameraCapture(CV_CAP_ANY);
		
		// initialze OpenGL texture		
		glEnable(GL_TEXTURE_RECTANGLE_ARB);

		glGenTextures(1, &cameraImageTextureID);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);

		glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			
		bInit = true;
	}

	IplImage* newImage = cvQueryFrame( cvCapture );
	
	IplImage* orangeness = cvCreateImage( cvGetSize(newImage), 8, 1 );
	//if(!imageops::isolateColor(newImage, orangeness, 12, 10, 180, 255))
	if(!imageops::isolateColor(newImage, orangeness, 60, 15, 110, 255))
  	{
  		std::cout << "IsolateColor failed." << std::endl;
  		exit(1);
  	}



	/*imageops::linepair* pair = imageops::findTriangle(orangeness,4,CV_PI/180,100);
	if(pair != NULL)
	{		
		//draw the onscreen update
		
		imageops::PrintLinePairIntoString(statusstring,pair);		
		imageops::DrawLinePairOnImg(pair,newImage);
		
		CvPoint isect = imageops::lineIntersect(pair->l1,pair->l2);
		int difference = isect.x - (newImage->width / 2);
		std::cout << isect.x << " - " << (newImage->width /2) << " = " << difference << std::endl;
		
		//char msgString[4];
		//sprintf(msgString,"\"%c",Percent2Char(difference/4));
		//ser.sendString(msgString);
		
		TC.setSteering(difference/4);
		
		delete pair;
	}
	else
	{
		sprintf(statusstring,"NULL");
	}*/
	
	std::vector<imageops::line> lineList = imageops::findLines(orangeness,4,CV_PI/180,100);
	
	imageops::line left;
	imageops::line right;
	int lCount = 0;
	int rCount = 0;
	
	left.rho = 0;
	left.theta = 0;
	
	right.rho = 0;
	right.theta = 0;
	
	bool leftPresence = false;
	bool rightPresence = false;
	
	for(int l = 0; l < lineList.size(); l++)
	{
		if( imageops::angleInRange(lineList[l].theta, CV_PI/16, 7*CV_PI/16, CV_PI) )
		{
			rCount++;
			right.rho += lineList[l].rho;
			right.theta += lineList[l].theta;
		}
		else if( imageops::angleInRange(lineList[l].theta, 9*CV_PI/16, 15*CV_PI/16, CV_PI) )
		{
			lCount++;
			left.rho += lineList[l].rho;
			left.theta += lineList[l].theta;
		}
		else
		{
			//std::cout << "theta is out of range" << std::endl;
		}
	
		DrawLineOnImg(lineList[l], newImage, CV_RGB(0,0,255));
	}
	
	int countThreshold = 4;
	
	if(lCount > 0)
	{
		left.rho = left.rho / (double) lCount;
		left.theta = left.theta / (double) lCount;
		
		if(lCount > countThreshold)
		{
			leftPresence = true;
			DrawLineOnImg(left, newImage, CV_RGB(0,255,0));
		}
	}
	
	if(rCount > 0)
	{
		right.rho = right.rho / (double) rCount;
		right.theta = right.theta / (double) rCount;
		
		if(rCount > countThreshold)
		{
			rightPresence = true;
			DrawLineOnImg(right, newImage, CV_RGB(255,0,0));
		}
	}
	
	
	int steering = 0;
	
	//four cases:
	if(leftPresence && rightPresence)
	{
		//we see two lines: compute intersection and head for apex
		CvPoint isect = imageops::lineIntersect(left,right);
		int difference = isect.x - (newImage->width / 2);
		steering = difference/4;
		std::cout << isect.x << " - " << (newImage->width /2) << " = " << difference << std::endl;	
	}
	else if(leftPresence)
	{
		//we only see lines on the left.  Turn right
		steering = 90;
	}
	else if(rightPresence)
	{
		//we only see lines on the right.  Turn left
		steering = -90;
	}
	else
	{
		//we see no lines.  Go straight
		steering = 0;
	}
	
	//TC.setSteering(steering);
	
	
	float lambda = 0.3;
	
	steering_filtered = (float) steering * lambda + (float) steering_filtered * (1.0-lambda);
	
	char msgString[4];
	sprintf(msgString,"\"%c",Percent2Char(-steering_filtered));
	ser.sendString(msgString);
	
	//draw the image to the screen

	if( (newImage->width > 0) && (newImage->height > 0)) {

		// clear the buffers
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glEnable(GL_TEXTURE_RECTANGLE_ARB);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluOrtho2D(0.0,(GLdouble)newImage->width,0.0,(GLdouble)newImage->height);	
		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);

		if(newImage->nChannels == 3)
			glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, newImage->width, newImage->height, 0, GL_BGR, GL_UNSIGNED_BYTE, newImage->imageData);
		else if(newImage->nChannels == 4)
			glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, newImage->width, newImage->height, 0, GL_BGRA, GL_UNSIGNED_BYTE, newImage->imageData);

		glBegin(GL_QUADS);
			/*glTexCoord2i(0,0);
			glVertex2i(0,0);
			
			glTexCoord2i(newImage->width,0);
			glVertex2i(newImage->width,0);
			
			glTexCoord2i(newImage->width,newImage->height);
			glVertex2i(newImage->width,newImage->height);
			
			glTexCoord2i(0,newImage->height);
			glVertex2i(0,newImage->height);*/
			
			glTexCoord2i(0,newImage->height);
			glVertex2i(0,0);
			
			glTexCoord2i(newImage->width,newImage->height);
			glVertex2i(newImage->width,0);
			
			glTexCoord2i(newImage->width,0);
			glVertex2i(newImage->width,newImage->height);
			
			glTexCoord2i(0,0);
			glVertex2i(0,newImage->height);
		glEnd();

	}

	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	countFrames();
	
	
	//draw in the status text ----------
	glDisable(GL_LIGHTING);
	glColor4f(1.0,1.0,1.0,1.0);
	glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, 200, 0, 200);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// render the string
	renderBitmapString(40,5,0.0,GLUT_BITMAP_HELVETICA_10,statusstring);
		
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	//----------------------------------

	glutSwapBuffers();
}

//-----------------------------------------------------------------------------

void initGlut(int argc, char **argv) {

	// GLUT Window Initialization:
	glutInit (&argc, argv);
	glutInitWindowSize (640, 480);
	glutInitWindowPosition(100, 100);
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	nWindowID = glutCreateWindow ("simpleGLUT - CvCamera");

	// Register callbacks:
	glutDisplayFunc		(displayFunc);
	glutReshapeFunc		(reshapeFunc);
	glutKeyboardFunc	(keyboardFunc);
	glutSpecialFunc		(specialFunc);
	glutMouseFunc		(mouseFunc);
	glutMotionFunc		(mouseMotionFunc);
	glutIdleFunc		(idleFunc);
}



//-----------------------------------------------------------------------------

void idleFunc(void) {
	glutPostRedisplay();
}

//-----------------------------------------------------------------------------

void reshapeFunc(int width, int height) {
	glViewport(0, 0, width, height);
}

//-----------------------------------------------------------------------------


// mouse callback
void mouseFunc(int button, int state, int x, int y) {
	
}

//-----------------------------------------------------------------------------

void mouseMotionFunc(int x, int y) {
	
}

//-----------------------------------------------------------------------------

void keyboardFunc(unsigned char key, int x, int y) {
    
    char msgString[4];
    
	switch(key) {
		
		// -----------------------------------------

#ifdef WIN32
		// exit on escape
		case '\033':

			if(bInit) {
				glDeleteTextures(1, &cameraImageTextureID);
				cvReleaseCapture( &cvCapture );
			}
			exit(0);
			break;
#endif

		// -----------------------------------------
			
		// switch to fullscreen
		case 'f':

			bFullsreen = !bFullsreen;
			if(bFullsreen) 
				glutFullScreen();
			else {
				glutSetWindow(nWindowID);
				glutPositionWindow(100, 100);
				glutReshapeWindow(640, 480);
			}
			break;
		
		//throttle control
		case '=':
			throttle += 2;
			
			sprintf(msgString,"!%c",Percent2Char(throttle));
			ser.sendString(msgString);
		break;
		case '-':
			throttle -= 2;
			sprintf(msgString,"!%c",Percent2Char(throttle));
			ser.sendString(msgString);
		break;
		case ' ':
			throttle = 0;
			sprintf(msgString,"!%c",Percent2Char(throttle));
			ser.sendString(msgString);
		break;
		case '`':
			exit(0);
		break;
		

		// -----------------------------------------
	}
}

//-----------------------------------------------------------------------------

void specialFunc(int key, int x, int y) {
	//printf("key pressed: %d\n", key);
}

//-----------------------------------------------------------------------------

void countFrames(void)  {

	time_v=glutGet(GLUT_ELAPSED_TIME);
	cframe++;
	if (time_v - timebase > 50) {
		sprintf(pixelstring, "fps: %4.2f", cframe*1000.0/(time_v-timebase));		
		timebase = time_v;
		cframe = 0;
	// Draw status text and uni-logo:
	} 
	glDisable(GL_LIGHTING);
	glColor4f(1.0,1.0,1.0,1.0);
	glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, 200, 0, 200);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// render the string
	renderBitmapString(5,5,0.0,GLUT_BITMAP_HELVETICA_10,pixelstring);
		
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

//-----------------------------------------------------------------------------

void renderBitmapString(float x, float y, float z, void *font, char *string) {
  char *c;
  glRasterPos3f(x, y,z);
  for (c=string; *c != '\0'; c++) {
    glutBitmapCharacter(font, *c);
  }
}

//-----------------------------------------------------------------------------


int main(int argc, char **argv) {

	initGlut(argc, argv);
	
  	ser.setObserverStream(&std::cout);
  	ser.init("/dev/tty.usbserial-A600480k",9600);
  	ser.delay(2.0);
  	
  	//TC.setObserver(&std::cout);
  	//TC.initialize(UDP_PORT,LOCALHOST_IP);
	
	
	
	glutMainLoop();
	
	//TC.finalize();
	
	ser.close();
	return 0;
}



