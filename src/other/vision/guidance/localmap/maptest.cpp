
#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <imageops.h>

#include "localmap.h"

using namespace std;

int map_img_width = 320;
int map_img_height = 320;
double map_width = 4.0;
double map_height = 6.0;

int ox = 160;
int oy = 280;

int key = 0;

int main(int argc, char* argv[]){

	vision::localmap map(cvSize(map_img_width,map_img_height), map_width, map_height, cvPoint(ox,oy));

	cvNamedWindow("Map");
	
	cvShowImage("Map",map.getDisplayImg());
	
	key = cvWaitKey();
	
	cvDestroyWindow("Map");

	return 0;
}
