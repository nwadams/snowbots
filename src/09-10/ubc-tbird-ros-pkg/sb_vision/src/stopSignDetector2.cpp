#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

//#include <ros/ros.h>
//#include <sb_msgs/StopSignState.h>

/* Struct Definitions */
// Point
typedef struct {
    int x;
    int y;
} Point;

// Line
typedef struct {
  double slope;
  double interception;
} Line;

// Cluster
typedef struct {
  int size;
  Point startPoint;
  Point flag;
} Cluster;

/* Constants */
int PX = 50;	//TODO: Kinda constant !

/* Global Variables */
int height,width,step,channels,size;
int redThreshold;

/* Function signatures */
void filterRGB(uchar *data, uchar *temp);
double detectStopSign(uchar *data, uchar *temp);
std::vector<Point> findRedCluster(uchar *data);
std::vector<Point> sortVector(std::vector<Point> vector);
std::vector<Line> sortSlopeVector(std::vector<Line> vector);
std::vector<Line> detectEdges(uchar *temp, std::vector<Point>& boundaryPoint);

int main(int argc, char *argv[])
{
  IplImage* originalImg = 0;
  IplImage* manipulatedImg = 0;

  uchar *data;

  if(argc != 2){ 
    printf("ERROR: Wrong number of input parameters\nUsage: stopSignDetector <red_threshold>\n");
    return -1; 
  }
  redThreshold = atoi(argv[1]);

  /* check if a picture is being passed */
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture ) {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }

  /* load the image */  
  originalImg=cvQueryFrame( capture );

  /* throw error and exit if can't open picture */
  if(!originalImg){
    printf("Could not load the camera\n");
    exit(0);
  }

  manipulatedImg=cvQueryFrame( capture );

  /* get the image data and initializing variables */
  while( 1 )
  {
    // Get one frame
    originalImg = cvQueryFrame( capture );

    if( !originalImg ) {
      fprintf( stderr, "ERROR: frame is null...\n" );
      getchar();
      break;
    }

    // get the image data
    height    = originalImg->height;
    width     = originalImg->width;
    step      = originalImg->widthStep;
    channels  = originalImg->nChannels;
    data      = (uchar *)originalImg->imageData;

    /* creating an array identical to "data[]" and copying values in "data[] into
       it; this array is used as a temperory holder of the manipulated data */
    uchar temp[originalImg->imageSize];
    for (int i =0; i < originalImg->imageSize; i++)
    {
       temp[i] = data[i];
    }

    /* filter the image to Red, Blue and Green components */
    filterRGB(data, temp);

    for (int i =0; i < originalImg->imageSize; i++)
      data[i] = temp[i];

    double confidence = detectStopSign(data, temp);

    std::cout << "CERTAINTY: " << confidence << std::endl;

    for (int i =0; i < originalImg->imageSize; i++)
      data[i] = temp[i];

    /* create a window */
    cvNamedWindow("Original", CV_WINDOW_AUTOSIZE); 
    cvMoveWindow("Original", 500, 100);
    cvNamedWindow("Filter", CV_WINDOW_AUTOSIZE); 
    cvMoveWindow("Filter", 100, 100);

    // show the image
    cvShowImage("Original", originalImg);
    cvShowImage("Filter", manipulatedImg );
    // Do not release the frame!

    //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
    //remove higher bits using AND operator
    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  /* Output some useful info */
  printf("Processing a %dx%d image with %d channels\n",height,width,channels);

  // release the image
  cvReleaseImage(&originalImg );
  cvReleaseImage(&manipulatedImg );
  return 0;
}

double detectStopSign(uchar *data, uchar *temp) //TODO: data array is useless, get rid of it
{
  std::vector<Point> redPixVector;             // a vector containing all the pixels in a cluster
  std::vector<Point> boundaryPoints;           // a vector containing all the pixel on the edges
  std::vector<Line> slopesVector;
  double certainty;

/******************************************************************************/
/******************** Finding the red-cluster in the image ********************/
/******************************************************************************/

  redPixVector  = findRedCluster(temp);

/*
  for(unsigned int i = 0; i < redPixVector.size(); i++)
    printf("[%3d, %3d]", redPixVector.at(i).x, redPixVector.at(i).y);
*/
 
/******************************************************************************/
/*************** Finding the Boundary Pixels of the Red Cluster ***************/
/******************************************************************************/
  if(redPixVector.size() != 0){

  Point tempPoint = redPixVector.front();
  boundaryPoints.push_back(tempPoint);

  // Go through the pixels in the cluster ...
  for(unsigned int i = 0; i < redPixVector.size(); i++){
    // whenever get to the next row ...
    if(redPixVector.at(i).x == tempPoint.x+1){
      // add the first pixel in this row and last pixel in last row to
      // the boundaryPoints vector
      boundaryPoints.push_back(redPixVector.at(i-1));
      boundaryPoints.push_back(redPixVector.at(i));
      tempPoint = redPixVector.at(i);
      if( i == redPixVector.size() - 1)
        boundaryPoints.push_back(redPixVector.at(i));
    }
  }

  /* Adding the pixels contained on the top and bottom sides of the picture
    ( in case they are horizental ) */

  Point endPoint = boundaryPoints.back();

  // This loop does it for the top side
  for(unsigned int i = 1; i < boundaryPoints.size()-1; i++){
    // if the pixels are placed in the first couple of rows ...
    if(boundaryPoints.at(i).x == boundaryPoints.front().x || 
       boundaryPoints.at(i).x == boundaryPoints.front().x-1){
/*
       printf("boundaryPoints.at(i).y = %d, boundaryPoints.front().y = %d\n",
               boundaryPoints.at(i).y, boundaryPoints.front().y);
       printf("boundaryPoints.at(i).x = %d, boundaryPoints.front().x = %d\n",
               boundaryPoints.at(i).x, boundaryPoints.front().x);
*/
      // and if this pixel is to the right of the next pixel on this row
      if(boundaryPoints.at(i).y < boundaryPoints.front().y){
        // Add all the pixels between these two pixels to the boundary vector
        for(int j=boundaryPoints.at(i).y; j < boundaryPoints.front().y; j++){
          tempPoint.x = boundaryPoints.at(i).x;
          tempPoint.y = j;
          boundaryPoints.push_back(tempPoint);
        }
      }
      // else if this pixel is to the left of the next pixel on this row
      else if(boundaryPoints.at(i).y > boundaryPoints.front().y){
        // Add all the pixels between these two pixels to the boundary vector
        for(int j=boundaryPoints.at(i).y; j > boundaryPoints.front().y; j--){
          tempPoint.x = boundaryPoints.at(i).x;
          tempPoint.y = j;
          boundaryPoints.push_back(tempPoint);
        }
      }
      printf("boundaryPoints.size()  = %d\n", boundaryPoints.size());
      break;
    }
  }

  // This loop does it for the bottom side
  for(unsigned int l = 0; l < boundaryPoints.size(); l++){
    // if the pixels are placed in the last two rows ...
    if(boundaryPoints.at(l).x == endPoint.x ||
       boundaryPoints.at(l).x == endPoint.x-1){
/*
       printf("boundaryPoints.at(l).y = %d, boundaryPoints.back().y = %d\n",
               boundaryPoints.at(l).y, endPoint.y);
       printf("boundaryPoints.at(l).x = %d, boundaryPoints.back().x = %d\n",
               boundaryPoints.at(l).x, endPoint.x);
*/
      if(boundaryPoints.at(l).y < endPoint.y){
        for(int m=boundaryPoints.at(l).y; m < endPoint.y; m++){
          tempPoint.x = boundaryPoints.at(l).x;
          tempPoint.y = m;
          boundaryPoints.push_back(tempPoint);
        //  printf("x = %d, y = %d\n", tempPoint.x, tempPoint.y);
        }
      }
      else if(boundaryPoints.at(l).y > endPoint.y){
        for(int m=boundaryPoints.at(l).y; m > endPoint.y; m--){
          tempPoint.x = boundaryPoints.at(l).x;
          tempPoint.y = m;
          boundaryPoints.push_back(tempPoint);
        }
      }
      printf("vectorSize  = %d\n", boundaryPoints.size());
      break;
    }
  }
/*************** Erasing everything (paiting the image white) *****************\
  for(int i=0;i<height;i++)
    for(int j=0;j<width;j++)
    {
        temp[i*step+(j)*channels+0] = 255;
        temp[i*step+(j)*channels+1] = 255;
        temp[i*step+(j)*channels+2] = 255;
    }
/******************************************************************************/

  boundaryPoints = sortVector(boundaryPoints);

/*
  for (unsigned int l = 0; l < boundaryPoints.size(); l++){
    printf("[%d, %d]\n", boundaryPoints.at(l).x, boundaryPoints.at(l).y);
  }
*/

/************* Changing the color of all boundary pixels to black *************/
  for (unsigned int l = 0; l < boundaryPoints.size(); l++){
    temp[boundaryPoints.at(l).x*step+boundaryPoints.at(l).y*channels+0] = 0;
    temp[boundaryPoints.at(l).x*step+boundaryPoints.at(l).y*channels+1] = 0;
    temp[boundaryPoints.at(l).x*step+boundaryPoints.at(l).y*channels+2] = 0;
  }
  printf("vectorSize  = %d\n", boundaryPoints.size());
/******************************************************************************/

//**************************** Slope Calculations ****************************//

  slopesVector = detectEdges(temp, boundaryPoints);
  slopesVector = sortSlopeVector(slopesVector);

  for (unsigned int l = 0; l < slopesVector.size(); l++){
    printf("Line: slope = %f, interception = %f\n", slopesVector.at(l).slope, slopesVector.at(l).interception);
  }


//****************************************************************************//

/****************************** IS STOP SIGN ?! *******************************/

  certainty = 0;

  std::cout << "ratio: " << (height*width)/redPixVector.size() << std::endl;

  if((height*width)/redPixVector.size() > 10){
    std::cout << "Too far..." << std::endl;
    certainty = 0;
  }
  else{
    if(slopesVector.size() <= 2)
      certainty = 0.50;
    else if(slopesVector.size() <= 4)
      certainty = 0.60;
    else if(slopesVector.size() <= 6)
      certainty = 0.70;
    else if(slopesVector.size() > 6 && slopesVector.size() < 10)
      certainty = 0.80;
  }

  int parallel = 0;

  if(certainty > 0.79 && certainty < 0.81){
    for(int i = 0; i < slopesVector.size()-1; i++){
      if(fabs(fabs(slopesVector.at(i).slope) - fabs(slopesVector.at(i+1).slope)) < 0.005){
        std::cout << "Yeah: " << slopesVector.at(i).slope << " = " << abs(slopesVector.at(i+1).slope) << std::endl;
        parallel++;
      }
    }
  }

  if (parallel == 4)
    certainty = 1.0;

  std::cout << "Parallel: " << parallel << std::endl;

  } //end of the big if
  else{
    std::cout << "No red pixel found... No way there is a stop sing!" << std::endl;
    certainty = 0;
  }
  std::cout << "Certainty: " << certainty * 100 << "%" << std::endl;

  return certainty;
/******************************************************************************/

/*
  printf("start point: i = %d, j = %d\n", startPoint[0], startPoint[1]);
  printf("end point:   i = %d, j = %d\n", flag[0], flag[1]);
  printf("height = %d, width = %d\n", height, width);
  printf("clusterSize = %d\n", clusterSize);
  printf("vectorSize  = %d\n", redPixVector.size());
  printf("clusterNum = %d\n", clusterNum);
*/
}

std::vector<Point> findRedCluster(uchar *temp)
{
  Cluster cluster;
  int clusterNum = 0;                 // # of red clusters
  int clusterSize = 0;                // # of pixels in a red cluster
//  int cluster[] = {0,0,0};            // [Num, Startpoint, Size]

  Point startPoint;           // starting pixel of a cluster
  Point flag;                 // TODO: change type to Point
  std::vector<Point> redPixVector;    // a vector containing all the pixels in a cluster
  std::vector<Cluster> clustersVector;  // a vector containing the starting points of each clusters

  bool clusterExists = false;

  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
        if(temp[i*step+(j)*channels+0] == 0 &&
           temp[i*step+(j)*channels+1] == 0 &&
           temp[i*step+(j)*channels+2] == 255){
             for(int k = 0; k < clustersVector.size(); k++){
               if(clustersVector.at(k).flag.x - PX <= i && clustersVector.at(k).flag.x + PX >= i &&
                  clustersVector.at(k).flag.y - PX <= j && clustersVector.at(k).flag.y + PX >= j){
                    clusterSize++;
                    clustersVector.at(k).flag.x = i;
                    clustersVector.at(k).flag.y = j;
                    clustersVector.at(k).size = clustersVector.at(k).size+1;
                    clusterExists = true;
                    break;
               }
             }
             if(clusterExists == false){
               clusterSize = 0;
               cluster.startPoint.x = i;
               cluster.startPoint.y = j;
               cluster.flag.x = i;
               cluster.flag.y = j;
               cluster.size = 1;
               clustersVector.push_back(cluster);
               clusterNum++;
               //printf("clustersVector.size() = %d\n", clustersVector.size());
             }
             clusterExists = false;
        }
    }
  }

std::cout << "clusterNum = " << clusterNum << std::endl;
std::cout << "clustersVector.size() = " << clustersVector.size() << std::endl;

  if(clustersVector.size() != 0){
  /* Finding the biggest cluster */
  cluster.size = clustersVector.at(0).size;
  cluster = clustersVector.at(0);

  for(int k = 0; k < clustersVector.size(); k++){
 //   std::cout << "cluster.startPoint [x,y] = [" << clustersVector.at(k).startPoint.x << ", " <<  clustersVector.at(k).startPoint.y << "]" << std::endl;
 //   std::cout << "clustersVector.size() = " << clustersVector.at(k).size << std::endl;
    if(clustersVector.at(k).size > cluster.size){
      cluster.size = clustersVector.at(k).size;
      cluster = clustersVector.at(k);
    }
  }

  /* Set the PX constant */
  int picSize = width * height;
/*
  if(cluster.size <  picSize/30){
    PX = 45;
    std::cout << "case 1" << std::endl;
  }
  else if(cluster.size < picSize/6){
    PX = 50;
    std::cout << "case 2" << std::endl;
  }*/

std::cout << "******** cluster ******** " << std::endl;
std::cout << "cluster.size = " << cluster.size << std::endl;
std::cout << "Area = " << picSize << std::endl;
std::cout << "Area/cluster.size = " << picSize/cluster.size << std::endl;
std::cout << "cluster.startPoint [x,y] = [" << cluster.startPoint.x << ", " << cluster.startPoint.y << "]" <<std::endl;
std::cout << "cluster.flag [x,y] = [" << cluster.flag.x << ", " << cluster.flag.y << "]" <<std::endl;
std::cout << "************************* " << std::endl;

  flag.x = cluster.startPoint.x;
  flag.y = cluster.startPoint.y;

  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      if(i%2 == 0){
        if(temp[i*step+(j)*channels+0] == 0 &&
           temp[i*step+(j)*channels+1] == 0 &&
           temp[i*step+(j)*channels+2] == 255){
             if(((i >= flag.x-PX && i <= flag.x+PX) && (j >= flag.y-PX && j <= flag.y+PX )))
             {
               flag.x = i;
               flag.y = j;
               Point point;
               point.x = flag.x;
               point.y = flag.y;
               redPixVector.push_back(point);
             }
        }
      }
      else{
        if(temp[i*step+(width-j)*channels+0] == 0 &&
           temp[i*step+(width-j)*channels+1] == 0 &&
           temp[i*step+(width-j)*channels+2] == 255){
             if(((i >= flag.x-PX && i <= flag.x+PX) && (width-j >= flag.y-PX && width-j <= flag.y +PX )))
             {
               flag.x = i;
               flag.y = width-j;
               Point point;
               point.x = flag.x;
               point.y = flag.y;
               redPixVector.push_back(point);
             }
             //std::cout << "flag [x,y] = [" << flag.x << ", " << flag.y << "]" <<std::endl;
        }
      }
    }
  }

  std::cout << "redPixVector.size() = " << redPixVector.size() << std::endl;
  std::cout << "Area = " << picSize << std::endl;
  std::cout << "Area/redPixVector.size() = " << picSize/redPixVector.size() << std::endl;

  /************* Visual representation of the red-cluster points **************/
  for (int l = 0; l < redPixVector.size(); l++){
    temp[redPixVector.at(l).x*step+redPixVector.at(l).y*channels+0] = 0;
    temp[redPixVector.at(l).x*step+redPixVector.at(l).y*channels+1] = 0;
    temp[redPixVector.at(l).x*step+redPixVector.at(l).y*channels+2] = 0;
  }
  /****************************************************************************/
  }
  return redPixVector;
}
/*
void findClusterLimits (Cluster& cluster)
{
  int xMin = width;
  int yMin = height;
  int xMax = 0;
  int yMax = 0;

  for (unsigned int l = 0; l < redPixVector.size(); l++){
    if (redPixVector.at(l).x > xMax)
      xMax = redPixVector.at(l).x;
    if (redPixVector.at(l).x < xMin)
      xMin = redPixVector.at(l).x;
    if (redPixVector.at(l).y > yMax)
      yMax = redPixVector.at(l).y;
    if (redPixVector.at(l).y < yMin)
      yMin = redPixVector.at(l).y;
  }

  cluster.xMin = xMin;
  cluster.xMax = xMax;
  cluster.yMin = yMin;
  cluster.yMax = yMax;

  /************ Enclosing the red-cluster area with boundary lines ************
  for(int l=0;l<width;l++){
    temp[xMax*step+l*channels+0] = 0;
    temp[xMax*step+l*channels+1] = 0;
    temp[xMax*step+l*channels+2] = 0;
  }
  for(int l=0;l<width;l++){
    temp[xMin*step+l*channels+0] = 0;
    temp[xMin*step+l*channels+1] = 0;
    temp[xMin*step+l*channels+2] = 0;
  }
  for(int l=0;l<height;l++){
    temp[l*step+yMax*channels+0] = 0;
    temp[l*step+yMax*channels+1] = 0;
    temp[l*step+yMax*channels+2] = 0;
  } 
  for(int l=0;l<height;l++){
    temp[l*step+yMin*channels+0] = 0;
    temp[l*step+yMin*channels+1] = 0;
    temp[l*step+yMin*channels+2] = 0;
  }
  /****************************************************************************
}*/

std::vector<Line> detectEdges(uchar *temp, std::vector<Point>& boundaryPoints)
{
  Line myLine;
  std::vector<Line> slopes;
  Point point1, point2;
  point1 = boundaryPoints.front();
  point2 = boundaryPoints.front();

  int repCounter=0;
  const int acceptableRep = 12;
  const double MAX_INTERCEPTION_RANGE = 20;
  const double MAX_SLOPE_RANGE = 0.2;
  double tempSlope = 999;
  bool slope_exists;
  const double infSlope = 0.04;

  /*** Finding the slope of edges at the right side of the object ***/

  //Among the boundary points...
  for(int i=0; i < boundaryPoints.size(); i++){
    // ... if the point is in the next row (relative to the current row) ...
    if(boundaryPoints.at(i).x==point1.x+1){
      // ... take that point as a second sample point
      point2=boundaryPoints.at(i);

//printf("point1 = [%d,%d], point2 = [%d,%d]\n", point1.x, point1.y, point2.x, point2.y);
      
      if(abs((double)point1.y-point2.y) < 0.001){
        if ((double)point1.y-point2.y < 0)
          myLine.slope = infSlope;
        else
          myLine.slope = -infSlope;
      }
      else
        myLine.slope = ((double)point1.x-point2.x)/(point1.y-point2.y);


      myLine.slope = 0.0001 * round(myLine.slope*10000.0);

      myLine.interception = point1.y - myLine.slope*point1.x;

      if(myLine.slope == tempSlope)
        repCounter++;
 
      slope_exists = false;    
      if(slopes.size() == 0 && repCounter == acceptableRep){
        slopes.push_back(myLine);
        slope_exists = true;
        repCounter = 0;
      }

      else{
        for(int j=0; j < slopes.size(); j++){
          if(slopes.at(j).slope < myLine.slope + MAX_SLOPE_RANGE &&
             slopes.at(j).slope > myLine.slope - MAX_SLOPE_RANGE){
               slope_exists = true;
               repCounter = 0;
               break;
          }
        }
      }
      if (slope_exists == false && repCounter == acceptableRep){
        slopes.push_back(myLine);
        repCounter = 0;
      }

      point1 = point2;
    }
    tempSlope = myLine.slope;
  }

  point1 = boundaryPoints.front();
  point2 = boundaryPoints.front();
  repCounter = 0;
  tempSlope = 999;

  for(int i=0; i < boundaryPoints.size(); i++){
    if(boundaryPoints.at(i).x==point1.x+1){
      for(int j=i; j<boundaryPoints.size() && boundaryPoints.at(j).x==point1.x+1; j++){
        point2=boundaryPoints.at(j);
      }
      
      if(abs((double)point1.y-point2.y) < 0.001){
        if ((double)point1.y-point2.y < 0)
          myLine.slope = -infSlope;
        else
          myLine.slope = infSlope;
      }
      else
        myLine.slope = ((double)point1.x-point2.x)/(point1.y-point2.y);


      myLine.slope = 0.001 * round(myLine.slope*1000.0);

      myLine.interception = point1.y - myLine.slope*point1.x;

      if(myLine.slope == tempSlope)
        repCounter++;
 
      slope_exists = false;    
      if(slopes.size() == 0 && repCounter == acceptableRep){
        slopes.push_back(myLine);
        slope_exists = true;
        repCounter = 0;
      }

      else{
        for(int j=0; j < slopes.size(); j++){
          if(slopes.at(j).slope < myLine.slope + MAX_SLOPE_RANGE &&
             slopes.at(j).slope > myLine.slope - MAX_SLOPE_RANGE){
               if(slopes.at(j).interception < myLine.interception + MAX_INTERCEPTION_RANGE &&
                  slopes.at(j).interception > myLine.interception - MAX_INTERCEPTION_RANGE){
                    slope_exists = true;
                    repCounter = 0;
                    break;
               }
          }
        }
      }
      if (slope_exists == false && repCounter == acceptableRep){
        slopes.push_back(myLine);
        repCounter = 0;
      }

      point1 = point2;
    }
    tempSlope = myLine.slope;
  }

  point1 = boundaryPoints.front();
  point2 = boundaryPoints.front();
  repCounter = 0;
  tempSlope = 999;

  for(int i=0; i < boundaryPoints.size(); i++){
    if(boundaryPoints.at(i).x==point1.x){
      point1 = boundaryPoints.at(i);
      slope_exists = false;
      if(point1.x != point2.x)
        repCounter = 0;
      else
        repCounter++;

        myLine.slope = 999;
        myLine.interception = point1.x;
        for(int j=0; j < slopes.size(); j++){
          if(slopes.at(j).slope > 998 && slopes.at(j).slope < 1000){
             if(slopes.at(j).interception < myLine.interception + MAX_INTERCEPTION_RANGE &&
                slopes.at(j).interception > myLine.interception - MAX_INTERCEPTION_RANGE){
                  slope_exists = true;
                  repCounter = 0;
                  break;
               }
          }
        }
        if (slope_exists == false && repCounter >= 5){
          slopes.push_back(myLine);
          repCounter = 0;
        }
    }
    point2 = point1;
    point1 = boundaryPoints.at(i);
  }

  printf("slopes size  = %d\n", slopes.size());
/*
  for (unsigned int l = 0; l < slopes.size(); l++){
    printf("Line: slope = %f, interception = %f\n", slopes.at(l).slope, slopes.at(l).interception);
  }*/

/*---------------------Drawing the boundary lines---------------------*/
  int x = 0, y = 0;

  for(int i=0; i < slopes.size();i++)
  {
    for(y=0; y < width; y++){
      if (slopes.at(i).slope > 998)
        x = slopes.at(i).interception;
      else
        x = (int)((y-slopes.at(i).interception)/slopes.at(i).slope);

      if(abs(slopes.at(i).slope) <infSlope+0.01 && x > 0 && x < height){
        int nextPointOnLine;
        if (slopes.at(i).slope<0)
          nextPointOnLine = (int)(((y+1)-slopes.at(i).interception)/slopes.at(i).slope);
        else
          nextPointOnLine = (int)(((y-1)-slopes.at(i).interception)/slopes.at(i).slope);

        for(int t=x; t > nextPointOnLine;t--){
          if(t > 0 && t < height){
            temp[t*step+y*channels+0] = 0;
            temp[t*step+y*channels+1] = 0;
            temp[t*step+y*channels+2] = 255; 
          }
        }
      }
      else{
      if ( x*step+y*channels+2 < size && x*step+y*channels+2 >0){
        temp[x*step+y*channels+0] = 0;
        temp[x*step+y*channels+1] = 0;
        temp[x*step+y*channels+2] = 255;
        
      }
      }
    }
  }
/*---------------------------------------------------------*/

  return slopes;
}


void filterRGB(uchar *data, uchar *temp)
{
  for(int i=0;i<height;i++)
    for(int j=0;j<width;j++)
    {
      if(temp[i*step+(j)*channels+2] > redThreshold+data[i*step+j*channels+0] &&
         temp[i*step+(j)*channels+2] > redThreshold+data[i*step+j*channels+1])
      {
        temp[i*step+(j)*channels+0] = 0;
        temp[i*step+(j)*channels+1] = 0;
        temp[i*step+(j)*channels+2] = 255;
      }

      if(temp[i*step+(j)*channels+0] > 20+data[i*step+j*channels+1] &&
         temp[i*step+(j)*channels+0] > 20+data[i*step+j*channels+2])
      {
        temp[i*step+(j)*channels+0] = 255;
        temp[i*step+(j)*channels+1] = 0;
        temp[i*step+(j)*channels+2] = 0;
      }

      if(temp[i*step+(j)*channels+1] > 10+data[i*step+j*channels+0] &&
         temp[i*step+(j)*channels+1] > 10+data[i*step+j*channels+2])
      {
        temp[i*step+(j)*channels+0] = 0;
        temp[i*step+(j)*channels+1] = 255;
        temp[i*step+(j)*channels+2] = 0;
      }
      if(data[i*step+(j)*channels+1] > 150 && data[i*step+(j)*channels+2] > 150 && data[i*step+(j)*channels+0] > 150)
      {
        temp[i*step+(j)*channels+0] = 255;
        temp[i*step+(j)*channels+1] = 255;
        temp[i*step+(j)*channels+2] = 255;
      }
    }
}

/***************************** Sorting Functions ******************************/

std::vector<Point> sortVector(std::vector<Point> vector)
{
  int i, j;
  Point tmp;
  for (i = 1; i < vector.size(); i++) {
    j = i;
    while (j > 0 && vector.at(j - 1).x > vector.at(j).x){
      tmp = vector.at(j);
      vector.at(j) = vector.at(j - 1);
      vector.at(j - 1) = tmp;
      j--;
    }
  }

  for (i=1;i<vector.size(); i++){
    j=i;
    while (j>0 && vector.at(j-1).x == vector.at(j).x && vector.at(j-1).y > vector.at(j).y){
      tmp = vector.at(j);
      vector.at(j) = vector.at(j - 1);
      vector.at(j - 1) = tmp;
      j--;
    }
  }
  
  //Removing any possible duplicate
  for (i=1;i<vector.size(); i++){
    if(vector.at(i-1).x == vector.at(i).x && vector.at(i-1).y == vector.at(i).y)
      vector.erase(vector.begin()+i-1);
  }
  return vector;
}

std::vector<Line> sortSlopeVector(std::vector<Line> vector)
{
  int i, j;
  Line tmp;
  for (i = 1; i < vector.size(); i++) {
    j = i;
    while (j > 0 && vector.at(j - 1).slope > vector.at(j).slope){
      tmp = vector.at(j);
      vector.at(j) = vector.at(j - 1);
      vector.at(j - 1) = tmp;
      j--;
    }
  }

  for (i=1;i<vector.size(); i++){
    j=i;
    while (j>0 && vector.at(j-1).slope == vector.at(j).slope && vector.at(j-1).interception > vector.at(j).interception){
      tmp = vector.at(j);
      vector.at(j) = vector.at(j - 1);
      vector.at(j - 1) = tmp;
      j--;
    }
  }
  
  //Removing any possible duplicate
  for (i=1;i<vector.size(); i++){
    if(vector.at(i-1).slope == vector.at(i).slope && vector.at(i-1).interception == vector.at(i).interception)
      vector.erase(vector.begin()+i-1);
  }
  return vector;
}

/*
std::vector<Point> quickSortVector(std::vector<Point> vector, int left, int right)
{
  int i = left, j = right;
  Point tmp;
  Point pivot = vector.at((right + left) / 2);
  /* partition 
  while (i <= j){
    while (vector.at(i).x < pivot.x)// && vector.at(i).y < pivot.y)
      i++;
    while (vector.at(j).x > pivot.x)// && vector.at(j).y > pivot.y)
      j--;
    if (i <= j){
      tmp = vector.at(i);
      vector.at(i) = vector.at(j);
      vector.at(j) = tmp;
      //printf("Swaped: [%d, %d] with [%d, %d]\n", vector.at(i).x, vector.at(i).y, tmp.x, tmp.y);
      i++;
      j--;
    }
  }
  /* recursion 
  if (left < j)
    quickSortVector(vector, left, j);

  if (i < right)
    quickSortVector(vector, i, right);
  
  return vector;
}
*/
