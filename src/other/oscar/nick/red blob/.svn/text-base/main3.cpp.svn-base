/*ALL the necessary header files*/

 #include <math.h>
 #include <opencv/cv.h>
 #include <opencv/highgui.h>
 #include <stdio.h>
int main( int argc, char** argv)
{
int i,j,k;//for iterations
//int temp=0;//if we use a temporary var
/*here lets look at the word “heighthsv” …now lets breadk up this word…here height means

height as a regular IplImage Structure has now the addition “hsv” to the word heigh means this
height attribute is for the image which is color converted to the hsv,Similar conventions
for the monochrome image…so you may find the attribute height for the monochrome image to be
heightmono…So i believe it is easy…*/
int heighthsv,widthhsv,stephsv,channelshsv;
int heightmono,widthmono,stepmono,channelsmono;
uchar *datahsv,*datamono;
int sthreshold=140;
double hlower=175,hupper=5;/*
here hlower is the lower cut off and the hupper is the upper cut off for hue values of red.*/

i=j=k=0;/*initializing the iteraiton variables to be zero*/

IplImage *frame=cvLoadImage(argv[1],1);

if(frame==NULL ) {
puts("unable to load the frame");exit(0);}
printf("frame loaded");
IplImage *colimgbot = cvCreateImage( cvGetSize(frame), 8, 3 );

IplImage *monoimgbot = cvCreateImage( cvGetSize(frame), 8, 1 );

//——————————————————————

heighthsv = colimgbot->height;
widthhsv = colimgbot->width;
stephsv =colimgbot->widthStep;
channelshsv = colimgbot->nChannels;
datahsv = (uchar *)colimgbot->imageData;
//————————–

heightmono = monoimgbot ->height;
widthmono = monoimgbot->width;
stepmono = monoimgbot->widthStep;
channelsmono = monoimgbot->nChannels;
datamono = (uchar *)monoimgbot->imageData;
cvCvtColor(frame,colimgbot,CV_RGB2HSV);
cvNamedWindow("original", CV_WINDOW_AUTOSIZE);

cvNamedWindow("Monochrome Of red Blob",CV_WINDOW_AUTOSIZE);
for(i=0;i< (heighthsv);i++)
{
for(j=0;j<(widthhsv);j++)
{
if((datahsv[(i)*stephsv+j*channelshsv]<=hlower) && (datahsv[(i)*stephsv+j*channelshsv]>=hupper))
{ if((datahsv[(i)*stephsv+j*(channelshsv)+1])>sthreshold){
datamono[i*stepmono+j*channelsmono]=255;}
else
/*A very simple concept with the loops here if the hue values are in the aforementioned range and the
threshold is met then logic one else logic zero*/
datamono[i*stepmono+j*channelsmono]=0;}
}}
for(i=0;i< (heighthsv);i++)
{
for(j=0;j<(widthhsv);j++){
if(!(datamono[i*stepmono+j*channelsmono]==0 || datamono[i*stepmono+j*channelsmono]==255))
datamono[i*stepmono+j*channelsmono]=0;}}/*Just a cross chek to ensure whether all the pixels have only
either 0 or 255*/
/*Please check these links for the explanation of the erosion and dilation functions

http://www.dca.fee.unicamp.br/dipcourse/html-dip/c9/s4/front-page.html*/

/*so now the last parameter in the function indicates how many times you want to apply dilation

or erosion*/

cvErode(monoimgbot,monoimgbot,0,6);
cvDilate( monoimgbot,monoimgbot,0,10);
/*here i have experimented with the values by changing them…and i have found
that i come to a good result by applying erosion 6 times and dilation 15 times
you can comment/uncomment play with the values and see what is going on
Sometimes you will find the areas which are shining in the image also get detected…

Please think why and then try to post a comment the best commment would get visible on this page*/

cvShowImage("original",frame);
cvSaveImage("red-ballmonochrome.jpg",monoimgbot);/*if you want to save the image*/
cvShowImage("Monochrome Of red Blob",monoimgbot);
cvWaitKey(0);
/*for all the other clarifications you can check the other posts…. you will find an answer
People who are getting started with the Opencv must make sure you

check the other posts on this blog*/

cvDestroyWindow("Monochrome Of red Blob");
cvDestroyWindow("original");
return 0;
}
