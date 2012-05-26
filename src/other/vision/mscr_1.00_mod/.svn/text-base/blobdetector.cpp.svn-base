/*
**  The blobdetector MSCR wrapper class
**
**   C source files for mscr. (c) 2007 Per-Erik Forssen
**   C++ wrapper (blobdetector namspace) (c) 2009 Matthew Baumann
**
**  This program is free software; you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation; either version 2 of the License, or
**  (at your option) any later version.
**  
**  See the file COPYING for details.
**
*/


#include "blobdetector.h"

namespace mscr_blob{


blobdetector::blobdetector(){
	tb_margin = DEF_MARGIN;
	tb_timesteps = DEF_TIMESTEPS;
	
	kb_n8flag=DEF_N8FLAG;
	kb_normfl=DEF_NORMFL;
	kb_blurfl=DEF_BLURFL; 
	kb_lowresfl=DEF_LOWRESFL;
	kb_cspace=DEF_CSPACE;
}

blobdetector::~blobdetector(){
	
}

blob blobdetector::extract_ellipse(fpnum *mvec,unsigned char *pvec){

	fpnum cogx,cogy;
	fpnum cov[]={0,0,0,0};
	fpnum D[]={0,0};
	fpnum E[]={0,0,0,0};

	cogx=mvec[1];
	cogy=mvec[2];
	cov[0]=mvec[3];
	cov[1]=mvec[4];
	cov[2]=mvec[4];
	cov[3]=mvec[5];
	/* Decompose covariance matrix */
	//D is eigenvalues (length 2)
	//E is eigenvectors (length 4)
	
	eigendec(cov,D,E);
	
	blob newblob;
	
	newblob.pos.x = cogx;
	newblob.pos.y = cogy;
	newblob.D[0] = D[0];
	newblob.D[1] = D[1];
	newblob.E[0] = E[0];
	newblob.E[1] = E[1];
	newblob.E[2] = E[2];
	newblob.E[3] = E[3];
		
	//newblob.color = pvec2rgb(pvec);
	newblob.color = CV_RGB( pvec[2], pvec[1], pvec[0]);  //RGB
	
	//printf("blob colour [%d %d %d]\n",pvec[0],pvec[1],pvec[2]);
	
	//printf("Ellipse pos [%f,%f] D=[%f,%f] E=[%f,%f,%f,%f]\n",cogx,cogy,D[0],D[1],E[0],E[1],E[2],E[3]);
	return newblob;
}


std::vector<blob> blobdetector::extract_ellipses(bbuffer *bf_img,buffer *bf_mvec,bbuffer *bf_pvec) {
	int k,l,m;
	int nblobs;
	int *indl;
	fpnum *mvec_data;
	unsigned char *pvec_data;
	
	std::vector<blob> bloblist;
	
	nblobs=bf_mvec->cols; 
	mvec_data=bf_mvec->data;
	pvec_data=bf_pvec->data;
	
	/* Allocate index list */  
	indl=(int *)calloc(nblobs,sizeof(int)); 
	
	/*
	** Fill in index list using
	** insertion sort of blob areas
	*/
	
	indl[0]=0;  /* Insert first index */
	for(k=1;k<nblobs;k++) {
		/* Find insertion point */
		l=0;
		while((mvec_data[indl[l]*6]>mvec_data[k*6])&&(l<k)) l++;
		
		/* Move old indices up */
		for(m=k;m>l;m--) {indl[m]=indl[m-1];}
		
		/* insert new index */
		indl[l]=k;
	} 
	for(k=0;k<nblobs;k++) {
		//bdraw_ellipse3(bf_img,&mvec_data[indl[k]*6],&pvec_data[indl[k]*3]);
		//printf("color: %d %d %d\n",pvec_data[indl[k]*3], pvec_data[indl[k]*3 + 1], pvec_data[indl[k]*3 +2]);
		bloblist.push_back( extract_ellipse(&mvec_data[indl[k]*6],&pvec_data[indl[k]*3]) );

	}
	free(indl);
	
 	return bloblist;
}

std::vector<blob> blobdetector::HSVFilterBlobs(const std::vector<blob>& bloblist, const CvScalar& color, const CvScalar& margin){
	std::vector<blob> result;
	
	for(int i = 0; i < (int) bloblist.size(); i++)
	{	
		//convert the colour to HSV colour space
		CvScalar HSVcolor = BGR2HSV(bloblist[i].color);
		
		//test if it's within the margins
		bool accept = true;
		for(int c = 0; c < 3; c++)
		{
			//c stands for channel
			
			double diff = 0;
			switch(c){
				case 0: 
					diff = fabs(HSVcolor.val[c] - color.val[c]);
					if(diff > 180){diff = 360-diff;}
				break;
				case 1:
				case 2: 
					diff = fabs(HSVcolor.val[c] - color.val[c]);
				break;
			}
			if(diff > margin.val[c] ){ accept = false;}
			
			//printf("%#0.2f - %#0.2f, diff = %#0.2f vs. %#0.2f\n",HSVcolor.val[c],color.val[c],diff,margin.val[c]);
			
			
		}
		
		//if it is, add it to the result list
		if(accept){ result.push_back(bloblist[i]); }
		
		/*printf("blob: RGB[%#0.2f,%#0.2f,%#0.2f] HSV[%#0.2f,%#0.2f,%#0.2f]\n",
			bloblist[i].color.val[2],bloblist[i].color.val[1],bloblist[i].color.val[0],
			HSVcolor.val[0],HSVcolor.val[1],HSVcolor.val[2]);*/
	}

	return result;
}

CvScalar blobdetector::BGR2HSV(const CvScalar& colorBGR){
	//Formulae from OpenCV Docs entry on CvCvtColor
	
	// V = max(R,G,B)
	// S = (V-min(R,G,B))/V   if V!=0 0 otherwise
	// H = (G-B)*60/S if V=R
	//   = 180+(B-R)*60/S if V=G
	//   = 240+(R-G)*60/S if V=B
	
	double R = colorBGR.val[2] / 255.0;
	double G = colorBGR.val[1] / 255.0;
	double B = colorBGR.val[0] / 255.0;
	
	int maxc = 2;		//index of maximum channel R = 0, G = 1, B = 2
	double V = R;
	if(G > V){V = G;maxc=1;}
	if(B > V){V = B;maxc=0;}
	
	double S = 0;
	if(V != 0)
	{
		double minval = R;	//value of minimum channel 
		if(G < minval){minval = G;}
		if(B < minval){minval = B;}
		
		S = (V-minval)/V;
	}

	double H = 0;
	if(S > 0.01)
	{
		switch(maxc){
			default:
			case 0:
				//V=R
				H = (G-B)*60.0/S;
			break;
			case 1:
				//V=G
				H = 180.0+(B-R)*60/S;
			break;
			case 2:
				//V=B
				H = 240.0+(R-G)*60.0/S;
			break;
		}
		while(H < 0)
		{
			H = H + 360.0;
		}
		while(H > 360.0)
		{
			H = H - 360.0;
		}
	}
	
	return cvScalar(H,S,V);
}

void blobdetector::drawMarkers(std::vector<blob>& bloblist,IplImage* img){

	for(int i = 0; i < (int) bloblist.size(); i++)
	{
		//for each blob, find the center
		CvPoint pos_corrected = bloblist[i].pos;
		pos_corrected.x = img->width - pos_corrected.x;
		
		//cvCircle(img, pos_corrected, 3, bloblist[i].color, 2);


		double a1 =  sqrtf(bloblist[i].D[0]);
		double a2 =  sqrtf(bloblist[i].D[1]);
		double angle = 90 - (360 / (2*PI) * atan(bloblist[i].E[0]/bloblist[i].E[1]));
		
		//printf("Ellipse D = [%f %f]\n",a1,a2);

		cvEllipse(img, pos_corrected, cvSize(a1,a2), angle,0, 360, bloblist[i].color,3);
		cvEllipse(img, pos_corrected, cvSize(a1+1,a2+1), angle,0, 360, CV_RGB(0,255,0),1);

		
		//find the endpoints of the lines along each eigenvector
		
		//draw the line
		//void cvLine( img, CvPoint pt1, CvPoint pt2, CvScalar color);

	}

}

std::vector<blob> blobdetector::detectblobs(IplImage* img){

	buffer *bf_image;
	bbuffer *bf_blobimg;
	
	bf_image = buffer_new(img->height,img->width,3);   //this will need releasing!
	bf_blobimg = bbuffer_new(img->height,img->width,3);    //this too!
	
	buffer_iplcopy(bf_image,img,1);
	
	buffer *bf_pvec,*bf_pvec2;
	bbuffer *bf_pvec8;
	ebuffer *bf_elist,*bf_thres=NULL;
	buffer *bf_mvec,*bf_mvec2,*bf_arate;
	buffer *bf_image2;
	int validcnt,nofedges,rows,cols,ndim;
	edgeval d_max;
	double d_mean;
	
	int min_size=60;          // Default 
	double ainc=1.01;         // Default 
	edgeval min_margin;       // Value after conversion 
	fpnum res=1e-4;           // Default 
	int timesteps=200;        // Default 
	int filter_size=3;        // Default 
	int n8flag=1;             // Default 
	int normfl=1;             // Default 
	int blurfl=0;             // Default 
	int verbosefl=0;          // Default 
	
	// Set margin 
	 min_margin  = tb_margin*EDGE_SCALE+EDGE_OFFSET;
	 
	 // Set timesteps 
	 timesteps = tb_timesteps;
	
	 // Set n8flag 
	 n8flag=kb_n8flag;
	
	 // Set n8flag 
	 normfl=kb_normfl;
	
	 // Set blurfl 
	 blurfl=kb_blurfl;
	
	// Set edge list size 
	 rows = bf_image->rows;
	 cols = bf_image->cols;
	 ndim = bf_image->ndim;
	 if(n8flag) 
	   nofedges=4*rows*cols-3*rows-3*cols+2;
	 else
	   nofedges=2*rows*cols-rows-cols;
	
	 if(kb_cspace) {
	   bf_image2=buffer_new(bf_image->rows,bf_image->cols,bf_image->ndim);
	   image_colourspace(bf_image,bf_image2);
	   bf_image=bf_image2;
	 }
	
	 bf_elist = ebuffer_new(4,nofedges,1);
	
	 if(blurfl) {
	   // Blur input image instead 
	   blur_buffer(bf_image,filter_size);
	   filter_size=1;
	 }
	 if(filter_size%2) {
	   if(n8flag) {
		 if(normfl)
		   d_max=image_to_edgelist_blur_n8_norm(bf_image,bf_elist,filter_size,verbosefl);
		 else
		   d_max=image_to_edgelist_blur_n8(bf_image,bf_elist,filter_size,verbosefl);
	   } else {
		 if(normfl)
		   d_max=image_to_edgelist_blur_norm(bf_image,bf_elist,filter_size,verbosefl);
		 else
		   d_max=image_to_edgelist_blur(bf_image,bf_elist,filter_size,verbosefl);
	   }
	 } else {
	   printf("bfz should be odd.\n");
	 }
	
	 // Call thresholds interpolation 
	 bf_thres=ebuffer_new(1,timesteps,1);
	 d_mean=evolution_thresholds2(bf_elist,bf_thres,ndim);
	 if(verbosefl) printf("d_mean=%g\n",d_mean);
	 // Try linear dependence on mean edge strength 
	 //      min_margin=d_mean*min_margin_sc; 
	
	 // Call computation function
	 edgelist_to_bloblist(&bf_mvec,&bf_pvec,&bf_arate,bf_image,bf_elist,bf_thres,min_size,ainc,res,verbosefl);
	 center_moments(bf_mvec,bf_pvec);
	 validcnt=bloblist_mark_invalid(bf_mvec,min_size,bf_arate,(fpnum)min_margin);
	 validcnt=bloblist_shape_invalid(bf_mvec);
	 if(verbosefl) printf("validcnt=%d\n",validcnt);
	 
	 // Release timestep list
	 ebuffer_free(bf_thres);

	 // Allocate out arrays
	 bf_mvec2 = buffer_new(6,validcnt,1);
	 bf_pvec2 = buffer_new(3,validcnt,1);
	 
	 bloblist_compact2(bf_mvec,bf_mvec2,bf_pvec,bf_pvec2);
	
	 buffer_free(bf_mvec);   // Release non-compacted mvec
	 buffer_free(bf_pvec);   // Release non-compacted pvec
	 buffer_free(bf_arate);  // Release non-compacted arate
	 
	 // Convert pvec to uint8 
 	if(kb_cspace) pvec_colourspace(bf_pvec2);
 	pvec_to_uint8(bf_pvec2,&bf_pvec8);
	 
	std::vector<blob> bloblist = blobdetector::extract_ellipses(bf_blobimg,bf_mvec2,bf_pvec8);

	ebuffer_free(bf_elist);
	buffer_free(bf_mvec2);
 	buffer_free(bf_pvec2);
 	bbuffer_free(bf_pvec8);

	buffer_free(bf_image);
	bbuffer_free(bf_blobimg);

	return bloblist;
}

void blobdetector::setMargin(double m){
	tb_margin = m;
	if(tb_margin > MARGIN_MAX){tb_margin = MARGIN_MAX;}
	if(tb_margin <= 0){tb_margin = 0.0000001;} 
}

void blobdetector::setTimestep(int t){
	tb_timesteps = t;
	if(tb_timesteps > TIMESTEP_MAX){tb_timesteps = TIMESTEP_MAX;}
	if(tb_timesteps <= 0){tb_timesteps = 1;}
}

double blobdetector::getMargin(){
	return tb_margin;
}

int blobdetector::getTimestep(){
	return tb_timesteps;
}


/*
 *  Copy and flip contents of image1 to image2
 *  If image2 is bigger than image1, only the upper left corner is used
 */
void blobdetector::flip_image(IplImage *image1,IplImage *image2) {
unsigned char *image1_data, *image2_data;
 int width,height,ndim,x,y,m;
 int width2,height2;

 image1_data = (unsigned char *)image1->imageData;
 image2_data = (unsigned char *)image2->imageData;
 height = image1->height;
 width = image1->width;
 height2 = image2->height;
 width2 = image2->width;
 ndim = image1->nChannels;
 for(y=0;y<height;y++) {
   for(x=0;x<width;x++) {
     for(m=0;m<ndim;m++) {
       image2_data[m+(width-1-x)*ndim+y*width2*ndim]=
	 image1_data[m+x*ndim+y*width*ndim];
     }
   }
 }
}
/*
 *  Copy and contents of image1 to image2 and upsample
 */
void blobdetector::upsample_image(IplImage *image1,IplImage *image2) {
	unsigned char *image1_data, *image2_data;
	int width,height,ndim,x,y,m;
	int width2,height2;
	int cind;
	unsigned char cval;
	
	image1_data = (unsigned char *)image1->imageData;
	image2_data = (unsigned char *)image2->imageData;
	height = image1->height;
	width = image1->width;
	height2 = image2->height;
	width2 = image2->width;
	ndim = image1->nChannels;
	
	for(y=0;y<height;y++) {
		for(x=0;x<width;x++) {
			for(m=0;m<ndim;m++) {
				cval = image1_data[m+x*ndim+y*width*ndim];
				cind = m+x*2*ndim+y*2*width2*ndim;
				image2_data[cind]= cval;
				image2_data[cind+ndim]= cval;
				image2_data[cind+width2*ndim]= cval;
				image2_data[cind+ndim+width2*ndim]= cval;
			}
		}
	}
}

/*
**  Copy an ipl image into a buffer struct, overwriting contents
**  optionally flipping the x-coordinate
*/
void blobdetector::buffer_iplcopy(buffer *bf_img,IplImage *ipl_img,int flipfl) {
	fpnum *img;
	unsigned char *iplimg_data;
	int width,height,ndim,x,y,m;
	height = ipl_img->height;
	width = ipl_img->width;
	ndim = ipl_img->nChannels;
	if(height != bf_img->rows) {
		printf("Error: image and buffer have different sizes!\n");
		printf("image size=%dx%dx%d, buffer size=%dx%dx%d\n",height,width,ndim,bf_img->rows,bf_img->cols,bf_img->ndim);
		printf("Maybe cvSetCaptureProperty is ignored? Try setting DEF_LOWRESFL 0 and recompile.\n");
		exit(1);
	}
	img = bf_img->data;
	iplimg_data = (unsigned char *)ipl_img->imageData;
	if(flipfl) {
		for(m=0;m<ndim;m++) {
			for(x=0;x<width;x++) {
				for(y=0;y<height;y++) {
					img[y+x*height+m*width*height] = (fpnum)iplimg_data[m+(width-1-x)*ndim+y*width*ndim]/255.0;
				}
			}
		}
	}
	else 
	{
		for(m=0;m<ndim;m++) {
			for(x=0;x<width;x++) {
				for(y=0;y<height;y++) {
					img[y+x*height+m*width*height] = (fpnum)iplimg_data[m+x*ndim+y*width*ndim]/255.0;
				}
			}
		}
	}
}

/*
**  Copy an image buffer into an ipl image, overwriting contents
*/
void blobdetector::paste_image(IplImage *ipl_img,bbuffer *bf_img,int xoffset) {
	unsigned char *img;
	unsigned char  *iplimg_data;
	int width,height,ndim,x,y,m;
	int width2,height2;
	
	img = bf_img->data;
	height = bf_img->rows;
	width = bf_img->cols;
	ndim = bf_img->ndim;
	iplimg_data = (unsigned char *)ipl_img->imageData;
	height2 = ipl_img->height;
	width2 = ipl_img->width;
	
	for(m=0;m<ndim;m++) {
		for(x=0;x<width;x++) {
			for(y=0;y<height;y++) {
				iplimg_data[m+(x+xoffset)*ndim+y*width2*ndim]=img[y+x*height+m*width*height];
			}
		}
	}
}


}
