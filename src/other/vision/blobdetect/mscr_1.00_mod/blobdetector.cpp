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

std::vector<blob> blobdetector::UnwarpBlobs(const std::vector<blob>& bloblist, CvSize src_size, CvSize dst_size){
	std::vector<blob> result;

	//double w1 = src_size.width;
	//double l1 = src_size.height;
	//double w2 = dst_size.width;
	//double l2 = dst_size.height;

	for(int i = 0; i < (int) bloblist.size(); i++){
		
		//determine new blob position
		//double xB = (w1-w2)/(w1*l1) * bloblist[i].pos.x * bloblist[i].pos.y + bloblist[i].pos.x;
		//double yB = bloblist[i].pos.y * l2/l1;
		
		CvPoint B = warp(bloblist[i].pos,src_size,dst_size);
		
		//determine new blob covariance eigenvalues
		
		//determine new blob covariance eigenvectors
		
		//insert the blob into the result list
		blob tmp;
		tmp.pos.x = B.x;
		tmp.pos.y = B.y;
		
		//CvPoint D = warp(cvPoint(bloblist[i].D[0],bloblist[i].D[1]),src_size,dst_size);
		
		//tmp.D[0] = D.x;
		//tmp.D[1] = D.y;
		
		tmp.D[0] = bloblist[i].D[0];
		tmp.D[1] = bloblist[i].D[1];
		
		//CvPoint E01 = unwarp(cvPoint(bloblist[i].E[0],bloblist[i].E[1]),src_size,dst_size);
		//CvPoint E23 = unwarp(cvPoint(bloblist[i].E[2],bloblist[i].E[3]),src_size,dst_size);
		
		tmp.E[0] = bloblist[i].E[0];
		tmp.E[1] = bloblist[i].E[1];
		tmp.E[2] = bloblist[i].E[2];
		tmp.E[3] = bloblist[i].E[3];
		
		//tmp.E[0] = E01.x;
		//tmp.E[1] = E01.y;
		//tmp.E[2] = E23.x;
		//tmp.E[3] = E23.y;
		
		tmp.color = bloblist[i].color;
		
		result.push_back(tmp);
	}
	return result;
}

CvPoint blobdetector::warp(CvPoint p, CvSize src_size, CvSize dst_size){
	//double w1 = src_size.width;
	//double l1 = src_size.height;
	//double w2 = dst_size.width;
	//double l2 = dst_size.height;
	
	//double xB = (w1-w2)/(w1*l1) * p.x * p.y + p.x;
	//double yB = p.y * l2/l1;
	
	//return cvPoint(xB,yB);
	
	//take point p, convert to a bottom-center origin coord space
	//perform the trapezoidal warp onto it
	//convert it to the top-left origin coord space
	//return its value.
	
	CvPoint p_map = scrn2map(p,src_size);
	CvPoint p_map_warped = tWarp( p_map, src_size, dst_size, (int) -( (double) dst_size.height * 0.2 ) );
	return map2scrn( p_map_warped, dst_size);
}

//trapezoidal warp
CvPoint blobdetector::tWarp(CvPoint& p_rect, CvSize& rect_size, CvSize& trap_size, double delta){
	double alpha = (double) trap_size.width / (double) (rect_size.width * rect_size.height);
	double beta = 0;
	double gamma = (double) (trap_size.height - delta) / (double) rect_size.height;
	
	double x1 = alpha * (double) p_rect.x * (double) p_rect.y + beta;
	double y1 = gamma * (double) p_rect.y + delta;
	
	return cvPoint((int) x1, (int) y1);
}

CvPoint blobdetector::tUnWarp(CvPoint& p_trap, CvSize& rect_size, CvSize& trap_size, double delta){
	return cvPoint(0,0);
}

//coordinate transforms
CvPoint blobdetector::scrn2map(const CvPoint& ps,const CvSize& s){
	return cvPoint(ps.x - (s.width / 2), s.height - ps.y);
}

CvPoint blobdetector::map2scrn(const CvPoint& pm,const CvSize& s){
	return cvPoint(pm.x + (s.width/2), -pm.y + s.height);
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
		//CvPoint pos_corrected = bloblist[i].pos;
		//pos_corrected.x = img->width - pos_corrected.x;
		
		//cvCircle(img, pos_corrected, 3, bloblist[i].color, 2);


		double a1 =  sqrtf(bloblist[i].D[0]);
		double a2 =  sqrtf(bloblist[i].D[1]);
		double angle = 90 - (360 / (2*PI) * atan(bloblist[i].E[0]/bloblist[i].E[1]));
		
		//printf("Ellipse D = [%f %f]\n",a1,a2);

		//cvEllipse(img, pos_corrected, cvSize(a1,a2), angle,0, 360, bloblist[i].color,3);
		//cvEllipse(img, pos_corrected, cvSize(a1+1,a2+1), angle,0, 360, CV_RGB(0,255,0),1);
		
		cvEllipse(img, bloblist[i].pos, cvSize(a1,a2), angle,0, 360, bloblist[i].color,3);
		cvEllipse(img, bloblist[i].pos, cvSize(a1+1,a2+1), angle,0, 360, CV_RGB(0,255,0),1);

		
		//find the endpoints of the lines along each eigenvector
		
		//draw the line
		//void cvLine( img, CvPoint pt1, CvPoint pt2, CvScalar color);

	}

}

void blobdetector::drawPath(IplImage* img, double r, int ox, int oy, CvScalar color){
	
	//int w = img->width;
	int h = img->height;
	
	int radius = 1000.0-fabs(r);
	
	//int steps = 8;
	
	if(fabs(r) < 50){
		//think straight... 
		cvLine( img, cvPoint(ox,oy), cvPoint(ox + (int) (r/1.0),0),color,3);
		cvLine( img, cvPoint(ox,oy), cvPoint(ox,h),color,3);
		//printf("straight line\n");
	}
	else
	{
		cvCircle(img, cvPoint( r > 0 ? ox + radius : ox - radius ,oy),radius,color,3);

	
		//printf("curve\n");
		/*for(int i = 0; i < steps; i++)
		{
			
			int x1 = ox + (int) ( (double) i * (double) abs(w-ox) / (double) steps );
			int x2 = ox + (int) ( (double) (i+1) * (double) abs(w-ox) / (double) steps );
			
			int y1 = oy + (int) sqrt( r*r - (double) abs(x1-ox));
			int y2 = oy + (int) sqrt( r*r - (double) abs(x2-ox));
			
			cvLine( img, cvPoint(x1,y1), cvPoint(x2,y2),color,3);
			printf("[%d,%d] [%d,%d]\n",x1,y1,x2,y2);
			
		}*/
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
	
	//correct the x-ccordinates of the blobs
	for(int b = 0; b < (int) bloblist.size(); b++){
		bloblist[b].pos.x = img->width - bloblist[b].pos.x;
	}

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

/************************************************************************/


pathFinder::pathFinder(int count, CvPoint anchor, double step, double steeringLimit){
	//create the curves
	
	double interval = steeringLimit * 2 / (double) (count-1);
	
	for(int i = 0; i < count; i++){
		curves.push_back(curve(anchor,step,steeringLimit));
		curves[i].setSteering(-steeringLimit + ( (double) i * interval) );
		penalties.push_back(0);
	}
	
	mainPath = new curve(anchor,step,steeringLimit);
	
	min_penalty = 0.0;
	max_penalty = 1.0;
}
		
pathFinder::~pathFinder(){
	delete mainPath;
}
		
double pathFinder::evaluate(std::vector<blob>& bloblist){
	int min = 0;
	double minval = DBL_MAX;
	double maxval = 0.0;
	
	
	for(int i = 0; i < (int) curves.size(); i++){
		penalties[i] = curves[i].curveError(bloblist);
		if(penalties[i] < minval){
			min = i;
			minval = penalties[i];
		}
		if(penalties[i] > minval){
			maxval = penalties[i];
		}
	}
	
	printf("penalties=[");
	for(int i = 0; i < (int) curves.size(); i++){
		printf(" %#1.3f,",penalties[i]);
	}
	printf(" ]\n");

	//if(minval < min_penalty){min_penalty = minval;}
	//if(maxval > max_penalty){max_penalty = maxval;}
	min_penalty = minval;
	max_penalty = maxval;
	
	
	double mix = 0.2;
	mainPath->setSteering(curves[min].getSteering() * mix + mainPath->getSteering() * (1.0-mix));
	
	return mainPath->getSteering();
	
}

void pathFinder::drawCurves(IplImage* img){
	for(int i = 0; i < (int) curves.size(); i++){
		//compute the colour
		double r = double_interpolate(penalties[i],
				min_penalty,(min_penalty+max_penalty)/2.0,max_penalty,
				0,255,255);
		double g = double_interpolate(penalties[i],
				min_penalty,(min_penalty+max_penalty)/2.0,max_penalty,
				255,255,0);
		
		curves[i].drawPath(img,CV_RGB((int) r,(int) g,0),1);
	}
}

void pathFinder::drawPath(IplImage* img){
	mainPath->drawPath(img,CV_RGB(0,255,255),3);
}



/************************************************************************/

void curve::drawPath(IplImage* img, CvScalar color, int thickness){
	for(int i = 0; i < PATH_NODES - 1; i++)
	{
		cvLine( img, nodes[i], nodes[i+1], color,thickness);
	}
}

curve::curve(const CvPoint& anchor, double step, double steeringLimit){
	this->anchor.x = anchor.x;
	this->anchor.y = anchor.y;
	this->step = step;
	this->steeringLimit = steeringLimit;
	steering = 0;
	refreshPath(steering);
}

/*double curve::radius(double steering){
	return wheelbase / tan(steering);
}

double curve::incrementAngle(double steering, int increments){
	return (PI - (steering / (double) increments)) / 2.0;
}

CvPoint curve::nextPerimeterPoint(CvPoint& prevPoint, double steering, double step, int increments, int index){
	printf("incrementAngle = %f\n",incrementAngle(steering,increments));
	
	double angle = (PI/2) - ((double) index + 1.0) * incrementAngle(steering,increments);
	printf("angle = %f\n",angle);
	return cvPoint( prevPoint.x + (int) (step*sin(angle)), prevPoint.y + (int) (step*cos(angle)) );
}*/

double curve::autoSteering(std::vector<blob>& blobList, double gain, int maxIterations){
	for(int k = 0; k < maxIterations; k++){
		
		double d = 0;
		//for each blob
		for(int b = 0; b < (int) blobList.size(); b++){
			//get the distance, add to sum
			double md = minDist(blobList[b].pos);
			if(md != 0.0){ d += 1.0/(md);}
		}
		//multiply the total force by the gain
		//add the product to the steering
		double delta = gain * d;
		printf("%d, delta = %f, %d\n",k,delta,(int) blobList.size());
		if(fabs(delta) < 0.001){break;}
		
		setSteering(steering - delta);
		printf("steering = %f\n",steering);
	}
	return steering;
}

double curve::curveFit(std::vector<blob>& blobList, double gain, int maxIterations){
	
	int exp = 2;
	
	for(int i = 0; i < maxIterations; i++){
		double force = 0;
		
		//for each blob
		for(int b = 0; b < (int) blobList.size(); b++){
			//compute blob area
			double a1 =  sqrtf(blobList[b].D[0]);
			double a2 =  sqrtf(blobList[b].D[1]);
			double area = PI * a1 * a2;
			
			double blobForce = 0;
			for(int k = 1; k < PATH_NODES; k++){
				//compute force direction
				double sign = 1.0;
				if( (blobList[b].pos.x - nodes[k].x) < 0)
				{
					sign = -1.0;
				}
				
				
				//compute 1/dist^exp
				double d = dist(blobList[b].pos,nodes[k]);
				double invDist = d;
				for(int e = 1; e < exp; e++)
				{
					invDist *= d;
				}
				invDist = 1.0 / invDist;
				
				//compute dot product
				math::R2 r(blobList[b].pos.x - nodes[k].x , blobList[b].pos.y - nodes[k].y);
				math::R2 c(nodes[k-1].x-nodes[k].x , nodes[k-1].y-nodes[k].y);
				double dotProd = r*c;
				
				double r_norm = r.norm();
				double c_norm = c.norm();
				
				dotProd /= r_norm;
				dotProd /= c_norm;
				
				dotProd = fabs(dotProd);
				
				//printf("r=[%d,%d],c=[%d,%d]\n",r.x,r.y,c.x,c.y);
				
				blobForce += (sign * invDist * dotProd); 
				//printf("blobforce = %f = %f * %f * %f\n",sign*invDist*dotProd,sign,invDist,dotProd);
			}
			
			
			force += area * blobForce;
			
		}
		printf("force = %f\n",force);
		
		if(force < 0.01){break;}
	
		setSteering(steering + (gain * force) );
	}
	return steering;
	
}

double curve::curveError(std::vector<blob>& blobList){
	
	double d = 0;
	//for each blob
	for(int b = 0; b < (int) blobList.size(); b++){
		//get the distance, add to sum
		double md = fabs(minDist(blobList[b].pos));
		double distPenalty = saturation(md,0,50,100,0);
		double area = PI * sqrtf(blobList[b].D[0]) * sqrtf(blobList[b].D[1]);
		
		printf("blob %d, dist=%#1.3f, pen=%#1.3f, area=%#1.3f, prod=%#1.3f\n",b,md,distPenalty,area,area*distPenalty);
		
		d += (area*distPenalty);
		
	}
	return d;
}

double curve::minDist(const CvPoint& p){
	double min = 100000.0;
	double dir = 1;
	for(int i = 1; i < PATH_NODES; i++){
		double d = dist(p,nodes[i]);
		if(d < min){
			min = d;
			p.x < nodes[i].x ? dir = 1 : dir = -1;
			//printf("p.x = %d, nodes[%d].x = %d, dir = %#1.1f\n",p.x,i,nodes[i].x,dir);
		}
	}
	return min * dir;
}

double curve::nearestDist(const CvPoint& p){
	double min = DBL_MAX;
	for(int i = 1; i < PATH_NODES; i++){
		double d = dist(p,nodes[i]);
		if(d < min){
			min = d;
		}
	}
	return min;
}

void curve::setSteering(double s){
	steering = s;
	if(steering > steeringLimit){steering = steeringLimit;}
	if(steering < -steeringLimit){steering = -steeringLimit;}
	refreshPath(steering);
}

double curve::getSteering(){
	return steering;
}

void curve::refreshPath(double steering){
	
	double step_angle = steering / PATH_NODES;
	nodes[0].x = anchor.x;
	nodes[0].y = anchor.y; 
	
	double step_length = step * (1.5-fabs(steering)/steeringLimit);
	
	for(int i = 1; i < PATH_NODES; i++){
		nodes[i].x = nodes[i-1].x + (step_length * sin(i*step_angle));
		nodes[i].y = nodes[i-1].y - (step_length * cos(i*step_angle));
	}
}

double curve::dist(const CvPoint& p1, const CvPoint& p2){
	double dx = (double) p1.x - (double) p2.x;
	double dy = (double) p1.y - (double) p2.y;
	return sqrtf( dx*dx + dy*dy );
}

double curve::dot(const CvPoint& p1, const CvPoint& p2){
	return p1.x*p2.x + p1.y*p2.y;
}

CvPoint curve::normalise(const CvPoint& p){
	double norm = sqrt(p.x * p.x + p.y * p.y);
	return cvPoint(p.x / norm , p.y / norm);
}

/************************************************************************************/

double interpolate(double x, double x0, double x1, double y0, double y1){
	return y0 + (x-x0)*(y1-y0)/(x1-x0);
}

double double_interpolate(double x, double x0, double x1, double x2, double y0, double y1, double y2){
	if(x < x1){
		return interpolate(x,x0,x1,y0,y1);
	}
	else{
		return interpolate(x,x1,x2,y1,y2);
	}
}

double saturation(double x, double x0, double x1, double y0, double y1){
	if(x < x0){return y0;}
	else if(x > x1){return y1;}
	else{return interpolate(x,x0,x1,y0,y1);}
}

}
