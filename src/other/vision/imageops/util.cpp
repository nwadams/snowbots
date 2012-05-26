//Copyright (C) 2007 Matthew Baumann

#include "util.h"


int util::hSize(CvSeq* seq){
//count the horizontal size of a CvSeq*
	int count = 0;
	CvSeq* iter = seq;
	while(iter != NULL)
	{
		count++;
		iter = iter->h_next;
	}
	return count;
}


int util::vSize(CvSeq* seq){
//count the vertical size of a CvSeq*
	int count = 0;
	CvSeq* iter = seq;
	while(iter != NULL)
	{
		count++;
		iter = iter->v_next;
	}
	return count;
}

int util::clampIndex(int val, int min, int max){
	if(val < min){return min;}
	if(val > max){return max;}
	return val;
}

CvSeq* util::trimShortContours(CvSeq* contours, int min)
//remove any contours that have a "total" less than min
{
	CvSeq* first = NULL;
	int count = 0;
	CvSeq* iter = contours;
	while(iter != NULL)
	{
		if(iter->total >= min)
		{
			//this seq is big enough, move on;
			if(NULL == first) {first = iter;}
			iter = iter->h_next;
			count++;
		}
		else
		{
			//this seq is to small, remove it and continue
			
			//case: current has both a prev and a next
			if(iter->h_prev != NULL && iter->h_next != NULL)
			{
				iter->h_prev->h_next = iter->h_next;
				iter->h_next->h_prev = iter->h_prev;
				CvSeq* temp = iter;
				iter = iter->h_next;
				//delete temp;
			}	
			//case: current has only a previous
			else if(iter->h_prev != NULL)
			{
				iter->h_prev->h_next = NULL;
				CvSeq* temp = iter;
				iter = NULL;
				//delete temp;
			}
			//case: current has only a next
			else if(iter->h_next != NULL)
			{
				iter->h_next->h_prev = NULL;
				first = iter->h_next;
				CvSeq* temp = iter;
				iter = iter->h_next;
				//delete temp;
			}
			//case: current has neither a prev nor a next
			else
			{
				CvSeq* temp = iter;
				iter = NULL;
				//delete temp;
			}
			
		}
		
		
		
	}
	return first;
}

double util::anglediff_d(double angle1, double angle2)
{
	double diff = abs(angle2-angle1);
	
	while(diff > 360.0)
	{
		diff-= 360.0;
	}
	
	return diff;
}

double util::anglediff_r(double angle1, double angle2)
{
	double diff = abs(angle2-angle1);
	
	while(diff > 2*PI)
	{
		diff-= 2*PI;
	}
	
	return diff;
}


















