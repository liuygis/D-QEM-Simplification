#pragma once
#ifndef POINT_H
#define POINT_H

#include <vector>
#include "PointSet.h"
#include <opencv2/opencv.hpp>

using namespace std;

struct PointSet;
 struct SPoint
{
	cv::Vec3d Point_Vector;
	cv::Vec3d Point_rgb;
	cv::Point2d PointUV;
	cv::Vec2d PointUV_Vector;         
	cv::Vec2d PointUV_Vector2;
	set<int> SPointUV_index;
	vector<int> neighbor;
	cv::Mat error;
	cv::Mat Texture_error;
	vector<cv::Mat> Texture_errors;

	bool point_changeable;

	SPoint();
	bool hasNeighbor(int neiId);
	void addNeighbor(int neiId);
	void removeNeighbor(int neiId);

	void calculateD_QEMMat(PointSet* set);

	void error_reset();
	void Texture_error_reset();
	void Texture_errors_reset();
	

	double SeamAngleError(PointSet* set, int num_SPoint_UV);

	bool operator == (const SPoint & obj) const 
	{
		return Point_Vector == obj.Point_Vector;
	}
};


 struct SPoint_UV
 {
	 cv::Vec2d PointUV_Vector;
	 vector<int> neighbor;

	 SPoint_UV();
	 bool hasNeighbor(int neiId);
	 void addNeighbor(int neiId);

	 bool operator == (const SPoint_UV & obj) const 
	 {
		 return PointUV_Vector == obj.PointUV_Vector;
	 }
 };
#endif