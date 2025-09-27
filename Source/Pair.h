#pragma once
#ifndef PAIR_H
#define PAIR_H

#include "Point.h"
#include "PointSet.h"
#include <osg/Vec3f>
#include <osg/Geometry>

using namespace std;
struct Pair
{
	int v1;
	int v2;
	int strat_Point;
	int end_Point;
	int Edge_type;

	SPoint bestPoint;
	double delCost;
	bool changeable;

	Pair();
	Pair(int _v1, int _v2);
	void sort();
	bool JudgeBoundary(PointSet* set);
	bool JudgeUVBoundary(PointSet* set, int &V1, int &V2);
	int JudgeTex(PointSet* set, int &V1, int &V2);
	int JudgeStrategy(PointSet* set);
	double calculate_TexComplexity(PointSet* set);
	
	void calculateBestPointD_QEM(PointSet* Set, bool &success, int edge_type );

	double calculateMaxNormalDeviation(PointSet* set);
	bool JudgeAbnormalCollpase(PointSet* set);
	bool ChangeTopology(PointSet* set);
	bool calculateVertexSharpness(PointSet* set, int v1, double &Sharpness);
	bool CalculateVertexSharpness(PointSet* set, int v1, double &Sharpness);
	double calculateTriangularArea(cv::Vec3d P1, cv::Vec3d P2, cv::Vec3d P3);

	bool calculateSeamError(PointSet* set, int v1, double &SeamError);

	void calculateBestUV(PointSet* set);
	cv::Vec3d calculateProjectionPoint(cv::Vec3d plane[3], cv::Vec3d P);
	bool JudgeInclusion(cv::Vec3d plane[3], cv::Vec3d P);
	
	void calculateDelCostD_QEM(PointSet* set, int type, double V_sharpness, double S_angleerror, double T_complexity);
	
	double calcuateAngleError(PointSet* set, int strat_Point, int end_Point);

};

#endif