#pragma once
#ifndef POINTSET_H
#define POINTSET_H

#include <deque>
#include <vector>
#include "Point.h"
#include <map>

using namespace std;
struct SPoint;
struct SPoint_UV;
struct PointSet//start from 1
{
	vector<SPoint>point;
	deque<bool>enabled;
	int count;
	int maxpos;
	
	vector<SPoint_UV>point_uv;
	deque<bool>enabled_uv;
	int count_uv;
	int maxpos_uv;

	PointSet();
	int addPoint(SPoint& _p);
	int addPoint(SPoint& _p, int index);

	int addPoint_uv(SPoint_UV& _p);
	int addPoint_uv(SPoint_UV& _p, int index);
	void calculateD_QEMUVMat();//Calculation error matrix
	

	void delPoint(int pos);
	void delPoint_uv(int pos);

	void updata();
};
#endif