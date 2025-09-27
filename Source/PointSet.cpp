#include "PointSet.h"
#include "Point.h"
#include <iostream>

PointSet::PointSet()
{
	count = 0;
	count_uv = 0;
	maxpos = 0;
	maxpos_uv = 0;
	
}

int PointSet::addPoint(SPoint& _p)
{
	count++;
	point.push_back(_p);
	enabled.push_back(true);
	return maxpos++;
}

int PointSet::addPoint(SPoint & _p, int index)
{
	count++;
	point.push_back(_p);
	enabled.push_back(true);
	Ptr_SPoint[maxpos] = index;
	return maxpos++;
}

int PointSet::addPoint_uv(SPoint_UV & _p)
{
	count_uv++;
	point_uv.push_back(_p);
	enabled_uv.push_back(true);
	return maxpos_uv++;
}

int PointSet::addPoint_uv(SPoint_UV & _p, int index)
{
	count_uv++;
	point_uv.push_back(_p);
	enabled_uv.push_back(true);
	Ptr_SPoint_UV[maxpos_uv] = index;
	return maxpos_uv++;
}


void PointSet::calculateD_QEMUVMat()
{
	for (int p = 0; p < count; ++p) {
		point.at(p).calculateD_QEMMat(this);
	}
}

void PointSet::delPoint(int pos)
{
	enabled[pos] = false;
	count--;
}

void PointSet::delPoint_uv(int pos)
{
	enabled_uv[pos] = false;
	count_uv--;
}


void PointSet::updata()
{
	point.swap(vector<SPoint>());
	count = 0;
	maxpos = 0;
	enabled.clear();
	
	point_uv.swap(vector<SPoint_UV>());
	count_uv = 0;
	maxpos_uv = 0;
	enabled_uv.clear();

	Ptr_SPoint.erase(Ptr_SPoint.begin(), Ptr_SPoint.end());
}


