#include "PairHeap.h"
#include <map>
#include <iostream>
#include <fstream>
#include "CU.h"

using namespace std;
PairHeap::PairHeap()
{
	count = 0;
}

void PairHeap::setupHeapD_QEM(PointSet * Set)
{
	int s1 = 0;
	int s2 = 0;

	int edge = 0;
	for (int i = 0; i < Set->count; ++i) {
		for (int j = 0; j < (int)Set->point.at(i).neighbor.size(); ++j)
		{
			int k = Set->point.at(i).neighbor[j];
			if (i < k)
			{
				edge++;
				Pair toAdd(i, k);
				if (toAdd.JudgeBoundary(Set) == true) {
					Set->point.at(i).point_changeable = false;
					Set->point.at(k).point_changeable = false;
				}

				double Sharpness1 = 0;
				double Sharpness2 = 0;
				toAdd.calculateVertexSharpness(Set, toAdd.v1, Sharpness1);
				toAdd.calculateVertexSharpness(Set, toAdd.v2, Sharpness2);
				double Vertex_sharpness = Sharpness1 + Sharpness2;
				
				double Seam1 = 0;
				double Seam2 = 0;
				toAdd.calculateSeamError(Set, toAdd.v1, Seam1);
				toAdd.calculateSeamError(Set, toAdd.v2, Seam2);
				double Seam_angleerror = (Seam1 + Seam2) / 2;
				
				double Texture_complexity = toAdd.calculate_TexComplexity(Set);

				
				int T_edge = toAdd.JudgeStrategy(Set);
				bool Success;
				toAdd.calculateBestPointD_QEM(Set, Success, T_edge);
				
				toAdd.calculateDelCostD_QEM(Set, T_edge, Vertex_sharpness, Seam_angleerror, Texture_complexity);
				
				if (toAdd.calculateMaxNormalDeviation(Set) >= 1) {
					double ppp = toAdd.delCost;
					toAdd.delCost = toAdd.delCost + 1000000000000000000;
				}

				addPair(&toAdd);
			}
		}
	}
}

void PairHeap::addPair(Pair* p)
{
	count++;
	pairQueue.push(*p);
	mapper.insert(map<pair<int, int>, bool> ::value_type(make_pair(p->v1, p->v2), true));//map
	
}

void PairHeap::deletePair(Pair* p)
{
	count--;
	mapper[make_pair(p->v1, p->v2)] = false;
}

void PairHeap::skipPair(Pair * p)
{
	count--;
	mapper[make_pair(p->v1, p->v2)] = false;
}

Pair PairHeap::top(PointSet* set)
{
	while (!pairQueue.empty())
	{
		Pair toret = pairQueue.top();
		pairQueue.pop();
		if (mapper[make_pair(toret.v1, toret.v2)] == true)
			if (set->point.at(toret.v1).point_changeable == true && set->point.at(toret.v2).point_changeable == true)
				return toret;
	}
	Pair empty(-1, -1);
	return empty;
}

void PairHeap::updata()
{
	count = 0;
	while (!pairQueue.empty()) pairQueue.pop();
	mapper.erase(mapper.begin(), mapper.end());
}

bool PairHeap::cmp::operator()(Pair& p1, Pair& p2)
{
	return p1.delCost > p2.delCost;
}
