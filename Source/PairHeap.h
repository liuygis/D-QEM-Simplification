#pragma once
#ifndef PAIRHEAP_H
#define PAIRHEAP_H

#include "Pair.h"
#include "PointSet.h"
#include <queue>
#include <vector>
#include <map>
#include "CU.h"

using namespace std;

struct PairHeap
{
	struct cmp
	{
		bool operator ()(Pair& p1, Pair& p2);
	};

	priority_queue<Pair, vector<Pair>, cmp> pairQueue;
	map<pair<int, int>, bool> mapper;
	int count;

	PairHeap();
	
	void setupHeapD_QEM(PointSet* Set);
	

	void addPair(Pair* p);
	void deletePair(Pair* p);
	void skipPair(Pair* p);
	Pair top(PointSet* set);

	void updata();
};
#endif