#pragma once
#ifndef SIMPLIFICATION_H
#define SIMPLIFICATION_H
#include <string>
#include "PointSet.h"
#include "PairHeap.h"
#include <opencv2/opencv.hpp>
#include <osg/Group>
#include <osg/NodeVisitor>
#include <qlist.h>
#include "CU.h"



using std::string;
class Simplification
{
	string inputFile;
	string inputMTL;
	string outputFile;
	string texture_file;
	string texture_filename;
	double ratio;
	
	PointSet pointSet;
	deque<cv::Point2f>pointUV;
	PairHeap pairHeap;

	bool Group_readFile(int stratLine, int &endLine);


	void updateDataUVPoint_D_QEM(Pair &toDel);

	void updateData_Point_D_QEM(Pair &toDel);
	

	osg::Geode* UVQEMCreateLeaf();
	

private:
	int Sim_Method;
	
	bool updatepointSet();
	bool updatepairHeap();

	int pointCount;
	int pointUVCount;
	int faceCount;
public:
	Simplification();

	
	void setInput(string _input);
	void setOutput(string _output);
	void setRatio(double _ratio);
	void D_QEM_Simplify();//D_QEM


	bool split(char* strLine, char ** &res); 
	bool split1(char* strLine, char ** &res); 
	ifstream & seek_to_line(ifstream & in, int line);
	void computeNormal(double v1[3], double v2[3], double v3[3], double n[3]);
	void computeNormalDirection(double v1[3], double v2[3], double v3[3], double n[3]);

	bool judgeTirSame(QList<int> &Tir_index, int Tir);
	bool IsInBox(vector<int> pointindex, int index[3]);
};

#endif