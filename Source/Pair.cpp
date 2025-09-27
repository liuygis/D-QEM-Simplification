#include "Pair.h"
#include <cmath>
#include<FitPlane.h>
#include <iostream>


using namespace std;
using namespace cv;
Pair::Pair()
{
	v1 = 0;
	v2 = 0;
	Edge_type = 0;
	delCost = 0;
}

Pair::Pair(int _v1, int _v2)
{
	v1 = _v1;
	v2 = _v2;
	delCost = 0;
	Edge_type = 0;
	changeable = true;
}

void Pair::sort()
{
	if (v1 > v2)
	{
		int tmp = v1;
		v1 = v2;
		v2 = tmp;
	}
}

bool Pair::JudgeBoundary(PointSet * set)
{
	int count = 0;
	for (auto iter = set->point.at(v1).neighbor.cbegin(); iter != set->point.at(v1).neighbor.cend(); iter++)
	{
		int pppp = *iter;
		if (std::find(set->point.at(v2).neighbor.begin(), set->point.at(v2).neighbor.end(), *iter) != set->point.at(v2).neighbor.end()) {
			count++;
		}
	}
	/*for (std::vector<int>::const_iterator nIterator = set->point.at(v1).neighbor.begin(); nIterator != set->point.at(v1).neighbor.end(); nIterator++)
	{
		if (std::find(set->point.at(v2).neighbor.begin(), set->point.at(v2).neighbor.end(), *nIterator) != set->point.at(v2).neighbor.end())
			count++;
	}*/
	if (count == 1) {
		return true;
	}
	//if (count <= 0 || count > 3) {
	//	return true;
	//}
	//if (count == 3) {
	//	return true;
	//}
	return false;
}

bool Pair::JudgeUVBoundary(PointSet * set, int &V1, int &V2)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 || UVnum_v2 == 1) {
		if (UVnum_v1 == 1 && UVnum_v2 == 1) {
			V1 = v1;
			V2 = v2;
			return true;
		}
		else 
		{
			if (UVnum_v1 == 1) {
				V1 = v1;
				V2 = -1;
				return true;
			}
			else
			{
				V1 = -1;
				V2 = v2;
				return true;
			}
		}
	}
	else
	{
		V1 = v1;
		V2 = v2;
		return false;
	}
}

int Pair::JudgeTex(PointSet * set, int & V1, int & V2)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 || UVnum_v2 == 1) {
		V1 = UVnum_v1;
		V2 = UVnum_v2;
		return 1;
	}
	else
	{
		if (UVnum_v1 >= 3 && UVnum_v2 >= 3) {
			V1 = UVnum_v1;
			V2 = UVnum_v2;
			return 2;
		}
		else
		{
			V1 = UVnum_v1;
			V2 = UVnum_v2;
			return 3;
		}
	}
	return 0;
}

int Pair::JudgeStrategy(PointSet * set)
{
	int UVnum_v1 = set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = set->point.at(v2).SPointUV_index.size();

	if (UVnum_v1 == 1 && UVnum_v2 == 1) {
		Edge_type = 1;
		return 1;
	}
	else
	{
		if (UVnum_v1 == 1 && UVnum_v2 == 2) {
			Edge_type = 2;
			return 2;
		}
		if (UVnum_v1 == 2 && UVnum_v2 == 1) {
			Edge_type = 2;
			return 2;
		}
		

		if (UVnum_v1 == 2 && UVnum_v2 == 2) {
			Edge_type = 3;
			return 3;
		}
		return 4;
	}
}


double Pair::calculate_TexComplexity(PointSet * Set)
{
	double TexComplexity = 0.0;

	set<int> neighbors;
	for (int i = 0; i < (int)Set->point.at(v1).neighbor.size(); ++i)
	{
		int k = Set->point.at(v1).neighbor[i];
		neighbors.insert(k);
		
	}
	for (int i = 0; i < (int)Set->point.at(v2).neighbor.size(); ++i)
	{
		int k = Set->point.at(v2).neighbor[i];
		neighbors.insert(k);
		
	}

	double denominator = neighbors.size();
	double molecule = 0;
	for (auto a : neighbors) {
		molecule += Set->point.at(a).SPointUV_index.size();
	}

	TexComplexity = molecule / denominator;

	return TexComplexity;
}


void Pair::calculateBestPointD_QEM(PointSet * Set, bool & success, int edge_type)
{
	int UVnum_v1 = Set->point.at(v1).SPointUV_index.size();
	int UVnum_v2 = Set->point.at(v2).SPointUV_index.size();

	switch (edge_type)
	{
	case 1:
		calculateBestPointQEM(Set, success);
		break;
	case 2:
		int startP, endP;
		int p_v1uv;
		int uvsize;
		if (UVnum_v1 == 1) {
			startP = v1;
			endP = v2;
			strat_Point = v1;
			end_Point = v2;
		}
		else
		{
			startP = v2;
			endP = v1;
			strat_Point = v2;
			end_Point = v1;
		}
		bestPoint.Point_Vector = Set->point.at(endP).Point_Vector;

		p_v1uv = *Set->point.at(startP).SPointUV_index.begin();
		uvsize = Set->point.at(endP).SPointUV_index.size();
		for (auto uv : Set->point.at(endP).SPointUV_index) {
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {
				bestPoint.PointUV_Vector = Set->point_uv.at(uv).PointUV_Vector;
				success = true;
			}
		}
		break;
	case 3:
		break;
	case 4:
		break;
	}
}



double Pair::calculateMaxNormalDeviation(PointSet * set)
{
	double Max_error = 0;
	//P1
	SPoint P1 = set->point.at(v1);
	int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	for (int i = 0; i < P1_neigsize; ++i)
	{
		SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j)
		{
			int temp2_index = set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index) && temp2_index != v2)
			{
				SPoint temp2= set->point[temp2_index];
				cv::Vec3d Normal1= (temp1.Point_Vector - temp2.Point_Vector).cross(P1.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal1 = Normal1/sqrt(Normal1[0] * Normal1[0] + Normal1[1] * Normal1[1] + Normal1[2] * Normal1[2]);
				double length = sqrt(Normal1[0] * Normal1[0] + Normal1[1] * Normal1[1] + Normal1[2] * Normal1[2]);
				cv::Vec3d Unit_Norm = Normal1 / length;

				cv::Vec3d Normal2 = (temp1.Point_Vector - temp2.Point_Vector).cross(bestPoint.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal2 = Normal2 / sqrt(Normal2[0] * Normal2[0] + Normal2[1] * Normal2[1] + Normal2[2] * Normal2[2]);
				double error = 1 - Unit_Normal1.dot(Unit_Normal2);
				if (error > Max_error) {
					Max_error = error;
				}
			}
		}
	}
	//P2
	SPoint P2 = set->point.at(v2);
	int P2_neigsize = (int)set->point.at(v2).neighbor.size();
	for (int i = 0; i < P2_neigsize; ++i)
	{
		SPoint temp1 = set->point[set->point.at(v2).neighbor[i]];
		for (int j = i + 1; j < P2_neigsize; ++j)
		{
			int temp2_index = set->point.at(v2).neighbor[j];
			if (temp1.hasNeighbor(temp2_index) && temp2_index != v1)
			{
				SPoint temp2 = set->point[temp2_index];
				cv::Vec3d Normal1 = (temp1.Point_Vector - temp2.Point_Vector).cross(P2.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal1 = Normal1 / sqrt(Normal1[0] * Normal1[0] + Normal1[1] * Normal1[1] + Normal1[2] * Normal1[2]);

				cv::Vec3d Normal2 = (temp1.Point_Vector - temp2.Point_Vector).cross(bestPoint.Point_Vector - temp1.Point_Vector);
				cv::Vec3d Unit_Normal2 = Normal2 / sqrt(Normal2[0] * Normal2[0] + Normal2[1] * Normal2[1] + Normal2[2] * Normal2[2]);
				double error = 1 - Unit_Normal1.dot(Unit_Normal2);
				if (error > Max_error) {
					Max_error = error;
				}
			}
		}
	}
	return Max_error;
}

bool Pair::JudgeAbnormalCollpase(PointSet * set)
{
	SPoint P1 = set->point.at(v1);
	SPoint P2 = set->point.at(v2);
	cv::Vec3d P = 0.5*(P1.Point_Vector + P2.Point_Vector);
	cv::Vec3d tem = bestPoint.Point_Vector - P;
	double D = sqrt(tem[0] * tem[0] + tem[1] * tem[1] + tem[2] * tem[2]);
	if (D > 6) {
		return true;
	}
	else
	{
		return false;
	}

	
}

bool Pair::ChangeTopology(PointSet * set)
{
	//p1
	bool topologychange;
	SPoint P1 = set->point.at(v1);
	int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	int count = 0;
	for (int i = 0; i < P1_neigsize; ++i)
	{
		SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
		if (temp1.hasNeighbor(v2))
		{
			count++;
		}
	}
	if (count <= 2) {
		topologychange == false;
	}
	else
	{
		topologychange == true;
	}
	return topologychange;
}

bool Pair::calculateVertexSharpness(PointSet * Set, int v1, double &Sharpness)
{

	bool abnormal = false;

	//P1
	int P1_neigsize = (int)Set->point.at(v1).neighbor.size();
	vector<int> neighborP;
	SPoint P1 = Set->point.at(v1);
	
	
	for (int i = 0; i < P1_neigsize; ++i)
	{
		SPoint temp1 = Set->point[Set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j) {
			int temp2_index = Set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))
			{
				neighborP.push_back(Set->point.at(v1).neighbor[i]);
				neighborP.push_back(temp2_index);
			}
		}
	}

	
	if (neighborP.size() != P1_neigsize * 2) {
		abnormal = true;
		Sharpness = 100;
		return abnormal;
	}

	if (neighborP.size() != P1_neigsize * 2) {
		
		if (neighborP.size() != P1_neigsize * 2 + 2) {
			int uuu = 66;
		}
		
		set<int> Outlier;
		for (int i = 0; i < neighborP.size(); i++) {
			int Detection_target = neighborP[i];
			int count = 0;
			for (int j = 0; j < neighborP.size(); j++) {
				if (Detection_target == neighborP[j]) {
					count++;
				}
			}
			if (count == 3) {
				Outlier.insert(Detection_target);
			}
		}

	
		for (int i = 0;i< neighborP.size(); i++) {
			if (neighborP[i] == *Outlier.begin()) {
				if (i % 2 == 0) {
					if (neighborP[i + 1] == *--Outlier.end()) {
						neighborP.erase(neighborP.begin() + i, neighborP.begin() + i + 2);
					}
				}
				else
				{
					if (neighborP[i - 1] == *--Outlier.end()) {
						neighborP.erase(neighborP.begin() + i - 1, neighborP.begin() + i + 1);
					}
				}
			}
		}
	}

	
	vector<int> NewneighborP;
	int startP = 0;
	int endP = 1;
	NewneighborP.push_back(neighborP[0]);
	NewneighborP.push_back(neighborP[1]);
	int newstartP = neighborP[endP];
	for (int j = 0; j < neighborP.size(); j++) {
		if (neighborP[j] == newstartP && (j != endP)) {
			if ((j + 1) % 2 == 0) {
				startP = j;
				endP = j - 1;
			}
			else
			{
				startP = j;
				endP = j + 1;
			}
			NewneighborP.push_back(neighborP[startP]);
			NewneighborP.push_back(neighborP[endP]);
			j = -1;
			newstartP = neighborP[endP];
		}
		if (newstartP == neighborP[0]) {
			break;
		}
	}

	if (NewneighborP.size()!= neighborP.size()) {
		abnormal = true;
		Sharpness = 100;
		return abnormal;
	}

	
	double AreaSum = 0;
	for (int i = 0; i < P1_neigsize; i++) {
		cv::Vec3d p111 = Set->point.at(NewneighborP[2 * i]).Point_Vector;
		cv::Vec3d p222 = Set->point.at(v1).Point_Vector;
		cv::Vec3d p333 = Set->point.at(NewneighborP[2 * i + 1]).Point_Vector;

		double Area = calculateTriangularArea(p111, p222, p333);
		AreaSum = AreaSum + Area;
	}

	
	double angle = 0.0;
	for (int i = 0; i < P1_neigsize; i++) {
		int T1_index = 2 * i;
		cv::Vec3d p1 = Set->point.at(NewneighborP[T1_index]).Point_Vector;//1
		cv::Vec3d p2 = Set->point.at(v1).Point_Vector;//2
		cv::Vec3d p3 = Set->point.at(NewneighborP[T1_index + 1]) .Point_Vector;//3

		cv::Vec3d V1 = p2 - p1;
		cv::Vec3d V2 = p3 - p2;
		
		cv::Vec3d vn1 = V1.cross(V2);
		double L1 = sqrt(vn1.dot(vn1));
		vn1[0] = vn1[0] / L1;
		vn1[1] = vn1[1] / L1;
		vn1[2] = vn1[2] / L1;

		double Area1= calculateTriangularArea(p1, p2, p3);

		for (int j = i + 1; j < P1_neigsize; j++) {
			int T2_index = 2 * j;

			cv::Vec3d p11 = Set->point.at(NewneighborP[T2_index]).Point_Vector;
			cv::Vec3d p22 = Set->point.at(v1).Point_Vector;
			cv::Vec3d p33 = Set->point.at(NewneighborP[T2_index + 1]) .Point_Vector;

			cv::Vec3d V11 = p22 - p11;
			cv::Vec3d V22 = p33 - p22;

			cv::Vec3d vn11 = V11.cross(V22);
			double L11 = sqrt(vn11.dot(vn11));
			vn11[0] = vn11[0] / L11;
			vn11[1] = vn11[1] / L11;
			vn11[2] = vn11[2] / L11;
			
			
			double Area2 = calculateTriangularArea(p11, p22, p33);
			angle = (Area1 + Area2) / (2 * AreaSum)*acos(vn1.dot(vn11)) + angle;

		}
	}
	Sharpness = angle;
	return abnormal;
}

bool Pair::CalculateVertexSharpness(PointSet * Set, int v1, double & Sharpness)
{
	bool abnormal = false;
	//P1
	int P1_neigsize = (int)Set->point.at(v1).neighbor.size();
	vector<int> neighborP;
	SPoint P1 = Set->point.at(v1);

	
	for (int i = 0; i < P1_neigsize; ++i)
	{
		SPoint temp1 = Set->point[Set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j) {
			int temp2_index = Set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))
			{
				neighborP.push_back(Set->point.at(v1).neighbor[i]);
				neighborP.push_back(temp2_index);
			}
		}
	}



	if (neighborP.size() != P1_neigsize * 2) {
		
		if (neighborP.size() != P1_neigsize * 2 + 2) {
			int uuu = 66;
		}
		
		set<int> Outlier;
		for (int i = 0; i < neighborP.size(); i++) {
			int Detection_target = neighborP[i];
			int count = 0;
			for (int j = 0; j < neighborP.size(); j++) {
				if (Detection_target == neighborP[j]) {
					count++;
				}
			}
			if (count == 3) {
				Outlier.insert(Detection_target);
			}
		}

		
		for (int i = 0; i < P1_neigsize; ++i)
		{
			SPoint temp1 = Set->point[Set->point.at(v1).neighbor[i]];
			if (temp1.hasNeighbor(*Outlier.begin()) && temp1.hasNeighbor(*--Outlier.end())) {
				Set->point[Set->point.at(v1).neighbor[i]].removeNeighbor(*Outlier.begin());
				Set->point[Set->point.at(v1).neighbor[i]].removeNeighbor(*--Outlier.end());
				Set->point[Set->point.at(v1).neighbor[i]].removeNeighbor(v1);
				Set->point[*Outlier.begin()].removeNeighbor(Set->point.at(v1).neighbor[i]);
				Set->point[*--Outlier.end()].removeNeighbor(Set->point.at(v1).neighbor[i]);
				Set->point[v1].removeNeighbor(Set->point.at(v1).neighbor[i]);
			}
		}

		vector<int>().swap(neighborP);
		
		P1_neigsize = (int)Set->point.at(v1).neighbor.size();
		for (int i = 0; i < P1_neigsize; ++i)
		{
			SPoint temp11 = Set->point[Set->point.at(v1).neighbor[i]];
			for (int j = i + 1; j < P1_neigsize; ++j) {
				int temp2_index = Set->point.at(v1).neighbor[j];
				if (temp11.hasNeighbor(temp2_index))
				{
					neighborP.push_back(Set->point.at(v1).neighbor[i]);
					neighborP.push_back(temp2_index);
				}
			}
		}
	}

	
	vector<int> NewneighborP;
	int startP = 0;
	int endP = 1;
	NewneighborP.push_back(neighborP[0]);
	NewneighborP.push_back(neighborP[1]);
	int newstartP = neighborP[endP];
	for (int j = 0; j < neighborP.size(); j++) {
		if (neighborP[j] == newstartP && (j != endP)) {
			if ((j + 1) % 2 == 0) {
				startP = j;
				endP = j - 1;
			}
			else
			{
				startP = j;
				endP = j + 1;
			}
			NewneighborP.push_back(neighborP[startP]);
			NewneighborP.push_back(neighborP[endP]);
			j = -1;
			newstartP = neighborP[endP];
		}
		if (newstartP == neighborP[0]) {
			break;
		}
	}

	if (NewneighborP.size() != neighborP.size()) {
		
		abnormal = true;
		Sharpness = 1000000;
		return abnormal;
	}

	
	double AreaSum = 0;
	for (int i = 0; i < P1_neigsize; i++) {
		cv::Vec3d p111 = Set->point.at(NewneighborP[2 * i]).Point_Vector;
		cv::Vec3d p222 = Set->point.at(v1).Point_Vector;
		cv::Vec3d p333 = Set->point.at(NewneighborP[2 * i + 1]).Point_Vector;

		double Area = calculateTriangularArea(p111, p222, p333);
		AreaSum = AreaSum + Area;
	}

	
	double angle = 0.0;
	for (int i = 0; i < P1_neigsize; i++) {
		int T1_index = 2 * i;
		cv::Vec3d p1 = Set->point.at(NewneighborP[T1_index]).Point_Vector;
		cv::Vec3d p2 = Set->point.at(v1).Point_Vector;
		cv::Vec3d p3 = Set->point.at(NewneighborP[T1_index + 1]).Point_Vector;

		cv::Vec3d V1 = p2 - p1;
		cv::Vec3d V2 = p3 - p2;

		cv::Vec3d vn1 = V1.cross(V2);
		double L1 = sqrt(vn1.dot(vn1));
		vn1[0] = vn1[0] / L1;
		vn1[1] = vn1[1] / L1;
		vn1[2] = vn1[2] / L1;

		double Area1 = calculateTriangularArea(p1, p2, p3);

		for (int j = i + 1; j < P1_neigsize; j++) {
			int T2_index = 2 * j;

			cv::Vec3d p11 = Set->point.at(NewneighborP[T2_index]).Point_Vector;//1
			cv::Vec3d p22 = Set->point.at(v1).Point_Vector;//2
			cv::Vec3d p33 = Set->point.at(NewneighborP[T2_index + 1]).Point_Vector;//3

			cv::Vec3d V11 = p22 - p11;
			cv::Vec3d V22 = p33 - p22;

			cv::Vec3d vn11 = V11.cross(V22);
			double L11 = sqrt(vn11.dot(vn11));
			vn11[0] = vn11[0] / L11;
			vn11[1] = vn11[1] / L11;
			vn11[2] = vn11[2] / L11;

			
			double Area2 = calculateTriangularArea(p11, p22, p33);
			angle = (Area1 + Area2) / (2 * AreaSum)*acos(vn1.dot(vn11)) + angle;

		}
	}
	Sharpness = angle;
	return abnormal;
}

double Pair::calculateTriangularArea(cv::Vec3d P1, cv::Vec3d P2, cv::Vec3d P3)
{
	double area = 0.0;
	double dis;
	double side[3];

	side[0] = sqrt(pow(P1[0] - P2[0], 2) + pow(P1[1] - P2[1], 2) + pow(P1[2] - P2[2], 2));
	side[1] = sqrt(pow(P1[0] - P3[0], 2) + pow(P1[1] - P3[1], 2) + pow(P1[2] - P3[2], 2));
	side[2] = sqrt(pow(P3[0] - P2[0], 2) + pow(P3[1] - P2[1], 2) + pow(P3[2] - P2[2], 2));

	double p = (side[0] + side[1] + side[2]) / 2; 
	area = sqrt(p * (p - side[0]) * (p - side[1]) * (p - side[2]));

	return area;
}

bool Pair::calculateSeamError(PointSet * set, int v1, double & SeamError)
{
	double E;
	int mun_uv = set->point.at(v1).SPointUV_index.size();
	E = set->point.at(v1).SeamAngleError(set, mun_uv);
	SeamError = E;
	return true;
}


void Pair::calculateBestUV(PointSet * set)
{
	double MinDistance = 99999999999;
	cv::Vec3d Effective_plane[3];//确定三角面XYZ
	cv::Point2d Effective_plane_UV[3];//确定三角面UV
	cv::Vec3d Effective_point;//确定投影点

	int P1_neigsize = (int)set->point.at(v1).neighbor.size();
	for (int i = 0; i < P1_neigsize; ++i)//遍历P1点的一领域点
	{
		SPoint temp1 = set->point[set->point.at(v1).neighbor[i]];
		for (int j = i + 1; j < P1_neigsize; ++j)
		{
			int temp2_index = set->point.at(v1).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))//判断是否组成一个三角面
			{
				cv::Vec3d Point1 = set->point.at(v1).Point_Vector;//P1
				cv::Vec3d Point2 = temp1.Point_Vector;//P2
				cv::Vec3d Point3 = set->point.at(temp2_index).Point_Vector;//P3
				cv::Vec3d plane[3] = { Point1 ,Point2 ,Point3 };//三角面
				
				cv::Vec3d ProjectionPoint = calculateProjectionPoint(plane, bestPoint.Point_Vector);//计算投影点
				if (JudgeInclusion(plane, ProjectionPoint)) {//判断投影点是否有效
					cv::Vec3d tem = ProjectionPoint - bestPoint.Point_Vector;
					double Distance = sqrt(tem.dot(tem));//计算距离
					if (Distance < MinDistance) {
						MinDistance = Distance;
						Effective_point = ProjectionPoint;
						Effective_plane[0] = Point1;
						Effective_plane[1] = Point2;
						Effective_plane[2] = Point3;
						Effective_plane_UV[0] = set->point.at(v1).PointUV;
						Effective_plane_UV[1] = temp1.PointUV;
						Effective_plane_UV[2] = set->point.at(temp2_index).PointUV;
					}
				}
			}
		}
	}

	int P2_neigsize = (int)set->point.at(v2).neighbor.size();
	for (int i = 0; i < P2_neigsize; ++i)//遍历P1点的一领域点
	{
		SPoint temp1 = set->point[set->point.at(v2).neighbor[i]];
		for (int j = i + 1; j < P2_neigsize; ++j)
		{
			int temp2_index = set->point.at(v2).neighbor[j];
			if (temp1.hasNeighbor(temp2_index))//判断是否组成一个三角面
			{
				cv::Vec3d Point1 = set->point.at(v2).Point_Vector;//P1
				cv::Vec3d Point2 = temp1.Point_Vector;//P2
				cv::Vec3d Point3 = set->point.at(temp2_index).Point_Vector;//P3
				cv::Vec3d plane[3] = { Point1 ,Point2 ,Point3 };//三角面

				cv::Vec3d ProjectionPoint = calculateProjectionPoint(plane, bestPoint.Point_Vector);//计算投影点
				if (JudgeInclusion(plane, ProjectionPoint)) {//判断投影点是否有效
					cv::Vec3d tem = ProjectionPoint - bestPoint.Point_Vector;
					double Distance = sqrt(tem.dot(tem));//计算距离
					if (Distance < MinDistance) {
						MinDistance = Distance;
						Effective_point = ProjectionPoint;
						Effective_plane[0] = Point1;
						Effective_plane[1] = Point2;
						Effective_plane[2] = Point3;
						Effective_plane_UV[0] = set->point.at(v1).PointUV;
						Effective_plane_UV[1] = temp1.PointUV;
						Effective_plane_UV[2] = set->point.at(temp2_index).PointUV;
					}
				}
			}
		}
	}
	//计算最佳UV
	if (Effective_point[0] == 0) {
		
	}
	else
	{
		osg::Matrix R, R_i;
		computeRotationMatrix(Effective_plane, R, R_i);//计算旋转矩阵及逆矩阵
		osg::Vec3d point1, point2, point3, P;
		point1 = getXYZ(Effective_plane[0], R);//旋转
		point2 = getXYZ(Effective_plane[1], R);
		point3 = getXYZ(Effective_plane[2], R);
		P = getXYZ(Effective_point, R);
		osg::Vec3d plane1[3] = { point1 ,point2 ,point3 };
		//求仿射变换矩阵,及纹理坐标
		affineTransformMatrix(plane1, Effective_plane_UV, P, bestPoint.PointUV);
	}
	
}

cv::Vec3d Pair::calculateProjectionPoint(cv::Vec3d plane[3], cv::Vec3d P)
{
	cv::Vec3d AB = plane[0] - plane[1];
	cv::Vec3d AC = plane[0] - plane[2];
	cv::Vec3d Normal = AB.cross(AC);

	cv::Vec3d ProjectionPoint;
	ProjectionPoint[0] = (Normal[0] * Normal[1] * plane[0][1] + Normal[1] * Normal[1] * P[0] - Normal[0] * Normal[1] * P[1] + Normal[0] * Normal[2] * plane[0][2] + Normal[2] * Normal[2] * P[0] - Normal[0] * Normal[2] * P[2] + Normal[0] * Normal[0] * plane[0][0]) / Normal.dot(Normal);
	ProjectionPoint[1] = (Normal[1] * Normal[2] * plane[0][2] + Normal[2] * Normal[2] * P[1] - Normal[1] * Normal[2] * P[2] + Normal[1] * Normal[0] * plane[0][0] + Normal[0] * Normal[0] * P[1] - Normal[0] * Normal[1] * P[0] + Normal[1] * Normal[1] * plane[0][1]) / Normal.dot(Normal);
	ProjectionPoint[2] = (Normal[0] * plane[0][0] * Normal[2] + Normal[0] * Normal[0] * P[2] - Normal[0] * P[0] * Normal[2] + Normal[1] * plane[0][1] * Normal[2] + Normal[1] * Normal[1] * P[2] - Normal[1] * P[1] * Normal[2] + Normal[2] * Normal[2] * plane[0][2]) / Normal.dot(Normal);
	return ProjectionPoint;
}

bool Pair::JudgeInclusion(cv::Vec3d plane[3], cv::Vec3d P)
{
	bool toLeft = true;
	for (int i = 0; i < 3; i++) {
		cv::Vec3d Vec = plane[(i + 1) % 3] - plane[i];
		cv::Vec3d Vec1 = P - plane[i];
		if (i != 0) {
			if (toLeft != Vec[0] * Vec1[1] - Vec[1] * Vec1[0] > 0) {
				return false;
			}
		}
		toLeft = Vec[0] * Vec1[1] - Vec[1] * Vec1[0] > 0;
	}
	return true;
}



void Pair::calculateDelCostD_QEM(PointSet * Set, int type, double V_sharpness, double S_angleerror, double T_complexity)
{
	switch (type)
	{
	case 1:
		calculateDelCostDQEMUV(Set);
	
		delCost = delCost * pow(3, V_sharpness) * T_complexity * S_angleerror;
		break;
	case 2:
	{
		int p_v1uv = *Set->point.at(strat_Point).SPointUV_index.begin();
		int uvsize = Set->point.at(end_Point).SPointUV_index.size();

		for (auto uv : Set->point.at(end_Point).SPointUV_index) {
			if (Set->point_uv.at(uv).hasNeighbor(p_v1uv)) {
			
				set<int>::iterator pos1 = find(Set->point.at(end_Point).SPointUV_index.begin(), Set->point.at(end_Point).SPointUV_index.end(), uv);
				int dex = distance(Set->point.at(end_Point).SPointUV_index.begin(), pos1);
				cv::Mat mat1 = Set->point.at(strat_Point).Texture_error;
				cv::Mat mat2 = Set->point.at(end_Point).Texture_errors[dex];
				
				calculateDelCostUV_Boundary(Set, mat1, mat2); 
				
				delCost = delCost * pow(3, V_sharpness) * T_complexity * S_angleerror;
				break;
			}
		}
	}
	break;
	case 3:
	{   double delCost_v1 = 0;
	    double delCost_v2 = 0;
	
		for (auto uv1 : Set->point.at(v1).SPointUV_index) {
			int UV1, UV2;
			set<int>::iterator pos2;
			set<int>::iterator pos3;
			bool special = true;
			pos2 = find(Set->point.at(v1).SPointUV_index.begin(), Set->point.at(v1).SPointUV_index.end(), uv1);
			for (auto uv2 : Set->point.at(v2).SPointUV_index) {
				pos3 = find(Set->point.at(v2).SPointUV_index.begin(), Set->point.at(v2).SPointUV_index.end(), uv2);
				if (Set->point_uv.at(uv1).hasNeighbor(uv2)) {
					UV1 = uv1;
					UV2 = uv2;
					special = false;
					break;
				}
			}
			if (special == true) {
				delCost = 1000000000000;
				return;
			}
			else
			{
				int dex1 = distance(Set->point.at(v1).SPointUV_index.begin(), pos2);
				int dex2 = distance(Set->point.at(v2).SPointUV_index.begin(), pos3);
				cv::Mat mat1 = Set->point.at(v1).Texture_errors[dex1];
				cv::Mat mat2 = Set->point.at(v2).Texture_errors[dex2];

				delCost_v1 += calculateDelCostUV_Boundary1(Set, v2, UV2, mat1, mat2);
				delCost_v2 += calculateDelCostUV_Boundary1(Set, v1, UV1, mat1, mat2);
			}
		}

		if (delCost_v1 < delCost_v2) {
			
			strat_Point = v1;
			end_Point = v2;
		}
		else
		{
		
			strat_Point = v2;
			end_Point = v1;
		}

		
		bestPoint.Point_Vector = Set->point.at(end_Point).Point_Vector;
		int a = 0;
		for (auto uv3 : Set->point.at(end_Point).SPointUV_index) {
			a++;
			if (a == 1) {
				bestPoint.PointUV_Vector = Set->point_uv.at(uv3).PointUV_Vector;
			}
			else
			{
				bestPoint.PointUV_Vector2 = Set->point_uv.at(uv3).PointUV_Vector;
			}
		}

		
		delCost = (delCost_v1 < delCost_v2) ? delCost_v1 : delCost_v2 ;
		delCost = delCost * pow(3, V_sharpness) * T_complexity * S_angleerror;
	
	}
	break;
	case 4:
		delCost = 1000000000000000000;
		break;
	}
}


double Pair::calcuateAngleError(PointSet * set, int strat_Point, int end_Point)
{
	int strat_neightorSize = set->point.at(strat_Point).neighbor.size();
	int tem_texSize = 1;
	bool tem = false;
	SPoint P;
	for (int i = 0; i < strat_neightorSize; i++) {
		if (set->point.at(set->point.at(strat_Point).neighbor[i]).SPointUV_index.size() > tem_texSize) {
			tem_texSize = set->point.at(set->point.at(strat_Point).neighbor[i]).SPointUV_index.size();
			P = set->point.at(set->point.at(strat_Point).neighbor[i]);
			tem = true;
		}
	}

	cv::Vec3d v0, v1, v2;
	v0 = set->point.at(strat_Point).Point_Vector;
	v1 = set->point.at(end_Point).Point_Vector;
	v2 = P.Point_Vector;

	
	if (tem == true) {
		E = (v0 - v1).dot(v2 - v1) / sqrt((v0 - v1).dot(v0 - v1))* sqrt((v2 - v0).dot(v2 - v0));
	}
	else
	{
		E = 0.0;
	}
	return E;
}


