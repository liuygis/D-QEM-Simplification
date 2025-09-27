#include "Simplification.h"
#include <fstream>
#include <iostream>
#include "Point.h"
#include <set>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include "Face.h"
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <osgDB/WriteFile>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include "MyReader.h"
#include <QProgressDialog>
#include <osg/CullFace>
#include <string>
#include <osgUtil/SmoothingVisitor>

#include"OBBbox.h"
#include"MyViewer.h"
#include"MyMainWidget.h"
#include"DOMLod.h"
#include <osg/LOD>

#include "modelError.h"
#include"CU.h"
#include "MovingLeastSquares.h"
#include"OptimizeUV.h"

using namespace std;



bool Simplification::Group_readFile(int stratLine, int &endLine)
{
	faceCount = 0;
	updatepointSet();
	updatepairHeap();

	int line = stratLine;
	ifstream file;
	file.open(inputFile);
	if (!file) {
		return false;
	}
		
	seek_to_line(file, line);

	string data;
	bool f_Open=false;
	
	while (getline(file, data))
	{
		line++;
		
		QString LineData = QString::fromLocal8Bit(data.data());
		QStringList coordlist = LineData.split(" ");
		string flag = coordlist[0].toStdString();

		if (flag == "v") {
			double x = coordlist[1].toDouble();
			double y = coordlist[2].toDouble();
			double z = coordlist[3].toDouble();
			SPoint newPoint;
			newPoint.Point_Vector[0] = x;
			newPoint.Point_Vector[1] = y;
			newPoint.Point_Vector[2] = z;
			pointSet.addPoint(newPoint);

		}
		else if (flag == "vt")
		{
			double u = coordlist[1].toDouble();
			double v = coordlist[2].toDouble();

			SPoint_UV newPoint_UV;
			newPoint_UV.PointUV_Vector[0] = u;
			newPoint_UV.PointUV_Vector[1] = v;

			pointSet.addPoint_uv(newPoint_UV);
		}
		else if (flag == "f")
		{
			f_Open = true;

			int index[3];
			int UV_index[3];
			for (int i = 0; i < 3; i++) {
				QStringList list = coordlist[i + 1] .split("/");
				index[i] = list[0].toInt() - 1 - pointCount;
				UV_index[i] = list[1].toInt() - 1 - pointUVCount;

				pointSet.point.at(index[i]).SPointUV_index.insert(UV_index[i]);
			}

			
			for (int j = 0; j < 3; ++j) {
				for (int k = j + 1; k < 3; ++k) {
					
					if (!pointSet.point.at(index[j]).hasNeighbor(index[k]))
					{
						pointSet.point.at(index[j]).addNeighbor(index[k]);
						pointSet.point.at(index[k]).addNeighbor(index[j]);
					}

					if (!pointSet.point_uv.at(UV_index[j]).hasNeighbor(UV_index[k]))
					{
						pointSet.point_uv.at(UV_index[j]).addNeighbor(UV_index[k]);
						pointSet.point_uv.at(UV_index[k]).addNeighbor(UV_index[j]);
					}
				}
			}
			faceCount++;

		}
		else if (flag == "mtllib")
		{
			inputMTL = inputFile.substr(0, inputFile.find_last_of('/') + 1) + coordlist[1].toStdString();//获取mtl文件绝对路径
		}
		else if (flag == "usemtl")
		{
			string newmtl = "newmtl " + coordlist[1].toStdString();
			ifstream mtlfile;
			mtlfile.open(inputMTL);
			if (!mtlfile)
				return false;

			bool sign = false;
			string rowdata;
			while (getline(mtlfile, rowdata)) {
				if (rowdata == newmtl)
					sign = true;
				QString Rowdata = QString::fromLocal8Bit(rowdata.data());
				QStringList Rowdatalist = Rowdata.split(" ");
				string mapKdflag = Rowdatalist[0].toStdString();

				if (sign == true && mapKdflag == "map_Kd") {
					texture_filename = Rowdatalist[1].toStdString();
					texture_file= inputFile.substr(0, inputFile.find_last_of('/') + 1) + Rowdatalist[1].toStdString();
					break;
				}
			}
		}
		
		if (f_Open == true && flag != "f") {
			pointCount = pointSet.point.size() + pointCount;
			pointUVCount = pointSet.point_uv.size() + pointUVCount;
			endLine = line - 1;
			file.close();
			return true;
		}
	}
	endLine = line;
	file.close();
	return true;
}

void Simplification::updateDataUVPoint_D_QEM(Pair &toDel)
{
	int v1_num_uv = pointSet.point.at(toDel.v1).SPointUV_index.size();
	int v2_num_uv = pointSet.point.at(toDel.v2).SPointUV_index.size();
    
	set<int> UVPoint_v1v2;
	for (set<int>::iterator it = pointSet.point.at(toDel.v1).SPointUV_index.begin(); it != pointSet.point.at(toDel.v1).SPointUV_index.end(); it++)
	{
		cout << *it << " occurs " << endl;
		UVPoint_v1v2.insert(*it);
	}
	for (set<int>::iterator it = pointSet.point.at(toDel.v2).SPointUV_index.begin(); it != pointSet.point.at(toDel.v2).SPointUV_index.end(); it++)
	{
		UVPoint_v1v2.insert(*it);
	}
	set<int> neighbors;
	for (set<int>::iterator it = UVPoint_v1v2.begin(); it != UVPoint_v1v2.end(); it++)
	{
		cout << *it << " occurs " << endl;
		for (int i = 0; i < (int)pointSet.point_uv.at(*it).neighbor.size(); ++i)
		{
			int k = pointSet.point_uv.at(*it).neighbor[i];
			if (UVPoint_v1v2.count(k) == 0)
				neighbors.insert(k);
		}
	}

	SPoint_UV newPoint_UV;
	newPoint_UV.PointUV_Vector[0] = toDel.bestPoint.PointUV_Vector[0];
	newPoint_UV.PointUV_Vector[1] = toDel.bestPoint.PointUV_Vector[1];
	int np = pointSet.addPoint_uv(newPoint_UV);

	switch (toDel.Edge_type)
	{
	case 0:
		break;
	case 1:
	{
		
		set<int>::iterator nb;
		for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
		{
			pointSet.point_uv.at(np).neighbor.push_back(*nb);
		}

		
		for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
		{
			vector<int>::iterator iter;
			for (iter = pointSet.point_uv.at(*nb).neighbor.begin(); iter != pointSet.point_uv.at(*nb).neighbor.end();)
			{
				if (UVPoint_v1v2.count(*iter) == 1) {
					iter = pointSet.point_uv.at(*nb).neighbor.erase(iter);
				}
				else
				{
					++iter;
				}
			}
			pointSet.point_uv.at(*nb).addNeighbor(np);
		}
	}
		break;
	case 2:
	{
		
		int enduv;
		int p_startuv = *pointSet.point.at(toDel.strat_Point).SPointUV_index.begin();
		for (auto uv : pointSet.point.at(toDel.end_Point).SPointUV_index) {
			if (pointSet.point_uv.at(uv).hasNeighbor(p_startuv) == false) {
				enduv = uv;
			}
		}
		toDel.bestPoint.SPointUV_index.insert(enduv);
		UVPoint_v1v2.erase(enduv);
		set<int>::iterator nb;
		for (nb = neighbors.begin(); nb != neighbors.end(); )
		{
			if (pointSet.point_uv.at(*nb).hasNeighbor(enduv)) {
				nb = neighbors.erase(nb);
			}
			else
			{
				++nb;
			}
		}

		
		for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
		{
			pointSet.point_uv.at(np).neighbor.push_back(*nb);
		}

		
		for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
		{
			vector<int>::iterator iter;
			for (iter = pointSet.point_uv.at(*nb).neighbor.begin(); iter != pointSet.point_uv.at(*nb).neighbor.end();)
			{
				if (UVPoint_v1v2.count(*iter) == 1) {
					iter = pointSet.point_uv.at(*nb).neighbor.erase(iter);
				}
				else
				{
					++iter;
				}
			}
			pointSet.point_uv.at(*nb).addNeighbor(np);
		}
	}
		break;
	case 3:
	{
		SPoint_UV newPoint_UV2;
		newPoint_UV2.PointUV_Vector[0] = toDel.bestPoint.PointUV_Vector2[0];
		newPoint_UV2.PointUV_Vector[1] = toDel.bestPoint.PointUV_Vector2[1];
		int np2 = pointSet.addPoint_uv(newPoint_UV2);

		
		set<int> UVPoint_v1v2_1;
		set<int> UVPoint_v1v2_2;
		int index = 0;
		for (auto uv1 : pointSet.point.at(toDel.end_Point).SPointUV_index) {
			index++;
			for (auto uv2 : UVPoint_v1v2) {
				if (pointSet.point_uv.at(uv1).hasNeighbor(uv2) == true) {
					switch (index)
					{
					case 1:
						UVPoint_v1v2_1.insert(uv1);
						UVPoint_v1v2_1.insert(uv2);
						break;
					case 2:
						UVPoint_v1v2_2.insert(uv1);
						UVPoint_v1v2_2.insert(uv2);
						break;
					}
					break;
				}
			}
		}

		
		set<int> neighbors_1;
		set<int> neighbors_2;
		for (auto uv : UVPoint_v1v2_1) {
			for (auto uv1 : neighbors) {
				if (pointSet.point_uv.at(uv).hasNeighbor(uv1) == true) {
					neighbors_1.insert(uv1);
			}
		}
		for (auto uv : UVPoint_v1v2_2) {
			for (auto uv1 : neighbors) {
				if (pointSet.point_uv.at(uv).hasNeighbor(uv1) == true) {
					neighbors_2.insert(uv1);
				}
			}
		}

		
		set<int>::iterator nb;
		for (nb = neighbors_1.begin(); nb != neighbors_1.end(); ++nb)
		{
			pointSet.point_uv.at(np).neighbor.push_back(*nb);
		}
		
		for (nb = neighbors_2.begin(); nb != neighbors_2.end(); ++nb)
		{
			pointSet.point_uv.at(np2).neighbor.push_back(*nb);
		}

		
		for (nb = neighbors_1.begin(); nb != neighbors_1.end(); ++nb)
		{
			vector<int>::iterator iter;
			for (iter = pointSet.point_uv.at(*nb).neighbor.begin(); iter != pointSet.point_uv.at(*nb).neighbor.end();)
			{
				if (UVPoint_v1v2_1.count(*iter) == 1) {
					iter = pointSet.point_uv.at(*nb).neighbor.erase(iter);
				}
				else
				{
					++iter;
				}
			}
			pointSet.point_uv.at(*nb).addNeighbor(np);
		}
	
		for (nb = neighbors_2.begin(); nb != neighbors_2.end(); ++nb)
		{
			vector<int>::iterator iter;
			for (iter = pointSet.point_uv.at(*nb).neighbor.begin(); iter != pointSet.point_uv.at(*nb).neighbor.end();)
			{
				if (UVPoint_v1v2_2.count(*iter) == 1) {
					iter = pointSet.point_uv.at(*nb).neighbor.erase(iter);
				}
				else
				{
					++iter;
				}
			}
			pointSet.point_uv.at(*nb).addNeighbor(np2);
		}
	}
		break;
	default:
		break;
	}

}



void Simplification::updateData_Point_D_QEM(Pair &toDel)
{
	
	pairHeap.deletePair(&toDel);

	
	switch (toDel.Edge_type)
	{
	case 1:
	case 2:
		toDel.bestPoint.SPointUV_index.insert(pointSet.maxpos_uv - 1);
		break;
	case 3:
		toDel.bestPoint.SPointUV_index.insert(pointSet.maxpos_uv - 2);
		toDel.bestPoint.SPointUV_index.insert(pointSet.maxpos_uv - 1);
		break;
	default:
		break;
	}
	
	
	int np = pointSet.addPoint(toDel.bestPoint);

	
	set<int> neighbors;
	for (int i = 0; i < (int)pointSet.point.at(toDel.v1).neighbor.size(); ++i)
	{
		int k = pointSet.point.at(toDel.v1).neighbor[i];
		if (k != toDel.v2)
			neighbors.insert(k);
	}
	for (int i = 0; i < (int)pointSet.point.at(toDel.v2).neighbor.size(); ++i)
	{
		int k = pointSet.point.at(toDel.v2).neighbor[i];
		if (k != toDel.v1)
			neighbors.insert(k);
	}

	
	set<int>::iterator nb;
	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		pointSet.point.at(np).neighbor.push_back(*nb);
	}


	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		vector<int>::iterator iter;
		for (iter = pointSet.point.at(*nb).neighbor.begin(); iter != pointSet.point.at(*nb).neighbor.end();)
		{
			if (*iter == toDel.v1 || *iter == toDel.v2)
				iter = pointSet.point.at(*nb).neighbor.erase(iter);
			else
				++iter;
		}
		pointSet.point.at(*nb).addNeighbor(np);
	}

	
	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb) {
		pointSet.point.at(*nb).calculateD_QEMMat(&pointSet);
	}
	
    pointSet.point.at(np).calculateD_QEMMat(&pointSet);

	
	for (nb = neighbors.begin(); nb != neighbors.end(); ++nb)
	{
		
		Pair p1 = Pair(*nb, toDel.v1);
		Pair p2 = Pair(*nb, toDel.v2);
		p1.sort();
		p2.sort();
		map<pair<int, int>, bool>::iterator iter;
		iter = pairHeap.mapper.find(make_pair(p1.v1, p1.v2));
		if (iter != pairHeap.mapper.end())
		{
			if (pairHeap.mapper[make_pair(p1.v1, p1.v2)] == true)
				pairHeap.deletePair(&Pair(p1.v1, p1.v2));
		}
		iter = pairHeap.mapper.find(make_pair(p2.v1, p2.v2));
		if (iter != pairHeap.mapper.end())
		{
			if (pairHeap.mapper[make_pair(p2.v1, p2.v2)] == true)
				pairHeap.deletePair(&Pair(p2.v1, p2.v2));
		}

		
		Pair toAdd = Pair(*nb, np);
		toAdd.sort();

	
		double Sharpness1 = 0;
		double Sharpness2 = 0;
		toAdd.calculateVertexSharpness(&pointSet, toAdd.v1, Sharpness1);
		toAdd.calculateVertexSharpness(&pointSet, toAdd.v2, Sharpness2);
		double Vertex_sharpness = Sharpness1 + Sharpness2;
		

		int T_edge = toAdd.JudgeStrategy(&pointSet);
		bool Success;
		toAdd.calculateBestPointD_QEM(&pointSet, Success, T_edge);
		
		toAdd.calculateDelCostD_QEM(&pointSet, T_edge, Vertex_sharpness, Seam_angleerror, Texture_complexity);
		
		if (toAdd.calculateMaxNormalDeviation(&pointSet) >= 1) {
			double ppp = toAdd.delCost;
			toAdd.delCost = toAdd.delCost + 1000000000000000000;
		}
		pairHeap.addPair(&toAdd);
	}

	pointSet.delPoint(toDel.v1);
	pointSet.point.at(toDel.v1).neighbor.clear();
	pointSet.delPoint(toDel.v2);
    pointSet.point.at(toDel.v2).neighbor.clear();
}



osg::Geode * Simplification::UVQEMCreateLeaf()
{
	osg::ref_ptr<osg::Geode> leaf(new osg::Geode);
	
	deprecated_osg::Geometry* geometry = new deprecated_osg::Geometry;
	osg::Vec3Array * vertexArray = new osg::Vec3Array;     
	osg::UByteArray* vecindex = new osg::UByteArray;        
	osg::Vec2Array * textureCoord = new osg::Vec2Array;      
	
	osg::UByteArray* texindex = new osg::UByteArray;                 
	osg::ref_ptr < osg::Vec3Array >normals = new osg::Vec3Array;     
	osg::UByteArray* norindex = new osg::UByteArray;                

	vector<int> pointOut;
	map<int, int> outMapper;
	int pos = 0;
	for (int outCount = 0; outCount < pointSet.count;)
	{
		if (pointSet.enabled[pos])
		{
			pointOut.push_back(pos);
			outMapper[pos] = outCount;
			outCount++;
		}
		pos++;
	}

	struct cmp
	{
		bool operator ()(Face f1, Face f2)
		{
			if (f1.p[0] < f2.p[0]) return true;
			if (f1.p[0] > f2.p[0]) return false;
			if (f1.p[1] < f2.p[1]) return true;
			if (f1.p[1] > f2.p[1]) return false;
			if (f1.p[2] < f2.p[2]) return true;
			return false;
		}
	};

	set<Face, cmp> faceOut;
	for (int i = 0; i < (int)pointOut.size(); i++)
	{
		int insize = (int)pointSet.point.at(pointOut[i]).neighbor.size();
		for (int v1 = 0; v1 < insize; v1++)
		{
			for (int v2 = v1 + 1; v2 < insize; v2++)
			{
				if (pointSet.point.at(pointSet.point.at(pointOut[i]).neighbor[v1]).hasNeighbor(
					pointSet.point.at(pointOut[i]).neighbor[v2]))
				{
					SPoint p = pointSet.point.at(pointOut[i]);
					
					Face toAdd(outMapper[pointOut[i]], outMapper[p.neighbor[v1]], outMapper[p.neighbor[v2]]);
					toAdd.pointSort();
					faceOut.insert(toAdd);
				}
			}
		}
	}

	set<Face, cmp>::iterator iter;
	
	double y;
	double z;
	double u;
	double v;


	for (iter = faceOut.begin(); iter != faceOut.end(); ++iter)
	{
		int one = (*iter).p[0];
		int two = (*iter).p[1];
		int three = (*iter).p[2];

		double v1[3];//1
		double v2[3];//2
		double v3[3];//3

		x = pointSet.point.at(pointOut[one]).Point_Vector[0];
		y = pointSet.point.at(pointOut[one]).Point_Vector[1];
		z = pointSet.point.at(pointOut[one]).Point_Vector[2];
		v1[0] = x;
		v1[1] = y;
		v1[2] = z;
		vertexArray->push_back(osg::Vec3d(x, y, z));

		x = pointSet.point.at(pointOut[two]).Point_Vector[0];
		y = pointSet.point.at(pointOut[two]).Point_Vector[1];
		z = pointSet.point.at(pointOut[two]).Point_Vector[2];
		v2[0] = x;
		v2[1] = y;
		v2[2] = z;
		vertexArray->push_back(osg::Vec3d(x, y, z));

		x = pointSet.point.at(pointOut[three]).Point_Vector[0];
		y = pointSet.point.at(pointOut[three]).Point_Vector[1];
		z = pointSet.point.at(pointOut[three]).Point_Vector[2];
		v3[0] = x;
		v3[1] = y;
		v3[2] = z;
		vertexArray->push_back(osg::Vec3d(x, y, z));
		
		float normal[3];
		double dnormal[3];
		computeNormal(v1, v2, v3, &dnormal[0]);
		normal[0] = dnormal[0];
		normal[1] = dnormal[1];
		normal[2] = dnormal[2];
		normals->push_back(osg::Vec3(normal[0], normal[1], normal[2]));
		normals->push_back(osg::Vec3(normal[0], normal[1], normal[2]));
		normals->push_back(osg::Vec3(normal[0], normal[1], normal[2]));


		bool Effective = false;
		for (set<int>::iterator it1 = pointSet.point.at(pointOut[one]).SPointUV_index.begin(); it1 != pointSet.point.at(pointOut[one]).SPointUV_index.end(); it1++)
		{
			if (Effective == false) {
				for (set<int>::iterator it2 = pointSet.point.at(pointOut[two]).SPointUV_index.begin(); it2 != pointSet.point.at(pointOut[two]).SPointUV_index.end(); it2++)
				{
					if (Effective == false) {
						for (set<int>::iterator it3 = pointSet.point.at(pointOut[three]).SPointUV_index.begin(); it3 != pointSet.point.at(pointOut[three]).SPointUV_index.end(); it3++)
						{
							bool a = pointSet.point_uv.at(*it1).hasNeighbor(*it2);
							bool a1 = pointSet.point_uv.at(*it2).hasNeighbor(*it1);
							bool b = pointSet.point_uv.at(*it2).hasNeighbor(*it3);
							bool b1 = pointSet.point_uv.at(*it3).hasNeighbor(*it2);
							bool c = pointSet.point_uv.at(*it1).hasNeighbor(*it3);
							bool c1 = pointSet.point_uv.at(*it3).hasNeighbor(*it1);
							if (a == true && a1 == true && b == true && b1 == true && c == true && c1 == true) {
								u = pointSet.point_uv.at(*it1).PointUV_Vector[0];
								v = pointSet.point_uv.at(*it1).PointUV_Vector[1];
								textureCoord->push_back(osg::Vec2d(u, v));

								u = pointSet.point_uv.at(*it2).PointUV_Vector[0];
								v = pointSet.point_uv.at(*it2).PointUV_Vector[1];
								textureCoord->push_back(osg::Vec2d(u, v));

								u = pointSet.point_uv.at(*it3).PointUV_Vector[0];
								v = pointSet.point_uv.at(*it3).PointUV_Vector[1];
								textureCoord->push_back(osg::Vec2d(u, v));
								Effective = true;
								break;
							}
						}
					}
				}
			}
		}
		if (Effective == false) {

		vertexArray->pop_back();
		vertexArray->pop_back();
		vertexArray->pop_back();
		normals->pop_back();
		normals->pop_back();
		normals->pop_back();
		}
	}

	geometry->setVertexArray(vertexArray);
	normals.get()->setBinding(osg::Array::BIND_PER_PRIMITIVE_SET);
	geometry->setNormalArray(normals.get());
	geometry->setTexCoordArray(0, textureCoord);
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, vertexArray->size()));
	leaf->addDrawable(geometry);
	return leaf.release();
}


bool Simplification::updatepointSet()
{
	pointSet.updata();
	return true;
}

bool Simplification::updatepairHeap()
{
	pairHeap.updata();
	return true;
}

Simplification::Simplification()
{
	inputFile = "";
	inputMTL = "";
	//outputFile = "";
	ratio = 0.5;
	pointCount = 0;
	pointUVCount = 0;
	faceCount = 0;

}


void Simplification::setInput(string _input)
{
	inputFile = _input;
}


void Simplification::setOutput(string _output)
{
	outputFile = _output;
}

void Simplification::setRatio(double _ratio)
{
	ratio = _ratio;
}

void Simplification::exacute()
{
	D_QEM_Simplify();
}

//Simplify
void Simplification::D_QEM_Simplify()
{
	CMyReader reader;
	gocPolLayerNode * layerNode = new gocPolLayerNode;

	int stratLine = 0;
	int endLine = 0;
	int num = 1;
	while (Group_readFile(stratLine, endLine))//Read file data
	{
		if (endLine == stratLine) {
			break;
		}
		
		num++;
		
		pointSet.calculateD_QEMUVMat();
		pairHeap.setupHeapD_QEM(&pointSet);
		
		int nowCount = faceCount;
		int EndFaceCount = faceCount * (1 - ratio);
		while (nowCount > EndFaceCount)
		{
			Pair toDel = pairHeap.top(&pointSet);
			if (toDel.delCost >= 1000000000000) {
				break;
			}
			if (toDel.v1 == -1 || toDel.v2 == -1) {
				break;
			}
			updateDataUVPoint_D_QEM(toDel);
			updateData_Point_D_QEM(toDel);
			nowCount -= 2;
		}

		osg::Geode* leaf = UVQEMCreateLeaf();
		osg::ref_ptr<osg::Texture2D> texture;
		osg::ref_ptr<osg::Image> image;
		texture = new osg::Texture2D;
		image = osgDB::readImageFile(texture_file);

		texture->setResizeNonPowerOfTwoHint(false);
		texture->setUnRefImageDataAfterApply(true);
		texture->setNumMipmapLevels(3);
		texture->setImage(image.get());
		texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
		if (texture->getImage() != 0L) {
			leaf->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
			leaf->getOrCreateStateSet()->setTextureAttribute(0, new osg::TexEnv(osg::TexEnv::DECAL));
		}

		layerNode->addChild(leaf);
		stratLine = endLine;
	}

	gocLayerNode * node = layerNode;
	osgDB::writeNodeFile(*node, outputFile);
}


bool Simplification::split(char * strLine, char **& res)
{
	char *p = NULL;
	int count = 0;
	p = strtok(strLine, " ");

	while (p)
	{
		res[count] = p;
		p = strtok(NULL, " ");
		count++;
	}
	return true;
}

bool Simplification::split1(char * strLine, char **& res)
{
	char *p = NULL;
	int count = 0;
	p = strtok(strLine, "/");

	while (p)
	{
		res[count] = p;
		p = strtok(NULL, "/");
		count++;
	}
	return true;
}

ifstream & Simplification::seek_to_line(ifstream & in, int line)
{

	int i;
	char buf[1024];
	in.seekg(0, ios::beg);  //定位到文件开始。
	for (i = 0; i < line; i++)
	{
		in.getline(buf, sizeof(buf));//读取行。
	}
	return in;

}

void Simplification::computeNormal(double v1[3], double v2[3], double v3[3], double n[3])
{
	double length;
	n[0] = n[2] = 0; n[1] = 1.0;
	computeNormalDirection(v1, v2, v3, n);
	length = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	if (length > 0.0) {
		n[0] /= length;
		n[1] /= length;
		n[2] /= length;
	}
}

void Simplification::computeNormalDirection(double v1[3], double v2[3], double v3[3], double n[3])
{
	double ax, ay, az, bx, by, bz;
	// order is important!!! maintain consistency with triangle vertex order
	ax = (double)v3[0] - v2[0];
	ay = (double)v3[1] - v2[1];
	az = (double)v3[2] - v2[2];
	bx = (double)v1[0] - v2[0];
	by = (double)v1[1] - v2[1];
	bz = (double)v1[2] - v2[2];
	n[0] = (double)(ay * bz - az * by);
	n[1] = (double)(az * bx - ax * bz);
	n[2] = (double)(ax * by - ay * bx);
}

bool Simplification::judgeTirSame(QList<int>& Tir_index, int Tir)
{
	for (int m = 0; m != Tir_index.size(); ++m)
	{
		int Tirs = Tir_index.at(m);
		if (Tirs == Tir)
		{
			return true;
		}
	}
	return false;
}

