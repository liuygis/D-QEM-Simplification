#include "Point.h"
#include <iostream>

using namespace std;
SPoint::SPoint()
{
	Point_Vector = cv::Vec3d(0, 0, 0);
	Point_rgb = cv::Vec3d(0, 0, 0);
	PointUV = cv::Point2d(0, 0);
	PointUV_Vector = cv::Vec2d(0, 0);
	error = cv::Mat::zeros(4, 4, CV_64F);
	Texture_error = cv::Mat::zeros(6, 6, CV_64F);
	point_changeable = true;
	
}


bool SPoint::hasNeighbor(int neiId)
{
	vector<int>::iterator iter;
	for (iter = neighbor.begin(); iter != neighbor.end(); ++iter)
		if (*iter == neiId)
			return true;
	return false;
}

void SPoint::addNeighbor(int neiId)
{
	if (!hasNeighbor(neiId))
		neighbor.push_back(neiId);
}

void SPoint::removeNeighbor(int neiId)
{
	if (hasNeighbor(neiId)) {
		vector<int>::iterator iter;
		for (iter = neighbor.begin(); iter != neighbor.end(); ++iter) {
			if (*iter == neiId) {
				iter = neighbor.erase(iter);
				break;
			}
		}
	}
}


void SPoint::calculateD_QEMMat(PointSet * set)
{
	int isize = (int)neighbor.size();//一环领域点个数
	Texture_errors_reset();//初始化顾及细节的二次误差矩阵集合
	
	//计算顶点的二次误差矩阵集合
	for (auto a : SPointUV_index) {//遍历该点的每一个纹理点标识
		Texture_error_reset();//初始化单个误差矩阵

		//计算每一个纹理点相关的误差矩阵
		for (int i = 0; i < isize; ++i) {
			for (int j = i + 1; j < isize; ++j) {
				if (set->point.at(neighbor[i]).hasNeighbor(neighbor[j])) {//判断是否能构成三角面
					SPoint P = set->point.at(neighbor[i]);
					SPoint Q = set->point.at(neighbor[j]);

					cv::Mat p = cv::Mat::zeros(5, 1, CV_64F);//点P的五维向量
					cv::Mat q = cv::Mat::zeros(5, 1, CV_64F);//点Q的五维向量
					cv::Mat r = cv::Mat::zeros(5, 1, CV_64F);//点R的五维向量
					r.at<double>(0, 0) = P.Point_Vector[0];
					r.at<double>(1, 0) = P.Point_Vector[1];
					r.at<double>(2, 0) = P.Point_Vector[2];
					q.at<double>(0, 0) = Q.Point_Vector[0];
					q.at<double>(1, 0) = Q.Point_Vector[1];
					q.at<double>(2, 0) = Q.Point_Vector[2];
					p.at<double>(0, 0) = Point_Vector[0];
					p.at<double>(1, 0) = Point_Vector[1];
					p.at<double>(2, 0) = Point_Vector[2];

					for (auto b : P.SPointUV_index) {//遍历P点的每一个纹理点
						for (auto c : Q.SPointUV_index) {//遍历Q点的每一个纹理点
							SPoint_UV A = set->point_uv.at(a);
							SPoint_UV B = set->point_uv.at(b);
							SPoint_UV C = set->point_uv.at(c);
							if (A.hasNeighbor(b) && A.hasNeighbor(c) && B.hasNeighbor(a) && B.hasNeighbor(c) && C.hasNeighbor(a) && C.hasNeighbor(b)) {//判断是否能构成纹理三角面
								r.at<double>(3, 0) = B.PointUV_Vector[0];
								r.at<double>(4, 0) = B.PointUV_Vector[1];
								q.at<double>(3, 0) = C.PointUV_Vector[0];
								q.at<double>(4, 0) = C.PointUV_Vector[1];
								p.at<double>(3, 0) = A.PointUV_Vector[0];
								p.at<double>(4, 0) = A.PointUV_Vector[1];

								double E1[5], E2[5], r_minus_p[5];
								//double E1[6], E2[6], r_minus_p[6];
								for (int i = 0; i < 5; i++) {
									if (i < 3) {
										E1[i] = Q.Point_Vector[i] - Point_Vector[i];
										r_minus_p[i] = P.Point_Vector[i] - Point_Vector[i];
									}
									else
									{
										//E1[i] = Q.Point_rgb[i % 3] - Point_rgb[i % 3];
										E1[i] = q.at<double>(i, 0) - p.at<double>(i, 0);
										r_minus_p[i] = r.at<double>(i, 0) - p.at<double>(i, 0);
									}
								}
								double E1_Length = sqrt(E1[0] * E1[0] + E1[1] * E1[1] + E1[2] * E1[2] + E1[3] * E1[3] + E1[4] * E1[4]);
								for (int i = 0; i < 5; i++) {//e1单位化
									E1[i] = E1[i] / E1_Length;
								}
								double e1rp = 0;
								for (int i = 0; i < 5; i++) {
									e1rp = e1rp + E1[i] * r_minus_p[i];
								}
								for (int i = 0; i < 5; i++) {//e2
									E2[i] = r_minus_p[i] - e1rp * E1[i];
								}
								double E2_Length = sqrt(E2[0] * E2[0] + E2[1] * E2[1] + E2[2] * E2[2] + E2[3] * E2[3] + E2[4] * E2[4]);
								for (int i = 0; i < 5; i++) {//e2单位化
									E2[i] = E2[i] / E2_Length;
								}

								double Mat_A[5][5];//A
								double Mat_I[5][5];//I
								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										if (i == j) {
											Mat_I[i][j] = 1;
										}
										else
										{
											Mat_I[i][j] = 0;
										}

									}
								}

								for (int i = 0; i < 5; i++) {
									for (int j = 0; j < 5; j++) {
										Mat_A[i][j] = Mat_I[i][j] - E1[i] * E1[j] - E2[i] * E2[j];
									}
								}

								double Vec_b[5], pe1, pe2, pp;//b,p*e1,p*e2,p*p
								pe1 = 0.0;
								pe2 = 0.0;
								pp = 0.0;

								for (int i = 0; i < 5; i++) {
									pe1 = pe1 + p.at<double>(i, 0)*E1[i];
									pe2 = pe2 + p.at<double>(i, 0)*E2[i];
									pp = pp + p.at<double>(i, 0)*p.at<double>(i, 0);
								}
								for (int i = 0; i < 5; i++) {
									Vec_b[i] = pe1 * E1[i] + pe2 * E2[i] - p.at<double>(i, 0);
								}
								double parameter_c;//c
								parameter_c = pp - pe1 * pe1 - pe2 * pe2;

								double M[6][6];//Q
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										if (i < 5 && j < 5) {
											M[i][j] = Mat_A[i][j];
										}
										if (i == 5 && j < 5) {
											M[i][j] = Vec_b[j];
										}
										if (j == 5 && i < 5) {
											M[i][j] = Vec_b[i];
										}
										if (i == 5 && j == 5) {
											M[i][j] = parameter_c;
										}
									}
								}

								//矩阵相加
								for (int i = 0; i < 6; i++) {
									for (int j = 0; j < 6; j++) {
										Texture_error.at<double>(i, j) = Texture_error.at<double>(i, j) + M[i][j];
									}
								}

							}
						}
					}

				}
			}
		}
		//误差矩阵加入集合
		Texture_errors.push_back(Texture_error);
	}

	//参数一：计算接缝角度误差
	int N_SPoint_UV = SPointUV_index.size();//顶点纹理坐标点个数
	double SAE = 1;
	//SAE = SeamAngleError(set, N_SPoint_UV);//计算角度误差

	//参数二：计算纹理复杂度


	//参数三：计算顶点尖锐度
}


void SPoint::error_reset()
{
	error = cv::Mat::zeros(4, 4, CV_64F);
}

void SPoint::Texture_error_reset()
{
	//Texture_error = cv::Mat::zeros(7, 7, CV_64F);
	Texture_error = cv::Mat::zeros(6, 6, CV_64F);
}

void SPoint::Texture_errors_reset()
{
	Texture_errors.clear();
}


double SPoint::SeamAngleError(PointSet* set, int num_SPoint_UV)
{
	double E;
	SPoint p1, p2;
	int n = 0;
	int isize = (int)neighbor.size();

	switch (num_SPoint_UV)
	{
	case 1:
		E = 1;
		break;
	case 2:
		for (int i = 0; i < isize; ++i) {
			if (set->point.at(neighbor[i]).SPointUV_index.size() > 1 && n <= 1) {
				if (n == 0) {
					p1 = set->point.at(neighbor[i]);
				}
				else
				{
					p2 = set->point.at(neighbor[i]);
				}
				n++;
			}
		}
		if (n == 2) {
			for (auto a : SPointUV_index) {
				for (auto b : p1.SPointUV_index) {
					for (auto c : p2.SPointUV_index) {
						SPoint_UV A = set->point_uv.at(a);
						SPoint_UV B = set->point_uv.at(b);
						SPoint_UV C = set->point_uv.at(c);
						if (A.hasNeighbor(b) && B.hasNeighbor(a) && A.hasNeighbor(c) && C.hasNeighbor(a)) {
							cv::Vec3d pp1_xyz = p1.Point_Vector - Point_Vector;
							cv::Vec2d pp1_uv = B.PointUV_Vector - A.PointUV_Vector;
							cv::Mat pp1 = cv::Mat::zeros(1, 5, CV_64F);
							pp1.at<double>(0, 0) = pp1_xyz[0];
							pp1.at<double>(0, 1) = pp1_xyz[1];
							pp1.at<double>(0, 2) = pp1_xyz[2];
							pp1.at<double>(0, 3) = pp1_uv[0];
							pp1.at<double>(0, 4) = pp1_uv[1];
							
							cv::Vec3d pp2_xyz = p2.Point_Vector - Point_Vector;
							cv::Vec2d pp2_uv = C.PointUV_Vector - A.PointUV_Vector;
							cv::Mat pp2 = cv::Mat::zeros(5, 1, CV_64F);
							pp2.at<double>(0, 0) = pp2_xyz[0];
							pp2.at<double>(1, 0) = pp2_xyz[1];
							pp2.at<double>(2, 0) = pp2_xyz[2];
							pp2.at<double>(3, 0) = pp2_uv[0];
							pp2.at<double>(4, 0) = pp2_uv[1];
							
							cv::Mat pp1_pp2 = pp1 * pp2;
							double ss = pp1_pp2.at<double>(0, 0);
							
							cv::Mat m1 = pp1 * pp1.t();
							double s_pp1 = sqrt(m1.at<double>(0, 0));
							
							cv::Mat m2 = pp2.t() * pp2;
							double s_pp2 = sqrt(m2.at<double>(0, 0));
							
							double cos_angle = ss/(s_pp1*s_pp2);
							
							double angle = acos(cos_angle);
							
							E = E + 1 + 1 + cos_angle;
						}
					}
				}
			}
			E = 0.5*E;
		}
		else
		{
			E = 10000;
		}
		break;
	default:
		E = 10000;
		break;
	}

	return E;
}

SPoint_UV::SPoint_UV()
{
	PointUV_Vector = cv::Vec2d(0, 0);
}

bool SPoint_UV::hasNeighbor(int neiId)
{
	vector<int>::iterator iter;
	for (iter = neighbor.begin(); iter != neighbor.end(); ++iter)
		if (*iter == neiId)
			return true;
	return false;
}

void SPoint_UV::addNeighbor(int neiId)
{
	if (!hasNeighbor(neiId))
		neighbor.push_back(neiId);
}
