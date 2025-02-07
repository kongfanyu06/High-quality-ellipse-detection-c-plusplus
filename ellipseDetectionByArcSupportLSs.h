#pragma once
#include <vector>
#include "opencv2/core/core.hpp" 
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
#include <cmath>
#include <algorithm>
#include<numeric>
#include "time.h"
#include "generateEllipseCandidates.h"

using namespace std;
using namespace cv;
using namespace ours;
//struct Ellipse {
//    double x, y, a, b, phi; // Ellipse center (x, y), major axis (a), minor axis (b), and angle (phi)
//};

//struct Point {
//    double x, y;
//};
// c++ 12中没有std::clamp需要自己实现   opencv2.4.9要求c++12
template<typename T>
inline T clamp(T x, T minV, T maxV)
{
    return (std::max)((std::min)(x, maxV), minV);
}


void ellipseDetection(vector<ours::Ellipse>& candidates, const vector<Point2d>& points, const vector<Point2d>& normals,
    double distance_tolerance, double normal_tolerance, double Tmin, double angleCoverage,
    vector<int>& labels, vector<int>& mylabels, vector<ours::Ellipse>& ellipses);

double dotProduct(const vector<Point2d>& normals, const vector<Point2d>& ellipse_normals, int idx);

std::vector<bool> takeInliers(const std::vector<Point2d>& x, const Point2d& center, int tbins);

std::vector<double> dRosin_square(const ours::Ellipse& param, const std::vector<Point2d>& points);

double dRosin_square(const ours::Ellipse& candidate, const Point& point);

double calcuCompleteness(const std::vector<Point2d>& x, const Point2d& center, int tbins);

std::vector<Point2d> computePointAngle(const ours::Ellipse& ellipse, const std::vector<Point2d>& points);

ours::Ellipse fitEllipse(const vector<double>& X, const vector<double>& Y, int& info);

void cart2pol(const std::vector<Point2d>& points, const Point2d& center, std::vector<double>& theta);

void subEllipseDetection(const std::vector<ours::Ellipse> list,
    const std::vector<Point2d>& points,
    const std::vector<Point2d>& normals,
    double distance_tolerance, double normal_tolerance,
    double Tmin, double angleCoverage, std::vector<int>& mylabels, 
    std::vector<int>& labels, std::vector<ours::Ellipse>& ellipses, std::vector<bool>& validCandidates);

int ellipse2Param(double* p, double param[]);

vector<ours::Ellipse> getEllipses(cv::Mat image, vector<ours::Ellipse> maxMin);

void findMaxBoundingRect(cv::Mat image, cv::Rect& max_rect);