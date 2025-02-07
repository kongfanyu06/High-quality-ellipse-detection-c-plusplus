#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <limits.h>
#include <float.h>
#include <iostream>
//#include <lapack.h>  //matlab 
//#include "include/lapacke_config.h"  //lapack手动，未成功
//#include "include/lapacke.h"
#include "opencv2/core/core.hpp" 
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2\opencv.hpp>
//#include <lapacke.h>
#include "Eigen/Dense"
using namespace cv;
using namespace std;



#ifndef FALSE
#define FALSE 0
#endif /* !FALSE */

#ifndef TRUE
#define TRUE 1
#endif /* !TRUE */

/** Label for pixels with undefined gradient. */
#define NOTDEF -1024.0
/** PI */
#ifndef M_PI
#define M_PI   3.14159265358979323846
#endif /* !M_PI */
#define M_1_2_PI 1.57079632679489661923
#define M_1_4_PI 0.785398163

#define M_3_4_PI 2.35619449

#define M_1_8_PI 0.392699081
#define M_3_8_PI 1.178097245
#define M_5_8_PI 1.963495408
#define M_7_8_PI 2.748893572
#define M_4_9_PI 1.396263401595464  //80°
#define M_1_9_PI  0.34906585  //20°
#define M_1_10_PI 0.314159265358979323846   //18°
#define M_1_12_PI 0.261799387   //15°
#define M_1_15_PI 0.20943951    //12°
#define M_1_18_PI 0.174532925   //10°
/** 3/2 pi */
#define M_3_2_PI 4.71238898038
/** 2 pi */
#define M_2__PI  6.28318530718
/** Doubles relative error factor
 */
#define RELATIVE_ERROR_FACTOR 100.0
 //#pragma comment(lib,"libmwlapack.lib")
 //#pragma comment(lib,"libmwblas.lib")
 //#pragma comment(lib,"tmglib.lib")
 //#pragma comment(lib,"libf2c.lib")


struct point2i //(or pixel).
{
	int x, y;
};

struct point2d
{
	double x, y;
};

struct point1d1i
{
	double data;
	int cnt;
};

struct point3d
{
	double x, y;
	double r;
};

struct point3i
{
	int x, y;
	int z;
};

struct point2d1i
{
	double x, y;
	int z;
};

struct  point5d
{
	double x, y;
	double a, b;
	double phi;
};
namespace ours
{
	struct Ellipse {
		double x, y, a, b, phi; // Ellipse center (x, y), major axis (a), minor axis (b), and angle (phi)
	};
}


/*----------------------------------------------------------------------------*/
/** Rectangle structure: line segment with width.
 */
struct rect
{
	double x1, y1, x2, y2;  /* first and second point of the line segment */
	double width;        /* rectangle width */
	double x, y;          /* center of the rectangle */
	double theta;        /* angle */
	double dx, dy;        /* (dx,dy) is vector oriented as the line segment,dx = cos(theta), dy = sin(theta) */
	int   polarity;     /* if the arc direction is the same as the edge direction, polarity = 1, else if opposite ,polarity = -1.*/
	double prec;         /* tolerance angle */
	double p;            /* probability of a point with angle within 'prec' */
};

typedef struct
{
	double vx[4];  /* rectangle's corner X coordinates in circular order */
	double vy[4];  /* rectangle's corner Y coordinates in circular order */
	double ys, ye;  /* start and end Y values of current 'column' */
	int x, y;       /* coordinates of currently explored pixel */
} rect_iter;

typedef struct image_double_s
{
	double* data;
	int xsize, ysize;
} *image_double;

using namespace ours;
void mexFunction1(const Mat& inputImg, int edge_process_select, int specified_polarity, vector<Ellipse>& candidates_out, cv::Mat& imageEdge, vector<Point2d>& normals, cv::Mat& ls_mat, vector<Point2d>& points);