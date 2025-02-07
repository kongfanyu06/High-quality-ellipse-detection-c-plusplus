#define _CRT_SECURE_NO_WARNINGS
#include "generateEllipseCandidates.h"

#include "double_camera.h"
#include "ellipseDetectionByArcSupportLSs.h"
using namespace ours;

// 从返回的多个椭圆中选出大小2个椭圆   理想情况下会传入4个椭圆   简单筛选，实际上情况很多种，这里只处理最简单的
vector<ours::Ellipse> getMaxMinEllipses(vector<ours::Ellipse> ellipses)
{
    // 找最大最小的椭圆
    double maxA = 0;
    for (int i = 0; i < ellipses.size(); i++)
    {
        ours::Ellipse ellipse = ellipses.at(i);
        if (ellipse.a > maxA)
        {
            maxA = ellipse.a;
        }

    }
    ellipses.erase(std::remove_if(ellipses.begin(), ellipses.end(), [&maxA](ours::Ellipse ellipse) {
        return ellipse.b < maxA / 5; }), ellipses.end());
    vector<ours::Ellipse> maxMin;
    if (ellipses.size() < 2)
    {
        maxMin.push_back(ellipses.at(0));
        maxMin.push_back(ellipses.at(0));
        return maxMin;
    }

    double minA = ellipses.at(0).a;
    maxA = 0;
    int indexMax = 0;
    int indexMin = 0;
    for (int i = 0; i < ellipses.size(); i++)
    {
        ours::Ellipse ellipse = ellipses.at(i);
        if (ellipse.a > maxA)
        {
            maxA = ellipse.a;
            indexMax = i;
        }
        if (ellipse.a < minA)
        {
            minA = ellipse.a;
            indexMin = i;
        }
    }
    maxMin.push_back(ellipses.at(indexMax));
    maxMin.push_back(ellipses.at(indexMin));

    // 获取圆心位置相同的圆
    vector<Point2d> maxPoints;
    vector<Point2d> minPoints;
    for (int i = 0; i < ellipses.size(); i++)
    {
        ours::Ellipse ellipse = ellipses.at(i);
        if (ellipse.a / maxA > 0.8 && abs(ellipse.x - maxMin.at(0).x < 10) && abs(ellipse.y - maxMin.at(0).y) < 10)
        {
            maxPoints.push_back({ ellipse.x , ellipse.y });
        }
        if (ellipse.a / minA > 0.8 && ellipse.a / minA < 1.5 && abs(ellipse.x - maxMin.at(1).x) < 10 && abs(ellipse.y - maxMin.at(1).y) < 10)
        {
            minPoints.push_back({ ellipse.x , ellipse.y });
        }
    }
    maxMin.at(0).x = 0;
    maxMin.at(0).y = 0;
    // 计算平均值
    for (Point2d p: maxPoints)
    {
        maxMin.at(0).x += p.x;
        maxMin.at(0).y += p.y;
    }
    maxMin.at(0).x /= maxPoints.size();
    maxMin.at(0).y /= maxPoints.size();

    maxMin.at(1).x = 0;
    maxMin.at(1).y = 0;
    for (Point2d p : minPoints)
    {
        maxMin.at(1).x += p.x;
        maxMin.at(1).y += p.y;
    }
    maxMin.at(1).x /= minPoints.size();
    maxMin.at(1).y /= minPoints.size();


    return maxMin;
}

int getPosition(int i, vector<ours::Ellipse>& maxMinL, vector<ours::Ellipse>& maxMinR)
{
    //cv::Mat imageL = imread("D:/code/c++/double_camera/demo/38L.png", cv::IMREAD_COLOR);;
   //cv::Mat imageR = imread("D:/code/c++/double_camera/demo/38R.png", cv::IMREAD_COLOR);;
    cout << i << endl;
    cv::Mat imageL = imread("D:/code/c++/double_camera/demo/move_day_0_4.0-0.5/" + to_string(i) + "L.png", cv::IMREAD_COLOR);;
    cv::Mat imageR = imread("D:/code/c++/double_camera/demo/move_day_0_4.0-0.5/" + to_string(i) + "R.png", cv::IMREAD_COLOR);;

    // 检查图片是否成功读取  
    if (imageL.empty() || imageR.empty()) {
        std::cout << "无法读取图片！" << std::endl;
        return -1;
    }
    vector<ours::Ellipse> ellipsesL = getEllipses(imageL, maxMinL);
   // return -1;
    vector<ours::Ellipse> ellipsesR = getEllipses(imageR, maxMinR);

    //for (ours::Ellipse e : ellipsesL)
    //{
    //    cout << e.x << " " << e.y << " " << e.a << " " << e.b <<" " << e.phi<<  endl;
    //}
    //for (ours::Ellipse e : ellipsesR)
    //{
    //    cout << e.x << " " << e.y << " " << e.a << " " << e.b << endl;
    //}


    //282
    //vector<ours::Ellipse> ellipsesL = { {607.856, 517.09, 63.6344, 63.3666, 0.411072},
    //                                        { 603.455, 515.244, 163.984, 158.901, 1.46474},
    //                                        {  607.748, 516.315, 51.9166, 51.2339, 1.2518},
    //                                     {608.523, 516.822, 162.844, 159.015, 0.518686},
    //                                     {605.465, 512.288, 182.778, 181.693, 0.923509}};

    //vector<ours::Ellipse> ellipsesR = { { 473.295, 508.851, 51.6789, 51.0531, 1.16794},
    //                                        { 481.586, 505.251, 182.706, 181.836, 0.952785},
    //                                        { 483.28, 508.146, 157.905, 155.181, -1.55036},
    //                                        {473.366, 509.673, 63.6523, 63.0356, 0.212549},
    //                                        {483.165, 508.705, 164.892, 163.303, -0.469696}};

    // 283
    /*vector<ours::Ellipse> ellipsesL = { {617.612, 515.076, 52.2056, 52.0956, - 0.43299},
                                            { 614.604, 511.781, 185.092, 184.493, 1.03643},
                                            {  614.322, 518.332, 166.741, 162.511, 0.175917},
     {490.1, 693.666, 17.7404, 6.59144, - 0.97889},
     {618.437, 508.122, 164.305, 160.148, - 1.21407},
     {617.822, 516.088, 64.6519, 64.0832, 0.0352962}};

    vector<ours::Ellipse> ellipsesR = { { 489.813, 504.827, 184.995, 184.615, 1.18512},
                                            { 482.121, 508.263, 52.631, 51.8845, 1.00913},
                                            { 491.706, 510.944, 167.239, 162.138, -0.271338},
                                            {493.877, 505.074, 160.184, 157.673, -1.45492},
                                            {371.23, 695.296, 23.6854, 15.0634, -1.15633}};*/
   /* vector<ours::Ellipse> maxMinL = getMaxMinEllipses(ellipsesL);
    vector<ours::Ellipse> maxMinR = getMaxMinEllipses(ellipsesR);*/
    maxMinL = getMaxMinEllipses(ellipsesL);
    maxMinR = getMaxMinEllipses(ellipsesR);

    /*614.604 511.781 185.092 184.493
    617.612 515.076 52.2056 52.0956
    489.813 504.827 184.995 184.615
    482.121 508.263 52.631 51.8845*/
    /*for (ours::Ellipse e: maxMinL)
    {
        cout << e.x << " " << e.y << " " << e.a << " " << e.b << endl;
    }
    for (ours::Ellipse e: maxMinR)
    {
        cout << e.x << " " << e.y << " " << e.a << " " << e.b << endl;
    }*/
    // 左边大小2个椭圆的圆心
    std::vector<cv::Point2d> maxMinPointsL = { cv::Point2d(maxMinL.at(0).x,  maxMinL.at(0).y), cv::Point2d(maxMinL.at(1).x,  maxMinL.at(1).y) };
    std::vector<cv::Point2f> maxMinPointsR = { cv::Point2d(maxMinR.at(0).x,  maxMinR.at(0).y), cv::Point2d(maxMinR.at(1).x,  maxMinR.at(1).y) };


    vector<Vec3d> points = get3DPosition(maxMinPointsL, maxMinPointsR);
    return 0;
}

int main()
{
    vector<ours::Ellipse> maxMinL;
    vector<ours::Ellipse> maxMinR;
    for (size_t i = 281; i < 282; i++)
    {
        clock_t start = clock();

        getPosition(i, maxMinL, maxMinR);
        clock_t t1 = clock();
        cout << "运行时间:" << (double)(t1 - start) / CLOCKS_PER_SEC << endl;
    }
    return 0;

 
    return 0;
}


