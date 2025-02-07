#include "ellipseDetectionByArcSupportLSs.h"
using namespace ours;


double dotProduct(const vector<Point2d>& normals, const vector<Point2d>& ellipse_normals, int idx) {
    return normals[idx].x * ellipse_normals[idx].x + normals[idx].y * ellipse_normals[idx].y;
}

vector<int> findInliers(const vector<Point2d>& points, const Ellipse& candidate, double tbins) {
    vector<int> inliers;
    // Assume dRosin_square function and conditions
    vector<double> distance = dRosin_square(candidate, points);
    for (int i = 0; i < distance.size(); i++) {
        if (distance.at(i) <= 1) {
            inliers.push_back(i);
        }
    }
   /* for (int i = 0; i < points.size(); i++) {
        if (dRosin_square(candidate, points[i]) <= 1) {
            inliers.push_back(i);
        }
    }*/
    return inliers;
}

void ellipseDetection(vector<Ellipse>& candidates, const vector<Point2d>& points, const vector<Point2d>& normals,
    double distance_tolerance, double normal_tolerance, double Tmin, double angleCoverage,
    vector<int>& labels, vector<int>& mylabels, vector<Ellipse>& ellipses) 
{

    vector<double> goodness(candidates.size(), 0);
    for (int i = 0; i < candidates.size(); i++) 
    {
        /*clock_t start, end;
        start = clock();*/
        const Ellipse& candidate = candidates[i];
        Point2d ellipseCenter = { candidate.x, candidate.y };
        Point2d ellipseAxes = { candidate.a, candidate.b };
        double tbins = std::min({ 180.0, floor(M_PI * (1.5 * (candidate.a + candidate.b) - sqrt(candidate.a * candidate.b)) * Tmin) });
        clock_t start, end;
        start = clock();
        // 加速计算，只挑出椭圆外接矩形内的边缘点
        std::vector<int> i_dx;
        for (int j = 0; j < points.size(); ++j)
        {
            if (points[j].x >= (ellipseCenter.x - ellipseAxes.x - distance_tolerance - 1) &&
                points[j].x <= (ellipseCenter.x + ellipseAxes.x + distance_tolerance + 1) &&
                points[j].y >= (ellipseCenter.y - ellipseAxes.x - distance_tolerance - 1) &&
                points[j].y <= (ellipseCenter.y + ellipseAxes.x + distance_tolerance + 1))
            {
                i_dx.push_back(j);
            }
        }

        std::vector<int> inliers;
        for (int j : i_dx) {
            if (dRosin_square(candidate, points[j]) <= 1)
            {
                inliers.push_back(j);
            }
        }

        //vector<int> inliers = findInliers(points, candidate, tbins);
        //clock_t t1 = clock();
        //cout << "t1运行时间:" << (double)(t1 - start) / CLOCKS_PER_SEC << endl;
        //vector<Point2d> ellipse_normals; // Compute ellipse normals for inliers
        //for (int j : inliers) {
        //    // Compute normal for each inlier
        //    ellipse_normals.push_back(normals.at(j));
        //}
        /*clock_t t2 = clock();
        cout << "t2运行时间:" << (double)(t2 - t1) / CLOCKS_PER_SEC << endl;*/
        /*vector<double> p_dot_temp(inliers.size(), 0);
        for (int j = 0; j < inliers.size(); j++) {
            p_dot_temp[j] = dotProduct(normals, ellipse_normals, j);
        }*/
       /* clock_t t3 = clock();
        cout << "t3运行时间:" << (double)(t3 - t2) / CLOCKS_PER_SEC << endl;*/
        //int p_cnt = count_if(p_dot_temp.begin(), p_dot_temp.end(), [](double x) { return x > 0; });
        //double ellipse_polarity = 0; // 椭圆极性
        
        //if (p_cnt > inliers.size() * 0.5)
        //{
        //    // 仅保留极性相同的内点
        //    int index = inliers.size();
        //      inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&p_dot_temp, &ellipse_polarity, & index](int idx) {
        //          index--;
        //                    return !(p_dot_temp[index] > 0 && p_dot_temp[index] >= 0.923879532511287);
        //                    }), inliers.end());
        //}
        //else
        //{
        //    int index = inliers.size();
        //    inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&p_dot_temp, &ellipse_polarity, &index](int idx) {
        //        index--;
        //        return !(p_dot_temp[index] < 0 && p_dot_temp[index] <= -0.923879532511287);
        //        }), inliers.end());
        //}
        std::vector<Point2d> newPoints1;
        for (int j: inliers)
        {
            newPoints1.push_back(points.at(j));
        }
        /*clock_t t4 = clock();
        cout << "t4运行时间:" << (double)(t4 - t3) / CLOCKS_PER_SEC << endl;*/
        std::vector<bool> inliersIdx = takeInliers(newPoints1, ellipseCenter, tbins); // 内点提纯
        /*clock_t t5 = clock();
        cout << "t5运行时间:" << (double)(t5 - t4) / CLOCKS_PER_SEC << endl;*/
        int index = inliers.size();
        inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&inliersIdx, &index](int idx) {
            index--;
            return inliersIdx[index] == false;
            }), inliers.end());

        double support_inliers_ratio = static_cast<double>(inliers.size()) / floor(M_PI * (1.5 * (candidate.a + candidate.b) - sqrt(candidate.a * candidate.b)));

        std::vector<Point2d> newPoints2;
        for (int j : inliers)
        {
            newPoints2.push_back(points.at(j));
        }
        /*clock_t t6 = clock();
        cout << "t6运行时间:" << (double)(t6 - t5) / CLOCKS_PER_SEC << endl;*/
        double completeness_ratio = calcuCompleteness(newPoints2, ellipseCenter, tbins) / 360;
        goodness[i] = sqrt(support_inliers_ratio * completeness_ratio);
        /*clock_t t7 = clock();
        cout << "t7运行时间:" << (double)(t7 - t6) / CLOCKS_PER_SEC << endl;*/
        /*end = clock();
        cout <<i<< "运行时间:" << (double)(end - start) / CLOCKS_PER_SEC << endl;*/
    }

    vector<int> sorted_indices(candidates.size());
    iota(sorted_indices.begin(), sorted_indices.end(), 0);
    sort(sorted_indices.begin(), sorted_indices.end(), [&goodness](int a, int b) {
        return goodness[a] > goodness[b];
        });

    vector<Ellipse> filtered_candidates;
    for (int i : sorted_indices) {
        if (goodness[i] > 0) {
            filtered_candidates.push_back(candidates[i]);
        }
    }
    candidates = filtered_candidates; // Update candidates with the sorted ones
    vector<int> angles = { 300, 210, 150, 90 };
    angles.erase(remove_if(angles.begin(), angles.end(), [&](int x) { return x < angleCoverage; }), angles.end());
    if (find(angles.begin(), angles.end(), angleCoverage) == angles.end()) {
        angles.push_back(angleCoverage);
    }

    for (int angleLoop : angles) {
        vector<int> idx; // find elements where labels are zero
        for (int i = 0; i < labels.size(); i++) 
        {
            if (labels.at(i) == 0)
            {
                idx.push_back(i);
            }
        }
        
        if (idx.size() < 2 * M_PI * (6 * distance_tolerance) * Tmin) {
            break;
        }
 
        int m = candidates.size();

        std::vector<Point2d> newPoints2;
        for (int j : idx)
        {
            newPoints2.push_back(points.at(j));
        }

        std::vector<Point2d> newNormals;
        for (int j : idx)
        {
            newNormals.push_back(normals.at(j));
        }

        std::vector<bool> validCandidates(m, true); // logical向量，大小 candidate_n x 1
        vector<Ellipse> C;
        vector<int> L(newPoints2.size(), 0);
        vector<int> L2(newPoints2.size(), 0);
        subEllipseDetection(candidates, newPoints2, newNormals, distance_tolerance, normal_tolerance, Tmin, angleLoop, L2, L, C, validCandidates);

        vector<Ellipse> new_filtered_candidates;
        for (int i = 0; i < m; i++) {
            if (validCandidates[i] > 0) {
                new_filtered_candidates.push_back(candidates[i]);
            }
        }
        candidates = new_filtered_candidates; // Update candidates after subDetection

        
        // Process results
        for (int i = 0; i < C.size(); i++) 
        {
            bool flag = false;
            for (int j = 0; j < ellipses.size(); j++) 
            {
                if (sqrt(pow(C[i].x - ellipses[j].x, 2) + pow(C[i].y - ellipses[j].y, 2)) <= distance_tolerance &&
                    sqrt(pow(C[i].a - ellipses[j].a, 2) + pow(C[i].b - ellipses[j].b, 2)) <= distance_tolerance &&
                    abs(C[i].phi - ellipses[j].phi) <= 0.314159265358979) { // pi/10 = 18°
                    flag = true;
                    for (int k = 0; k < idx.size(); k++)
                    {
                        if (L.at(k) == i)
                        {
                            labels.at(idx.at(k)) = j;
                        }
                        if (L2.at(k) == i)
                        {
                            mylabels.at(idx.at(k)) = j;
                        }
                    }
                    break;
                   
                }
            }
            if (!flag) 
            {
                for (int k = 0; k < idx.size(); k++)
                {
                    if (L.at(k) == i)
                    {
                        labels.at(idx.at(k)) = ellipses.size() + 1;
                    }
                    if (L2.at(k) == i)
                    {
                        mylabels.at(idx.at(k)) = ellipses.size() + 1;
                    }
                }
                ellipses.push_back(C[i]);  
            }
        }
    }
}


//// 判断是否是有效的内点（根据距离和法线）
//bool isValidInlier(const std::vector<double>& normal, const std::vector<std::vector<double>>& ellipseNormal, double normal_tolerance) {
//    return (std::abs(std::acos(std::min(1.0, std::abs(std::inner_product(normal.begin(), normal.end(), ellipseNormal.begin(), 0.0))))) <= normal_tolerance);
//}

// 定义subEllipseDetection函数
void subEllipseDetection(const std::vector<Ellipse> list,
    const std::vector<Point2d>& points,
    const std::vector<Point2d>& normals,
    double distance_tolerance, double normal_tolerance,
    double Tmin, double angleCoverage, std::vector<int>& mylabels, std::vector<int>& labels, std::vector<Ellipse>& ellipses, std::vector<bool>& validCandidates) 
{
    
    int m = list.size();

    double ellipse_polarity = 0; // 椭圆极性
    double max_dis = std::max_element(points.begin(), points.end(), [](const Point2d& a, const Point2d& b) {
        return a.x < b.x; // 替换为您的度量方法来获取最大距离
        }) - std::min_element(points.begin(), points.end(), [](const Point2d& a, const Point2d& b) 
            {
            return a.x < b.x; // 替换为您的度量方法来获取最小距离
            });

        double maxSemiMajor = max_dis; // 最大的可能半径(此处可改为/2)
        double maxSemiMinor = max_dis; // 最大可能的短半轴

        double distance_tolerance_square = distance_tolerance * distance_tolerance;
        std::vector<Ellipse> convergence = list; // 候选椭圆副本

    for (int i = 0; i < m; ++i) 
    {

        Point2d ellipseCenter = { list[i].x, list[i].y };
        Point2d ellipseAxes = { list[i].a, list[i].b };
        double ellipsePhi = list[i].phi;

        // 椭圆的周长估算
        int tbins = std::min(180, static_cast<int>(M_PI * (1.5 * (ellipseAxes.x + ellipseAxes.y) - sqrt(ellipseAxes.x * ellipseAxes.y)) * Tmin));

        // 加速计算，只挑出椭圆外接矩形内的边缘点
        std::vector<int> i_dx;
        for (int j = 0; j < points.size(); ++j) 
        {
            if (points[j].x >= (ellipseCenter.x - ellipseAxes.x - distance_tolerance - 1) &&
                points[j].x <= (ellipseCenter.x + ellipseAxes.x + distance_tolerance + 1) &&
                points[j].y >= (ellipseCenter.y - ellipseAxes.x - distance_tolerance - 1) &&
                points[j].y <= (ellipseCenter.y + ellipseAxes.x + distance_tolerance + 1)) 
            {
                i_dx.push_back(j);
            }
        }

        std::vector<int> inliers;
        std::vector<int> inliers3;
        std::vector<Point2d> points0;
        for (int j : i_dx) {
            if (labels[j] == 0 && dRosin_square(list[i], points[j]) <= distance_tolerance_square) 
            {
                inliers.push_back(j);
                points0.push_back(points.at(j));
            }
        }

        std::vector<Point2d> normals0;
        std::vector<Point2d> ellipse_normals = computePointAngle(list[i], points0); // 计算椭圆的法线
        std::vector<double> p_dot_temp(inliers.size());
        for (int j: inliers) 
        {
            normals0.push_back(normals.at(j));
        }
        for (size_t j = 0; j < inliers.size(); ++j) 
        {
            p_dot_temp[j] = dotProduct(normals0, ellipse_normals, j);
        }

        int p_cnt = std::count_if(p_dot_temp.begin(), p_dot_temp.end(), [](double val) { return val > 0; });

        //if (p_cnt > inliers.size() * 0.5) 
        //{
        //    ellipse_polarity = -1;
        //    // 仅保留极性相同的内点
        //    int index = inliers.size();
        //    inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&p_dot_temp, &ellipse_polarity, &index](int idx) 
        //        {
        //        index--;
        //        return !(p_dot_temp[index] > 0 && p_dot_temp[index] >= 0.923879532511287);
        //        }), inliers.end());
        //}
        //else {
        //    ellipse_polarity = 1;
        //    int index = inliers.size();
        //    inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&p_dot_temp, &ellipse_polarity, &index](int idx) 
        //        {
        //        index--;
        //        return !(p_dot_temp[index] < 0 &&  p_dot_temp[index] <= -0.923879532511287);
        //        }), inliers.end());
        //}
        std::vector<Point2d> newPoints1;
        for (int j : inliers)
        {
            newPoints1.push_back(points.at(j));
        }
        std::vector<bool> inliersIdx = takeInliers(newPoints1, ellipseCenter, tbins); // 内点提纯
        int index = inliers.size();
        inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&inliersIdx, &index](int idx) 
            {
            index--;
            return inliersIdx[index] == false;
            }), inliers.end());

        //std::vector<Point2d> newPoints = points.
        vector<double> X;
        vector<double> Y;
        for (unsigned int i = 0; i < inliers.size(); i++)
        {
            X.push_back(points[inliers.at(i)].x);
            Y.push_back(points[inliers.at(i)].y);
        }
        int new_info;
        Ellipse new_ellipse;
        if (inliers.size() > 0)
        {
            new_ellipse = fitEllipse(X, Y, new_info); // 最小二乘法拟合椭圆
        }
        else 
        {
            new_info = 0;
        }
        

        // 如果新椭圆拟合成功
        if (new_info == 1) 
        {
            // 判断是否满足距离条件和倾角条件
            if (((pow(new_ellipse.x - ellipseCenter.x, 2) + pow(new_ellipse.y - ellipseCenter.y, 2)) <= 16 * distance_tolerance_square) &&
                ((pow(new_ellipse.a - ellipseAxes.x, 2) + pow(new_ellipse.b - ellipseAxes.y, 2)) <= 16 * distance_tolerance_square) &&
                (abs(new_ellipse.phi - ellipsePhi) <= 0.314159265358979)) 
            {

                // 计算椭圆法线
                //std::vector<Point2d> new_ellipse_normals = computePointAngle(new_ellipse, points);

                // 找到边缘点在椭圆外接矩形内
                vector<int> new_i_dx;
                std::vector<Point2d> newPoints1;
                std::vector<Point2d> normals1;
                for (int i = 0; i < points.size(); ++i) 
                {
                    if (points[i].x >= (new_ellipse.x - new_ellipse.a - distance_tolerance - 1) &&
                        points[i].x <= (new_ellipse.x + new_ellipse.a + distance_tolerance + 1) &&
                        points[i].y >= (new_ellipse.y - new_ellipse.a - distance_tolerance - 1) &&
                        points[i].y <= (new_ellipse.y + new_ellipse.a + distance_tolerance + 1)) 
                    {
                        new_i_dx.push_back(i);
                        newPoints1.push_back(points.at(i));
                        normals1.push_back(normals.at(i));
                    }
                }

                // 计算这些点的法线
                std::vector<Point2d> new_ellipse_normals = computePointAngle(new_ellipse, newPoints1); // 计算椭圆的法线
                // 根据条件找内点
                std::vector<int> newInliers;
                std::vector<Point2d> newPoints;
                for (int ii = 0; ii < new_i_dx.size(); ii++)
                {
                    int ii_idx = new_i_dx.at(ii);
                   /* if (labels[ii_idx] == 0 && (dRosin_square(new_ellipse, points.at(ii_idx)) <= distance_tolerance_square) &&
                        (dotProduct(normals1, new_ellipse_normals, ii) * (-ellipse_polarity) >= 0.923879532511287))*/ 
                    if (labels[ii_idx] == 0 && (dRosin_square(new_ellipse, points.at(ii_idx)) <= distance_tolerance_square))
                    {
                        newInliers.push_back(ii_idx);
                        newPoints.push_back(points.at(ii_idx));
                    }
                }

                // 进一步处理新的内点
                Point2d newEllipseCenter = { new_ellipse.x, new_ellipse.y };
              /*  if (i == 0 || i == 1 | i == 2 || i == 8)
                {
                    cv::Mat show = Mat::zeros(720, 1280, CV_8UC1);
                    for (Point2d p : newPoints)
                    {
                        show.at<uchar>(p) = 255;
                    }
                    imshow("show", show);
                    waitKey(0);
                }*/
                
                std::vector<bool> newInliersIdx = takeInliers(newPoints, newEllipseCenter, tbins);
                int index = newInliers.size();
                newInliers.erase(std::remove_if(newInliers.begin(), newInliers.end(), [&newInliersIdx, &index](int idx) {
                    index--;
                    return newInliersIdx[index] == false;
                    }), newInliers.end());

                if (newInliers.size() >= inliers.size()) 
                {
                    inliers = newInliers;
                    inliers3 = newInliers;

                       
                    // 二次拟合
                    // 新椭圆拟合
                    vector<double> X1;
                    vector<double> Y1;
                    for (unsigned int i = 0; i < inliers.size(); i++)
                    {
                        X1.push_back(points[inliers.at(i)].x);
                        Y1.push_back(points[inliers.at(i)].y);
                    }
                    int new_new_info;
                    Ellipse new_new_ellipse = fitEllipse(X1, Y1, new_new_info); // 最小二乘法拟合椭圆
                    // 假设新拟合成功
                    if (new_new_info == 1) 
                    {
                        new_ellipse = new_new_ellipse;
                    }
                }
            }
        }
        else {
            continue;
            // 如果新拟合失败，直接使用候选
            new_ellipse = list.at(i);  // 假设使用候选值
        }

        // 计算内点是否足够并且满足angleCoverage的完整度
        if (inliers.size() >= std::floor(M_PI * (1.5 * (new_ellipse.a + new_ellipse.b) - sqrt(new_ellipse.a * new_ellipse.b)) * Tmin)) 
        {
            convergence[i] = new_ellipse;
            // 如果新椭圆和现有椭圆重复则淘汰
            if (std::any_of(convergence.begin(), convergence.begin() + i, [&new_ellipse, &distance_tolerance](const Ellipse& old_ellipse) {
                return std::sqrt(std::pow(old_ellipse.x - new_ellipse.x, 2) + std::pow(old_ellipse.y - new_ellipse.y, 2)) <= distance_tolerance &&
                    std::sqrt(std::pow(old_ellipse.a - new_ellipse.a, 2) + std::pow(old_ellipse.b - new_ellipse.b, 2)) <= distance_tolerance &&
                    std::abs(old_ellipse.phi - new_ellipse.phi) <= 0.314159265358979;
                })) {
                validCandidates[i] = false;
            }

            // 根据条件找内点
            std::vector<Point2d> points1;
            for (int ii : inliers)
            {
                points1.push_back(points.at(ii));
            }
  
            Point2d ellipseCenter1 = { new_ellipse.x, new_ellipse.y };
            // 计算 completeness
            bool completeOrNot = calcuCompleteness(points1, ellipseCenter1, tbins) >= angleCoverage;
            
            if (new_info == 1 && new_ellipse.a < maxSemiMajor && new_ellipse.b < maxSemiMinor && completeOrNot) 
            {
                // 检查新椭圆是否与现有椭圆的参数相差超过 distance_tolerance
                bool validEllipse = true;

                for (int j = 0; j < ellipses.size(); ++j) 
                {
                    double dist_center = sqrt(pow(ellipses[j].x - new_ellipse.x, 2) + pow(ellipses[j].y - new_ellipse.y, 2));
                    double dist_axes = sqrt(pow(ellipses[j].a - new_ellipse.a, 2) + pow(ellipses[j].b - new_ellipse.b, 2));
                    double angle_diff = std::abs(ellipses[j].phi - new_ellipse.phi);

                    if (dist_center <= distance_tolerance && dist_axes <= distance_tolerance && angle_diff < 0.314159265358979) 
                    {
                        validEllipse = false;
                        break;
                    }
                }

                if (validEllipse) 
                {
                    // 标记内点
                    for (int idx : inliers) {
                        labels[idx] = ellipses.size() + 1;
                    }

                    if (std::all_of(inliers3.begin(), inliers3.end(), [](bool val) { return val == true; })) 
                    {
                        for (int idx : inliers3) {
                            mylabels[idx] = ellipses.size() + 1;
                        }
                    }
                    
                    // 将新椭圆加入 ellipses
                    ellipses.push_back(new_ellipse);

                    // 标记当前候选圆已处理
                    validCandidates[i] = false;
                }
            }
        }
        else
        {
            // 其它情况，淘汰该候选圆
            validCandidates[i] = false;
        }
    }
}


// 计算圆的完整度
bool isComplete(const std::vector<Point2d>& x, const Point2d& center, int tbins, double angleCoverage, std::vector<int>& longest_inliers) {
    std::vector<double> theta;
    cart2pol(x, center, theta);  // 计算极角theta

    int tmin = -M_PI, tmax = M_PI;
    std::vector<int> tt(x.size());  // 存储每个点对应的bin索引

    for (size_t i = 0; i < theta.size(); ++i) {
        tt[i] = std::round((theta[i] - tmin) / (tmax - tmin) * tbins + 0.5);
        tt[i] = clamp(tt[i], 1, tbins);  // 确保tt[i]在[1, tbins]范围内
    }

    std::vector<int> h(tbins, 0);  // 每个bin内的点的数量
    for (int bin : tt) {
        ++h[bin - 1];
    }

    int longest_run = 0;
    int start_idx = 0;
    int end_idx = 0;

    // 寻找最长的连通区域
    while (start_idx < tbins) {
        if (h[start_idx] > 0) {  // 找到一个非零的bin
            end_idx = start_idx;
            while (end_idx < tbins && h[end_idx] > 0) {
                ++end_idx;
            }

            // 在这些bin区间中寻找内点
            std::vector<int> inliers;
            for (size_t i = 0; i < tt.size(); ++i) {
                if (tt[i] >= start_idx + 1 && tt[i] <= end_idx) {
                    inliers.push_back(i);
                }
            }

            double run = *std::max_element(theta.begin() + inliers.front(), theta.begin() + inliers.back()) -
                *std::min_element(theta.begin() + inliers.front(), theta.begin() + inliers.back());

            if (longest_run < run) {
                longest_run = run;
                longest_inliers = inliers;
            }
        }
        ++start_idx;
    }

    // 处理头尾相连的情况
    if (h[0] > 0 && h[tbins - 1] > 0) {
        start_idx = 0;
        while (start_idx < tbins && h[start_idx] > 0) {
            ++start_idx;
        }
        end_idx = tbins;
        while (end_idx > 1 && end_idx > start_idx && h[end_idx - 1] > 0) {
            --end_idx;
        }

        std::vector<int> inliers;
        for (size_t i = 0; i < tt.size(); ++i) {
            if (tt[i] <= start_idx || tt[i] >= end_idx) {
                inliers.push_back(i);
            }
        }

        double run = *std::max_element(theta.begin() + inliers.front(), theta.begin() + inliers.back()) + 2 * M_PI -
            *std::min_element(theta.begin() + inliers.front(), theta.begin() + inliers.back());

        if (longest_run < run) {
            longest_run = run;
            longest_inliers = inliers;
        }
    }

    double longest_run_deg = longest_run * 180.0 / M_PI;  // 将弧度转换为角度
    int h_greatthanzero_num = std::count_if(h.begin(), h.end(), [](int x) { return x > 0; });

    bool result = longest_run_deg >= angleCoverage || h_greatthanzero_num * (360.0 / tbins) >= std::min(360.0, 1.2 * angleCoverage);
    return result;
}

// 计算圆的完整度
double calcuCompleteness(const std::vector<Point2d>& x, const Point2d& center, int tbins) {
    std::vector<double> theta;
    cart2pol(x, center, theta);  // 计算极角theta

    int tmin = -M_PI, tmax = M_PI;
    std::vector<int> tt(x.size());  // 存储每个点对应的bin索引

    for (size_t i = 0; i < theta.size(); ++i) {
        tt[i] = std::round((theta[i] - tmin) / (tmax - tmin) * tbins + 0.5);
        tt[i] = clamp(tt[i], 1, tbins);  // 确保tt[i]在[1, tbins]范围内
    }

    std::vector<int> h(tbins, 0);  // 每个bin内的点的数量
    for (int bin : tt) {
        ++h[bin - 1];
    }

    int h_greatthanzero_num = std::count_if(h.begin(), h.end(), [](int x) { return x > 0; });
    double completeness = h_greatthanzero_num * (360.0 / tbins);
    return completeness;
}


// 计算笛卡尔坐标点到椭圆中心的方位角，并将点分配到相应的分区
void cart2pol(const std::vector<Point2d>& points, const Point2d& center, std::vector<double>& theta) {
    for (const auto& pt : points) {
        double x = pt.x - center.x;
        double y = pt.y - center.y;
        theta.push_back(atan2(y, x));  // atan2 返回 -pi 到 pi 之间的角度
    }
}

// 连通性分析，提纯椭圆周上的内点
std::vector<bool> takeInliers(const std::vector<Point2d>& x, const Point2d& center, int tbins) {
    std::vector<double> theta;
    cart2pol(x, center, theta);  // 计算极角theta

    int tmin = -M_PI, tmax = M_PI;
    std::vector<int> tt(x.size());  // 存储每个点对应的bin索引

    for (size_t i = 0; i < theta.size(); ++i) {
        tt[i] = std::round((theta[i] - tmin) / (tmax - tmin) * tbins + 0.5);
        tt[i] = clamp(tt[i], 1, tbins);  // 确保tt[i]在[1, tbins]范围内
    }

    std::vector<int> h(tbins, 0);  // 每个bin内的点的数量
    for (int bin : tt) {
        ++h[bin - 1];
    }

    std::vector<int> mark(tbins, 0);
    std::vector<int> compSize(tbins, 0);
    int nComps = 0;
    std::vector<int> queue(tbins, 0);
    std::vector<int> du = { -1, 1 };

    // 遍历每个分区，分析连通区域
    for (int i = 0; i < tbins; ++i) 
    {
        if (h[i] > 0 && mark[i] == 0) 
        {  // 找到未标记且有内点的分区
            ++nComps;
            mark[i] = nComps;  // 标记为第nComps个连通区域
            int front = 0, rear = 0;
            queue[front] = i;
            while (front <= rear) 
            {
                int u = queue[front];
                ++front;
                for (int j = 0; j < 2; ++j) 
                {
                    int v = u + du[j];
                    if (v == -1) v = tbins - 1;
                    if (v > tbins - 1) v = 0;
                    if (mark[v] == 0 && h[v] > 0) 
                    {
                        ++rear;
                        queue[rear] = v;
                        mark[v] = nComps;
                    }
                }
            }

            // 计算连通区域的内点数量
            compSize[nComps - 1] = std::count_if(tt.begin(), tt.end(), [nComps, &mark](int val) {
                return mark[val - 1] == nComps;
                });
        }
    }

    // 找到有效的连通区域（大于最大连通区域的10%且数量大于10）
    int maxCompSize = *std::max_element(compSize.begin(), compSize.end());
    std::vector<int> validComps;
    for (int i = 0; i < nComps; ++i) 
    {
        if (compSize[i] >= maxCompSize * 0.1 && compSize[i] > 10) 
        {
            validComps.push_back(i + 1);  // 存储有效区域的编号
        }
    }

    // 标记有效的内点
    std::vector<bool> idx(x.size(), false);
    for (size_t i = 0; i < tt.size(); ++i) 
    {
        if (std::find(validComps.begin(), validComps.end(), mark[tt[i] - 1]) != validComps.end()) 
        {
            idx[i] = true;
        }
    }
    return idx;
}

// 计算椭圆上每个点的法向量（已标准化）
std::vector<Point2d> computePointAngle(const Ellipse& ellipse, const std::vector<Point2d>& points) {
    double a_square = ellipse.a * ellipse.a;
    double b_square = ellipse.b * ellipse.b;
    double sin_phi = sin(ellipse.phi);
    double cos_phi = cos(ellipse.phi);
    double sin_square = sin_phi * sin_phi;
    double cos_square = cos_phi * cos_phi;

    // 椭圆参数转换为标准二次方程系数
    double A = b_square * cos_square + a_square * sin_square;
    double B = (b_square - a_square) * 2 * sin_phi * cos_phi;
    double C = b_square * sin_square + a_square * cos_square;
    double D = -2 * A * ellipse.x - B * ellipse.y;
    double E = -2 * C * ellipse.y - B * ellipse.x;

    std::vector<Point2d> normals;
    for (const auto& pt : points) {
        double x = pt.x;
        double y = pt.y;
        // 计算点到椭圆的法向量角度
        double angle = atan2(C * y + B / 2 * x + E / 2, A * x + B / 2 * y + D / 2);
        normals.push_back({ cos(angle), sin(angle) });
    }
    return normals;
}

// 计算Rosin距离的平方
std::vector<double> dRosin_square(const Ellipse& param, const std::vector<Point2d>& points) 
{
    double ae2 = param.a * param.a;  // a^2
    double be2 = param.b * param.b;  // b^2
    double fe2 = ae2 - be2;
    double cos_phi = cos(-param.phi);
    double sin_phi = sin(-param.phi);

    std::vector<double> dmin;

    for (const auto& pt : points) {
        double x = pt.x - param.x;  // x - x0
        double y = pt.y - param.y;  // y - y0

        // 旋转坐标
        double xp = x * cos_phi - y * sin_phi;
        double yp = x * sin_phi + y * cos_phi;

        // 计算中间变量
        double X = xp * xp;
        double Y = yp * yp;
        double delta = (X + Y + fe2) * (X + Y + fe2) - 4 * fe2 * X;
        double A = (X + Y + fe2 - sqrt(delta)) / 2;
        double ah = sqrt(A);
        double bh2 = fe2 - A;

        // 计算各项值
        double term = A * be2 + ae2 * bh2;
        double xi = ah * sqrt(ae2 * (be2 + bh2) / term);
        double yi = param.b * sqrt(bh2 * (ae2 - A) / term);

        // 计算距离的平方
        std::vector<double> d(4);
        d[0] = (xp - xi) * (xp - xi) + (yp - yi) * (yp - yi);
        d[1] = (xp - xi) * (xp - xi) + (yp + yi) * (yp + yi);
        d[2] = (xp + xi) * (xp + xi) + (yp - yi) * (yp - yi);
        d[3] = (xp + xi) * (xp + xi) + (yp + yi) * (yp + yi);

        // 获取最小值
        dmin.push_back(*std::min_element(d.begin(), d.end()));
    }

    return dmin;
     // 将输入的点转换为矩阵形式
    //size_t n_points = points.size();
    //Eigen::MatrixXd points_matrix(n_points, 2); // 每一行是一个点 (x, y)

    //for (size_t i = 0; i < n_points; ++i) {
    //    points_matrix(i, 0) = points[i].x;
    //    points_matrix(i, 1) = points[i].y;
    //}

    //double ae2 = param.a * param.a;  // a^2
    //double be2 = param.b * param.b;  // b^2
    //double fe2 = ae2 - be2;
    //double cos_phi = cos(-param.phi);
    //double sin_phi = sin(-param.phi);

    //// 将旋转矩阵应用于所有点
    //Eigen::MatrixXd rotation_matrix(2, 2);
    //rotation_matrix << cos_phi, -sin_phi,
    //    sin_phi, cos_phi;

    //Eigen::MatrixXd rotated_points = points_matrix.rowwise() - Eigen::RowVector2d(param.x, param.y);  // 移动到原点
    //rotated_points = rotated_points * rotation_matrix.transpose();  // 应用旋转矩阵

    //// 计算中间变量
    //Eigen::MatrixXd X = rotated_points.col(0).array().square();
    //Eigen::MatrixXd Y = rotated_points.col(1).array().square();
    //Eigen::MatrixXd XY_sum = X + Y;

    //// 将 fe2 和 be2 扩展为与 XY_sum 相同的大小
    //Eigen::MatrixXd fe2_matrix = Eigen::MatrixXd::Constant(XY_sum.rows(), XY_sum.cols(), fe2);
    //Eigen::MatrixXd be2_matrix = Eigen::MatrixXd::Constant(XY_sum.rows(), XY_sum.cols(), be2);

    //// 计算 delta
    //Eigen::MatrixXd delta = (XY_sum + fe2_matrix).array().square() - 4 * fe2 * X.array();

    //// 计算 A 和 bh2
    //Eigen::MatrixXd A = (XY_sum + fe2_matrix).array() - delta.array().sqrt();
    //A /= 2;
    //Eigen::MatrixXd ah = A.array().sqrt();
    //Eigen::MatrixXd bh2 = fe2_matrix - A;

    //// 将 ae2 扩展为与 A 相同大小的矩阵
    //Eigen::MatrixXd ae2_matrix = Eigen::MatrixXd::Constant(XY_sum.rows(), XY_sum.cols(), ae2);

    //// 按元素乘法：按元素乘法时，使用 .array() 运算符
    //Eigen::MatrixXd term = A.array() * be2_matrix.array() + ae2_matrix.array() * bh2.array();

    //// 计算 xi 和 yi
    //Eigen::MatrixXd xi = ah.array() * (ae2_matrix.array() * (be2_matrix.array() + bh2.array()) / term.array()).sqrt();
    //Eigen::MatrixXd yi = param.b * (bh2.array() * (ae2_matrix.array() - A.array()) / term.array()).sqrt();

    //// 计算四个距离的平方
    //Eigen::MatrixXd d1 = (rotated_points.col(0).array() - xi.array()).square() + (rotated_points.col(1).array() - yi.array()).square();
    //Eigen::MatrixXd d2 = (rotated_points.col(0).array() - xi.array()).square() + (rotated_points.col(1).array() + yi.array()).square();
    //Eigen::MatrixXd d3 = (rotated_points.col(0).array() + xi.array()).square() + (rotated_points.col(1).array() - yi.array()).square();
    //Eigen::MatrixXd d4 = (rotated_points.col(0).array() + xi.array()).square() + (rotated_points.col(1).array() + yi.array()).square();

    //// 将每个点的四个距离平方组合成一个矩阵
    //Eigen::MatrixXd d_matrix(n_points, 4);
    //d_matrix.col(0) = d1.col(0);
    //d_matrix.col(1) = d2.col(0);
    //d_matrix.col(2) = d3.col(0);
    //d_matrix.col(3) = d4.col(0);

    //// 计算每个点的最小距离平方
    //Eigen::MatrixXd dmin = d_matrix.rowwise().minCoeff();  // 按列计算最小值

    //std::vector<double> result(dmin.data(), dmin.data() + dmin.size());
    //return result;
}

double dRosin_square(const Ellipse& candidate, const Point& pt)
{
    /*std::vector<Point2d> points;
    points.push_back(point);
    return dRosin_square(candidate, points).at(0);*/
    double ae2 = candidate.a * candidate.a;  // a^2
    double be2 = candidate.b * candidate.b;  // b^2
    double fe2 = ae2 - be2;
    double cos_phi = cos(-candidate.phi);
    double sin_phi = sin(-candidate.phi);

    std::vector<double> dmin;


    double x = pt.x - candidate.x;  // x - x0
    double y = pt.y - candidate.y;  // y - y0

    // 旋转坐标
    double xp = x * cos_phi - y * sin_phi;
    double yp = x * sin_phi + y * cos_phi;

    // 计算中间变量
    double X = xp * xp;
    double Y = yp * yp;
    double delta = (X + Y + fe2) * (X + Y + fe2) - 4 * fe2 * X;
    double A = (X + Y + fe2 - sqrt(delta)) / 2;
    double ah = sqrt(A);
    double bh2 = fe2 - A;

    // 计算各项值
    double term = A * be2 + ae2 * bh2;
    double xi = ah * sqrt(ae2 * (be2 + bh2) / term);
    double yi = candidate.b * sqrt(bh2 * (ae2 - A) / term);

    // 计算距离的平方
    std::vector<double> d(4);
    d[0] = (xp - xi) * (xp - xi) + (yp - yi) * (yp - yi);
    d[1] = (xp - xi) * (xp - xi) + (yp + yi) * (yp + yi);
    d[2] = (xp + xi) * (xp + xi) + (yp - yi) * (yp - yi);
    d[3] = (xp + xi) * (xp + xi) + (yp + yi) * (yp + yi);

    // 获取最小值
    return *std::min_element(d.begin(), d.end());
    

}

Ellipse fitEllipse(const vector<double>& X, const vector<double>& Y, int& info) 
{
    // 将输入向量转化为 Eigen 向量（列向量形式）
    int n = X.size();
    Eigen::VectorXd x(n), y(n);
    for (int i = 0; i < n; ++i) 
    {
        x(i) = X[i];
        y(i) = Y[i];
    }

    // 构造设计矩阵 D
    Eigen::MatrixXd D(n, 6);
    for (int i = 0; i < n; ++i) 
    {
        D(i, 0) = x(i) * x(i);  // x^2
        D(i, 1) = x(i) * y(i);  // xy
        D(i, 2) = y(i) * y(i);  // y^2
        D(i, 3) = x(i);         // x
        D(i, 4) = y(i);         // y
        D(i, 5) = 1;            // 1
    }

    // 计算矩阵 S = D' * D
    Eigen::MatrixXd S = D.transpose() * D;

    // 构建约束矩阵 C
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6);
    C(0, 2) = 2;
    C(1, 1) = -1;
    C(2, 0) = 2;

    // 求解广义特征值问题
    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> solver(S, C);

    // 获取特征值和特征向量
    Eigen::VectorXd realPart = solver.alphas().real();
    Eigen::VectorXd imagPart = solver.alphas().imag();
    Eigen::VectorXd beta = solver.betas();
    Eigen::MatrixXd vr = solver.eigenvectors().real();

    //int info = solver.info();
    double realPartValue;

    int index = -1;
    for (int i = 0; i < realPart.size(); i++)
    {
        realPartValue = realPart[i];
        if (beta[i] < 0)
        {
            realPartValue = -realPartValue;
        }
        if (realPartValue >= -1e-8 )
        {
            index = i;
            break;
        }
    }

    // 如果找到合适的特征值，返回特征向量作为拟合参数
    if (index != -1)
    {
        Eigen::VectorXd coeff = vr.col(index);
        if (coeff[0] < 0)
        {
            coeff = -coeff; // 规范化正负号
        }

        // 转换到 ellipara 输出格式
        double beta[6];
        // 将结果写入输出参数 ellicoeff
        for (int i = 0; i < 6; i++) {
            beta[i] = coeff[i];
        }
        double ellipara[6];
        ellipse2Param(beta, ellipara);//ax^2 + bxy + cy^2 + dx + ey + f = 0, transform to (x0,y0,a,b,phi)
        info = 1;
        Ellipse ellipse = { ellipara[0], ellipara[1], ellipara[2], ellipara[3], ellipara[4] };
        return ellipse;
    }
    else
    {

        info = 0;
        return Ellipse();
            
    }
}
void findMaxBoundingRect(cv::Mat image, cv::Rect& max_rect)
{
    
    // 将图像从BGR转换为HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // 设置黄色区域的HSV范围
    cv::Scalar lower_yellow(20, 100, 100); // 黄色的最小阈值
    cv::Scalar upper_yellow(30, 255, 255); // 黄色的最大阈值

    // 创建掩膜
    cv::Mat mask;
    cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);

    // 查找轮廓
    std::vector<Mat> contours(1000);
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 6. 找到最大外接矩形
    
    double max_area = 0;

    for (size_t i = 0; i < contours.size(); i++) {
        // 计算轮廓的外接矩形
        cv::Rect bounding_rect = cv::boundingRect(contours[i]);
        double area = bounding_rect.area();  // 获取矩形的面积

        // 如果当前矩形的面积比最大面积大，更新最大矩形
        if (area > max_area) {
            max_area = area;
            max_rect = bounding_rect;
        }
    }

    // 7. 在图像中绘制最大外接矩形
  /*  cv::Mat result_image = image.clone();
    if (max_area > 0) {
         绘制矩形，颜色为红色，厚度为2
        cv::rectangle(result_image, max_rect, cv::Scalar(0, 0, 255), 2);
    }*/

    // 显示结果
   // cv::imshow("原图", image);
    //cv::imshow("黄色区域掩膜", mask);
   // cv::imshow("最大外接矩形标记", result_image);

    //// 等待用户按键
    //cv::waitKey(0);
    //return max_rect;
}



void findMaxBoundingRect()
{

    cv::Mat image  = imread("D:/code/c++/double_camera/demo/move_day_0_4.0-0.5/1L.png", cv::IMREAD_COLOR);
        cv::Rect max_rect;

    // 将图像从BGR转换为HSV颜色空间
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // 设置黄色区域的HSV范围
    cv::Scalar lower_yellow(20, 100, 100); // 黄色的最小阈值
    cv::Scalar upper_yellow(30, 255, 255); // 黄色的最大阈值

    // 创建掩膜
    cv::Mat mask;
    cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);

    //// 查找轮廓
    std::vector<std::vector<cv::Point>> contours(20, std::vector<cv::Point>(300, {0, 0}));
    //std::vector<std::vector<cv::Point>> contours;
    //std::vector<Mat> contours(100);
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

    //// 6. 找到最大外接矩形

    double max_area = 0;

    //for (size_t i = 0; i < contours.size(); i++) {
    //    // 计算轮廓的外接矩形
    //    cv::Rect bounding_rect = cv::boundingRect(contours[i]);
    //    double area = bounding_rect.area();  // 获取矩形的面积

    //    // 如果当前矩形的面积比最大面积大，更新最大矩形
    //    if (area > max_area) {
    //        max_area = area;
    //        max_rect = bounding_rect;
    //    }
    //}
    //std::vector<std::vector<cv::Point>>().swap(contours);
    // 添加数据
    //contours.clear();  // 清空内容
    //contours.shrink_to_fit(); // 尝试释放多余的内存
    /*for (auto c: contours)
    {
        delete &c;
    }
    delete& contours;*/
}
vector<Ellipse> getEllipses(cv::Mat image, vector<ours::Ellipse> maxMin)
{

    Rect max_rect;
    clock_t start = clock();
    findMaxBoundingRect(image, max_rect);
    //clock_t t1 = clock();
   // cout << "find area运行时间:" << (double)(t1 - start) / CLOCKS_PER_SEC << endl;
    //findMaxBoundingRect();
    // 

    // 将矩形范围外的像素置为黑色
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            // 检查当前像素是否在矩形范围外
            if (!max_rect.contains(cv::Point(x, y))) {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // 将该像素置为黑色
            }
        }
    }
    // 创建一个Mat对象来存储灰度图像  
    cv::Mat grayImage;

    // 使用cv::cvtColor函数将彩色图像转换为灰度图像  
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    cv::Mat imageEdge;
    cv::Mat ls_mat;
    vector<Point2d> normals;

    int edge_process_select = 2;
    int specified_polarity = 0;
    vector<Ellipse> candidates;
    vector<Point2d> points;
    mexFunction1(grayImage, edge_process_select, specified_polarity, candidates, imageEdge, normals, ls_mat, points);
    /*imshow("ls_mat", ls_mat);
    waitKey(0);*/
    //clock_t t2 = clock();
    //cout << "get candidates运行时间:" << (double)(t2 - t1) / CLOCKS_PER_SEC << endl;
    //cout << candidates.size() << endl;
    // 根据卫星位置筛选候选框
    vector<Ellipse> candidatesFilter;
    for (Ellipse c: candidates)
    {   
        // 根据前一帧检测到的椭圆对候选椭圆进行筛选
        if (maxMin.size() == 2)
        {
            if ((abs(c.x - maxMin.at(0).x) < 8 && abs(c.y - maxMin.at(0).y) < 8 && abs(c.a - maxMin.at(0).a) < 15 && abs(c.b - maxMin.at(0).b) < 15) ||
                (abs(c.x - maxMin.at(1).x) < 8 && abs(c.y - maxMin.at(1).y) < 8 && abs(c.a - maxMin.at(1).a) < 15 && abs(c.b - maxMin.at(1).b) < 15))
            {
                    candidatesFilter.push_back(c);                          
            }
            /*else if(maxMin.size() == 1 && (abs(c.x - maxMin.at(0).x) < 10 && abs(c.y - maxMin.at(0).y) < 10 && abs(c.a - maxMin.at(0).a) < 10 && abs(c.b - maxMin.at(0).b) < 10))
            {
                candidatesFilter.push_back(c);
            }*/
        }
        else if ((max_rect.x <= (c.x - c.a) &&
            max_rect.x + max_rect.width >= (c.x + c.a) &&
            max_rect.y <= (c.y - c.a) &&
            max_rect.y + max_rect.height >= (c.y + c.a)) && c.b / c.a > 0.9  && ((c.a >= max_rect.width * 0.25 && c.a <= max_rect.width * 0.36) || (c.a >= max_rect.width * 0.05 && c.a <= max_rect.width * 0.13)))
        {
            candidatesFilter.push_back(c);
        }
    }
    // 异常处理，如果全被筛除了，就不进行筛选了
    if (candidatesFilter.size() == 0)
    {
        candidatesFilter = candidates;
    }
    vector<Point2d> normalsFilter;
    vector<Point2d> pointsFilter;
    for (int i = 0; i < points.size(); i++)
    {
        Point2d point = points.at(i);
        // 根据前一帧检测到的椭圆对点进行筛选
        if (maxMin.size() == 2)
        {
            if (point.x >= (maxMin.at(0).x - maxMin.at(0).a - 5 - 1) &&
                point.x <= (maxMin.at(0).x + maxMin.at(0).a + 5 + 1) &&
                point.y >= (maxMin.at(0).y - maxMin.at(0).a - 5 - 1) &&
                point.y <= (maxMin.at(0).y + maxMin.at(0).a + 5 + 1))
            {
                pointsFilter.push_back(point);
                normalsFilter.push_back(normals.at(i));
            }
        }
        else if ((max_rect.x <= point.x &&
            max_rect.x + max_rect.width >= point.x &&
            max_rect.y <= point.y &&
            max_rect.y + max_rect.height >= point.y))
        {
            pointsFilter.push_back(point);
            normalsFilter.push_back(normals.at(i));
        }
    }
    //clock_t t3 = clock();
    //cout << "points filter运行时间:" << (double)(t3 - t2) / CLOCKS_PER_SEC << endl;
    // 根据卫星位置筛选边缘点及梯度向量
    // 
    //for (int i = 0; i < candidatesFilter.size(); i++)
    //{
    //    ellipse(image, cv::Point((int)candidatesFilter.at(i).x, (int)candidatesFilter.at(i).y), cv::Size(candidatesFilter.at(i).a, candidatesFilter.at(i).b), candidatesFilter.at(i).phi * 180 / M_PI, 0, 360, (Scalar(0, 0, 255)), 1);
    //    //ellipse(show, cv::Point((int)ellipses.at(i).x, (int)ellipses.at(i).y), cv::Size(ellipses.at(i).a, ellipses.at(i).b), ellipses.at(i).phi * 180 / M_PI, 0, 360, (Scalar(255, 0, 0)), 1);
    //}

    //imshow("imageEdge", imageEdge);
    //waitKey(0);
    //imshow("image", image);
    //waitKey(0);
    //vector<Point> pointsI; // Initialize with edge points
    vector<Ellipse> ellipses;
    //cout << "ellipses" << ellipses.size() << endl;
    //Mat points1;
    //findNonZero(imageEdge, pointsI);

    //vector<Point2d> points(pointsI.begin(), pointsI.end());
    //int n = points.size();
    //std::vector<int> mylabels(n, 0); // 测试用
    //std::vector<int> labels(n, 0); // 边缘像素点的总数量n, n x 1

    //ellipseDetection(candidates, points, normals, 2, M_PI / 9, 0.6, 165, labels, mylabels, ellipses);

    int n = pointsFilter.size();
    std::vector<int> mylabels(n, 0); // 测试用
    std::vector<int> labels(n, 0); // 边缘像素点的总数量n, n x 1

    ellipseDetection(candidatesFilter, pointsFilter, normalsFilter, 2, M_PI / 9, 0.6, 165, labels, mylabels, ellipses);
    //clock_t t4 = clock();
    //cout << "ellipses filter运行时间:" << (double)(t4 - t3) / CLOCKS_PER_SEC << endl;
    /*for (ours::Ellipse e : ellipses)
    {
        cout << e.x << " " << e.y << " " << e.a << " " << e.b << " " << e.phi << endl;
    }
    cout << "ellipses" << ellipses.size() << endl;*/


    //for (int i = 0; i < ellipses.size(); i++)
    //{
    //    ellipse(image, cv::Point((int)ellipses.at(i).x, (int)ellipses.at(i).y), cv::Size(ellipses.at(i).a, ellipses.at(i).b), ellipses.at(i).phi * 180 / M_PI, 0, 360, (Scalar(0, 0, 255)), 1);
    //    cv::circle(image, cv::Point((int)ellipses.at(i).x, (int)ellipses.at(i).y), 5, cv::Scalar(0, 0, 255), -1);
    //    //ellipse(show, cv::Point((int)ellipses.at(i).x, (int)ellipses.at(i).y), cv::Size(ellipses.at(i).a, ellipses.at(i).b), ellipses.at(i).phi * 180 / M_PI, 0, 360, (Scalar(255, 0, 0)), 1);
    //}
    //cv::imwrite("281L.jpg", image);
    //imshow("image", image);
    //waitKey(0);
    return ellipses;
}


