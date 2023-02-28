vector<Point2f> getTxt(){
    fstream file("/home/yc/Desktop/xy1.txt");

    float x,y;
    vector<Point2f> vec;
    while (!file.eof()){
        file>>x>>y;
        vec.emplace_back(x,y);
    }

    return vec;
}

bool cmp1(Point2f &p1, Point2f &p2) {
    if (p1.x == p2.x) return p1.y < p2.y;
    return p1.x < p2.x;
}

float peak(Point2f *p1, Point2f *p2, Point2f *p3) {
    auto area = p1->x * p2->y + p3->x * p1->y + p2->x * p3->y -
                p3->x * p2->y - p2->x * p1->y - p1->x * p3->y;
    return area;
}

//算法1
//浮点坐标三点共线,叉乘面积1e-9,但是不等于0. 需要用这个函数判断是否共线
bool epsilon(Point2f* left,Point2f* right,Point2f*p){
    return p == left || p == right;
}

void hull(vector<Point2f*> &Points, vector<Point2f *> &stack, int sign) {

    if (Points.size() <= 2) {
        for (auto& p: Points) {
            stack.emplace_back(p);
        }
        return;
    }

    int pmaxy = -1;
    int pminy = -1;
    auto left = Points.front();
    auto right = Points.back();

    float maxArea{FLT_MIN}, minArea{FLT_MAX};
    for (int i = 0; i < Points.size(); ++i) {
        auto area = peak(left, right, Points[i]);
        if (area > maxArea && area >= 0) {
            pmaxy = i;
            maxArea = area;
        }
        if (area < minArea && area <= 0) {
            pminy = i;
            minArea = area;
        }
    }

    vector<Point2f *> points1,points2,points3,points4;

    for (int i = 0; i < Points.size(); ++i) {
        if (pmaxy >= 0){
            auto area1 = peak(left, Points[pmaxy], Points[i]);
            auto area2 = peak(Points[pmaxy], right, Points[i]);

            if (area1 > 0 || epsilon(left, Points[pmaxy], Points[i])) points1.emplace_back(Points[i]);
            if (area2 > 0 || epsilon(Points[pmaxy], right, Points[i])) points2.emplace_back(Points[i]);
        }

        if (pminy >= 0){
            auto area3 = peak(left, Points[pminy], Points[i]);
            auto area4 = peak(Points[pminy], right, Points[i]);

            if (area3 <0 || epsilon(left, Points[pminy], Points[i])) points3.emplace_back(Points[i]);
            if (area4 <0 || epsilon(Points[pminy], right, Points[i])) points4.emplace_back(Points[i]);
        }
    }

    //第一次需要递归左上 右上 左下 右下四个方向切出来的候选点
    if (sign == 0){
        hull(points1, stack, 1);
        hull(points2, stack, 1);
        hull(points3, stack, -1);
        hull(points4, stack, -1);
    }

    //切出来点集只需要再切两次
    if (sign == 1){
        hull(points1, stack, 1);
        hull(points2, stack, 1);
    }

    if(sign == -1){
        hull(points3, stack, -1);
        hull(points4, stack, -1);
    }
}

//算法2--只给边界点,遍历所有点找到与边界构成三角面积最大的点,递归
void hull(Point2f **arrays, int len, Point2f *left, Point2f *right, int *stack, int sign) {

    Point2f *maxPt = left;//裸指针一定要初始化,否则有可能返回空指针
    float maxArea = FLT_MIN;
    for (int i = 0; i < len; ++i) {
        auto area = peak(left, right, arrays[i]);
        area *= (float) sign;
        if (area > 0 && area > maxArea) {
            maxArea = area;
            maxPt = arrays[i];
        }
    }

    if (maxPt == left || maxPt == right) {
        stack[left - arrays[0]] = 1;
        stack[right - arrays[0]] = 1;
        return;
    }

    hull(arrays, len, left, maxPt, stack, sign);
    hull(arrays, len, maxPt, right, stack, sign);
}

void convexHull(vector<Point2f> &points) {
    int total = (int) points.size();
    vector<Point2f *> arrays(total);
    vector<int> stack_(total, 0);
    int *stack = &stack_[0];

    for (int i = 0; i < total; ++i) {
        arrays[i] = &points[i];
    }

    Point2f *left = arrays[0];
    Point2f *right = arrays.back();

    int pmaxy = 0;
    int pminy = 0;

    float maxArea, minArea = peak(left, right, left);
    maxArea = minArea;
    for (int i = 0; i < total; ++i) {
        auto area = peak(left, right, arrays[i]);
        if (area > maxArea) {
            pmaxy = i;
            maxArea = area;
        }
        if (area < minArea) {
            pminy = i;
            minArea = area;
        }
    }

    hull(&arrays[0], total, left, arrays[pmaxy], stack, 1);
    hull(&arrays[0], total, arrays[pmaxy], right, stack, 1);

    hull(&arrays[0], total, left, arrays[pminy], stack, -1);
    hull(&arrays[0], total, arrays[pminy], right, stack, -1);

//    vector<Point2f> vec_;
//    for (int i = 0; i < total; ++i) {
//        if (stack[i] == 1)vec_.emplace_back(arrays[i][0]);
//    }
//
//    vector<Point2f>vec;
//    cv::convexHull(vec_,vec);
//
//    //visualize
//    {
//
//        Mat src((int) vec.size(), 2, CV_32F, &vec[0]);
//        Mat dst(300, 300, 0,Scalar(0));
//        normalize(src, src, 0, 255, NORM_MINMAX);
//        src.convertTo(src, CV_32S);
//
//        Point last = Point(src.at<int>(0, 0), src.at<int>(0, 1));
//        for (int i = 0; i < vec.size(); ++i) {
//            auto p = Point(src.at<int>(i, 0), src.at<int>(i, 1));
//            line(dst, last, p, 255);
//            last = p;
//        }
//        namedWindow("dst0", 2);
//        imshow("dst0", dst);
//        waitKey();
//    }
}

//对结果进行重新排序:分上下两部分点集,上半部分从左到右存点,下半部分逆时针方向存进去
void order_border(std::vector<Point2f *> &points) {
    vector<Point2f *> convHullLines_top, convHullLines_bottom;
    auto delim_start = points.front();
    auto delim_end = points.back();

    for (int i = 0; i < points.size(); ++i) {
        if (i > 0 && points[i][0] == *delim_start) continue;//初始点只需要放进排列一次
        if (peak(delim_start, delim_end, points[i]) > 0) convHullLines_top.emplace_back(points[i]);
        if (peak(delim_start, delim_end, points[i]) < 0) convHullLines_bottom.emplace_back(points[i]);
    }

    auto it = convHullLines_bottom.rbegin();
    while (it != convHullLines_bottom.rend()) {
        convHullLines_top.emplace_back(*it);
        it++;
    }

    swap(points, convHullLines_top);
    points.pop_back();
}

int main(){
  auto pts = getTxt();
  sort(pts.begin(),pts.end(),cmp1);
  
  //凸包算法1的接口
  convexHull(pts);

  //凸包算法2的接口
  vector<Point2f *> points, stack;
  for (auto& p: pts) {
      points.emplace_back(&p);
  }
  hull(points, stack, 0);
  order_border(stack);
}
