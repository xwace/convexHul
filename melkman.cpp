//可视化
void visualize(vector<Point2f> pts) {
    Mat src((int) pts.size(), 2, CV_32FC1, &pts[0]);
    Mat dst(300, 300, 0, Scalar(0));
    normalize(src, src, 0, 255, NORM_MINMAX);
    src.convertTo(src, CV_32S);

    Point last = Point(src.at<int>(0, 0), src.at<int>(0, 1));
    for (int i = 0; i < pts.size(); ++i) {
        auto p = Point(src.at<int>(i, 0), src.at<int>(i, 1));
        line(dst, last, p, 255);
        last = p;
    }

    namedWindow("dst0", 2);
    imshow("dst0", dst);
    waitKey();
}

//用双指针模拟双端队列实现
void melkman(vector<Point2f> &points) {

    int dbot{(int) (points.size() - 2)}, dtop = dbot + 3;
    int pnext = 3;
    vector<int> stack_(2 * points.size() + 1);//注意!!!空间分配不够会动态数组赋值越界,invalid free next size
    Point2f *buf = &points[0];
    int *deq = &stack_[0];//!!注意是从0开始,bot在0+n-2处

    //初始化前三个点,确定是v2,v0,v1,v2 或者 v2,v1,v0,v2
    if (peak(&buf[0], &buf[1], &buf[2]) > 0) {
        deq[dbot] = 2;
        deq[dbot + 1] = 0;
        deq[dbot + 2] = 1;
        deq[dtop] = 2;
    } else {
        deq[dbot] = 2;
        deq[dbot + 1] = 1;
        deq[dbot + 2] = 0;
        deq[dtop] = 2;
    }


    while (pnext != points.size()) {

        if (peak(&buf[deq[dbot]], &buf[deq[dbot + 1]], &buf[pnext]) > 0 and
            peak(&buf[deq[dtop - 1]], &buf[deq[dtop]], &buf[pnext]) > 0) {
            pnext++;
            continue;
        }

      //弹出前面元素直到前两个与pnext构成左拐
        while (peak(&buf[deq[dbot]], &buf[deq[dbot + 1]], &buf[pnext]) < 0) {
            ++dbot;
        }
        deq[--dbot] = pnext;

      //弹出最后两个元素直到与pnext构成左拐
        while (peak(&buf[deq[dtop - 1]], &buf[deq[dtop]], &buf[pnext]) < 0) {
            --dtop;
        }
        deq[++dtop] = pnext;
        pnext++;
    }


    vector<Point2f> pts;
    for (int i = 0; i < dtop - dbot; ++i) {
        pts.emplace_back(points[deq[dbot + i]]);
    }

    //可视化
    visualize(pts);
}

//双端队列实现
void melkman_d(vector<Point2f> &points) {
    deque<Point2f*> D;
    D.emplace_back(&points[2]);

    //初始化前三个点,确定是v2,v0,v1,v2 或者 v2,v1,v0,v2
    if (peak(&points[0], &points[1], &points[2]) > 0) {
        D.emplace_back(&points[0]);
        D.emplace_back(&points[1]);
        D.emplace_back(&points[2]);

    } else {
        D.emplace_back(&points[1]);
        D.emplace_back(&points[0]);
        D.emplace_back(&points[2]);
    }

    for (int i = 3; i < points.size(); i++) {
        if (peak(D[0], D[1], &points[i]) > 0 and
            peak(*(D.end() - 2), D.back(), &points[i]) > 0) {
            continue;
        }

        while (peak(D[0], D[1], &points[i]) < 0) {
            D.pop_front();
        }
        D.emplace_front(&points[i]);

        while (peak(*(D.end() - 2), D.back(), &points[i]) < 0) {
            D.pop_back();
        }
        D.emplace_back(&points[i]);
    }
}
