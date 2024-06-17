#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CDRT.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;


// 回调函数，当鼠标左键按下时调用
int32_t updata = false, goalUpdata = false;
BIpoint mousePoint = {-1, -1}, goalPoint = {-1, -1};

void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
    {
        updata = true;
        mousePoint.x = x;
        mousePoint.y = y;
        // 在控制台输出鼠标左击位置的坐标
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    if (event == cv::EVENT_RBUTTONDOWN)
    {
        goalUpdata = true;
        goalPoint.x = x;
        goalPoint.y = y;
    }
}

#define speed 100.0
int main()
{
    CTrtrt_DynamicObstacle obs1, obs2, obs3, obs4;
    obs1.x = 250;
    obs1.y = 250;
    obs1.angle = 0;
    obs1.DObsType = 0;
    obs1.radius = 40;
    obs1.sampleRadius = 500;
    obs1.OutLine.push_back({0, 0});

    obs2.x = 500;
    obs2.y = 300;
    obs2.angle = 0;
    obs2.DObsType = 1;
    // obs2.radius = 20;
    obs2.sampleRadius = 500;
    obs2.OutLine.push_back({40, 0});
    obs2.OutLine.push_back({-40, 30});
    obs2.OutLine.push_back({-40, -30});

    obs3.x = 500;
    obs3.y = 1500;
    obs3.angle = 0;
    obs3.DObsType = 0;
    obs3.radius = 50;
    obs3.sampleRadius = 500;
    obs3.OutLine.push_back({0, 0});

    obs4.x = 1500;
    obs4.y = 500;
    obs4.angle = 0;
    obs4.DObsType = 1;
    obs4.radius = 20;
    obs4.sampleRadius = 500;
    obs4.OutLine.push_back({-30, 30});
    obs4.OutLine.push_back({30, 30});
    obs4.OutLine.push_back({30, -30});
    obs4.OutLine.push_back({-30, -30});

    BImap cemap;

    // 创建窗口并注册鼠标回调函数
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1400);
    cv::setMouseCallback("Video", onMouse, NULL);
    cemap.MaptoBInavi("/home/tzyr/WorkSpace/CDRT2/map/dogmap.png", 0.1, 0.8);
    cemap.DrawBImap(true);
    cv::imshow("Video", cemap.BIdmap);
    cv::waitKey(0);

    cemap.RTRT_AddDynamicObstacle(1, obs1, 3);
    cemap.RTRT_AddDynamicObstacle(2, obs2, 3);
    cemap.RTRT_AddDynamicObstacle(3, obs3, 3);
    cemap.RTRT_AddDynamicObstacle(4, obs4, 3);

    int64_t t0 = utime_ns();
    BIpoint S = {800, 300};
    cemap.RTRT_SetAgent(S);
    cemap.RTRT_start(20);//以20像素离散化

    int keymod = 0;
    int key = 0;
    int runkey = 1;

    while (runkey)
    {
        int64_t tt0 = utime_ns();

        cemap.RTRT_task();
        cemap.GetPath();
        cv::Mat BIdmap = cemap.BIdmap.clone();
        drawDObstacles(cemap.DObstacles, BIdmap);
        for (const auto &pair : cemap.Node_Map)
        {
            if (pair.second.invM == originIndexFun())
                continue;
            cv::Point startPoint((int)pair.first.x, (int)pair.first.y);
            cv::Point endPoint((int)pair.second.par.x, (int)pair.second.par.y);
            if (pair.second.cost.collision)
                cv::line(BIdmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 1);
            else
                cv::line(BIdmap, startPoint, endPoint, cv::Scalar(255, 0, 0), 1);
        }
        if (cemap.agent_path.size() >= 2)
        {
            BIpoint point0 = cemap.agent_path.front();
            for (BIpoint point1 : cemap.agent_path)
            {
                cv::Point startPoint((int)point0.x, (int)point0.y);
                cv::Point endPoint((int)point1.x, (int)point1.y);
                cv::line(BIdmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 3);
                point0 = point1;
            }
        }
        int imgx = cemap.agent.x;
        int imgy = cemap.agent.y;
        cv::circle(BIdmap, cv::Point(imgx, imgy), 10, cv::Scalar(0, 255, 0), cv::FILLED);

        cv::imshow("Video", BIdmap);
        key = cv::waitKey(30);
        if (key == 'q')
            runkey = false;
        if (key >= '0' && key <= '9')
            keymod = key - '0';
        if (goalUpdata == 1)
        {
            goalUpdata = 0;
            cemap.RTRT_SetGoal(goalPoint);
        }
        if (updata == 1)
        {
            updata = 0;
            int32_t minIndex = 0;
            double mincost = doubleMax;
            for (const auto &pair : cemap.DObstacles)
            {
                double dx = pair.second.x - mousePoint.x;
                double dy = pair.second.y - mousePoint.y;
                double cost = dx * dx + dy * dy;
                if (mincost > cost)
                {
                    minIndex = pair.first;
                    mincost = cost;
                }
            }
            cemap.DObstacles[minIndex].x = mousePoint.x;
            cemap.DObstacles[minIndex].y = mousePoint.y;
        }
        if (cemap.agent_path.size() >= 2)
        {
            BIpoint agent_tag = cemap.agent_path.back();
            double dlen = speed * (utime_ns() - tt0) / 1e9;
            // sumlen += dlen;
            BIpoint p_old = cemap.agent_path.front();
            for (BIpoint &p : cemap.agent_path)
            {
                double len = p % p_old;
                if (len < dlen)
                {
                    dlen -= len;
                }
                else
                {
                    double t = dlen / len;
                    agent_tag = p * t + p_old * (1 - t);
                    break;
                }
                p_old = p;
            }
            cemap.RTRT_SetAgent(agent_tag);
        }
        std::vector<double> dataToWrite = {(utime_ns() - t0) / 1e6, cemap.agent.x, cemap.agent.y, cemap.goal.x, cemap.goal.y};
        for (const auto &pair : cemap.DObstacles)
        {
            dataToWrite.push_back(pair.second.x);
            dataToWrite.push_back(pair.second.y);
        }
    }
    cv::Mat BIdmap = cemap.BIdmap.clone();
    for (const auto &pair : cemap.Node_Map)
    {
        if (pair.second.invM == originIndexFun())
            continue;
        cv::Point startPoint((int)pair.first.x, (int)pair.first.y);
        cv::Point endPoint((int)pair.second.par.x, (int)pair.second.par.y);
        if (pair.second.cost.collision)
            cv::line(BIdmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 2);
        else
            cv::line(BIdmap, startPoint, endPoint, cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow("Video", BIdmap);
    cv::waitKey(0);

    return 0;
}
