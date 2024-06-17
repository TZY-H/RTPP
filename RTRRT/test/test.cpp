#include <iostream>
#include <stdio.h>
#include <queue>
#include <deque>
#include <vector>
#include <set>
#include <map>
#include <thread>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "RTRRT.h"

// 回调函数，用于处理鼠标点击事件
int32_t testKey = 0;
RTpoint tagpoint = {0, 0};

void onMouse(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_MOUSEMOVE)
    {
        if (flags & cv::EVENT_FLAG_LBUTTON)
        {
            testKey = 1;
            tagpoint = {(float)x, (float)y};
            printf("tagpoint:%d ,%d\r\n", x, y);
        }
    }
}

int main()
{
    cv::namedWindow("Obsmap");
    cv::setMouseCallback("Obsmap", onMouse);

    RTrrt_DynamicObstacle obs1, obs2;
    obs1.x = 250;
    obs1.y = 250;
    obs1.angle = 0;
    obs1.DObsType = 0;
    obs1.radius = 20;
    obs1.sampleRadius = 100;
    obs1.OutLine.push_back({40, 0});
    obs1.OutLine.push_back({-40, 30});
    obs1.OutLine.push_back({-40, -30});
    obs2.x = 550;
    obs2.y = 350;
    obs2.angle = 0;
    obs2.DObsType = 1;
    obs2.radius = 20;
    obs2.sampleRadius = 150;
    obs2.OutLine.push_back({40, 0});
    obs2.OutLine.push_back({-40, 30});
    obs2.OutLine.push_back({-40, -30});

    RTRRTplanner testRT;
    const char *IMGmapc = (char *)"../map/GAME0.png";
    int threshold_value = 128; // 二值化阈值，根据实际需要设置

    testRT.InitializeInvironment(IMGmapc, threshold_value);
    testRT.AddDynamicObstacle(1, obs1);
    testRT.AddDynamicObstacle(2, obs2);

    RTpoint Agent = {550, 550};
    testRT.SetAgent(Agent);
    std::list<RTpoint> path;
    int keymod = 0;
    int key = 0;

    while (1)
    {
        // long long t0 = utime_ns();
        // while (utime_ns() - t0 < 50e6)
        testRT.MainTask();
        testRT.GetPath(path);
        std::cout << "Node_Map.size" << testRT.Node_Map.size() << std::endl;

        cv::Mat dmap = testRT.Freemap.clone();
        testRT.DrawDObsmapDBUG(dmap);
        for (const auto &KVpair : testRT.Node_Map)
        {
            cv::Point endPoint((int)KVpair.first.x, (int)KVpair.first.y);
            const RTpoint &par = KVpair.second.par;
            if (par.x < 0)
                continue;
            if (KVpair.second.cost < RTrrt_Cost{1, 0})
            {
                cv::Point startPoint((int)(0.2 * par.x + 0.8 * KVpair.first.x), (int)(0.2 * par.y + 0.8 * KVpair.first.y));
                cv::line(dmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 1);
                endPoint = startPoint;
                startPoint = {(int)par.x, (int)par.y};
                cv::line(dmap, startPoint, endPoint, cv::Scalar(255, 0, 0), 1);
            }
            else
            {
                cv::Point startPoint((int)par.x, (int)par.y);
                cv::line(dmap, startPoint, endPoint, cv::Scalar(255, 0, 255), 1);
            }
        }
        if (path.size() >= 2)
        {
            RTpoint point0 = path.front();
            for (RTpoint point1 : path)
            {
                cv::Point startPoint((int)point0.x, (int)point0.y);
                cv::Point endPoint((int)point1.x, (int)point1.y);
                cv::line(dmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 2);
                point0 = point1;
            }
        }
        cv::imshow("Obsmap", dmap);
        key = cv::waitKey(10);
        if (key == 'q')
        {
            break;
        }
        else if (key == '0')
            keymod = 0;
        else if (key == '1')
            keymod = 1;
        else if (key == '2')
            keymod = 2;
        else if (key == '3')
            keymod = 3;
        if (testKey == 1)
        {
            testKey = 0;
            switch (keymod)
            {
            case 0:
                testRT.SetAgent(tagpoint);
                break;
            case 1:
                testRT.DObstacles[1].x = tagpoint.x;
                testRT.DObstacles[1].y = tagpoint.y;
                break;
            case 2:
                testRT.DObstacles[2].x = tagpoint.x;
                testRT.DObstacles[2].y = tagpoint.y;
                break;
            case 3:
                testRT.SetGoal(tagpoint);
                break;
            }
        }
    }

    return 0;
}
