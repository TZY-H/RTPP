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
int32_t updata = false;
BIpoint mousePoint = {-1, -1};
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
}

struct BItask
{
    BIpoint S;
    std::vector<BIpoint> G;
    double miniCost;
    std::string Path;
    double parameters[3];
};

std::map<std::string, BItask> taskmap;
void InitTaskMap(void)
{
    taskmap["FLOOR2"] = {{1100, 2200}, {{440, 110}, {790, 1440}, {1040, 220}, {1650, 350}}, 0, "/home/tzyr/WorkSpace/CDRT2/map/FLOOR2.png", {0.1, 0.9, 20}}; // 0.1,0.9
    taskmap["GAME6"] = {{550, 2048}, {{520, 980}, {245, 320}, {2130, 1500}, {1980, 460}}, 0, "/home/tzyr/WorkSpace/CDRT2/map/GAME6.png", {0.1, 1.1, 20}};    // 0.1,1.1
    taskmap["MAZE5"] = {{1600, 1780}, {{105, 160}, {3100, 90}, {150, 3010}, {3100, 3000}}, 0, "/home/tzyr/WorkSpace/CDRT2/map/MAZE5.png", {0.1, 0.9, 26}};   // 0.1,0.9
    taskmap["Trap"] = {{660, 660}, {{1160, 1160}, {130, 1160}, {130, 130}, {1160, 130}}, 0, "/home/tzyr/WorkSpace/CDRT2/map/Trap.png", {0.1, 0.9, 11}};      // 0.1,0.9
}

int main()
{
    InitTaskMap();
    BItask &task = taskmap["FLOOR2"];
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
    obs3.radius = 20;
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
    cv::resizeWindow("Video", 1200, 1200);
    cv::setMouseCallback("Video", onMouse, NULL);
    cemap.MaptoBInavi(task.Path.data(), task.parameters[0], task.parameters[1]);
    cemap.DrawBImap(true);
    cv::imshow("Video", cemap.BIdmap);
    cv::waitKey(0);

    cemap.RTRT_AddDynamicObstacle(1, obs1, 3);
    cemap.RTRT_AddDynamicObstacle(2, obs2, 3);
    cemap.RTRT_AddDynamicObstacle(3, obs3, 3);
    cemap.RTRT_AddDynamicObstacle(4, obs4, 3);

    // BIpoint S = task.S;
    cemap.RTRT_SetAgent(task.S);
    cemap.RTRT_start(20);
    int64_t t0 = utime_ns();

    int keymod = 0;
    int key = 0;
    int runkey = 1;
    // std::list<BIpoint> path;

    while (runkey)
    {
        std::cout << utime_ns() - t0 << std::endl;
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
                cv::line(BIdmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 2);
            else
                cv::line(BIdmap, startPoint, endPoint, cv::Scalar(255, 0, 0), 2);
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

        cv::imshow("Video", BIdmap);
        key = cv::waitKey(30);
        switch (key)
        {
        case 'q':
            runkey = false;
            break;
        case '0':
            keymod = 0;
            break;
        case '1':
            keymod = 1;
            break;
        case '2':
            keymod = 2;
            break;
        case '3':
            keymod = 3;
            break;
        case '4':
            keymod = 4;
            break;
        case '9':
            keymod = 9;
            break;

        default:
            break;
        }
        if (updata == 1)
        {
            updata = 0;
            switch (keymod)
            {
            case 0:
                cemap.RTRT_SetAgent(mousePoint);
                break;
            case 1:
                cemap.DObstacles[1].x = mousePoint.x;
                cemap.DObstacles[1].y = mousePoint.y;
                break;
            case 2:
                cemap.DObstacles[2].x = mousePoint.x;
                cemap.DObstacles[2].y = mousePoint.y;
                break;
            case 3:
                cemap.DObstacles[3].x = mousePoint.x;
                cemap.DObstacles[3].y = mousePoint.y;
                break;
            case 4:
                cemap.DObstacles[4].x = mousePoint.x;
                cemap.DObstacles[4].y = mousePoint.y;
                break;
            case 9:
            {
                cemap.RTRT_SetGoal(mousePoint);
            }
            break;
            }
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
