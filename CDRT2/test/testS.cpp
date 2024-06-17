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
        // std::cout << "BIimap: " << bimap->BIimap.at<uint16_t>(y, x) << std::endl;
        // std::cout << "informedFun: " << task->informed.informedFun({(double)x, (double)y}) << std::endl;
    }
}

struct BItask
{
    BIpoint S;
    std::vector<BIpoint> G;
    std::vector<double> miniCost;
    std::string Path;
    double parameters[3];
};

std::map<std::string, BItask> taskmap;
void InitTaskMap(void)
{
    taskmap["FLOOR2"] = {{1100, 2200}, {{440, 110}, {790, 1440}, {1040, 220}, {1650, 350}}, {2356.376, 1104.037, 2112.879, 2075.546}, "/home/tzyr/WorkSpace/CDRT2/map/FLOOR2.png", {0.1, 0.9, 20}}; // 0.1,0.9
    taskmap["GAME6"] = {{550, 2048}, {{520, 980}, {245, 320}, {2130, 1500}, {1980, 460}}, {1984.248, 2043.672, 2222.7, 2865.941}, "/home/tzyr/WorkSpace/CDRT2/map/GAME6.png", {0.1, 1.1, 20}};      // 0.1,1.1
    taskmap["MAZE5"] = {{1600, 1780}, {{105, 160}, {3100, 90}, {150, 3010}, {3100, 3000}}, {3538.925, 3265.729, 2945.855, 4097.176}, "/home/tzyr/WorkSpace/CDRT2/map/MAZE5.png", {0.1, 0.9, 26}};   // 0.1,0.9
    taskmap["Trap"] = {{660, 660}, {{1160, 1160}, {130, 1160}, {130, 130}, {1160, 130}}, {1346.624, 2142.021, 3063.204, 3959.145}, "/home/tzyr/WorkSpace/CDRT2/map/Trap.png", {0.1, 0.9, 11}};      // 0.1,0.9
}

int main()
{
    InitTaskMap();

    for (const auto &pair : taskmap)
    {
        const std::string &name = pair.first;
        const BItask &task = pair.second;
        std::cout << task.Path << std::endl;
        for (int32_t i = 0; i < task.G.size(); i++)
        {
            BImap cemap;
            cemap.MaptoBInavi(task.Path.data(), task.parameters[0], task.parameters[1]);
            cemap.DrawBImap(false);
            cemap.RTRT_SetAgent(task.S);
            cemap.RTRT_SetGoal(task.G[i]);
            int64_t t0 = utime_ns();
            cemap.RTRT_start(task.parameters[2]);
            // for (;;)
            // {
            //     cemap.RTRT_task();
            //     CTrtrt_Cost cost = cemap.GetPath();
            //     if (cost.cost < task.miniCost[i] * 1.02)
            //         break;
            // }
            std::cout << "      "<< i <<":" << (utime_ns() - t0)*3.0/1000 << std::endl;
        }
    }

    // BImap cemap;
    // // 创建窗口并注册鼠标回调函数
    // cv::namedWindow("Video", 0);
    // cv::resizeWindow("Video", 1200, 1200);
    // cv::setMouseCallback("Video", onMouse, NULL);
    // cemap.MaptoBInavi(task.Path.data(), task.parameters[0], task.parameters[1]);
    // cemap.DrawBImap(true);
    // cv::imshow("Video", cemap.BIdmap);
    // cv::waitKey(0);
    // cemap.RTRT_SetAgent(task.S);
    // cemap.RTRT_start(task.parameters[2]);

    // std::cout << cemap.NodeBSample_List.size() << std::endl;
    // std::cout << cemap.eQr.size() << std::endl;

    // int64_t t0 = utime_ns();
    // while (cemap.NodeBSample_List.size() || cemap.eQr.size())
    // {
    //     cemap.RTRT_task();
    //     // cemap.GetPath();
    // }
    // std::cout << utime_ns() - t0 << std::endl;

    // std::cout << task.Path << std::endl;
    // for (BIpoint &goal : task.G)
    // {
    //     std::list<BIpoint> path;
    //     CTrtrt_Cost cost = cemap.GetPath(goal, path);
    //     if (path.size() >= 2)
    //     {
    //         BIpoint point0 = path.front();
    //         for (BIpoint point1 : path)
    //         {
    //             cv::Point startPoint((int)point0.x, (int)point0.y);
    //             cv::Point endPoint((int)point1.x, (int)point1.y);
    //             cv::line(cemap.BIdmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 3);
    //             point0 = point1;
    //         }
    //     }
    //     printf(" %8.3f\r\n", cost.cost);
    // }

    // cv::imshow("Video", cemap.BIdmap);
    // cv::waitKey(0);

    return 0;
}
