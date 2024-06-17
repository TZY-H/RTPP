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
std::map<std::string, std::vector<CTrtrt_DynamicObstacle>> taskObs;
std::map<std::string, std::vector<std::list<BIpoint>>> taskObsPaths;
void InitTaskMap(void)
{
    taskmap["FLOOR2"] = {{1100, 2200}, {{440, 110}, {790, 1440}, {1040, 220}, {1650, 350}}, {2356.376, 1104.037, 2112.879, 2075.546}, "/home/tzyr/WorkSpace/CDRT2/map/FLOOR2.png", {0.1, 0.9, 20}}; // 0.1,0.9
    taskmap["GAME6"] = {{550, 2048}, {{520, 980}, {245, 320}, {2130, 1500}, {1980, 460}}, {1984.248, 2043.672, 2222.7, 2865.941}, "/home/tzyr/WorkSpace/CDRT2/map/GAME6.png", {0.1, 1.1, 20}};      // 0.1,1.1
    taskmap["MAZE5"] = {{1600, 1780}, {{105, 160}, {3100, 90}, {150, 3010}, {3100, 3000}}, {3538.925, 3265.729, 2945.855, 4097.176}, "/home/tzyr/WorkSpace/CDRT2/map/MAZE5.png", {0.1, 0.9, 26}};   // 0.1,0.9
    taskmap["Trap"] = {{660, 660}, {{1160, 1160}, {130, 1160}, {130, 130}, {1160, 130}}, {1346.624, 2142.021, 3063.204, 3959.145}, "/home/tzyr/WorkSpace/CDRT2/map/Trap.png", {0.1, 0.9, 11}};      // 0.1,0.9
}

void InitTaskMapD(void)
{
    taskmap["FLOOR5"] = {{1000, 1200}, {{400, 300}, {1920, 600}, {1750, 900}, {1460, 2100}}, {1848.505, 3161.531, 2328.463, 1508.339}, "/home/tzyr/WorkSpace/CDRT2/map/FLOOR5.png", {0.1, 0.5, 20}}; // 0.1,0.5
    taskmap["GAME0"] = {{350, 2350}, {{1400, 2050}, {888, 800}, {2500, 1500}, {2000, 400}}, {1261.356, 1757.369, 2566.572, 2596.944}, "/home/tzyr/WorkSpace/CDRT2/map/GAME0.png", {0.1, 0.5, 23}};   // 0.1,0.5
    taskmap["MAZE0"] = {{1500, 1850}, {{200, 200}, {3100, 200}, {3100, 3100}, {200, 3100}}, {2250.927, 2406.131, 2967.605, 1931.922}, "/home/tzyr/WorkSpace/CDRT2/map/MAZE0.png", {0.1, 1.1, 28}};   // 0.1,0.5
    taskmap["MESS6"] = {{720, 720}, {{70, 70}, {1240, 70}, {1240, 1240}, {70, 1240}}, {948.194, 926.498, 739.732, 841.725}, "/home/tzyr/WorkSpace/CDRT2/map/MESS6.png", {0.1, 0.5, 11}};             // 0.1,0.5

    taskObs["MESS6"] = {{1, 284, 284, 0, 0, 150, {{-65, 65}, {65, 65}, {65, -65}, {-65, -65}}, {}, {}},
                        {1, 1034, 198, 0, 0, 150, {{-54, -28}, {-28, -28}, {-28, -54}, {28, -54}, {28, -28}, {54, -28}, {54, 28}, {28, 28}, {28, 54}, {-28, 54}, {-28, 28}, {-54, 28}}, {}, {}},
                        {1, 1044, 889, 0, 0, 150, {{0, -81}, {-81, 81}, {81, 81}}, {}, {}},
                        {0, 350, 955, 0, 82, 150, {{0, 0}}, {}, {}}};
    taskObs["MAZE0"] = {{0, 485, 1040, 0, 70, 150, {{0, 0}}, {}, {}},
                        {0, 2650, 520, 0, 70, 150, {{0, 0}}, {}, {}},
                        {0, 1930, 2750, 0, 70, 150, {{0, 0}}, {}, {}},
                        {0, 850, 2780, 0, 70, 150, {{0, 0}}, {}, {}}};
    taskObs["GAME0"] = {{0, 370, 388, 0, 57, 120, {{0, 0}}, {}, {}},
                        {0, 2370, 630, 0, 114, 180, {{0, 0}}, {}, {}},
                        {1, 1364, 1364, 0.78539816f, 0, 120, {{0, -57}, {57, 0}, {57, 57}, {-57, 57}, {-57, 0}}, {}, {}},
                        {1, 1340, 2520, 0, 0, 120, {{-57, -57}, {57, -57}, {57, 57}, {-57, 57}}, {}, {}}};
    taskObs["FLOOR5"] = {{0, 535, 435, 0, 49, 120, {{0, 0}}, {}, {}},
                         {1, 425, 1750, 0, 0, 120, {{-49, -49}, {49, -49}, {49, 49}, {-49, 49}}, {}, {}},
                         {1, 1246, 2022, 0, 0, 120, {{0, -49}, {49, 49}, {-49, 49}}, {}, {}},
                         {1, 1378, 888, 0, 0, 120, {{-52, 0}, {-31, -49}, {31, -49}, {52, 0}, {31, 49}, {-31, 49}}, {}, {}}};

    taskObsPaths["MESS6"] = {
        {{284, 284}, {880, 230}, {777, 530}},
        {{1034, 198}, {1200, 920}, {747, 980}},
        {{1044, 889}, {1044, 425}},
        {{350, 955}, {60, 630}, {260, 447}}};
    taskObsPaths["MAZE0"] = {
        {{485, 1040}, {465, 1426}, {170, 1420}, {180, 870}},
        {{2650, 520}, {2420, 525}, {2420, 1130}, {1600, 1130}},
        {{1930, 2750}, {2100, 2760}, {2100, 2150}, {1800, 2180}, {1800, 2500}},
        {{850, 2780}, {850, 2400}, {100, 2400}}};
    taskObsPaths["GAME0"] = {
        {{370, 388}, {225, 630}, {2230, 1460}},
        {{2370, 630}, {2340, 240}, {2000, 200}},
        {{1364, 1364}, {1900, 850}},
        {{1340, 2520}, {1490, 2500}, {1480, 2350}, {1760, 2260}}};
    taskObsPaths["FLOOR5"] = {
        {{535, 435}, {700, 490}, {700, 1130}},
        {{425, 1750}, {240, 1800}, {400, 1650}, {580, 1850}},
        {{1246, 2022}, {1300, 1970}, {1300, 1750}, {1900, 1740}},
        {{1378, 888}, {1300, 930}, {1025, 900}}};
}
#define speed 400.0
#define obsspeed 100.0
int main()
{
    InitTaskMapD();
    cv::namedWindow("Video", 0);
    cv::resizeWindow("Video", 1200, 1200);

    for (const auto &pair : taskmap)
    {
        const std::string &name = pair.first;
        const BItask &task = pair.second;
        std::cout << task.Path << std::endl;
        for (int32_t i = 0; i < task.G.size(); i++)
        {
            BImap cemap;
            cemap.MaptoBInavi(task.Path.data(), task.parameters[0], task.parameters[1]);
            cemap.DrawBImap(true);
            cemap.RTRT_AddDynamicObstacle(0, taskObs[pair.first][0], 2);
            cemap.RTRT_AddDynamicObstacle(1, taskObs[pair.first][1], 2);
            cemap.RTRT_AddDynamicObstacle(2, taskObs[pair.first][2], 2);
            cemap.RTRT_AddDynamicObstacle(3, taskObs[pair.first][3], 2);
            cemap.RTRT_SetAgent(task.S);
            cemap.RTRT_SetGoal(task.G[i]);
            int64_t t_1 = -1;
            int64_t t0 = utime_ns();
            double sumlen = 0;
            cemap.RTRT_start(task.parameters[2]);
            for (;;)
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
                    if (t_1 < 0)
                        t_1 = utime_ns() - t0;
                    std::list<BIpoint> &path = cemap.agent_path;
                    BIpoint point0 = path.front();
                    for (BIpoint point1 : path)
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
                cv::waitKey(50);

                std::vector<std::list<BIpoint>> &ObsPaths = taskObsPaths[name];
                for (int32_t obsIndex = 0; obsIndex < 4; obsIndex++)
                {
                    std::list<BIpoint> &ObsPath = ObsPaths[obsIndex];
                    double dlen = obsspeed * (utime_ns() - t0) / 1e9;
                    BIpoint p_old = ObsPath.front();
                    BIpoint Obs_tag = ObsPath.back();
                    for (BIpoint &p : ObsPath)
                    {
                        double len = p % p_old;
                        if (len < dlen)
                        {
                            dlen -= len;
                        }
                        else
                        {
                            double t = dlen / len;
                            Obs_tag = p * t + p_old * (1 - t);
                            break;
                        }
                        p_old = p;
                    }
                    cemap.DObstacles[obsIndex].x = Obs_tag.x;
                    cemap.DObstacles[obsIndex].y = Obs_tag.y;
                }
                if (cemap.agent_path.size() >= 2)
                {
                    BIpoint agent_tag = cemap.agent_path.back();
                    double dlen = speed * (utime_ns() - tt0) / 1e9;
                    sumlen += dlen;
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
                if (cemap.agent % task.G[i] <= 1.0)
                    break;
            }
            std::cout << "      " << i << ":" << utime_ns() - t0 << " ; " << t_1 << " ; " << sumlen << std::endl;
        }
    }

    return 0;
}
