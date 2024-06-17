#ifndef __RTRRT_H
#define __RTRRT_H
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

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_2.h>
#include <CGAL/Kd_tree.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Search_traits_2<K> Traits;
typedef CGAL::Kd_tree<Traits> Kd_tree;
using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<Traits>;
using Point_with_distance = Neighbor_search::Point_with_transformed_distance;

struct RTpoint
{
    float x;
    float y;
    bool operator!=(const RTpoint &other) const
    {
        return x != other.x || y != other.y;
    }
    bool operator<(const RTpoint &other) const
    {
        // 在这里定义比较规则
        if (x != other.x)
            return x < other.x;
        return y < other.y;
    }
    RTpoint operator+(const RTpoint &other) const
    {
        return {x + other.x, y + other.y};
    }
    float operator%(const RTpoint &other) const
    {
        float dx = x - other.x;
        float dy = y - other.y;
        return sqrtf32(dx * dx + dy * dy);
    }
};

struct RTrrt_Cost
{
    bool collision;
    float cost;
    RTrrt_Cost operator+(const RTrrt_Cost &other) const
    {
        return {(collision || other.collision), cost + other.cost};
    }
    bool operator==(const RTrrt_Cost &other) const
    {
        return (collision == other.collision) && (cost == other.cost);
    }
    bool operator<(const RTrrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost < other.cost;
        return collision < other.collision;
    }
    bool operator>(const RTrrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost > other.cost;
        return collision > other.collision;
    }
    bool operator<=(const RTrrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost <= other.cost;
        return collision <= other.collision;
    }
    bool operator>=(const RTrrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost >= other.cost;
        return collision >= other.collision;
    }
};
typedef struct
{
    int DObsType; //-1:不工作，0:圆，1：多边形
    float x;
    float y;
    float angle;
    float radius;
    float sampleRadius;
    std::vector<RTpoint> OutLine;
    std::vector<RTpoint> CheckPoints;
    std::set<int> ObsIslands;
    // float img_x0;
    // float img_y0;
    // cv::Mat DObsImg;
} RTrrt_DynamicObstacle;

struct RTrrt_NodeAttribute // 简化的
{
    RTrrt_Cost cost;
    RTpoint par;            // 父节点
    std::set<RTpoint> subs; // 子节点!!!
};

class RTRRTplanner
{
private:
    RTpoint agent_tag;
    RTpoint agent;
    RTpoint root = {-1, -1};
    RTpoint goal = {-1, -1};
    float c_base;

public:
    int32_t shapeX; // 地图x轴像素数
    int32_t shapeY; // 地图y轴像素数

    std::map<int, RTrrt_DynamicObstacle> DObstacles;
    Kd_tree NeighborSearchTree;
    std::map<RTpoint, RTrrt_NodeAttribute> Node_Map;
    std::queue<RTpoint> Qr;
    std::queue<RTpoint> Qs;
    std::set<RTpoint> Qs_set;
    std::set<RTpoint> visited_set;
    std::list<RTpoint> OutPath;

    cv::Mat Freemap; // free图象碰撞检测
    cv::Mat Obsmap;  // obs图象碰撞检测
    cv::Mat DObsmap; // Dobs图象碰撞检测

    void SetAgent(RTpoint &point);
    void SetGoal(RTpoint &point);

    void InitializeInvironment(const char *IMGmap, int th);
    void AddDynamicObstacle(int num, RTrrt_DynamicObstacle &obs);
    void DelDynamicObstacle(int num);
    void UpdateTree(void);

    void Start(void);
    void MainTask(void);
    void ExpandRewiring(long RunTime); // MS

    void DrawDObsmap(void);
    void DrawDObsmapDBUG(cv::Mat &DBUGmap);
    bool CrashDetection(const RTpoint &ps, const RTpoint &pe);

    void GetPath(std::list<RTpoint> &path);
};
long long utime_ns(void);
#endif
