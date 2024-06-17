#ifndef __BImap_H
#define __BImap_H
#include <iostream>
#include <stdio.h>
#include <queue>
#include <deque>
#include <vector>
#include <set>
#include <map>
#include <thread>
#include <random>
// #include <zip.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "CSubdivision.h"

class BIbridge;
class BIisland;

typedef struct
{
    float cost;
    float x;
    float y;
    int32_t par; // 父节点
    // int32_t sub;    //子节点
    int32_t bridge; // 所属分割线

} Node;

struct BIpoint
{
    float x;
    float y;
    bool operator!=(const BIpoint &other) const
    {
        return x != other.x || y != other.y;
    }
    bool operator<(const BIpoint &other) const
    {
        // 在这里定义比较规则
        if (x != other.x)
        {
            return x < other.x;
        }
        return y < other.y;
    }
    BIpoint operator+(const BIpoint &other) const
    {
        return {x + other.x, y + other.y};
    }
    float operator%(const BIpoint &other) const
    {
        float dx = x - other.x;
        float dy = y - other.y;
        return sqrtf32(dx * dx + dy * dy);
    }
};
// struct CTrtrt_Node; // 简化的

class BIbridge ///////////////////////////////
{
private:
    /* data */
public:
    int32_t pointS;
    int32_t pointE;
    float length;
    // uint32_t pointslen;
    BIpoint core;
    std::vector<int32_t> Ilink; // 改编号
    uint32_t Ilinklen;
    std::vector<int32_t> Blink; // 改编号
    uint32_t Blinklen;
    std::set<BIpoint> Nodes;
};

class BIisland /////////////////////////////
{
private:
    /* data */
public:
    std::vector<int32_t> points; // 改编号
    uint32_t pointslen;
    std::vector<int32_t> Blink; // 改编号
    uint32_t Blinklen;
    // std::set<CTrtrt_Node*> Nodes;
};

class BIarchipelago
{
private:
    /* data */
public:
    int32_t aplnum;
    std::vector<BIpoint> pointmap;
    uint32_t pointmaplen;
    std::vector<BIisland> islandlist;
    uint32_t islandlistlen;
    std::vector<BIbridge> bridgelist;
    uint32_t bridgelistlen;
    std::vector<int32_t> peninsula; // 半岛列表

    BIpoint CTonly_Pinit = {-1.0f, -1.0f};
    std::vector<Node> CTonly_BNodes;
    std::vector<int32_t> CTonly_BNlink;
};

#define BIdebug 1
class BImap
{
private:
    BIpoint CTonly_Pinit = {0, 0};
    BIpoint ConnecDetection(BIpoint &S, BIpoint &E);
    bool DetectIntersection(Node &S0, Node &E0, BIpoint &S1, BIpoint &E1);

public:
    int32_t DisconnectedPeninsula(int32_t anum, int32_t inum_S, int32_t inum_E,
                                  std::vector<int32_t> &BrokenBridges, std::vector<int32_t> &fpath); //, std::vector<int32_t> &BrokenIslands);

    uint32_t shapeX;                    // 地图x轴像素数
    uint32_t shapeY;                    // 地图y轴像素数
    float mapratio;                     // 地图分辨率
    float robotsize;                    // 机器人直径
    std::vector<BIarchipelago> apllist; // 岛群列表
    // BIarchipelago* aplfree;
    // uint32_t apllistlen;
    cv::Mat BIamap; // 岛群图像
    cv::Mat BIimap; // 岛号图像
    cv::Mat BIdmap; // debug图象
    cv::Mat BIfmap; // free图象
    cv::Mat ShowMap;
    void MaptoBInavi(char *IMGmap, float ratio, float rsize); //,vector<vector<CSpoint>> &returnlist);
    void CSsetBInavi(CSubdivision &csdata, BIarchipelago &apl, uint32_t num);
    // void CSsetBInavi(CSubdivision& csdata,BIarchipelago& apl);
    void DrawBImap(bool debugmap);
    void DrawBImap(bool debugmap, std::vector<int32_t> &BrokenBridges, std::vector<int32_t> &BrokenIslands);
    void BImaptoJson(char *name);
    void JsontoBImap(char *name, bool debugmap);

    void CTRW(BIpoint &S, BIpoint &E, int32_t N, std::list<BIpoint> &Path);
    void CT_RRTstar(BIpoint &S, BIpoint &E, float weaken, int32_t N, std::list<BIpoint> &Path);
    // void ShortestTonglenPath();
    void CT_onlyBegin(float Resolution);
    int CT_onlySetInit(BIpoint &S);
    float CT_onlyGetPath(BIpoint &E, std::list<BIpoint> &Path);
    void CT_onlyGetPath(BIpoint &S, BIpoint &E, std::list<BIpoint> &Path);
    void GetLeastHomotopyPath(BIarchipelago &apl, BIpoint &S, BIpoint &E, std::vector<int32_t> &f_path, std::list<BIpoint> &Path, float &PathMinCost);
    BImap();
    ~BImap();
};

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

// typedef std::pair<BIpoint, BIpoint> BIedge;
struct BIedge // 考虑是否添加E节点所属桥信息
{
    BIpoint S;
    BIpoint E;
    int32_t island;
    bool operator<(const BIedge &other) const
    {
        if (S != other.S)
            return S < other.S;
        if (E != other.E)
            return E < other.E;
        return island < other.island;
    }
};
struct CTrtrt_Cost
{
    bool collision;
    float cost;
    CTrtrt_Cost operator+(const CTrtrt_Cost &other) const
    {
        return {(collision || other.collision), cost + other.cost};
    }
    bool operator==(const CTrtrt_Cost &other) const
    {
        return (collision == other.collision) && (cost == other.cost);
    }
    bool operator<(const CTrtrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost < other.cost;
        return collision < other.collision;
    }
    bool operator>(const CTrtrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost > other.cost;
        return collision > other.collision;
    }
    bool operator<=(const CTrtrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost <= other.cost;
        return collision <= other.collision;
    }
    bool operator>=(const CTrtrt_Cost &other) const
    {
        if (collision == other.collision)
            return cost >= other.cost;
        return collision >= other.collision;
    }
};
struct CTrtrt_NodeAttribute // 简化的
{
    CTrtrt_Cost cost;
    BIpoint par;            // 父节点
    std::set<BIpoint> subs; // 子节点!!!
    int32_t bridge;         // 所属分割线，-1表示动态障碍物闭包内的点，-2表示agent
    int32_t island;         // 所属岛，-1表示不确定
};
typedef struct
{
    int DObsType; //-1:不工作，0:圆，1：多边形
    float x;
    float y;
    float angle;
    float radius;
    float sampleRadius;
    std::vector<BIpoint> OutLine;
    std::vector<BIpoint> CheckPoints;
    std::set<int> ObsIslands;
    // float img_x0;
    // float img_y0;
    // cv::Mat DObsImg;
} CTrtrt_DynamicObstacle;
typedef std::pair<float, BIpoint> VPpair;
typedef std::pair<CTrtrt_Cost, BIedge> VEpair;

class CTrtrt : protected BImap
{
private:
    thread thread_handle;
    BIpoint agent_tag;
    BIpoint agent;
    BIpoint goal;
    int agent_apl;
    int agent_island;
    uint64_t nodes_stamp = 0;

public:
    bool runkey = false;
    // int32_t testKey = 0;

    cv::Mat DObsmap;                      // obs图象碰撞检测
    std::map<float, int> rouletteOds_Map; // 动态障碍物闭包采样概率
    // std::map<float, int> rouletteCut_Map; // 桥轮盘赌采样概率

    std::map<int, CTrtrt_DynamicObstacle> DObstacles;
    std::map<int, Kd_tree> global_ObsIslands;                  // 岛号、岛中的节点!!!
    std::vector<std::pair<BIpoint, int32_t>> NodeBSample_List; // 当前岛群上所有桥上点的离散
    std::map<BIpoint, CTrtrt_NodeAttribute> Node_Map;

    using BImap::DisconnectedPeninsula;

    using BImap::apllist;   // 岛群列表
    using BImap::mapratio;  // 地图分辨率
    using BImap::robotsize; // 机器人直径
    using BImap::shapeX;    // 地图x轴像素数
    using BImap::shapeY;    // 地图y轴像素数

    using BImap::BIamap; // 岛群图像
    using BImap::BIdmap; // debug图象
    using BImap::BIimap; // 岛号图像
    using BImap::BImaptoJson;
    using BImap::CSsetBInavi;
    using BImap::DrawBImap;
    using BImap::JsontoBImap;
    using BImap::MaptoBInavi; //,vector<vector<CSpoint>> &returnlist);
    using BImap::ShowMap;

    // void SamplePoints(std::vector<BIpoint> &points_list);
    void RTRT_task(void);
    void RTRT_start(float Resolution);
    void RTRT_join(void);

    void RTRT_SetAgent(BIpoint &point);
    void RTRT_SetGoal(BIpoint &point);

    void RTRT_AddDynamicObstacle(int num, CTrtrt_DynamicObstacle &obs, float Check = 0);
    void RTRT_DelDynamicObstacle(int num);
    bool RTRT_CrashDObsPoint(CTrtrt_DynamicObstacle &obs, BIpoint &point); // 更严格，计划弃用
    void RTRT_DrawDObsmap(void);                                           // 绘制动态障碍物碰撞区域
    void RTRT_CrashDObsIsland(std::list<BIpoint> &noParPointQueue);        // 更新被障碍物覆盖的多边形

    bool RTRT_SampleOnobs(BIpoint &randpoint, int errCount_max = 20);                             // 动态障碍物采样
    bool RTRT_SampleOnobs(std::list<BIpoint> &randpoints, int Sample_num, int errCount_max = 20); // 动态障碍物采样
    bool RTRT_SampleOnline(BIpoint &randpoint);                                                   // 桥采样
    bool RTRT_SampleOnline(std::list<BIpoint> &randpoints, int Sample_num);                       // 桥采样
    void RTRT_AddPotentialEdges(const BIpoint &randpoint,
                                std::priority_queue<VEpair, vector<VEpair>, greater<VEpair>> &Qr);

    bool RTRT_CrashDetection(const BIedge &edge);
    bool RTRT_CrashDetection(const BIpoint &ps, const BIpoint &pe, int32_t islandNum);
    void RTRT_LinkPoint(BIpoint par, BIpoint sub);                          // 暂时弃用
    void RTRT_FindNearNodes(std::set<BIpoint *> &NearNodes, BIpoint &node); // 暂时弃用
    // CTrtrt(/* args */);
    // ~CTrtrt();
};

long long utime_ns(void);
extern std::list<float> testCost;
extern std::list<int32_t> testNum;
extern std::list<long long> testTime;
extern long long testTime0;
extern float testMinCost;
extern cv::Mat showmat;

#endif