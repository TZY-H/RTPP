#ifndef __CDRT_H
#define __CDRT_H
#include <iostream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <random>
#include <functional>
// #include <zip.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

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

struct BIpoint
{
    double x;
    double y;
    bool operator==(const BIpoint &other) const
    {
        return std::abs(x - other.x) <= 1e-16 &&
               std::abs(y - other.y) <= 1e-16;
    }
    bool operator!=(const BIpoint &other) const
    {
        return std::abs(x - other.x) > 1e-16 ||
               std::abs(y - other.y) > 1e-16;
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
    BIpoint operator-(const BIpoint &other) const
    {
        return {x - other.x, y - other.y};
    }
    BIpoint operator*(const double &ratio) const
    {
        return {ratio * x, ratio * y};
    }
    BIpoint operator/(const double &ratio) const
    {
        return {x / ratio, y / ratio};
    }
    double operator%(const BIpoint &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return sqrt(dx * dx + dy * dy);
    }
    // 叉乘
    double cross(const BIpoint &other) const
    {
        return x * other.y - y * other.x;
    }
    // 从json到BIpoint的转换函数
    void from_json(const json &j)
    {
        x = j.at(0).get<double>();
        y = j.at(1).get<double>();
    }
};

struct CTrtrt_Cost
{
    bool collision;
    double cost;

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

struct BIangle
{
    BIpoint O, S, E;
    bool Concave;
    bool operator<(const BIangle &other) const
    {
        if (O < other.O)
            return true;
        if (O != other.O)
            return false;
        if (S < other.S)
            return true;
        if (S != other.S)
            return false;
        return E < other.E;
    }
    bool operator==(const BIangle &other) const
    {
        return (O == other.O) && (S == other.S) && (E == other.E);
    }
};
typedef std::vector<BIpoint> BIpolygon;
typedef std::vector<BIpolygon> BIpolygons;

struct BIline
{
    BIpoint S;
    BIpoint E;
    // 构造函数，自动排序端点
    BIline(BIpoint p1, BIpoint p2)
    {
        if (p2 < p1)
        {
            S = p2;
            E = p1;
        }
        else
        {
            S = p1;
            E = p2;
        }
    }
    bool operator<(const BIline &other) const
    {
        if (S < other.S)
            return true;
        if (S != other.S)
            return false;
        return E < other.E;
    }
    bool operator==(const BIline &other) const
    {
        return (S == other.S) && (E == other.E);
    }
};

struct BIcutline
{
    int32_t index;
    BIline line;
    BIpoint core;
    std::set<int32_t> polygonlink; // polygon index set
    std::set<int32_t> cutlinelink; // cutline index set
};

struct BIobspolygon
{
    int32_t index;
    BIpolygon polygon;
    std::map<int32_t, int32_t> polygonRmap; // 周边的多边形索引集合
    std::vector<int32_t> polygonRlist;      // 周边的多边形索引有序列表
};

struct BIfreepolygon
{
    int32_t index;
    BIpolygon polygon;
    BIpoint core;
    std::set<int32_t> polygonlink; // polygon index set
    std::set<int32_t> cutlinelink; // cutline index set
};

struct BIinvnode
{
    int32_t polyS; // 源多边形引索，BIfreepolygon
    int32_t polyE; // 汇多边形引索，BIfreepolygon
    // int32_t index; // 同伦不变节点引索，到分割线(BIcutline)引索：((int)(index/2))
    bool operator<(const BIinvnode &other) const
    {
        if (polyS < other.polyS)
            return true;
        if (polyS != other.polyS)
            return false;
        return polyE < other.polyE;
    }
    bool operator==(const BIinvnode &other) const
    {
        return (polyS == other.polyS) && (polyE == other.polyE);
    }
};

// 将agentNode、cutNode、obsNode进行引索抽象(or 分割线引索的抽象)
#define originIndexFun() ((int32_t)(0x40000000))
#define obsNodeIndexFun(x) ((int32_t)(0x80000000 | x))
#define cutNodeIndexFun(x) ((int32_t)x)
typedef struct
{
    CTrtrt_Cost cost;
    BIpoint par;            // 父节点
    std::set<BIpoint> subs; // 子节点
    int32_t invM;           // 挂载对象逆映射
    // int32_t index;
} Node;

struct BIgraph
{
    int32_t cutlineBaseNum;
    int32_t freepolygonBaseNum;
    std::vector<BIcutline> cutlineList;
    std::vector<BIobspolygon> obspolygonList;
    std::vector<BIfreepolygon> freepolygonList;
    std::map<BIinvnode, int32_t> invnode2cutlineMap;
};

typedef struct
{
    int DObsType; //-1:不工作，0:圆，1：多边形
    double x;
    double y;
    double angle;
    double radius;
    double sampleRadius;
    std::vector<BIpoint> OutLine;
    std::vector<BIpoint> CheckPoints;
    std::set<int> ObsIslands;
} CTrtrt_DynamicObstacle;

struct BIedge // 考虑是否添加E节点所属桥信息
{
    BIpoint S;
    BIpoint E;
    // int32_t island;
    bool operator<(const BIedge &other) const
    {
        if (S != other.S)
            return S < other.S;
        // if (E != other.E)
        return E < other.E;
        // return island < other.island;
    }
};
typedef std::pair<double, BIpoint> VPpair;
typedef std::pair<CTrtrt_Cost, BIedge> VEpair;
#define BIdebug 1
class BImap
{
private:
public:
    // int32_t DisconnectedPeninsula(int32_t anum, int32_t inum_S, int32_t inum_E,
    //                               std::vector<int32_t> &BrokenBridges, std::vector<int32_t> &fpath); //, std::vector<int32_t> &BrokenIslands);

    uint32_t shapeX;  // 地图x轴像素数
    uint32_t shapeY;  // 地图y轴像素数
    double mapratio;  // 地图分辨率
    double robotsize; // 机器人直径

    cv::Mat BIamap;  // 岛群图像
    cv::Mat BIimap;  // 岛号图像
    cv::Mat BIdmap;  // debug图象
    cv::Mat BIfmap;  // free图象
    cv::Mat DObsmap; // obs图象碰撞检测
    int testcount = 0;

    void MaptoBInavi(const char *IMGmap, double ratio, double rsize, const char *addr = "127.0.0.1", int port = 23231);
    void DrawBImap(bool debugmap);

    std::vector<BIgraph> BIgraphList;
    void FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet);
    void FindConcave(const BIpolygons &polygons, std::set<BIangle> &ConcaveSet, std::set<BIangle> &AngleSet);
    void ViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> &cutlineList, std::vector<BIpoint> &VPList);
    BIpoint WeightViewablePoint(const BIangle &ConcaveAngle, const BIpolygons &polygons, const std::list<BIline> &cutlineList);
    void StartCut(const BIpolygons &polygons, BIgraph &graph);

    double r_th = 10;
    double p_th = 0.85;
    int32_t nearK = 20;
    int32_t RewireN = 100;

    BIpoint agent, agent_tag, goal;
    int32_t agent_apl, agent_island, goalIsSet = false;
    std::list<BIpoint> agent_path;

    std::map<BIpoint, Node> Node_Map;
    std::list<std::pair<BIpoint, int32_t>> NodeBSample_List;
    std::map<int32_t, Kd_tree> obsKdtreeMap; // F_{Iobs}
    std::map<int32_t, CTrtrt_DynamicObstacle> DObstacles;
    std::map<int32_t, std::set<BIpoint>> Mnode; //仅记录cutline上挂载的点，obs上点由obsKdtreeMap记录

    std::priority_queue<VEpair, std::vector<VEpair>, std::greater<VEpair>> eQr;
    std::set<BIpoint> largerSet;  // RTRT_UpdateTreeCost中变大过的节点
    std::set<BIpoint> smallerSet; // RTRT_UpdateTreeCost中变小过的节点

    void RTRT_Neighbor(const BIpoint &x, std::list<BIpoint> &Xnear);
    void RTRT_Neighbor(const BIpoint &x, std::list<BIpoint> &Xnear, int32_t invM);
    void RTRT_task(void);
    void RTRT_start(double Resolution);
    void RTRT_SetAgent(const BIpoint &point);
    void RTRT_SetGoal(const BIpoint &point);
    CTrtrt_Cost GetPath(void);
    CTrtrt_Cost GetPath(BIpoint &point, std::list<BIpoint> &path);

    void RTRT_AddDynamicObstacle(int num, CTrtrt_DynamicObstacle &obs, double Check = 0);
    void RTRT_DelDynamicObstacle(int num);
    bool RTRT_CrashDetection(const BIpoint &ps, int32_t invMps, const BIpoint &pe, int32_t invMpe);
    void RTRT_AddPotentialLinks(const BIpoint &p, bool larger);

    void RTRT_DrawDObsmap(void);     // 绘制动态障碍物碰撞区域
    void RTRT_CrashDObsIsland(void); // 更新被障碍物覆盖的多边形
    void RTRT_UpdateTreeCost(void);  // 更新树的代价使其一致
    bool RTRT_Sampling(BIpoint &randpoint, std::list<BIpoint> &Xnear); 

    

    BImap();
    ~BImap();
};

// 通用函数与参数 >>>
#define doubleMax (1.79769e+307)
// vector<pair<int32_t, size_t>>的哈希计算器
size_t hash_vec64(const std::vector<std::pair<int32_t, size_t>> &vec);
// 返回第一个共同元素，已弃用
int32_t findCommonElement(const std::set<int32_t> &set1, const std::set<int32_t> &set2);
// 简化的计时器
int64_t utime_ns(void);
// 计算点p到点o连线沿x轴正方向顺时针的夹角（单位：弧度）
double calculateAngle(BIpoint o, BIpoint p);
// 计算叉乘
double crossProduct(const BIpoint &O, const BIpoint &A, const BIpoint &B);
// 松弛的：检查两条线段是否相交，无视端点，线段长度不为0
bool doIntersect(const BIline &l1, const BIline &l2);
// 严谨的：检查两条线段是否相交，检查端点，长度可为0
bool doIntersect_rigorous(const BIline &l1, const BIline &l2);

// 可视化调试函数 >>>
void drawBIgraphObs(BIgraph &graph, cv::Mat image);
void drawPolygons(const BIpolygons &polygons, cv::Mat image);
void drawConcaves(std::list<BIangle> &ConcaveSet, cv::Mat image);
void drawAngle(BIangle &nowAngle, cv::Mat image);
void drawDObstacles(std::map<int, CTrtrt_DynamicObstacle> &DObstacles, cv::Mat image);
// void drawBInodesMap(BIgraph &graph,
//                     SSOtask &task,
//                     std::unordered_map<int32_t, NodeType2> &nodesMap,
//                     cv::Mat &image);
// void drawBIring(BIgraph &graph,
//                 SSOtask &task,
//                 std::list<int32_t> &ring,
//                 cv::Mat &image);
// void drawBIring(std::list<BIpoint> &Path, cv::Mat &image);
// void drawBIvalidPoly(BIgraph &graph,
//                      SSOtask &task,
//                      cv::Mat &inimage);
// void drawBIpath(const std::list<BIpoint> &path, cv::Mat &image);
// void drawBIinformed(const BIgraph &graph, const SSOtask &task, cv::Mat &image);

#endif