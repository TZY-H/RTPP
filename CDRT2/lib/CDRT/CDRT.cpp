#include "CDRT.h"

#include <math.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// using namespace std;
using json = nlohmann::json;
cv::Mat showmat;

int64_t utime_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(1000000000UL) * static_cast<long long>(ts.tv_sec) +
           static_cast<int64_t>(ts.tv_nsec);
}

// 从json到BIpoint的转换
void from_json(const json &j, BIpoint &p)
{
    p.x = j.at(0).get<double>();
    p.y = j.at(1).get<double>();
}

// 递归函数，用于生成包含 ringSet 的所有排列
void generatePermutations(std::vector<int32_t> &currentPermutation, const std::set<int32_t> &ringSet, std::vector<std::vector<int32_t>> &permutations)
{
    // 如果当前排列包含 ringSet 中所有元素，则将其添加到 permutations 中
    if (currentPermutation.size() == ringSet.size())
    {
        permutations.push_back(currentPermutation);
        return;
    }

    // 遍历 ringSet 中的每个元素
    for (int32_t elem : ringSet)
    {
        // 如果当前排列不包含该元素，则将其添加到排列中
        if (std::find(currentPermutation.begin(), currentPermutation.end(), elem) == currentPermutation.end())
        {
            currentPermutation.push_back(elem);
            // 递归调用，继续生成下一个元素的排列
            generatePermutations(currentPermutation, ringSet, permutations);
            // 回溯，将当前元素从排列中移除，以生成其他排列
            currentPermutation.pop_back();
        }
    }
}

// 生成所有包含 ringSet 的排列
void generateAllPermutations(const std::set<int32_t> &ringSet, std::vector<std::vector<int32_t>> &permutations)
{
    std::vector<int32_t> currentPermutation;
    generatePermutations(currentPermutation, ringSet, permutations);
}

void BImap::MaptoBInavi(const char *IMGmap, double ratio, double rsize, const char *addr, int port) // vector<vector<CSpoint>> &returnlist)
{
    mapratio = ratio;
    robotsize = rsize;

    json data;
    data["path"] = IMGmap;
    data["ratio"] = ratio;
    data["robotsize"] = rsize;
    data["thresh"] = 128;
    std::string pretty_str = data.dump();

    /***建立通讯***/
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
    /* 将套接字和IP、端口绑定 */
    sockaddr_in addr_serv, addr_client;
    int socklen;
    memset(&addr_serv, 0, sizeof(sockaddr_in));   // 每个字节都用0填充
    memset(&addr_client, 0, sizeof(sockaddr_in)); // 每个字节都用0填充
    addr_serv.sin_family = AF_INET;               // 使用IPV4地址
    addr_serv.sin_port = htons(port);             // 端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_serv.sin_addr.s_addr = inet_addr(addr);
    socklen = sizeof(addr_serv);

    int send_num = sendto(sock_fd, pretty_str.data(), pretty_str.size(), 0, (struct sockaddr *)&addr_serv, socklen);
    if (send_num < 0)
    {
        perror("sendto error:");
        exit(1);
    }
    char recv_buf[204800];
    int recv_num = recvfrom(sock_fd, recv_buf, 204799, 0, (struct sockaddr *)&addr_serv, (socklen_t *)&socklen);
    if (recv_num < 0)
    {
        perror("recvfrom error:");
        exit(1);
    }
    recv_buf[recv_num] = 0;
    close(sock_fd);
    /***解析数据***/
    json redata = json::parse(recv_buf);
    shapeX = redata[1][1];
    shapeY = redata[1][0];
    json &mapdata = redata[0];
    BIgraphList.resize(mapdata.size());
    for (int32_t i = 0; i < mapdata.size(); i++)
    {
        BIpolygons polygons;
        polygons = mapdata[i].get<BIpolygons>();
        StartCut(polygons, BIgraphList[i]);
    }
}

void BImap::DrawBImap(bool debugmap)
{
    BIamap = cv::Mat::ones(shapeY, shapeX, CV_16U) * 0xFFFF;
    BIimap = cv::Mat::ones(shapeY, shapeX, CV_16U) * 0xFFFF;
    if (debugmap)
    {
        BIdmap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
        BIfmap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
    }
    for (int32_t graphIndex = 0; graphIndex < BIgraphList.size(); graphIndex++)
    {
        BIgraph &graph = BIgraphList[graphIndex];
        for (int32_t polyIndex = 0; polyIndex < graph.freepolygonList.size(); polyIndex++)
        {
            BIfreepolygon &freepolygon = graph.freepolygonList[polyIndex];
            BIpolygon &polygon = freepolygon.polygon;
            cv::Point PointArray[polygon.size()];
            for (int32_t pnum = 0; pnum < polygon.size(); pnum++)
            {
                PointArray[pnum].x = polygon[pnum].x;
                PointArray[pnum].y = polygon[pnum].y;
            }
            cv::fillConvexPoly(BIamap, PointArray, polygon.size(), cv::Scalar(graphIndex));
            cv::fillConvexPoly(BIimap, PointArray, polygon.size(), cv::Scalar(polyIndex));
            if (debugmap)
            {
                cv::Scalar color(255, 255, 255);
                cv::fillConvexPoly(BIfmap, PointArray, polygon.size(), color);
                color[0] = (int)(rand() / (double(RAND_MAX)) * 127) + 127;
                color[1] = (int)(rand() / (double(RAND_MAX)) * 127) + 127;
                color[2] = (int)(rand() / (double(RAND_MAX)) * 127) + 127;
                cv::fillConvexPoly(BIdmap, PointArray, polygon.size(), color);
            }
        }
        for (int32_t polyIndex = 0; polyIndex < graph.obspolygonList.size(); polyIndex++)
        {
            BIobspolygon &obspolygon = graph.obspolygonList[polyIndex];
            BIpolygon &polygon = obspolygon.polygon;
            std::vector<cv::Point> PointArray;
            PointArray.resize(polygon.size());
            for (int32_t pnum = 0; pnum < polygon.size(); pnum++)
            {
                PointArray[pnum].x = polygon[pnum].x;
                PointArray[pnum].y = polygon[pnum].y;
            }

            std::vector<std::vector<cv::Point>> contours;
            contours.push_back(PointArray);
            // cv::fillPoly(BIimap, contours, cv::Scalar(polyIndex | 0xc000));
            if (debugmap)
                cv::fillPoly(BIdmap, contours, cv::Scalar(32, 32, 32));
        }
    }
    showmat = BIdmap;
}

void BImap::RTRT_Neighbor(const BIpoint &x, std::list<BIpoint> &Xnear)
{
    if (Node_Map.count(x) == 0)
        return;
    RTRT_Neighbor(x, Xnear, Node_Map[x].invM);
}
void BImap::RTRT_Neighbor(const BIpoint &x, std::list<BIpoint> &Xnear, int32_t invM)
{
    Xnear.clear();
    // Node &xnode = Node_Map[x];

    BIgraph &graph = BIgraphList[agent_apl];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    if (invM == originIndexFun())
    {
        BIfreepolygon &freepolygon = freepolygonList[agent_island];
        // 相邻交集上的点
        for (const int32_t cutlineIndex : freepolygon.cutlinelink)
        {
            auto it = Mnode.find(cutNodeIndexFun(cutlineIndex));
            if (it == Mnode.end())
                continue;
            Xnear.insert(Xnear.end(), it->second.begin(), it->second.end());
        }
        // 当前自由空间欧式邻域上的点
        if (obsKdtreeMap.count(agent_island))
        {
            Kd_tree &kdtree = obsKdtreeMap[agent_island];
            Neighbor_search search(kdtree, {agent.x, agent.y}, nearK);
            for (const Point_with_distance pwd : search)
            {
                BIpoint pointNear = {(double)pwd.first.x(), (double)pwd.first.y()};
                Xnear.push_back({pwd.first.x(), pwd.first.y()});
            }
        }
    }
    else if (invM & 0xC0000000)
    {
        int32_t islandIndex = invM & 0x3FFFFFFF;
        BIfreepolygon &freepolygon = freepolygonList[islandIndex];

        // 相邻交集上的点
        for (const int32_t cutlineIndex : freepolygon.cutlinelink)
        {
            auto it = Mnode.find(cutNodeIndexFun(cutlineIndex));
            if (it == Mnode.end())
                continue;
            Xnear.insert(Xnear.end(), it->second.begin(), it->second.end());
        }
        // 当前自由空间欧式邻域上的点
        if (obsKdtreeMap.count(islandIndex))
        {
            Kd_tree &kdtree = obsKdtreeMap[islandIndex];
            Neighbor_search search(kdtree, {x.x, x.y}, nearK);
            for (const Point_with_distance pwd : search)
            {
                if (pwd.second < 1e-16)
                    continue;
                BIpoint pointNear = {(double)pwd.first.x(), (double)pwd.first.y()};
                Xnear.push_back({pwd.first.x(), pwd.first.y()});
            }
        }
        if (islandIndex == agent_island)
            Xnear.push_back(agent);
    }
    else
    {
        int32_t cutlineIndex = invM & 0x3FFFFFFF;
        BIcutline &cutline = cutlineList[cutlineIndex];
        // 相邻交集上的点
        for (const int32_t cutlineIndex : cutline.cutlinelink)
        {
            auto it = Mnode.find(cutNodeIndexFun(cutlineIndex));
            if (it == Mnode.end())
                continue;
            Xnear.insert(Xnear.end(), it->second.begin(), it->second.end());
        }
        // 当前自由空间欧式邻域上的点
        for (const int32_t islandIndex : cutline.polygonlink)
        {
            if (obsKdtreeMap.count(islandIndex))
            {
                Kd_tree &kdtree = obsKdtreeMap[islandIndex];
                Neighbor_search search(kdtree, {x.x, x.y}, nearK);
                for (const Point_with_distance pwd : search)
                {
                    if (pwd.second < 1e-16)
                        continue;
                    BIpoint pointNear = {(double)pwd.first.x(), (double)pwd.first.y()};
                    Xnear.push_back({pwd.first.x(), pwd.first.y()});
                }
            }
            if (islandIndex == agent_island)
                Xnear.push_back(agent);
        }
    }
}

void BImap::RTRT_start(double Resolution) // 初始化环境
{
    agent = agent_tag;
    if (agent.x >= shapeX || agent.y >= shapeY)
    {
        printf("RTRT_task error: Init point is too large!!\r\n");
        return;
    }
    uint32_t anum_agent = BIamap.at<uint16_t>(agent.y, agent.x);
    uint32_t inum_agent = BIimap.at<uint16_t>(agent.y, agent.x);
    if (anum_agent == 0xffff || (inum_agent & 0xC000))
    {
        printf("RTRT_task error: robot can't be here!!\r\n");
        return;
    }
    agent_apl = anum_agent;
    agent_island = inum_agent;

    BIgraph &graph = BIgraphList[agent_apl];
    // 初始化自由凸多边形的kdtree
    obsKdtreeMap.clear();
    // KdtreeList.resize(graph.freepolygonList.size());

    // 初始化分割线采用集
    // uint32_t BNodesNUM = 0;
    // for (uint32_t j = 0; j < graph.cutlineList.size(); j++)
    // {
    //     BIcutline &bridge = graph.cutlineList[j];
    //     BNodesNUM += (bridge.line.S % bridge.line.E) / Resolution + 2;
    // }
    // NodeBSample_List.reserve(BNodesNUM);
    NodeBSample_List.clear();
    std::vector<std::pair<BIpoint, int32_t>> SampleListtemp;
    for (uint32_t j = 0; j < graph.cutlineList.size(); j++)
    {
        BIcutline &bridge = graph.cutlineList[j];
        // int32_t BNpoint = apl.CTonly_BNlink[j];
        double dlen = bridge.line.S % bridge.line.E;
        int32_t dnum = dlen / Resolution + 4;
        if (20 < dlen)
        {
            double t = 8.0 / dlen;
            SampleListtemp.push_back({bridge.line.S * t + bridge.line.E * (1 - t), j});
            SampleListtemp.push_back({bridge.line.S * (1 - t) + bridge.line.E * t, j});
        }
        for (int32_t k = 1; k < dnum; k++)
        {
            if (2 * k == dnum)
                continue;
            double t = (double)k / dnum;
            double t_1 = 1.0f - t;
            SampleListtemp.push_back({bridge.line.S * t_1 + bridge.line.E * t, j});
        }
    }
    std::random_shuffle(SampleListtemp.begin(), SampleListtemp.end());
    NodeBSample_List.insert(NodeBSample_List.end(), SampleListtemp.begin(), SampleListtemp.end());

    // 构建初始树
    Mnode.clear();
    Node_Map.clear();
    Node_Map[agent].invM = originIndexFun();
    Node_Map[agent].cost = {0, 0};
    Node_Map[agent].par = {-1, -1};
    Node_Map[agent].subs = {};
    std::priority_queue<VEpair, std::vector<VEpair>, std::greater<VEpair>> pQInit;
    for (int32_t cutlineIndex = 0; cutlineIndex < graph.cutlineList.size(); cutlineIndex++)
    {
        BIcutline &bridge_now = graph.cutlineList[cutlineIndex];
        BIpoint node_now = bridge_now.core;
        Node_Map[node_now] = {{1, doubleMax}, {-1, -1}, {}, cutNodeIndexFun(cutlineIndex)};
        Mnode[cutNodeIndexFun(cutlineIndex)].insert(node_now);
    }

    std::set<BIpoint> existPoint;
    existPoint.insert(agent);
    std::list<BIpoint> agentNears;
    RTRT_Neighbor(agent, agentNears);
    for (const BIpoint &xNear : agentNears)
    {
        pQInit.push({CTrtrt_Cost{false, agent % xNear}, {agent, xNear}});
    }
    while (pQInit.size())
    {
        CTrtrt_Cost cost = pQInit.top().first;
        BIpoint pointS = pQInit.top().second.S;
        BIpoint pointE = pQInit.top().second.E;
        pQInit.pop();
        if (existPoint.count(pointE))
            continue;
        Node &nodeE = Node_Map[pointE];
        if (nodeE.cost > cost)
        {
            nodeE.par = pointS;
            nodeE.cost = cost;
            Node_Map[pointS].subs.insert(pointE);

            existPoint.insert(pointE);
            std::list<BIpoint> xNears;
            RTRT_Neighbor(pointE, xNears);
            for (const BIpoint &xNear : xNears)
            {
                if (existPoint.count(xNear))
                    continue;
                pQInit.push({cost + CTrtrt_Cost{false, pointE % xNear}, {pointE, xNear}});
            }
        }
    }
}
void BImap::RTRT_SetAgent(const BIpoint &point) // 设置机器人当前位置
{
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("SetAgent error: Init point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(point.y, point.x); //|| DObsmap.at<uint16_t>(point.y, point.x)
    uint32_t inum_point = BIimap.at<uint16_t>(point.y, point.x);
    if (anum_S == 0xffff || (inum_point & 0xC000))
    {
        printf("SetAgent error: robot can't be here!!\r\n");
        return;
    }

    agent_tag = point;
}

void BImap::RTRT_SetGoal(const BIpoint &point)
{
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("SetGoal error: Init point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(point.y, point.x); //|| DObsmap.at<uint16_t>(point.y, point.x)
    uint32_t inum_point = BIimap.at<uint16_t>(point.y, point.x);
    if (anum_S == 0xffff || (inum_point & 0xC000))
    {
        printf("SetGoal error: robot can't be here!!\r\n");
        return;
    }

    goal = point;
    goalIsSet = true;
}

void BImap::RTRT_AddDynamicObstacle(int num, CTrtrt_DynamicObstacle &obs, double CheckLen)
{
    DObstacles[num] = obs;

    if (CheckLen > 0)
    {
        CTrtrt_DynamicObstacle &nowobs = DObstacles[num];
        std::vector<BIpoint> &OutLine = nowobs.OutLine;
        std::vector<BIpoint> &CheckPoints = nowobs.CheckPoints;
        CheckPoints.resize(0);
        if (nowobs.DObsType == 0)
        {
            float radius = nowobs.radius;
            int n = 2 * 3.14159265f * radius / CheckLen;
            float dangle = 2 * 3.14159265f / n;
            CheckPoints.reserve(OutLine.size() * n);
            for (const BIpoint &obspoint : OutLine)
            {
                for (int i = 0; i < n; i++)
                {
                    float x = sinf32(dangle * i) * radius + obspoint.x;
                    float y = cosf32(dangle * i) * radius + obspoint.y;
                    CheckPoints.push_back({x, y});
                }
            }
        }
        else if (nowobs.DObsType == 1)
        {
            uint32_t N = 0;
            int vertex = 0;
            int vertex_old = vertex;
            for (++vertex; vertex < OutLine.size(); ++vertex)
            {
                float dx = OutLine[vertex].x - OutLine[vertex_old].x;
                float dy = OutLine[vertex].y - OutLine[vertex_old].y;
                float length = std::sqrt(dx * dx + dy * dy);
                N += (uint32_t)(length / CheckLen) + 1;
                vertex_old = vertex;
            }
            {
                float dx = OutLine[0].x - OutLine[vertex_old].x;
                float dy = OutLine[0].y - OutLine[vertex_old].y;
                float length = std::sqrt(dx * dx + dy * dy);
                N += (uint32_t)(length / CheckLen) + 1;
            }
            CheckPoints.resize(N);
            N = 0;
            uint32_t n = 0;
            vertex = 0;
            vertex_old = vertex;
            for (++vertex; vertex < OutLine.size(); ++vertex)
            {
                float dx = OutLine[vertex].x - OutLine[vertex_old].x;
                float dy = OutLine[vertex].y - OutLine[vertex_old].y;
                float length = std::sqrt(dx * dx + dy * dy);
                uint32_t n = (uint32_t)(length / CheckLen) + 1;
                for (uint32_t i = 0; i < n; i++)
                {
                    float t = (float)(i) / n;
                    CheckPoints[N + i] = {OutLine[vertex_old].x + t * dx, OutLine[vertex_old].y + t * dy};
                }
                vertex_old = vertex;
                N += n;
            }
            {
                float dx = OutLine[0].x - OutLine[vertex_old].x;
                float dy = OutLine[0].y - OutLine[vertex_old].y;
                float length = std::sqrt(dx * dx + dy * dy);
                uint32_t n = (uint32_t)(length / CheckLen) + 1;
                for (uint32_t i = 0; i < n; i++)
                {
                    float t = (float)(i) / n;
                    CheckPoints[N + i] = {OutLine[vertex_old].x + t * dx, OutLine[vertex_old].y + t * dy};
                }
                vertex_old = vertex;
                N += n;
            }
        }
    }
}

void BImap::RTRT_DelDynamicObstacle(int num)
{
    DObstacles.erase(num);
}

bool BImap::RTRT_CrashDetection(const BIpoint &ps, int32_t invMps, const BIpoint &pe, int32_t invMpe)
{
    BIgraph &graph = BIgraphList[agent_apl];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;
    // std::map<BIinvnode, int32_t> &invnode2cutlineMap = ;

    // 判断在邻域图中ps pe是否相邻
    bool obsCras = false;
    int32_t subIndex_ps = invMps & 0x3FFFFFFF;
    int32_t subIndex_pe = invMpe & 0x3FFFFFFF;
    switch (invMps & 0xC0000000)
    {
    case 0x00000000: // cutNode
        switch (invMpe & 0xC0000000)
        {
        case 0x00000000: // cutNode
        {
            if (cutlineList[subIndex_ps].cutlinelink.count(subIndex_pe) == 0)
                return true;
            int32_t island_temp = *cutlineList[subIndex_ps].polygonlink.begin();
            if (!cutlineList[subIndex_pe].polygonlink.count(island_temp))
                island_temp = *cutlineList[subIndex_ps].polygonlink.rbegin();
            if (obsKdtreeMap.count(island_temp))
                obsCras = true;
        }
        break;
        case 0x40000000: // origin
            if (freepolygonList[agent_island].cutlinelink.count(subIndex_ps) == 0)
                return true;
            if (obsKdtreeMap.count(agent_island))
                obsCras = true;
            break;
        case 0x80000000: // obsNode
            if (cutlineList[subIndex_ps].polygonlink.count(subIndex_pe) == 0)
                return true;
            if (obsKdtreeMap.count(subIndex_pe))
                obsCras = true;
            break;
        }
        break;
    case 0x40000000: // origin
        switch (invMpe & 0xC0000000)
        {
        case 0x00000000: // cutNode
            if (freepolygonList[agent_island].cutlinelink.count(subIndex_pe) == 0)
                return true;
            break;
        case 0x80000000: // obsNode
            if (agent_island != subIndex_pe)
                return true;
            break;
        }
        if (obsKdtreeMap.count(agent_island))
            obsCras = true;
        break;
    case 0x80000000: // obsNode
        switch (invMpe & 0xC0000000)
        {
        case 0x00000000: // cutNode
            if (freepolygonList[subIndex_ps].cutlinelink.count(subIndex_pe) == 0)
                return true;
            break;
        case 0x40000000: // origin
            if (agent_island != subIndex_ps)
                return true;
            break;
        case 0x80000000: // obsNode
            if (subIndex_ps != subIndex_pe)
                return true;
            break;
        }
        if (obsKdtreeMap.count(subIndex_ps))
            obsCras = true;
        break;
    }

    if (obsCras)
    {
        if (DObsmap.at<uint8_t>(ps.y, ps.x) || DObsmap.at<uint8_t>(pe.y, pe.x))
            return true;
        float len = ps % pe;
        int32_t MaximumDiscrete = len / 10 + 2; //$$$
        for (float denominator = 2; denominator <= MaximumDiscrete; denominator *= 2)
        {
            for (int32_t i = 1; i < denominator; i += 2)
            {
                float t = i / denominator;
                int32_t x = ps.x * t + (1 - t) * pe.x;
                int32_t y = ps.y * t + (1 - t) * pe.y;
                if (DObsmap.at<uint8_t>(y, x))
                    return true;
            }
        }
    }
    return false;
}

void BImap::RTRT_DrawDObsmap(void)
{
    DObsmap = cv::Mat::zeros(shapeY, shapeX, CV_8U);

    for (const auto &pair : DObstacles)
    {
        const CTrtrt_DynamicObstacle &obs = pair.second;
        float sinangle = sinf32(obs.angle);
        float cosangle = cosf32(obs.angle);
        if (obs.DObsType == 0)
        {
            for (const BIpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                cv::circle(DObsmap, cv::Point(imgx, imgy), obs.radius, cv::Scalar(127), cv::FILLED);
            }
        }
        else if (obs.DObsType == 1)
        {
            std::vector<std::vector<cv::Point>> polygons(1);
            polygons[0].reserve(obs.OutLine.size() + 5);
            for (const BIpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                polygons[0].push_back({imgx, imgy});
            }
            cv::fillPoly(DObsmap, polygons, cv::Scalar(127));
        }
    }
}

void BImap::RTRT_CrashDObsIsland(void)
{
    BIgraph &graph = BIgraphList[agent_apl];
    std::vector<BIcutline> &cutlineList = graph.cutlineList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;

    std::set<int32_t> ObsIslands_temp;
    for (auto &pair : DObstacles)
    {
        CTrtrt_DynamicObstacle &obs = pair.second;
        obs.ObsIslands.clear();
        float sinangle = sinf32(obs.angle);
        float cosangle = cosf32(obs.angle);
        int32_t inum_old = -1;
        for (const BIpoint &point : obs.CheckPoints)
        {
            int imgx = cosangle * point.x - sinangle * point.y + obs.x;
            int imgy = sinangle * point.x + cosangle * point.y + obs.y;
            if (imgx >= shapeX || imgy >= shapeY || imgx < 0 || imgy < 0)
                continue;
            uint32_t anum = BIamap.at<uint16_t>(imgy, imgx);
            if (anum != agent_apl)
                continue;
            int32_t inum = BIimap.at<uint16_t>(imgy, imgx);
            if (!(inum & 0xC000) && inum != inum_old)
            {
                obs.ObsIslands.insert(inum);
                ObsIslands_temp.insert(inum);
                inum_old = inum;
            }
        }
    }
    // 删除global_ObsIslands中在ObsIslands_temp中没有的键值对
    std::set<int32_t> ObsIslands_Diff;
    for (auto it = obsKdtreeMap.begin(); it != obsKdtreeMap.end(); ++it)
    {

        // std::set<int32_t>::iterator IslandDiff_it = ObsIslands_temp.find(it->first);
        if (ObsIslands_temp.count(it->first))
            ObsIslands_temp.erase(it->first);
        else
            ObsIslands_Diff.insert(it->first);
    }
    std::set<int32_t> bridgesNear;
    std::set<int32_t> bridgeNearSearched;
    for (int32_t IslandIndex : ObsIslands_Diff)
    {
        for (int32_t bridgeNearIndex : freepolygonList[IslandIndex].cutlinelink)
        {
            if (bridgeNearSearched.count(bridgeNearIndex))
                continue;
            for (const BIpoint &x : Mnode[cutNodeIndexFun(bridgeNearIndex)])
            {
                Node &xnode = Node_Map[x];
                Node &xparnode = Node_Map[xnode.par];
                // int32_t parIndex
                if (((xparnode.invM & 0xC0000000) == 0x80000000) &&
                    ObsIslands_Diff.count(xparnode.invM & 0x3FFFFFFF))
                {
                    BIpoint *x_p = &xnode.par;
                    for (;;)
                    {
                        Node &xpnode = Node_Map[*x_p];
                        if ((xpnode.invM & 0xC0000000) != 0x80000000)
                            break;
                        x_p = &Node_Map[*x_p].par;
                    }
                    xnode.par = *x_p;
                    Node_Map[*x_p].subs.insert(x);
                }
            }
            bridgeNearSearched.insert(bridgeNearIndex);
        }
    }

    for (int32_t IslandIndex : ObsIslands_Diff)
    {
        // std::set<BIpoint> &MX_I = Mnode[obsNodeIndexFun(IslandIndex)];
        Kd_tree &MX_I = obsKdtreeMap[IslandIndex];
        for (const Point_2 &p : MX_I)
        {
            Node_Map.erase({p.x(), p.y()});
        }
        // Mnode.erase(obsNodeIndexFun(IslandIndex));
        obsKdtreeMap.erase(IslandIndex);
    }
    bridgeNearSearched.clear();
    for (int32_t IslandIndex : ObsIslands_Diff)
    {
        if (IslandIndex == agent_island)
        {
            Node &xnode = Node_Map[agent];
            std::set<BIpoint> &xnode_subs = xnode.subs;
            for (auto it = xnode_subs.begin(); it != xnode_subs.end();)
            {
                if (!Node_Map.count(*it))
                    it = xnode_subs.erase(it);
                else
                    ++it;
            }
        }
        for (int32_t bridgeNearIndex : freepolygonList[IslandIndex].cutlinelink)
        {
            if (bridgeNearSearched.count(bridgeNearIndex))
                continue;
            for (const BIpoint &x : Mnode[cutNodeIndexFun(bridgeNearIndex)])
            {
                Node &xnode = Node_Map[x];
                std::set<BIpoint> &xnode_subs = xnode.subs;
                for (auto it = xnode_subs.begin(); it != xnode_subs.end();)
                {
                    if (!Node_Map.count(*it))
                        it = xnode_subs.erase(it);
                    else
                        ++it;
                }
            }
            bridgeNearSearched.insert(bridgeNearIndex);
        }
    }

    // 将ObsIslands_temp中有但obsKdtreeMap中没有的键值对添加到obsKdtreeMap中
    for (int key : ObsIslands_temp)
        obsKdtreeMap[key];
}

void BImap::RTRT_UpdateTreeCost(void) // 更新树的代价使其一致
{
    /*********************************更新点代价*********************************/
    std::queue<BIpoint> NodeUpdateQueue;
    NodeUpdateQueue.push(agent);
    while (NodeUpdateQueue.size())
    {
        BIpoint Pnow = NodeUpdateQueue.front();
        Node &Pnow_Node = Node_Map[Pnow];
        for (const BIpoint &Psub : Pnow_Node.subs)
        {
            Node &Psub_Node = Node_Map[Psub];
            CTrtrt_Cost cost_temp = Pnow_Node.cost +
                                    CTrtrt_Cost{RTRT_CrashDetection(Pnow, Pnow_Node.invM,
                                                                    Psub, Psub_Node.invM),
                                                Pnow % Psub};

            if (Psub_Node.cost < cost_temp)
            {
                largerSet.insert(Psub);
                smallerSet.erase(Psub);
            }
            else if (cost_temp < Psub_Node.cost)
            {
                smallerSet.insert(Psub);
                // largerSet.erase(Psub);
            }
            Psub_Node.cost = cost_temp;
            NodeUpdateQueue.push(Psub);
        }
        NodeUpdateQueue.pop();
    }
}
void BImap::RTRT_AddPotentialLinks(const BIpoint &p, bool larger)
{
    std::list<BIpoint> Xnear;
    RTRT_Neighbor(p, Xnear);
    Node &p_node = Node_Map[p];
    if (larger)
    {
        for (const BIpoint &p_near : Xnear)
        {
            Node &pnear_node = Node_Map[p_near];
            CTrtrt_Cost costh_temp = pnear_node.cost + CTrtrt_Cost{0, p % p_near};
            if (p_node.cost > costh_temp)
            {
                if (goalIsSet)
                    costh_temp.cost += (p % goal);
                eQr.push({costh_temp, {p_near, p}});
            }
        }
    }
    else
    {
        for (const BIpoint &p_near : Xnear)
        {
            Node &pnear_node = Node_Map[p_near];
            CTrtrt_Cost costh_temp = p_node.cost + CTrtrt_Cost{0, p % p_near};
            if (pnear_node.cost > costh_temp)
            {
                if (goalIsSet)
                    costh_temp.cost += (p_near % goal);
                eQr.push({costh_temp, {p, p_near}});
            }
        }
    }
}

bool BImap::RTRT_Sampling(BIpoint &randpoint, std::list<BIpoint> &Xnear)
{
    float random_p1 = rand() / (float(RAND_MAX));
    if (random_p1 < p_th || obsKdtreeMap.size() == 0) // 在桥上采样
    {
        if (NodeBSample_List.size() == 0)
            return false;
        randpoint = NodeBSample_List.front().first;
        int32_t bridgeIndex = NodeBSample_List.front().second;
        NodeBSample_List.pop_front();
        Node &randNode = Node_Map[randpoint];
        randNode.invM = cutNodeIndexFun(bridgeIndex);
        RTRT_Neighbor(randpoint, Xnear);
        CTrtrt_Cost cost_min = {true, doubleMax};
        BIpoint par_min = {-1, -1};
        for (const BIpoint &x_near : Xnear)
        {
            Node &xnearNode = Node_Map[x_near];
            CTrtrt_Cost cost_temp = xnearNode.cost +
                                    CTrtrt_Cost{RTRT_CrashDetection(randpoint, randNode.invM,
                                                                    x_near, xnearNode.invM),
                                                randpoint % x_near};
            if (cost_min > cost_temp)
            {
                cost_min = cost_temp;
                par_min = x_near;
            }
        }
        randNode.cost = cost_min;
        randNode.par = par_min;
        Node_Map[par_min].subs.insert(randpoint);
        Mnode[cutNodeIndexFun(bridgeIndex)].insert(randpoint);

        return true;
    }
    else
    {
        for (size_t errCount = 0; errCount < 20; errCount++) // 采样容错次数//@@@
        {
            bool valkey = false;
            double x, y;
            int randIsland_num;
            for (size_t rejectCount = 0; rejectCount < 100; rejectCount++)
            {
                x = rand() / (float(RAND_MAX)) * shapeX;
                y = rand() / (float(RAND_MAX)) * shapeY;
                int imgx = (int)x, imgy = (int)y;
                if (DObsmap.at<uint8_t>(imgy, imgx) || BIamap.at<uint16_t>(imgy, imgx) != agent_apl)
                    continue;
                randIsland_num = BIimap.at<uint16_t>(imgy, imgx);
                if (obsKdtreeMap.find(randIsland_num) != obsKdtreeMap.end())
                {
                    valkey = true;
                    break;
                }
            }
            if (valkey)
            {
                Neighbor_search search(obsKdtreeMap[randIsland_num], {x, y}, 1);
                if (search.begin() != search.end() && search.begin()->second < 10 * r_th * r_th)
                    continue;
                randpoint.x = x;
                randpoint.y = y;
                Node &randNode = Node_Map[randpoint];
                randNode.invM = obsNodeIndexFun(randIsland_num);

                RTRT_Neighbor(randpoint, Xnear);

                CTrtrt_Cost cost_min = {true, doubleMax};
                BIpoint par_min = {-1, -1};
                for (const BIpoint &x_near : Xnear)
                {
                    Node &xnearNode = Node_Map[x_near];
                    CTrtrt_Cost cost_temp = xnearNode.cost +
                                            CTrtrt_Cost{RTRT_CrashDetection(randpoint, randNode.invM,
                                                                            x_near, xnearNode.invM),
                                                        randpoint % x_near};
                    if (cost_min > cost_temp)
                    {
                        cost_min = cost_temp;
                        par_min = x_near;
                    }
                }
                randNode.cost = cost_min;
                randNode.par = par_min;
                Node_Map[par_min].subs.insert(randpoint);
                obsKdtreeMap[randIsland_num].insert({x, y});
                return true;
            }
        }
    }

    return false;
}

void BImap::RTRT_task(void) // 规划器主程序
{
    static int64_t updateTimeOld = utime_ns();
    static int64_t smallerTimeOld = utime_ns();
    int64_t Time0 = utime_ns();

    BIgraph &graph = BIgraphList[agent_apl];
    std::vector<BIobspolygon> &obspolygonList = graph.obspolygonList;
    std::vector<BIfreepolygon> &freepolygonList = graph.freepolygonList;

    /********************************更新环境信息********************************/
    RTRT_DrawDObsmap();     // 更新碰撞检测地图
    RTRT_CrashDObsIsland(); // 更新碰撞检测岛

    if (agent % agent_tag > r_th || utime_ns() - updateTimeOld > 50e6)
    {
        updateTimeOld = utime_ns();

        if (agent != agent_tag)
        {
            int32_t agenttag_island = BIimap.at<uint16_t>(agent_tag.y, agent_tag.x);
            if (agent_island == agenttag_island)
            {
                Node &agenttag_ndoe = Node_Map[agent_tag];
                agenttag_ndoe.invM = originIndexFun();
                agenttag_ndoe.cost = {0, 0}; //@@@
                agenttag_ndoe.par = {-1, -1};
                agenttag_ndoe.subs = Node_Map[agent].subs;
                Node_Map.erase(agent);
                for (const BIpoint &psub : agenttag_ndoe.subs)
                    Node_Map[psub].par = agent_tag;
            }
            else
            {
                std::list<BIpoint> path;
                GetPath(agent_tag, path);
                Node &agent_ndoe = Node_Map[agent];
                path.pop_front();
                BIpoint x1 = path.front();
                Node &x1node = Node_Map[x1];
                for (const BIpoint &psub : agent_ndoe.subs)
                {
                    if (psub == x1)
                        continue;
                    Node_Map[psub].par = x1;
                    x1node.subs.insert(psub);
                }
                Node_Map.erase(agent);

                Node &agenttag_ndoe = Node_Map[agent_tag];
                agenttag_ndoe.invM = originIndexFun();
                agenttag_ndoe.cost = {0, 0};
                agenttag_ndoe.par = {-1, -1};

                auto pathit = path.begin();
                auto pathit_old = pathit;
                for (++pathit; pathit != path.end(); ++pathit)
                {
                    Node_Map[*pathit].subs.insert(*pathit_old);
                    Node_Map[*pathit_old].subs.erase(*pathit);
                    Node_Map[*pathit_old].par = *pathit;
                    pathit_old = pathit;
                }
            }
            agent_island = agenttag_island;
            agent = agent_tag;
        }
        RTRT_UpdateTreeCost();
    }
    for (const BIpoint &p_larger : largerSet)
    {
        if (Node_Map.count(p_larger))
            RTRT_AddPotentialLinks(p_larger, true);
    }
    largerSet.clear();
    if (utime_ns() - smallerTimeOld > 100e6)
    {
        smallerTimeOld = utime_ns();
        for (const BIpoint &p_smaller : smallerSet)
        {
            if (Node_Map.count(p_smaller))
                RTRT_AddPotentialLinks(p_smaller, false);
        }
        smallerSet.clear();
    }
    for (int32_t SampleCount = 0; SampleCount < RewireN; SampleCount++)
    {
        BIpoint x_rand;
        std::list<BIpoint> Xnear;
        bool SamplingSuccessful = RTRT_Sampling(x_rand, Xnear);
        if (SamplingSuccessful)
        {
            Node &xrand_node = Node_Map[x_rand];
            for (const BIpoint &x_near : Xnear)
            {
                Node &xnear_node = Node_Map[x_near];
                CTrtrt_Cost costh_temp = xrand_node.cost + CTrtrt_Cost{0, x_near % x_rand};

                if (xnear_node.cost > costh_temp)
                {
                    if (goalIsSet)
                        costh_temp.cost += (x_near % goal);
                    eQr.push({costh_temp, {x_rand, x_near}});
                }
            }
        }
    }

    int64_t Rewire_t0 = utime_ns();
    while (eQr.size() && utime_ns() - Rewire_t0 < 20e6)
    {
        VEpair ve_top = eQr.top();
        eQr.pop();
        BIedge &edge_top = ve_top.second;
        const BIpoint &point_topS = edge_top.S;
        const BIpoint &point_topE = edge_top.E;
        if (Node_Map.count(point_topS) == 0 || Node_Map.count(point_topE) == 0)
            continue;
        Node &pointNode_Stop = Node_Map[point_topS];
        Node &pointNode_Etop = Node_Map[point_topE];
        CTrtrt_Cost hatCost = pointNode_Stop.cost +
                              CTrtrt_Cost{0, point_topS % point_topE};
        if (pointNode_Etop.cost > hatCost)
        {
            CTrtrt_Cost trueCost = hatCost + CTrtrt_Cost{
                                                 RTRT_CrashDetection(point_topS, pointNode_Stop.invM,
                                                                     point_topE, pointNode_Etop.invM),
                                                 0};
            if (pointNode_Etop.cost > trueCost)
            {
                // 更新当前节点的相关属性
                pointNode_Etop.cost = trueCost;
                // 更新树的链接状态
                Node_Map[pointNode_Etop.par].subs.erase(point_topE);
                Node_Map[point_topS].subs.insert(point_topE);
                pointNode_Etop.par = point_topS;
                // 添加待优化节点的潜在边
                RTRT_AddPotentialLinks(point_topE, false);
            }
        }
    }
}
CTrtrt_Cost BImap::GetPath(void)
{
    if (goalIsSet)
        return GetPath(goal, agent_path);
    else
        return {true, doubleMax};
}
CTrtrt_Cost BImap::GetPath(BIpoint &point, std::list<BIpoint> &path)
{
    path.clear();
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("GetPath error: Init point is too large!!\r\n");
        return {true, doubleMax};
    }
    uint32_t anum_G = BIamap.at<uint16_t>(point.y, point.x); //|| DObsmap.at<uint16_t>(point.y, point.x)
    uint32_t inum_point = BIimap.at<uint16_t>(point.y, point.x);
    if (anum_G == 0xffff || (inum_point & 0xC000))
    {
        printf("GetPath error: robot can't be here!!\r\n");
        return {true, doubleMax};
    }
    std::list<BIpoint> Xnear;
    RTRT_Neighbor(point, Xnear, obsNodeIndexFun(inum_point));

    CTrtrt_Cost cost_min = {true, doubleMax};
    BIpoint par_min = {-1, -1};
    for (const BIpoint &x_near : Xnear)
    {
        Node &xnearNode = Node_Map[x_near];
        CTrtrt_Cost cost_temp = xnearNode.cost +
                                CTrtrt_Cost{RTRT_CrashDetection(point, obsNodeIndexFun(inum_point),
                                                                x_near, xnearNode.invM),
                                            point % x_near};
        if (cost_min > cost_temp)
        {
            cost_min = cost_temp;
            par_min = x_near;
        }
    }

    path.push_front(point);
    BIpoint x_i = par_min;
    while (x_i.x >= 0)
    {
        path.push_front(x_i);
        x_i = Node_Map[x_i].par;
        if (path.size() > 2000)
        {
            path.clear();
            return {1, 3.4e38f};
        }
    }
    return cost_min;
}

BImap::BImap()
{
}
BImap::~BImap()
{
}

void drawDObstacles(std::map<int, CTrtrt_DynamicObstacle> &DObstacles, cv::Mat image)
{
    for (const auto &pair : DObstacles)
    {
        const CTrtrt_DynamicObstacle &obs = pair.second;
        float sinangle = sinf32(obs.angle);
        float cosangle = cosf32(obs.angle);
        if (obs.DObsType == 0)
        {
            for (const BIpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                cv::circle(image, cv::Point(imgx, imgy), obs.radius, cv::Scalar(255, 0, 255), cv::FILLED);
            }
        }
        else if (obs.DObsType == 1)
        {
            std::vector<std::vector<cv::Point>> polygons(1);
            polygons[0].reserve(obs.OutLine.size() + 5);
            for (const BIpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                polygons[0].push_back({imgx, imgy});
            }
            cv::fillPoly(image, polygons, cv::Scalar(255, 0, 255));
        }
    }
}
