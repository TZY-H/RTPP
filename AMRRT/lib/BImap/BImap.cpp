#include "BImap.h"
// #include <jsoncpp/json/json.h>
// #include "configor/json.hpp"
#include <math.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

using namespace std;
// using namespace configor;
using json = nlohmann::json;
cv::Mat showmat;
long long utime_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<long long>(1000000000UL) * static_cast<long long>(ts.tv_sec) +
           static_cast<long long>(ts.tv_nsec);
}

void BImap::MaptoBInavi(char *IMGmap, float ratio, float rsize) // vector<vector<CSpoint>> &returnlist)
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
    addr_serv.sin_port = htons(23231);            // 端口
    /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
    addr_serv.sin_addr.s_addr = inet_addr("127.0.0.1");
    socklen = sizeof(addr_serv);
    // /* 绑定socket */
    // if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    // {
    //     perror("bind error:");
    //     exit(1);
    // }
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
    apllist.resize(mapdata.size());
    for (uint32_t i = 0; i < mapdata.size(); i++)
    {
        vector<CSpoint> points;
        json &csdata = mapdata[i];
        uint32_t psize = csdata.size();
        points.resize(psize);
        for (uint32_t j = 0; j < psize; j++)
        {
            points[j].x = csdata[j][0];
            points[j].y = csdata[j][1];
            // points[j].x = csdata[j][0].as_float();
            // points[j].y = csdata[j][1].as_float();
        }

        CSubdivision CStemp(points);
        CStemp.StartCut();
        CStemp.GetPloygon();
        CSsetBInavi(CStemp, apllist[i], i);
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
        ShowMap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
    }
    for (uint32_t aplnum = 0; aplnum < apllist.size(); aplnum++)
    {
        BIarchipelago &apl = apllist[aplnum];
        std::vector<BIpoint> &pointmap = apllist[aplnum].pointmap;
        for (uint32_t inum = 0; inum < apl.islandlistlen; inum++)
        {
            std::vector<int32_t> &points = apl.islandlist[inum].points;
            cv::Point PointArray[apl.islandlist[inum].pointslen];
            uint32_t pnum = apl.islandlist[inum].pointslen;
            PointArray[0].x = pointmap[points[0]].x;
            PointArray[0].y = pointmap[points[0]].y;
            while (--pnum)
            {
                PointArray[pnum].x = pointmap[points[pnum]].x;
                PointArray[pnum].y = pointmap[points[pnum]].y;
            }
            cv::fillConvexPoly(BIamap, PointArray, apl.islandlist[inum].pointslen, cv::Scalar(aplnum));
            cv::fillConvexPoly(BIimap, PointArray, apl.islandlist[inum].pointslen, cv::Scalar(inum));
            if (debugmap)
            {
                cv::Scalar color(255, 255, 255);
                cv::fillConvexPoly(BIfmap, PointArray, apl.islandlist[inum].pointslen, color);
                color[0] = (int)(rand() / (float(RAND_MAX)) * 255);
                color[1] = (int)(rand() / (float(RAND_MAX)) * 255);
                color[2] = (int)(rand() / (float(RAND_MAX)) * 255);
                cv::fillConvexPoly(BIdmap, PointArray, apl.islandlist[inum].pointslen, color);
            }
        }
    }
}
void BImap::DrawBImap(bool debugmap, std::vector<int32_t> &BrokenBridges, std::vector<int32_t> &BrokenIslands)
{
    BIamap = cv::Mat::ones(shapeY, shapeX, CV_16U) * 0xFFFF;
    BIimap = cv::Mat::ones(shapeY, shapeX, CV_16U) * 0xFFFF;
    if (debugmap)
    {
        BIdmap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
        BIfmap = cv::Mat::zeros(shapeY, shapeX, CV_8UC3);
    }
    for (uint32_t aplnum = 0; aplnum < apllist.size(); aplnum++)
    {
        BIarchipelago &apl = apllist[aplnum];
        std::vector<BIpoint> &pointmap = apllist[aplnum].pointmap;
        for (uint32_t inum = 0; inum < apl.islandlistlen; inum++)
        {
            std::vector<int32_t> &points = apl.islandlist[inum].points;
            cv::Point PointArray[apl.islandlist[inum].pointslen];
            uint32_t pnum = apl.islandlist[inum].pointslen;
            float Cx, Cy;
            Cx = PointArray[0].x = pointmap[points[0]].x;
            Cy = PointArray[0].y = pointmap[points[0]].y;
            while (--pnum)
            {
                PointArray[pnum].x = pointmap[points[pnum]].x;
                PointArray[pnum].y = pointmap[points[pnum]].y;
                Cx += PointArray[pnum].x;
                Cy += PointArray[pnum].y;
            }
            Cx /= apl.islandlist[inum].pointslen;
            Cy /= apl.islandlist[inum].pointslen;
            cv::fillConvexPoly(BIamap, PointArray, apl.islandlist[inum].pointslen, cv::Scalar(aplnum));
            cv::fillConvexPoly(BIimap, PointArray, apl.islandlist[inum].pointslen, cv::Scalar(inum));

            if (debugmap && BrokenIslands[inum])
            {
                cv::Scalar color(255, 255, 255);
                cv::fillConvexPoly(BIfmap, PointArray, apl.islandlist[inum].pointslen, color);
                color[0] = (int)(rand() / (float(RAND_MAX)) * 128);
                color[1] = (int)(rand() / (float(RAND_MAX)) * 128);
                color[2] = (int)(rand() / (float(RAND_MAX)) * 128);
                cv::fillConvexPoly(BIdmap, PointArray, apl.islandlist[inum].pointslen, color);
            }
        }
    }
}

void BImap::CSsetBInavi(CSubdivision &csdata, BIarchipelago &apl, uint32_t num)
{
    float dx, dy;
    apl.aplnum = num;
    apl.pointmaplen = csdata.PointMaplen;
    apl.pointmap.resize(apl.pointmaplen); // = (BIpoint *)malloc(sizeof(BIpoint) * apl.pointmaplen);
    for (uint32_t i = 0; i < csdata.PointMaplen; i++)
        apl.pointmap[i] = {csdata.PointMap[i].x, csdata.PointMap[i].y};
    apl.islandlistlen = csdata.islandlist.size();
    apl.bridgelistlen = csdata.bridgelist.size();
    apl.islandlist.resize(apl.islandlistlen); // = (BIisland *)malloc(sizeof(BIisland) * apl.islandlistlen);
    apl.bridgelist.resize(apl.bridgelistlen); // = (BIbridge *)malloc(sizeof(BIbridge) * apl.bridgelistlen);
    for (uint32_t bnum = 0; bnum < apl.bridgelistlen; bnum++)
    {
        BIbridge &bridge = apl.bridgelist[bnum];
        CSpoint &bridgetempS = csdata.PointMap[csdata.cutlinelist[bnum].S];
        CSpoint &bridgetempE = csdata.PointMap[csdata.cutlinelist[bnum].E];
        bridge.pointS = csdata.cutlinelist[bnum].S;
        bridge.pointE = csdata.cutlinelist[bnum].E;
        dx = bridgetempE.x - bridgetempS.x;
        dy = bridgetempE.y - bridgetempS.y;
        bridge.length = sqrtf(dx * dx + dy * dy);
        bridge.core.x = (bridgetempS.x + bridgetempE.x) / 2;
        bridge.core.y = (bridgetempS.y + bridgetempE.y) / 2;
        bridge.Ilinklen = csdata.bridgelist[bnum].size();
        bridge.Ilink.resize(bridge.Ilinklen); // = (int32_t *)malloc(sizeof(int32_t) * bridge.Ilinklen);
        for (uint32_t linknum = 0; linknum < bridge.Ilinklen; linknum++)
            bridge.Ilink[linknum] = csdata.bridgelist[bnum][linknum];
        bridge.Blinklen = 0;
        for (uint32_t i = 0; i < bridge.Ilinklen; i++)
            bridge.Blinklen += csdata.islandlist[csdata.bridgelist[bnum][i]].cutline.size() - 1;
        bridge.Blink.resize(bridge.Blinklen); // = (int32_t *)malloc(sizeof(int32_t) * bridge.Blinklen);
        uint32_t count = 0;
        for (uint32_t i = 0; i < bridge.Ilinklen; i++)
        {
            std::vector<uint32_t> &cuttemp = csdata.islandlist[csdata.bridgelist[bnum][i]].cutline;
            for (uint32_t j = 0; j < cuttemp.size(); j++)
            {
                uint32_t &bias = cuttemp[j];
                if (bias != bnum)
                    bridge.Blink[count++] = bias;
            }
        }
    }
    for (uint32_t inum = 0; inum < apl.islandlistlen; inum++)
    {
        BIisland &island = apl.islandlist[inum];

        island.pointslen = csdata.islandlist[inum].point.size();
        island.points.resize(island.pointslen); // = (int32_t *)malloc(sizeof(int32_t) * island.pointslen);
        for (uint32_t pointnum = 0; pointnum < island.pointslen; pointnum++)
            island.points[pointnum] = csdata.islandlist[inum].point[pointnum];

        island.Blinklen = csdata.islandlist[inum].cutline.size();
        island.Blink.resize(island.Blinklen); // = (int32_t *)malloc(sizeof(int32_t) * island.Blinklen);
        for (uint32_t linknum = 0; linknum < island.Blinklen; linknum++)
            island.Blink[linknum] = csdata.islandlist[inum].cutline[linknum];
        if (island.Blinklen == 1)
            apl.peninsula.push_back(inum);
    }
}

int32_t BImap::DisconnectedPeninsula(int32_t anum, int32_t inum_S, int32_t inum_E,
                                     std::vector<int32_t> &BrokenBridges, std::vector<int32_t> &fpath) //, std::vector<int32_t> &BrokenIslands)
{
    BIarchipelago &apl = apllist[anum];
    std::vector<int> &peninsula = apl.peninsula;
    std::vector<BIisland> &islandlist = apl.islandlist;
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    BrokenBridges.clear();
    BrokenBridges.resize(apl.bridgelistlen, 1);
    // BrokenIslands.clear();
    // BrokenIslands.resize(apl.islandlistlen, 1);
    int32_t returndata = 0;
    int32_t peninsula_len = peninsula.size();
    for (int32_t num = 0; num < peninsula_len; num++)
    {
        int32_t island_temp = peninsula[num];
        int32_t UBB = islandlist[island_temp].Blink[0];
        int32_t UBB_count = 1;

        while (UBB_count == 1 && island_temp != inum_S && island_temp != inum_E)
        {
            BrokenBridges[UBB] = 0;
            // BrokenIslands[island_temp] = 0;
            // BrokenBridges.push_back(UBB);
            returndata++;
            if (island_temp == bridgelist[UBB].Ilink[0])
                island_temp = bridgelist[UBB].Ilink[1];
            else
                island_temp = bridgelist[UBB].Ilink[0];

            UBB_count = 0;
            int32_t Blinklen = islandlist[island_temp].Blinklen;
            for (int32_t i = 0; i < Blinklen; i++)
            {
                if (BrokenBridges[islandlist[island_temp].Blink[i]])
                {
                    UBB = islandlist[island_temp].Blink[i];
                    UBB_count++;
                }
            }
        }
    }

    fpath.clear();
    std::list<int32_t> fb_init;
    int32_t island_temp = inum_S;

    while (true)
    {
        int32_t link_bridge;
        int32_t link_bridgeold = -1;
        int32_t link_count = 0;

        int32_t Blinklen = islandlist[island_temp].Blinklen;
        for (int32_t i = 0; i < Blinklen; i++)
        {
            int32_t temp_Blink_i = islandlist[island_temp].Blink[i];
            if (BrokenBridges[temp_Blink_i] && temp_Blink_i != link_bridgeold)
            {
                link_bridge = temp_Blink_i;
                link_count++;
            }
        }

        if (link_count > 1)
            break;
        if (island_temp == bridgelist[link_bridge].Ilink[0])
            island_temp = bridgelist[link_bridge].Ilink[1];
        else
            island_temp = bridgelist[link_bridge].Ilink[0];
        // printf("init %d; ",island_temp);
        fb_init.push_back(link_bridge);
        if (island_temp == inum_E)
        {
            std::copy(fb_init.begin(), fb_init.end(), std::back_inserter(fpath));
            return apl.bridgelistlen - returndata;
        }
        link_bridgeold = link_bridge;
    }
    // printf("fb_init,%d\r\n", fb_init.size());
    int32_t finit_end = island_temp;
    std::list<int32_t> fb_goal;
    island_temp = inum_E;
    while (true)
    {
        int32_t link_bridge;
        int32_t link_bridgeold = -1;
        int32_t link_count = 0;

        int32_t Blinklen = islandlist[island_temp].Blinklen;
        for (int32_t i = 0; i < Blinklen; i++)
        {
            int32_t temp_Blink_i = islandlist[island_temp].Blink[i];
            if (BrokenBridges[temp_Blink_i] && temp_Blink_i != link_bridgeold)
            {
                link_bridge = temp_Blink_i;
                link_count++;
            }
        }

        if (link_count > 1)
            break;
        if (island_temp == bridgelist[link_bridge].Ilink[0])
            island_temp = bridgelist[link_bridge].Ilink[1];
        else
            island_temp = bridgelist[link_bridge].Ilink[0];
        fb_goal.push_back(link_bridge);
        if (island_temp == inum_S)
        {
            std::copy(fb_goal.rbegin(), fb_goal.rend(), std::back_inserter(fpath));
            return apl.bridgelistlen - returndata;
        }
        link_bridgeold = link_bridge;
    }
    if (island_temp == finit_end)
    {
        std::copy(fb_init.begin(), fb_init.end(), std::back_inserter(fpath));
        std::copy(fb_goal.rbegin(), fb_goal.rend(), std::back_inserter(fpath));
    }
    // printf("fb_goal,%d\r\n", fb_goal.size());
    return apl.bridgelistlen - returndata;
}
std::list<float> testCost;
std::list<int32_t> testNum;
std::list<long long> testTime;
long long testTime0;
float testMinCost = 0;
void BImap::CTRW(BIpoint &S, BIpoint &E, int32_t N, std::list<BIpoint> &Path)
{
    if (S.x >= shapeX || S.y >= shapeY)
    {
        printf("CT_RRTstar error: Init point is too large!!\r\n");
        return;
    }
    if (E.x >= shapeX || E.y >= shapeY)
    {
        printf("CT_RRTstar error: End point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(S.y, S.x);
    uint32_t anum_E = BIamap.at<uint16_t>(E.y, E.x);
    if (anum_S == 0xffff)
    {
        printf("CT_RRTstar error: robot can't be here!!\r\n");
        return;
    }
    if (anum_S != anum_E)
    {
        printf("CT_RRTstar error: The path cannot be connected!!\r\n");
        return;
    }
    int32_t inum_S = BIimap.at<uint16_t>(S.y, S.x);
    int32_t inum_E = BIimap.at<uint16_t>(E.y, E.x);
    Path.clear();
    if (inum_S == inum_E)
    {
        Path.push_front(E);
        Path.push_front(S);
        return;
    }
    BIarchipelago &apl = apllist[anum_S];
    std::vector<BIisland> &islandlist = apl.islandlist;
    std::vector<BIbridge> &bridgelist = apl.bridgelist;

    std::set<uint32_t> HomotopyEX; // 同伦类哈希值记录集合
    std::map<int32_t, int32_t> f_map, g_map;
    std::vector<int32_t> f_path, g_path;
    std::vector<int32_t> fb_path, gb_path;
    std::list<BIpoint> &minPath = Path;
    float PathMinCost = 3.3e38f;
    f_path.reserve(apl.islandlistlen);
    g_path.reserve(apl.islandlistlen);
    for (int32_t RWnum = 0; RWnum < N; RWnum++)
    {
        f_path.clear();
        g_path.clear();
        fb_path.clear();
        gb_path.clear();
        f_map[inum_S] = f_path.size();
        g_map[inum_E] = g_path.size();
        f_path.push_back(inum_S);
        g_path.push_back(inum_E);
        while (true)
        {
            int32_t f_Tnum = f_path.back();
            int32_t g_Tnum = g_path.back();
            BIisland &f_T = islandlist[f_Tnum];
            BIisland &g_T = islandlist[g_Tnum];
            std::vector<int32_t> fn_set, gn_set, fbn_set, gbn_set;
            for (size_t i = 0; i < f_T.Blinklen; i++)
            {
                BIbridge &b_temp = bridgelist[f_T.Blink[i]];
                if (f_map.count(b_temp.Ilink[0]) == 0) // b_temp.Ilink[0] != f_Tnum &&
                {
                    fn_set.push_back(b_temp.Ilink[0]);
                    fbn_set.push_back(f_T.Blink[i]);
                }
                else if (f_map.count(b_temp.Ilink[1]) == 0) // b_temp.Ilink[1] != f_Tnum &&
                {
                    fn_set.push_back(b_temp.Ilink[1]);
                    fbn_set.push_back(f_T.Blink[i]);
                }
            }
            for (size_t i = 0; i < g_T.Blinklen; i++)
            {
                BIbridge &b_temp = bridgelist[g_T.Blink[i]];
                if (g_map.count(b_temp.Ilink[0]) == 0) // b_temp.Ilink[0] != g_Tnum &&
                {
                    gn_set.push_back(b_temp.Ilink[0]);
                    gbn_set.push_back(f_T.Blink[i]);
                }
                else if (g_map.count(b_temp.Ilink[0]) == 0) // b_temp.Ilink[1] != g_Tnum &&
                {
                    gn_set.push_back(b_temp.Ilink[1]);
                    gbn_set.push_back(f_T.Blink[i]);
                }
            }
            if (fn_set.size() == 0 && gn_set.size() == 0)
            {
                f_path.clear();
                g_path.clear();
                fb_path.clear();
                gb_path.clear();
                f_map.clear();
                g_map.clear();
                f_map[inum_S] = f_path.size();
                g_map[inum_E] = g_path.size();
                f_path.push_back(inum_S);
                g_path.push_back(inum_E);
            }
            if (fn_set.size() != 0)
            {
                int32_t x_newnum = rand() % fn_set.size();
                int32_t x_new = fn_set[x_newnum];
                int32_t xb_new = fbn_set[x_newnum];
                f_map[x_new] = f_path.size();
                f_path.push_back(x_new);
                fb_path.push_back(xb_new);
                std::map<int32_t, int32_t>::iterator it = g_map.find(x_new);
                if (it != g_map.end())
                {
                    uint32_t gx_new = it->second;
                    while (gx_new)
                    {
                        gx_new--;
                        f_path.push_back(g_path[gx_new]);
                        fb_path.push_back(gb_path[gx_new]);
                    }
                    break;
                }
            }
            if (gn_set.size() != 0)
            {
                int32_t x_newnum = rand() % gn_set.size();
                int32_t x_new = gn_set[x_newnum];
                int32_t xb_new = gbn_set[x_newnum];
                g_map[x_new] = g_path.size();
                g_path.push_back(x_new);
                gb_path.push_back(xb_new);
                std::map<int32_t, int32_t>::iterator it = f_map.find(x_new);
                if (it != f_map.end())
                {
                    uint32_t fx_new = it->second;
                    uint32_t gx_new = g_map[x_new];
                    f_path.resize(fx_new + 1);
                    fb_path.resize(fx_new);
                    while (gx_new)
                    {
                        gx_new--;
                        f_path.push_back(g_path[gx_new]);
                        fb_path.push_back(gb_path[gx_new]);
                    }
                    break;
                }
            }
        }

        /******************************哈希值计算******************************/
        uint32_t hash_vel = 2166136261UL;
        int32_t x_temp = f_path.size();
        while (x_temp)
        {
            x_temp--;
            hash_vel ^= f_path[x_temp]; // Node_temp;//
            hash_vel *= 16777619UL;
            // printf("%3d,", f_path[x_temp]);
        }
        // printf("\r\n");
        if (!HomotopyEX.count(hash_vel))
        {
            static int i = 0;
            static float minPath_old = 3.2e38;
            HomotopyEX.insert(hash_vel);
            GetLeastHomotopyPath(apl, S, E, fb_path, minPath, PathMinCost);
            i++;
            if (PathMinCost < testCost.back())
            {
                testNum.push_back(i);
                testTime.push_back(utime_ns() - testTime0);
                testCost.push_back(PathMinCost);
                if (PathMinCost < testMinCost)
                    break;
            }
            // if (minPath_old > PathMinCost)
            // {
            //     minPath_old = PathMinCost;
            //     printf("%3d CTRW Cost: %f;\r\n", i++, PathMinCost);
            // }
        }
    }
}

void BImap::GetLeastHomotopyPath(BIarchipelago &apl, BIpoint &S, BIpoint &E, std::vector<int32_t> &fb_path, std::list<BIpoint> &Path, float &PathMinCost)
{
    std::vector<BIpoint> &pointmap = apl.pointmap;
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    Node PathTemp[fb_path.size() * 2 + 2];

    PathTemp[0].x = E.x;
    PathTemp[0].y = E.y;
    PathTemp[0].cost = 0.0f;

    float dx, dy;
    int32_t Node_Bnum, Path_Bnum = fb_path.size() - 1;
    Node_Bnum = fb_path[Path_Bnum];
    BIbridge &BFirst = bridgelist[Node_Bnum];
    PathTemp[1].x = pointmap[BFirst.pointS].x;
    PathTemp[1].y = pointmap[BFirst.pointS].y;
    dx = PathTemp[1].x - E.x;
    dy = PathTemp[1].y - E.y;
    PathTemp[1].cost = sqrtf(dx * dx + dy * dy);
    PathTemp[1].par = 0;
    PathTemp[1].bridge = Node_Bnum;
    PathTemp[2].x = pointmap[BFirst.pointE].x;
    PathTemp[2].y = pointmap[BFirst.pointE].y;
    dx = PathTemp[2].x - E.x;
    dy = PathTemp[2].y - E.y;
    PathTemp[2].cost = sqrtf(dx * dx + dy * dy);
    PathTemp[2].par = 0;
    PathTemp[2].bridge = Node_Bnum;
    Path_Bnum--;
    uint32_t PathTempLen = 2;
    uint8_t while2_key;
    while (Path_Bnum >= 0) // 从尾部回溯路�??
    {
        uint32_t PathNodeTemp;
        uint32_t PathNodeTempold;
        Node_Bnum = fb_path[Path_Bnum];
        BIbridge &Bpathtemp = bridgelist[Node_Bnum];
        Node &PathPointNowS = PathTemp[++PathTempLen];
        PathPointNowS.x = pointmap[Bpathtemp.pointS].x;
        PathPointNowS.y = pointmap[Bpathtemp.pointS].y;
        PathPointNowS.bridge = Node_Bnum;
        PathPointNowS.cost = 3.3e38f;
        PathNodeTemp = PathTempLen - 2; // 前分割线起点
        while2_key = 1;
        while (PathNodeTemp && while2_key)
        {
            PathNodeTempold = PathNodeTemp;
            PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线起点父节点
            uint32_t BDetectTemp = PathTempLen;
            uint32_t BDetectKey = (PathNodeTemp + 1) / 2;
            while ((BDetectTemp - 1) / 2 != BDetectKey) // 检测中间分割线相交性
            {
                BDetectTemp -= 2;
                BIbridge &BDetect = bridgelist[PathTemp[BDetectTemp].bridge];
                if (DetectIntersection(PathTemp[PathNodeTemp], PathPointNowS, pointmap[BDetect.pointS], pointmap[BDetect.pointE]))
                {
                    PathPointNowS.par = PathNodeTempold;
                    Node &PathPointNowPar = PathTemp[PathNodeTempold];
                    dx = PathPointNowPar.x - PathPointNowS.x;
                    dy = PathPointNowPar.y - PathPointNowS.y;
                    PathPointNowS.cost = PathPointNowPar.cost + sqrtf(dx * dx + dy * dy);
                    while2_key = 0;
                    break;
                }
            }
        }
        if (while2_key)
        {
            PathPointNowS.par = 0;
            Node &PathPointNowPar = PathTemp[0];
            dx = PathPointNowPar.x - PathPointNowS.x;
            dy = PathPointNowPar.y - PathPointNowS.y;
            PathPointNowS.cost = sqrtf(dx * dx + dy * dy);
        }
        else
        {
            while2_key = 1;
            PathNodeTemp = PathTempLen - 1; // 前分割线终点
            while (PathNodeTemp && while2_key)
            {
                PathNodeTempold = PathNodeTemp;
                PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线终点父节点
                uint32_t BDetectTemp = PathTempLen;
                uint32_t BDetectKey = (PathNodeTemp + 1) / 2;
                while ((BDetectTemp - 1) / 2 != BDetectKey) // 检测中间分割线相交性
                {
                    BDetectTemp -= 2;
                    BIbridge &BDetect = bridgelist[PathTemp[BDetectTemp].bridge];
                    if (DetectIntersection(PathTemp[PathNodeTemp], PathPointNowS, pointmap[BDetect.pointS], pointmap[BDetect.pointE]))
                    {
                        Node &PathPointNowPar = PathTemp[PathNodeTempold];
                        dx = PathPointNowPar.x - PathPointNowS.x;
                        dy = PathPointNowPar.y - PathPointNowS.y;
                        float costtemp = PathPointNowPar.cost + sqrtf(dx * dx + dy * dy);
                        if (costtemp < PathPointNowS.cost)
                        {
                            PathPointNowS.par = PathNodeTempold;
                            PathPointNowS.cost = costtemp;
                        }
                        while2_key = 0;
                        break;
                    }
                }
            }
            if (while2_key)
            {
                PathPointNowS.par = 0;
                Node &PathPointNowPar = PathTemp[0];
                dx = PathPointNowPar.x - PathPointNowS.x;
                dy = PathPointNowPar.y - PathPointNowS.y;
                PathPointNowS.cost = sqrtf(dx * dx + dy * dy);
            }
        }
        Node &PathPointNowE = PathTemp[++PathTempLen];
        PathPointNowE.x = pointmap[Bpathtemp.pointE].x;
        PathPointNowE.y = pointmap[Bpathtemp.pointE].y;
        PathPointNowE.cost = 3.3e38f;
        PathPointNowE.bridge = Node_Bnum;
        PathNodeTemp = PathTempLen - 3; // 前分割线起点
        while2_key = 1;
        while (PathNodeTemp && while2_key)
        {
            PathNodeTempold = PathNodeTemp;
            PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线起点父节点
            uint32_t BDetectTemp = PathTempLen;
            uint32_t BDetectKey = (PathNodeTemp + 1) / 2;
            while ((BDetectTemp - 1) / 2 != BDetectKey) // 检测中间分割线相交性
            {
                BDetectTemp -= 2;
                BIbridge &BDetect = bridgelist[PathTemp[BDetectTemp].bridge];
                if (DetectIntersection(PathTemp[PathNodeTemp], PathPointNowE, pointmap[BDetect.pointS], pointmap[BDetect.pointE]))
                {
                    PathPointNowE.par = PathNodeTempold;
                    Node &PathPointNowPar = PathTemp[PathNodeTempold];
                    dx = PathPointNowPar.x - PathPointNowE.x;
                    dy = PathPointNowPar.y - PathPointNowE.y;
                    PathPointNowE.cost = PathPointNowPar.cost + sqrtf(dx * dx + dy * dy);
                    while2_key = 0;
                    break;
                }
            }
        }
        if (while2_key)
        {
            PathPointNowE.par = 0;
            Node &PathPointNowPar = PathTemp[0];
            dx = PathPointNowPar.x - PathPointNowE.x;
            dy = PathPointNowPar.y - PathPointNowE.y;
            PathPointNowE.cost = sqrtf(dx * dx + dy * dy);
        }
        else
        {
            while2_key = 1;
            PathNodeTemp = PathTempLen - 2; // 前分割线终点
            while (PathNodeTemp && while2_key)
            {
                PathNodeTempold = PathNodeTemp;
                PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线终点父节点
                uint32_t BDetectTemp = PathTempLen;
                uint32_t BDetectKey = (PathNodeTemp + 1) / 2;
                while ((BDetectTemp - 1) / 2 != BDetectKey) // 检测中间分割线相交性
                {
                    BDetectTemp -= 2;
                    BIbridge &BDetect = bridgelist[PathTemp[BDetectTemp].bridge];
                    if (DetectIntersection(PathTemp[PathNodeTemp], PathPointNowE, pointmap[BDetect.pointS], pointmap[BDetect.pointE]))
                    {
                        Node &PathPointNowPar = PathTemp[PathNodeTempold];
                        dx = PathPointNowPar.x - PathPointNowE.x;
                        dy = PathPointNowPar.y - PathPointNowE.y;
                        float costtemp = PathPointNowPar.cost + sqrtf(dx * dx + dy * dy);
                        if (costtemp < PathPointNowE.cost)
                        {
                            PathPointNowE.par = PathNodeTempold;
                            PathPointNowE.cost = costtemp;
                        }
                        while2_key = 0;
                        break;
                    }
                }
            }
            if (while2_key)
            {
                PathPointNowE.par = 0;
                Node &PathPointNowPar = PathTemp[0];
                dx = PathPointNowPar.x - PathPointNowE.x;
                dy = PathPointNowPar.y - PathPointNowE.y;
                PathPointNowE.cost = sqrtf(dx * dx + dy * dy);
            }
        }

        Path_Bnum--;
    }
    while2_key = 1;
    Node &PathPointS = PathTemp[++PathTempLen];
    PathPointS.x = S.x;
    PathPointS.y = S.y;
    PathPointS.cost = 3.3e38f;
    int32_t PathNodeTempold;
    int32_t PathNodeTemp = PathTempLen - 2; // 前分割线起点
    while (PathNodeTemp && while2_key)
    {
        PathNodeTempold = PathNodeTemp;
        PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线起点父节点
        uint32_t BDetectTemp = PathTempLen;
        uint32_t BDetectKey = (PathNodeTemp + 1) / 2;
        while ((BDetectTemp - 1) / 2 != BDetectKey) // 检测中间分割线相交性
        {
            BDetectTemp -= 2;
            BIbridge &BDetect = bridgelist[PathTemp[BDetectTemp].bridge];
            if (DetectIntersection(PathTemp[PathNodeTemp], PathPointS, pointmap[BDetect.pointS], pointmap[BDetect.pointE]))
            {
                PathPointS.par = PathNodeTempold;
                Node &PathPointNowPar = PathTemp[PathNodeTempold];
                dx = PathPointNowPar.x - PathPointS.x;
                dy = PathPointNowPar.y - PathPointS.y;
                PathPointS.cost = PathPointNowPar.cost + sqrtf(dx * dx + dy * dy);
                while2_key = 0;
                break;
            }
        }
    }
    if (while2_key)
    {
        PathPointS.par = 0;
        Node &PathPointNowPar = PathTemp[0];
        dx = PathPointNowPar.x - PathPointS.x;
        dy = PathPointNowPar.y - PathPointS.y;
        PathPointS.cost = sqrtf(dx * dx + dy * dy);
    }
    else
    {
        while2_key = 1;
        PathNodeTemp = PathTempLen - 1; // 前分割线终点
        while (PathNodeTemp && while2_key)
        {
            PathNodeTempold = PathNodeTemp;
            PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线终点父节点
            uint32_t BDetectTemp = PathTempLen;
            uint32_t BDetectKey = (PathNodeTemp + 1) / 2;
            while ((BDetectTemp - 1) / 2 != BDetectKey) // 检测中间分割线相交性
            {
                BDetectTemp -= 2;
                BIbridge &BDetect = bridgelist[PathTemp[BDetectTemp].bridge];
                if (DetectIntersection(PathTemp[PathNodeTemp], PathPointS, pointmap[BDetect.pointS], pointmap[BDetect.pointE]))
                {
                    Node &PathPointNowPar = PathTemp[PathNodeTempold];
                    dx = PathPointNowPar.x - PathPointS.x;
                    dy = PathPointNowPar.y - PathPointS.y;
                    float costtemp = PathPointNowPar.cost + sqrtf(dx * dx + dy * dy);
                    if (costtemp < PathPointS.cost)
                    {
                        PathPointS.par = PathNodeTempold;
                        PathPointS.cost = costtemp;
                    }
                    while2_key = 0;
                    break;
                }
            }
        }
        if (while2_key)
        {
            PathPointS.par = 0;
            Node &PathPointNowPar = PathTemp[0];
            dx = PathPointNowPar.x - PathPointS.x;
            dy = PathPointNowPar.y - PathPointS.y;
            PathPointS.cost = sqrtf(dx * dx + dy * dy);
        }
    }
    if (PathPointS.cost < PathMinCost)
    {
        Path.clear();
        PathNodeTemp = PathTempLen;
        // PathMinLen = 1;
        Path.emplace_back(BIpoint{PathTemp[PathNodeTemp].x, PathTemp[PathNodeTemp].y});
        while (PathNodeTemp)
        {
            PathNodeTemp = PathTemp[PathNodeTemp].par; // 向前索引分割线终点父节点
            Path.emplace_back(BIpoint{PathTemp[PathNodeTemp].x, PathTemp[PathNodeTemp].y});
        }
        PathMinCost = PathPointS.cost;
    }
}

void BImap::CT_RRTstar(BIpoint &S, BIpoint &E, float weaken, int32_t N, std::list<BIpoint> &Path)
{
    if (S.x >= shapeX || S.y >= shapeY)
    {
        printf("CT_RRTstar error: Init point is too large!!\r\n");
        return;
    }
    if (E.x >= shapeX || E.y >= shapeY)
    {
        printf("CT_RRTstar error: End point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(S.y, S.x);
    uint32_t anum_E = BIamap.at<uint16_t>(E.y, E.x);
    if (anum_S == 0xffff)
    {
        printf("CT_RRTstar error: robot can't be here!!\r\n");
        return;
    }
    if (anum_S != anum_E)
    {
        printf("CT_RRTstar error: The path cannot be connected!!\r\n");
        return;
    }
    int32_t inum_S = BIimap.at<uint16_t>(S.y, S.x);
    int32_t inum_E = BIimap.at<uint16_t>(E.y, E.x);
    Path.clear();
    if (inum_S == inum_E)
    {
        Path.push_front(E);
        Path.push_front(S);
        return;
    }

    // cv::Point Scenter(S.x, S.y);
    // cv::Point Ccenter(E.x, E.y);
    // cv::Mat copyMat;
    // cv::resize(BIdmap,copyMat,cv::Size(0, 0),0.5,0.5);
    // cv::circle(copyMat, Scenter*0.5, 10, cv::Scalar(0, 0, 255), -1);
    // cv::circle(copyMat, Ccenter*0.5, 10, cv::Scalar(0, 255, 0), -1);
    // char buff[512];
    // sprintf(buff,"/home/tzyh/WorkSpace/BIRRT/data/vpng/V_%d.png",0);
    // cv::imwrite(buff, copyMat);

    BIarchipelago &apl = apllist[anum_S];
    std::vector<BIpoint> &pointmap = apl.pointmap;
    std::vector<BIisland> &islandlist = apl.islandlist;
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    std::vector<Node> &CTonly_BNodes = apl.CTonly_BNodes;
    std::vector<int32_t> &CTonly_BNlink = apl.CTonly_BNlink;

    std::vector<int32_t> fb_path;
    fb_path.reserve(apl.bridgelistlen);
    fb_path.clear();
    std::list<BIpoint> &PathMin = Path;
    float PathMinCost = 3.3e38f;
    std::vector<int32_t> BrokenBridges;
    int32_t UBnum = DisconnectedPeninsula(anum_S, inum_S, inum_E, BrokenBridges, fb_path);
    if (fb_path.size() != 0)
    {
        // printf("GetLeastHomotopyPath\r\n");
        GetLeastHomotopyPath(apl, S, E, fb_path, PathMin, PathMinCost);
        testNum.push_back(0);
        testTime.push_back(utime_ns() - testTime0);
        testCost.push_back(PathMinCost);
        return;
    }
    // printf("apl.bridgelistlen %d, UB %d\r\n",apl.bridgelistlen,UBnum);
    // int32_t UBnum = apl.bridgelistlen;          //无减枝对比实验
    // BrokenBridges.clear();                      //无减枝对比实验
    // BrokenBridges.resize(apl.bridgelistlen, 1); //无减枝对比实验

    int8_t NActive[apl.CTonly_BNodes.size()]; // 已激活节点
    int8_t BActive[apl.bridgelistlen];        // 已激活分割线
    memset(BActive, 0, sizeof(int8_t) * apl.bridgelistlen);
    memset(NActive, 0, sizeof(int8_t) * apl.CTonly_BNodes.size());
    for (size_t i = 0; i < islandlist[inum_S].Blinklen; i++)
    {
        BActive[islandlist[inum_S].Blink[i]] = 1;
    }
    int8_t UBBUnFirst[UBnum]; // 有效分割线序号首次采样标志
    memset(UBBUnFirst, 1, sizeof(int8_t) * UBnum);
    int32_t Expansion[UBnum];
    memset(Expansion, 0, sizeof(int32_t) * UBnum);
    int32_t HNcount[apl.bridgelistlen]; // HN计数
    // memset(HNcount, 0, sizeof(int32_t) * apl.bridgelistlen);
    int32_t UBBridges[UBnum]; // 有效分割线序列
    int32_t count_temp = 0;
    for (size_t i = 0; count_temp < UBnum; i++)
    {
        if (BrokenBridges[i])
        {
            HNcount[i] = 0;
            UBBridges[count_temp++] = i;
        }
    }

    int32_t BValid[UBnum]; // 已可采样分割线
    int32_t Nmax = 0;
    for (int32_t i = 0; i < UBnum; i++)
    {
        BValid[i] = CTonly_BNlink[UBBridges[i] + 1] - CTonly_BNlink[UBBridges[i]];
        Nmax += BValid[i];
    }
    Nmax = Nmax < N ? Nmax : N;
    std::list<int32_t> UBLink[apl.bridgelistlen]; // 分割线挂载链表
    float VirtualLength[UBnum + 1];               // 轮盘赌长度
    VirtualLength[0] = 0;
    Node RRTNodelist[N + 1];
    RRTNodelist[0].cost = 0;
    RRTNodelist[0].x = S.x;
    RRTNodelist[0].y = S.y;
    std::set<uint32_t> HomotopyEX; // 同伦类哈希值记录集
    // Node PathMin[UBnum + 2];
    // uint32_t PathMinLen = 0;
    Node PathTemp[UBnum * 2 + 2];
    int8_t UpdataVirtualLengthKey = 1;
    int32_t node_num;
    std::list<int32_t> Qr;
    for (node_num = 1; node_num < Nmax; node_num++)
    {
        /******************************更新轮盘赌长度******************************/
        if (UpdataVirtualLengthKey)
        {
            for (int32_t temp = 0; temp < UBnum; temp++)
            {
                if (BActive[UBBridges[temp]] && BValid[temp])
                    VirtualLength[temp + 1] = VirtualLength[temp] + powf(weaken, HNcount[UBBridges[temp]]) + 1000.0f * powf(0, Expansion[temp]);
                //   bridgelist[UBBridges[temp]].length * powf(weaken, HNcount[temp]);
                else
                    VirtualLength[temp + 1] = VirtualLength[temp];
            }
            UpdataVirtualLengthKey = 0;
        }
        /******************************随机采样******************************/
        float Prandom1 = rand() / (float(RAND_MAX));
        float Prandom2 = rand() / (float(RAND_MAX)) * VirtualLength[UBnum];
        int32_t SampleLine;
        for (SampleLine = UBnum - 1; SampleLine != 0; SampleLine--)
        {
            if (Prandom2 >= VirtualLength[SampleLine])
                break;
        }
        Expansion[SampleLine]++;
        // SampleLine--;
        int32_t OSampleLine = UBBridges[SampleLine];
        UBLink[OSampleLine].push_back(node_num);
        // BIpoint &SampleLine_S = pointmap[bridgelist[OSampleLine].pointS];
        // BIpoint &SampleLine_E = pointmap[bridgelist[OSampleLine].pointE];
        RRTNodelist[node_num].bridge = OSampleLine;
        if (UBBUnFirst[SampleLine])
        {
            UBBUnFirst[SampleLine] = 0;
            for (size_t i = 0; i < bridgelist[OSampleLine].Blinklen; i++)
            {
                BActive[bridgelist[OSampleLine].Blink[i]] = 1;
            }
            UpdataVirtualLengthKey = 1;
        }
        // RRTNodelist[node_num].x = (1.0f - Prandom1) * SampleLine_S.x + Prandom1 * SampleLine_E.x;
        // RRTNodelist[node_num].y = (1.0f - Prandom1) * SampleLine_S.y + Prandom1 * SampleLine_E.y;
        uint32_t nodepointk = BValid[SampleLine] * Prandom1;
        for (int32_t i = CTonly_BNlink[OSampleLine]; i < CTonly_BNlink[OSampleLine + 1]; i++)
        {
            if (!NActive[i])
            {
                if (!nodepointk)
                {
                    NActive[i] = 1;
                    RRTNodelist[node_num].x = CTonly_BNodes[i].x;
                    RRTNodelist[node_num].y = CTonly_BNodes[i].y;
                    break;
                }
                nodepointk--;
            }
        }
        if ((--BValid[SampleLine]) == 0)
        {
            UpdataVirtualLengthKey = 1;
        }

        /******************************查找紧邻节点******************************/
        if (inum_S == bridgelist[OSampleLine].Ilink[0] || inum_S == bridgelist[OSampleLine].Ilink[1])
        {
            float dx = S.x - RRTNodelist[node_num].x;
            float dy = S.y - RRTNodelist[node_num].y;
            RRTNodelist[node_num].cost = sqrtf(dx * dx + dy * dy);
            RRTNodelist[node_num].par = 0;
        }
        else
        {
            int32_t minNode = -1;
            float minCost = 3.3e38f;
            for (size_t temp = 0; temp < bridgelist[OSampleLine].Blinklen; temp++)
            {
                int32_t Blink_temp = bridgelist[OSampleLine].Blink[temp];
                for (std::list<int32_t>::iterator it = UBLink[Blink_temp].begin(); it != UBLink[Blink_temp].end(); ++it)
                {
                    Node &Nnode = RRTNodelist[*it];
                    float dx = Nnode.x - RRTNodelist[node_num].x;
                    float dy = Nnode.y - RRTNodelist[node_num].y;
                    float Ncost = sqrtf(dx * dx + dy * dy) + Nnode.cost;
                    if (Ncost < minCost)
                    {
                        minCost = Ncost;
                        minNode = *it;
                    }
                }
            }
            RRTNodelist[node_num].cost = minCost;
            RRTNodelist[node_num].par = minNode;
        }
        Qr.push_back(node_num);
        if (node_num % 1 == 0)
        {
            /******************************重连接******************************/
            while (Qr.size())
            {
                int32_t xr = Qr.front();
                Qr.pop_front();
                int32_t B_xr = RRTNodelist[xr].bridge;
                for (size_t temp = 0; temp < bridgelist[B_xr].Blinklen; temp++)
                {
                    int32_t Blink_temp = bridgelist[B_xr].Blink[temp];
                    for (std::list<int32_t>::iterator it = UBLink[Blink_temp].begin(); it != UBLink[Blink_temp].end(); ++it)
                    {
                        Node &Nnode = RRTNodelist[*it];
                        float dx = Nnode.x - RRTNodelist[xr].x;
                        float dy = Nnode.y - RRTNodelist[xr].y;
                        float Ncost = sqrtf(dx * dx + dy * dy) + RRTNodelist[xr].cost;
                        if (Ncost < Nnode.cost)
                        {
                            Nnode.cost = Ncost;
                            Nnode.par = xr;
                            Qr.push_back(*it);
                        }
                    }
                }
            }
            /******************************终点同伦类检测******************************/
            for (size_t temp = 0; temp < islandlist[inum_E].Blinklen; temp++)
            {
                int32_t Blink_temp = islandlist[inum_E].Blink[temp];
                for (std::list<int32_t>::iterator it = UBLink[Blink_temp].begin(); it != UBLink[Blink_temp].end(); ++it)
                {
                    Node &Nnode = RRTNodelist[*it];
                    Node &Nnode_par = RRTNodelist[Nnode.par];
                    if (bridgelist[Nnode_par.bridge].Ilink[0] != inum_E && bridgelist[Nnode_par.bridge].Ilink[1] != inum_E)
                    {
                        /******************************哈希值计算******************************/
                        uint32_t hash_vel = 2166136261UL;
                        int32_t Node_temp = *it;
                        fb_path.resize(0);
                        while (Node_temp)
                        {
                            HNcount[RRTNodelist[Node_temp].bridge]++;
                            fb_path.push_back(RRTNodelist[Node_temp].bridge);
                            hash_vel ^= RRTNodelist[Node_temp].bridge; // Node_temp;//
                            hash_vel *= 16777619UL;
                            Node_temp = RRTNodelist[Node_temp].par;
                        }
                        if (!HomotopyEX.count(hash_vel))
                        {
                            HomotopyEX.insert(hash_vel);
                            GetLeastHomotopyPath(apl, E, S, fb_path, PathMin, PathMinCost);
                        }
                    }
                }
            }
            if (PathMinCost < testCost.back())
            {
                testNum.push_back(node_num);
                testTime.push_back(utime_ns() - testTime0);
                testCost.push_back(PathMinCost);
            }
            // cv::Point p0, p1;
            // cv::Mat tempMat = copyMat.clone();
            // for (int32_t i = 1; i <= node_num; i++)
            // {
            //     p0 = {(int32_t)(RRTNodelist[i].x*0.5), (int32_t)(RRTNodelist[i].y*0.5)};
            //     p1 = {(int32_t)(RRTNodelist[RRTNodelist[i].par].x*0.5), (int32_t)(RRTNodelist[RRTNodelist[i].par].y*0.5)};
            //     cv::line(tempMat, p0, p1, cv::Scalar(255, 0, 0), 2);
            // }
            // p1 = {(int32_t)(PathMin.front().x*0.5), (int32_t)(PathMin.front().y*0.5)};
            // for (list<BIpoint>::iterator it = PathMin.begin(); it != PathMin.end(); ++it)
            // {
            //     p0 = p1;
            //     p1 = {(int32_t)(it->x*0.5), (int32_t)(it->y*0.5)};
            //     cv::line(tempMat, p0, p1, cv::Scalar(0, 0, 255), 3);
            // }
            // sprintf(buff,"/home/tzyh/WorkSpace/BIRRT/data/vpng/V_%d.png",node_num);
            // cv::imwrite(buff, tempMat);

            if (PathMinCost < testMinCost)
                return;
        }
    }
    // for (uint32_t i = 0; i < PathMinLen; i++)
    // {
    //     Node &Pn = PathMin[i];
    //     Path.emplace_back(BIpoint{Pn.x, Pn.y});
    // }

    // cv::Point p0, p1;
    // for (int32_t i = 1; i < node_num; i++)
    // {
    //     p0 = {(int32_t)(RRTNodelist[i].x), (int32_t)(RRTNodelist[i].y)};
    //     p1 = {(int32_t)(RRTNodelist[RRTNodelist[i].par].x), (int32_t)(RRTNodelist[RRTNodelist[i].par].y)};
    //     cv::line(showmat, p0, p1, cv::Scalar(255, 0, 0), 1);
    // }
    // for (int32_t i = 0; i < apl.bridgelist.size(); i++)
    // {
    //     BIbridge &brg = apl.bridgelist[i];
    //     p0 = {(int32_t)(apl.pointmap[brg.pointS].x/2), (int32_t)(apl.pointmap[brg.pointS].y/2)};
    //     p1 = {(int32_t)(apl.pointmap[brg.pointE].x/2), (int32_t)(apl.pointmap[brg.pointE].y/2)};
    //     if (BActive[i])
    //         cv::line(copyMat, p0, p1, cv::Scalar(255, 255, 255), 3);
    //     else
    //         cv::line(copyMat, p0, p1, cv::Scalar(0, 0, 255), 3);
    // }
    // sprintf(buff,"/home/tzyh/WorkSpace/BIRRT/data/vpng/V_%d.png",999);
    // cv::imwrite(buff, copyMat);
    // printf("%d  ::%s\r\n",UBnum,buff);
    // cout << "PathMinLen:" << PathMinLen << endl;
    // cout << "Cost:" << PathMinCost << endl;
}
bool BImap::DetectIntersection(Node &S0, Node &E0, BIpoint &S1, BIpoint &E1)
{
    float v1_x, v1_y, v2_x, v2_y, v3_x, v3_y, r1, r2;

    v1_x = S1.x - S0.x;
    v1_y = S1.y - S0.y;
    v2_x = E0.x - S0.x;
    v2_y = E0.y - S0.y;
    v3_x = E1.x - S0.x;
    v3_y = E1.y - S0.y;
    r1 = (v1_x * v2_y - v1_y * v2_x) * (v3_x * v2_y - v3_y * v2_x);

    v1_x = S0.x - S1.x;
    v1_y = S0.y - S1.y;
    v2_x = E1.x - S1.x;
    v2_y = E1.y - S1.y;
    v3_x = E0.x - S1.x;
    v3_y = E0.y - S1.y;
    r2 = (v1_x * v2_y - v1_y * v2_x) * (v3_x * v2_y - v3_y * v2_x);
    if (r1 <= 0 && r2 <= 0)
        return 0;
    else
        return 1;
}

void BImap::CT_onlyBegin(float Resolution)
{
    for (uint32_t i = 0; i < apllist.size(); i++)
    {
        BIarchipelago &apl = apllist[i];
        std::vector<BIpoint> &pointmap = apl.pointmap;
        apl.CTonly_BNlink.resize(apl.bridgelistlen + 1);
        apl.CTonly_Pinit = {-1.0f, -1.0f};
        uint32_t BNodesNUM = 1;
        for (uint32_t j = 0; j < apl.bridgelistlen; j++)
        {
            BIbridge &bridge = apl.bridgelist[j];
            // float dx = pointmap[bridge.pointS].x - pointmap[bridge.pointE].x;
            // float dy = pointmap[bridge.pointS].y - pointmap[bridge.pointE].y;
            apl.CTonly_BNlink[j] = BNodesNUM;
            BNodesNUM += bridge.length / Resolution + 2;
        }
        apl.CTonly_BNlink.back() = BNodesNUM;
        apl.CTonly_BNodes.resize(BNodesNUM);
        for (uint32_t j = 0; j < apl.bridgelistlen; j++)
        {
            BIbridge &bridge = apl.bridgelist[j];
            int32_t BNpoint = apl.CTonly_BNlink[j];
            int32_t dnum = apl.CTonly_BNlink[j + 1] - BNpoint - 1;
            for (int32_t k = 0; k <= dnum; k++)
            {
                float t = (float)k / dnum;
                float t_1 = 1.0f - t;
                apl.CTonly_BNodes[BNpoint + k].bridge = j;
                apl.CTonly_BNodes[BNpoint + k].x = pointmap[bridge.pointS].x * t_1 + pointmap[bridge.pointE].x * t;
                apl.CTonly_BNodes[BNpoint + k].y = pointmap[bridge.pointS].y * t_1 + pointmap[bridge.pointE].y * t;
            }
        }
    }
}
int BImap::CT_onlySetInit(BIpoint &S)
{
    // int32_t fig_count = 0;
    // uint32_t change_num = 0;
    if (S.x >= shapeX || S.y >= shapeY)
    {
        printf("CT_only error: Init point is too large!!\r\n");
        return -1;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(S.y, S.x);
    if (anum_S == 0xffff)
    {
        printf("CT_only error: robot can't be here!!\r\n");
        return -2;
    }
    int32_t inum_S = BIimap.at<uint16_t>(S.y, S.x);
    BIarchipelago &apl = apllist[anum_S];
    std::vector<BIisland> &islandlist = apl.islandlist;
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    if (fabs(S.x - apl.CTonly_Pinit.x) < 0.1f && fabs(S.y - apl.CTonly_Pinit.y) < 0.1f)
        return 0;
    apl.CTonly_Pinit.x = S.x;
    apl.CTonly_Pinit.y = S.y;
    std::vector<Node> &CTonly_BNodes = apl.CTonly_BNodes;
    std::vector<int32_t> &CTonly_BNlink = apl.CTonly_BNlink;
    CTonly_BNodes[0].cost = 0;
    CTonly_BNodes[0].x = S.x;
    CTonly_BNodes[0].y = S.y;

    int32_t Bexs[apl.bridgelistlen]; // 已存在分割线
    memset(Bexs, 0, sizeof(int32_t) * apl.bridgelistlen);
    for (size_t i = 0; i < islandlist[inum_S].Blinklen; i++)
    {
        float dx, dy;
        int32_t Bnum_temp = islandlist[inum_S].Blink[i];
        Bexs[Bnum_temp] = 1;
        for (int32_t k = CTonly_BNlink[Bnum_temp]; k < CTonly_BNlink[Bnum_temp + 1]; k++)
        {
            CTonly_BNodes[k].par = 0;
            dx = CTonly_BNodes[k].x - S.x;
            dy = CTonly_BNodes[k].y - S.y;
            CTonly_BNodes[k].cost = sqrtf(dx * dx + dy * dy);
        }
    }
    std::list<int32_t> Qadd;
    for (size_t i = 0; i < islandlist[inum_S].Blinklen; i++) // 创建Qadd
    {
        BIbridge &Btemp = bridgelist[islandlist[inum_S].Blink[i]];
        for (size_t j = 0; j < Btemp.Blinklen; j++)
        {
            if (!Bexs[Btemp.Blink[j]])
                Qadd.push_back(Btemp.Blink[j]);
        }
    }
    while (Qadd.size())
    {
        std::list<int32_t> Qr;
        int32_t Ck = Qadd.front();
        Qadd.pop_front();
        BIbridge &Bck = bridgelist[Ck];
        int32_t BCkNear[Bck.Blinklen]; // 查找有效的近邻分割线
        int32_t BCkNearLen = 0;
        for (uint32_t i = 0; i < Bck.Blinklen; i++)
        {
            if (Bexs[Bck.Blink[i]])
                BCkNear[BCkNearLen++] = Bck.Blink[i];
            else
                Qadd.push_back(Bck.Blink[i]);
        }
        for (int32_t i = CTonly_BNlink[Ck]; i < CTonly_BNlink[Ck + 1]; i++) // 对Ck所有点计算
        {
            float dx, dy, dcost;
            int32_t minNode = -1;
            float minCost = 3.3e38f;
            Node &NodeCk_temp = CTonly_BNodes[i];
            for (int32_t j = 0; j < BCkNearLen; j++) // 遍历所有近邻线
            {
                int32_t BCkNear_j_1 = CTonly_BNlink[BCkNear[j] + 1];
                for (int32_t k = CTonly_BNlink[BCkNear[j]]; k < BCkNear_j_1; k++) // 遍历近邻线所有点
                {
                    Node &BCkNear_k = CTonly_BNodes[k];
                    dx = NodeCk_temp.x - BCkNear_k.x;
                    dy = NodeCk_temp.y - BCkNear_k.y;
                    dcost = sqrtf(dx * dx + dy * dy) + BCkNear_k.cost;
                    if (dcost < minCost && BCkNear_k.par != i)
                    {
                        minCost = dcost;
                        minNode = k;
                    }
                }
            }

            NodeCk_temp.par = minNode;
            NodeCk_temp.cost = minCost;
            Qr.push_back(i);
        }
        Bexs[Ck] = 1;
        while (Qr.size()) // 重连接优
        {
            int32_t Nr_temp = Qr.front();
            int32_t Cr = CTonly_BNodes[Nr_temp].bridge;
            int32_t NrsLen = 0;
            int32_t NrsLenMax = CTonly_BNlink[Cr + 1] - CTonly_BNlink[Cr];
            int32_t Nrs[NrsLenMax];
            while (CTonly_BNodes[Nr_temp].bridge == Cr && Qr.size()) // && NrsLen < NrsLenMax)
            {
                Nrs[NrsLen++] = Nr_temp;
                Qr.pop_front();
                Nr_temp = Qr.front();
            }
            BIbridge &Bcr = bridgelist[Cr];
            int32_t BCrNear[Bcr.Blinklen]; // 查找Cr有效的近邻分割线
            int32_t BCrNearLen = 0;
            for (uint32_t i = 0; i < Bcr.Blinklen; i++)
            {
                if (Bexs[Bcr.Blink[i]])
                    BCrNear[BCrNearLen++] = Bcr.Blink[i];
            }

            for (int32_t i = 0; i < BCrNearLen; i++)
            {
                int32_t BCrNear_i_1 = CTonly_BNlink[BCrNear[i] + 1];
                for (int32_t j = CTonly_BNlink[BCrNear[i]]; j < BCrNear_i_1; j++) // 遍历近邻线所有点
                {
                    float dx, dy, dcost;
                    Node &BCrNear_ij = CTonly_BNodes[j];
                    uint8_t ChangeKey = 0;
                    for (int32_t *k = Nrs; k < Nrs + NrsLen; k++)
                    {
                        Node &BCr_k = CTonly_BNodes[*k];
                        dx = BCrNear_ij.x - BCr_k.x;
                        dy = BCrNear_ij.y - BCr_k.y;
                        dcost = sqrtf(dx * dx + dy * dy) + BCr_k.cost;
                        if (dcost < BCrNear_ij.cost && BCr_k.par != j)
                        {
                            BCrNear_ij.par = *k;
                            BCrNear_ij.cost = dcost;
                            ChangeKey = 1;
                        }
                    }
                    if (ChangeKey)
                    {
                        Qr.push_back(j);
                        // change_num++;
                    }
                }
            }
        }
        // {
        //     char namebuf[100];
        //     cv::Point p0, p1;
        //     showmat = BIdmap.clone();
        //     for (uint32_t i = 1; i < CTonly_BNodes.size(); i++)
        //     {
        //         if (Bexs[CTonly_BNodes[i].bridge])
        //         {
        //             p0 = {(int32_t)(CTonly_BNodes[i].x), (int32_t)(CTonly_BNodes[i].y)};
        //             p1 = {(int32_t)(CTonly_BNodes[CTonly_BNodes[i].par].x), (int32_t)(CTonly_BNodes[CTonly_BNodes[i].par].y)};
        //             cv::line(showmat, p0, p1, cv::Scalar(255, 0, 0), 1);
        //         }
        //     }
        //     sprintf(namebuf, "/home/tzy_h/WorkSpace/BIRRT/IROS2023/CTonlyimg/debungimg%d_b.png", fig_count);
        //     cv::imwrite(namebuf, showmat);
        // }
        // fig_count++;
    }
    return 1;
    // printf("change:%d\r\n", change_num);
    // cv::Point p0, p1;
    // for (uint32_t i = 1; i < CTonly_BNodes.size(); i++)
    // {
    //     p0 = {(int32_t)(CTonly_BNodes[i].x), (int32_t)(CTonly_BNodes[i].y)};
    //     p1 = {(int32_t)(CTonly_BNodes[CTonly_BNodes[i].par].x), (int32_t)(CTonly_BNodes[CTonly_BNodes[i].par].y)};
    //     cv::line(showmat, p0, p1, cv::Scalar(255, 0, 0), 1);
    // }
}
float BImap::CT_onlyGetPath(BIpoint &E, std::list<BIpoint> &Path)
{
    if (E.x >= shapeX || E.y >= shapeY)
    {
        printf("CT_only error: End point is too large!!\r\n");
        return -1;
    }
    uint32_t anum_E = BIamap.at<uint16_t>(E.y, E.x);
    if (anum_E == 0xffff)
    {
        printf("CT_only error: robot can't be here!!\r\n");
        return -1;
    }
    BIarchipelago &apl = apllist[anum_E];
    std::vector<BIisland> &islandlist = apl.islandlist;
    std::vector<Node> &CTonly_BNodes = apl.CTonly_BNodes;
    int32_t inum_S = BIimap.at<uint16_t>(apl.CTonly_Pinit.y, apl.CTonly_Pinit.x);
    int32_t inum_E = BIimap.at<uint16_t>(E.y, E.x);
    Path.clear();
    Path.push_front(E);
    if (inum_E == inum_S)
    {
        float dx, dy;
        Path.push_front(apl.CTonly_Pinit);
        dx = E.x - apl.CTonly_Pinit.x;
        dy = E.y - apl.CTonly_Pinit.y;
        return sqrtf(dx * dx + dy * dy);
    }
    else
    {
        int32_t Pnode = -1;
        float dx, dy, dcost;
        float mincost = 3.3e38f;
        BIisland &islandE = islandlist[inum_E];
        for (uint32_t i = 0; i < islandE.Blinklen; i++)
        {
            int32_t BNi_End = apl.CTonly_BNlink[islandE.Blink[i] + 1];
            for (int32_t j = apl.CTonly_BNlink[islandE.Blink[i]]; j < BNi_End; j++)
            {
                Node &BN_ij = CTonly_BNodes[j];
                dx = E.x - BN_ij.x;
                dy = E.y - BN_ij.y;
                dcost = BN_ij.cost + sqrtf(dx * dx + dy * dy);
                if (dcost < mincost)
                {
                    Pnode = j;
                    mincost = dcost;
                }
            }
        }
        int debug_count = 0;
        while (Pnode && ++debug_count < 200)
        {
            Node &Pn = CTonly_BNodes[Pnode];
            Path.emplace_front(BIpoint{Pn.x, Pn.y});
            Pnode = Pn.par;
        }
        Path.push_front(apl.CTonly_Pinit);
        return mincost;
    }
}
void BImap::CT_onlyGetPath(BIpoint &S, BIpoint &E, std::list<BIpoint> &Path)
{
    if (S.x >= shapeX || S.y >= shapeY)
    {
        printf("CT_RRTstar error: Init point is too large!!\r\n");
        return;
    }
    if (E.x >= shapeX || E.y >= shapeY)
    {
        printf("CT_RRTstar error: End point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(S.y, S.x);
    uint32_t anum_E = BIamap.at<uint16_t>(E.y, E.x);
    if (anum_S == 0xffff)
    {
        printf("CT_RRTstar error: robot can't be here!!\r\n");
        return;
    }
    if (anum_S != anum_E)
    {
        printf("CT_RRTstar error: The path cannot be connected!!\r\n");
        return;
    }
    CT_onlySetInit(S);
    CT_onlyGetPath(E, Path);
}

void BImap::JsontoBImap(char *name, bool debugmap)
{
    json BIdata;
    ifstream BIfjson(name);
    BIfjson >> BIdata;
    BIfjson.close();

    shapeX = BIdata["shapeX"];
    shapeY = BIdata["shapeY"];
    mapratio = BIdata["mapratio"];
    robotsize = BIdata["robotsize"];
    apllist.resize(BIdata["apllist"].size());
    for (uint32_t i = 0; i < apllist.size(); i++)
    {
        BIarchipelago &apl = apllist[i];
        json &apljson = BIdata["apllist"][i];
        apl.aplnum = apljson["aplnum"];

        json &PMjson = apljson["pointmap"];
        apl.pointmaplen = PMjson.size();
        apl.pointmap.resize(apl.pointmaplen); // = (BIpoint *)malloc(sizeof(BIpoint) * apl.pointmaplen);
        for (uint32_t j = 0; j < apl.pointmaplen; j++)
            apl.pointmap[j] = {PMjson[j][0], PMjson[j][1]};

        json &ILjson = apljson["islandlist"];
        apl.islandlistlen = apljson["islandlist"].size();
        apl.islandlist.resize(apl.islandlistlen); // = (BIisland *)malloc(sizeof(BIisland) * apl.islandlistlen);
        for (uint32_t j = 0; j < apl.islandlistlen; j++)
        {
            BIisland &island_ij = apl.islandlist[j];
            json &ILjson_j = ILjson[j];
            island_ij.pointslen = ILjson_j["points"].size();
            island_ij.points.resize(island_ij.pointslen); // = (int32_t *)malloc(sizeof(int32_t) * island_ij.pointslen);
            for (uint32_t k = 0; k < island_ij.pointslen; k++)
                island_ij.points[k] = ILjson_j["points"][k];
            island_ij.Blinklen = ILjson_j["Blink"].size();
            island_ij.Blink.resize(island_ij.Blinklen); // = (int32_t *)malloc(sizeof(int32_t) * island_ij.Blinklen);
            for (uint32_t k = 0; k < island_ij.Blinklen; k++)
                island_ij.Blink[k] = ILjson_j["Blink"][k];
        }

        json &BLjson = apljson["bridgelist"];
        apl.bridgelistlen = apljson["bridgelist"].size();
        apl.bridgelist.resize(apl.bridgelistlen); // = (BIbridge *)malloc(sizeof(BIbridge) * apl.bridgelistlen);
        for (uint32_t j = 0; j < apl.bridgelistlen; j++)
        {
            BIbridge &bridge_ij = apl.bridgelist[j];
            json &BLjson_j = BLjson[j];
            bridge_ij.pointS = BLjson_j["pointS"];
            bridge_ij.pointE = BLjson_j["pointE"];
            bridge_ij.length = BLjson_j["length"];
            bridge_ij.core = {BLjson_j["core"][0], BLjson_j["core"][1]};

            bridge_ij.Ilinklen = BLjson_j["Ilink"].size();
            bridge_ij.Ilink.resize(bridge_ij.Ilinklen); // = (int32_t *)malloc(sizeof(int32_t) * bridge_ij.Ilinklen);
            for (uint32_t k = 0; k < bridge_ij.Ilinklen; k++)
                bridge_ij.Ilink[k] = BLjson_j["Ilink"][k];

            bridge_ij.Blinklen = BLjson_j["Blink"].size();
            bridge_ij.Blink.resize(bridge_ij.Blinklen); // = (int32_t *)malloc(sizeof(int32_t) * bridge_ij.Blinklen);
            for (uint32_t k = 0; k < bridge_ij.Blinklen; k++)
                bridge_ij.Blink[k] = BLjson_j["Blink"][k];
        }

        apl.peninsula.resize(apljson["peninsula"].size());
        for (uint32_t i = 0; i < apl.peninsula.size(); i++)
            apl.peninsula[i] = apljson["peninsula"][i];
    }
    DrawBImap(debugmap);
}
void BImap::BImaptoJson(char *name)
{
    json BIdata;
    BIdata["shapeX"] = shapeX;
    BIdata["shapeY"] = shapeY;
    BIdata["mapratio"] = mapratio;
    BIdata["robotsize"] = robotsize;
    BIdata["apllist"] = json::array();
    for (uint32_t i = 0; i < apllist.size(); i++)
    {
        BIarchipelago &apl = apllist[i];
        json &apljson = BIdata["apllist"][i];
        apljson["aplnum"] = apl.aplnum;

        apljson["pointmap"] = json::array();
        json &PMjson = apljson["pointmap"];
        for (uint32_t j = 0; j < apl.pointmaplen; j++)
            PMjson[j] = {apl.pointmap[j].x, apl.pointmap[j].y};

        apljson["islandlist"] = json::array();
        json &ILjson = apljson["islandlist"];
        for (uint32_t j = 0; j < apl.islandlistlen; j++)
        {
            BIisland &island_ij = apl.islandlist[j];
            json &ILjson_j = ILjson[j];
            ILjson_j["points"] = json::array();
            for (uint32_t k = 0; k < island_ij.pointslen; k++)
                ILjson_j["points"][k] = island_ij.points[k];
            ILjson_j["Blink"] = json::array();
            for (uint32_t k = 0; k < island_ij.Blinklen; k++)
                ILjson_j["Blink"][k] = island_ij.Blink[k];
        }

        apljson["bridgelist"] = json::array();
        json &BLjson = apljson["bridgelist"];
        for (uint32_t j = 0; j < apl.bridgelistlen; j++)
        {
            BIbridge &bridge_ij = apl.bridgelist[j];
            json &BLjson_j = BLjson[j];
            BLjson_j["pointS"] = bridge_ij.pointS;
            BLjson_j["pointE"] = bridge_ij.pointE;
            BLjson_j["length"] = bridge_ij.length;
            BLjson_j["core"] = {bridge_ij.core.x, bridge_ij.core.y};
            BLjson_j["Ilink"] = json::array();
            for (uint32_t k = 0; k < bridge_ij.Ilinklen; k++)
                BLjson_j["Ilink"][k] = bridge_ij.Ilink[k];
            BLjson_j["Blink"] = json::array();
            for (uint32_t k = 0; k < bridge_ij.Blinklen; k++)
                BLjson_j["Blink"][k] = bridge_ij.Blink[k];
        }
        apljson["peninsula"] = apl.peninsula;
    }
    ofstream BIfjson(name);
    BIfjson << BIdata.dump();
    BIfjson.close();
}

BImap::BImap()
{
}
BImap::~BImap()
{
}

void CTrtrt::RTRT_start(float Resolution)
{
    agent = agent_tag;

    if (agent.x >= shapeX || agent.y >= shapeY)
    {
        printf("RTRT_task error: Init point is too large!!\r\n");
        return;
    }
    uint32_t anum_agent = BIamap.at<uint16_t>(agent.y, agent.x);
    if (anum_agent == 0xffff)
    {
        printf("RTRT_task error: robot can't be here!!\r\n");
        return;
    }
    agent_apl = anum_agent;
    agent_island = BIimap.at<uint16_t>(agent.y, agent.x);
    // rouletteCut_Map.clear();
    // std::vector<BIbridge> &bridgelist = apllist[anum_agent].bridgelist;
    // for (int32_t num = 0; num < bridgelist.size(); num++)
    // {
    //     // float bridge_len = bridgelist[num].length;
    //     if (rouletteCut_Map.size() == 0)
    //     {
    //         // rouletteCut_Map[1.0f + 0.0f*bridge_len] = num;
    //         rouletteCut_Map[1.0f] = num;
    //     }
    //     else
    //     {
    //         float endkey = rouletteCut_Map.rbegin()->first;
    //         // rouletteCut_Map[endkey + 1.0f + 0.0f*bridge_len] = num;
    //         rouletteCut_Map[endkey + 1.0f] = num;
    //     }
    // }

    BIarchipelago &apl = apllist[anum_agent];
    std::vector<BIpoint> &pointmap = apl.pointmap;
    uint32_t BNodesNUM = 0;
    for (uint32_t j = 0; j < apl.bridgelistlen; j++)
    {
        BIbridge &bridge = apl.bridgelist[j];
        BNodesNUM += bridge.length / Resolution + 2;
    }
    NodeBSample_List.clear();
    NodeBSample_List.reserve(BNodesNUM);
    for (int32_t j = 0; j < apl.bridgelistlen; j++)
    {
        BIbridge &bridge = apl.bridgelist[j];
        // int32_t BNpoint = apl.CTonly_BNlink[j];
        int32_t dnum = bridge.length / Resolution + 4;
        for (int32_t k = 1; k < dnum; k++)
        {
            if (2 * k == dnum)
                continue;
            float t = (float)k / dnum;
            float t_1 = 1.0f - t;
            NodeBSample_List.push_back({{pointmap[bridge.pointS].x * t_1 + pointmap[bridge.pointE].x * t,
                                         pointmap[bridge.pointS].y * t_1 + pointmap[bridge.pointE].y * t},
                                        j});
        }
    }
    std::random_shuffle(NodeBSample_List.begin(), NodeBSample_List.end());
    std::cout << "NodeBSample_ListMax: " << NodeBSample_List.size() << std::endl;
    runkey = true;
    thread_handle = thread(&CTrtrt::RTRT_task, this);
}

void CTrtrt::RTRT_join(void)
{
    thread_handle.join();
}

bool CTrtrt::RTRT_CrashDetection(const BIedge &edge)
{
    if (global_ObsIslands.find(edge.island) != global_ObsIslands.end())
    {
        if (DObsmap.at<uint8_t>(edge.S.y, edge.S.x) || DObsmap.at<uint8_t>(edge.E.y, edge.E.x))
            return true;
        float len = edge.S % edge.E;
        int32_t MaximumDiscrete = len / 10 + 2; //$$$
        for (float denominator = 2; denominator <= MaximumDiscrete; denominator *= 2)
        {
            for (int32_t i = 1; i < denominator; i += 2)
            {
                float t = i / denominator;
                int32_t x = edge.S.x * t + (1 - t) * edge.E.x;
                int32_t y = edge.S.y * t + (1 - t) * edge.E.y;
                if (DObsmap.at<uint8_t>(y, x))
                    return true;
            }
        }
    }
    return false;
}
bool CTrtrt::RTRT_CrashDetection(const BIpoint &ps, const BIpoint &pe, int32_t islandNum)
{
    if (global_ObsIslands.find(islandNum) != global_ObsIslands.end())
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
// void CTrtrt::RTRT_LinkPoint(BIpoint par, BIpoint sub)
// {
//     CTrtrt_NodeAttribute &par_node = Node_Map[par];
//     CTrtrt_NodeAttribute &sub_node = Node_Map[sub];
//     par_node.subs.insert(sub);
//     Node_Map[sub_node.par].subs.erase(sub);
//     sub_node.par = par;
// }
void CTrtrt::RTRT_task(void)
{

    BIarchipelago &apl = apllist[agent_apl];
    std::vector<BIpoint> &pointmap = apl.pointmap;
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    std::vector<BIisland> &islandlist = apl.islandlist;
    int32_t Bpointcount = 0, Ipointcount = 0;

    std::priority_queue<VPpair, vector<VPpair>, greater<VPpair>> pQInit;
    for (int32_t bridge_num = 0; bridge_num < bridgelist.size(); bridge_num++)
    {
        BIbridge &bridge_now = bridgelist[bridge_num];
        BIpoint node_now = bridge_now.core;
        Node_Map[node_now] = {{1, 3.3e38f}, {-1, -1}, {}, bridge_num, -1};
        bridge_now.Nodes.insert(node_now);
        Bpointcount++;
    }
    for (int32_t i = 0; i < islandlist[agent_island].Blinklen; i++)
    {
        int32_t bridge_num = islandlist[agent_island].Blink[i];
        BIbridge &bridge_now = bridgelist[bridge_num];
        const BIpoint &bridge_point = *bridge_now.Nodes.begin();
        float bridge_point_cost = bridge_point % agent;
        Node_Map[bridge_point].cost = {0, bridge_point_cost};
        Node_Map[bridge_point].par = agent;
        Node_Map[bridge_point].island = agent_island;
        // Node_Map[agent].subs.insert(bridge_point);
        for (int32_t near_num : bridge_now.Blink)
        {
            BIbridge &bridge_near = bridgelist[near_num];
            const BIpoint &near_point = *bridge_near.Nodes.begin();
            float nearcost_new = (near_point % bridge_point) + bridge_point_cost;
            CTrtrt_NodeAttribute &near_point_Node = Node_Map[near_point];
            if (nearcost_new < near_point_Node.cost.cost)
            {
                near_point_Node.cost = {0, nearcost_new};
                near_point_Node.par = bridge_point;
                if (bridge_near.Ilink[0] == bridge_now.Ilink[0] ||
                    bridge_near.Ilink[1] == bridge_now.Ilink[0])
                    near_point_Node.island = bridge_now.Ilink[0];
                else
                    near_point_Node.island = bridge_now.Ilink[1];
                // Node_Map[bridge_point].subs.insert(bridge_point);
                pQInit.push({nearcost_new, near_point});
            }
        }
    }
    while (pQInit.size())
    {
        float current_cost = pQInit.top().first;
        BIpoint current_point = pQInit.top().second;
        pQInit.pop();
        if (current_cost > Node_Map[current_point].cost.cost)
            continue;
        BIbridge &bridge_now = bridgelist[Node_Map[current_point].bridge];
        const BIpoint &bridge_point = *bridge_now.Nodes.begin();
        // float bridge_point_cost = Node_Map[current_point].cost;
        for (int32_t near_num : bridge_now.Blink)
        {
            BIbridge &bridge_near = bridgelist[near_num];
            const BIpoint &near_point = *bridge_near.Nodes.begin();
            float nearcost_new = (near_point % bridge_point) + current_cost;
            CTrtrt_NodeAttribute &near_point_Node = Node_Map[near_point];
            if (nearcost_new < near_point_Node.cost.cost)
            {
                near_point_Node.cost = {0, nearcost_new};
                near_point_Node.par = bridge_point;
                if (bridge_near.Ilink[0] == bridge_now.Ilink[0] ||
                    bridge_near.Ilink[1] == bridge_now.Ilink[0])
                    near_point_Node.island = bridge_now.Ilink[0];
                else
                    near_point_Node.island = bridge_now.Ilink[1];
                pQInit.push({nearcost_new, near_point});
            }
        }
    }

    for (const auto pair : Node_Map)
        Node_Map[pair.second.par].subs.insert(pair.first);
    std::cout << "Node_MapSize: " << Node_Map.size() << std::endl;
#define RewireN ((int32_t)100)
    // 创建优先队列
    std::priority_queue<VEpair, vector<VEpair>, greater<VEpair>> eQr;
    int32_t NodeBSample_List_Count = 0;
    int tttkey = 0;
    while (runkey)
    // for (size_t whilecount = 0; whilecount < 50; whilecount++)
    {
        cv::Mat dmap = BIdmap.clone();
        // cv::Mat dmap0 = BIdmap.clone();

        // switch (whilecount)
        // {
        // case 10:
        //     DObstacles[2].x = 500;
        //     DObstacles[2].y = 250;
        //     break;
        // case 15:
        //     DObstacles[2].x = 650;
        //     DObstacles[2].y = 250;
        //     break;
        // case 25:
        //     DObstacles[2].x = 500;
        //     DObstacles[2].y = 250;
        //     break;
        // }

        long long RTRT_t0 = utime_ns();
        std::list<BIpoint> noParPointQueue;
        RTRT_DrawDObsmap();                    // 更新碰撞检测地图
        RTRT_CrashDObsIsland(noParPointQueue); // 更新碰撞检测岛

        // std::cout << "whilecount: " << whilecount << std::endl;
        // std::cout << "Node_Map25: " << Node_Map.size() << std::endl;

        // for (const auto &KVpair : Node_Map)
        // {
        //     cv::Point endPoint((int)KVpair.first.x, (int)KVpair.first.y);
        //     const BIpoint &par = KVpair.second.par;
        //     // if (par.x < 0)
        //     //     continue;
        //     cv::Point startPoint((int)(0.8 * par.x + 0.2 * KVpair.first.x), (int)(0.8 * par.y + 0.2 * KVpair.first.y));
        //     cv::line(dmap0, startPoint, endPoint, cv::Scalar(255, 0, 0), 1);
        //     endPoint = startPoint;
        //     startPoint = {(int)par.x, (int)par.y};
        //     cv::line(dmap0, startPoint, endPoint, cv::Scalar(0, 0, 255), 1);
        // }
        // cv::imshow("123", dmap0);
        /********************************更新环境信息********************************/
        if (agent_tag % agent > 10) //@@@
        {
            std::queue<BIpoint> NodeUpdateQueue;

            agent_island = BIimap.at<uint16_t>(agent_tag.y, agent_tag.x);
            Node_Map[agent_tag].bridge = -2;
            Node_Map[agent_tag].cost = {0, 0}; //@@@
            Node_Map[agent_tag].par = {-1, -1};
            Node_Map[agent_tag].subs = Node_Map[agent].subs;

            if (agent != agent_tag)
                Node_Map.erase(agent);
            agent = agent_tag;
            for (const BIpoint &Psub : Node_Map[agent].subs)
            {
                CTrtrt_NodeAttribute &Psub_Node = Node_Map[Psub];
                Psub_Node.cost = CTrtrt_Cost{RTRT_CrashDetection(agent, Psub, Psub_Node.island),
                                             20000.0f + (agent % Psub)};
                Psub_Node.par = agent;
                NodeUpdateQueue.push(Psub);
            }
            /*********************************更新点代价*********************************/
            while (NodeUpdateQueue.size())
            {
                BIpoint Pnow = NodeUpdateQueue.front();
                CTrtrt_NodeAttribute &Pnow_Node = Node_Map[Pnow];
                for (const BIpoint &Psub : Pnow_Node.subs)
                {
                    CTrtrt_NodeAttribute &Psub_Node = Node_Map[Psub];
                    Psub_Node.cost = Pnow_Node.cost +
                                     CTrtrt_Cost{RTRT_CrashDetection(Pnow, Psub, Psub_Node.island), Pnow % Psub};
                    NodeUpdateQueue.push(Psub);
                }
                NodeUpdateQueue.pop();
            }
            // 拓展待改进边
            for (const BIpoint &Psub : Node_Map[agent].subs)
            {
                if (Node_Map[Psub].island == agent_island)
                    continue;
                RTRT_AddPotentialEdges(Psub, eQr);
            }
            for (int32_t &bridgeNum : islandlist[agent_island].Blink)
            {
                BIbridge &bridge = bridgelist[bridgeNum];
                for (const BIpoint &Pbrid : bridge.Nodes)
                {
                    std::map<int, Kd_tree>::iterator ObsIsland_it = global_ObsIslands.find(agent_island);
                    if (ObsIsland_it != global_ObsIslands.end()) // 包含障碍物的岛内搜索近邻点
                    {
                        Neighbor_search search(ObsIsland_it->second, {Pbrid.x, Pbrid.y}, 25); //@@@
                        for (const Point_with_distance pwd : search)
                        {
                            BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                            // if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                            //     continue;
                            VEpair VEtemp;
                            VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % Pbrid};
                            VEtemp.second = {pointNear, Pbrid, agent_island};
                            eQr.push(VEtemp); //???优先队列是否加入启发项。
                        }
                    }
                    {
                        VEpair VEtemp;
                        VEtemp.first = Node_Map[agent].cost + CTrtrt_Cost{0, agent % Pbrid};
                        VEtemp.second = {agent, Pbrid, agent_island};
                        eQr.push(VEtemp); //???优先队列是否加入启发项。
                    }
                }
            }
            std::map<int, Kd_tree>::iterator ObsIsland_it = global_ObsIslands.find(agent_island);
            if (ObsIsland_it != global_ObsIslands.end()) // 包含障碍物的岛内搜索近邻点
            {
                Neighbor_search search(ObsIsland_it->second, {agent.x, agent.y}, 25); //@@@
                for (const Point_with_distance pwd : search)
                {
                    BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                    // if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                    //     continue;
                    VEpair VEtemp;
                    VEtemp.first = CTrtrt_Cost{RTRT_CrashDetection(agent, pointNear, agent_island), pointNear % agent};
                    VEtemp.second = {agent, pointNear, agent_island};
                    eQr.push(VEtemp); //???优先队列是否加入启发项。
                }
            }
        }
        else
        {
            /*********************************更新点代价*********************************/
            std::queue<BIpoint> NodeUpdateQueue;
            NodeUpdateQueue.push(agent);
            while (NodeUpdateQueue.size())
            {
                BIpoint Pnow = NodeUpdateQueue.front();
                CTrtrt_NodeAttribute &Pnow_Node = Node_Map[Pnow];
                for (const BIpoint &Psub : Pnow_Node.subs)
                {
                    CTrtrt_NodeAttribute &Psub_Node = Node_Map[Psub];
                    Psub_Node.cost = Pnow_Node.cost +
                                     CTrtrt_Cost{RTRT_CrashDetection(Pnow, Psub, Psub_Node.island), Pnow % Psub};
                    NodeUpdateQueue.push(Psub);
                }
                NodeUpdateQueue.pop();
            }
        }
        for (BIpoint &noParPoint : noParPointQueue)
            RTRT_AddPotentialEdges(noParPoint, eQr);
        // std::cout << "Node_Map3: " << Node_Map.size() << std::endl;

        /**********************************随机采样**********************************/
        for (int32_t SampleCount = 0; SampleCount < RewireN; SampleCount++)
        {
            BIpoint randpoint;
            float random_p1 = rand() / (float(RAND_MAX));
            if (random_p1 < 0.85f || global_ObsIslands.size() == 0) // 在桥上采样//@@@
            {
                if (NodeBSample_List_Count >= NodeBSample_List.size()) //@@@
                    continue;
                auto &BSample_Pair = NodeBSample_List[NodeBSample_List_Count++];
                int32_t bridgeNum = BSample_Pair.second;
                BIbridge &bridge = bridgelist[bridgeNum];
                randpoint = BSample_Pair.first;

                for (int32_t Ilink_num : bridge.Ilink)
                {
                    for (int32_t BnearNum : islandlist[Ilink_num].Blink) // 添加通过桥上点改进树的潜力边
                    {
                        if (BnearNum == bridgeNum)
                            continue;
                        BIbridge &bridgeNear = bridgelist[BnearNum];
                        for (const BIpoint &pointNear : bridgeNear.Nodes) // 惰性思想，代价的保守估计
                        {
                            if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                                continue;
                            VEpair VEtemp;
                            VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % randpoint};
                            VEtemp.second = {pointNear, randpoint, Ilink_num};
                            eQr.push(VEtemp); //???优先队列是否加入启发项。
                        }
                    }
                    std::map<int, Kd_tree>::iterator ObsIsland_it = global_ObsIslands.find(Ilink_num);
                    if (ObsIsland_it != global_ObsIslands.end()) // 包含障碍物的岛内搜索近邻点
                    {
                        Neighbor_search search(ObsIsland_it->second, {randpoint.x, randpoint.y}, 25); //@@@
                        for (const Point_with_distance pwd : search)
                        {
                            BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                            if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                                continue;
                            VEpair VEtemp;
                            VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % randpoint};
                            VEtemp.second = {pointNear, randpoint, Ilink_num};
                            eQr.push(VEtemp); //???优先队列是否加入启发项。
                        }
                    }
                    if (Ilink_num == agent_island)
                    {
                        VEpair VEtemp;
                        VEtemp.first = Node_Map[agent].cost + CTrtrt_Cost{0, agent % randpoint};
                        VEtemp.second = {agent, randpoint, Ilink_num};
                        eQr.push(VEtemp); //???优先队列是否加入启发项。
                    }
                }
                bridge.Nodes.insert(randpoint);
                Node_Map[randpoint] = {{1, 3.3e38f}, {-1, -1}, {}, bridgeNum, -1}; //!!!
                // RTRT_AddPotentialEdges(randpoint, eQr);
                Bpointcount++;
            }
            else // 在包含障碍物的岛上采样
            {
                float roulette_max = rouletteOds_Map.rbegin()->first;
                float Prandom1 = rand() / (float(RAND_MAX)) * roulette_max;
                auto randObsMap = rouletteOds_Map.upper_bound(Prandom1);
                CTrtrt_DynamicObstacle &Obs = DObstacles[randObsMap->second];
                for (size_t errCount = 0; errCount < 20; errCount++) // 采样容错次数//@@@
                {
                    float random_theta = rand() / (float(RAND_MAX)) * 2 * 3.141592954f;
                    float random_r = rand() / (float(RAND_MAX));
                    random_r = sqrtf32(random_r) * Obs.sampleRadius;
                    float x = cosf32(random_theta) * random_r + Obs.x;
                    float y = sinf32(random_theta) * random_r + Obs.y;
                    int imgx = (int)x, imgy = (int)y;
                    if (imgx < 0 || imgy < 0 || imgx >= shapeX || imgy >= shapeY)
                        continue;
                    if (DObsmap.at<uint8_t>(imgy, imgx))
                        continue;
                    int randIsland_num = BIimap.at<uint16_t>(imgy, imgx);
                    if (Obs.ObsIslands.find(randIsland_num) != Obs.ObsIslands.end())
                    {
                        Neighbor_search search(global_ObsIslands[randIsland_num], {x, y}, 25); //@@@
                        if (search.begin() != search.end() && search.begin()->second < 64.0f) //@@@
                            continue;

                        randpoint.x = x;
                        randpoint.y = y;
                        for (int32_t BnearNum : islandlist[randIsland_num].Blink) // 添加通过桥上点改进树的潜力边
                        {
                            BIbridge &bridgeNear = bridgelist[BnearNum];
                            for (const BIpoint &pointNear : bridgeNear.Nodes) // 惰性思想，代价的保守估计
                            {
                                if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                                    continue;
                                VEpair VEtemp;
                                VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % randpoint};
                                VEtemp.second = {pointNear, randpoint, randIsland_num};
                                eQr.push(VEtemp); //???优先队列是否加入启发项。
                            }
                        }
                        for (const Point_with_distance pwd : search)
                        {
                            BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                            if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                                continue;
                            VEpair VEtemp;
                            VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % randpoint};
                            VEtemp.second = {pointNear, randpoint, randIsland_num};
                            eQr.push(VEtemp); //???优先队列是否加入启发项。
                        }
                        if (randIsland_num == agent_island)
                        {
                            VEpair VEtemp;
                            VEtemp.first = Node_Map[agent].cost + CTrtrt_Cost{0, agent % randpoint};
                            VEtemp.second = {agent, randpoint, randIsland_num};
                            eQr.push(VEtemp); //???优先队列是否加入启发项。
                        }
                        global_ObsIslands[randIsland_num].insert(Point_2{randpoint.x, randpoint.y});
                        Node_Map[randpoint] = {{1, 3.3e38f}, {-1, -1}, {}, -1, randIsland_num};
                        // RTRT_AddPotentialEdges(randpoint, eQr);
                        Ipointcount++;
                        break;
                    }
                }
            }
        }
        // std::cout << "Node_MapSize4: " << Node_Map.size() << std::endl;

        // std::priority_queue<VEpair, std::vector<VEpair>, std::greater<VEpair>> Qr_temp(eQr);
        // while (Qr_temp.size())
        // {
        //     const VEpair &ve_top = Qr_temp.top();
        //     cv::Point startPoint((int)ve_top.second.S.x, (int)ve_top.second.S.y);

        //     cv::Point endPoint((int)ve_top.second.E.x, (int)ve_top.second.E.y);

        //     cv::line(dmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 1.5);
        //     Qr_temp.pop();
        // }

        /***********************************重连接***********************************/
        long long Rewire_t0 = utime_ns();
        while (eQr.size()) // && utime_ns() - Rewire_t0 < 35e6)
        {
            const VEpair &ve_top = eQr.top();
            const BIedge &edge_top = ve_top.second;
            const BIpoint &point_topE = edge_top.E;
            CTrtrt_NodeAttribute &pointNode_Etop = Node_Map[point_topE];
            if (pointNode_Etop.cost > ve_top.first) //???优先队列是否加入启发项。
            {
                CTrtrt_Cost trueCost = Node_Map[edge_top.S].cost +
                                       CTrtrt_Cost{RTRT_CrashDetection(edge_top), edge_top.S % point_topE};
                if (pointNode_Etop.cost > trueCost)
                {
                    // 更新当前节点的相关属性
                    pointNode_Etop.cost = trueCost;
                    pointNode_Etop.island = edge_top.island;
                    // 更新树的链接状态
                    Node_Map[pointNode_Etop.par].subs.erase(point_topE);
                    Node_Map[edge_top.S].subs.insert(point_topE);
                    pointNode_Etop.par = edge_top.S;
                    // 添加待优化节点的潜在边
                    if (pointNode_Etop.bridge >= 0)
                    {
                        for (int32_t Ilink_num : bridgelist[pointNode_Etop.bridge].Ilink)
                        {
                            for (int32_t BnearNum : islandlist[Ilink_num].Blink) // 添加通过桥上点改进树的潜力边
                            {
                                if (BnearNum == pointNode_Etop.bridge)
                                    continue;
                                BIbridge &bridgeNear = bridgelist[BnearNum];
                                for (const BIpoint &pointNear : bridgeNear.Nodes) // 惰性思想，代价的保守估计
                                {
                                    CTrtrt_Cost ConservativeCost = trueCost + CTrtrt_Cost{0, pointNear % point_topE};
                                    if (Node_Map[pointNear].cost > ConservativeCost)
                                    {
                                        VEpair VEtemp;
                                        VEtemp.first = ConservativeCost;
                                        VEtemp.second = {point_topE, pointNear, Ilink_num};
                                        eQr.push(VEtemp); //???优先队列是否加入启发项。
                                    }
                                }
                            }
                            std::map<int, Kd_tree>::iterator ObsIsland_it = global_ObsIslands.find(Ilink_num);
                            if (ObsIsland_it != global_ObsIslands.end()) // 包含障碍物的岛内搜索近邻点
                            {
                                Neighbor_search search(ObsIsland_it->second, {point_topE.x, point_topE.y}, 25); //@@@
                                for (const Point_with_distance pwd : search)
                                {
                                    BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                                    CTrtrt_Cost ConservativeCost = trueCost + CTrtrt_Cost{0, pointNear % point_topE};
                                    if (Node_Map[pointNear].cost > ConservativeCost)
                                    {
                                        VEpair VEtemp;
                                        VEtemp.first = ConservativeCost;
                                        VEtemp.second = {point_topE, pointNear, Ilink_num};
                                        eQr.push(VEtemp); //???优先队列是否加入启发项。
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        Neighbor_search search(global_ObsIslands[pointNode_Etop.island], {point_topE.x, point_topE.y}, 25); //@@@
                        for (int32_t BnearNum : islandlist[pointNode_Etop.island].Blink)                                    // 添加通过桥上点改进树的潜力边
                        {
                            BIbridge &bridgeNear = bridgelist[BnearNum];
                            for (const BIpoint &pointNear : bridgeNear.Nodes) // 惰性思想，代价的保守估计
                            {
                                CTrtrt_Cost ConservativeCost = trueCost + CTrtrt_Cost{0, pointNear % point_topE};
                                if (Node_Map[pointNear].cost > ConservativeCost)
                                {
                                    VEpair VEtemp;
                                    VEtemp.first = ConservativeCost;
                                    VEtemp.second = {point_topE, pointNear, pointNode_Etop.island};
                                    eQr.push(VEtemp); //???优先队列是否加入启发项。
                                }
                            }
                        }
                        for (const Point_with_distance pwd : search)
                        {
                            BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                            CTrtrt_Cost ConservativeCost = trueCost + CTrtrt_Cost{0, pointNear % point_topE};
                            if (Node_Map[pointNear].cost > ConservativeCost)
                            {
                                VEpair VEtemp;
                                VEtemp.first = ConservativeCost;
                                VEtemp.second = {point_topE, pointNear, pointNode_Etop.island};
                                eQr.push(VEtemp); //???优先队列是否加入启发项。
                            }
                        }
                    }
                }
            }
            eQr.pop();
        }
        std::cout << "RTRT_t: " << utime_ns() - RTRT_t0 << std::endl;
        for (const auto &KVpair : Node_Map)
        {
            cv::Point endPoint((int)KVpair.first.x, (int)KVpair.first.y);
            const BIpoint &par = KVpair.second.par;
            if (par.x < 0)
                continue;
            cv::Point startPoint((int)(0.2 * par.x + 0.8 * KVpair.first.x), (int)(0.2 * par.y + 0.8 * KVpair.first.y));
            cv::line(dmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 1);
            endPoint = startPoint;
            startPoint = {(int)par.x, (int)par.y};
            cv::line(dmap, startPoint, endPoint, cv::Scalar(255, 0, 0), 1);
        }
        for (const auto ObsPair : DObstacles)
        {
            float sinangle = sinf32(ObsPair.second.angle);
            float cosangle = cosf32(ObsPair.second.angle);
            for (const BIpoint &point : ObsPair.second.CheckPoints)
            {
                int imgx = cosangle * point.x - sinangle * point.y + ObsPair.second.x;
                int imgy = sinangle * point.x + cosangle * point.y + ObsPair.second.y;

                cv::Point center(imgx, imgy);
                cv::circle(dmap, center, 2, cv::Scalar(255, 255, 255), cv::FILLED);
            }
            /* code */
        }
        std::cout << "Node_MapSize: " << Node_Map.size() << std::endl;
        std::cout << Bpointcount << "," << Ipointcount << "," << Bpointcount + Ipointcount << std::endl
                  << std::endl;
        ShowMap = dmap.clone();
        // cv::imshow("BIdmap", dmap);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // cv::imshow("BIdmap", ShowMap);
        // cv::waitKey(0);
    }
}
void CTrtrt::RTRT_AddPotentialEdges(const BIpoint &point,
                                    std::priority_queue<VEpair, vector<VEpair>, greater<VEpair>> &Qr)
{
    BIarchipelago &apl = apllist[agent_apl];
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    std::vector<BIisland> &islandlist = apl.islandlist;
    CTrtrt_NodeAttribute &point_Node = Node_Map[point];
    int32_t bridgeNum = point_Node.bridge;
    if (bridgeNum >= 0)
    {
        BIbridge &bridge = bridgelist[bridgeNum];
        for (int32_t Ilink_num : bridge.Ilink)
        {
            for (int32_t BnearNum : islandlist[Ilink_num].Blink) // 添加通过桥上点改进树的潜力边
            {
                if (BnearNum == bridgeNum)
                    continue;
                BIbridge &bridgeNear = bridgelist[BnearNum];
                for (const BIpoint &pointNear : bridgeNear.Nodes) // 惰性思想，代价的保守估计
                {
                    // if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                    //     continue;
                    VEpair VEtemp;
                    VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % point};
                    VEtemp.second = {pointNear, point, Ilink_num};
                    Qr.push(VEtemp); //???优先队列是否加入启发项。
                }
            }
            std::map<int, Kd_tree>::iterator ObsIsland_it = global_ObsIslands.find(Ilink_num);
            if (ObsIsland_it != global_ObsIslands.end()) // 包含障碍物的岛内搜索近邻点
            {
                Neighbor_search search(ObsIsland_it->second, {point.x, point.y}, 25); //@@@
                for (const Point_with_distance pwd : search)
                {
                    BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
                    // if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                    //     continue;
                    VEpair VEtemp;
                    VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % point};
                    VEtemp.second = {pointNear, point, Ilink_num};
                    Qr.push(VEtemp); //???优先队列是否加入启发项。
                }
            }
            if (Ilink_num == agent_island)
            {
                VEpair VEtemp;
                VEtemp.first = Node_Map[agent].cost + CTrtrt_Cost{0, agent % point};
                VEtemp.second = {agent, point, Ilink_num};
                Qr.push(VEtemp); //???优先队列是否加入启发项。
            }
        }
    }
    else
    {
        int32_t randIsland_num = point_Node.island;
        for (int32_t BnearNum : islandlist[randIsland_num].Blink) // 添加通过桥上点改进树的潜力边
        {
            BIbridge &bridgeNear = bridgelist[BnearNum];
            for (const BIpoint &pointNear : bridgeNear.Nodes) // 惰性思想，代价的保守估计
            {
                // if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
                //     continue;
                VEpair VEtemp;
                VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % point};
                VEtemp.second = {pointNear, point, randIsland_num};
                Qr.push(VEtemp); //???优先队列是否加入启发项。
            }
        }
        Neighbor_search search(global_ObsIslands[randIsland_num], {point.x, point.y}, 25); //@@@
        for (const Point_with_distance pwd : search)
        {
            BIpoint pointNear = {(float)pwd.first.x(), (float)pwd.first.y()};
            // if (Node_Map[pointNear].cost >= CTrtrt_Cost{1, 3.3e37f})
            //     continue;
            VEpair VEtemp;
            VEtemp.first = Node_Map[pointNear].cost + CTrtrt_Cost{0, pointNear % point};
            VEtemp.second = {pointNear, point, randIsland_num};
            Qr.push(VEtemp); //???优先队列是否加入启发项。
        }
        if (randIsland_num == agent_island)
        {
            VEpair VEtemp;
            VEtemp.first = Node_Map[agent].cost + CTrtrt_Cost{0, agent % point};
            VEtemp.second = {agent, point, randIsland_num};
            Qr.push(VEtemp); //???优先队列是否加入启发项。
        }
    }
}
void CTrtrt::RTRT_SetAgent(BIpoint &point)
{
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("SetAgent error: Init point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(point.y, point.x);
    if (anum_S == 0xffff)
    {
        printf("SetAgent error: robot can't be here!!\r\n");
        return;
    }
    agent_tag = point;
}
void CTrtrt::RTRT_SetGoal(BIpoint &point)
{
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("SetGoal error: Init point is too large!!\r\n");
        return;
    }
    uint32_t anum_S = BIamap.at<uint16_t>(point.y, point.x);
    if (anum_S == 0xffff)
    {
        printf("SetGoal error: robot can't be here!!\r\n");
        return;
    }
    goal = point;
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    // goal.bridge = -1;
    // goal.cost = 0;
}

void CTrtrt::RTRT_AddDynamicObstacle(int num, CTrtrt_DynamicObstacle &obs, float CheckLen)
{
    DObstacles[num] = obs;
    if (rouletteOds_Map.size() == 0)
    {
        rouletteOds_Map[3.1415926f * obs.sampleRadius * obs.sampleRadius] = num;
        // printf("%f\r\n",);
    }
    else
    {
        float endkey = rouletteOds_Map.rbegin()->first;
        rouletteOds_Map[endkey + 3.1415926f * obs.sampleRadius * obs.sampleRadius] = num;
    }

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

void CTrtrt::RTRT_DelDynamicObstacle(int num)
{
    DObstacles.erase(num);
    std::map<float, int> roulette_temp;
    auto it = DObstacles.begin();
    float sampleRadius = it->second.sampleRadius;
    float endkey = 3.1415926f * sampleRadius * sampleRadius;
    roulette_temp[endkey] = it->first;
    ++it;
    for (; it != DObstacles.end(); ++it)
    {
        sampleRadius = it->second.sampleRadius;
        endkey += 3.1415926f * sampleRadius * sampleRadius;
        roulette_temp[endkey] = it->first;
    }
    rouletteOds_Map = roulette_temp;
}
bool CTrtrt::RTRT_CrashDObsPoint(CTrtrt_DynamicObstacle &obs, BIpoint &point)
{
    if (obs.DObsType == 0)
    {
        for (const BIpoint &obspoint : obs.OutLine)
        {
            float dx = obspoint.x - point.x;
            float dy = obspoint.y - point.y;
            if (sqrtf32(dx * dx + dy * dy) <= obs.radius)
                return true;
        }
    }
    else
    {
        BIpoint &point0 = obs.OutLine[0];
        BIpoint &point1 = obs.OutLine[1];
        if (point.x > point0.x && point.y > point0.y &&
            point.x < point1.x && point.y < point1.y)
            return true;
    }
    return false;
}

void CTrtrt::RTRT_DrawDObsmap(void)
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
void CTrtrt::RTRT_CrashDObsIsland(std::list<BIpoint> &noParPointQueue)
{
    BIarchipelago &apl = apllist[agent_apl];
    std::vector<BIbridge> &bridgelist = apl.bridgelist;
    std::vector<BIisland> &islandlist = apl.islandlist;

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
            if (inum != inum_old)
            {
                obs.ObsIslands.insert(inum);
                ObsIslands_temp.insert(inum);
                inum_old = inum;
            }
        }
    }
    // 删除global_ObsIslands中在ObsIslands_temp中没有的键值对
    std::set<int32_t> PendingBridge; // 记录所有受障碍物变化影响的桥
    for (auto it = global_ObsIslands.begin(); it != global_ObsIslands.end();)
    {
        std::set<int32_t>::iterator key = ObsIslands_temp.find(it->first);
        if (key == ObsIslands_temp.end())
        {
            std::cout << "it->second: " << it->second.size() << std::endl;
            std::cout << "Node_Map1: " << Node_Map.size() << std::endl;
            for (auto KDpoint : it->second) // 迭代删除自由岛中所有的节点
                Node_Map.erase({(float)KDpoint.x(), (float)KDpoint.y()});
            std::cout << "Node_Map2: " << Node_Map.size() << std::endl;
            BIisland &island = islandlist[it->first];
            PendingBridge.insert(island.Blink.begin(), island.Blink.end());
            if (it->first == agent_island)
            {
                CTrtrt_NodeAttribute &agentNode = Node_Map[agent];
                for (auto agent_it = agentNode.subs.begin(); agent_it != agentNode.subs.end();)
                {
                    if (Node_Map.count(*agent_it) == 0)
                        agent_it = agentNode.subs.erase(agent_it); // 删除元素，并返回指向下一个元素的迭代器
                    else
                        ++agent_it; // 不删除元素，移动到下一个元素
                }
            }
            it = global_ObsIslands.erase(it);
        }
        else
        {
            ObsIslands_temp.erase(key);
            ++it;
        }
    }

    std::queue<BIpoint> NodeUpdateQueue;
    for (int32_t bridgeNum : PendingBridge)
    {
        BIbridge &bridge = bridgelist[bridgeNum];
        for (const BIpoint &point : bridge.Nodes)
        {
            CTrtrt_NodeAttribute &pointNode = Node_Map[point];
            // for (const BIpoint &subPoint : pointNode.subs) // 擦除不存在的子节点//岛中被删除的节点
            //     if (Node_Map.count(subPoint) == 0)
            //         pointNode.subs.erase(subPoint);
            for (auto it = pointNode.subs.begin(); it != pointNode.subs.end();)
            {
                // const BIpoint &subPoint = *it;
                if (Node_Map.count(*it) == 0)
                    it = pointNode.subs.erase(it); // 删除元素，并返回指向下一个元素的迭代器
                else
                    ++it; // 不删除元素，移动到下一个元素
            }
            if (Node_Map.count(pointNode.par) == 0)
            {
                pointNode.par = {-1, -1};
                pointNode.cost = {1, 20000.0f};
                NodeUpdateQueue.push(point);
                noParPointQueue.push_back(point);
            }
        }
    }
    while (NodeUpdateQueue.size())
    {
        BIpoint Pnow = NodeUpdateQueue.front();
        CTrtrt_NodeAttribute &Pnow_Node = Node_Map[Pnow];
        for (const BIpoint &Psub : Pnow_Node.subs)
        {
            CTrtrt_NodeAttribute &Psub_Node = Node_Map[Psub];
            Psub_Node.cost = Pnow_Node.cost + CTrtrt_Cost{1, Pnow % Psub};
            NodeUpdateQueue.push(Psub);
        }
        NodeUpdateQueue.pop();
    }

    // 将ObsIslands_temp中有但global_ObsIslands中没有的键值对添加到global_ObsIslands中
    for (int key : ObsIslands_temp)
        global_ObsIslands[key]; // 添加一个空的 vector，或者你可以初始化为特定的值
}

// bool CTrtrt::RTRT_SampleOnline(BIpoint &randpoint) // 分割线上采样
// {
//     float roulette_max = rouletteCut_Map.rbegin()->first;
//     float Prandom1 = rand() / (float(RAND_MAX)) * roulette_max;
//     auto randBridgeMap = rouletteCut_Map.upper_bound(Prandom1);
//     BIbridge &bridge = apllist[agent_apl].bridgelist[randBridgeMap->second];

//     BIpoint &pointS = apllist[agent_apl].pointmap[bridge.pointS];
//     BIpoint &pointE = apllist[agent_apl].pointmap[bridge.pointE];
//     float random_t = rand() / (float(RAND_MAX));
//     randpoint.x = random_t * pointS.x + (1 - random_t) * pointE.x;
//     randpoint.y = random_t * pointS.y + (1 - random_t) * pointE.y;

//     Node_Map[randpoint] = {{1, 3.3e38f}, {-1, -1}, {}, randBridgeMap->second, -1};
//     // randpoint.bridge = randBridgeMap->second;
//     // randpoint.cost = -1.0f;
//     // randpoint = {-1.0f,
//     //              {random_t * pointS.x + (1 - random_t) * pointE.x,
//     //               random_t * pointS.y + (1 - random_t) * pointE.y},
//     //              NULL,
//     //              {},
//     //              randBridgeMap->second};

//     return true;
// }

// bool CTrtrt::RTRT_SampleOnline(std::list<BIpoint> &randpoints, int Sample_num) // 分割线上采样
// {
//     randpoints.clear();
//     float roulette_max = rouletteCut_Map.rbegin()->first;
//     for (int Sample_n = 0; Sample_n < Sample_num; Sample_n++)
//     {
//         float Prandom1 = rand() / (float(RAND_MAX)) * roulette_max;
//         auto randBridgeMap = rouletteCut_Map.upper_bound(Prandom1);
//         BIbridge &bridge = apllist[agent_apl].bridgelist[randBridgeMap->second];

//         BIpoint &pointS = apllist[agent_apl].pointmap[bridge.pointS];
//         BIpoint &pointE = apllist[agent_apl].pointmap[bridge.pointE];
//         float random_t = rand() / (float(RAND_MAX));
//         float x = random_t * pointS.x + (1 - random_t) * pointE.x;
//         float y = random_t * pointS.y + (1 - random_t) * pointE.y;
//         randpoints.push_back({x, y});
//         Node_Map[{x, y}] = {{1, 3.3e38f}, {-1, -1}, {}, randBridgeMap->second, -1};
//     }

//     return true;
// }

bool CTrtrt::RTRT_SampleOnobs(BIpoint &randpoint, int errCount_max) // 动态障碍物采样,单点
{
    float roulette_max = rouletteOds_Map.rbegin()->first;
    float Prandom1 = rand() / (float(RAND_MAX)) * roulette_max;
    auto randObsMap = rouletteOds_Map.upper_bound(Prandom1);
    CTrtrt_DynamicObstacle &Obs = DObstacles[randObsMap->second];
    // printf("%f,%d;\r\n", randObs->first, randObs->second);
    for (size_t errCount = 0; errCount < errCount_max; errCount++) // 采样容错次数
    {
        float random_theta = rand() / (float(RAND_MAX)) * 2 * 3.141592954f;
        float random_r = rand() / (float(RAND_MAX));
        random_r = sqrtf32(random_r) * Obs.sampleRadius;
        float x = cosf32(random_theta) * random_r + Obs.x;
        float y = sinf32(random_theta) * random_r + Obs.y;
        int imgx = (int)x, imgy = (int)y;
        if (DObsmap.at<uint8_t>(imgy, imgx))
            continue;
        int randIsland_num = BIimap.at<uint16_t>(imgy, imgx);
        if (Obs.ObsIslands.find(randIsland_num) != Obs.ObsIslands.end())
        {
            randpoint.x = x;
            randpoint.y = y;
            return true;
        }
    }
    return false;
}

bool CTrtrt::RTRT_SampleOnobs(std::list<BIpoint> &randpoints, int Sample_num, int errCount_max) // 动态障碍物采样，批量
{
    randpoints.clear();
    float roulette_max = rouletteOds_Map.rbegin()->first;
    for (int Sample_n = 0; Sample_n < Sample_num; Sample_n++)
    {
        float Prandom1 = rand() / (float(RAND_MAX)) * roulette_max;
        auto randObsMap = rouletteOds_Map.upper_bound(Prandom1);
        CTrtrt_DynamicObstacle &Obs = DObstacles[randObsMap->second];
        for (int errCount = 0; errCount < errCount_max; errCount++) // 采样容错率
        {
            float random_theta = rand() / (float(RAND_MAX)) * 2 * 3.141592954f;
            float random_r = rand() / (float(RAND_MAX));
            random_r = sqrtf32(random_r) * Obs.sampleRadius;
            float x = cosf32(random_theta) * random_r + Obs.x;
            float y = sinf32(random_theta) * random_r + Obs.y;
            int imgx = (int)x, imgy = (int)y;
            if (DObsmap.at<uint8_t>(imgy, imgx))
                continue;
            int randIsland_num = BIimap.at<uint16_t>(imgy, imgx);
            if (Obs.ObsIslands.find(randIsland_num) != Obs.ObsIslands.end())
            {
                randpoints.push_back({x, y});
                break;
            }
        }
    }
    if (randpoints.size())
        return true;
    else
        return false;
}

// void RTRT_AddDynamicObstacle(int num, CTrtrt_DynamicObstacle &obs) // CTrtrt::
// {
//     // DObstacles[num] = obs;
//     CTrtrt_DynamicObstacle &newObs = obs;
//     // CTrtrt_DynamicObstacle &newObs = DObstacles[num];

//     float min_x = 3.3e38f, min_y = 3.3e38f, max_x = -3.3e38f, max_y = -3.3e38f;
//     for (const BIpoint &point : obs.OutLine)
//     {
//         if (min_x > point.x)
//             min_x = point.x;
//         if (min_y > point.y)
//             min_y = point.y;
//         if (max_x < point.x)
//             max_x = point.x;
//         if (max_y < point.y)
//             max_y = point.y;
//     }
//     min_x -= 5;
//     min_y -= 5;
//     max_x += 5;
//     max_y += 5;

//     if (obs.DObsType == 0)
//     {
//         min_x -= obs.radius;
//         min_y -= obs.radius;
//         newObs.img_x0 = min_x;
//         newObs.img_y0 = min_y;
//         max_x += obs.radius;
//         max_y += obs.radius;

//         newObs.DObsImg = cv::Mat::zeros((int)(max_y - min_y), (int)(max_x - min_x), CV_8UC1);
//         for (const BIpoint &point : obs.OutLine)
//         {
//             cv::circle(newObs.DObsImg, cv::Point((int)(point.x - min_x), (int)(point.y - min_y)),
//                        newObs.radius, cv::Scalar(1), cv::FILLED);
//         }
//     }
//     else
//     {
//         newObs.img_x0 = min_x;
//         newObs.img_y0 = min_y;
//         newObs.DObsImg = cv::Mat::zeros((int)(max_y - min_y), (int)(max_x - min_x), CV_8UC1);
//         std::vector<std::vector<cv::Point>> polygons(1);
//         polygons[0].reserve(newObs.OutLine.size() + 5);
//         for (const BIpoint &point : obs.OutLine)
//         {
//             polygons[0].push_back({(int)point.x, (int)point.y});
//         }
//         cv::fillPoly(newObs.DObsImg, polygons, cv::Scalar(1));
//     }
// }
