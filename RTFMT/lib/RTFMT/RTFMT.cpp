#include "RTFMT.h"

long long utime_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<long long>(1000000000UL) * static_cast<long long>(ts.tv_sec) +
           static_cast<long long>(ts.tv_nsec);
}

void RTFMTplanner::SetAgent(RTpoint &point)
{
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("SetAgent error: Init point is too large!!\r\n");
        return;
    }
    if (Obsmap.at<uint8_t>(point.y, point.x))
    {
        printf("SetAgent error: robot can't be here!!\r\n");
        return;
    }
    if (Node_Map.size() == 0)
    {
        Node_Map[point].cost = {0, 0};
        Node_Map[point].par = {-1, -1};
        Node_Map[point].subs = {};
        root = point;
        NeighborSearchTree.insert({point.x, point.y});
        agent = root;
    }
    else
    {
        agent = point;
    }
}

void RTFMTplanner::SetGoal(RTpoint &point)
{
    if (point.x >= shapeX || point.y >= shapeY)
    {
        printf("SetGoal error: Init point is too large!!\r\n");
        return;
    }
    if (Obsmap.at<uint8_t>(point.y, point.x))
    {
        printf("SetAgent error: robot can't be here!!\r\n");
        return;
    }
    if (goal != point)
    {
        goal = point;
        visited_set.clear();
    }
}

void RTFMTplanner::InitializeInvironment(const char *IMGmapc, int th)
{
    // 读取输入的图像
    cv::Mat inputImage = cv::imread(IMGmapc, cv::IMREAD_GRAYSCALE);

    if (inputImage.empty())
    {
        std::cerr << "Failed to load input image." << std::endl;
        return;
    }

    // 对图像进行二值化处理
    cv::threshold(inputImage, Freemap, th, 255, cv::THRESH_BINARY);

    // 确保输出矩阵Freemap的数据类型为CV_8U
    if (Freemap.type() != CV_8U)
    {
        Freemap.convertTo(Freemap, CV_8U);
    }
    cv::bitwise_not(Freemap, Obsmap);
    Freemap = cv::imread(IMGmapc);
    shapeX = Obsmap.cols;
    shapeY = Obsmap.rows;
}

void RTFMTplanner::AddDynamicObstacle(int num, RTFMT_DynamicObstacle &obs)
{
    DObstacles[num] = obs;
}

void RTFMTplanner::DelDynamicObstacle(int num)
{
    DObstacles.erase(num);
}

void RTFMTplanner::UpdateTree(void)
{
    // int debugkey = 0;
    if (agent != root)
    {
        Neighbor_search search_near(NeighborSearchTree, {agent.x, agent.y}, 1); //@@@
        RTpoint x_near = {(float)search_near.begin()->first.x(), (float)search_near.begin()->first.y()};
        if (x_near != root)
        {
            RTpoint x_temp = x_near;
            RTpoint x_old = {-1, -1};
            while (x_temp != root)
            {
                RTpoint x_par = Node_Map[x_temp].par;
                Node_Map[x_temp].par = x_old;
                Node_Map[x_temp].subs.insert(x_par);
                Node_Map[x_temp].subs.erase(x_old);

                // Node_Map[x_par].subs.erase(x_temp);
                // Node_Map[x_par].par = x_temp;
                x_old = x_temp;
                x_temp = x_par;
            }
            Node_Map[x_temp].par = x_old;
            Node_Map[x_temp].subs.erase(x_old);
            root = x_near;
            Node_Map[root].cost = {0, 0};
            // Node_Map[root].par = {-1,-1};
        }
    }
    // {
    //     cv::Mat dmap = Freemap.clone();
    //     for (const auto &KVpair : Node_Map)
    //     {
    //         for (const auto psub : Node_Map[KVpair.first].subs)
    //         {
    //             cv::Point endPoint((int)psub.x, (int)psub.y);
    //             // const RTpoint &par = KVpair.second.par;
    //             cv::circle(dmap, endPoint, 2, cv::Scalar(0, 255, 0), cv::FILLED);
    //             // if (par.x < 0)
    //             //     continue;
    //             if (KVpair.second.cost < RTFMT_Cost{1, 0})
    //             {
    //                 cv::Point startPoint((int)(0.2 * KVpair.first.x + 0.8 * psub.x), (int)(0.2 * KVpair.first.y + 0.8 * psub.y));
    //                 cv::line(dmap, startPoint, endPoint, cv::Scalar(0, 0, 255), 1);
    //                 endPoint = startPoint;
    //                 startPoint = {(int)KVpair.first.x, (int)KVpair.first.y};
    //                 cv::line(dmap, startPoint, endPoint, cv::Scalar(255, 0, 0), 1);
    //             }
    //             else
    //             {
    //                 cv::Point startPoint((int)KVpair.first.x, (int)KVpair.first.y);
    //                 cv::line(dmap, startPoint, endPoint, cv::Scalar(255, 0, 255), 1);
    //             }
    //         }
    //     }
    //     cv::imshow("DEBUGmap", dmap);
    //     cv::waitKey(1);
    // }
    // printf("RTpoint x_near");
    std::queue<RTpoint> Qup;
    // std::set<RTpoint> upset;
    // upset.insert(root);
    Qup.push(root);
    while (Qup.size())
    {
        RTpoint Pnow = Qup.front();
        Qup.pop();
        RTFMT_NodeAttribute &Pnow_Node = Node_Map[Pnow];
        for (const RTpoint &Psub : Pnow_Node.subs)
        {
            // if(upset.count(Psub))
            //     continue;
            // upset.insert(Psub);
            RTFMT_NodeAttribute &Psub_Node = Node_Map[Psub];
            bool collision_old = Psub_Node.cost.collision;
            Psub_Node.cost = Pnow_Node.cost +
                             RTFMT_Cost{CrashDetection(Pnow, Psub), Pnow % Psub};
            if (Psub_Node.cost.collision == true && collision_old == false)
                N_b.insert(Psub);
            Qup.push(Psub);
        }
    }
    // printf("while (Qup.size())\r\n");
}

void RTFMTplanner::DrawDObsmap(void)
{
    DObsmap = cv::Mat::zeros(shapeY, shapeX, CV_8U);

    for (const auto &pair : DObstacles)
    {
        const RTFMT_DynamicObstacle &obs = pair.second;
        float sinangle = sinf32(obs.angle);
        float cosangle = cosf32(obs.angle);
        if (obs.DObsType == 0)
        {
            for (const RTpoint &obspoint : obs.OutLine)
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
            for (const RTpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                polygons[0].push_back({imgx, imgy});
            }
            cv::fillPoly(DObsmap, polygons, cv::Scalar(127));
        }
    }
}

void RTFMTplanner::DrawDObsmapDBUG(cv::Mat &DBUGmap)
{
    for (const auto &pair : DObstacles)
    {
        const RTFMT_DynamicObstacle &obs = pair.second;
        float sinangle = sinf32(obs.angle);
        float cosangle = cosf32(obs.angle);
        if (obs.DObsType == 0)
        {
            for (const RTpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                cv::circle(DBUGmap, cv::Point(imgx, imgy), obs.radius, cv::Scalar(127, 127, 127), cv::FILLED);
            }
        }
        else if (obs.DObsType == 1)
        {
            std::vector<std::vector<cv::Point>> polygons(1);
            polygons[0].reserve(obs.OutLine.size() + 5);
            for (const RTpoint &obspoint : obs.OutLine)
            {
                int imgx = cosangle * obspoint.x - sinangle * obspoint.y + obs.x;
                int imgy = sinangle * obspoint.x + cosangle * obspoint.y + obs.y;
                polygons[0].push_back({imgx, imgy});
            }
            cv::fillPoly(DBUGmap, polygons, cv::Scalar(127, 127, 127));
        }
    }
}

bool RTFMTplanner::CrashDetection(const RTpoint &ps, const RTpoint &pe)
{
    if (Obsmap.at<uint8_t>(ps.y, ps.x) || Obsmap.at<uint8_t>(pe.y, pe.x))
        return true;
    if (DObsmap.at<uint8_t>(ps.y, ps.x) || DObsmap.at<uint8_t>(pe.y, pe.x))
        return true;
    float len = ps % pe;
    int32_t MaximumDiscrete = len / 5 + 2; //$$$
    for (float denominator = 2; denominator <= MaximumDiscrete; denominator *= 2)
    {
        for (int32_t i = 1; i < denominator; i += 2)
        {
            float t = i / denominator;
            int32_t x = ps.x * t + (1 - t) * pe.x;
            int32_t y = ps.y * t + (1 - t) * pe.y;
            if (Obsmap.at<uint8_t>(y, x) || DObsmap.at<uint8_t>(y, x))
                return true;
        }
    }
    return false;
}

bool RTFMTplanner::CrashDetectionNDObs(const RTpoint &ps, const RTpoint &pe)
{
    if (Obsmap.at<uint8_t>(ps.y, ps.x) || Obsmap.at<uint8_t>(pe.y, pe.x))
        return true;

    float len = ps % pe;
    int32_t MaximumDiscrete = len / 5 + 2; //$$$
    for (float denominator = 2; denominator <= MaximumDiscrete; denominator *= 2)
    {
        for (int32_t i = 1; i < denominator; i += 2)
        {
            float t = i / denominator;
            int32_t x = ps.x * t + (1 - t) * pe.x;
            int32_t y = ps.y * t + (1 - t) * pe.y;
            if (Obsmap.at<uint8_t>(y, x))
                return true;
        }
    }
    return false;
}

void RTFMTplanner::GetPath(std::list<RTpoint> &path)
{
    // c_base = -1;
    path.clear();
    if (goal.x <= 0)
        return;
    Neighbor_search search_near(NeighborSearchTree, {goal.x, goal.y}, 25);
    RTpoint x_min = {-1, -1};
    RTFMT_Cost c_min = RTFMT_Cost{1, FLT_MAX};
    for (const Point_with_distance pwd : search_near)
    {
        RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
        RTFMT_Cost c_new = Node_Map[x_near].cost +
                           RTFMT_Cost{CrashDetection(goal, x_near), goal % x_near};
        if (c_new < c_min)
        {
            c_min = c_new;
            x_min = x_near;
        }
    }
    if (c_min < RTFMT_Cost{0, 20000.0f})
    {
        path.push_front(goal);
        RTpoint x_i = x_min;
        while (x_i.x >= 0)
        {
            path.push_front(x_i);
            x_i = Node_Map[x_i].par;
            if (path.size() > 2000)
            {
                path.clear();
                return;
            }
        }
        // c_base = c_min.cost;
    }
    else
    {
        RTpoint x_min = {-1, -1};
        RTFMT_Cost c_min = RTFMT_Cost{1, 3.3e38f};
        std::queue<std::pair<int, RTpoint>> Qpk;
        Qpk.push({0, root});
        while (Qpk.size())
        {
            std::pair<int, RTpoint> pk_now = Qpk.front();
            Qpk.pop();
            if (pk_now.first < 5 && Node_Map[pk_now.second].subs.size())
            {
                for (const RTpoint &subPoint : Node_Map[pk_now.second].subs)
                {
                    if (!Node_Map[subPoint].cost.collision)
                        Qpk.push({pk_now.first + 1, subPoint});
                }
            }
            RTFMT_Cost c_now = {Node_Map[pk_now.second].cost.collision, pk_now.second % goal};
            if (c_now < c_min)
            {
                c_min = c_now;
                x_min = pk_now.second;
            }
        }
        // path.push_front(x_min);
        RTpoint x_i = x_min;
        while (x_i.x >= 0)
        {
            path.push_front(x_i);
            x_i = Node_Map[x_i].par;
            if (path.size() > 2000)
            {
                path.clear();
                return; //{1, 2.4e38};
            }
        }

        // RTpoint curr_point = root;
        // path.push_back(curr_point);
        // while (1)
        // {
        //     RTpoint x_min = {-1, -1};
        //     RTFMT_Cost c_min = RTFMT_Cost{1, 3.3e38f};
        //     for (const RTpoint &subPoint : Node_Map[curr_point].subs)
        //     {
        //         float H;
        //         if (visited_set.count(subPoint))
        //             H = 3.4e38f;
        //         else
        //             H = subPoint % goal;

        //         RTFMT_Cost c_now = Node_Map[subPoint].cost + RTFMT_Cost{0, H};
        //         if (c_now < c_min)
        //         {
        //             c_min = c_now;
        //             x_min = subPoint;
        //         }
        //     }
        //     if (c_min > RTFMT_Cost{0, 25000.0f}) //@@@
        //     {
        //         visited_set.insert(x_min);
        //         break;
        //     }
        //     if (path.size() >= 10)
        //         break;
        //     path.push_back(x_min);
        //     curr_point = x_min;
        // }
    }
}
#define SampleN 2000
void RTFMTplanner::Start(void)
{
    root = agent;
    // Node_Map[root] = {{0, 0}, {-1, -1}, {}};
    // NeighborSearchTree.insert({root.x, root.y});
    V_open.clear();
    V_open.insert(root);
    V_opennew.clear();
    V_closed.clear();
    z = root;

    std::queue<RTpoint> emptyQueue;
    std::swap(Qr, emptyQueue);
    std::swap(Qo, emptyQueue);

    for (size_t i = 0; i < SampleN;)
    {
        float rand_x, rand_y;
        rand_x = rand() / (float(RAND_MAX)) * shapeX;
        rand_y = rand() / (float(RAND_MAX)) * shapeY;
        if (Obsmap.at<uint8_t>(rand_y, rand_x) == 0)
        {
            RTpoint randpoint = {rand_x, rand_y};
            Node_Map[randpoint] = {{1, 3.4e38}, {-1, -1}, {}};
            NeighborSearchTree.insert({rand_x, rand_y});
            V_unv.insert(randpoint);
            i++;
        }
    }
}
void RTFMTplanner::ExpandFMT(void)
{
    if (V_unv.size() && X_near.size() == 0 && z.x >= 0)
    {
        Neighbor_search search_Znear(NeighborSearchTree, {z.x, z.y}, 25); //@@!!!
        for (const Point_with_distance pwd : search_Znear)
        {
            RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
            if (V_unv.count(x_near))
                X_near.push_back(x_near);
        }
    }
    // std::cout << "!" << 1 << std::endl;
    if (X_near.size())
    {
        RTpoint x = X_near.front();
        X_near.pop_front();
        Neighbor_search search_Ynear(NeighborSearchTree, {x.x, x.y}, 25); //@@!!!
        float cost_min = 3.4e38f;
        RTpoint y_min = {-1, -1};
        for (const Point_with_distance pwd : search_Ynear)
        {
            RTpoint y_near = {(float)pwd.first.x(), (float)pwd.first.y()};
            if (V_open.count(y_near))
            {
                float cost_now = Node_Map[y_near].cost.cost + y_near % x;
                if (cost_min > cost_now)
                {
                    cost_min = cost_now;
                    y_min = y_near;
                }
            }
        }
        if (y_min.x >= 0 && !CrashDetection(x, y_min) && Node_Map[y_min].cost.collision == 0)
        {
            V_opennew.insert(x);
            V_unv.erase(x);
            Node_Map[x].cost = Node_Map[y_min].cost + RTFMT_Cost{0, cost_min};
            Node_Map[x].par = y_min;
            Node_Map[y_min].subs.insert(x);
        }
    }
    // std::cout << "!" << 2 << std::endl;

    if (X_near.size() == 0 && z.x >= 0)
    {
        V_open.insert(V_opennew.begin(), V_opennew.end());
        // std::cout << "  * " << V_open.size() << std::endl;
        // std::cout << "  *1 " << V_opennew.size() << std::endl;
        V_opennew.clear();

        V_open.erase(z);
        V_closed.insert(z);

        Neighbor_search search_Znear(NeighborSearchTree, {z.x, z.y}, 25); //@@!!!
        for (const Point_with_distance pwd : search_Znear)
        {
            RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
            if (V_unv.count(x_near) && !CrashDetectionNDObs(z, x_near))
            {
                V_toopen.insert(z);
                break;
            }
        }
        // std::cout << "  *" << 1 << std::endl;
        RTpoint z_min = {-1, -1};
        RTFMT_Cost zCost_min = {1, 3.4e38};
        for (const RTpoint &z_open : V_open)
        {
            if (Node_Map[z_open].cost < zCost_min)
            {
                zCost_min = Node_Map[z_open].cost;
                z_min = z_open;
            }
        }
        // std::cout << "  *" << V_open.size() << std::endl;
        z = z_min;
    }

    if (z.x < 0)
    {
        // std::cout << "V_open:   " << V_open.size() << std::endl;
        // std::cout << "V_closed: " << V_closed.size() << std::endl;
        // std::cout << "V_toopen: " << V_toopen.size() << std::endl;
        V_open.insert(V_toopen.begin(), V_toopen.end());
        std::set<RTpoint> result;
        std::set_difference(V_closed.begin(), V_closed.end(), V_toopen.begin(), V_toopen.end(),
                            std::inserter(result, result.begin()));
        V_closed.swap(result);
        V_toopen.clear();
        RTpoint z_min = {-1, -1};
        RTFMT_Cost zCost_min = {1, 3.4e38};
        for (const RTpoint &z_open : V_open)
        {
            if (Node_Map[z_open].cost < zCost_min)
            {
                zCost_min = Node_Map[z_open].cost;
                z_min = z_open;
            }
        }
        z = z_min;
    }
    // std::cout << "V_unv:    " << V_unv.size() << std::endl;
    // std::cout << "V_open:   " << V_open.size() << std::endl;
    // std::cout << "V_closed: " << V_closed.size() << std::endl;
    // std::cout << "V_toopen: " << V_toopen.size() << std::endl;
}

void RTFMTplanner::RewireFromObstacles(void)
{
    if (Qo.size() == 0)
    {
        for (const RTpoint &x_b : N_b)
            Qo.push(x_b);
        N_b.clear();
    }
    else
    {
        RTpoint x_b = Qo.front();
        Qo.pop();

        // if (DObsmap.at<uint8_t>(x_b.y, x_b.x) == 0)
        if (DObsmap.at<uint8_t>(x_b.y, x_b.x) == 0)
        {
            float cost_min = 3.4e38f;
            RTpoint y_min = {-1, -1};
            Neighbor_search search_Ynear(NeighborSearchTree, {x_b.x, x_b.y}, 25); //@@!!!
            for (const Point_with_distance pwd : search_Ynear)
            {
                RTpoint y_near = {(float)pwd.first.x(), (float)pwd.first.y()};
                if (y_near != x_b && (V_open.count(y_near) || V_closed.count(y_near)))
                {
                    float cost_now = Node_Map[y_near].cost.cost + y_near % x_b;
                    if (cost_now < cost_min && Node_Map[y_near].cost.collision == 0)
                    {
                        cost_min = cost_now;
                        y_min = y_near;
                    }
                }
            } 
            if (!CrashDetection(y_min, x_b) && cost_min < Node_Map[x_b].cost.cost)//&& cost_min < 3.3e38f
            {

                if (Node_Map[x_b].par.x >= 0)
                    Node_Map[Node_Map[x_b].par].subs.erase(x_b);
                Node_Map[y_min].subs.insert(x_b);
                Node_Map[x_b].par = y_min;
                Node_Map[x_b].cost = Node_Map[y_min].cost + RTFMT_Cost{0, x_b % y_min};

                RTFMT_Cost &pcost = Node_Map[x_b].cost;
                for (const RTpoint &psub : Node_Map[x_b].subs)
                {
                    Node_Map[psub].cost = pcost + RTFMT_Cost{CrashDetection(x_b, psub), x_b % psub};
                    Qo.push(psub);
                }

                // std::queue<RTpoint> uQ;
                // uQ.push(x_b);
                // while (uQ.size())
                // {
                //     RTpoint p = uQ.front();
                //     RTFMT_Cost &pcost = Node_Map[p].cost;
                //     uQ.pop();
                //     for (const RTpoint &psub : Node_Map[p].subs)
                //     {
                //         Node_Map[psub].cost = pcost + RTFMT_Cost{CrashDetection(p, psub), p % psub};
                //         uQ.push(psub);
                //     }
                // }
            }
        }
    }
}
// void RTFMTplanner::RewireFromRoot(void)
// {
//     if (Qr.size() == 0)
//     {
//         Qr_set.clear();
//         Neighbor_search search_near(NeighborSearchTree, {root.x, root.y}, 50); //@@!!!
//         // Qr_set.insert(root);
//         for (const Point_with_distance pwd : search_near)
//         {
//             RTpoint pnear = {(float)pwd.first.x(), (float)pwd.first.y()};
//             if (V_open.count(pnear) || V_closed.count(pnear))
//             {
//                 if (pnear != root)
//                     Qr.push(pnear);
//                 Qr_set.insert(pnear);
//             }
//         }
//     }
//     else
//     {
//         RTpoint x_b = Qr.front();
//         Qr.pop();

//         if (DObsmap.at<uint8_t>(x_b.y, x_b.x) == 0)
//         {
//             float cost_min = 3.4e38f;
//             RTpoint y_min = {-1, -1};
//             Neighbor_search search_Ynear(NeighborSearchTree, {x_b.x, x_b.y}, 50); //@@!!!
//             for (const Point_with_distance pwd : search_Ynear)
//             {
//                 RTpoint y_near = {(float)pwd.first.x(), (float)pwd.first.y()};
//                 if (y_near != x_b && (V_open.count(y_near) || V_closed.count(y_near)))
//                 {
//                     float cost_now = Node_Map[y_near].cost.cost + y_near % x_b;
//                     if (cost_now < cost_min && Node_Map[y_near].cost.collision == 0)
//                     {
//                         cost_min = cost_now;
//                         y_min = y_near;
//                     }
//                 }
//             }
//             if (!CrashDetection(y_min, x_b) && cost_min < 3.3e38f)
//             {
//                 if (Node_Map[x_b].par.x >= 0)
//                     Node_Map[Node_Map[x_b].par].subs.erase(x_b);
//                 Node_Map[y_min].subs.insert(x_b);
//                 Node_Map[x_b].par = y_min;
//                 Node_Map[x_b].cost = Node_Map[y_min].cost + RTFMT_Cost{0, x_b % y_min};

//                 Neighbor_search search_near(NeighborSearchTree, {x_b.x, x_b.y}, 50); //@@!!!
//                 for (const Point_with_distance pwd : search_near)
//                 {
//                     RTpoint pnear = {(float)pwd.first.x(), (float)pwd.first.y()};
//                     if (pnear != x_b && Qr_set.count(pnear) == 0 && (V_open.count(pnear) || V_closed.count(pnear)))
//                     {
//                         Qr.push(pnear);
//                         Qr_set.insert(pnear);
//                     }
//                 }
//                 // RTFMT_Cost &pcost = Node_Map[x_b].cost;
//                 // for (const RTpoint &psub : Node_Map[x_b].subs)
//                 // {
//                 //     Node_Map[psub].cost = pcost + RTFMT_Cost{CrashDetection(x_b, psub), x_b % psub};
//                 //     Qr.push(psub);
//                 // }
//             }
//         }
//     }
// }
void RTFMTplanner::RewireFromRoot(void)
{
    if (Qr.size() == 0)
    {
        Qr_set.clear();
        Qr.push(root);
        Qr_set.insert(root);
    }
    else
    {
        RTpoint x_b = Qr.front();
        Qr.pop();
        Neighbor_search search_Ynear(NeighborSearchTree, {x_b.x, x_b.y}, 20); //@@!!!
        for (const Point_with_distance pwd : search_Ynear)
        {
            RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
            if (!(x_b != x_near))
                continue;
            if (V_open.count(x_near) || V_closed.count(x_near))
            {
                RTFMT_Cost c_old = Node_Map[x_near].cost;
                RTFMT_Cost c_new = Node_Map[x_b].cost +
                                   RTFMT_Cost{CrashDetection(x_b, x_near), x_b % x_near};
                if (c_new < c_old)
                {
                    Node_Map[Node_Map[x_near].par].subs.erase(x_near);
                    Node_Map[x_b].subs.insert(x_near);
                    Node_Map[x_near].par = x_b;
                    Node_Map[x_near].cost = c_new;
                }
                if (Qr_set.count(x_near) == 0)
                {
                    Qr.push(x_near);
                    Qr_set.insert(x_near);
                }
            }
        }
    }
}
void RTFMTplanner::MainTask(void)
{
    long long t0 = utime_ns();
    // while (utime_ns() - t0 < 100e6)
    {
        /********************************更新环境信息********************************/
        DrawDObsmap();
        UpdateTree();

        /********************************扩展树重连接********************************/
        for (size_t i = 0; i < 500; i++)
        {
            ExpandFMT();
            RewireFromObstacles();
            RewireFromRoot();
        }

        /**********************************生成路径**********************************/
    }
    // GetPath(OutPath);
    // std::cout << "V_unv:    " << V_unv.size() << std::endl;
    // std::cout << "V_open:   " << V_open.size() << std::endl;
    // std::cout << "V_closed: " << V_closed.size() << std::endl;
    // std::cout << "V_toopen: " << V_toopen.size() << std::endl;
}
