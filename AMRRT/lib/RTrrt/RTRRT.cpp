#include "RTRRT.h"

long long utime_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<long long>(1000000000UL) * static_cast<long long>(ts.tv_sec) +
           static_cast<long long>(ts.tv_nsec);
}

void RTRRTplanner::SetAgent(RTpoint &point)
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

void RTRRTplanner::SetGoal(RTpoint &point)
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

void RTRRTplanner::InitializeInvironment(const char *IMGmapc, int th)
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

void RTRRTplanner::AddDynamicObstacle(int num, RTrrt_DynamicObstacle &obs)
{
    DObstacles[num] = obs;
}

void RTRRTplanner::DelDynamicObstacle(int num)
{
    DObstacles.erase(num);
}

void RTRRTplanner::UpdateTree(void)
{
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
    std::queue<RTpoint> Qup;
    Qup.push(root);
    while (Qup.size())
    {
        RTpoint Pnow = Qup.front();
        Qup.pop();
        RTrrt_NodeAttribute &Pnow_Node = Node_Map[Pnow];
        for (const RTpoint &Psub : Pnow_Node.subs)
        {
            RTrrt_NodeAttribute &Psub_Node = Node_Map[Psub];
            Psub_Node.cost = Pnow_Node.cost +
                             RTrrt_Cost{CrashDetection(Pnow, Psub), Pnow % Psub};
            Qup.push(Psub);
        }
    }
}

void RTRRTplanner::DrawDObsmap(void)
{
    DObsmap = cv::Mat::zeros(shapeY, shapeX, CV_8U);

    for (const auto &pair : DObstacles)
    {
        const RTrrt_DynamicObstacle &obs = pair.second;
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

void RTRRTplanner::DrawDObsmapDBUG(cv::Mat &DBUGmap)
{
    for (const auto &pair : DObstacles)
    {
        const RTrrt_DynamicObstacle &obs = pair.second;
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

bool RTRRTplanner::CrashDetection(const RTpoint &ps, const RTpoint &pe)
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

void RTRRTplanner::ExpandRewiring(long RunTime)
{
    long long t0 = utime_ns();
    RunTime *= 1000000;
    while (utime_ns() - t0 < RunTime)
    {

        /**********************************随机采样**********************************/
        float probabilit0 = rand() / (float(RAND_MAX));
        float rand_x, rand_y;
        if (probabilit0 < 0.1f || c_base < 0) // || OutPath.size() == 0 || OutPath.back() != goal)
        {
            rand_x = rand() / (float(RAND_MAX)) * shapeX;
            rand_y = rand() / (float(RAND_MAX)) * shapeY;
        }
        else
        {
            float c_min = goal % root;
            float a = c_base / 2.0f;
            float b = sqrt(c_base * c_base - c_min * c_min) / 2.0f;
            float angle = atan2(goal.y - root.y, goal.x - root.x);
            while (1)
            {
                float random_theta = rand() / (float(RAND_MAX)) * 2 * 3.141592954f;
                float random_r = sqrtf32(rand() / (float(RAND_MAX)));
                float x = a * cosf32(random_theta) * random_r;
                float y = b * sinf32(random_theta) * random_r;

                rand_x = cosf32(angle) * x - sinf32(angle) * y;
                rand_y = sinf32(angle) * x + cosf32(angle) * y;

                rand_x = rand_x + (goal.x + root.x) / 2;
                rand_y = rand_y + (goal.y + root.y) / 2;
                if (rand_x >= 0 && rand_x < shapeX && rand_y >= 0 && rand_y < shapeY)
                    break;
            }
        }
        {
            RTpoint randpoint = {rand_x, rand_y};
            Neighbor_search search_Closest(NeighborSearchTree, {rand_x, rand_y}, 50); //@@@
            RTpoint closestpoint = {(float)search_Closest.begin()->first.x(),
                                    (float)search_Closest.begin()->first.y()};
            if (!CrashDetection(randpoint, closestpoint))
            {
                if (sqrt(search_Closest.begin()->second) > 10)
                {
                    RTpoint x_min = closestpoint;
                    RTrrt_Cost c_min = Node_Map[closestpoint].cost + RTrrt_Cost{0, randpoint % closestpoint};
                    for (const Point_with_distance pwd : search_Closest)
                    {
                        RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
                        RTrrt_Cost c_new = Node_Map[x_near].cost +
                                           RTrrt_Cost{CrashDetection(randpoint, x_near), randpoint % x_near};
                        if (c_new < c_min)
                        {
                            c_min = c_new;
                            x_min = x_near;
                        }
                    }

                    Node_Map[randpoint].par = x_min;
                    Node_Map[randpoint].cost = c_min;
                    Node_Map[x_min].subs.insert(randpoint);

                    Qr.push(randpoint);
                    NeighborSearchTree.insert({randpoint.x, randpoint.y});
                }
                else
                {
                    Qr.push(closestpoint);
                }
            }
        }
        /***********************************重连接***********************************/
        while (Qr.size() && (utime_ns() - t0 < RunTime))
        {
            RTpoint x_r = Qr.front();
            Qr.pop();
            Neighbor_search search_near(NeighborSearchTree, {x_r.x, x_r.y}, 50); //@@@
            for (const Point_with_distance pwd : search_near)
            {
                RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
                RTrrt_Cost c_old = Node_Map[x_near].cost;
                RTrrt_Cost c_new = Node_Map[x_r].cost +
                                   RTrrt_Cost{CrashDetection(x_r, x_near), x_r % x_near};
                if (c_new < c_old)
                {
                    Node_Map[Node_Map[x_near].par].subs.erase(x_near);
                    Node_Map[x_r].subs.insert(x_near);
                    Node_Map[x_near].par = x_r;
                    Node_Map[x_near].cost = c_new;
                    Qr.push(x_near);
                }
            }
        }

        /********************************重连接根节点***********************************/
        if (Qs.size() == 0)
        {
            Qs_set.clear();
            Qs.push(root);
            Qs_set.insert(root);
        }
        while (Qs.size() && (utime_ns() - t0 < RunTime))
        {
            RTpoint x_s = Qs.front();
            Qs.pop();
            Neighbor_search search_near(NeighborSearchTree, {x_s.x, x_s.y}, 50); //@@@
            for (const Point_with_distance pwd : search_near)
            {
                RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
                RTrrt_Cost c_old = Node_Map[x_near].cost;
                RTrrt_Cost c_new = Node_Map[x_s].cost +
                                   RTrrt_Cost{CrashDetection(x_s, x_near), x_s % x_near};
                if (c_new < c_old)
                {
                    Node_Map[Node_Map[x_near].par].subs.erase(x_near);
                    Node_Map[x_s].subs.insert(x_near);
                    Node_Map[x_near].par = x_s;
                    Node_Map[x_near].cost = c_new;
                }
                if (Qs_set.count(x_near) == 0)
                {
                    Qs.push(x_near);
                    Qs_set.insert(x_near);
                }
            }
        }
    }
}
void RTRRTplanner::GetPath(std::list<RTpoint> &path)
{
    c_base = -1;
    path.clear();
    if (goal.x <= 0)
        return;
    Neighbor_search search_near(NeighborSearchTree, {goal.x, goal.y}, 50);
    RTpoint x_min = {-1, -1};
    RTrrt_Cost c_min = RTrrt_Cost{1, FLT_MAX};
    for (const Point_with_distance pwd : search_near)
    {
        RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
        RTrrt_Cost c_new = Node_Map[x_near].cost +
                           RTrrt_Cost{CrashDetection(goal, x_near), goal % x_near};
        if (c_new < c_min)
        {
            c_min = c_new;
            x_min = x_near;
        }
    }
    if (c_min < RTrrt_Cost{0, 20000.0f})
    {
        path.push_front(goal);
        c_base = goal % x_min;
        RTpoint x_i = x_min;
        while (x_i.x >= 0)
        {
            path.push_front(x_i);
            x_i = Node_Map[x_i].par;
            c_base = c_base + (x_i % Node_Map[x_i].par);
        }
        // c_base = c_min.cost;
    }
    else
    {
        RTpoint curr_point = root;
        path.push_back(curr_point);
        // while (Node_Map[curr_point].subs.size() != 0)
        while (1)
        {
            // std::set<RTpoint>::iterator it = Node_Map[curr_point].subs.begin();
            // RTpoint tempPoint = *it;
            // float cost_ = ;
            RTpoint x_min = {-1, -1};
            RTrrt_Cost c_min = RTrrt_Cost{1, 3.3e38f};
            for (const RTpoint &subPoint : Node_Map[curr_point].subs)
            {
                float H;
                if (visited_set.count(subPoint))
                    H = 3.4e38f;
                else
                    H = subPoint % goal;

                RTrrt_Cost c_now = Node_Map[subPoint].cost + RTrrt_Cost{0, H};
                if (c_now < c_min)
                {
                    c_min = c_now;
                    x_min = subPoint;
                }
            }
            if (c_min > RTrrt_Cost{0, 25000.0f}) //@@@
            {
                visited_set.insert(x_min);
                break;
            }
            if (path.size() >= 10)
            {
                break;
            }
            path.push_back(x_min);
            curr_point = x_min;
        }
    }
}

void RTRRTplanner::Start(void)
{
}
void RTRRTplanner::MainTask(void)
{
    /********************************更新环境信息********************************/
    DrawDObsmap();
    UpdateTree();

    /********************************扩展树重连接********************************/
    ExpandRewiring(100);

    /**********************************生成路径**********************************/
    GetPath(OutPath);
}
