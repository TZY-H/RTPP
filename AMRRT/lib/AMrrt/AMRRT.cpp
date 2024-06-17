#include "AMRRT.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

long long utime_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<long long>(1000000000UL) * static_cast<long long>(ts.tv_sec) +
           static_cast<long long>(ts.tv_nsec);
}

void AMRRTplanner::SetAgent(RTpoint &point)
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

void AMRRTplanner::SetGoal(RTpoint &point)
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
void AMRRTplanner::InitializeDiffusion(const char *DiffFile, const char *IndexFile, float ratios)
{
    diffusion_ratios = ratios;
    json DiffjsonData, IndexjsonData;
    // 打开 JSON 文件
    std::ifstream DiffjsonFile(DiffFile);
    if (!DiffjsonFile.is_open())
    {
        std::cerr << "无法打开 JSON 文件." << std::endl;
        return;
    }
    DiffjsonFile >> DiffjsonData;
    DiffjsonFile.close();
    std::ifstream IndexjsonFile(IndexFile);
    if (!IndexjsonFile.is_open())
    {
        std::cerr << "无法打开 JSON 文件." << std::endl;
        return;
    }
    IndexjsonFile >> IndexjsonData;
    IndexjsonFile.close();

    diffusion_matrix.resize(DiffjsonData.size());
    for (int i = 0; i < diffusion_matrix.size(); i++)
    {
        json &element = DiffjsonData[i];
        std::vector<float> &row = diffusion_matrix[i];
        for (const auto &data : element)
            row.push_back(data);
    }
    for (int i = 0; i < IndexjsonData.size(); i++)
    {
        // DIFFpoint diffP = {IndexjsonData[i][0],IndexjsonData[i][1]};
        index_to_pos[{IndexjsonData[i][0], IndexjsonData[i][1]}] = i;
    }
}
float AMRRTplanner::DiffusivityMetric(RTpoint &x_s, RTpoint &x_e)
{
    int i_s, j_s;
    i_s = (int)(x_s.x * diffusion_ratios);
    j_s = (int)(x_s.y * diffusion_ratios);
    if (index_to_pos.count({i_s, j_s}) == 0)
        return 3.4e38;
    int i_e, j_e;
    i_e = (int)(x_e.x * diffusion_ratios);
    j_e = (int)(x_e.y * diffusion_ratios);
    if (index_to_pos.count({i_e, j_e}) == 0)
        return 3.4e38;
    std::vector<float> &diffusion_pos_s = diffusion_matrix[index_to_pos[{i_s, j_s}]];
    std::vector<float> &diffusion_pos_e = diffusion_matrix[index_to_pos[{i_e, j_e}]];
    float len = 0;
    for (size_t i = 0; i < diffusion_pos_s.size(); i++)
    {
        float dlen = diffusion_pos_s[i] - diffusion_pos_e[i];
        len = len + dlen * dlen;
    }
    len = sqrtf32(len) / diffusion_ratios;
    return len;
}

void AMRRTplanner::InitializeInvironment(const char *IMGmapc, int th)
{
    // 读取输入的图像
    cv::Mat inputImage = cv::imread(IMGmapc, cv::IMREAD_GRAYSCALE);

    if (inputImage.empty())
    {
        std::cerr << "Failed to load input image." << std::endl;
        return;
    }
    // cv::circle(inputImage, cv::Point(343, 434), 6, cv::Scalar(0, 0, 0), cv::FILLED);

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

void AMRRTplanner::AddDynamicObstacle(int num, RTrrt_DynamicObstacle &obs)
{
    DObstacles[num] = obs;
}

void AMRRTplanner::DelDynamicObstacle(int num)
{
    DObstacles.erase(num);
}
#define KnearN 25
#define RnearR 8

void AMRRTplanner::UpdateTree(void)
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

void AMRRTplanner::DrawDObsmap(void)
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

void AMRRTplanner::DrawDObsmapDBUG(cv::Mat &DBUGmap)
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

bool AMRRTplanner::CrashDetection(const RTpoint &ps, const RTpoint &pe)
{
    if (Obsmap.at<uint8_t>(ps.y, ps.x) || Obsmap.at<uint8_t>(pe.y, pe.x))
        return true;
    if (DObsmap.at<uint8_t>(ps.y, ps.x) || DObsmap.at<uint8_t>(pe.y, pe.x))
        return true;
    float len = ps % pe;
    int32_t MaximumDiscrete = len / 2 + 2; //$$$
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
bool AMRRTplanner::CrashDetectionStart(const RTpoint &ps, const RTpoint &pe, RTpoint &pstart)
{
    pstart = ps;
    if (Obsmap.at<uint8_t>(ps.y, ps.x) || Obsmap.at<uint8_t>(pe.y, pe.x))
        return true;
    if (DObsmap.at<uint8_t>(ps.y, ps.x) || DObsmap.at<uint8_t>(pe.y, pe.x))
        return true;
    float len = ps % pe;
    int32_t MaximumDiscrete = len / 2 + 2; //$$$

    for (int32_t i = 1; i < MaximumDiscrete; i++)
    {
        float t = (float)i / MaximumDiscrete;
        float x = pe.x * t + (1 - t) * ps.x;
        float y = pe.y * t + (1 - t) * ps.y;
        if (Obsmap.at<uint8_t>((int32_t)y, (int32_t)x) || DObsmap.at<uint8_t>((int32_t)y, (int32_t)x))
            return true;
        pstart = {x, y};
    }
    pstart = pe;
    return false;
}

RTrrt_Cost AMRRTplanner::GetPath(std::list<RTpoint> &path)
{
    c_base = -1;
    path.clear();
    if (goal.x <= 0)
        return {1, 3.4e38};
    Neighbor_search search_near(NeighborSearchTree, {goal.x, goal.y}, KnearN);
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
    if (c_min < RTrrt_Cost{1, 0.0f})
    {
        c_base = 0;
        path.push_front(goal);
        RTpoint x_i = x_min;
        while (x_i.x >= 0)
        {
            c_base = c_base + (x_i % path.front());
            path.push_front(x_i);
            x_i = Node_Map[x_i].par;
            if (path.size() > 2000)
            {
                c_base = -1;
                path.clear();
                return {1, 2.4e38};
            }
        }
        return {0, c_base};
    }
    else
        return {1, 3.4e38};
}

#define SteerLenMAX 100.0f
RTpoint AMRRTplanner::Steer(RTpoint x_init, RTpoint x_end)
{
    RTpoint x_start;
    if (!CrashDetectionStart(x_init, x_end, x_start))
    {
        float len = x_init % x_end;
        float p = SteerLenMAX / len;
        if (p < 1.0f)
        {
            float x = (1 - p) * x_init.x + p * x_end.x;
            float y = (1 - p) * x_init.y + p * x_end.y;
            return {x, y};
        }
        return x_end;
    }
    RTpoint x_min = x_start;
    float c_min = DiffusivityMetric(x_start, x_end);
    float r_sam = min(SteerLenMAX, x_init % x_end);
    long long t0 = utime_ns();
    while (utime_ns() - t0 < 5e6) //@@@
    {
        float random_theta = rand() / (float(RAND_MAX)) * 2 * 3.141592954f;
        float random_r = rand() / (float(RAND_MAX));
        random_r = sqrtf32(random_r) * r_sam;
        float x = cosf32(random_theta) * random_r + x_init.x;
        float y = sinf32(random_theta) * random_r + x_init.y;

        RTpoint x_new = {x, y};
        float c_new = DiffusivityMetric(x_new, x_end);
        if (c_new < c_min && !CrashDetection(x_init, x_new))
        {
            x_min = x_new;
            c_min = c_new;
        }
    }
    return x_min;
}

#define alpha 0.9f
#define beta 4.0f
#define k_max 4.0f
void AMRRTplanner::Expand(void)
{
    RTpoint x_rand;

    while (1) // SampleState
    {
        float p = rand() / (float(RAND_MAX));
        if (p > alpha && OutPath.size() == 0 && goal.x >= 0)
            x_rand = goal;
        else if (p < alpha / beta || OutPath.size() == 0)
        {
            float rand_x = rand() / (float(RAND_MAX)) * shapeX;
            float rand_y = rand() / (float(RAND_MAX)) * shapeY;
            if (!(Obsmap.at<uint8_t>(rand_y, rand_x) || DObsmap.at<uint8_t>(rand_y, rand_x)))
            {
                x_rand.x = rand_x;
                x_rand.y = rand_y;
                break;
            }
        }
        else if (goal.x >= 0)
        {
            float rand_x, rand_y;
            float c_min = goal % root;
            float a = c_base / 2.0f;
            float b = sqrt(c_base * c_base - c_min * c_min) / 2.0f;
            float angle = atan2(goal.y - root.y, goal.x - root.x);

            float random_theta = rand() / (float(RAND_MAX)) * 2 * 3.141592954f;
            float random_r = sqrtf32(rand() / (float(RAND_MAX)));
            float x = a * cosf32(random_theta) * random_r;
            float y = b * sinf32(random_theta) * random_r;

            rand_x = cosf32(angle) * x - sinf32(angle) * y;
            rand_y = sinf32(angle) * x + cosf32(angle) * y;

            rand_x = rand_x + (goal.x + root.x) / 2;
            rand_y = rand_y + (goal.y + root.y) / 2;
            if (rand_x >= 0 && rand_x < shapeX && rand_y >= 0 && rand_y < shapeY)
            {
                if (!(Obsmap.at<uint8_t>(rand_y, rand_x) || DObsmap.at<uint8_t>(rand_y, rand_x)))
                {
                    x_rand.x = rand_x;
                    x_rand.y = rand_y;
                    break;
                }
            }
        }
    }
    Neighbor_search search_Closest(NeighborSearchTree, {x_rand.x, x_rand.y}, KnearN); //@@@
    RTpoint x_nrst = {(float)search_Closest.begin()->first.x(),
                      (float)search_Closest.begin()->first.y()};
    RTpoint x_new = Steer(x_nrst, x_rand);

    Neighbor_search search_Near(NeighborSearchTree, {x_new.x, x_new.y}, KnearN);
    RTpoint nearest = {(float)search_Near.begin()->first.x(),
                       (float)search_Near.begin()->first.y()};

    if (nearest % x_new > k_max)//x_nrst % x_rand > SteerLenMAX || 
    {
        RTpoint x_min = x_nrst;
        RTrrt_Cost c_min = Node_Map[x_nrst].cost + RTrrt_Cost{0, x_nrst % x_new};
        for (const Point_with_distance pwd : search_Near)
        {
            RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};

            RTrrt_Cost c_new = Node_Map[x_near].cost + RTrrt_Cost{CrashDetection(x_near, x_new), x_near % x_new};
            if (c_new < c_min && !c_new.collision)
            {
                x_min = x_near;
                c_min = c_new;
            }
        }
        if (!c_min.collision)
        {
            int Node_Map_size = Node_Map.size();
            int NeighborSearchTree_size = NeighborSearchTree.size();
            Node_Map[x_min].subs.insert(x_new);
            Node_Map[x_new] = RTrrt_NodeAttribute{c_min, x_min, {}};
            NeighborSearchTree.insert({x_new.x, x_new.y});
            // if ((Node_Map.size() - Node_Map_size) != (NeighborSearchTree.size() - NeighborSearchTree_size))
            // {
            //     std::cout << "kdtree: " << NeighborSearchTree.size() << ", node: " << Node_Map.size() << std::endl;
            // }
        }
    }
}

void AMRRTplanner::RewireRoot(void)
{
    if (Qroot.size() == 0)
    {
        Qroot.push(root);
        rewired_root.clear();
        rewired_root.insert(root);
    }
    long long t0 = utime_ns();
    while (utime_ns() - t0 < 20e6 && Qroot.size())
    {
        RTpoint x_r = Qroot.front();
        Qroot.pop();
        Neighbor_search search_Near(NeighborSearchTree, {x_r.x, x_r.y}, KnearN);
        for (const Point_with_distance pwd : search_Near)
        {
            RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
            if (!CrashDetection(x_near, x_r))
            {
                RTrrt_Cost c_old = Node_Map[x_near].cost;
                RTrrt_Cost c_new = Node_Map[x_r].cost + RTrrt_Cost{0, x_near % x_r};
                if (c_new < c_old && !c_new.collision)
                {
                    Node_Map[Node_Map[x_near].par].subs.erase(x_near);
                    Node_Map[x_near].par = x_r;
                    Node_Map[x_near].cost = c_new;
                    Node_Map[x_r].subs.insert(x_near);
                }
                if (rewired_root.count(x_near) == 0)
                {
                    rewired_root.insert(x_near);
                    Qroot.push(x_near);
                }
            }
        }
    }
}
typedef std::pair<float, RTpoint> NBpair;
void AMRRTplanner::RewireGoal(void)
{
    if (Qgoal.size() == 0 && Sgoal.size() == 0)
    {
        Sgoal.push_front(root);
        seen_goal.clear();
    }
    long long t0 = utime_ns();
    while (utime_ns() - t0 < 100e6 && (Sgoal.size() || Qgoal.size())) //@@@
    {
        RTpoint x_r;
        if (Sgoal.size())
        {
            x_r = Sgoal.front();
            Sgoal.pop_front();
        }
        else
        {
            x_r = Qgoal.front();
            Qgoal.pop();
        }
        if ((root % x_r) + (goal % x_r) <= c_base)
        {
            std::list<RTpoint> X_next;
            std::priority_queue<NBpair, std::vector<NBpair>, std::greater<NBpair>> nearby_sorted;
            Neighbor_search search_Near(NeighborSearchTree, {x_r.x, x_r.y}, KnearN); //@@@
            for (const Point_with_distance pwd : search_Near)
            {
                RTpoint x_near = {(float)pwd.first.x(), (float)pwd.first.y()};
                nearby_sorted.push({DiffusivityMetric(goal, x_near), x_near});
            }
            while (nearby_sorted.size())
            {
                RTpoint x_near = nearby_sorted.top().second;
                nearby_sorted.pop();
                if (!CrashDetection(x_near, x_r))
                {
                    RTrrt_Cost c_new = Node_Map[x_r].cost + RTrrt_Cost{0, x_near % x_r};
                    if (c_new < Node_Map[x_near].cost && !c_new.collision)
                    {
                        Node_Map[Node_Map[x_near].par].subs.erase(x_near);
                        Node_Map[x_near].par = x_r;
                        Node_Map[x_near].cost = c_new;
                        Node_Map[x_r].subs.insert(x_near);
                    }
                    if (seen_goal.count(x_near) == 0)
                    {
                        seen_goal.insert(x_near);
                        X_next.push_back(x_near);
                    }
                }
            }
            for (const RTpoint x_next : X_next)
            {
                Sgoal.push_front(x_next);
                Qgoal.push(x_next);
            }
        }
        if (Sgoal.size())
        {
            if (DiffusivityMetric(Sgoal.front(), goal) > DiffusivityMetric(x_r, goal))
                Sgoal.clear();
        }
    }
}

void AMRRTplanner::Start(void)
{
}
void AMRRTplanner::MainTask(void)
{
    /********************************更新环境信息********************************/
    DrawDObsmap();
    UpdateTree();

    /********************************扩展树重连接********************************/
    // ExpandRewiring(100);
    long long t0 = utime_ns();
    // std::cout << "kdtree: " << NeighborSearchTree.size() << ", node: " << Node_Map.size() << std::endl;
    while (utime_ns() - t0 < 80e6)
    {
        Expand();
        RewireRoot();
        if (goal.x >= 0)
        {
            auto goal_it = Node_Map.find(goal);
            if (goal_it != Node_Map.end())
            {
                if (!(goal_it->second.cost.collision))
                    RewireGoal();
            }
        }
    }

    /**********************************生成路径**********************************/
    // GetPath(OutPath);
}