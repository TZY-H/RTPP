#include "CDRT.h"

// vector<pair<int32_t, size_t>>的哈希计算器
size_t hash_vec64(const std::vector<std::pair<int32_t, size_t>> &vec)
{
    size_t hashValue = 0;
    for (auto &elem : vec)
    {
        // 将每个元素的哈希值与累积的哈希值进行组合
        size_t elemHash = std::hash<int32_t>{}(elem.first) ^ std::hash<size_t>{}(elem.second);
        // 使用一种更均匀的组合方式，比如加法
        hashValue += elemHash;
        // 使用一种更均匀的混合哈希值的方式，比如乘法
        hashValue ^= elemHash + 0x9e3779b9 + (hashValue << 6) + (hashValue >> 2);
    }
    return hashValue;
}

// 计算点p到点o连线沿x轴正方向顺时针的夹角（单位：弧度）
double calculateAngle(BIpoint o, BIpoint p)
{
    // 计算向量op的方向角度（弧度）
    double angle = atan2(p.y - o.y, p.x - o.x);

    // 将弧度转换为角度（0~360度）
    angle = fmod(angle * 180.0 / M_PI + 360.0, 360.0);

    // 调整角度，使其沿x轴正方向顺时针
    angle = 360.0 - angle;

    return angle;
}

// 计算叉乘
double crossProduct(const BIpoint &O, const BIpoint &A, const BIpoint &B)
{
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

// // 判断点 p 是在线段 (s, e) 的左侧还是右侧
// int judgeSide(BIpoint s, BIpoint e, BIpoint p) {
//     BIpoint ps = {p.x - s.x, p.y - s.y}; // 向量 ps
//     BIpoint pe = {e.x - s.x, e.y - s.y}; // 向量 pe

//     double cross = crossProduct(ps, pe); // 计算叉积

//     if (cross > 0) {
//         return 1; // 在线段左侧
//     } else if (cross < 0) {
//         return -1; // 在线段右侧
//     } else {
//         return 0; // 在线段上
//     }
// }

// 松弛的：检查两条线段是否相交，无视端点，线段长度不为0
bool doIntersect(const BIline &l1, const BIline &l2)
{
    double cp1 = crossProduct(l1.S, l1.E, l2.S);
    double cp2 = crossProduct(l1.S, l1.E, l2.E);
    double cp3 = crossProduct(l2.S, l2.E, l1.S);
    double cp4 = crossProduct(l2.S, l2.E, l1.E);

    // 如果两个线段位于同一条直线上
    if (cp1 == 0 && cp2 == 0 && cp3 == 0 && cp4 == 0)
    {
        // 检查线段端点是否在对方线段上
        if (std::max(l1.S.x, l1.E.x) <= std::min(l2.S.x, l2.E.x) ||
            std::max(l1.S.y, l1.E.y) <= std::min(l2.S.y, l2.E.y) ||
            std::max(l2.S.x, l2.E.x) <= std::min(l1.S.x, l1.E.x) ||
            std::max(l2.S.y, l2.E.y) <= std::min(l1.S.y, l1.E.y))
            return false;
        else
            return true;
    }

    // 如果两个线段在端点处相交
    // if ((cp1 == 0 && cp2 == 0) || (cp3 == 0 && cp4 == 0))
    //     return true;

    // 如果两个线段一条线段的端点在另一条线段的两侧
    if ((cp1 * cp2 < 0) && (cp3 * cp4 < 0))
        return true;

    return false;
}

// 严谨的：检查两条线段是否相交，检查端点，长度可为0
bool doIntersect_rigorous(const BIline &l1, const BIline &l2)
{
    if (l1.S == l1.E)
    {
        if (fabs((l2.S % l2.E) - (l2.S % l1.S) - (l1.S % l2.E)) < 1e-32)
            return true;
        else
            return false;
    }
    else if (l2.S == l2.E)
    {
        if (fabs((l1.S % l1.E) - (l1.S % l2.S) - (l2.S % l1.E)) < 1e-32)
            return true;
        else
            return false;
    }
    double cp1 = crossProduct(l1.S, l1.E, l2.S);
    double cp2 = crossProduct(l1.S, l1.E, l2.E);
    double cp3 = crossProduct(l2.S, l2.E, l1.S);
    double cp4 = crossProduct(l2.S, l2.E, l1.E);

    // 如果两个线段位于同一条直线上
    if (cp1 == 0 && cp2 == 0 && cp3 == 0 && cp4 == 0)
    {
        // 检查线段端点是否在对方线段上
        if (std::max(l1.S.x, l1.E.x) < std::min(l2.S.x, l2.E.x) ||
            std::max(l1.S.y, l1.E.y) < std::min(l2.S.y, l2.E.y) ||
            std::max(l2.S.x, l2.E.x) < std::min(l1.S.x, l1.E.x) ||
            std::max(l2.S.y, l2.E.y) < std::min(l1.S.y, l1.E.y))
            return false;
        else
            return true;
    }

    // 如果两个线段一条线段的端点在另一条线段的两侧
    if ((cp1 * cp2 <= 0) && (cp3 * cp4 <= 0))
        return true;

    return false;
}

