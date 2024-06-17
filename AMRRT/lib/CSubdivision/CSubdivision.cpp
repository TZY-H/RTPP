#include "CSubdivision.h"
#include <math.h>
#include <list>
void CSubdivision::FindConcave(void)
{
    uint32_t SPoint_temp = 0;
    uint32_t NPoint_temp = 0;
    uint32_t PointListlen = 0;
    double v1_x, v2_x, v1_y, v2_y;
    double r;
    for (;;)
    {
        PointListlen++;
        for (;;)
        {
            if (NPoint_temp + 2 >= PointMaplen || PointMap[NPoint_temp + 2].x < 0)
            {
                linelist[linelistlen++] = {NPoint_temp, NPoint_temp + 1};
                v1_x = PointMap[NPoint_temp + 1].x - PointMap[NPoint_temp].x;
                v1_y = PointMap[NPoint_temp + 1].y - PointMap[NPoint_temp].y;
                v2_x = PointMap[SPoint_temp].x - PointMap[NPoint_temp + 1].x;
                v2_y = PointMap[SPoint_temp].y - PointMap[NPoint_temp + 1].y;
                r = v1_x * v2_y - v1_y * v2_x;
                if (r > 0)
                    PointList[PointListlen++] = {NPoint_temp, NPoint_temp + 1, SPoint_temp, 1};
                else
                {
                    PointList[PointListlen++] = {NPoint_temp, NPoint_temp + 1, SPoint_temp, -1};
                    ConcaveList[ConcaveListlen++] = {NPoint_temp, NPoint_temp + 1, SPoint_temp, -1};
                }
                NPoint_temp++;
                linelist[linelistlen++] = {NPoint_temp, SPoint_temp};

                v1_x = PointMap[SPoint_temp].x - PointMap[NPoint_temp].x;
                v1_y = PointMap[SPoint_temp].y - PointMap[NPoint_temp].y;
                v2_x = PointMap[SPoint_temp + 1].x - PointMap[SPoint_temp].x;
                v2_y = PointMap[SPoint_temp + 1].y - PointMap[SPoint_temp].y;
                r = v1_x * v2_y - v1_y * v2_x;
                if (r > 0)
                    PointList[SPoint_temp] = {NPoint_temp, SPoint_temp, SPoint_temp + 1, 1};
                else
                {
                    PointList[SPoint_temp] = {NPoint_temp, SPoint_temp, SPoint_temp + 1, -1};
                    ConcaveList[ConcaveListlen++] = {NPoint_temp, SPoint_temp, SPoint_temp + 1, -1};
                }
                PointListlen++;
                break;
            }
            else
            {
                linelist[linelistlen++] = {NPoint_temp, NPoint_temp + 1};
                v1_x = PointMap[NPoint_temp + 1].x - PointMap[NPoint_temp].x;
                v1_y = PointMap[NPoint_temp + 1].y - PointMap[NPoint_temp].y;
                v2_x = PointMap[NPoint_temp + 2].x - PointMap[NPoint_temp + 1].x;
                v2_y = PointMap[NPoint_temp + 2].y - PointMap[NPoint_temp + 1].y;
                r = v1_x * v2_y - v1_y * v2_x;
                if (r > 0)
                    PointList[PointListlen++] = {NPoint_temp, NPoint_temp + 1, NPoint_temp + 2, 1};
                else
                {
                    PointList[PointListlen++] = {NPoint_temp, NPoint_temp + 1, NPoint_temp + 2, -1};
                    ConcaveList[ConcaveListlen++] = {NPoint_temp, NPoint_temp + 1, NPoint_temp + 2, -1};
                }
            }
            NPoint_temp++;
        }
        NPoint_temp++;
        if (NPoint_temp >= PointMaplen)
            break;
        else
        {
            NPoint_temp++;
            SPoint_temp = NPoint_temp;
        }
    }
}

void CSubdivision::ViewablePoint(PList& vp)
{
    int32_t ViewableSign;
    float v1_x, v1_y, v2_x, v2_y, v3_x, v3_y;
    float v1p, v2p, r1, r2, d1, d2;
    vplistlen = 0;
    // if( vp.N ==61)
    // {
    //     cout<<61<<endl;
    // }
    for (uint32_t i = 0; i < PointMaplen; i++)
    {
        ViewableSign = 0;
        if (i != vp.S && i != vp.E && i != vp.N && PointMap[i].x >= 0)
        {
            v1_x = PointMap[vp.N].x - PointMap[vp.S].x;
            v1_y = PointMap[vp.N].y - PointMap[vp.S].y;
            v2_x = PointMap[vp.E].x - PointMap[vp.N].x;
            v2_y = PointMap[vp.E].y - PointMap[vp.N].y;
            v3_x = PointMap[i].x - PointMap[vp.N].x;
            v3_y = PointMap[i].y - PointMap[vp.N].y;
            v1p = v1_x * v3_y - v1_y * v3_x;
            v2p = v2_x * v3_y - v2_y * v3_x;
            if ((vp.key > 0 && v1p > 0 && v2p > 0) || (vp.key < 0 && (v1p > 0 || v2p > 0)))
            {
                ViewableSign = 1;
                for (uint32_t line_num = 0; line_num < linelistlen; line_num++)
                {
                    LList &line = linelist[line_num];
                    if (vp.N == line.S || vp.N == line.E || i == line.S || i == line.E)
                        continue;
                    v1_x = PointMap[line.S].x - PointMap[vp.N].x;
                    v1_y = PointMap[line.S].y - PointMap[vp.N].y;
                    v2_x = PointMap[i].x - PointMap[vp.N].x;
                    v2_y = PointMap[i].y - PointMap[vp.N].y;
                    v3_x = PointMap[line.E].x - PointMap[vp.N].x;
                    v3_y = PointMap[line.E].y - PointMap[vp.N].y;
                    r1 = (v1_x * v2_y - v1_y * v2_x) * (v3_x * v2_y - v3_y * v2_x);

                    v1_x = PointMap[i].x - PointMap[line.S].x;
                    v1_y = PointMap[i].y - PointMap[line.S].y;
                    v2_x = PointMap[line.E].x - PointMap[line.S].x;
                    v2_y = PointMap[line.E].y - PointMap[line.S].y;
                    v3_x = PointMap[vp.N].x - PointMap[line.S].x;
                    v3_y = PointMap[vp.N].y - PointMap[line.S].y;
                    r2 = (v1_x * v2_y - v1_y * v2_x) * (v3_x * v2_y - v3_y * v2_x);
                    if (r1 <= 0 && r2 <= 0)
                    {
                        if (r1 == 0 && r2 == 0)
                        {
                            d1 = (PointMap[i].x - PointMap[line.S].x) *
                                     (PointMap[i].x - PointMap[line.E].x) +
                                 (PointMap[i].y - PointMap[line.S].y) *
                                     (PointMap[i].y - PointMap[line.E].y);
                            d2 = (PointMap[vp.N].x - PointMap[line.S].x) *
                                     (PointMap[vp.N].x - PointMap[line.E].x) +
                                 (PointMap[vp.N].y - PointMap[line.S].y) *
                                     (PointMap[vp.N].y - PointMap[line.E].y);
                            if (d1 < 0 || d2 < 0)
                            {
                                ViewableSign = 0;
                                break;
                            }
                        }
                        else
                        {
                            ViewableSign = 0;
                            break;
                        }
                    }
                }
                for (uint32_t line_num = 0; line_num < cutlinelistlen; line_num++)
                {
                    LList &line = cutlinelist[line_num];
                    if ((vp.N == line.S || vp.N == line.E) || (i == line.S || i == line.E))
                        continue;
                    v1_x = PointMap[line.S].x - PointMap[vp.N].x;
                    v1_y = PointMap[line.S].y - PointMap[vp.N].y;
                    v2_x = PointMap[i].x - PointMap[vp.N].x;
                    v2_y = PointMap[i].y - PointMap[vp.N].y;
                    v3_x = PointMap[line.E].x - PointMap[vp.N].x;
                    v3_y = PointMap[line.E].y - PointMap[vp.N].y;
                    r1 = (v1_x * v2_y - v1_y * v2_x) * (v3_x * v2_y - v3_y * v2_x);

                    v1_x = PointMap[i].x - PointMap[line.S].x;
                    v1_y = PointMap[i].y - PointMap[line.S].y;
                    v2_x = PointMap[line.E].x - PointMap[line.S].x;
                    v2_y = PointMap[line.E].y - PointMap[line.S].y;
                    v3_x = PointMap[vp.N].x - PointMap[line.S].x;
                    v3_y = PointMap[vp.N].y - PointMap[line.S].y;
                    r2 = (v1_x * v2_y - v1_y * v2_x) * (v3_x * v2_y - v3_y * v2_x);
                    if (r1 <= 0 && r2 <= 0)
                    {
                        if (r1 == 0 && r2 == 0)
                        {
                            d1 = (PointMap[i].x - PointMap[line.S].x) *
                                     (PointMap[i].x - PointMap[line.E].x) +
                                 (PointMap[i].y - PointMap[line.S].y) *
                                     (PointMap[i].y - PointMap[line.E].y);
                            d2 = (PointMap[vp.N].x - PointMap[line.S].x) *
                                     (PointMap[vp.N].x - PointMap[line.E].x) +
                                 (PointMap[vp.N].y - PointMap[line.S].y) *
                                     (PointMap[vp.N].y - PointMap[line.E].y);
                            if (d1 < 0 || d2 < 0)
                            {
                                ViewableSign = 0;
                                break;
                            }
                        }
                        else
                        {
                            ViewableSign = 0;
                            break;
                        }
                    }
                }
                if (ViewableSign == 1)
                    vplist[vplistlen++] = i;
            }
        }
    }
}
void CSubdivision::WeightCut0(PList &vp, LList &cutline, PList &newconcave)
{
    // if(vplistlen==0)
    // {
    //     cout<<"vplistlen=0"<<endl;
    //     return;
    // }
    float temp_vcx, temp_vcy;
    temp_vcx = PointMap[vp.N].x - PointMap[vplist[0]].x;
    temp_vcy = PointMap[vp.N].y - PointMap[vplist[0]].y;
    float temp_s = sqrtf(temp_vcx * temp_vcx + temp_vcy * temp_vcy);
    temp_vcx /= temp_s;
    temp_vcy /= temp_s;
    float temp_v0x, temp_v0y, temp_v0s;
    temp_v0x = 2 * PointMap[vp.N].x - PointMap[vp.S].x - PointMap[vp.E].x;
    temp_v0y = 2 * PointMap[vp.N].y - PointMap[vp.S].y - PointMap[vp.E].y;
    temp_v0s = sqrt(temp_v0x * temp_v0x + temp_v0y * temp_v0y);
    temp_v0x /= temp_v0s;
    temp_v0y /= temp_v0s;
    float temp_min = temp_s * ((temp_vcx * temp_v0x + temp_vcy * temp_v0y) + 1);
    float temp;
    uint32_t p_min = vplist[0];
    for (uint32_t i = 1; i < vplistlen; i++)
    {
        uint32_t p = vplist[i];
        temp_vcx = PointMap[vp.N].x - PointMap[p].x;
        temp_vcy = PointMap[vp.N].y - PointMap[p].y;
        temp_s = sqrtf(temp_vcx * temp_vcx + temp_vcy * temp_vcy);
        temp_vcx /= temp_s;
        temp_vcy /= temp_s;
        temp = temp_s * ((temp_vcx * temp_v0x + temp_vcy * temp_v0y) + 1);
        if (temp < temp_min)
        {
            temp_min = temp;
            p_min = p;
        }
    }
    cutline = {vp.N, p_min};
    float v1_x, v1_y, v3_x, v3_y;
    v3_x = PointMap[p_min].x - PointMap[vp.N].x;
    v3_y = PointMap[p_min].y - PointMap[vp.N].y;
    v1_x = PointMap[vp.N].x - PointMap[vp.S].x;
    v1_y = PointMap[vp.N].y - PointMap[vp.S].y;
    float r = v1_x * v3_y - v1_y * v3_x;
    if (r < 0)
    {
        newconcave = {vp.S, vp.N, p_min, -1};
        return;
    }
    v1_x = PointMap[vp.E].x - PointMap[vp.N].x;
    v1_y = PointMap[vp.E].y - PointMap[vp.N].y;
    r = v1_x * v3_y - v1_y * v3_x;
    if (r < 0)
    {
        newconcave = {p_min, vp.N, vp.E, -1};
        return;
    }
    newconcave = {0, 0, 0, 0};
}

void CSubdivision::WeightCut(PList &vp, LList &cutline, PList &newconcave)
{
    float temp_vcx, temp_vcy;
    temp_vcx = PointMap[vp.N].x - PointMap[vplist[0]].x;
    temp_vcy = PointMap[vp.N].y - PointMap[vplist[0]].y;
    float temp_s = sqrtf(temp_vcx * temp_vcx + temp_vcy * temp_vcy);
    // temp_vcx /= temp_s;
    // temp_vcy /= temp_s;
    // float temp_v0x, temp_v0y, temp_v0s;
    // temp_v0x = 2 * PointMap[vp.N].x - PointMap[vp.S].x - PointMap[vp.E].x;
    // temp_v0y = 2 * PointMap[vp.N].y - PointMap[vp.S].y - PointMap[vp.E].y;
    // temp_v0s = sqrt(temp_v0x * temp_v0x + temp_v0y * temp_v0y);
    // temp_v0x /= temp_v0s;
    // temp_v0y /= temp_v0s;
    // float temp_min = temp_s * ((temp_vcx * temp_v0x + temp_vcy * temp_v0y) + 1);
    float temp_min = temp_s;
    float temp;
    uint32_t p_min = vplist[0];
    for (uint32_t i = 1; i < vplistlen; i++)
    {
        uint32_t p = vplist[i];
        temp_vcx = PointMap[vp.N].x - PointMap[p].x;
        temp_vcy = PointMap[vp.N].y - PointMap[p].y;
        temp_s = sqrtf(temp_vcx * temp_vcx + temp_vcy * temp_vcy);
        // temp_vcx /= temp_s;
        // temp_vcy /= temp_s;
        // temp = temp_s * ((temp_vcx * temp_v0x + temp_vcy * temp_v0y) + 1);
        temp = temp_s;
        if (temp < temp_min)
        {
            temp_min = temp;
            p_min = p;
        }
    }
    cutline = {vp.N, p_min};
    float v1_x, v1_y, v3_x, v3_y;
    v3_x = PointMap[p_min].x - PointMap[vp.N].x;
    v3_y = PointMap[p_min].y - PointMap[vp.N].y;
    v1_x = PointMap[vp.N].x - PointMap[vp.S].x;
    v1_y = PointMap[vp.N].y - PointMap[vp.S].y;
    float r = v1_x * v3_y - v1_y * v3_x;
    if (r < 0)
    {
        newconcave = {vp.S, vp.N, p_min, -1};
        return;
    }
    v1_x = PointMap[vp.E].x - PointMap[vp.N].x;
    v1_y = PointMap[vp.E].y - PointMap[vp.N].y;
    r = v1_x * v3_y - v1_y * v3_x;
    if (r < 0)
    {
        newconcave = {p_min, vp.N, vp.E, -1};
        return;
    }
    newconcave = {0, 0, 0, 0};
}

void CSubdivision::StartCut(void)
{
    FindConcave();

    cutlinelistlen = 0;
    list<PList> concavelist;
    list<PList>::iterator concave;

    for (size_t i = 0; i < ConcaveListlen; i++)
        concavelist.push_back(ConcaveList[i]);

    uint32_t cutline_count = 0;
    while (concavelist.size())
    {
        // cout<<concavelist.size()<<endl;
        PList vp = concavelist.front();
        concavelist.pop_front();

        ViewablePoint(vp);
        LList cutline;
        PList newconcave;
        WeightCut(vp, cutline, newconcave);
        pointcutlist[cutline.S].push_back(PClink{cutline.E, cutline_count});
        pointcutlist[cutline.E].push_back(PClink{cutline.S, cutline_count});
        cutline_count++;
        cutlinelist[cutlinelistlen++] = cutline;

        if (newconcave.key != 0)
        {
            concavelist.push_front(newconcave);
        }
        for(concave = concavelist.begin();concave != concavelist.end();concave++)
        {
            if(concave->N==cutline.E)
            {
                float v1_x, v1_y, v3_x, v3_y;
                v3_x = PointMap[vp.N].x - PointMap[concave->N].x;
                v3_y = PointMap[vp.N].y - PointMap[concave->N].y;
                v1_x = PointMap[concave->N].x - PointMap[concave->S].x;
                v1_y = PointMap[concave->N].y - PointMap[concave->S].y;
                float r = v1_x * v3_y - v1_y * v3_x;
                newconcave.key = 0;
                if (r < 0)
                    newconcave = {concave->S, concave->N, vp.N, -1};
                else
                {
                    v1_x = PointMap[concave->E].x - PointMap[concave->N].x;
                    v1_y = PointMap[concave->E].y - PointMap[concave->N].y;
                    r = v1_x * v3_y - v1_y * v3_x;
                    if (r < 0)
                        newconcave = {vp.N, concave->N, concave->E, -1};
                }
                concavelist.erase(concave);
                if (newconcave.key != 0)
                {
                    concavelist.push_front(newconcave);
                }
                break;

            }
        }
    }
    // cout<<"Cut OK! cutlinelistlen: "<<cutlinelistlen<<endl;
}

void CSubdivision::GetPloygon(void)
{
    bridgelist.resize(cutlinelistlen);
    if (cutlinelistlen == 0)
    {
        islandlist.resize(1);
        for (uint32_t i = 0; i < PointMaplen; i++)
            islandlist[0].point.push_back(i);
        islandlist[0].cutline.clear();
        return;
    }
    // islandlist.emplace_back();
    for (uint32_t pointcut_count = 0; pointcut_count < PointMaplen; pointcut_count++)
    {
        // cout<<pointcut_count<<endl;
        list<PClink> &pointcut = pointcutlist[pointcut_count];
        if (pointcut.size() == 0)
            continue;
        list<PClink>::iterator cutline_count;
        for (cutline_count = pointcut.begin(); cutline_count != pointcut.end(); ++cutline_count)
        {

            // island island_temp = islandlist.back();
            island island_temp;
            // island_temp.cutline.clear();
            // island_temp.point.clear();
            PClink &cutline = *cutline_count;//pointcut[cutline_count];
            // if (cutline.cutline == 0xffffffff)
            //     continue;
            island_temp.point.push_back(pointcut_count);
            island_temp.cutline.push_back(cutline.cutline);
            bridgelist[cutline.cutline].push_back(islandlist.size());
            uint32_t Pago = pointcut_count;
            uint32_t Pnow = cutline.plink;
            while (Pnow != pointcut_count)
            {
                island_temp.point.push_back(Pnow);
                if (pointcutlist[Pnow].size()==0) //(pointcutlist[Pnow].size() == 0) ///////////////////
                {
                    Pago = Pnow;
                    Pnow = PointList[Pnow].E;
                }
                else
                {
                    // int32_t minpoint_count = -1;
                    uint32_t Pnex, pointlist_nex;
                    Pnex = pointlist_nex = PointList[Pnow].E;
                    CSpoint nnex = PointMap[Pnex];
                    CSpoint nnow = PointMap[Pnow];
                    CSpoint nago = PointMap[Pago];
                    float v1_x = nnow.x - nago.x;
                    float v1_y = nnow.y - nago.y;
                    float v2_x = nnex.x - nnow.x;
                    float v2_y = nnex.y - nnow.y;
                    float minangle;
                    if (v1_x * v2_y - v1_y * v2_x >= 0)
                        minangle = (v1_x * v2_x + v1_y * v2_y)        //
                                   / sqrtf(v1_x * v1_x + v1_y * v1_y) //
                                   / sqrtf(v2_x * v2_x + v2_y * v2_y);
                    else
                        minangle = 1.20;
                    list<PClink>::iterator Pnex_count;
                    list<PClink>::iterator minpoint_count=pointcutlist[Pnow].end();
                    for(Pnex_count=pointcutlist[Pnow].begin();Pnex_count!=pointcutlist[Pnow].end();++Pnex_count)
                    {
                        Pnex = Pnex_count->plink;
                        if (Pnex != Pago)
                        {
                            nnex = PointMap[Pnex];
                            nnow = PointMap[Pnow];
                            nago = PointMap[Pago];
                            v1_x = nnow.x - nago.x;
                            v1_y = nnow.y - nago.y;
                            v2_x = nnex.x - nnow.x;
                            v2_y = nnex.y - nnow.y;
                            if (v1_x * v2_y - v1_y * v2_x >= 0)
                            {
                                float cosangle = (v1_x * v2_x + v1_y * v2_y)        //
                                                 / sqrtf(v1_x * v1_x + v1_y * v1_y) //
                                                 / sqrtf(v2_x * v2_x + v2_y * v2_y);
                                if (cosangle < minangle)
                                {
                                    minpoint_count = Pnex_count;
                                    minangle = cosangle;
                                }
                            }
                        }
                    }
                    if (minpoint_count != pointcutlist[Pnow].end())
                    {
                        island_temp.cutline.push_back(minpoint_count->cutline);
                        bridgelist[minpoint_count->cutline].push_back(islandlist.size());
                        Pago = Pnow;
                        Pnow = minpoint_count->plink;
                        pointcutlist[Pago].erase(minpoint_count);
                    }
                    else
                    {
                        Pago = Pnow;
                        Pnow = pointlist_nex;
                    }
                    
                }
            }
            islandlist.push_back(island_temp);
        }
    }
}

CSubdivision::CSubdivision(vector<CSpoint> &Points)
{
    PointMaplen = Points.size();
    PointMap.resize(PointMaplen); 
    for (uint32_t i = 0; i < PointMaplen; i++)
        PointMap[i] = Points[i];
    ConcaveList.resize(PointMaplen*1000);
    ConcaveListlen = 0;

    PointList.resize(PointMaplen);

    linelist.resize(PointMaplen);
    linelistlen = 0;
    cutlinelist.resize(PointMaplen * 2000);
    cutlinelistlen = 0;

    vplist.resize(PointMaplen);
    vplistlen = 0;
    pointcutlist.resize(PointMaplen);
}

CSubdivision::~CSubdivision()
{
}
