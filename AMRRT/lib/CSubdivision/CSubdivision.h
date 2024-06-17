#ifndef __CSubdivision_H
#define __CSubdivision_H
#include <iostream>
#include <stdio.h>
#include <vector>
#include <deque>
#include <list>
using namespace std;

typedef struct
{
    float x;
    float y;
} CSpoint;
typedef struct
{
    uint32_t S;
    uint32_t N;
    uint32_t E;
    int32_t key;
} PList;
typedef struct
{
    uint32_t S;
    uint32_t E;
} LList;
typedef struct CSnode
{
    PList node;
    // CSnode* last;
    CSnode *next;
} CSnode;

typedef struct
{
    vector<uint32_t> point;
    vector<uint32_t> cutline;
}island;

typedef struct
{
    uint32_t plink;
    uint32_t cutline;
}PClink;

class CSubdivision
{
private:
public:
    vector<PList> PointList;
    // uint32_t PointListlen;

    vector<list<PClink>> pointcutlist;
    // int32_t *pointcutlistcount;


    // CSnode *CSnodeRoot;
    // CSnode *CSnodeFree;
    // uint32_t CSnodeFreelen;

    vector<LList> linelist;
    uint32_t linelistlen;
    vector<LList> cutlinelist;
    uint32_t cutlinelistlen;

    vector<uint32_t> vplist; //用于ViewablePoint与StartCut的可视点交换
    uint32_t vplistlen;

    vector<CSpoint> PointMap;
    uint32_t PointMaplen;

    vector<PList> ConcaveList;
    uint32_t ConcaveListlen;

    vector<vector<uint32_t>> bridgelist;
    vector<island> islandlist;

    void FindConcave(void);
    void ViewablePoint(PList &vp);
    void WeightCut0(PList &vp, LList &cutline, PList &newconcave);
    void WeightCut(PList &vp, LList &cutline, PList &newconcave);
    void StartCut(void);
    void GetPloygon(void);

    CSubdivision(vector<CSpoint>& Points);
    ~CSubdivision();
};
#endif