/*####################*/
// Author => Ramith Hettiarachchi < im@ramith.fyi >
// CSIRO Data61
/*####################*/

#include "app.hpp"
#include<iostream>
#include <queue>
#include <limits>
#include <fstream>
#include <cmath>
#include <sstream>
#include <chrono>
#include <thread>
#include <random>

#define FRAMES_PER_SECOND 5

double Inf = 10e3;
using namespace std;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

struct vertex{  //Stores a vertex along with k1,k2 Costs

    int x,y;
    float k1;
    float k2;
    vertex(int,int,float,float);
    vertex():x(0),y(0),k1(0),k2(0){}
    
};
vertex::vertex(int p_x,int p_y,float p_k1, float p_k2):x{p_x},y{p_y},k1{p_k1},k2{p_k2}{}
//Goal & start
vertex s_goal(25,25,0,0);
vertex s_start(0,0,0,0);
vertex s_last(0,0,0,0);
int km = 0;

//constraints 
#define grid_s_x 50
#define grid_s_y 50

double rhs[grid_s_x+1][grid_s_y+1];
double   g[grid_s_x+1][grid_s_y+1];

bool GRID[grid_s_x+1][grid_s_y+1]=
{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,0,0,0,0,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,1,1,1,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,0,0,1,0,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,0,1,0,0,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,0,0,1,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,0,0,0,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,1,1,1,1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
bool PATH[grid_s_x+1][grid_s_y+1];

bool   Ukey[grid_s_x+1][grid_s_y+1];
double Ukey_k1[grid_s_x+1][grid_s_y+1];
double Ukey_k2[grid_s_x+1][grid_s_y+1];

    
struct compare{ //Custom Comparison Function
    bool operator()(const vertex & a, const vertex & b){   
            if(a.k1 > b.k1){
                return 1;
            }else if((a.k1 == b.k1)){
                if(a.k2 > b.k2)return 1;
                else return 0;
            }else return 0;
    }
};

bool isVertexEqual(vertex v1,vertex v2){
    if(v1.x == v2.x && v1.y == v2.y){
        return 1;
    }
    return 0;
}

typedef priority_queue<vertex, vector<vertex>, compare > m_priority_queue; //Min Priority Queue

m_priority_queue U;

void showpq(m_priority_queue gq){
    m_priority_queue g = gq;
    while (!g.empty()) {
        vertex c_v = g.top();

        cout << '\t' <<c_v.x<<","<<c_v.y<<"("<<c_v.k1<<","<<c_v.k2<<")"<<"   " ;
        g.pop();
    }
    cout << '\n';
}

double h(vertex s1,vertex s2){   
    //heuristic function
    return sqrt(pow((s1.x-s2.x),2) + pow((s1.y-s2.y),2));
}

bool isInQueue(vertex s){
    if(Ukey[s.x][s.y]==1){
        return 1;
    }
    return 0;
}

void pushToQueue(vertex s){
    Ukey[s.x][s.y] = 1;
    Ukey_k1[s.x][s.y] = s.k1;
    Ukey_k2[s.x][s.y] = s.k2;
    U.push(s);
}

bool isCostLower(vertex b, vertex a){   
    if(a.k1 > b.k1){
        return 1;
    }else if(a.k1 == b.k1){
        if(a.k2 > b.k2)return 1;
        else return 0;
    }else return 0;
}

vertex CalculateKey(vertex s){
    //cout<<"✨";
    if(s.x < 0 || s.x > grid_s_x || s.y < 0 || s.y > grid_s_y){
        s.k1 = Inf;
        s.k2 = Inf;
        return s;
    }


    double k1  = min(g[s.x][s.y],rhs[s.x][s.y]) + h(s_start,s) + km;
    double k2  = min(g[s.x][s.y],rhs[s.x][s.y]);

    s.k1 = k1;
    s.k2 = k2;
    return s;
}


void UpdateVertex(vertex u){
    
    if(u.x < 0 || u.x > grid_s_x || u.y < 0 || u.y > grid_s_y){
        return;
    }


    if(!isVertexEqual(u,s_goal)){
        double c1,c2,c3,c4,c5,c6,c7,c8;

        if(u.y+1 > grid_s_y)c1 = Inf;
        else c1 = g[u.x  ][u.y+1] + 1     + (GRID[u.x][u.y] + GRID[u.x  ][u.y+1])*Inf;

        if(u.x+1 > grid_s_x)c2 = Inf;
        else c2 = g[u.x+1][u.y  ] + 1     + (GRID[u.x][u.y] + GRID[u.x+1][u.y]  )*Inf;

        if(u.y-1 < 0) c3 = Inf;
        else c3 = g[u.x  ][u.y-1] + 1     + (GRID[u.x][u.y] + GRID[u.x  ][u.y-1])*Inf;

        if(u.x-1 < 0) c4 = Inf;
        else c4 = g[u.x-1][u.y]   + 1     + (GRID[u.x][u.y] + GRID[u.x-1][u.y  ])*Inf;

        if(u.x-1 < 0 || u.y - 1 < 0) c5 = Inf;
        else c5 = g[u.x-1][u.y-1] + 1.414 + (GRID[u.x][u.y] + GRID[u.x-1][u.y-1])*Inf;

        if(u.x-1 < 0 || u.y + 1 > grid_s_y) c6 = Inf;
        else c6 = g[u.x-1][u.y+1] + 1.414 + (GRID[u.x][u.y] + GRID[u.x-1][u.y+1])*Inf;

        if(u.x + 1 > grid_s_x || u.y - 1 < 0) c7 = Inf;
        else c7 = g[u.x+1][u.y-1] + 1.414 + (GRID[u.x][u.y] + GRID[u.x+1][u.y-1])*Inf;

        if(u.x + 1 > grid_s_x || u.y + 1 > grid_s_y) c8 = Inf;
        else c8 = g[u.x+1][u.y+1] + 1.414 + (GRID[u.x][u.y] + GRID[u.x+1][u.y+1])*Inf;

        rhs[u.x][u.y] = min(min(min(c3,c4),min(c1,c2)),min(min(c7,c8),min(c5,c6)));
    }
    u = CalculateKey(u);
    if(isInQueue(u)){
        Ukey[u.x][u.y] = 0; //remove from Priority Queue
    } 
    if(rhs[u.x][u.y]!=g[u.x][u.y]){
        pushToQueue(u);
    }
}

bool isGhost(vertex s){
    if(Ukey[s.x][s.y]==1 && Ukey_k1[s.x][s.y]==s.k1 && Ukey_k2[s.x][s.y]==s.k2){
        return 0;
    }
    return 1;
}

void pop(){
    vertex s = U.top();
    Ukey[s.x][s.y]=0;
    U.pop();
}

vertex TopKey(){
    if(U.size()==0)return vertex(0,0,Inf,Inf);

    vertex temp = U.top();

    while(isGhost(temp)){
        pop(); //pop unwanted ones
        if(U.size()==0) return vertex(0,0,Inf,Inf);
        temp = U.top();
    }
    return temp; //return top most vertex which isn't a ghost
}


void ComputeShortestPath(){

    while(isCostLower(TopKey(),CalculateKey(s_start)) ||
        rhs[s_start.x][s_start.y] != g[s_start.x][s_start.y])
    {

        //cout<<"🍀 => "<<U.size();
        vertex k_old = TopKey();
        pop();
        vertex u     = k_old;
        //cout<<" <= Selected "<<u.x<<","<<u.y<<endl;
        //cout<<k_old.k1<<","<<k_old.k2<<endl;

        if(isCostLower(k_old,CalculateKey(u))){
            u = CalculateKey(u);
            pushToQueue(u);
        }else if(g[u.x][u.y] > rhs[u.x][u.y]){
            g[u.x][u.y] = rhs[u.x][u.y];
            //cout<<" => g[u.x][u.y] > rhs[u.x][u.y]"<<endl;
            UpdateVertex(vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y+1,0,0));
        }else{
            g[u.x][u.y] = Inf;
            //cout<<" => else"<<endl;

            UpdateVertex(vertex(u.x   ,u.y  ,0,0));

            UpdateVertex(vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(vertex(u.x +1  ,u.y+1,0,0));
        }



    }

}
void fillGRID(){
    string line;
    ifstream textfile("in.in");

    int i = 0;
    while (getline (textfile, line)) {
        std::stringstream ss(line);

        int j=0;
        for (int x; ss >> x;j++) {
            GRID[j][i]=x;
            if (ss.peek() == ',')
                ss.ignore();
        }
        i++;
    }
    textfile.close();
}

void fillGRID_(bool random=0){

    if(random){
        for(int i=0;i <= grid_s_x;i++)
            for(int j=0;j<= grid_s_y;j++)
                GRID[i][j] = rand() & 1;
    }else{
        for(int i=0;i <= grid_s_x;i++)
            for(int j=0;j<= grid_s_y;j++)
                GRID[i][j] = 0;
    }
    GRID[s_goal.x ][s_goal.y] = 0;
    GRID[s_start.x][s_start.y] = 0;
    
}
void initiliaze(){
    s_last = s_start;

    while(U.size()){
        U.pop();
    }

    km = 0;
    for(int i=0;i <= grid_s_x;i++)
        for(int j=0;j<= grid_s_y;j++){
            rhs[i][j] = Inf;
              g[i][j] = Inf;
            PATH[i][j]= 0;
        }
    rhs[s_goal.x][s_goal.y] = 0;
}

void run(){
    initiliaze();
    
    cout<<"Successfully loaded GRID"<<endl;

    for(int k=0;k<20;k++){
        for(int m=0;m<20;m++){
            cout<<GRID[k][m]<<" ";
        } 
        cout<<endl;
    }
    cout<<"Priority Queue Size = "<<U.size()<<endl;

    s_goal = CalculateKey(s_goal);
    cout<<"going to push to queue...";
    pushToQueue(s_goal);
    cout<<"going to compute shortest path...";
    ComputeShortestPath();

    //while(s_goal.x!=s_start.x || s_goal.y!=s_start.y){


    //}

    cout<<"Successfully Computed Cost => \n";
    /*
    for(int k=0;k<40;k++){
        for(int m=0;m<40;m++){
            if(g[k][m]>=Inf)cout<<"XX ";
            else {
                if(g[k][m]<10)cout<<" "<<int(g[k][m])<<" ";
                else cout<<int(g[k][m])<<" ";
            }
        } 
        cout<<endl;
    }*/
    
    cout<<g[s_start.x][s_start.y]<<endl;

  
}
void App::setup() {
    // load fonts and images here
    cout<<"Starting!";
    run();
}

void Traverse(vertex pos){
    if(pos.x < 0 || pos.x > grid_s_x || pos.y < 0 || pos.y > grid_s_y){
      return;
    }
    PATH[pos.x][pos.y] = 1;
}

int indexofSmallestElement(double array[]){
    int index = 0;

    for(int i = 1; i < 8; i++){
        if(array[i] < array[index])
            index = i;              
    }

    return index;
}
int  moves[8][2] = {{-1,0},{0,-1},{1,0},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};
string moves_d[8]  = {"↑","←","↓","→","⬉","⬈","⬋","⬊"};
string out= "";
string msg= "";

bool enabled = 0;
bool r = 0;

int mouseX;
int mouseY;

double step_cost(int x,int y){
    if(x < 0 || x > grid_s_x || y < 0 || y > grid_s_y){
        return Inf;
    }else return g[x][y];
}

void onestep(){
    
    double c_cost = g[s_last.x][s_last.y];
    
    double arr[8] = {};
    for(int i=0; i<8; i++){ 
        arr[i] = step_cost(s_last.x + moves[i][0],s_last.y + moves[i][1]);
        //cout<<arr[i]<<",";
    }
    cout<<" ✨ "<<s_last.x<<","<<s_last.y<<endl;


    int min_index = indexofSmallestElement(arr);

    out += moves_d[min_index];

    s_last.x = s_last.x + moves[min_index][0];
    s_last.y = s_last.y + moves[min_index][1];

    Traverse(s_last);
}



void App::keyPressed(int key){
    cout<<key<<endl;
    //g.rect(100, 100, 400, 400);
    if(key==92){ //run algo 
        run();
    }

    if(key==257){
        if(g[s_start.x][s_start.y]==Inf)run();
        onestep();
    }
    if(key==82){
        cout<<"🏃‍♀️"<<endl;
        run();
        out = "";
        r = 0;
        
        if(g[s_start.x][s_start.y]>=Inf){
            msg="No Path Exists! :( ";
            enabled = 0;
            r = 1;
        }else{
            cout<<"\n";
            enabled = 2;
        }
    }
    if(key==32){  //space bar => clear
        r = 0;
        msg =" ";
        enabled = 0;
        initiliaze();
    }

    if(key==70){  // F => random
        initiliaze();
        fillGRID_(1);
    }
    if(key==67){  // C => clear
        initiliaze();
        fillGRID_();
    }
}

void App::mouseMoved(int x, int y){
    mouseX = x;
    mouseY = y;
}


void App::mousePressed(int button){

    int blocksize = int(height/grid_s_y);
    
    int x_pos = int(mouseX/blocksize);
    int y_pos = int(mouseY/blocksize);

    if(GRID[y_pos][x_pos]==1){
        GRID[y_pos][x_pos]=0;
    }else{
        GRID[y_pos][x_pos]=1;
    }
}



void App::draw(piksel::Graphics& gcanvas) {
    int blocksize = int(height/grid_s_y);
    //int left_align = width - blocksize*grid_s_x + 250;


    gcanvas.background(glm::vec4(0.1171875f, 0.1171875f, 0.1171875f,1.0f));

    for(int x=0;x     < blocksize*grid_s_x ;  x+=blocksize){
        for(int y=0;y < blocksize*grid_s_y ;  y+=blocksize){
            int x_pos = x/blocksize;
            int y_pos = y/blocksize;

            if(GRID[x_pos][y_pos]==1){
                gcanvas.fill(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
            }else{
                gcanvas.fill(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
            }
            if(PATH[x_pos][y_pos]==1){
                gcanvas.fill(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
            }
            if(s_goal.x==x_pos && s_goal.y==y_pos){
                gcanvas.fill(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
            }
            if(s_start.x==x_pos && s_start.y==y_pos){
                gcanvas.fill(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
            }
  
            gcanvas.rect(y, x, blocksize, blocksize);
        }   
    }
    if(enabled){
        if(s_last.x < 0 || s_last.x > grid_s_x || s_last.y < 0 || s_last.y >grid_s_y)enabled=0;

        if(s_last.x!=s_goal.x or s_last.y!=s_goal.y){
            onestep();
            sleep_until(system_clock::now() + nanoseconds(40000000));
        }else{
            //cout<<out<<endl;

            std::ostringstream ss;
            ss << g[s_start.x][s_start.y];
            std::string s(ss.str());

            msg = "Done! cost = " + s;
            enabled-=1;
        }
    }
    
    gcanvas.fill(glm::vec4(1, 1, 1, 1.0f));
    gcanvas.textSize(16);
    gcanvas.text("Press 'r' to start traversing!",810,70);
    gcanvas.text("Press 'Enter' to make one step",810,100);
    gcanvas.text("Press 'space' to reset path",810,160);
    gcanvas.text("Press 'F' to Fill grid randomly",810,190);
    gcanvas.text("Press 'C' to Clear grid ",810,220);
    gcanvas.fill(glm::vec4(r, !r, 0.0f, 1.0f));
    gcanvas.text(msg,810,30);
   
}
