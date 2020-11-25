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

#define FRAMES_PER_SECOND 5

double Inf = 10e9;
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
vertex s_goal(10,14,0,0);
vertex s_start(0,0,0,0);
vertex s_last(0,0,0,0);
int km = 0;

//constraints 
#define grid_s_x 50
#define grid_s_y 50

double rhs[grid_s_x][grid_s_y];
double g[grid_s_x][grid_s_y];

bool GRID[grid_s_x][grid_s_y]=
{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,0,0,0,0,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,1,1,1,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,1,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,0,0,1,0,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,0,1,0,0,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,0,0,1,1,0,1,0,1,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,0,0,0,0,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,1,1,1,1,1,1,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,1,1,1,1,1,1,1,0,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
bool PATH[grid_s_x][grid_s_y];

bool Ukey[grid_s_x][grid_s_y];
float Ukey_k1[grid_s_x][grid_s_y];
float Ukey_k2[grid_s_x][grid_s_y];

    
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
    U.push(s);
    Ukey[s.x][s.y] = 1;
    Ukey_k1[s.x][s.y] = s.k1;
    Ukey_k2[s.x][s.y] = s.k2;
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
    double k1  = min(g[s.x][s.y],rhs[s.x][s.y]) + h(s_start,s) + km;
    double k2  = min(g[s.x][s.y],rhs[s.x][s.y]);

    s.k1 = k1;
    s.k2 = k2;
    return s;
}


void UpdateVertex(vertex u){
    

    //cout<<" => Update Vertex "<<u.x<<","<<u.y<<endl;
    if(u.x < 0 || u.x > grid_s_x || u.y < 0 || u.y > grid_s_y){
        return;
    }


    if(!isVertexEqual(u,s_goal)){
        double c1,c2,c3,c4,c5,c6,c7,c8;
        
        c1 = g[u.x  ][u.y+1] + 1 + GRID[u.x][u.y]*Inf;
        c2 = g[u.x+1][u.y  ] + 1 + GRID[u.x][u.y]*Inf;
        c3 = g[u.x  ][u.y-1] + 1 + GRID[u.x][u.y]*Inf;
        c4 = g[u.x-1][u.y]   + 1 + GRID[u.x][u.y]*Inf;
        c5 = g[u.x-1][u.y-1] + 1.414 + GRID[u.x][u.y]*Inf;
        c6 = g[u.x-1][u.y+1] + 1.414 + GRID[u.x][u.y]*Inf;
        c7 = g[u.x+1][u.y-1] + 1.414 + GRID[u.x][u.y]*Inf;
        c8 = g[u.x+1][u.y+1] + 1.414 + GRID[u.x][u.y]*Inf;

        if(u.y+1 > grid_s_y || GRID[u.x][u.y+1]==1){c1 = Inf; c6 = Inf; c8 = Inf; }
        if(u.x+1 > grid_s_x || GRID[u.x+1][u.y]==1){c2 = Inf; c7 = Inf; c8 = Inf; }
        if(u.y-1 < 0        || GRID[u.x][u.y-1]==1){c3 = Inf; c5 = Inf; c7 = Inf; }
        if(u.x-1 < 0        || GRID[u.x-1][u.y]==1){c4 = Inf; c5 = Inf; c6 = Inf; }

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
    if(U.size()==0)return vertex(-1,-1,Inf,Inf);

    vertex temp = U.top();

    while(isGhost(temp)){
        pop(); //pop unwanted ones
        if(U.size()==0) return vertex(-1,-1,Inf,Inf);
        temp = U.top();
    }
    return temp; //return top most vertex which isn't a ghost
}


void ComputeShortestPath(){

    while(isCostLower(TopKey(),CalculateKey(s_start)) ||
        rhs[s_start.x][s_start.y] != g[s_start.x][s_start.y])
    {

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


void run(){

    //Initialize
    //fillGRID();
    cout<<"Successfully loaded GRID"<<endl;

    for(int k=0;k<20;k++){
        for(int m=0;m<20;m++){
            cout<<GRID[k][m]<<" ";
        } 
        cout<<endl;
    }

    km = 0;
    for(int i=0;i < grid_s_x;i++)
        for(int j=0;j<grid_s_y;j++){
            rhs[i][j] = Inf;
              g[i][j] = Inf;
        }
    rhs[s_goal.x][s_goal.y] = 0;


    

    s_goal = CalculateKey(s_goal);
    pushToQueue(s_goal);
    
    ComputeShortestPath();
    //showpq(U);

    cout<<"Successfully Computed Cost => ";
    /*
    for(int k=0;k<40;k++){
        for(int m=0;m<40;m++){
            if(g[k][m]>=Inf)cout<<"XX ";
            else {
                if(g[k][m]<10)cout<<" "<<g[k][m]<<" ";
                else cout<<g[k][m]<<" ";
            }
        } 
        cout<<endl;
    }
    */
    cout<<g[s_start.x][s_start.y]<<endl;

  
}
void App::setup() {
    // load fonts and images here
    cout<<"Starting!";
    run();
}

void Traverse(vertex pos){
    if(pos.x < 0 || pos.x > grid_s_x || pos.y < 0 || pos.y >grid_s_y){
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


void onestep(){
    double c_cost = g[s_last.x][s_last.y];
    
    double arr[8] = {};
    for(int i=0; i<8; i++)arr[i] = g[s_last.x + moves[i][0]][s_last.y + moves[i][1]];

    int min_index = indexofSmallestElement(arr);
    //printf("%s",moves_d[min_index].c_str());
    out+=moves_d[min_index];
    //cout<<moves_d[min_index]<<"\n";

    s_last.x = s_last.x + moves[min_index][0];
    s_last.y = s_last.y + moves[min_index][1];

    Traverse(s_last);
}

bool enabled = 0;
void App::keyPressed(int key){
    //cout<<key<<endl;
    //g.rect(100, 100, 400, 400);
    if(key==257){
        onestep();
    }
    if(key==82){
        cout<<"\n";
        enabled = 2;
    }

}

void App::draw(piksel::Graphics& g) {


    for(int x=0;x     < int(height/grid_s_x)*grid_s_x -100; x+=int(height/grid_s_x)){
        for(int y=0;y < int(height/grid_s_x)*grid_s_y -100;  y+=int(height/grid_s_y)){

            int blocksize = int(height/grid_s_y);
            int x_pos = x/blocksize;
            int y_pos = y/blocksize;

            if(GRID[x_pos][y_pos]==1){
                g.fill(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
            }else{
                g.fill(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
            }
            if(PATH[x_pos][y_pos]==1){
                g.fill(glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
            }
            if(s_goal.x==x_pos && s_goal.y==y_pos){
                g.fill(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
            }
            if(s_start.x==x_pos && s_start.y==y_pos){
                g.fill(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
            }
  
            g.rect(y, x, blocksize, blocksize);

        }   
    }
    if(enabled){
        if(s_last.x!=s_goal.x or s_last.y!=s_goal.y){
            onestep();
            sleep_until(system_clock::now() + nanoseconds(40000000));
        }else{
            cout<<out<<endl;
            enabled-=1;
        }
    }
    g.textSize(16);
    g.fill(glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    g.text("Press 'r' to start traversing!",50,750);
   
}
