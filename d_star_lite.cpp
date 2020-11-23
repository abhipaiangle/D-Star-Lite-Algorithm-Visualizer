#include<iostream>
#include <queue>
using namespace std;

struct vertex{
    int x,y;
    float k1;
    float k2;
};

struct compare{
    bool operator()(const vertex & a, const vertex & b)
        {   
            if(a.k1 > b.k1){
                return 1;
            }else if((a.k1 == b.k1)){
                if(a.k2 > b.k2)return 1;
                else return 0;
            }else{
                return 0;
            }

    }
};


typedef priority_queue<vertex, vector<vertex>, compare > m_priority_queue;


void showpq(m_priority_queue gq)
{
    m_priority_queue g = gq;
    while (!g.empty()) {
        vertex c_v = g.top();

        cout << '\t' <<c_v.x<<","<<c_v.y ;
        g.pop();
    }
    cout << '\n';
}

int main(){
    m_priority_queue gquiz;


    vertex v1 = {0,0,100,40};
    vertex v2 = {1,0,800,40};
    vertex v3 = {1,1,3,40  };
    vertex v4 = {1,2,3,30  };
   
    cout<<v1.k2<<endl;
    gquiz.push(v1);
    gquiz.push(v2);
    gquiz.push(v3);
    gquiz.push(v4);
  
    cout << "The priority queue gquiz is : ";
    showpq(gquiz);
 
    cout << "\ngquiz.size() : " << gquiz.size();
    cout << "\ngquiz.top() : " << gquiz.top().x<< gquiz.top().y;
 
    cout << "\ngquiz.pop() : ";
    gquiz.pop();
    showpq(gquiz);

    return 0;
}