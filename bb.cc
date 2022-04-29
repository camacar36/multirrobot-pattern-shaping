#include <iostream>
#include <vector>
#include <chrono>
#include <climits>
#include <queue>
#include <cmath>
#include <iomanip> 

using namespace std;

struct Point{
    double x, y;
};

    
// tipo de dato que va a formar la cola de prioridad
struct Node{
    double          current_weight;
    double          pessimistic;
    double          optimistic;
    uint             r;
    uint             p;
    vector<bool>    path;
    vector<uint>     sol;
};

// función de ordenación de la cola de prioridad
struct is_worse{
    bool operator() (const Node &a, const Node &b){
        // return a.opt_bound > b.opt_bound;
        // return a.cost > b.cost;
        return a.pessimistic > b.pessimistic;
    }
};


void create_matrix(const vector<Point> &robots, const vector<Point> &points,  vector<vector<double>> &m, vector<bool> &path, vector<double> &m_opt){

    path[0] = true;
    vector<double> aux_opt;

    for(uint i = 1; i < m.size(); i++){

        path[i] = false;
        double best_local = INT_MAX;

        for(uint j = 1; j < m[i].size(); j++){

            double px = points[i].x;
            double py = points[i].y;

            double rx = robots[j].x;
            double ry = robots[j].y;

            m[i][j] = sqrt(pow(px-rx, 2) + pow(py - ry, 2));

            if(m[i][j] < best_local)
                best_local = m[i][j];
        }
        aux_opt.push_back(best_local);
    }

    for(uint i = 0; i < aux_opt.size(); i++){
        double sum = 0;
        for(uint j = i + 1; j < aux_opt.size(); j++){
            sum += aux_opt[j];
        }
        m_opt.push_back(sum);
    }

}

double greedy(const vector<vector<double>> &m, const vector<bool> &path, int p){

    double greedy = 0;
    vector<bool> aux_path = path;
    
    for(uint i = p+1; i < m.size(); i++){

        double best_val = 1000;
        int pos_best = 0;

        for(uint j = 1; j < m[0].size();j++){

            if (!aux_path[j]){

                if(m[i][j] < best_val){

                    best_val = m[i][j];
                    pos_best = j;
                }
            }
        }
        aux_path[pos_best] = true;
        greedy += best_val;
    }

    return greedy;

}

double greedy_opt(const vector<vector<double>> &m, int p){

    double greedy = 0;

    for(uint i = p+1; i < m.size(); i++){
        double best_val = 1000;
        for(uint j = 1; j < m[0].size();j++){
            if(m[i][j] < best_val){
                best_val = m[i][j];
            }            
        }
        greedy += best_val;
    }

    return greedy;

}

bool is_leaf(int p, int total_points){
    return p == total_points;
}

bool is_feasible(bool b){
    return !b;
}

void print_node(Node n){
    cout << "----Node----" << endl;
    cout << "Point: " << n.p << endl;
    cout << "Robot: " << n.r << endl;
    cout << "Current weight: " << n.current_weight << endl;
    cout << "Pessimistic: " << n.pessimistic << endl;
    cout << "Optimistic: " << n.optimistic << endl;
    cout << "Path: " << endl;
    for(uint j = 0; j < n.path.size(); j++){
        cout << n.path[j] << " ";
    }
    cout << endl;
    cout << "Sol: " << endl;
    for(uint j = 0; j < n.sol.size(); j++){
        cout << n.sol[j] << " ";
    }
    cout << endl;

}

void bb_algorithm(const vector<vector<double>> &m, vector<bool> &path, vector<uint> &sol, const vector<double> &m_opt){
    auto start = clock();
    // calcular pessimistic bound con greedy e igualar al mejor valor actual (bast_val)
    double pessimistic_bound = greedy(m, path, 0);

    // calcular optimistic bound
    double optimistic_bound = m_opt[0];

    // mejor valor hasta ahora es la cota pesimista inicial
    double best = 12345678;
    vector<uint> best_sol;

    // crear la cola de prioridad
    priority_queue <Node, vector<Node>, is_worse> pq;
    pq.push({0, pessimistic_bound, optimistic_bound, 0, 0, path, {}});

    // bucle principal
    while(!pq.empty()){

        Node n = pq.top();
        pq.pop();     
        
        
        // print_node(n);

        // comprobar si es un nodo hoja
        // if(is_leaf(n.p, m.size() - 1)){
        if(n.p == m.size()-1){
            // cout << "hoja" << endl;
            if(n.current_weight <= best){
                best = n.current_weight;
                best_sol = n.sol;
                // cout << "mejor camino encontrado" << endl;
            }
            continue;
        }

        

        // expandir hijos
        for(uint r = 1; r < m.size(); r++){
            
            // comprobar que es factible
            //if(is_feasible(n.path[r])){
            if(!n.path[r]){
                
                vector<uint> aux_sol = n.sol;
                aux_sol.push_back(r);

                
                // calcular el peso del hijo
                double weight = n.current_weight + m[n.p + 1][r];

                // calcular un camino auxiliar para incluir el nodo hijo como ya recorrido
                vector<bool> aux_path = n.path;
                aux_path[r] = true;

                //calcular la cota pesimista del hijo
                pessimistic_bound = weight + greedy(m, aux_path, n.p + 1);

                // comprobar que la cota pesimista es menor que el mejor valor actual
                optimistic_bound = weight + m_opt[n.p];

                if(pessimistic_bound < best){
                    // cout << "nuevo best encontrado" << endl;
                    best = pessimistic_bound;
                }

                // cout << "opt: " << optimistic_bound << " weight: " << weight << " best: " << best << " pes: " << pessimistic_bound <<  endl; 
                // cout << "=: " << (optimistic_bound == best) << " >: " << (optimistic_bound > best) << " <: " << (optimistic_bound < best) << endl;
                if(optimistic_bound > best or weight > best or optimistic_bound > pessimistic_bound){
                    continue;
                }
                // if (optimistic_bound <= best and weight <= best and optimistic_bound <= pessimistic_bound){
                //     cout << "putaa" << endl;
                pq.push({weight, pessimistic_bound, optimistic_bound, r, n.p+1, aux_path, aux_sol});
                // }
                
            }
        }
    }

    auto end = clock();
    sol = best_sol;
    cout << "-------best value-------" << endl << best << endl;
    cout << "-------best sol-------" << endl;
    for(uint j = 0; j < sol.size(); j++){
        cout << sol[j] <<   " ";
    }
    cout << endl;
    cout << ((1.0*(end-start))/CLOCKS_PER_SEC) << endl;
}

int main(){
    // vector<Point> points = {{0,0},{1,1},{2,2},{3,3},{4,4},{5,5}};
    // vector<Point> robots = {{0,0},{0,0},{1,2},{5,3},{1,1},{3,3}};

    vector<Point> robots = {{0,0},{1, 0},{2, 0},{3,0},{4, 0},{5, 0},{0,1},{0,2},{0,3},{0,4},{0,5}};
    vector<Point> points = {{0,0},{1,0.6},{1.4,0.6},{1.8,0.6},{2.4,0.6},{2.8,0.6},{3.2,0.6},{3.8,0.6},{4.2,0.6},{4.6,0.6},{5,0.6}};
    vector<bool> path(robots.size());
    vector<vector<double>> m(points.size(), vector<double>(robots.size()));
    vector<uint> sol(robots.size()-1);
    vector<double> m_opt;

    create_matrix(robots, points, m, path, m_opt);
    bb_algorithm(m, path, sol, m_opt);

    // cout << "-------matrix-------" << endl;
    // for(int i = 0; i < m.size(); i++){
    //     for(int j = 0; j < m[i].size(); j++){
    //         cout << setw(10) << m[i][j];
    //     }
    //     cout << endl;
    // }

    // cout << "-------m_opt-------"<< endl;
    // for(int j = 0; j < m_opt.size(); j++){
    //     cout << m_opt[j] <<   " ";
    // }
    // cout << endl;
    


    return 0;
}