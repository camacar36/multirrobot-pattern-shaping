#include <iostream>
#include <vector>
#include <chrono>
#include <climits>
#include <queue>
#include <cmath>
#include <iomanip>  // para el wset()

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <geometry_msgs/TransformStamped.h>


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

// void get_points(const geometry_msgs::Polygon &msg){
//     points_vec = msg;
// }

void get_robots(vector<Point> &robots, int n_points){
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    robots.push_back({0,0});
    for(int i = 1; i < n_points; i++){
        bool done = false;
        tf::StampedTransform transform;

        while (!done){            
            try{
                listener.lookupTransform("/map", "/robot" + to_string(i) + "_tf/base_footprint", ros::Time(0), transform);
                done = true;
            }
            catch (tf::TransformException ex){
            }
        }   
        robots.push_back({transform.getOrigin().getX(), transform.getOrigin().getY()});
    }
    
}

void extract_points(geometry_msgs::PolygonConstPtr vec, vector<Point> &p){
    p.push_back({0,0});
    for(int i = 0; i < vec->points.size(); i++){
        p.push_back({vec->points[i].x, vec->points[i].y});
    }
}

void bb_algorithm(const vector<vector<double>> &m, vector<bool> &path, vector<uint> &sol, const vector<double> &m_opt){
    auto start = clock();
    // calcular pessimistic bound con greedy e igualar al mejor valor actual (bast_val)
    double pessimistic_bound = greedy(m, path, 0);

    // calcular optimistic bound
    double optimistic_bound = m_opt[0];

    // mejor valor hasta ahora es la cota pesimista inicial
    double best = INT_MAX;
    vector<uint> best_sol;

    // crear la cola de prioridad
    priority_queue <Node, vector<Node>, is_worse> pq;
    pq.push({0, pessimistic_bound, optimistic_bound, 0, 0, path, {}});

    // bucle principal
    while(!pq.empty()){

        Node n = pq.top();
        pq.pop();     

        // comprobar si es un nodo hoja
        if(is_leaf(n.p, m.size() - 1)){
            if(n.current_weight <= best){
                best = n.current_weight;
                best_sol = n.sol;
            }
            continue;
        }

        

        // expandir hijos
        for(uint r = 1; r < m.size(); r++){
            
            // comprobar que es factible
            if(is_feasible(n.path[r])){
                
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
                    best = pessimistic_bound;
                }
                
                if(optimistic_bound > best or weight > best or optimistic_bound > pessimistic_bound){
                    continue;
                }

                pq.push({weight, pessimistic_bound, optimistic_bound, r, n.p+1, aux_path, aux_sol});
                
            }
        }
    }

    auto end = clock();
    sol = best_sol;
    cout << "-------best value-------" << endl << best << endl;
    cout << "-------best sol-------" << endl;
    for(uint j = 0; j < sol.size(); j++){
        cout << sol[j] << " ";
    }
    cout << endl;
    cout << ((1.0*(end-start))/CLOCKS_PER_SEC) << endl;
}

int main(int argc, char **argv){
    // vector<Point> points = {{0,0},{1,1},{2,2},{3,3},{4,4},{5,5}};
    // vector<Point> robots = {{0,0},{0,0},{1,2},{5,3},{1,1},{3,3}};
    ros::init(argc, argv, "bb_alg");
    ros::NodeHandle n;
    geometry_msgs::PolygonConstPtr points_vec;
    ros::Publisher pub = n.advertise<std_msgs::UInt16MultiArray>("/final_goal_points", 5);
    

    vector<Point> robots; // = {{0,0},{1, 0},{2, 0},{3,0},{4, 0},{5, 0},{0,1},{0,2},{0,3},{0,4},{0,5}};
    vector<Point> points; // = {{0,0},{1,0.6},{1.4,0.6},{1.8,0.6},{2.4,0.6},{2.8,0.6},{3.2,0.6},{3.8,0.6},{4.2,0.6},{4.6,0.6},{5,0.6}};

   
    while(ros::ok){
        if(points.size() == 0){
            points_vec = ros::topic::waitForMessage<geometry_msgs::Polygon>("/goal_points");
            extract_points(points_vec, points);
            
            get_robots(robots, points.size());
            // cout << "len_points: " << points.size() << " len_points_vec: " << points_vec->points.size() << " len_robots: " << robots.size() << endl;

            for(int i = 0; i < robots.size(); i++){
                cout << "x: " << robots[i].x << ", y: " << robots[i].y << endl;
            }
        }
        else{
            vector<bool> path(robots.size());
            vector<vector<double>> m(points.size(), vector<double>(robots.size()));
            vector<uint> sol(robots.size()-1);
            vector<double> m_opt;

            create_matrix(robots, points, m, path, m_opt);
            
            bb_algorithm(m, path, sol, m_opt);
            std_msgs::UInt16MultiArray sol_array = std_msgs::UInt16MultiArray();
            if(sol.size() > 0){
                

                for(int i = 0; i < sol.size(); i++){

                    sol_array.data.push_back(sol[i]);
                }

                
            }
            else{
                for(int i = 0; i < 10; i++){

                    sol_array.data.push_back(i+1);
                }
            }
            pub.publish(sol_array);


            points.clear();
            robots.clear();
        }
        ros::spinOnce();
    }

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