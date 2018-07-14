//
// Created by tsbertalan on 7/8/18.
//
#include "logger.h"
using namespace std;

PyLogger::PyLogger(CoordinateTransformer &transform, string filePath) : transform(transform) {
    pyfile = ofstream("data.py");
    pyfile.precision(10);
    logging = true;

    pyfile << "data = {}" << std::endl;

    begin_item("map");

    vector<vector<double>> waypoints = transform.get_waypoints();
    (*this)("map_x", waypoints[0]);
    (*this)("map_y", waypoints[1]);
    (*this)("map_dx", waypoints[2]);
    (*this)("map_dy", waypoints[3]);
    (*this)("map_s", waypoints[4]);

    vector<vector<vector<double>>> lines;
    double offset = -12;
    for(int i=0; i<7; i++) {
        if(i==3) {
            offset += 4;
            continue;
        }
        vector<vector<double>> line;
        for(double s : waypoints[4]){
            WorldPose p1 = transform.to_world(FrenetPose({.s=s, .d=offset, .yaw=0}));
            line.push_back({p1.x, p1.y});
        }
        lines.push_back(line);
        offset += 4;
    }
    (*this)("map_lines", lines);
    end_item();
}

void PyLogger::begin_item(long tag) {
    if(!logging) return;
    pyfile << "data[" << tag << "] = dict(" << endl;
}

void PyLogger::begin_item(std::string tag) {
    if(!logging) return;
    pyfile << "data['" << tag << "'] = dict(" << endl;
}

void PyLogger::end_item() {
    if(!logging) return;
    pyfile << ")" << endl;
}

void PyLogger::operator()(std::string name, std::string data) {
    if(!logging) return;
    pyfile << name << " = '" << data << "'," << endl;
}

void PyLogger::operator()(std::string name, std::string desc, Trajectory tj) {
    if(!logging) return;

    pyfile << name << " = dict(" << endl;
    
    pyfile << "x" << " = [";
    for (WorldPose p : tj.poses)
    pyfile << p.x << ",";
    pyfile << "]," << endl;

    pyfile << "y" << " = [";
    for (WorldPose p : tj.poses)
    pyfile << p.y << ",";
    pyfile << "]," << endl;

    pyfile << "t" << " = [";
    for (double t : tj.times)
    pyfile << t << ",";
    pyfile << "]," << endl;

    pyfile << "s" << " = [";
    for (WorldPose p : tj.poses) {
    FrenetPose fp = transform.to_frenet(p);
    pyfile << fp.s << ",";
    }
    pyfile << "]," << endl;

    pyfile << "d" << " = [";
    for (WorldPose p : tj.poses) {
    FrenetPose fp = transform.to_frenet(p);
    pyfile << fp.d << ",";
    }
    pyfile << "]," << endl;

    (*this)("desc", desc);

    pyfile << ")," << endl;
}

void PyLogger::operator()(std::string name, std::vector<Neighbor> neighbors, double tproj) {
    if(!logging) return;
    pyfile << name << " = [";
    for (auto n : neighbors) {
        WorldPose p = n.current_wp;
        pyfile << "(" << n.id << "," << p.x << "," << p.y << "," << n.vx << "," << n.vy << ",";
        if(tproj > 0) {
            WorldPose fp = n.future_position(tproj);
            pyfile << fp.x << "," << fp.y << ",";
        }
        pyfile << "),";
    }
    pyfile << "]," << endl;
}

void PyLogger::operator()(std::string name, std::vector<double> vec) {
    if(!logging) return;
    pyfile << name << " = [";
    for (double x : vec) {
        pyfile << x << ",";
    }
    pyfile << "]," << endl;
}

void PyLogger::operator()(std::string name, std::vector<std::vector<double>> arr) {
    if(!logging) return;

    pyfile << name << " = [";
    for (vector<double> vec : arr) {
        pyfile << "[";
        for (double x : vec) {
            pyfile << x << ",";
        }
        pyfile << "], ";
    }
    pyfile << "]," << endl;
}

void PyLogger::operator()(std::string name, std::vector<std::vector<std::vector<double>>> arr) {
    if(!logging) return;
    pyfile << name << " = [";
    for (vector<vector<double>> sheet : arr) {
        pyfile << "[";
        for (vector<double> vec : sheet ) {
            pyfile << "[";
            for (double x : vec) {
                pyfile << x << ",";
            }
            pyfile << "], ";
        }
        pyfile << "]," << endl;
    }
    pyfile << "]," << endl;
}

void PyLogger::operator()(std::string name, long int data) {
    if(!logging) return;
    pyfile << name << " = " << data << "," << endl;
}


void PyLogger::operator()(std::string name, int data) {
    if(!logging) return;
    pyfile << name << " = " << data << "," << endl;
}


void PyLogger::operator()(std::string name, double data) {
    if(!logging) return;
    pyfile << name << " = " << data << "," << endl;
}

bool PyLogger::set_status(bool new_status) {
    bool old_status = logging;
    logging = new_status;
    return old_status;
}

