#pragma once
#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace plvins::simulation {
inline std::vector<std::vector<double>> rows(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) throw std::runtime_error("Cannot open simulation file: " + filename);
    std::vector<std::vector<double>> result;
    std::string line;
    while (std::getline(file,line)) {
        auto first=line.find_first_not_of(" \t\r");
        if (first==std::string::npos || line[first]=='#') continue;
        std::istringstream stream(line); std::vector<double> values; double value;
        while (stream>>value) {
            if (!std::isfinite(value)) throw std::runtime_error("Nonfinite simulation value: " + filename);
            values.push_back(value);
        }
        if (!stream.eof()) throw std::runtime_error("Malformed simulation row: " + filename);
        result.push_back(std::move(values));
    }
    return result;
}
inline void LoadPose(const std::string& filename,std::vector<double>& timestamps,
                     std::vector<Eigen::Vector3d>& gyros,std::vector<Eigen::Vector3d>& accelerations) {
    for (const auto& row:rows(filename)) {
        if (row.size()!=14 || row[0]<0. || (!timestamps.empty() && row[0]<=timestamps.back()))
            throw std::runtime_error("Expected increasing time,qw,qx,qy,qz,px,py,pz,wx,wy,wz,ax,ay,az rows: " + filename);
        timestamps.push_back(row[0]);gyros.emplace_back(row[8],row[9],row[10]);accelerations.emplace_back(row[11],row[12],row[13]);
    }
    if (timestamps.empty()) throw std::runtime_error("Empty simulation pose file: " + filename);
}
inline void LoadPointObs(const std::string& filename,std::vector<Eigen::Vector2d>& observations) {
    for (const auto& row:rows(filename)) {
        // Legacy generators produce either XYZ,UV or XYZ,W,UV; normalized-only rows also work.
        if (row.size()!=2 && row.size()!=5 && row.size()!=6)
            throw std::runtime_error("Expected UV, XYZUV or XYZWUV point row: " + filename);
        observations.emplace_back(row[row.size()-2],row.back());
    }
}
inline void LoadLineObs(const std::string& filename,std::vector<Eigen::Vector4d>& observations) {
    for (const auto& row:rows(filename)) {
        if (row.size()!=4) throw std::runtime_error("Expected start_x,start_y,end_x,end_y line row: " + filename);
        observations.emplace_back(row[0],row[1],row[2],row[3]);
    }
}
}
