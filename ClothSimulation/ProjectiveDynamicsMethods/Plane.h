#pragma once
#include <Eigen/Dense>
class Plane {
public:
    Eigen::VectorXf nodes;
    Eigen::VectorXi indices;
    Eigen::VectorXf normals;
    float height;
    Plane(float height) {
        this->height = height;
        nodes.resize(4 * 3);
        normals.resize(4 * 3);
        indices.resize(3 * 2);

        nodes[0] = 5000.0f; nodes[1] = height; nodes[2] = 5000.0f;
        nodes[3] = 5000.0f; nodes[4] = height; nodes[5] = -5000.0f;
        nodes[6] = -5000.0f; nodes[7] = height; nodes[8] = -5000.0f;
        nodes[9] = -5000.0f; nodes[10] = height; nodes[11] = 5000.0f;

        normals[0] = 0.0f; normals[1] =1.0f; normals[2] = 0.0f;
        normals[3] = 0.0f; normals[4] =1.0f; normals[5] = 0.0f;
        normals[6] = 0.0f; normals[7] =1.0f; normals[8] = 0.0f;
        normals[9] = 0.0f; normals[10] =1.0f; normals[11] = 0.0f;

        indices[0] = 0; indices[1] = 1; indices[2] = 2;
        indices[3] = 2; indices[4] = 3; indices[5] = 0;
    }
};