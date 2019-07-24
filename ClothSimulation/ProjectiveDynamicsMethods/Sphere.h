#pragma once
#include <Eigen/Dense>
# define PI 3.14159265358979323846
class Sphere {
public:
    Eigen::Vector3f center;
    float radius;
    Eigen::VectorXf nodes;
    Eigen::VectorXi indices;
    Eigen::VectorXf normals;
    Sphere(Eigen::Vector3f coordinate, float radius) {
        center = coordinate;
        this->radius = radius;
        unsigned int lats = 64, longs = 64;
        nodes.resize((lats + 1)*(longs + 1) * 3);
        normals.resize((lats + 1)*(longs + 1) *3);
        indices.resize(lats*(longs + 1) *2);
        for (unsigned int i = 0; i <= lats; i++)
        {
            float ySegment = (float)i / (float)lats;
            float cosy = std::cos(ySegment * PI);
            float siny = std::sin(ySegment * PI);
            for (unsigned int j = 0; j <= longs; j++){
                unsigned int index = ((lats+1) * i + j) * 3;
                float xSegment = (float)j / (float)longs;
                float xPos = std::cos(xSegment * 2.0f * PI) * siny * radius;
                float yPos = cosy * radius;
                float zPos = std::sin(xSegment * 2.0f * PI) * siny * radius;
                nodes[index] = xPos + center[0]; nodes[index + 1] = yPos + center[1]; nodes[index + 2] = zPos + center[2];
                normals[index] = xPos; normals[index + 1] = yPos; normals[index + 2] = zPos;
            }
        }
        int index = 0;
        for (int i = 0; i < lats; i++){
            if (i%2)
                for (int j = longs; j >= 0; j--){
                    indices[index] = i * (longs + 1) + j;
                    indices[index +1] = (i + 1) * (longs + 1) + j;
                    index += 2;
                }
            else    
                for (int j = 0; j <= longs; j++){
                    indices[index] = (i + 1) * (longs + 1) + j;
                    indices[index + 1] = i * (longs + 1) + j;
                    index += 2;
                }     
        }
    }
};