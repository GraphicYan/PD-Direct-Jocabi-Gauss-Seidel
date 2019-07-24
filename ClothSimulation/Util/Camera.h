#pragma once
#include <vector>
#include <GL/gl3w.h>
#include <Eigen/Dense>

#define DEG2RAD(deg) (deg * 0.01745329251994329576923690768489)

enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

const GLfloat YAW = -43.0f;
const GLfloat PITCH = -29.0f;
const GLfloat SPEED = 3.0f;
const GLfloat SENSITIVTY = 0.25f;
const GLfloat ZOOM = 45.0f;


class Camera
{
public:
    Eigen::Vector3f position;
    Eigen::Vector3f front;
    Eigen::Vector3f up;
    Eigen::Vector3f right;
    Eigen::Vector3f worldUp;
    Eigen::Matrix4f viewMatrix;
    Eigen::Matrix4f perspectiveMatrix;
    Eigen::Matrix4f lightSpaceMatrix;
    GLfloat yaw;
    GLfloat pitch;
    GLfloat movementSpeed;
    GLfloat mouseSensitivity;
    GLfloat zoom;
    Camera(Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
           Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f), 
           GLfloat yaw = YAW, GLfloat pitch = PITCH) : movementSpeed(SPEED), mouseSensitivity(SENSITIVTY), zoom(ZOOM)   {
        this->position = position;
        this->worldUp = up;
        this->yaw = yaw;
        this->pitch = pitch;
        viewMatrix = Eigen::Matrix4f::Zero();
        perspectiveMatrix = Eigen::Matrix4f::Zero();
        lightSpaceMatrix = Eigen::Matrix4f::Zero();
        this->updateCameraVectors();
    }

    void ProcessKeyboard(Camera_Movement direction, GLfloat deltaTime){
        GLfloat velocity = this->movementSpeed * deltaTime;
        if (direction == FORWARD)
            this->position += this->front * velocity;
        if (direction == BACKWARD)
            this->position -= this->front * velocity;
        if (direction == LEFT)
            this->position -= this->right * velocity;
        if (direction == RIGHT)
            this->position += this->right * velocity;
    }

    void ProcessMouseMovement(GLfloat xoffset, GLfloat yoffset, GLboolean constrainPitch = true){
        xoffset *= this->mouseSensitivity;
        yoffset *= this->mouseSensitivity;
        this->yaw += xoffset;
        this->pitch += yoffset;
        if (constrainPitch){
            if (this->pitch > 89.0f)
                this->pitch = 89.0f;
            if (this->pitch < -89.0f)
                this->pitch = -89.0f;
        }
        this->updateCameraVectors();
    }

    void ProcessMouseScroll(GLfloat yoffset){
        if (this->zoom >= 1.0f && this->zoom <= 45.0f)
            this->zoom -= yoffset;
        if (this->zoom <= 1.0f)
            this->zoom = 1.0f;
        if (this->zoom >= 45.0f)
            this->zoom = 45.0f;
    }

    void UpdateViewMatrix(){
        Eigen::Matrix3f R;
        R.col(2) = (this->position - (this->position + this->front)).normalized();
        R.col(0) = this->up.cross(R.col(2)).normalized();
        R.col(1) = R.col(2).cross(R.col(0));
        viewMatrix.topLeftCorner<3, 3>() = R.transpose();
        viewMatrix.topRightCorner<3, 1>() = -R.transpose() * this->position;
        viewMatrix(3, 3) = 1.0f;
    }

    void SetPerspectiveMatrix(float aspect, float nearP, float farP)    {
        float theta = DEG2RAD(this->zoom) * 0.5f;
        float range = farP - nearP;
        float invtan = 1. / tan(theta);

        perspectiveMatrix(0, 0) = invtan / aspect;
        perspectiveMatrix(1, 1) = invtan;
        perspectiveMatrix(2, 2) = -(nearP + farP) / range;
        perspectiveMatrix(3, 2) = -1;
        perspectiveMatrix(2, 3) = -2 * nearP * farP / range;
        perspectiveMatrix(3, 3) = 0;
    }

    void SetLightSpaceMatrix(float left, float right, float bottom, float top, float nearP, float farP) {
        Eigen::Matrix4f lightOrthographicMatrix = Eigen::Matrix4f::Zero();
        lightOrthographicMatrix(0, 0) = 2 / (right - left);
        lightOrthographicMatrix(1, 1) = 2 / (top - bottom);
        lightOrthographicMatrix(2, 2) = -2 / (farP - nearP);
        lightOrthographicMatrix(3, 0) = -(right + left) / (right - left);
        lightOrthographicMatrix(3, 1) = -(top + bottom) / (top - bottom);
        lightOrthographicMatrix(3, 2) = -(farP + nearP) / (farP - nearP);
        lightOrthographicMatrix(3, 3) = 1;
        Eigen::Matrix4f lightViewMatrix = Eigen::Matrix4f::Zero();
        Eigen::Matrix3f R = Eigen::Matrix3f::Zero();
        Eigen::Vector3f up(0, 1, 0);
        Eigen::Vector3f pos(30, 80, 80);
        R.col(2) = Eigen::Vector3f(0, 0.707106781, 0.707106781);;
        R.col(0) = up.cross(R.col(2)).normalized();
        R.col(1) = R.col(2).cross(R.col(0));
        lightViewMatrix.topLeftCorner<3, 3>() = R.transpose();
        lightViewMatrix.topRightCorner<3, 1>() = -R.transpose() *pos;
        lightViewMatrix(3, 3) = 1.0f;
        lightSpaceMatrix = lightOrthographicMatrix * lightViewMatrix;
    }

private:
    void updateCameraVectors(){
        this->front[0] = cos(DEG2RAD(this->yaw)) * cos(DEG2RAD(this->pitch));
        this->front[1] = sin(DEG2RAD(this->pitch));
        this->front[2] = sin(DEG2RAD(this->yaw)) * cos(DEG2RAD(this->pitch));
        this->front.normalize();
        this->right = this->front.cross( this->worldUp );
        this->right.normalize();
        this->up = this->right.cross(this->front);
        this->up.normalize();
    }
};

