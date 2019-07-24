#pragma once
#include <GL/gl3w.h>
#include <GLFW/glfw3.h>
#include "../Util/Camera.h"
#include "../Util/Shader.h"
class Cloth;
class Sphere;
class Plane;
class Renderer {

public:
    Renderer(int screenWidth, int screenHeight, Cloth& cloth, Sphere& sphere, Plane& ground);
    void Render(Cloth& cloth, Sphere& sphere, Plane& ground);
    void UpdateNormal(Cloth& cloth);
    void Terminate();
    GLFWwindow* GetWindow() { return window;}

private:
    static void do_movement();
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
    static void mouseMove_callback(GLFWwindow* window, double xpos, double ypos);
    static void mouseButton_callback(GLFWwindow* window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static bool keys[1024];
    static float lastX, lastY;
    static bool cameraRotEnabled;

    static GLFWwindow* window;
    static Shader* lightShader;
    static Shader* depthShader;
    static Camera*  camera;
    static int screenWidth, screenHeight;

    static float deltaTime;
    static float lastFrame;

    static unsigned int clothEBO, clothVAO, clothPositionVBO, clothNormalVBO;
    static unsigned int sphereEBO, sphereVAO, spherePositionVBO, sphereNormalVBO;
    static unsigned int groundEBO, groundVAO, groundPositionVBO, groundNormalVBO;
    static unsigned int depthMap, depthMapFBO;
};
