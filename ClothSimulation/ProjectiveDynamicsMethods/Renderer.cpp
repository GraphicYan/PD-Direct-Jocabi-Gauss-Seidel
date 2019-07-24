#include "Renderer.h"
#include "Cloth.h"
#include "Sphere.h"
#include "Plane.h"
#include<sstream>
#include <iostream>

float Renderer::lastFrame;
float Renderer::deltaTime;
float Renderer::lastX;
float Renderer::lastY;
GLFWwindow* Renderer::window;
Shader* Renderer::lightShader;
Shader* Renderer::depthShader;
Camera*  Renderer::camera;
int Renderer::screenWidth, Renderer::screenHeight;
unsigned int Renderer::clothEBO, Renderer::clothVAO, Renderer::clothPositionVBO, Renderer::clothNormalVBO;
unsigned int Renderer::sphereEBO, Renderer::sphereVAO, Renderer::spherePositionVBO, Renderer::sphereNormalVBO;
unsigned int Renderer::groundEBO, Renderer::groundVAO, Renderer::groundPositionVBO, Renderer::groundNormalVBO;
unsigned int Renderer::depthMap, Renderer::depthMapFBO;
bool Renderer::keys[1024];
bool Renderer::cameraRotEnabled;

float FPSdeltaTime=0.0f;
Renderer::Renderer(int screenWidth, int screenHeight, Cloth& cloth, Sphere& sphere, Plane& ground) {
    camera = new Camera(Eigen::Vector3f(-40.00f, 20.00f, 80.00f));
    this->screenWidth = screenWidth;
    this->screenHeight = screenHeight;
    this->lastX = screenWidth / 2.0;
    this->lastY = screenHeight / 2.0;
    cameraRotEnabled = false;
	deltaTime = 0.0f;
    lastFrame = 0.0f;
    camera->SetLightSpaceMatrix(-1.0f, 1.0f, -1.0f, 1.0f, 1.0f, 60.0f);
    std::cout << "Starting GLFW context, OpenGL 3.3" << std::endl;

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	//std::ostringstream frame;
	//frame << 1.0f / (FPSdeltaTime + 0.000001f);
	//std::string str = frame.str();
	//str += "FPS.";
	//str = "Cloth Simulation :" + str;
	//const char* p = str.data();


    window = glfwCreateWindow(screenWidth, screenHeight,"Cloth Simulation " , nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, mouseMove_callback);
    glfwSetMouseButtonCallback(window, mouseButton_callback);
    glfwSetScrollCallback(window, scroll_callback);
    if (gl3wInit()) 
        fprintf(stderr, "failed to initialize OpenGL\n");

    if (!gl3wIsSupported(3, 3)) 
        fprintf(stderr, "OpenGL 3.3 not supported\n");
    printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION),
        glGetString(GL_SHADING_LANGUAGE_VERSION));

    lightShader = new Shader("light.vert", "light.frag");
    depthShader = new Shader("depth.vert", "depth.frag");
    //Ground Mesh Initialization 
    glGenBuffers(1, &groundNormalVBO);
    glGenVertexArrays(1, &groundVAO);
    glGenBuffers(1, &groundEBO);
    glGenBuffers(1, &groundPositionVBO);
    glBindVertexArray(groundVAO);
    glBindBuffer(GL_ARRAY_BUFFER, groundPositionVBO);
    glBufferData(GL_ARRAY_BUFFER, ground.nodes.size() * sizeof(ground.nodes[0]), ground.nodes.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, groundNormalVBO);
    glBufferData(GL_ARRAY_BUFFER, ground.normals.size() * sizeof(ground.normals[0]), ground.normals.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, groundEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ground.indices.size() * sizeof(ground.indices[0]), ground.indices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, groundPositionVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(ground.nodes[0]), (GLvoid*)0);
    glBindBuffer(GL_ARRAY_BUFFER, groundNormalVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(ground.normals[0]), (GLvoid*)0);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    //Sphere Mesh Initialization 
    glGenBuffers(1, &sphereNormalVBO);
    glGenVertexArrays(1, &sphereVAO);
    glGenBuffers(1, &sphereEBO);
    glGenBuffers(1, &spherePositionVBO);
    glBindVertexArray(sphereVAO);
    glBindBuffer(GL_ARRAY_BUFFER, spherePositionVBO);
    glBufferData(GL_ARRAY_BUFFER, sphere.nodes.size() * sizeof(sphere.nodes[0]), sphere.nodes.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, sphereNormalVBO);
    glBufferData(GL_ARRAY_BUFFER, sphere.normals.size() * sizeof(sphere.normals[0]), sphere.normals.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphere.indices.size() * sizeof(sphere.indices[0]), sphere.indices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, spherePositionVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(sphere.nodes[0]), (GLvoid*)0);
    glBindBuffer(GL_ARRAY_BUFFER, sphereNormalVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(sphere.normals[0]), (GLvoid*)0);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Cloth Mesh Initialization 
    glGenBuffers(1, &clothNormalVBO);
    glGenVertexArrays(1, &clothVAO);
    glGenBuffers(1, &clothEBO);
    glGenBuffers(1, &clothPositionVBO);
    glBindVertexArray(clothVAO);
    glBindBuffer(GL_ARRAY_BUFFER, clothPositionVBO);
    glBufferData(GL_ARRAY_BUFFER, cloth.nodes.size() * sizeof(cloth.nodes[0]), cloth.nodes.data(), GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, clothNormalVBO);
    glBufferData(GL_ARRAY_BUFFER, cloth.normals.size() * sizeof(cloth.normals[0]), cloth.normals.data(), GL_DYNAMIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, clothEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, cloth.indices.size() * sizeof(cloth.indices[0]), cloth.indices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, clothPositionVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(cloth.nodes[0]), (GLvoid*)0);
    glBindBuffer(GL_ARRAY_BUFFER, clothNormalVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(cloth.normals[0]), (GLvoid*)0);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    // Depth Map 
    glGenFramebuffers(1, &depthMapFBO);
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, screenWidth, screenWidth, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, screenWidth, screenHeight);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
}

void Renderer::Render(Cloth& cloth, Sphere& sphere, Plane& ground) {
    GLfloat currentFrame = glfwGetTime();
    FPSdeltaTime = currentFrame - lastFrame;
	deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
	std::cout << 1.0f / FPSdeltaTime << "FPS" << std::endl;
    glfwPollEvents();
    do_movement();

    glBindBuffer(GL_ARRAY_BUFFER, clothPositionVBO);
    void *ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(ptr, cloth.nodes.data(), cloth.nodes.size() * sizeof(cloth.nodes[0]));
    glUnmapBuffer(GL_ARRAY_BUFFER);

    glBindBuffer(GL_ARRAY_BUFFER, clothNormalVBO);
    ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    memcpy(ptr, cloth.normals.data(), cloth.normals.size() * sizeof(cloth.normals[0]));
    glUnmapBuffer(GL_ARRAY_BUFFER);

    camera->UpdateViewMatrix();
    camera->SetPerspectiveMatrix((float)screenWidth / (float)screenHeight, 0.1f, 100000.0f);

    glClearColor(0.1f, 0.1f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glCullFace(GL_FRONT);

    glViewport(0, 0, screenWidth, screenWidth);
    depthShader->use();
    depthShader->setMat4("lightSpaceMatrix", camera->lightSpaceMatrix);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glClear(GL_DEPTH_BUFFER_BIT);

    glBindVertexArray(clothVAO);
    glDrawElements(GL_TRIANGLES, (unsigned int)cloth.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glBindVertexArray(sphereVAO);
    glDrawElements(GL_TRIANGLE_STRIP, (unsigned int)sphere.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glBindVertexArray(groundVAO);
    glDrawElements(GL_TRIANGLES, (unsigned int)ground.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glCullFace(GL_BACK);

    glViewport(0, 0, screenWidth, screenHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    lightShader->use();
    lightShader->setMat4("view", camera->viewMatrix);
    lightShader->setMat4("projection", camera->perspectiveMatrix);
    lightShader->setMat4("lightSpaceMatrix", camera->lightSpaceMatrix);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depthMap);

    lightShader->setVec3("matColor", Eigen::Vector3f(0.9f, 0.9f, 0.9f));
    glBindVertexArray(clothVAO);
    glDrawElements(GL_TRIANGLES, (unsigned int)cloth.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    
    lightShader->setVec3("matColor", Eigen::Vector3f(0.8f, 0.0f, 0.0f));
    glBindVertexArray(sphereVAO);
    glDrawElements(GL_TRIANGLE_STRIP, (unsigned int)sphere.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    lightShader->setVec3("matColor", Eigen::Vector3f(0.0f, 0.2f, 0.5f));
    glBindVertexArray(groundVAO);
    glDrawElements(GL_TRIANGLES, (unsigned int)ground.indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glfwSwapBuffers(window);
}

void Renderer::key_callback(GLFWwindow* window, int key, int scancode, int action, int mode){
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
    if (key >= 0 && key < 1024)
    {
        if (action == GLFW_PRESS)
            keys[key] = true;
        else if (action == GLFW_RELEASE)
            keys[key] = false;
    }
}

void Renderer::do_movement(){
    if (keys[GLFW_KEY_W])
        camera->ProcessKeyboard(FORWARD, deltaTime*10.2f);
    if (keys[GLFW_KEY_S])
        camera->ProcessKeyboard(BACKWARD, deltaTime*10.2f);
    if (keys[GLFW_KEY_A])
        camera->ProcessKeyboard(LEFT, deltaTime*10.2f);
    if (keys[GLFW_KEY_D])
        camera->ProcessKeyboard(RIGHT, deltaTime*10.2f);
}

bool firstMouse = true;
void Renderer::mouseMove_callback(GLFWwindow* window, double xpos, double ypos){
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }
    GLfloat xoffset = xpos - lastX;
    GLfloat yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;
    if (cameraRotEnabled)
        camera->ProcessMouseMovement(xoffset, yoffset);
}

void Renderer::mouseButton_callback(GLFWwindow * window, int button, int action, int mods){
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
        cameraRotEnabled = true;
    else if (button == GLFW_RELEASE) {
        cameraRotEnabled = false;
    }
}

void Renderer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset){
    camera->ProcessMouseScroll(yoffset);
}
void Renderer::Terminate() {
    glDeleteVertexArrays(1, &clothVAO);
    glDeleteBuffers(1, &clothPositionVBO);
    glDeleteBuffers(1, &clothEBO);
    glfwTerminate();
    delete lightShader;
    delete depthShader;
    delete camera;
}
