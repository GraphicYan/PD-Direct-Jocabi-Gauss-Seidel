#include "Cloth.h"
#include "FastBox.h"
#include "Sphere.h"
#include "Plane.h"
#include "Renderer.h"

int main() {
    Cloth cloth(40, 0.01f, 4000.0f, 2000.0f);
    FastBox fastbox(cloth);
    Sphere sphere(Eigen::Vector3f(25.0, -25.0,25.0), 20.0f);
    Plane ground(-50.0f);
    Renderer renderer(1200, 900, cloth, sphere, ground);
    while (!glfwWindowShouldClose(renderer.GetWindow())){
        cloth.Run(1);
        fastbox.PlaneBoxCollision(cloth, ground, true);
        cloth.PlaneCollision(fastbox, ground);
        fastbox.SphereBoxCollision(cloth, sphere, true);
        cloth.SphereCollision(fastbox, sphere);
        //fastbox.SelfBoxCollision(cloth);
        //cloth.SelfCollision(fastbox);
        cloth.ComputeNormals();
        renderer.Render(cloth, sphere, ground);
    }
    return 0;
}


