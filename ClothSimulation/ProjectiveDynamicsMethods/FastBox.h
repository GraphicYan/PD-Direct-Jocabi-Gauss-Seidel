#pragma once
#include "Cloth.h"
#include "Plane.h"
#include "Sphere.h"

#include <Eigen/Dense>
#include <vector>
#include <ctime>
typedef std::pair<Eigen::Vector3f, Eigen::Vector3f> Box;
typedef std::vector<Box> Boxes;
typedef std::vector<Box*> PBoxes;
typedef PBoxes::iterator it_PBoxes;

class Compare {
public:
    int dim;
    Compare(int argdim): dim(argdim){}
    bool operator() (Box* a, Box* b) const{
        return b->first[dim] > a->first[dim];
    }
};

class FastBox {
friend class Cloth;
friend class Ground;
friend class Sphere;
public:
    FastBox(Cloth& cloth);
    void SelfBoxCollision(Cloth& cloth);
    void SphereBoxCollision(Cloth& cloth, Sphere& sphere, bool update);
    void PlaneBoxCollision(Cloth& cloth, Plane& plane, bool update);
private:
    inline bool LowLowCompare(it_PBoxes a, it_PBoxes b, int dim);
    inline bool LowLowCompareInv(it_PBoxes a, it_PBoxes b, int dim);
    inline bool LowHighCompare(it_PBoxes a, it_PBoxes b, int dim);
    inline bool ContainsLow(it_PBoxes a, it_PBoxes b, int dim);
    inline bool DoesIntersect(it_PBoxes a, it_PBoxes b, int dim);
    void OneWayScan(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, it_PBoxes pointsBegin, it_PBoxes pointsEnd, int dim);
    void ModifiedTwoWayScan(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, it_PBoxes pointsBegin, it_PBoxes pointsEnd, int dim);
    inline bool DoesSpan(it_PBoxes a, float low, float high, int dim);
    it_PBoxes SpanInterval(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, float low, float high, int dim);
    it_PBoxes OverlapsL(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, float middle, int dim);
    it_PBoxes OverlapsH(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, float middle, int dim);
    it_PBoxes MedianOfThree(it_PBoxes a, it_PBoxes b, it_PBoxes c, int dim);
    it_PBoxes Median(it_PBoxes pointsBegin, it_PBoxes pointsEnd, int dim, int h);
    inline int Height(int n);
    void Stream(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, it_PBoxes pointsBegin, it_PBoxes pointsEnd, float low, float high, int dim);
    void UpdateTriBoxes(Cloth& cloth);
    void UpdateNodeBoxes(Cloth& cloth);
    void ChangeObjToSphere(Sphere& sphere);
    void ChangeObjToPlane(Plane& plane);
    Boxes pointsNObj, tris;
    PBoxes ppointsNObj, ptris;
    PBoxes Intersections;   
    float cutoff;
};