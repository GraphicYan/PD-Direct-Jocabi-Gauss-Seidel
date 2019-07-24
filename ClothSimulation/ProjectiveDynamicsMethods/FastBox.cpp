#include "FastBox.h"
#include <iostream>

FastBox::FastBox(Cloth& cloth) {
    unsigned int triSize = cloth.indices.size() / 3;
    tris.resize(triSize);
    ptris.resize(triSize*2);
    pointsNObj.resize(cloth.nodeCount + 1);
    ppointsNObj.resize(cloth.nodeCount + 1);
    for (int i = 0; i < ptris.size(); i++){
        ptris[i] = &tris[i%triSize];
    }
    for (int i = 0; i < pointsNObj.size(); i++) {
        ppointsNObj[i] = &pointsNObj[i];
    }
    srand(time(NULL));
}


void FastBox::UpdateTriBoxes(Cloth& cloth) {
    Eigen::VectorXf& v = cloth.nodes;
    Eigen::VectorXi& in = cloth.indices;
    unsigned int triSize = cloth.indices.size() / 3;
    for (unsigned int i = 0; i < triSize; i++) {
        unsigned int index = i * 3;
        tris[i].first[0] = std::min(std::min(v[in[index] * 3], v[in[index + 1] * 3]), cloth.nodes[in[index + 2] * 3]) - 0.02;
        tris[i].first[1] = std::min(std::min(v[in[index] * 3 + 1], v[in[index + 1] * 3 + 1]), cloth.nodes[in[index + 2] * 3 + 1]) - 0.02;
        tris[i].first[2] = std::min(std::min(v[in[index] * 3 + 2], v[in[index + 1] * 3 + 2]), cloth.nodes[in[index + 2] * 3 + 2]) - 0.02;

        tris[i].second[0] = std::max(std::max(v[in[index] * 3], v[in[index + 1] * 3]), cloth.nodes[in[index + 2] * 3]) + 0.2;
        tris[i].second[1] = std::max(std::max(v[in[index] * 3 + 1], v[in[index + 1] * 3 + 1]), cloth.nodes[in[index + 2] * 3 + 1]) + 0.02;
        tris[i].second[2] = std::max(std::max(v[in[index] * 3 + 2], v[in[index + 1] * 3 + 2]), cloth.nodes[in[index + 2] * 3 + 2]) + 0.02;
    }
}

void FastBox::UpdateNodeBoxes(Cloth& cloth) {
    for (unsigned int i = 0; i < cloth.nodeCount; i++) {
        unsigned int index = i * 3;
        pointsNObj[i].first[0] = cloth.nodes[index] - 0.2;
        pointsNObj[i].first[1] = cloth.nodes[index + 1] - 0.2;
        pointsNObj[i].first[2] = cloth.nodes[index + 2] - 0.2;

        pointsNObj[i].second[0] = cloth.nodes[index] + 0.2;
        pointsNObj[i].second[1] = cloth.nodes[index + 1] + 0.2;
        pointsNObj[i].second[2] = cloth.nodes[index + 2] + 0.2;
    }
}

void FastBox::ChangeObjToSphere(Sphere& sphere) {
    unsigned int index = pointsNObj.size() - 1;
    pointsNObj[index].first[0] = sphere.center[0] - sphere.radius - 0.2;
    pointsNObj[index].first[1] = sphere.center[1] - sphere.radius - 0.2;
    pointsNObj[index].first[2] = sphere.center[2] - sphere.radius - 0.2;

    pointsNObj[index].second[0] = sphere.center[0] + sphere.radius + 0.2;
    pointsNObj[index].second[1] = sphere.center[1] + sphere.radius + 0.2;
    pointsNObj[index].second[2] = sphere.center[2] + sphere.radius + 0.2;
}

void FastBox::ChangeObjToPlane(Plane& ground) {
    unsigned int index = pointsNObj.size() - 1;
    pointsNObj[index].first[0] = -999;
    pointsNObj[index].first[1] = ground.height - 0.2;
    pointsNObj[index].first[2] = -999;

    pointsNObj[index].second[0] = 999;
    pointsNObj[index].second[1] = ground.height + 0.2;
    pointsNObj[index].second[2] = 999;
}

inline bool FastBox::LowLowCompare(it_PBoxes a, it_PBoxes b, int dim) {
    return (*a)->first[dim] < (*b)->first[dim] || ((*a)->first[dim] == (*b)->first[dim] && a < b);
}

inline bool FastBox::LowLowCompareInv(it_PBoxes a, it_PBoxes b, int dim) {
    return (*a)->first[dim] < (*b)->first[dim] || ((*a)->first[dim] == (*b)->first[dim] && b < a);
}

inline bool FastBox::LowHighCompare(it_PBoxes a, it_PBoxes b, int dim) {
    return (*a)->first[dim] < (*b)->second[dim] || ((*a)->first[dim] == (*b)->second[dim] && a < b);
}

inline bool FastBox::ContainsLow(it_PBoxes a, it_PBoxes b, int dim) {
    return ( (*a)->first[dim] < (*b)->first[dim] || ((*a)->first[dim] == (*b)->first[dim] && a < b) ) &&
           (*a)->first[dim] <= (*b)->second[dim];
}

inline bool FastBox::DoesIntersect(it_PBoxes a, it_PBoxes b, int dim) {
    bool intersects = true;
    for (; dim >= 0; dim--) {
        intersects = intersects && LowHighCompare(a, b, dim) && LowHighCompare(b, a, dim);
    }
    return intersects;
}

void FastBox::OneWayScan(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, it_PBoxes pointsBegin, it_PBoxes pointsEnd, int dim) {
    std::sort(pointsBegin, pointsEnd, Compare(0));
    std::sort(intervalsBegin, intervalsEnd, Compare(0));
    for (it_PBoxes iti = intervalsBegin; iti != intervalsEnd; iti++) {
        for (; pointsBegin != pointsEnd && LowLowCompare(pointsBegin, iti, 0); ++pointsBegin) {}
        for (it_PBoxes itp = pointsBegin; itp != pointsEnd && LowHighCompare(itp, iti, 0); itp++) {
            if (DoesIntersect(itp, iti, dim)) {
                if (iti > itp) {
                    Intersections.push_back(*itp);
                    Intersections.push_back(*iti);
                }
                else {
                    Intersections.push_back(*iti);
                    Intersections.push_back(*itp);
                }
            }
        }
    }
}
void FastBox::ModifiedTwoWayScan(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, it_PBoxes pointsBegin, it_PBoxes pointsEnd, int dim) {
    std::sort(pointsBegin, pointsEnd, Compare(0));
    std::sort(intervalsBegin, intervalsEnd, Compare(0));

    while (intervalsBegin != intervalsEnd && pointsBegin != pointsEnd) {
        if (LowLowCompareInv(intervalsBegin, pointsBegin, 0)) {
            for (it_PBoxes itp = pointsBegin; itp != pointsEnd && LowHighCompare(itp, intervalsBegin, 0); itp++) {
                if (DoesIntersect(itp, intervalsBegin, dim) && ContainsLow(intervalsBegin, itp, dim)) {
                    if (intervalsBegin > itp) {
                        Intersections.push_back(*itp);
                        Intersections.push_back(*intervalsBegin);
                    }
                    else {
                        Intersections.push_back(*intervalsBegin);
                        Intersections.push_back(*itp);
                    }
                }
            }
            intervalsBegin++;
        }
        else {
            for (it_PBoxes iti = intervalsBegin; iti != intervalsEnd && LowHighCompare(iti, pointsBegin, 0); iti++)
                if (DoesIntersect(pointsBegin, iti, dim) && ContainsLow(iti, pointsBegin, dim)) {
                    if (iti > pointsBegin) {
                        Intersections.push_back(*pointsBegin);
                        Intersections.push_back(*iti);
                    }
                    else {
                        Intersections.push_back(*iti);
                        Intersections.push_back(*pointsBegin);
                    }
                }
            pointsBegin++;
        }   
    }
}

inline bool FastBox::DoesSpan(it_PBoxes a, float low, float high, int dim) {
    return (*a)->first[dim] <= low && (*a)->second[dim] > high;
}

it_PBoxes FastBox::SpanInterval(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, float low, float high, int dim) {
    it_PBoxes i, j;
    Box* k;
    for (i = intervalsBegin, j = intervalsEnd; i < j;) {
        while (i < j && DoesSpan(i, low, high, dim)) i++;
        do { j--; } while (i < j && !DoesSpan(i, low, high, dim));
        if (j > i) {
            k = *i;
            *i = *j;
            *j = k;
        }
    }
    return i;
}

it_PBoxes FastBox::OverlapsL(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, float middle, int dim) {
    it_PBoxes i, j;
    Box* k;
    for (i = intervalsBegin, j = intervalsEnd; i < j;) {
        while (i < j && (*i)->first[dim] < middle)
            i++;
        do { j--; } while (i < j && (*j)->first[dim] >= middle);
        if (j > i) {
            k = *i;
            *i = *j;
            *j = k;
        }
    }
    return i;
}

it_PBoxes FastBox::OverlapsH(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, float middle, int dim) {
    it_PBoxes i, j;
    Box* k;
    for (i = intervalsBegin, j = intervalsEnd; i < j;) {
        while (i < j && (*i)->second[dim] >= middle)
            i++;
        do { j--; } while (i < j && (*j)->second[dim] < middle);
        if (j > i) {
            k = *i;
            *i = *j;
            *j = k;
        }
    }
    return i;
}

it_PBoxes FastBox::MedianOfThree(it_PBoxes a, it_PBoxes b, it_PBoxes c, int dim) {
    if ((*a)->first[dim] <= (*b)->first[dim]) {
        if ((*b)->first[dim] <= (*c)->first[dim])
            return b;
        else if ((*a)->first[dim] <= (*c)->first[dim])
            return c;
        else
            return a;
    }
    else if ((*a)->first[dim] <= (*c)->first[dim])
        return a;
    else if ((*b)->first[dim] <= (*c)->first[dim])
        return c;
    else
        return b;
}

it_PBoxes FastBox::Median(it_PBoxes pointsBegin, it_PBoxes pointsEnd, int dim, int h) {
    if (h == 0) {
        int random = rand();
        it_PBoxes ret = pointsBegin + (random % std::distance(pointsBegin, pointsEnd));
        return ret;
    }
    else
        return MedianOfThree( Median(pointsBegin, pointsEnd, dim, h - 1),
                              Median(pointsBegin, pointsEnd, dim, h - 1),
                              Median(pointsBegin, pointsEnd, dim, h - 1), dim);

}

inline int FastBox::Height(int n) {
    return (int)(1.0f + 0.5f * log((float)n) / 1.0986122); //log_3 n
}

void FastBox::Stream(it_PBoxes intervalsBegin, it_PBoxes intervalsEnd, it_PBoxes pointsBegin, it_PBoxes pointsEnd, float low, float high, int dim) {
    if (intervalsBegin >= intervalsEnd || pointsBegin >= pointsEnd || low >= high)
        return;
    else if (dim == 0) {
        OneWayScan(intervalsBegin, intervalsEnd, pointsBegin, pointsEnd, dim);
        return;
    }
    else if (std::distance(intervalsBegin, intervalsEnd) < cutoff || std::distance(pointsBegin, pointsEnd) < cutoff) {
        ModifiedTwoWayScan(intervalsBegin, intervalsEnd, pointsBegin, pointsEnd, dim);
        return;
    }
    else {
        it_PBoxes im = SpanInterval(intervalsBegin, intervalsEnd, low, high, dim);
        if (im != intervalsBegin) {
            Stream(intervalsBegin, im, pointsBegin, pointsEnd, -FLT_MAX, FLT_MAX, dim - 1);
            Stream(pointsBegin, pointsEnd, intervalsBegin, im, -FLT_MAX, FLT_MAX, dim - 1);
        }
    
        it_PBoxes median = Median(pointsBegin, pointsEnd, dim, Height(std::distance(pointsBegin, pointsEnd)));
        float middle = (*median)->first[dim];
        it_PBoxes pointsMiddle = OverlapsL(pointsBegin, pointsEnd, middle, dim);
        if (pointsMiddle == pointsBegin || pointsMiddle == pointsEnd) {
            ModifiedTwoWayScan(im, intervalsEnd, pointsBegin, pointsEnd, dim);
            return;
        }
        it_PBoxes il = OverlapsL(im, intervalsEnd, middle, dim);
        Stream(im, il, pointsBegin, pointsMiddle, low, middle, dim);
        it_PBoxes ir = OverlapsH(im, intervalsEnd, middle, dim);
        Stream(im, ir, pointsMiddle, pointsEnd, middle, high, dim);
    }   
}

void FastBox::SelfBoxCollision(Cloth& cloth) {
    unsigned int triSize = cloth.indices.size() / 3;
    cutoff = sqrt(triSize);
    UpdateTriBoxes(cloth);
    Intersections.clear();
    Stream(ptris.begin(), ptris.begin() + triSize, ptris.begin() + triSize, ptris.end(), -FLT_MAX, FLT_MAX, 2);
}

void FastBox::SphereBoxCollision(Cloth& cloth, Sphere& sphere, bool update) {
    cutoff = 1;
    if (update)
        UpdateNodeBoxes(cloth);
    ChangeObjToSphere(sphere);
    Intersections.clear();
    Stream(ppointsNObj.begin(), ppointsNObj.end() -1, ppointsNObj.end() - 1, ppointsNObj.end(), -FLT_MAX, FLT_MAX, 2);
    Stream(ppointsNObj.end() - 1, ppointsNObj.end(), ppointsNObj.begin(), ppointsNObj.end() -1,  -FLT_MAX, FLT_MAX, 2);
}

void FastBox::PlaneBoxCollision(Cloth& cloth, Plane& ground, bool update) {
    cutoff = 1;
    if(update)
        UpdateNodeBoxes(cloth);
    ChangeObjToPlane(ground);
    Intersections.clear();
    Stream(ppointsNObj.begin(), ppointsNObj.end() - 1, ppointsNObj.end() - 1, ppointsNObj.end(), -FLT_MAX, FLT_MAX, 2);
    Stream(ppointsNObj.end() - 1, ppointsNObj.end(), ppointsNObj.begin(), ppointsNObj.end() - 1, -FLT_MAX, FLT_MAX, 2);
}