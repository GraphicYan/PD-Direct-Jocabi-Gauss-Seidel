#include <omp.h>
#include "Cloth.h"
#include <ctime>
#include <vector>
#include <iostream>
#include <vector>
#include<fstream>

Cloth::Cloth(unsigned int nNodes, float nodeMass, float shearStretchK, float bendK, float timeStep, float damp) {
    this->nodeMass = nodeMass;
    this->timeStep = timeStep;
	this->damping = damp;
    snodeCount = nNodes;
    nodeCount = nNodes * nNodes;
    springCount = (2 * nNodes - 2) * (2 * nNodes - 1) +(2 * nNodes - 4)*nNodes;
    y.resize(nodeCount * 3);
    normals.resize(nodeCount * 3);
    indices.resize((snodeCount - 1)*(snodeCount - 1) * 2 * 3);
    triPointConstraints.resize(nodeCount, indices.size() / 3);

    NodeInit();
    SpringInit();
    IndicesInit();
    LMatrixBuild(shearStretchK, bendK);
    JMatrixBuild(shearStretchK, bendK);
    MMatrixBuild(nodeMass);
    Mph2LMatrixBuild();
    SetExternalForce();
}

const int IterationType = 2; // 0 for direct // 1 for Jocabi // 2 for Gauss-Seidel
const int IterationCnt = 48;
void Cloth::Run(unsigned int iterations) {
    y =  nodes + damping * (nodes - previousNodes); // y = qn + h*vn
    previousNodes = nodes;
	Eigen::VectorXf* eachIter = new Eigen::VectorXf[IterationCnt+1];
	eachIter[0] = nodes;
    for (unsigned int i = 1; i <= iterations || i <= IterationCnt; i++) {
        LocalStep();
        GlobalStep();
		for (unsigned int it = 0; it < 1; ++it) {
			if(IterationType==1)
				Jocabi();
			else if(IterationType==2)
				GaussSeidel();
		}
		eachIter[i] = nodes;
    }

#if 0
	vector<float> relativeError(IterationCnt);
	float Denominator = (eachIter[0] - eachIter[IterationCnt]).squaredNorm();
	ofstream ofs("D:\\errors.txt", ios::out);
	for (unsigned int i = 1; i <= iterations || i <= IterationCnt; i++)
	{
		relativeError[i-1] = ((eachIter[i] - eachIter[IterationCnt]).squaredNorm()) / Denominator;
		ofs << i << ' ' << relativeError[i - 1] << endl;
	}
	ofs.close();
	
	nodes = eachIter[0];
	for (unsigned int i = 1; i <= iterations || i <= IterationCnt; i++) {
		LocalStep();
		GlobalStepCmp();
	}
	ofstream ofs2("D:\\CompareErrors.txt", ios::out);
	Denominator = (eachIter[0] - nodes).squaredNorm();
	for (unsigned int i = 1; i <= iterations || i <= IterationCnt; i++)
	{
		relativeError[i - 1] = ((eachIter[i] - nodes).squaredNorm()) / Denominator;
		ofs2 << i << ' ' << relativeError[i - 1] << endl;
	}
	ofs2.close();
#endif

}

inline void Cloth::LocalStep() {
    for (unsigned int i = 0; i < springCount; i++) {
        unsigned int p1 = springs[i * 2];
        unsigned int p2 = springs[i * 2 + 1];
        Eigen::Vector3f p12(nodes[p1 * 3]     - nodes[p2 * 3],
                            nodes[p1 * 3 + 1] - nodes[p2 * 3 + 1],
                            nodes[p1 * 3 + 2] - nodes[p2 * 3 + 2]);
        p12.normalize();
        springsDirection[i * 3] = p12[0] * springsRestLength[i];
        springsDirection[i * 3 + 1] = p12[1] * springsRestLength[i];
        springsDirection[i * 3 + 2] = p12[2] * springsRestLength[i];
    }
}

inline void Cloth::GlobalStep() {
	Eigen::VectorXf tmp(nodes);
    Eigen::VectorXf h2JdpMy = mMatrix * y + timeStep * timeStep * jMatrix * springsDirection; // M*y + (h^2)*J*d
    h2JdpMy += timeStep * timeStep * fext; // M*y + (h^2)*J*d + (h^2)*f_ext 
	if (!IterationType) {
		nodes = Mph2L.solve(h2JdpMy);
		nodes(0) = tmp(0); nodes(1) = tmp(1); nodes(2) = tmp(2);

	}
	b = h2JdpMy;
}

inline void Cloth::GlobalStepCmp() {
	Eigen::VectorXf tmp(nodes);
	Eigen::VectorXf h2JdpMy = mMatrix * y + timeStep * timeStep * jMatrix * springsDirection; // M*y + (h^2)*J*d
	h2JdpMy += timeStep * timeStep * fext; // M*y + (h^2)*J*d + (h^2)*f_ext 
	nodes = Mph2L.solve(h2JdpMy);
	nodes(0) = tmp(0); nodes(1) = tmp(1); nodes(2) = tmp(2);
	b = h2JdpMy;
}

inline void Cloth::Jocabi() {
	Eigen::VectorXf newNodes(b);
	Eigen::VectorXf aii(3*nodeCount);
	
	for (int k = 0; k < A_Sparse.outerSize();++k)
		for (Eigen::SparseMatrix<float>::InnerIterator it(A_Sparse, k); it; ++it)
		{
			int i = it.row(), j = it.col();
			if (i == j) {
				aii(i) = it.value();
				//std::cout << i << ' ' << j << ' '<<aij<<std::endl;
			}
			else{
				newNodes(i) -= it.value() * nodes(j);
				//std::cout << it.value()<< std::endl;
			}
		}
	for (int i = 0; i < aii.size(); ++i) {
		newNodes(i) /= aii(i);
	}
	newNodes(0) = nodes(0); newNodes(1) = nodes(1); newNodes(2) = nodes(2);
	nodes = newNodes;	
}

inline void Cloth::GaussSeidel() {
	Eigen::VectorXf newNodes(b);
	
	Eigen::VectorXf aii(3 * nodeCount);
	std::vector<std::vector<int>>  neigh(3 * nodeCount);
	std::vector<std::vector<float>> aij(3 * nodeCount);

	for (int k = 0; k < A_Sparse.outerSize(); ++k)
		for (Eigen::SparseMatrix<float>::InnerIterator it(A_Sparse, k); it; ++it)
		{
			int i = it.row(), j = it.col();
			if (i == j) {
				aii(i) = it.value();
				//std::cout << i << ' ' << j << ' '<<aij<<std::endl;
			}
			else {
				//newNodes(i) -= it.value() * nodes(j);
				neigh[i].push_back(j);
				aij[i].push_back(it.value());
				//std::cout << it.value()<< std::endl;
			}
		}
	float omega = 1.0f;
	for (auto p : Partitions)
		for (int i : p) {
			for (int neiIdx = 0; neiIdx < neigh[i].size(); ++neiIdx) {
				newNodes(i) -= aij[i][neiIdx] * nodes(neigh[i][neiIdx]);
			}
			if(i&&i!=1&&i!=2)
				nodes(i) = (omega * (newNodes(i) / aii(i)) - nodes(i)) + nodes(i);
	}
	
}

void Cloth::NodeInit() {
    nodes.resize(nodeCount * 3);
    previousNodes.resize(nodeCount * 3);
    for (unsigned int j = 0; j < snodeCount; j++)
        for (unsigned int i = 0; i < snodeCount; i++) {
            unsigned int index = (snodeCount * j + i) * 3;
            nodes[index] = (float)i;
            nodes[index + 2] = (float)j+5.0f;
            nodes[index + 1] = 10.0f;
        }
    previousNodes = nodes;
}

void Cloth::SpringInit() {
    springs.resize(springCount * 2);
    springsRestLength.resize(springCount);
    springsDirection.resize(springCount * 3);
    unsigned int sIndex = 0; //spring index
    for (unsigned int i = 0; i < snodeCount; i++)
        for (unsigned int j = 0; j < snodeCount; j++) {
            unsigned int nIndex = (snodeCount * i + j); // node index
            if (j > 0 && i < (snodeCount - 1)) {
                springs[sIndex] = nIndex;
                springs[sIndex + 1] = nIndex + snodeCount - 1;
                sIndex += 2;
            }
            if (i < (snodeCount - 1)) {
                springs[sIndex] = nIndex;
                springs[sIndex + 1] = nIndex + snodeCount;
                sIndex += 2;
            }
            if (j < (snodeCount - 1) && i < (snodeCount - 1)) {
                springs[sIndex] = nIndex;
                springs[sIndex + 1] = nIndex + snodeCount + 1;
                sIndex += 2;
            }
            if (j < (snodeCount - 1)) {
                springs[sIndex] = nIndex;
                springs[sIndex + 1] = nIndex + 1;
                sIndex += 2;
            }
        }
    for (unsigned int i = 0; i < snodeCount; i++)
        for (unsigned int j = 0; j < snodeCount; j++) {
            unsigned int nIndex = (snodeCount * i + j); // node index
            if (i < (snodeCount - 2)) {
                springs[sIndex] = nIndex;
                springs[sIndex + 1] = nIndex + snodeCount * 2;
                sIndex += 2;
            }
            if (j < (snodeCount - 2)) {
                springs[sIndex] = nIndex;
                springs[sIndex + 1] = nIndex + 2;
                sIndex += 2;
            }      
        }
    float one = 1.0f;
    float two = 2.0f;
    float sqrt2 = sqrtf(2.0f);
    for (unsigned int i = 0; i < springCount; i++) {
        unsigned int index = i * 2;
        unsigned int neigh = abs(springs[index] - springs[index + 1]);
        if (neigh == 1 || neigh == snodeCount)
            springsRestLength[i] = one;
        else if (neigh == 2 || neigh == (snodeCount*2))
            springsRestLength[i] = two;
        else
            springsRestLength[i] = sqrt2;
    }
}

void Cloth::LMatrixBuild(float shearStretchK, float bendK) {
    unsigned int beforeBend = (2 * snodeCount - 2) * (2 * snodeCount - 1);

    for (unsigned int i = 0; i < springCount; i++) {
        unsigned int index = i * 2;
        for (unsigned int j = 0; j < 3; j++) {
            if (i < beforeBend) {
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index] + j, 3 * springs[index] + j, shearStretchK));
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index + 1] + j, 3 * springs[index + 1] + j, shearStretchK));
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index + 1] + j, 3 * springs[index] + j, -shearStretchK));
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index] + j, 3 * springs[index + 1] + j, -shearStretchK));
            }
            else {
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index] + j, 3 * springs[index] + j, bendK));
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index + 1] + j, 3 * springs[index + 1] + j, bendK));
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index + 1] + j, 3 * springs[index] + j, -bendK));
                lCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index] + j, 3 * springs[index + 1] + j, -bendK));           
            }
        }
    }
    lMatrix.resize(nodeCount * 3, nodeCount * 3);
    lMatrix.setFromTriplets(lCoefficients.begin(), lCoefficients.end());
    lMatrix.makeCompressed();
}

void Cloth::JMatrixBuild(float shearStretchK, float bendK) {
    unsigned int beforeBend = (2 * snodeCount - 2) * (2 * snodeCount - 1);
    std::vector< Eigen::Triplet<float> > jCoefficients;
    for (unsigned int i = 0; i < springCount; i ++) {
        for (unsigned int j = 0; j < 3; j++) {
            unsigned int index = i * 2;
            if (i < beforeBend) {
                jCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index] + j, 3 * i + j, shearStretchK));
                jCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index + 1] + j, 3 * i + j, -shearStretchK));
            }
            else {
                jCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index] + j, 3 * i + j, bendK));
                jCoefficients.push_back(Eigen::Triplet<float>(3 * springs[index + 1] + j, 3 * i + j, -bendK));
            }
        }
    }
    jMatrix.resize(nodeCount * 3, springCount * 3);
    jMatrix.setFromTriplets(jCoefficients.begin(), jCoefficients.end());
    jMatrix.makeCompressed();
}

void Cloth::MMatrixBuild(float nodeMass) {
    mMatrix.resize(nodeCount * 3, nodeCount * 3);
    //mMatrix.setIdentity();
    std::vector< Eigen::Triplet<float> > mCoefficients;
    for (unsigned int i = 0; i < nodeCount * 3; i++)
        mCoefficients.push_back(Eigen::Triplet<float>(i, i, 1));
    //mCoefficients.push_back(Eigen::Triplet<float>((nodeCount - 1) * 3, (nodeCount - 1) * 3, 999999900));
    //mCoefficients.push_back(Eigen::Triplet<float>((nodeCount - 1) * 3 + 1, (nodeCount - 1) * 3 + 1, 999999900));
    //mCoefficients.push_back(Eigen::Triplet<float>((nodeCount - 1) * 3 + 2, (nodeCount - 1) * 3 + 2, 999999900));

    //mCoefficients.push_back(Eigen::Triplet<float>((nodeCount - snodeCount) * 3, (nodeCount - snodeCount) * 3, 999999900));
    //mCoefficients.push_back(Eigen::Triplet<float>((nodeCount - snodeCount) * 3 + 1, (nodeCount - snodeCount) * 3 + 1, 999999900));
    //mCoefficients.push_back(Eigen::Triplet<float>((nodeCount - snodeCount) * 3 + 2, (nodeCount - snodeCount) * 3 + 2, 999999900));
    mMatrix *= nodeMass;
    mMatrix.setFromTriplets(mCoefficients.begin(), mCoefficients.end());
    mMatrix.makeCompressed();
}

void Cloth::Mph2LMatrixBuild() {
    Eigen::SparseMatrix<float> h2L = timeStep * timeStep* lMatrix;
    Mph2L.compute(mMatrix + h2L);
	if (Mph2L.info() != Eigen::Success)
		std::cerr << "Mph2LMatrixBuild ERROR!" << std::endl; 

	A_Sparse = (mMatrix + h2L);
	if (IterationType == 2) {
		A_Dense = A_Sparse.toDense();
		Partitions = randomizedGraphColoring(A_Dense
		//);
		 , indices, nodes);
	}
}

void Cloth::IndicesInit() {
    for (unsigned int j = 0; j < (snodeCount - 1); j++)
        for (unsigned int i = 0; i < (snodeCount - 1); i++) {
            unsigned int index = ((snodeCount - 1) * j + i) * 6;
            indices[index] = i + j * snodeCount;
            indices[index + 1] = i + j * snodeCount + 1;
            indices[index + 2] = i + (j + 1)*snodeCount;
            indices[index + 3] = i + (j + 1)*snodeCount;
            indices[index + 4] = i + j * snodeCount + 1;
            indices[index + 5] = i + (j + 1)*snodeCount + 1;
        }
}

void Cloth::ComputeNormals() {
    normals = Eigen::VectorXf::Zero(nodeCount * 3);
    unsigned int i = 0;
    //#pragma oemp parallel for num_threads(2)
    for (i = 0; i < indices.size(); i += 3) {
        Eigen::Vector3f a(nodes[indices[i] * 3], nodes[indices[i] * 3 + 1], nodes[indices[i] * 3 + 2]);
        Eigen::Vector3f b(nodes[indices[i + 1] * 3], nodes[indices[i + 1] * 3 + 1], nodes[indices[i + 1] * 3 + 2]);
        Eigen::Vector3f c(nodes[indices[i + 2] * 3], nodes[indices[i + 2] * 3 + 1], nodes[indices[i + 2] * 3 + 2]);
        Eigen::Vector3f a2b = b - a;
        Eigen::Vector3f a2c = c - a;
        Eigen::Vector3f triNormal = a2b.cross(a2c);

        normals[indices[i] * 3] += triNormal[0];
        normals[indices[i] * 3 + 1] += triNormal[1];
        normals[indices[i] * 3 + 2] += triNormal[2];
        normals[indices[i + 1] * 3] += triNormal[0];
        normals[indices[i + 1] * 3 + 1] += triNormal[1];
        normals[indices[i + 1] * 3 + 2] += triNormal[2];
        normals[indices[i + 2] * 3] += triNormal[0];
        normals[indices[i + 2] * 3 + 1] += triNormal[1];
        normals[indices[i + 2] * 3 + 2] += triNormal[2];
    }
}

void Cloth::SphereCollision(FastBox& fastbox, Sphere& sphere) {
    for (unsigned int i = 0; i < fastbox.Intersections.size(); i += 2) {
        unsigned int nodeInd = (fastbox.Intersections[i] - &fastbox.pointsNObj[0]) * 3;
        Eigen::Vector3f nodePos(nodes[nodeInd], nodes[nodeInd + 1], nodes[nodeInd + 2]);
        Eigen::Vector3f dir = nodePos - sphere.center;
        float dist = dir.norm();
        if (dist < (sphere.radius + 0.1f)) {
            dir.normalize();
            Eigen::Vector3f newPos = sphere.center + dir * (sphere.radius + 0.1f);
            nodes[nodeInd] = newPos[0];
            nodes[nodeInd + 1] = newPos[1];
            nodes[nodeInd + 2] = newPos[2];

            previousNodes[nodeInd ] += (nodes[nodeInd ] - previousNodes[nodeInd ])*0.1f;
            previousNodes[nodeInd + 1] += (nodes[nodeInd + 1] - previousNodes[nodeInd + 1])*0.1f;
            previousNodes[nodeInd + 2] += (nodes[nodeInd + 2] - previousNodes[nodeInd + 2])*0.1f;
        }
    }
}

void Cloth::PlaneCollision(FastBox& fastbox, Plane& ground) {
    for (unsigned int i = 0; i < fastbox.Intersections.size(); i += 2) {
        unsigned int nodeInd = (fastbox.Intersections[i] - &fastbox.pointsNObj[0])*3;
        Eigen::Vector3f nodePos(nodes[nodeInd], nodes[nodeInd + 1], nodes[nodeInd + 2]);
        if (nodePos[1] < ground.height + 0.1) {
            nodes[nodeInd + 1] = ground.height + 0.1;
            previousNodes[nodeInd + 1] = (nodes[nodeInd + 1] + previousNodes[nodeInd + 1])*0.5;
        }
    }
}

bool Cloth::TrianglePointProj(unsigned int triInd, unsigned int nodeInd, float& u, float& v) {
    Eigen::Vector3f a(nodes(indices[triInd] * 3), nodes(indices[triInd] * 3 + 1), nodes(indices[triInd] * 3 + 2));
    Eigen::Vector3f b(nodes(indices[triInd + 1] * 3), nodes(indices[triInd + 1] * 3 + 1), nodes(indices[triInd + 1] * 3 + 2));
    Eigen::Vector3f c(nodes(indices[triInd + 2] * 3), nodes(indices[triInd + 2] * 3 + 1), nodes(indices[triInd + 2] * 3 + 2));
    Eigen::Vector3f p(nodes(nodeInd * 3), nodes(nodeInd * 3 + 1), nodes(nodeInd * 3 + 2));
    Eigen::Vector3f a2b = b - a;
    Eigen::Vector3f a2c = c - a;
    Eigen::Vector3f a2p = p - a;
    Eigen::Vector3f n = a2b.cross(a2c);
    n.normalize();
    float a2pDotN = a2p.dot(n);
    float dist;
    float th;
    if (a2pDotN <= 0) {
        dist = -sqrt(fabs(a2pDotN));
        th = -0.3f;
    }
    else {
        dist = sqrt(a2pDotN);
        th = 0.3f;
    }
    if (fabs(dist) > 0.3)
        return false;
    Eigen::Vector3f projpc = p - dist * n;

    a2p = projpc - a;
    float a2cDota2c = a2c.dot(a2c); 
    float a2cDota2b = a2c.dot(a2b);
    float a2cDota2p = a2c.dot(a2p);
    float a2bDota2b = a2b.dot(a2b);
    float a2bDota2p = a2b.dot(a2p);

    float invDenom = 1 / (a2cDota2c * a2bDota2b - a2cDota2b * a2cDota2b);
    u = (a2bDota2b * a2cDota2p - a2cDota2b * a2bDota2p) * invDenom;
    v = (a2cDota2c * a2bDota2p - a2cDota2b * a2cDota2p) * invDenom;

    if (!( (u >= -0.0001) && (v >= -0.0001) && (u + v < 1.0001) ))
        return false;
    float w = 1 - u - v;

    if (triPointConstraints.coeff(nodeInd, triInd / 3) == 0)
        triPointConstraints.insert(nodeInd, triInd / 3) = (int)(a2pDotN / fabs(a2pDotN));
    else if ( (triPointConstraints.coeff(nodeInd, triInd / 3) == -1 && dist > 0) || (triPointConstraints.coeff(nodeInd, triInd / 3) == 1 && dist < 0))
        th *= -1.0;

    nodes[indices[triInd] * 3] -= n[0]*w*(th - dist)*0.5;
    nodes[indices[triInd] * 3 + 1] -= n[1] *w*(th - dist)*0.5;
    nodes[indices[triInd] * 3 + 2] -= n[2] *w*(th - dist)*0.5;
    
    nodes[indices[triInd + 1] * 3] -= n[0] *v*(th - dist)*0.5;
    nodes[indices[triInd + 1] * 3 + 1] -= n[1] *v*(th - dist)*0.5;
    nodes[indices[triInd + 1] * 3 + 2] -= n[2] *v*(th - dist)*0.5;
    
    nodes[indices[triInd + 2] * 3] -= n[0] *u*(th - dist)*0.5;
    nodes[indices[triInd + 2] * 3 + 1] -= n[1] *u*(th - dist)*0.5;
    nodes[indices[triInd + 2] * 3 + 2] -= n[2] *u*(th - dist)*0.5;
    
    nodes[nodeInd * 3] += n[0] * (th - dist)*0.5;
    nodes[nodeInd * 3 + 1] += n[1] * (th - dist)*0.5;
    nodes[nodeInd * 3 + 2] += n[2] * (th - dist)*0.5;
}

bool Cloth::EdgeProj(unsigned int p0, unsigned int p1, unsigned int q0, unsigned int q1) { 
    Eigen::Vector3f S1P1(nodes(p0 * 3), nodes(p0 * 3 + 1), nodes(p0 * 3 + 2));
    Eigen::Vector3f S1P2(nodes(p1 * 3), nodes(p1 * 3 + 1), nodes(p1 * 3 + 2));
    Eigen::Vector3f S2P1(nodes(q0 * 3), nodes(q0 * 3 + 1), nodes(q0 * 3 + 2));
    Eigen::Vector3f S2P2(nodes(q1 * 3), nodes(q1 * 3 + 1), nodes(q1 * 3 + 2));
    Eigen::Vector3f   p0p1 = S1P2 - S1P1;
    Eigen::Vector3f   q0q1 = S2P2 - S2P1;
    Eigen::Vector3f   q0p0 = S1P1 - S2P1;
    float p0p1Dotp0p1 = p0p1.dot(p0p1); 
    float p0p1Dotq0q1 = p0p1.dot(q0q1); 
    float q0q1Dotq0q1 = q0q1.dot(q0q1); 
    float p0p1Dotq0p1 = p0p1.dot(q0p0); 
    float q0q1Dotq0p0 = q0q1.dot(q0p0); 
    float    crossP = p0p1Dotp0p1 * q0q1Dotq0q1 - p0p1Dotq0q1 * p0p1Dotq0q1; 
    float    pu, su, sd = crossP;
    float    qu, tu, td = crossP;
    if (crossP < 0.00001) {
        su = 0.0;
        sd = 1.0;
        tu = q0q1Dotq0p0;
        td = q0q1Dotq0q1;
    }
    else {
        su = (p0p1Dotq0q1*q0q1Dotq0p0 - q0q1Dotq0q1 * p0p1Dotq0p1);
        tu = (p0p1Dotp0p1*q0q1Dotq0p0 - p0p1Dotq0q1 * p0p1Dotq0p1);
        if (su < 0.0) {
            su = 0.0;
            tu = q0q1Dotq0p0;
            td = q0q1Dotq0q1;
        }
        else if (su > sd) {
            su = sd;
            tu = q0q1Dotq0p0 + p0p1Dotq0q1;
            td = q0q1Dotq0q1;
        }
    }
    if (tu < 0.0) {
        tu = 0.0;

        if (-p0p1Dotq0p1 < 0.0)
            su = 0.0;
        else if (-p0p1Dotq0p1 > p0p1Dotp0p1)
            su = sd;
        else {
            su = -p0p1Dotq0p1;
            sd = p0p1Dotp0p1;
        }
    }
    else if (tu > td) {
        tu = td;

        if ((-p0p1Dotq0p1 + p0p1Dotq0q1) < 0.0)
            su = 0;
        else if ((-p0p1Dotq0p1 + p0p1Dotq0q1) > p0p1Dotp0p1)
            su = sd;
        else {
            su = (-p0p1Dotq0p1 + p0p1Dotq0q1);
            sd = p0p1Dotp0p1;
        }
    }
    pu = (abs(su) < 0.0000001 ? 0.0 : su / sd);
    qu = (abs(tu) < 0.0000001 ? 0.0 : tu / td);

    if ((fabs(pu - 1.0f) < 0.01 || fabs(pu) < 0.01) && (fabs(qu - 1.0f) < 0.01 || fabs(qu) < 0.01))
        return false;
    Eigen::Vector3f   dP = q0p0 + (pu * p0p1) - (qu * q0q1);
    float a2pDotN = dP.dot(dP);
    float dist;
    float th;
    if (a2pDotN <= 0) {
        dist = -sqrt(fabs(a2pDotN));
        th = -0.1f;
    }
    else {
        dist = sqrt(a2pDotN);
        th = 0.1f;
    }
    if (fabs(dist) > 0.1)
        return false;

    dP.normalize();
    nodes(p0 * 3)     += (th - dist)*dP[0] *0.5*pu;
    nodes(p0 * 3 + 1) += (th - dist)*dP[1] *0.5*pu;
    nodes(p0 * 3 + 2) += (th - dist)*dP[2] *0.5*pu;
    nodes(p1 * 3)     += (th - dist)*dP[0] *0.5*(1 - pu);
    nodes(p1 * 3 + 1) += (th - dist)*dP[1] *0.5*(1 - pu);
    nodes(p1 * 3 + 2) += (th - dist)*dP[2] *0.5*(1 - pu);
    nodes(q0 * 3)     -= (th - dist)*dP[0] *0.5*qu;
    nodes(q0 * 3 + 1) -= (th - dist)*dP[1] *0.5*qu;
    nodes(q0 * 3 + 2) -= (th - dist)*dP[2] *0.5*qu;
    nodes(q1 * 3)     -= (th - dist)*dP[0] *0.5*(1 - qu);
    nodes(q1 * 3 + 1) -= (th - dist)*dP[1] *0.5*(1 - qu);
    nodes(q1 * 3 + 2) -= (th - dist)*dP[2] *0.5*(1 - qu);
}

void Cloth::SelfCollision(FastBox& fastbox) {
    unsigned int triSize = indices.size() / 3;
    float u, v;
    for (unsigned int i = 0; i < fastbox.Intersections.size(); i += 2) {
        unsigned int triInd = (fastbox.Intersections[i] - &fastbox.tris[0]) * 3;
        unsigned int triInd2 = (fastbox.Intersections[i + 1] - &fastbox.tris[0]) * 3;

        bool neigh = false;
        for (int j = 0; j < 3; j++) {
            if (indices[triInd2 + j] == indices[triInd] || indices[triInd2 + j] == indices[triInd + 1] || indices[triInd2 + j] == indices[triInd + 2]) {
                neigh = true;
                break;
            }
        }
        if (neigh)
            continue;
        for (int j = 0; j < 3; j++) 
            TrianglePointProj(triInd, indices[triInd2 + j], u, v);
        for (int j = 0; j < 3; j++)
            TrianglePointProj(triInd2, indices[triInd + j], u, v);
        for (int j = 0; j < 3; j++) 
            EdgeProj(indices[triInd], indices[triInd+1], indices[triInd2 + j], indices[triInd2 + ((j+1)%3)]);
        for (int j = 0; j < 3; j++) 
            EdgeProj(indices[triInd+1], indices[triInd+2], indices[triInd2 + j], indices[triInd2 + ((j + 1) % 3)]);
        for (int j = 0; j < 3; j++)
            EdgeProj(indices[triInd+2], indices[triInd], indices[triInd2 + j], indices[triInd2 + ((j + 1) % 3)]);
    }
}

void Cloth::SetExternalForce() {
    float force =  10;
    fext.resize(nodeCount * 3);
    fext = Eigen::VectorXf::Zero(nodeCount * 3);
    for (unsigned int i = 0; i < nodeCount; i++) {
		//if (i > nodeCount * 0.9) fext[i * 3] = 10;
        unsigned int index = i * 3 + 1;
        fext[index] = -force;
    }
}

