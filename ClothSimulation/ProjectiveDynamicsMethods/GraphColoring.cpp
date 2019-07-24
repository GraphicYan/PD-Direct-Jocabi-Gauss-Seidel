#include "GraphColoring.h"
#include <fstream>

vector<vector<int>> randomizedGraphColoring(Eigen::MatrixXf& A) {
	int N = A.rows();
	set<int>* mNeighbours = new set<int>[N];

	int* mColors = new int[N];

	set<int>* mPalettes = new set<int>[N];
	set<int> U;

	//find neighbours
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < N; ++j) {
			if (i != j && A(i, j) != 0) {
				if (mNeighbours[i].find(j) == mNeighbours[i].end())
					mNeighbours[i].insert(j);
				if (mNeighbours[j].find(i) == mNeighbours[j].end())
					mNeighbours[j].insert(i);
			}
		}
	}

	vector<int> delta_v(N);
	//initial palettes
	for (int i = 0; i < N; ++i) {
		delta_v[i] = mNeighbours[i].size() / SHRINKING;
		for (int c = 0; c < delta_v[i]; ++c) {
			mPalettes[i].insert(c);
		}
	}

	for (int i = 0; i < N; ++i) {
		U.insert(i);
	}

	while (!U.empty()) {
		for (int i : U) {
			if (!mPalettes[i].size())
				mPalettes[i].insert(delta_v[i]++);
			srand((int)time(0));
			int m = rand() % mPalettes[i].size();
			auto it = mPalettes[i].begin();
			advance(it, m);
			mColors[i] = *it;
		}

		set<int> I;

		for (int i : U) {
			int iColor = mColors[i];
			bool differentFromNeighs = true;
			for (auto neighbour : mNeighbours[i]) {
				if (mColors[neighbour] == iColor) {
					differentFromNeighs = false;
					break;
				}
			}

			if (differentFromNeighs) {
				for (int neighbour : mNeighbours[i]) {
					mPalettes[neighbour].erase(iColor);
				}
			}
			else {
				I.insert(i);
			}
		}

		U = I;
	}
	int maxColors = INT_MIN;
	for (int i = 0; i < N; ++i) {
		if (maxColors < mColors[i])
			maxColors = mColors[i];
	}


	vector<vector<int>> partitions;
	for (int c = 0; c < maxColors; ++c) {
		vector<int> partition;
		for (int i = 0; i < N; ++i) {
			if (mColors[i] == c) {
				partition.push_back(i);
			}
		}
		partitions.push_back(partition);
	}

	return partitions;
}

vector<vector<int>> randomizedGraphColoring(Eigen::MatrixXf& A, Eigen::VectorXi& indices, Eigen::VectorXf& nodes) {
	int N = A.rows();
	set<int>* mNeighbours = new set<int>[N];

	int* mColors = new int[N];

	set<int>* mPalettes = new set<int>[N];
	set<int> U;

	//find neighbours
	for (int i = 0; i < N; ++i) {
		for (int j = 0; j < N; ++j) {
			if (i != j && A(i, j) != 0) {
				if (mNeighbours[i].find(j) == mNeighbours[i].end())
					mNeighbours[i].insert(j);
				if (mNeighbours[j].find(i) == mNeighbours[j].end())
					mNeighbours[j].insert(i);
			}
		}
	}

	int maxNeibour = 0;
	for (int i = 0; i < N; ++i) {
		if (maxNeibour < mNeighbours[i].size())
			maxNeibour = mNeighbours[i].size();
	}
	int maxColors = maxNeibour / SHRINKING;

	vector<int> delta(N);

	//initial palettes
	for (int i = 0; i < N; ++i) {
		delta[i] = mNeighbours[i].size() / SHRINKING;
		for (int c = 0; c <= delta[i]; ++c) {
			mPalettes[i].insert(c);
		}
	}

	for (int i = 0; i < N; ++i) {
		U.insert(i);
	}

	while (!U.empty()) {
		for (int i : U) {
			srand((int)time(0));
			if (mPalettes[i].empty()) {
				mPalettes[i].insert(++delta[i]);
			}
			int m = rand() % mPalettes[i].size();
			auto it = mPalettes[i].begin();
			advance(it, m);
			mColors[i] = *it;

		}

		set<int> I;

		for (int i : U) {
			int iColor = mColors[i];
			bool differentFromNeighs = true;
			for (auto neighbour : mNeighbours[i]) {
				if (mColors[neighbour] == iColor && i<neighbour) {
					differentFromNeighs = false;
					break;
				}
			}

			if (differentFromNeighs) {
				for (int neighbour : mNeighbours[i]) {
					mPalettes[neighbour].erase(iColor);
				}
			}
			else {
				I.insert(i);
			}
		}

		U = I;
	}

	vector<vector<int>> partitions;
	for (int c = 0; c < maxColors; ++c) {
		vector<int> partition;
		for (int i = 0; i < N; ++i) {
			if (mColors[i] == c) {
				partition.push_back(i);
			}
		}
		partitions.push_back(partition);
	}
#if 0
	auto mapJet = [](float v, float vmin, float vmax, float& dr, float& dg, float& db)
	{
		if (v < vmin) {
			v = vmin;
		}

		if (v > vmax) {
			v = vmax;
		}

		v -= vmin;
		v /= (vmax - vmin);

		if (v < 0.1242) {
			db = 0.504 + ((1. - 0.504) / 0.1242) * v;
			dg = dr = 0.;
		}
		else if (v < 0.3747) {
			db = 1.;
			dr = 0.;
			dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
		}
		else if (v < 0.6253) {
			db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
			dg = 1.;
			dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
		}
		else if (v < 0.8758) {
			db = 0.;
			dr = 1.;
			dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
		}
		else {
			db = 0.;
			dg = 0.;
			dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
		}
	};

	ofstream ofout("D:\\color.obj",ios::out);
	for (int i = 0; i < N; i+=3) {
		ofout << "v " << nodes(i) << ' ' << nodes(i + 1) << ' ' << nodes(i + 2) << ' ';
		float r, g, b;
		mapJet(mColors[i], 0, maxColors-1, r, g, b);
		ofout << r << ' ' << g << ' ' << b << endl;
	}

	cout << "ÑÕÉ«ÊýÁ¿£º" << maxColors << endl;
	for (int i = 0; i < indices.size(); i+=3) {
		ofout << "f " << indices(i)+1 << ' ' << indices(i+1)+1 << ' ' << indices(i+2)+1 << endl;
	}
	ofout.close();
#endif
	return partitions;
}