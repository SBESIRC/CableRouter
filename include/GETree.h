#include "CR_CGALUtils.h"

#ifndef _CABLEROUTER_TREE_H_
#define _CABLEROUTER_TREE_H_

namespace CableRouter
{
	typedef rbush::Bbox Bbox;

	struct GENode
	{
		int			parent;		// parent in tree
		vector<int>	children;	// children list
		Point		coord;		// position
		int			mass;		// devices num
		double		weight;		// weight of edge (parent -> this)
	};

	struct GETree
	{
		vector<GENode>	nodes;

		GETree(vector<Point>& coords, vector<int>& mass, double** G);

		vector<int> prim(double** G, int n, int start);
	};

	int	count(vector<GENode>& tree, int root);
	void breaking(vector<GENode>& tree, int now);
	void reconnect(vector<GENode>& tree, int parent, int now);

	Bbox treebox(vector<GENode>& tree, int root);
	int count_in_box(vector<GENode>& tree, int root, Bbox box);
	bool point_in_box(Point p, Bbox box);
	Polygon convex(vector<GENode>& tree, int root);

	double meanlen(vector<GENode>& tree, int root);
	double cohesion(vector<GENode>& tree, int root, double mean);

	vector<int> get_points(vector<GENode>& tree, int root);
}

#endif