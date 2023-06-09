#include "GroupEngine.h"
#include <queue>

using namespace CableRouter;


vector<vector<int>> CableRouter::GroupEngine::grouping(MapInfo* const data, const GroupParam* param)
{
	getParam(param);

	if (data->devices.size() <= param->min_dev_size)
	{
		vector<int> pts;
		for (int i = 0; i < data->devices.size(); i++)
		{
			pts.push_back(i);
		}
		partition_result.push_back(pts);
		return partition_result;
	}

	CDTP dt = buildTriangulation(data);
	int n_all = (int)(dt.number_of_vertices() + data->powers.size());

	double** G = buildGraphAll(data, dt, n_all, true, true);
	addDeviceEdges(data, G, true, true);
	removeObstacles(data, G, n_all);
	adjustByLayoutType(data, G);

	// construct mst
	printf("MST construct begin\n");
	vector<Point> dev_pts(data->devices.size());
	vector<int> dev_mass(data->devices.size());
	for (int i = 0; i < data->devices.size(); i++)
	{
		dev_pts[data->devices[i].id] = data->devices[i].coord;
		dev_mass[data->devices[i].id] = data->devices[i].mass;
	}
	GETree mst(dev_pts, dev_mass, G);
	printf("MST construct over\n");

	// divide tree
	printf("Partition begin\n");
	reset(partition_result);
	divide(mst.nodes, 0);
	printf("Partition end\n");

	deleteGraph(G, n_all);
	dt.clear();
	return partition_result;
}


void CableRouter::GroupEngine::getParam(const GroupParam* param)
{
	dev_min = param->min_dev_size;
	dev_max = param->max_dev_size;
	w1 = param->weight_pos;
	w2 = param->weight_even;
	w3 = param->weight_cohesion;
	w4 = param->weight_big;
	w5 = param->weight_cut_len;
}


void CableRouter::GroupEngine::divide(vector<GENode>& tree, int root)
{
	int num = count(tree, root);
	cout << "Tree size = " << num << endl;
	if (dev_min <= num && num <= dev_max)
	{
		vector<int> pts = get_points(tree, root);
		partition_result.push_back(pts);
		return;
	}

	std::queue<int> q;
	for (int i = 0; i < tree[root].children.size(); i++)
	{
		q.push(tree[root].children[i]);
	}
	double MAX = -1;
	int best = -1;
	while (!q.empty())
	{
		int now = q.front();
		q.pop();

		int parent = tree[now].parent;
		breaking(tree, now);
		double w = evaluate(tree, root, now);
		reconnect(tree, parent, now);

		if (w > MAX) {
			MAX = w;
			best = now;
		}

		for (int i = 0; i < tree[now].children.size(); i++)
		{
			q.push(tree[now].children[i]);
		}
	}
	// divide
	if (best == -1 || MAX < 0) {
		reset(q);
		for (int i = 0; i < tree[root].children.size(); i++)
		{
			q.push(tree[root].children[i]);
		}
		int MIN = num + 1;
		best = -1;
		while (!q.empty())
		{
			int now = q.front();
			q.pop();

			int parent = tree[now].parent;
			breaking(tree, now);
			int c = abs(num - 2 * count(tree, root));
			reconnect(tree, parent, now);

			if (c < MIN) {
				MIN = c;
				best = now;
			}

			for (int i = 0; i < tree[now].children.size(); i++)
			{
				q.push(tree[now].children[i]);
			}
		}		
		if (best == -1) return;
		breaking(tree, best);
		vector<int> pts = get_points(tree, root);
		partition_result.push_back(pts);
		pts = get_points(tree, best);
		partition_result.push_back(pts);
		return;
	}
	std::cout << "Best w = " << MAX << std::endl;
	breaking(tree, best);
	divide(tree, best);
	divide(tree, root);
}

double CableRouter::GroupEngine::evaluate(vector<GENode>& tree, int root, int now)
{

	int size1 = count(tree, root);
	int size2 = count(tree, now);

	// factor 0
	if (size1 < dev_min || size2 < dev_min)
	{
		return -10;
	}

	double f1, f2, f3, f4, f5;

	// factor 1
	Bbox rbb1 = treebox(tree, root);
	Bbox rbb2 = treebox(tree, now);
	int one_in_two = count_in_box(tree, root, rbb2);
	int two_in_one = count_in_box(tree, now, rbb1);
	f1 = 3.0 / (2 + one_in_two + two_in_one);

	// factor 2
	f2 = 1.0 / (abs(size1 - size2) + 1);

	// factor 3
	double avg1 = meanlen(tree, root);
	double avg2 = meanlen(tree, now);
	double off1 = cohesion(tree, root, avg1);
	double off2 = cohesion(tree, now, avg2);
	f3 = 1000.0 / ((off1 + 1) * (off2 + 1));

	// factor 4
	int rest1 = size1 % dev_max;
	int rest2 = size2 % dev_max;
	rest1 = rest1 == 0 ? dev_max : rest1;
	rest2 = rest2 == 0 ? dev_max : rest2;
	f4 = rest1 * 1.0 / dev_max * rest2 * 1.0 / dev_max;

	// factor 5
	f5 = 5.0 * tree[now].weight / (avg1 + avg2 + 1);

	return w1 * f1 + w2 * f2 + w3 * f3 + w4 * f4 + w5 * f5;
}