#include "GETree.h"
#include <queue>

using namespace CableRouter;

CableRouter::GETree::GETree(vector<Point>& coords, double** G)
{
	int n = (int)coords.size();
	nodes = vector<GENode>(n);

	vector<int> parent = this->prim(G, n, 0);
	for (int i = 0; i < n; i++)
	{
		int pa = parent[i];
		nodes[i].parent = pa;
		nodes[i].coord = coords[i];
		if (pa != -1) {
			nodes[pa].children.push_back(i);
			nodes[i].weight = G[pa][i];
		}
	}
}

vector<int> CableRouter::GETree::prim(double** G, int n, int start)
{
	vector<int> res;

	double* disg = new double[n];
	bool* vis = new bool[n];
	int* parent = new int[n];
	fill(disg, disg + n, CR_INF);
	fill(vis, vis + n, false);
	fill(parent, parent + n, -1);
	disg[start] = 0;
	for (int i = 0; i < n; i++)
	{
		double MIN = CR_INF;
		int u = -1;
		for (int j = 0; j < n; j++)
		{
			if (!vis[j] && disg[j] < MIN)
			{
				MIN = disg[j];
				u = j;
			}
		}
		if (u == -1) break;
		vis[u] = true;

		for (int v = 0; v < n; v++)
		{
			if (!vis[v] && G[u][v] != CR_INF)
			{
				if (G[u][v] < disg[v])
				{
					disg[v] = G[u][v];
					parent[v] = u;
				}
			}
		}
	}
	res.assign(parent, parent + n);
	delete[] disg;
	delete[] vis;
	delete[] parent;
	return res;
}

int CableRouter::count(vector<GENode>& tree, int root)
{
	int res = 1;
	for (int i = 0; i < tree[root].children.size(); i++)
	{
		res += count(tree, tree[root].children[i]);
	}
	return res;
}

void CableRouter::breaking(vector<GENode>& tree, int now)
{
	int parent = tree[now].parent;
	for (auto i = tree[parent].children.begin(); i != tree[parent].children.end(); i++)
	{
		if ((*i) == now) {
			tree[parent].children.erase(i);
			break;
		}
	}
	tree[now].parent = -1;
}

void CableRouter::reconnect(vector<GENode>& tree, int parent, int now)
{
	tree[now].parent = parent;
	tree[parent].children.push_back(now);
}

Bbox CableRouter::treebox(vector<GENode>& tree, int root)
{
	Bbox res;
	res.maxX = DOUBLE(tree[root].coord.hx());
	res.maxY = DOUBLE(tree[root].coord.hy());
	res.minX = DOUBLE(tree[root].coord.hx());
	res.minY = DOUBLE(tree[root].coord.hy());
	for (int i = 0; i < tree[root].children.size(); i++)
	{
		Bbox ch_box = treebox(tree, tree[root].children[i]);
		res.maxX = std::max(res.maxX, ch_box.maxX);
		res.maxY = std::max(res.maxY, ch_box.maxY);
		res.minX = std::min(res.minX, ch_box.minX);
		res.minY = std::min(res.minY, ch_box.minY);
	}
	return res;
}

int CableRouter::count_in_box(vector<GENode>& tree, int root, Bbox box)
{
	int count = 0;
	std::queue<int> q;
	q.push(root);
	while (!q.empty())
	{
		int now = q.front();
		q.pop();

		if (point_in_box(tree[now].coord, box)) count++;

		for (int i = 0; i < tree[now].children.size(); i++)
		{
			q.push(tree[now].children[i]);
		}
	}

	return count;
}

bool CableRouter::point_in_box(Point p, Bbox box)
{
	return p.hx() <= box.maxX && p.hx() >= box.minX && p.hy() <= box.maxY && p.hy() >= box.minY;
}

Polygon CableRouter::convex(vector<GENode>& tree, int root)
{
	vector<int> ids = get_points(tree, root);
	vector<Point> pts, res;
	for (auto id = ids.begin(); id != ids.end(); id++)
	{
		pts.push_back(tree[*id].coord);
	}
	CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(res));
	return Polygon(res.begin(), res.end());
}

double CableRouter::meanlen(vector<GENode>& tree, int root)
{
	double mean = 0;
	int size = 0;
	std::queue<int> q;
	for (int i = 0; i < tree[root].children.size(); i++)
	{
		q.push(tree[root].children[i]);
	}
	while (!q.empty())
	{
		int now = q.front();
		q.pop();

		size++;
		mean += tree[now].weight;

		for (int i = 0; i < tree[now].children.size(); i++)
		{
			q.push(tree[now].children[i]);
		}
	}
	if (size == 0) return 0;

	return mean / size;
}

double CableRouter::cohesion(vector<GENode>& tree, int root, double mean)
{
	double st = 0;
	int size = 0;
	std::queue<int> q;
	for (int i = 0; i < tree[root].children.size(); i++)
	{
		q.push(tree[root].children[i]);
	}
	while (!q.empty())
	{
		int now = q.front();
		q.pop();

		st += pow(mean - tree[now].weight, 2);
		size++;

		for (int i = 0; i < tree[now].children.size(); i++)
		{
			q.push(tree[now].children[i]);
		}
	}
	return st / size;
}

vector<int> CableRouter::get_points(vector<GENode>& tree, int root)
{
	vector<int> res;

	std::queue<int> q;
	q.push(root);
	while (!q.empty())
	{
		int now = q.front();
		q.pop();

		res.push_back(now);

		for (int i = 0; i < tree[now].children.size(); i++)
		{
			q.push(tree[now].children[i]);
		}
	}

	return res;
}