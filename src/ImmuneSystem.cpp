#include "ImmuneSystem.h"

using namespace CableRouter;

#define LOCAL_SIZE			16
#define LOCAL_CROSS_RATE	0.4

#define GLOBAL_SIZE			8
#define GLOBAL_CROSS_RATE	0.8

#define VOLATILE_RATE		0.05

#define ANT_SIZE			30

#define DEBUG_MOD			0

#define DEV_GAP				500

vector<int> CableRouter::PruferEncode(const vector<vector<int>>& adj)
{
	//printf("prufer encode begin");
	auto n = adj.size();
	vector<int> code;
	vector<int> d(n);

	for (int u = 0; u < n; u++)
	{
		for (int v = 0; v < adj[u].size(); v++)
		{
			d[u]++;
		}
	}

	int leaf_hunter = 0;
	while (d[leaf_hunter] != 1)
		leaf_hunter++;
	int leaf = leaf_hunter;


	for (int i = 0; i < n - 2; i++)
	{
		//printf("leaf = %d\n", leaf);
		int v;
		for (int vi = 0; vi < adj[leaf].size(); vi++)
		{
			v = adj[leaf][vi];
			if (d[v] != 0)
				break;
		}

		//printf("find neighbor %d\n", v);
		code.push_back(v);
		d[leaf]--;
		d[v]--;
		if (v < leaf_hunter && d[v] == 1)
		{
			leaf = v;
		}
		else
		{
			while (d[leaf_hunter] != 1)
				leaf_hunter++;
			leaf = leaf_hunter;
		}
	}

	//printf("prufer encode end");
	return code;
}

vector<vector<int>> CableRouter::PruferDecode(const vector<int>& codes)
{
	//printf("prufer decode begin");
	int n = (int)(codes.size() + 2);

	vector<vector<int>> adj(n);
	vector<int> p(n, 0);

	vector<int> code = codes;
	code.push_back(n - 1);
	for (int i = 0; i < code.size(); i++)
	{
		p[code[i]]++;
	}

	int leaf_hunter = 0;
	while (p[leaf_hunter] != 0)
		leaf_hunter++;
	int leaf = leaf_hunter;



	for (int i = 0; i < n - 1; i++)
	{
		int u = code[i];
		adj[u].push_back(leaf);
		adj[leaf].push_back(u);
		p[u]--;
		p[leaf]--;
		if (u < leaf_hunter && p[u] == 0)
		{
			leaf = u;
		}
		else
		{
			while (p[leaf_hunter] != 0)
				leaf_hunter++;
			leaf = leaf_hunter;
		}
	}


	//printf("p: ");
	//for (int i = 0; i < n; i++)
	//{
	//	printf("%d ", p[i]);
	//}
	//printf("\n");

	//printf("prufer decode end");
	return adj;
}

vector<ISData> CableRouter::parse_groups(MapInfo* data, vector<vector<int>>& groups)
{
	CDTP dt = buildTriangulation(data);
	int n_all = (int)(dt.number_of_vertices() + data->powers.size());

	double** G = buildGraphAll(data, dt, n_all, false, true);
	addDeviceEdges(data, G, false, true);
	addPowerEdges(data, dt, G, false, true);
	removeObstacles(data, G, n_all);
	adjustByLayoutType(data, G);

	// convert result to data for run engine
	printf("Init group info begin\n");
	vector<ISData> res;
	for (int i = 0; i < groups.size(); i++)
	{
		vector<Device> devs;
		vector<Power> pwrs;

		bool* vis = new bool[groups[i].size()];
		fill(vis, vis + groups[i].size(), false);
		for (int j = 0; j < groups[i].size(); j++)
		{
			int u = groups[i][j];
			if (vis[j]) continue;

			//int choose = -1;
			//for (int k = j + 1; k < groups[i].size(); k++)
			//{
			//	int v = groups[i][k];
			//	if (G[u][v] <= DEV_GAP)
			//	{
			//		vis[k] = true;
			//		if (data->devices[u].coord.hx() != data->devices[v].coord.hx())
			//		{
			//			for (int k = 0; k < n_all; k++)
			//			{
			//				G[u][k] = G[v][k] =
			//					G[k][u] = G[k][v] =
			//					std::min(G[u][k], G[v][k]);
			//			}
			//			choose = data->devices[u].coord.hx() < data->devices[v].coord.hx() ?
			//				u : v;
			//		}
			//		else
			//		{
			//			for (int k = 0; k < n_all; k++)
			//			{
			//				G[u][k] = G[v][k] =
			//					G[k][u] = G[k][v] =
			//					std::min(G[u][k], G[v][k]);
			//			}
			//			choose = data->devices[u].coord.hy() < data->devices[v].coord.hy() ?
			//				u : v;
			//		}
			//		devs.push_back(data->devices[choose]);
			//		break;
			//	}
			//}
			//if (choose == -1)
			//	devs.push_back(data->devices[u]);
			devs.push_back(data->devices[u]);

			vis[j] = true;
		}
		delete[] vis;
		pwrs = data->powers;
		int dev_n = (int)devs.size();
		int pwr_n = (int)pwrs.size();
		vector<vector<double>> g(dev_n + pwr_n);
		for (int p = 0; p < dev_n; p++)
		{
			for (int q = 0; q < dev_n; q++)
			{
				g[p].push_back(G[devs[p].id][devs[q].id]);
			}
			for (int q = 0; q < pwr_n; q++)
			{
				g[p].push_back(G[devs[p].id][pwrs[q].id]);
			}
		}
		for (int p = dev_n; p < g.size(); p++)
		{
			for (int q = 0; q < dev_n; q++)
			{
				g[p].push_back(G[pwrs[p - dev_n].id][devs[q].id]);
			}
			for (int q = 0; q < pwr_n; q++)
			{
				g[p].push_back(G[pwrs[p - dev_n].id][pwrs[q].id]);
			}
		}

		ISData data;
		data.devices = devs;
		data.powers = pwrs;
		data.G = g;
		res.push_back(data);
	}
	printf("Init group info end\n");

	deleteGraph(G, n_all);
	dt.clear();
	return res;
}


double CableRouter::ImmuneSystem::affinity(vector<vector<int>>& adj, vector<vector<double>>& G, int dn)
{
	//printf("affinity begin\n");
	auto n = adj.size();

	for (int u = 0; u < adj.size(); u++)
	{
		for (int i = 0; i < adj[u].size(); i++)
		{
			int v = adj[u][i];
			if (G[u][v] >= CR_INF) return -1;
		}
	}

	// path length
	double len_sum = 0.0;

	for (int u = 0; u < adj.size(); u++)
	{
		if (adj[u].size() > 4)
		{
			//printf("affinity: degree exceed 3\n");
			return -1;
		}
		for (int i = 0; i < adj[u].size(); i++)
		{
			int v = adj[u][i];
			len_sum += G[u][v];
		}
	}

	// dijkstra
	int start = -1;
	int start_dev = -1;
	for (int s = dn; s < adj.size(); s++)
	{
		for (auto t : adj[s])
		{
			if (t < dn) {
				start = s;
				start_dev = t;
			}
		}
	}
	if (start == -1)
	{
		printf("affinity: no power source\n");
		return -1;
	}

	bool* vis = new bool[n];
	double* dis = new double[n];
	int* parent = new int[n];
	fill(vis, vis + n, false);
	fill(dis, dis + n, CR_INF);
	fill(parent, parent + n, -1);
	dis[start] = 0;
	vis[start] = true;
	queue<int> q;
	q.push(start);
	while (!q.empty())
	{
		int now = q.front();
		q.pop();
		for (auto v : adj[now])
		{
			if (vis[v]) continue;
			dis[v] = dis[now] + G[now][v];
			parent[v] = now;
			vis[v] = true;
			q.push(v);
		}
	}

	// back path
	vector<Segment> segs;
	double back = 0.0;
	for (int i = 0; i < dn; i++)
	{
		if (i == start_dev) continue;
		pair<double, double> dir = make_pair(
			DOUBLE(data.devices[parent[i]].coord.hx() - data.powers[start - dn].points[0].hx()),
			DOUBLE(data.devices[parent[i]].coord.hy() - data.powers[start - dn].points[0].hy()));
		pair<double, double> path = make_pair(
			DOUBLE(data.devices[i].coord.hx() - data.devices[parent[i]].coord.hx()),
			DOUBLE(data.devices[i].coord.hy() - data.devices[parent[i]].coord.hy()));
		segs.push_back(Segment(data.devices[i].coord, data.devices[parent[i]].coord));
		double mod = sqrt(path.first * path.first + path.second * path.second);
		if (dir.first * path.first < 0)
		{
			back += abs(path.first) / mod * G[i][parent[i]];
		}
		if (dir.second * path.second < 0)
		{
			back += abs(path.second) / mod * G[i][parent[i]];
		}
	}

	// intersection
	int cross_n = 0;
	for (int i = 0; i < segs.size(); i++)
	{
		for (int j = i + 1; j < segs.size(); j++)
		{
			if (!CGAL::do_intersect(segs[i], segs[j])) continue;
			CGAL::Object result = CGAL::intersection(segs[i], segs[j]);
			Point pt; Segment seg;
			if (CGAL::assign(seg, result))
			{
				cross_n += 2;
			}
			else if (CGAL::assign(pt, result))
			{
				if (CGAL::do_intersect(shrink_segment(segs[i]), shrink_segment(segs[j])))
					cross_n += 1;
			}
		}
	}

	// horizontal and vertical
	int beauty_n = 0;
	for (int i = 0; i < segs.size(); i++)
	{
		if (EQUAL(segs[i].source().hx(), segs[i].target().hx()) ||
			EQUAL(segs[i].source().hy(), segs[i].target().hy()))
			beauty_n++;
	}

	// length from source
	double len_source = 0.0;
	for (int i = 0; i < dn; i++)
	{
		len_source += dis[i];
	}

	delete[] vis;
	delete[] dis;
	delete[] parent;


	//printf("affinity end\n");

	len_source /= (1.0 * dn);
	len_sum /= (1.0 * dn);
	back /= (1.0 * dn);

	//printf("len_source = %lf, len_sum = %lf, back = %lf\n", len_source, len_sum, back);

	//double value = 1000000.0 / (len_sum + 1) + 2000.0 / (back + 100) + 0.0 / (len_source - len_sum + 1) - 40 * cross_n + 0.0 * beauty_n;
	double value = 1000000.0 / (len_sum + back + 1000) - 40 * cross_n + 0.0 * beauty_n;

	return value > 0 ? value : 0;
}

bool CableRouter::ImmuneSystem::init(ISData* d)
{
	data = *d;
	// parse_groups pheromone
	printf("Init pheromone begin\n");
	pheromone = data.G;
	for (auto v = pheromone.begin(); v != pheromone.end(); v++)
	{
		for (auto ph = v->begin(); ph != v->end(); ph++)
		{
			if ((*ph) >= CR_INF)
				(*ph) = 0;
			else
				(*ph) = 5.0 + 200000.0 / ((*ph) + 0.001);
		}
	}
	printf("Init pheromone end_node\n");

	int n = (int)data.G.size();
	auto dn = (int)data.devices.size();
	vector<vector<int>> adj(n);

	int start = -1;
	int pwr = -1;
	double MIN = CR_INF;
	for (int i = dn; i < n; i++)
	{
		if (i > dn)
		{
			adj[i - 1].push_back(i);
			adj[i].push_back(i - 1);
		}
		for (int v = 0; v < dn; v++)
		{
			if (data.G[i][v] < MIN)
			{
				MIN = data.G[i][v];
				start = v;
				pwr = i;
			}
		}
	}
	if (pwr == -1)
	{
		printf("Error: can't connect to power\n");
		return false;
	}
	adj[start].push_back(pwr);
	adj[pwr].push_back(start);

	double* disg = new double[dn];
	bool* vis = new bool[dn];
	int* parent = new int[dn];
	fill(vis, vis + dn, false);
	fill(parent, parent + dn, -1);
	fill(disg, disg + dn, CR_INF);
	disg[start] = 0;
	for (int i = 0; i < dn; i++)
	{
		int u = -1;
		MIN = CR_INF;
		for (int j = 0; j < dn; j++)
		{
			if (!vis[j] && disg[j] < MIN)
			{
				MIN = disg[j];
				u = j;
			}
		}

		if (u == -1) break;
		vis[u] = true;

		if (parent[u] != -1)
		{
			adj[parent[u]].push_back(u);
			adj[u].push_back(parent[u]);
		}

		for (int v = 0; v < dn; v++)
		{
			if (!vis[v] && data.G[u][v] != CR_INF)
			{
				if (data.G[u][v] < disg[v])
				{
					disg[v] = data.G[u][v];
					parent[v] = u;
				}
			}
		}
	}
	delete[] vis;
	delete[] parent;
	delete[] disg;

	double value = affinity(adj, data.G, dn);
	if (value > 0)
	{
		Antibody an;
		an.adj = adj;
		an.prufer_code = PruferEncode(adj);
		an.value = value;
		globlMem.insert(an);
	}

	return true;
}

void CableRouter::ImmuneSystem::run()
{
	vector<Device>& devices = data.devices;
	vector<Power>& powers = data.powers;

	int n = (int) data.G.size();
	auto dn = (int) devices.size();

	set<Antibody> mem;

	if (DEBUG_MOD)
		printf("Ant searching begin\n");

	int M = ANT_SIZE;
	for (int k = 0; k < M; k++)
	{
		vector<vector<int>> adj(n);

		double* disg = new double[dn];
		bool* vis = new bool[dn];
		int* parent = new int[dn];
		fill(vis, vis + dn, false);
		fill(parent, parent + dn, -1);
		fill(disg, disg + dn, 0);
		int start = cgal_rand.get_int(0, dn);
		disg[start] = 1;

		for (int i = 0; i < dn; i++)
		{
			int u = -1;
			vector<pair<int, double>> rou;
			double sum = 0;
			for (int j = 0; j < dn; j++)
			{
				if (!vis[j])
				{
					sum += disg[j];
					rou.push_back(make_pair(j, sum));
				}
			}
			double x = cgal_rand.get_double(0, sum);
			for (int j = 0; j < rou.size(); j++)
			{
				if (x <= rou[j].second)
				{
					u = rou[j].first;
					break;
				}
			}

			if (u == -1) break;
			vis[u] = true;

			if (parent[u] != -1)
			{
				adj[parent[u]].push_back(u);
				adj[u].push_back(parent[u]);
			}

			for (int v = 0; v < dn; v++)
			{
				if (!vis[v] && pheromone[u][v] > disg[v])
				{
					disg[v] = pheromone[u][v];
					parent[v] = u;
				}
			}
		}
		delete[] vis;
		delete[] parent;
		delete[] disg;

		vector<pair<pair<int, int>, double>> rou;
		double sum = 0;
		for (int i = dn; i < n; i++)
		{
			if (i > dn)
			{
				adj[i - 1].push_back(i);
				adj[i].push_back(i - 1);
			}
			for (int v = 0; v < dn; v++)
			{
				sum += pheromone[i][v];
				rou.push_back(make_pair(make_pair(i, v), sum));
			}
		}
		double x = cgal_rand.get_double(0, sum);
		for (int j = 0; j < rou.size(); j++)
		{
			if (x <= rou[j].second)
			{
				adj[rou[j].first.first].push_back(rou[j].first.second);
				adj[rou[j].first.second].push_back(rou[j].first.first);
				break;
			}
		}

		double value = affinity(adj, data.G, dn);
		if (value > 0)
		{
			Antibody an;
			an.adj = adj;
			an.prufer_code = PruferEncode(adj);
			an.value = value;
			mem.insert(an);
		}
	}
	if (DEBUG_MOD)
		printf("Ant searching end_node\n");


	priority_queue<Antibody, vector<Antibody>, less<Antibody>> freshMem;

	// cross
	if (DEBUG_MOD)
		printf("Cross begin\n");
	for (auto ai = mem.begin(); ai != mem.end(); ai++)
	{
		freshMem.push(*ai);
		vector<int> code1 = ai->prufer_code;

		for (auto aj = localMem.begin(); aj != localMem.end(); aj++)
		{
			if (cgal_rand.get_double() > LOCAL_CROSS_RATE) continue;
			vector<int> code2 = aj->prufer_code;
			vector<int> code3;
			for (int x = 0; x < code1.size(); x++)
			{
				if (cgal_rand.get_double() < 0.5)
				{
					code3.push_back(code1[x]);
				}
				else
				{
					code3.push_back(code2[x]);
				}
			}
			Antibody a;
			a.adj = PruferDecode(code3);
			a.value = affinity(a.adj, data.G, dn);
			if (a.value > 0)
			{
				a.prufer_code = code3;
				freshMem.push(a);
			}
		}

		for (auto aj = globlMem.begin(); aj != globlMem.end(); aj++)
		{
			if (cgal_rand.get_double() > GLOBAL_CROSS_RATE) continue;
			vector<int> code2 = aj->prufer_code;
			vector<int> code3;
			for (int x = 0; x < code1.size(); x++)
			{
				if (cgal_rand.get_double() < 0.5)
				{
					code3.push_back(code1[x]);
				}
				else
				{
					code3.push_back(code2[x]);
				}
			}
			Antibody a;
			a.adj = PruferDecode(code3);
			a.value = affinity(a.adj, data.G, dn);
			if (a.value > 0)
			{
				a.prufer_code = code3;
				freshMem.push(a);
			}
		}
	}
	if (DEBUG_MOD)
		printf("Cross end_node\n");


	// best 10 push to local
	// best 5 push to global and 
	if (DEBUG_MOD)
		printf("Save begin\n");
	reset(localMem);
	for (int i = 0; i < freshMem.size() && i < LOCAL_SIZE; i++)
	{
		if (globlMem.size() >= GLOBAL_SIZE)
		{
			if (globlMem.begin()->value < freshMem.top().value&&
				globlMem.find(freshMem.top()) == globlMem.end())
			{
				globlMem.erase(globlMem.begin());
				globlMem.insert(freshMem.top());
			}
		}
		else
		{
			globlMem.insert(freshMem.top());
		}
		localMem.insert(freshMem.top());
		freshMem.pop();
	}

	if (DEBUG_MOD)
		printf("Save end_node\n");

	if (DEBUG_MOD)
		printf("Update begin\n");
	// update pheromone
	for (int i = 0; i < pheromone.size(); i++)
	{
		for (int j = 0; j < pheromone.size(); j++)
		{
			pheromone[i][j] *= (1 - VOLATILE_RATE);
		}
	}
	for (auto ai = localMem.begin(); ai != localMem.end(); ai++)
	{
		auto adj = ai->adj;
		for (int u = 0; u < adj.size(); u++)
		{
			for (int j = 0; j < adj[u].size(); j++)
			{
				int v = adj[u][j];
				pheromone[u][v] += (ai->value * VOLATILE_RATE);
			}
		}
	}
	if (DEBUG_MOD)
		printf("Update end_node\n");
}

