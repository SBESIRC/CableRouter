#include "CableRouteEngine.h"
#include "CR_IO.h"
#include "GroupEngine.h"
#include "ImmuneSystem.h"
#include "RouteEngine.h"
#include "OptimizeEngine.h"

using namespace CableRouter;

// FOR PARTITION
#define MAX_DEV_IN_GROUP 25
#define MIN_DEV_IN_GROUP 10

#define ALPHA	30		// pos relation
#define BETA	10		// size even
#define OMEGA	50		// inner cohesion
#define DELTA	30		// size big
#define GAMMA	10		// cut line len

//string CableRouter::grouping(string datastr)
//{
//	MapInfo map;
//	vector<Block> data; 
//	set<string> categories;
//	set<string> spacenames;
//
//	// parse geojson file
//	data = parse_geojson_string(datastr, categories, spacenames);
//	map = read_blocks(data);
//	GroupEngine ge;
//	ge.grouping(&map, NULL, NULL);
//
//	return string();
//}
//
//string CableRouter::inter_routing(string datastr)
//{
//	return string();
//}

bool path_compare(std::pair<Point, Point> a, std::pair<Point, Point> b)
{
	int a_right = (a.second.hx() - a.first.hx()) > 0 ? 1 : -1;
	int b_right = (b.second.hx() - b.first.hx()) > 0 ? 1 : -1;
	if (a_right != b_right) return a_right < b_right;
	return a_right * abs(DOUBLE(a.second.hy() - a.first.hy())) >
		b_right * abs(DOUBLE(b.second.hy() - b.first.hy()));
}

string CableRouter::CableRouteEngine::routing(string datastr, int loop_max_count)
{
	MapInfo map;
	set<string> categories;
	set<string> spacenames;

	// parse geojson file
	map = read_from_geojson_string(datastr);
	GroupEngine ge;
	
	// grouping
	GroupParam param;
	param.max_dev_size = loop_max_count;
	param.min_dev_size = MIN_DEV_IN_GROUP;
	param.weight_pos = ALPHA;
	param.weight_even = BETA;
	param.weight_cohesion = OMEGA;
	param.weight_big = DELTA;
	param.weight_cut_len = GAMMA;

	vector<vector<int>> group_info = ge.grouping(&map, &param, NULL);

	// inter connect
	vector<ISData> groups = parse_groups(&map, group_info);

	vector<ImmuneSystem> systems;
	for (int i = 0; i < groups.size(); i++)
	{
		ImmuneSystem is;
		is.init(&groups[i]);
		systems.push_back(is);
	}

	vector<Segment> cables;

	vector<std::pair<Point, Point>> power_paths;
	for (int e = 0; e < systems.size(); e++)
	{
		for (int k = 0; k < 10; k++)
			systems[e].run();

		if (systems[e].globlMem.size() < 1)
		{
			printf("Group %d No result!\n", e);
			continue;
		}

		auto adj = systems[e].globlMem.rbegin()->adj;
		printf("Best value = %lf\n", systems[e].globlMem.rbegin()->value);

		vector<vector<Point>> paths;
		vector<Device>& devices = systems[e].data.devices;
		vector<Power>& powers = systems[e].data.powers;
		int dn = devices.size();

		vector<DreamNode*> dev_nodes;
		for (int i = 0; i < dn; i++)
		{
			DreamNode* no = newDreamNode(devices[i].coord);
			no->is_device = true;
			dev_nodes.push_back(no);
		}
		int root;
		for (int i = dn; i < adj.size(); i++)
		{
			bool found = false;
			for (int j = 0; j < adj[i].size(); j++)
			{
				if (adj[i][j] < dn)
				{
					found = true;
					root = adj[i][j];
					power_paths.push_back(make_pair(powers[i - dn].points[0], devices[root].coord));
					break;
				}
			}
			if (found) break;
		}

		vector<int> vis(dn, 0);
		queue<int> dev_queue;

		DreamTree path_tree = dev_nodes[root];
		dev_queue.push(root);
		while (!dev_queue.empty())
		{
			int now = dev_queue.front();
			dev_queue.pop();
			vis[now] = 1;
			for (int i = 0; i < adj[now].size(); i++)
			{
				int ch = adj[now][i];
				if (ch < dn && vis[ch] == 0)
				{
					dev_nodes[now]->children.push_back(dev_nodes[ch]);
					dev_nodes[ch]->parent = dev_nodes[now];
					vis[ch] = 1;
					dev_queue.push(ch);
				}
			}
		}
		get_manhattan_lines(&map, path_tree, cables);
	}


	// power to device connect

	vector<vector<Point>> result_paths;
	vector<Segment> exist_lines = cables;
	//for (int i = 0; i < power_paths.size(); i++)
	for (int i = 0; i < power_paths.size(); i++)
	{
		Point pwr = Point(power_paths[i].first.hx(), power_paths[i].first.hy());
		Point dev = power_paths[i].second;
		printf("Looking for path %d\n", i);
		vector<Point> pp = obstacle_avoid_connect_p2p(&map, pwr, dev, exist_lines);
		//vector<Point> pp = obstacle_avoid_connect_p2s(&data, dev, pwr, exist_lines);

		printf("Path size: %d\n", pp.size());

		result_paths.push_back(pp);
	}

	DreamTree dream_tree = merge_to_a_tree(result_paths);

	vector<Segment> tree_lines = get_dream_tree_lines(dream_tree);

	cables.insert(cables.end(), tree_lines.begin(), tree_lines.end());

	deleteDreamTree(dream_tree);


	// output
	stringstream fout;
	fout << fixed;
	fout << "{\r\n";
	fout << "    \"type\": \"FeatureCollection\",\r\n";
	fout << "    \"features\": [\r\n";
	for (int i = 0; i < cables.size(); i++)
	{
		//output_string(fout, &data[i]);
		//if (i == data.size() - 1)
		//{
		//	fout << "\r\n";
		//}
		//else
		//{
		//	fout << ",\r\n";
		//}
	}

	fout << "    ]\r\n";
	fout << "}";
	return fout.str();
}
