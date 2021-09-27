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
	if (loop_max_count < 1)
	{
		return "error: loop max count < 1";
	}

	MapInfo map;
	GroupParam param;
	GroupEngine ge;

	// parse geojson file
	map = read_from_geojson_string(datastr);

	if (map.devices.size() == 0)
	{
		return "error: no Wiring Position";
	}
	else if (map.powers.size() == 0)
	{
		return "error: no Power Position";
	}
	else if (map.area.info.boundary.size() == 0)
	{
		return "error: no Fire Apart";
	}
	
	// grouping
	param.max_dev_size = loop_max_count;
	param.min_dev_size = MIN_DEV_IN_GROUP;
	param.weight_pos = ALPHA;
	param.weight_even = BETA;
	param.weight_cohesion = OMEGA;
	param.weight_big = DELTA;
	param.weight_cut_len = GAMMA;

	vector<vector<int>> group_info = ge.grouping(&map, &param);

	// inter connect
	vector<ISData> groups = parse_groups(&map, group_info);

	vector<ImmuneSystem> systems;
	for (int i = 0; i < groups.size(); i++)
	{
		ImmuneSystem is;
		is.init(&groups[i]);
		systems.push_back(is);
	}

	vector<Polyline> cables;

	vector<Polyline> power_paths(map.powers.size());
	for (int e = 0; e < systems.size(); e++)
	{
		for (int k = 0; k < 20; k++)
			systems[e].run();

		if (systems[e].globlMem.size() < 1)
		{
			printf("Group %d No result!\n", e);
			continue;
		}

		auto adj = systems[e].globlMem.rbegin()->adj;
		printf("Best value = %lf\n", systems[e].globlMem.rbegin()->value);

		vector<Device>& devices = systems[e].data.devices;
		int dn = (int)devices.size();

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
					power_paths[i - dn].push_back(devices[root].coord);
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
		get_manhattan_tree(&map, path_tree, cables);
		avoid_coincidence(path_tree);
		vector<Polyline> paths = get_dream_tree_paths(path_tree);
		cables.insert(cables.end(), paths.begin(), paths.end());
		deleteDreamTree(path_tree);
	}

	return write_to_geojson_string(cables);

	// power to device connect

	vector<Polyline> result_paths;
	vector<Segment> exist_lines = get_segments_from_polylines(cables);
	//for (int i = 0; i < power_paths.size(); i++)
	for (int i = 0; i < power_paths.size(); i++)
	{
		Power pwr = map.powers[i];
		for (int j = 0; j < power_paths[i].size(); j++)
		{
			Point dev = power_paths[i][j];
			printf("Power %d: Looking for path %d\n", i, j);
			Polyline pp;
			if (pwr.is_point())
				pp = obstacle_avoid_connect_p2p(&map, pwr.points[0], dev, exist_lines);
			else if (pwr.is_segment())
				pp = obstacle_avoid_connect_p2s(&map, dev, Segment(pwr.points[0], pwr.points[1]), exist_lines);
			else
				return "invalid power";

			if (pp.size() > 1)
				result_paths.push_back(pp);
			//vector<Segment> pp_segs = get_segments_from_polyline(pp);
			//exist_lines.insert(exist_lines.end(), pp_segs.begin(), pp_segs.end());
		}
	}

	//DreamTree dream_tree = merge_to_a_tree(result_paths);

	//vector<Segment> tree_lines = get_dream_tree_lines(dream_tree);

	//deleteDreamTree(dream_tree);

	cables.insert(cables.end(), result_paths.begin(), result_paths.end());


	// output
	return write_to_geojson_string(cables);
}
