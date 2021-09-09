#include "CableRouteEngine.h"
#include "CR_Parse.h"
#include "CR_IO.h"
#include "GroupEngine.h"
#include "ImmuneSystem.h"
#include "RouteEngine.h"

using namespace CableRouter;
using namespace CableRouterParse;

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
	vector<Block> data; 
	set<string> categories;
	set<string> spacenames;

	// parse geojson file
	data = parse_geojson_string(datastr, categories, spacenames);
	map = read_blocks(data);
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

		auto adj = systems[e].globlMem.rbegin()->adj;
		printf("Best value = %lf\n", systems[e].globlMem.rbegin()->value);

		vector<vector<Point>> paths;
		vector<Device>& devices = systems[e].data.devices;
		vector<Power>& powers = systems[e].data.powers;
		int dn = (int)devices.size();

		vector<int> d(adj.size());
		for (int i = 0; i < adj.size(); i++)
			d[i] = (int)adj[i].size();
		for (int i = 0; i < adj.size(); i++)
		{
			Point u = i < devices.size() ? devices[i].coord : powers[i - dn].points[0];
			for (int j = 0; j < adj[i].size(); j++)
			{
				if (d[adj[i][j]] == 0) continue;
				Point v = adj[i][j] < dn ? devices[adj[i][j]].coord : powers[adj[i][j] - dn].points[0];

				if (i < dn && adj[i][j] < dn)
				{
					paths.push_back(manhattan_connect(&map, u, v, cables));
				}
				else if (i >= dn && adj[i][j] < dn)
				{
					power_paths.push_back(std::make_pair(u, v));
				}
				else if (i < dn && adj[i][j] >= dn)
				{
					power_paths.push_back(std::make_pair(v, u));
				}

				d[i]--;
				d[adj[i][j]]--;
			}
		}
		for (int i = 0; i < paths.size(); i++)
		{
			for (int j = 0; j < paths[i].size() - 1; j++)
			{
				cables.push_back(Segment(paths[i][j], paths[i][j + 1]));
			}
		}
	}


	// power to device connect

	std::sort(power_paths.begin(), power_paths.end(), path_compare);

	vector<int> order;
	for (int i = 0; i < power_paths.size(); i++)
	{
		order.push_back(i);
	}

	vector<Segment> exist_lines = cables;
	//for (int i = 0; i < power_paths.size(); i++)
	for (int ii = 0; ii < order.size(); ii++)
	{
		int i = order[ii];
		int off = ii - power_paths.size() / 2;
		Point pwr = Point(power_paths[i].first.hx(), power_paths[i].first.hy());
		//Point pwr = Point(power_paths[i].first.hx() + off * 500, power_paths[i].first.hy() + off * 500);
		//Segment pwr = Segment(power_paths[i].first, Point(power_paths[i].first.hx() + 7 * 400, power_paths[i].first.hy()));
		Point dev = power_paths[i].second;
		printf("Looking for path %d\n", i);
		vector<Point> pp = obstacle_avoid_connect_p2p(&map, pwr, dev, exist_lines);

		printf("Path size: %d\n", pp.size());

		for (int j = 0; j < pp.size() - 1; j++)
		{
			cables.push_back(Segment(pp[j], pp[j + 1]));
		}
	}


	// output
	stringstream fout;
	fout << fixed;
	fout << "{\r\n";
	fout << "    \"type\": \"FeatureCollection\",\r\n";
	fout << "    \"features\": [\r\n";
	for (int i = 0; i < cables.size(); i++)
	{
		output_string(fout, &data[i]);
		if (i == data.size() - 1)
		{
			fout << "\r\n";
		}
		else
		{
			fout << ",\r\n";
		}
	}

	fout << "    ]\r\n";
	fout << "}";
	return fout.str();
}
