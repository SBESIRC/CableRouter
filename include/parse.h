#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <set>
#include <map>
#include <regex>
#include <cmath>
#include <assert.h>

#ifndef _CABLEROUTER_PARSE_H
#define _CABLEROUTER_PARSE_H

#define MAX_LENGTH 1e10

using namespace std;

namespace CableRouterParse {

	struct point{
		double x;
		double y;
	};

	class Block
	{
	public:
		Block();
		Block(vector<point> coords, string category, string name);

	public:
		vector<point> coords;
		vector<string> neighbor_ids;
		point alignment_vector;
		point direction;
		string category;
		string name;
		string id;
		string area_id;
		string group_id;
		bool isolated;
	};

	static void splits(const string& s, vector<string>& tokens, const string& delimiters = " ");
	static void splits_properties(const string& s, vector<string>& tokens, const string& delimiters = " ");
	vector<Block> parse_geojson(const string filename, set<string> &categories, set<string> &spacenames);
	vector<Block> parse_geojson_string(string datastr, set<string> &categories, set<string> &spacenames);
	void analysis(vector<Block>& data, map<string, Block*>& region, map<string, vector<Block*>>& space, map<string, vector<Block*>>& column, map<string, vector<Block*>>& input, map<string, vector<Block*>>& output);
	void output_file(string filename, vector<Block>& data);
	void output_single(ofstream& fout, Block* data);
	void output_string(stringstream& fout, Block* data);
	void get_region_size(Block* region, double& min_x, double& min_y);
	void copy_block(Block* old_block, Block* new_block);
	void rotate_point(point& pt, double angle, double min_x, double min_y);
	void align_region(double& angle, point& origin, Block* region, vector<Block*>& space, vector<Block*>& input, vector<Block*>& output, vector<Block*>& column, Block* new_region, vector<Block*>& new_space, vector<Block*>& new_input, vector<Block*>& new_output, vector<Block*>& new_column);
	void rotate_region(double& angle, point& origin, Block* region, vector<Block*>& space, vector<Block*>& input, vector<Block*>& output, vector<Block*>& column);
	double pp_distance(point x1, point x2);
	double point_wall_distance(point wall_p1, point wall_p2, point pt);
	void get_space_size(Block* space, double& area, double& min_x, double& max_x, double& min_y, double& max_y);
	double pp_mul(point p1, point p2, point p3);
	void get_align_vector(Block * region, double& angle, double& min_x, double& min_y);

}

#endif