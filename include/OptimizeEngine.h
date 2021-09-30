#include "CR_CGALUtils.h"
#include "MapUtils.h"

#ifndef _CABLEROUTER_OPTIMIZE_ENGINE_H_
#define _CABLEROUTER_OPTIMIZE_ENGINE_H_

namespace CableRouter
{
	struct DreamNode;
	typedef shared_ptr<DreamNode> DreamNodePtr;
	typedef DreamNodePtr DreamTree;

	struct DreamNode
	{
		DreamNodePtr parent;
		Point coord;
		vector<DreamNodePtr> children;

		// for inter group connect
		int line_num_to_parent;
		bool is_device;

		// for inner group connect
		Vector dir_from_parent = Vector(0.0, 0.0);
	};

	enum NodeDir
	{
		N_LEFT = 0,
		N_RIGHT = 1,
		N_DOWN = 2,
		N_UP = 3
	};

	DreamNodePtr newDreamNode(Point coord);
	vector<DreamNodePtr> getAllNodes(DreamTree tree);

	// inner connect
	void get_manhattan_tree(MapInfo* map, DreamTree tree, vector<Polyline>& exist);
	void avoid_coincidence(DreamTree tree);
	void avoid_coincidence_non_device(DreamTree tree);
	void avoid_left(
		vector<DreamNodePtr>& left, DreamNodePtr now, DreamNodePtr now_pa,
		vector<DreamNodePtr>& down, vector<DreamNodePtr>& up, int fix = -1);
	void avoid_right(
		vector<DreamNodePtr>& right, DreamNodePtr now, DreamNodePtr now_pa,
		vector<DreamNodePtr>& down, vector<DreamNodePtr>& up, int fix = -1);
	void avoid_down(
		vector<DreamNodePtr>& down, DreamNodePtr now, DreamNodePtr now_pa,
		vector<DreamNodePtr>& left, vector<DreamNodePtr>& right, int fix = -1);
	void avoid_up(
		vector<DreamNodePtr>& up, DreamNodePtr now, DreamNodePtr now_pa,
		vector<DreamNodePtr>& left, vector<DreamNodePtr>& right, int fix = -1);

	// inter connect
	DreamTree merge_to_a_tree(vector<Polyline>& paths);

	vector<pair<DreamNodePtr, Point>> intersect_dream_tree(DreamTree tree, Segment seg);
	void add_line_num(DreamNodePtr node);
	void init_line_num(DreamTree tree);

	vector<Polyline> get_dream_tree_paths(DreamTree tree);
	Polyline get_path(DreamNodePtr node);

	vector<Segment> get_dream_tree_lines(DreamTree tree, bool opened = false);
	void horizontal_count(DreamNodePtr node, int& up, int& down);
	void vertical_count(DreamNodePtr node, int& left, int& right);
}

#endif