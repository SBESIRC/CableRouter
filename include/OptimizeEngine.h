#include "CR_CGALUtils.h"
#include "MapUtils.h"

#ifndef _CABLEROUTER_OPTIMIZE_ENGINE_H_
#define _CABLEROUTER_OPTIMIZE_ENGINE_H_

namespace CableRouter
{
	struct DreamNode
	{
		DreamNode* parent;
		Point coord;
		vector<DreamNode*> children;

		// for inter group connect
		int line_num_to_parent;
		bool is_device;

		// for inner group connect
		Vector dir_from_parent = Vector(0.0, 0.0);
	};

	typedef DreamNode* DreamTree;

	DreamNode* newDreamNode(Point coord);
	void deleteDreamTree(DreamTree root);

	// inner connect
	void get_manhattan_tree(MapInfo* map, DreamTree tree, vector<Polyline>& exist);
	void avoid_coincidence(DreamTree tree);
	void avoid_coincidence_non_device(DreamTree tree);

	// inter connect
	DreamTree merge_to_a_tree(vector<Polyline>& paths);

	vector<pair<DreamNode*, Point>> intersect_dream_tree(DreamTree tree, Segment seg);
	void add_line_num(DreamNode* node);
	void init_line_num(DreamTree tree);

	vector<Polyline> get_dream_tree_paths(DreamTree tree);
	Polyline get_path(DreamNode* node);

	vector<Segment> get_dream_tree_lines(DreamTree tree, bool opened = false);
	void horizontal_count(DreamNode* node, int& up, int& down);
	void vertical_count(DreamNode* node, int& left, int& right);
}

#endif