﻿#include "OptimizeEngine.h"
#include "RouteEngine.h"

using namespace CableRouter;

#define LINE_GAP	150
#define MAX_OVERLAP	50
#define MERGE_GAP	200
#define SMALL_TURN	500

DreamNodePtr CableRouter::newDreamNode(Point coord)
{
	DreamNodePtr n(new DreamNode());
	n->parent = NULL;
	n->line_num_to_parent = 0;
	n->coord = coord;
	n->children = vector<DreamNodePtr>();
	n->is_device = false;
	return n;
}

vector<DreamNodePtr> CableRouter::getAllNodes(DreamTree tree)
{
	vector<DreamNodePtr> ret;
	set<DreamNodePtr> visited;
	queue<DreamNodePtr> q;
	q.push((DreamNodePtr)tree);
	while (!q.empty())
	{
		DreamNodePtr now = q.front();
		q.pop();

		if (visited.find(now) != visited.end()) continue;

		visited.insert(now);
		ret.push_back(now);

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}
	return ret;
}

//void CableRouter::deleteDreamTree(DreamTree root)
//{
//	vector<DreamNodePtr> chs = root->children;
//	delete root;
//	for (int i = 0; i < chs.size(); i++)
//	{
//		deleteDreamTree(chs[i]);
//	}
//}

void CableRouter::get_manhattan_tree(MapInfo* map, DreamTree tree, vector<Polyline>& exist)
{
	vector<Segment> exist_lines = get_segments_from_polylines(exist);

	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int i = 0; i < all.size(); i++)
	{
		DreamNodePtr now = all[i];

		if (!now->parent) continue;
		if (!now->is_device) continue;

		DreamNodePtr pa = now->parent;
		
		Polyline path = manhattan_connect(map, pa->coord, now->coord, pa->dir_from_parent, exist_lines);

		if (path.size() > 1 && !pa->is_device && pa->parent)
		{
			Vector dir1(path[0], path[1]);
			Vector dir2(pa->parent->coord, pa->coord);
			if (EQUAL(dir1.hx(), 0) && EQUAL(dir2.hx(), 0) ||
				EQUAL(dir1.hy(), 0) && EQUAL(dir2.hy(), 0))
			{
				DreamNodePtr fake = pa;
				pa = fake->parent;
				for (auto ch = pa->children.begin(); ch != pa->children.end(); ch++)
				{
					if ((*ch) == fake)
					{
						pa->children.erase(ch);
						break;
					}
				}
				path[0] = pa->coord;
			}
		}
		for (auto ch = pa->children.begin(); ch != pa->children.end(); ch++)
		{
			if ((*ch) == now)
			{
				pa->children.erase(ch);
				break;
			}
		}
		for (int j = 0; j < (int)path.size() - 1; j++)
		{
			exist_lines.push_back(Segment(path[j], path[j + 1]));
			DreamNodePtr ch = NULL;
			if (j + 1 == (int)path.size() - 1)
				ch = now;
			else
				ch = newDreamNode(path[j + 1]);
			pa->children.push_back(ch);
			ch->parent = pa;
			ch->dir_from_parent = Vector(path[j], path[j + 1]);
			pa = ch;
		}

		if (path.size() > 2 && LEN(exist_lines.back()) < SMALL_TURN)
		{
			vector<DreamNodePtr> children = now->children;
			reset(now->children);
			for (int i = 0; i < children.size(); i++)
			{
				Vector ab(now->parent->coord, now->coord);
				Vector ap(now->parent->coord, children[i]->coord);
				double abap = DOUBLE(ab * ap);
				double abab = DOUBLE(ab * ab);
				double apap = DOUBLE(ap * ap);

				double cos_theta = abap / sqrt(abab) / sqrt(apap);
				if (EQUAL(abab, 0) ||
					EQUAL(apap, 0) ||
					(abap / abab >= 1 && cos_theta > sqrt(2.0) / 2))
				{
					children[i]->parent = now;
					now->children.push_back(children[i]);
				}
				else
				{
					DreamNodePtr mid = newDreamNode(now->parent->coord);
					mid->parent = now;
					mid->dir_from_parent = now->parent->dir_from_parent;
					now->children.push_back(mid);
					children[i]->parent = mid;
					mid->children.push_back(children[i]);
				}
			}
		}
	}
}

bool compare_left_from_down_to_up(pair<DreamNodePtr, DreamNodePtr> pair_a, pair<DreamNodePtr, DreamNodePtr> pair_b)
{
	int aturn;
	DreamNodePtr a = pair_a.first;
	DreamNodePtr b = pair_b.first;
	DreamNodePtr pa = pair_a.second;
	DreamNodePtr pb = pair_b.second;
	if (a->is_device)
		aturn = 0;
	else if (a == pa)
	{
		if (a->parent && (a->parent->coord.hy() - a->coord.hy()) > 0)
			aturn = 1;
		else if (a->parent && (a->parent->coord.hy() - a->coord.hy()) < 0)
			aturn = -1;
		else
			aturn = 0;
	}
	else
	{
		Vector dir(a->coord, a->children[0]->coord);
		if (a->children.size() > 0 && dir.hy() > 0)
			aturn = 1;
		else if (a->children.size() > 0 && dir.hy() < 0)
			aturn = -1;
		else
			aturn = 0;
	}

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b == pb)
	{
		if (b->parent && (b->parent->coord.hy() - b->coord.hy()) > 0)
			bturn = 1;
		else if (b->parent && (b->parent->coord.hy() - b->coord.hy()) < 0)
			bturn = -1;
		else
			bturn = 0;
	}
	else
	{
		Vector dir(b->coord, b->children[0]->coord);
		if (b->children.size() > 0 && dir.hy() > 0)
			bturn = 1;
		else if (b->children.size() > 0 && dir.hy() < 0)
			bturn = -1;
		else
			bturn = 0;
	}

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hx() > b->coord.hx();

	return a->coord.hx() < b->coord.hx();
}

bool compare_right_from_down_to_up(pair<DreamNodePtr, DreamNodePtr> pair_a, pair<DreamNodePtr, DreamNodePtr> pair_b)
{
	int aturn;
	DreamNodePtr a = pair_a.first;
	DreamNodePtr b = pair_b.first;
	DreamNodePtr pa = pair_a.second;
	DreamNodePtr pb = pair_b.second;
	if (a->is_device)
		aturn = 0;
	else if (a == pa)
	{
		if (a->parent && (a->parent->coord.hy() - a->coord.hy()) > 0)
			aturn = 1;
		else if (a->parent && (a->parent->coord.hy() - a->coord.hy()) < 0)
			aturn = -1;
		else
			aturn = 0;
	}
	else
	{
		Vector dir(a->coord, a->children[0]->coord);
		if (a->children.size() > 0 && dir.hy() > 0)
			aturn = 1;
		else if (a->children.size() > 0 && dir.hy() < 0)
			aturn = -1;
		else
			aturn = 0;
	}

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b == pb)
	{
		if (b->parent && (b->parent->coord.hy() - b->coord.hy()) > 0)
			bturn = 1;
		else if (b->parent && (b->parent->coord.hy() - b->coord.hy()) < 0)
			bturn = -1;
		else
			bturn = 0;
	}
	else
	{
		Vector dir(b->coord, b->children[0]->coord);
		if (b->children.size() > 0 && dir.hy() > 0)
			bturn = 1;
		else if (b->children.size() > 0 && dir.hy() < 0)
			bturn = -1;
		else
			bturn = 0;
	}

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hx() < b->coord.hx();

	return a->coord.hx() > b->coord.hx();
}

bool compare_down_from_left_to_right(pair<DreamNodePtr, DreamNodePtr> pair_a, pair<DreamNodePtr, DreamNodePtr> pair_b)
{
	int aturn;
	DreamNodePtr a = pair_a.first;
	DreamNodePtr b = pair_b.first;
	DreamNodePtr pa = pair_a.second;
	DreamNodePtr pb = pair_b.second;
	if (a->is_device)
		aturn = 0;
	else if (a == pa)
	{
		if (a->parent && (a->parent->coord.hx() - a->coord.hx()) > 0)
			aturn = 1;
		else if (a->parent && (a->parent->coord.hx() - a->coord.hx()) < 0)
			aturn = -1;
		else
			aturn = 0;
	}
	else
	{
		Vector dir(a->coord, a->children[0]->coord);
		if (a->children.size() > 0 && dir.hx() > 0)
			aturn = 1;
		else if (a->children.size() > 0 && dir.hx() < 0)
			aturn = -1;
		else
			aturn = 0;
	}

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b == pb)
	{
		if (b->parent && (b->parent->coord.hx() - b->coord.hx()) > 0)
			bturn = 1;
		else if (b->parent && (b->parent->coord.hx() - b->coord.hx()) < 0)
			bturn = -1;
		else
			bturn = 0;
	}
	else
	{
		Vector dir(b->coord, b->children[0]->coord);
		if (b->children.size() > 0 && dir.hx() > 0)
			bturn = 1;
		else if (b->children.size() > 0 && dir.hx() < 0)
			bturn = -1;
		else
			bturn = 0;
	}

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hy() > b->coord.hy();

	return a->coord.hx() < b->coord.hx();
}

bool compare_up_from_left_to_right(pair<DreamNodePtr, DreamNodePtr> pair_a, pair<DreamNodePtr, DreamNodePtr> pair_b)
{
	int aturn;
	DreamNodePtr a = pair_a.first;
	DreamNodePtr b = pair_b.first;
	DreamNodePtr pa = pair_a.second;
	DreamNodePtr pb = pair_b.second;
	if (a->is_device)
		aturn = 0;
	else if (a == pa)
	{
		if (a->parent && (a->parent->coord.hx() - a->coord.hx()) > 0)
			aturn = 1;
		else if (a->parent && (a->parent->coord.hx() - a->coord.hx()) < 0)
			aturn = -1;
		else
			aturn = 0;
	}
	else
	{
		Vector dir(a->coord, a->children[0]->coord);
		if (a->children.size() > 0 && dir.hx() > 0)
			aturn = 1;
		else if (a->children.size() > 0 && dir.hx() < 0)
			aturn = -1;
		else
			aturn = 0;
	}

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b == pb)
	{
		if (b->parent && (b->parent->coord.hx() - b->coord.hx()) > 0)
			bturn = 1;
		else if (b->parent && (b->parent->coord.hx() - b->coord.hx()) < 0)
			bturn = -1;
		else
			bturn = 0;
	}
	else
	{
		Vector dir(b->coord, b->children[0]->coord);
		if (b->children.size() > 0 && dir.hx() > 0)
			bturn = 1;
		else if (b->children.size() > 0 && dir.hx() < 0)
			bturn = -1;
		else
			bturn = 0;
	}

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hy() < b->coord.hy();

	return a->coord.hx() > b->coord.hx();
}

void CableRouter::avoid_coincidence(DreamTree tree)
{
	if (tree == NULL) return;

	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int idx = 0; idx < all.size(); idx++)
	{
		DreamNodePtr now = all[idx];

		if (!now->is_device) continue;

		vector<DreamNodePtr> children = now->children;
		DreamNodePtr now_pa = now->parent;
		if (now->parent) children.push_back(now->parent);

		if (children.size() <= 1) continue;

		vector<vector<DreamNodePtr>> dir_nodes(4);

		for (int i = 0; i < children.size(); i++)
		{

			Vector dir(now->coord, children[i]->coord);
			//if (LEN(dir) <= MAX_OVERLAP) continue;
			if (dir.hy() == 0 && dir.hx() < 0) dir_nodes[N_LEFT].push_back(children[i]);
			else if (EQUAL(dir.hy(), 0) && dir.hx() > 0) dir_nodes[N_RIGHT].push_back(children[i]);
			else if (EQUAL(dir.hx(), 0) && dir.hy() < 0) dir_nodes[N_DOWN].push_back(children[i]);
			else if (EQUAL(dir.hx(), 0) && dir.hy() > 0) dir_nodes[N_UP].push_back(children[i]);
		}

		if (children.size() > 4) {
			printf("ImmuneSystem\n");
			avoid_left(dir_nodes[N_LEFT], now, now_pa, dir_nodes[N_DOWN], dir_nodes[N_UP]);
			avoid_right(dir_nodes[N_RIGHT], now, now_pa, dir_nodes[N_DOWN], dir_nodes[N_UP]);
			avoid_down(dir_nodes[N_DOWN], now, now_pa, dir_nodes[N_LEFT], dir_nodes[N_RIGHT]);
			avoid_up(dir_nodes[N_UP], now, now_pa, dir_nodes[N_LEFT], dir_nodes[N_RIGHT]);
			continue;
		}

		int max_size = 0;
		int max_dir = 0;
		for (int i = 0; i < 4; i++)
		{
			if (max_size < (int)dir_nodes[i].size())
			{
				max_size = dir_nodes[i].size();
				max_dir = i;
			}
		}

		if (max_size == 2)
		{
			switch (max_dir)
			{
			case N_LEFT:
			{
				if (dir_nodes[N_DOWN].size() == 0 && dir_nodes[N_UP].size() != 0)
				{
					avoid_left(		// to down
						dir_nodes[N_LEFT], now, now_pa, 
						dir_nodes[N_DOWN], dir_nodes[N_UP], 
						(int)dir_nodes[N_LEFT].size() - 1);
					avoid_right(	// to up
						dir_nodes[N_RIGHT], now, now_pa, 
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
					avoid_up(		// to right
						dir_nodes[N_UP], now, now_pa, 
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
				}
				else if (dir_nodes[N_DOWN].size() != 0 && dir_nodes[N_UP].size() == 0)
				{
					avoid_left(		// to up
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
					avoid_right(	// to down
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_RIGHT].size() - 1);
					avoid_down(		// to right
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
				}
				else
				{
					int down_size = (int)dir_nodes[N_DOWN].size();
					int up_size = (int)dir_nodes[N_UP].size();
					avoid_left(dir_nodes[N_LEFT], now, now_pa, dir_nodes[N_DOWN], dir_nodes[N_UP]);
					if (dir_nodes[N_DOWN].size() > down_size)
					{
						avoid_right(	// to up
							dir_nodes[N_RIGHT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							0);
						avoid_down(		// to right
							dir_nodes[N_DOWN], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							0);
					}
					else if (dir_nodes[N_UP].size() > up_size)
					{
						avoid_right(	// to down
							dir_nodes[N_RIGHT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							(int)dir_nodes[N_RIGHT].size() - 1);
						avoid_up(		// to right
							dir_nodes[N_UP], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							0);
					}
				}
				break;
			}
			case N_RIGHT:
			{
				if (dir_nodes[N_DOWN].size() == 0 && dir_nodes[N_UP].size() != 0)
				{
					avoid_right(	// to down
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_RIGHT].size() - 1);
					avoid_left(		// to up
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
					avoid_up(		// to left
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_UP].size() - 1);
				}
				else if (dir_nodes[N_DOWN].size() != 0 && dir_nodes[N_UP].size() == 0)
				{
					avoid_right(	// to up
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
					avoid_left(		// to down
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_LEFT].size() - 1);
					avoid_down(		// to left
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_DOWN].size() - 1);
				}
				else
				{
					int down_size = (int)dir_nodes[N_DOWN].size();
					int up_size = (int)dir_nodes[N_UP].size();
					avoid_right(dir_nodes[N_RIGHT], now, now_pa, dir_nodes[N_DOWN], dir_nodes[N_UP]);
					if (dir_nodes[N_DOWN].size() > down_size)
					{
						avoid_left(		// to up
							dir_nodes[N_LEFT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							0);
						avoid_down(		// to left
							dir_nodes[N_DOWN], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							(int)dir_nodes[N_DOWN].size() - 1);
					}
					else if (dir_nodes[N_UP].size() > up_size)
					{
						avoid_left(		// to down
							dir_nodes[N_LEFT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							(int)dir_nodes[N_LEFT].size() - 1);
						avoid_up(		// to left
							dir_nodes[N_UP], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							(int)dir_nodes[N_UP].size() - 1);
					}
				}
				break;
			}
			case N_DOWN:
			{
				if (dir_nodes[N_LEFT].size() == 0 && dir_nodes[N_RIGHT].size() != 0)
				{
					avoid_down(		// to left
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_DOWN].size() - 1);
					avoid_up(		// to right
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
					avoid_right(	// to up
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
				}
				else if (dir_nodes[N_LEFT].size() != 0 && dir_nodes[N_RIGHT].size() == 0)
				{
					avoid_down(		// to right
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
					avoid_up(		// to left
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_UP].size() - 1);
					avoid_left(		// to up
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
				}
				else
				{
					int left_size = (int)dir_nodes[N_LEFT].size();
					int right_size = (int)dir_nodes[N_RIGHT].size();
					avoid_down(dir_nodes[N_DOWN], now, now_pa, dir_nodes[N_LEFT], dir_nodes[N_RIGHT]);
					if (dir_nodes[N_LEFT].size() > left_size)
					{
						avoid_up(		// to right
							dir_nodes[N_UP], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							0);
						avoid_left(		// to up
							dir_nodes[N_LEFT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							0);
					}
					else if (dir_nodes[N_RIGHT].size() > right_size)
					{
						avoid_up(		// to left
							dir_nodes[N_UP], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							(int)dir_nodes[N_UP].size() - 1);
						avoid_right(	// to up
							dir_nodes[N_RIGHT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							0);
					}
				}
				break;
			}
			case N_UP:
			{
				if (dir_nodes[N_LEFT].size() == 0 && dir_nodes[N_RIGHT].size() != 0)
				{
					avoid_up(		// to left
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_UP].size() - 1);
					avoid_down(		// to right
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
					avoid_right(	// to down
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_RIGHT].size() - 1);
				}
				else if (dir_nodes[N_LEFT].size() != 0 && dir_nodes[N_RIGHT].size() == 0)
				{
					avoid_up(		// to right
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
					avoid_down(		// to left
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_DOWN].size() - 1);
					avoid_left(		// to down
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_LEFT].size() - 1);
				}
				else
				{
					int left_size = (int)dir_nodes[N_LEFT].size();
					int right_size = (int)dir_nodes[N_RIGHT].size();
					avoid_up(dir_nodes[N_UP], now, now_pa, dir_nodes[N_LEFT], dir_nodes[N_RIGHT]);
					if (dir_nodes[N_LEFT].size() > left_size)
					{
						avoid_down(		// to right
							dir_nodes[N_DOWN], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							0);
						avoid_left(		// to down
							dir_nodes[N_LEFT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							(int)dir_nodes[N_LEFT].size() - 1);
					}
					else if (dir_nodes[N_RIGHT].size() > right_size)
					{
						avoid_down(		// to left
							dir_nodes[N_DOWN], now, now_pa,
							dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
							(int)dir_nodes[N_DOWN].size() - 1);
						avoid_right(	// to down
							dir_nodes[N_RIGHT], now, now_pa,
							dir_nodes[N_DOWN], dir_nodes[N_UP],
							(int)dir_nodes[N_RIGHT].size() - 1);
					}
				}
				break;
			}
			default:
			{
				break;
			}
			}
		}
		else if (max_size == 3)
		{
			switch (max_dir)
			{
			case N_LEFT:
			{
				if (dir_nodes[N_DOWN].size() == 0 && dir_nodes[N_UP].size() == 0)
				{
					avoid_left(
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						1);
				}
				else if (dir_nodes[N_DOWN].size() == 0)
				{
					avoid_left(		// to down
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_LEFT].size() - 1);
					avoid_down(		// to right
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
				}
				else if (dir_nodes[N_UP].size() == 0)
				{
					avoid_left(		// to up
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
					avoid_up(		// to right
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
				}
				break;
			}
			case N_RIGHT:
			{
				if (dir_nodes[N_DOWN].size() == 0 && dir_nodes[N_UP].size() == 0)
				{
					avoid_right(
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						1);
				}
				else if (dir_nodes[N_DOWN].size() == 0)
				{
					avoid_right(	// to down
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_RIGHT].size() - 1);
					avoid_down(		// to left
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_DOWN].size() - 1);
				}
				else if (dir_nodes[N_UP].size() == 0)
				{
					avoid_right(	// to up
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
					avoid_up(		// to left
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_UP].size() - 1);
				}
				break;
			}
			case N_DOWN:
			{
				if (dir_nodes[N_LEFT].size() == 0 && dir_nodes[N_RIGHT].size() == 0)
				{
					avoid_down(
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						1);
				}
				else if (dir_nodes[N_LEFT].size() == 0)
				{
					avoid_down(		// to left
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_DOWN].size() - 1);
					avoid_left(		// to up
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
				}
				else if (dir_nodes[N_RIGHT].size() == 0)
				{
					avoid_down(		// to right
						dir_nodes[N_DOWN], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
					avoid_right(	// to up
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						0);
				}
				break;
			}
			case N_UP:
			{
				if (dir_nodes[N_LEFT].size() == 0 && dir_nodes[N_RIGHT].size() == 0)
				{
					avoid_up(
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						1);
				}
				else if (dir_nodes[N_LEFT].size() == 0)
				{
					avoid_up(		// to left
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						(int)dir_nodes[N_UP].size() - 1);
					avoid_left(		// to down
						dir_nodes[N_LEFT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_LEFT].size() - 1);
				}
				else if (dir_nodes[N_RIGHT].size() == 0)
				{
					avoid_up(		// to right
						dir_nodes[N_UP], now, now_pa,
						dir_nodes[N_LEFT], dir_nodes[N_RIGHT],
						0);
					avoid_right(	// to down
						dir_nodes[N_RIGHT], now, now_pa,
						dir_nodes[N_DOWN], dir_nodes[N_UP],
						(int)dir_nodes[N_RIGHT].size() - 1);
				}
				break;
			}
			default:
			{
				break;
			}
			}
		}
		else if (max_size == 4)
		{
			printf("route engine\n");
			switch (max_dir)
			{
			case N_LEFT:
			{
				break;
			}
			case N_RIGHT:
			{
			}
			case N_DOWN:
			{
				break;
			}
			case N_UP:
			{
				break;
			}
			default:
			{
				break;
			}
		}
		}
	}
	avoid_coincidence_non_device(tree);
}

void CableRouter::avoid_coincidence_non_device(DreamTree tree)
{
	if (tree == NULL) return;

	// init dream node search tree
	vector<rbush::TreeNode<DreamNodePtr>* > dream_node_nodes;

	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int i = 0; i < all.size(); i++)
	{
		DreamNodePtr now = all[i];

		if (!now->parent) continue;

		Segment seg = Segment(now->parent->coord, now->coord);

		rbush::TreeNode<DreamNodePtr>* node = new rbush::TreeNode<DreamNodePtr>();
		node->data = new DreamNodePtr(now);
		node->bbox.minX = DOUBLE(seg.bbox().xmin() - LINE_GAP);
		node->bbox.minY = DOUBLE(seg.bbox().ymin() - LINE_GAP);
		node->bbox.maxX = DOUBLE(seg.bbox().xmax() + LINE_GAP);
		node->bbox.maxY = DOUBLE(seg.bbox().ymax() + LINE_GAP);
		dream_node_nodes.push_back(node);
	}

	auto dn_tree = new rbush::RBush<DreamNodePtr>(dream_node_nodes);

	// move non-device edges if overlap with other edges

	for (int i = 0; i < all.size(); i++)
	{
		DreamNodePtr now = all[i];

		if (!now->parent) continue;

		if (now->is_device || now->parent->is_device) continue;

		Segment seg_now = Segment(now->parent->coord, now->coord);
		bool overlap = false;

		vector<rbush::TreeNode<DreamNodePtr>* > search_ret;
		if (dn_tree) {
			rbush::Bbox bbox;
			bbox.minX = DOUBLE(seg_now.bbox().xmin());
			bbox.minY = DOUBLE(seg_now.bbox().ymin());
			bbox.maxX = DOUBLE(seg_now.bbox().xmax());
			bbox.maxY = DOUBLE(seg_now.bbox().ymax());
			auto feedback = dn_tree->search(bbox);
			if (feedback) {
				for (auto fit = feedback->begin(); fit != feedback->end(); ++fit) {
					search_ret.push_back(*fit);
				}
			}
			delete feedback;
		}
		for (auto sit = search_ret.begin(); sit != search_ret.end(); sit++)
		{
			DreamNodePtr sn = *(*sit)->data;
			if (sn == now) continue;
			Segment s = Segment(sn->parent->coord, sn->coord);
			if (!CGAL::do_intersect(s, seg_now)) continue;
			CGAL::Object result = CGAL::intersection(s, seg_now);
			Segment ss;
			if (CGAL::assign(ss, result))
			{
				overlap = true;
				break;
			}
		}

		if (overlap)
		{
			if (seg_now.is_horizontal())
			{
				if (now->children.size() > 0 && now->children[0]->coord.hy() < now->coord.hy())
				{
					now->coord = Point(now->coord.hx(), now->coord.hy() - LINE_GAP);
					now->parent->coord = Point(now->parent->coord.hx(), now->parent->coord.hy() - LINE_GAP);
				}
				if (now->children.size() > 0 && now->children[0]->coord.hy() > now->coord.hy())
				{
					now->coord = Point(now->coord.hx(), now->coord.hy() + LINE_GAP);
					now->parent->coord = Point(now->parent->coord.hx(), now->parent->coord.hy() + LINE_GAP);
				}
			}
			if (seg_now.is_vertical())
			{
				if (now->children.size() > 0 && now->children[0]->coord.hx() < now->coord.hx())
				{
					now->coord = Point(now->coord.hx() - LINE_GAP, now->coord.hy());
					now->parent->coord = Point(now->parent->coord.hx() - LINE_GAP, now->parent->coord.hy());
				}
				if (now->children.size() > 0 && now->children[0]->coord.hx() > now->coord.hx())
				{
					now->coord = Point(now->coord.hx() + LINE_GAP, now->coord.hy());
					now->parent->coord = Point(now->parent->coord.hx() + LINE_GAP, now->parent->coord.hy());
				}
			}
		}

	}
	delete dn_tree;
	for (int i = 0; i < dream_node_nodes.size(); i++)
	{
		delete dream_node_nodes[i]->data;
		delete dream_node_nodes[i];
		dream_node_nodes[i] = NULL;
	}
}

void CableRouter::avoid_left(
	vector<DreamNodePtr>& left, DreamNodePtr now, DreamNodePtr now_pa,
	vector<DreamNodePtr>& down, vector<DreamNodePtr>& up, int fix)
{
	if (left.size() <= 1) return;

	vector<pair<DreamNodePtr, DreamNodePtr>> pair_nodes;
	for (int i = 0; i < left.size(); i++)
	{
		pair_nodes.push_back(make_pair(left[i], now_pa));
	}
	sort(pair_nodes.begin(), pair_nodes.end(), compare_left_from_down_to_up);
	for (int i = 0; i < pair_nodes.size(); i++)
	{
		left[i] = pair_nodes[i].first;
	}
	if (fix == -1)
	{
		for (int i = 0; i < left.size(); i++)
		{
			if (left[i]->is_device)
			{
				fix = i;
				break;
			}
		}
		if (fix == -1)
		{
			fix = left.size() / 2;
		}
	}
	for (int i = 0; i < left.size(); i++)
	{
		if (i == fix) continue;

		DreamNodePtr mid = newDreamNode(Point(now->coord.hx(), now->coord.hy() + (i - fix) * 1.0 * LINE_GAP));

		if (left[i] == now_pa)
		{
			for (auto ch = now_pa->children.begin(); ch != now_pa->children.end(); ch++)
			{
				if ((*ch) == now)
				{
					now_pa->children.erase(ch);
					break;
				}
			}
			now->parent = mid;
			mid->children.push_back(now);
		}
		else
		{
			for (auto ch = now->children.begin(); ch != now->children.end(); ch++)
			{
				if ((*ch) == left[i])
				{
					now->children.erase(ch);
					break;
				}
			}
			mid->parent = now;
			now->children.push_back(mid);
		}

		if (i < fix) down.push_back(mid);
		else if (i > fix) up.push_back(mid);

		Point dd(left[i]->coord.hx(), left[i]->coord.hy() + (i - fix) * 1.0 * LINE_GAP);
		if (left[i]->is_device)
		{
			DreamNodePtr mid2 = newDreamNode(dd);
			if (left[i] == now_pa)
			{
				mid->parent = mid2;
				mid2->children.push_back(mid);
			}
			else
			{
				mid2->parent = mid;
				mid->children.push_back(mid2);
			}
			mid = mid2;
		}
		else
		{
			left[i]->coord = dd;
		}
		if (left[i] == now_pa)
		{
			mid->parent = left[i];
			left[i]->children.push_back(mid);
		}
		else
		{
			left[i]->parent = mid;
			mid->children.push_back(left[i]);
		}
	}
}

void CableRouter::avoid_right(
	vector<DreamNodePtr>& right, DreamNodePtr now, DreamNodePtr now_pa,
	vector<DreamNodePtr>& down, vector<DreamNodePtr>& up, int fix)
{
	if (right.size() <= 1) return;

	vector<pair<DreamNodePtr, DreamNodePtr>> pair_nodes;
	for (int i = 0; i < right.size(); i++)
	{
		pair_nodes.push_back(make_pair(right[i], now_pa));
	}
	sort(pair_nodes.begin(), pair_nodes.end(), compare_right_from_down_to_up);
	for (int i = 0; i < pair_nodes.size(); i++)
	{
		right[i] = pair_nodes[i].first;
	}
	if (fix == -1)
	{
		for (int i = 0; i < right.size(); i++)
		{
			if (right[i]->is_device)
			{
				fix = i;
				break;
			}
		}
		if (fix == -1)
		{
			fix = right.size() / 2;
		}
	}
	for (int i = 0; i < right.size(); i++)
	{
		if (i == fix) continue;

		DreamNodePtr mid = newDreamNode(Point(now->coord.hx(), now->coord.hy() + (i - fix) * 1.0 * LINE_GAP));

		if (right[i] == now_pa)
		{
			for (auto ch = now_pa->children.begin(); ch != now_pa->children.end(); ch++)
			{
				if ((*ch) == now)
				{
					now_pa->children.erase(ch);
					break;
				}
			}
			now->parent = mid;
			mid->children.push_back(now);
		}
		else
		{
			for (auto ch = now->children.begin(); ch != now->children.end(); ch++)
			{
				if ((*ch) == right[i])
				{
					now->children.erase(ch);
					break;
				}
			}
			mid->parent = now;
			now->children.push_back(mid);
		}

		if (i < fix) down.push_back(mid);
		else if (i > fix) up.push_back(mid);

		Point dd(right[i]->coord.hx(), right[i]->coord.hy() + (i - fix) * 1.0 * LINE_GAP);
		if (right[i]->is_device)
		{
			DreamNodePtr mid2 = newDreamNode(dd);
			if (right[i] == now_pa)
			{
				mid->parent = mid2;
				mid2->children.push_back(mid);
			}
			else
			{
				mid2->parent = mid;
				mid->children.push_back(mid2);
			}
			mid = mid2;
		}
		else
		{
			right[i]->coord = dd;
		}
		if (right[i] == now_pa)
		{
			mid->parent = right[i];
			right[i]->children.push_back(mid);
		}
		else
		{
			right[i]->parent = mid;
			mid->children.push_back(right[i]);
		}
	}
}

void CableRouter::avoid_down(
	vector<DreamNodePtr>& down, DreamNodePtr now, DreamNodePtr now_pa,
	vector<DreamNodePtr>& left, vector<DreamNodePtr>& right, int fix)
{
	if (down.size() <= 1) return;

	vector<pair<DreamNodePtr, DreamNodePtr>> pair_nodes;
	for (int i = 0; i < down.size(); i++)
	{
		pair_nodes.push_back(make_pair(down[i], now_pa));
	}
	sort(pair_nodes.begin(), pair_nodes.end(), compare_down_from_left_to_right);
	for (int i = 0; i < pair_nodes.size(); i++)
	{
		down[i] = pair_nodes[i].first;
	}
	if (fix == -1)
	{
		for (int i = 0; i < down.size(); i++)
		{
			if (down[i]->is_device)
			{
				fix = i;
				break;
			}
		}
		if (fix == -1)
		{
			fix = down.size() / 2;
		}
	}
	for (int i = 0; i < down.size(); i++)
	{
		if (i == fix) continue;

		DreamNodePtr mid = newDreamNode(Point(now->coord.hx() + (i - fix) * 1.0 * LINE_GAP, now->coord.hy()));

		if (down[i] == now_pa)
		{
			for (auto ch = now_pa->children.begin(); ch != now_pa->children.end(); ch++)
			{
				if ((*ch) == now)
				{
					now_pa->children.erase(ch);
					break;
				}
			}
			now->parent = mid;
			mid->children.push_back(now);
		}
		else
		{
			for (auto ch = now->children.begin(); ch != now->children.end(); ch++)
			{
				if ((*ch) == down[i])
				{
					now->children.erase(ch);
					break;
				}
			}
			mid->parent = now;
			now->children.push_back(mid);
		}

		if (i < fix) left.push_back(mid);
		else if (i > fix) right.push_back(mid);

		Point dd(down[i]->coord.hx() + (i - fix) * 1.0 * LINE_GAP, down[i]->coord.hy());
		if (down[i]->is_device)
		{
			DreamNodePtr mid2 = newDreamNode(dd);
			if (down[i] == now_pa)
			{
				mid->parent = mid2;
				mid2->children.push_back(mid);
			}
			else
			{
				mid2->parent = mid;
				mid->children.push_back(mid2);
			}
			mid = mid2;
		}
		else
		{
			down[i]->coord = dd;
		}
		if (down[i] == now_pa)
		{
			mid->parent = down[i];
			down[i]->children.push_back(mid);
		}
		else
		{
			down[i]->parent = mid;
			mid->children.push_back(down[i]);
		}
	}
}

void CableRouter::avoid_up(
	vector<DreamNodePtr>& up, DreamNodePtr now, DreamNodePtr now_pa,
	vector<DreamNodePtr>& left, vector<DreamNodePtr>& right, int fix)
{
	if (up.size() <= 1) return;

	vector<pair<DreamNodePtr, DreamNodePtr>> pair_nodes;
	for (int i = 0; i < up.size(); i++)
	{
		pair_nodes.push_back(make_pair(up[i], now_pa));
	}
	sort(pair_nodes.begin(), pair_nodes.end(), compare_up_from_left_to_right);
	for (int i = 0; i < pair_nodes.size(); i++)
	{
		up[i] = pair_nodes[i].first;
	}
	if (fix == -1)
	{
		for (int i = 0; i < up.size(); i++)
		{
			if (up[i]->is_device)
			{
				fix = i;
				break;
			}
		}
		if (fix == -1)
		{
			fix = up.size() / 2;
		}
	}
	for (int i = 0; i < up.size(); i++)
	{
		if (i == fix) continue;

		DreamNodePtr mid = newDreamNode(Point(now->coord.hx() + (i - fix) * 1.0 * LINE_GAP, now->coord.hy()));

		if (up[i] == now_pa)
		{
			for (auto ch = now_pa->children.begin(); ch != now_pa->children.end(); ch++)
			{
				if ((*ch) == now)
				{
					now_pa->children.erase(ch);
					break;
				}
			}
			now->parent = mid;
			mid->children.push_back(now);
		}
		else
		{
			for (auto ch = now->children.begin(); ch != now->children.end(); ch++)
			{
				if ((*ch) == up[i])
				{
					now->children.erase(ch);
					break;
				}
			}
			mid->parent = now;
			now->children.push_back(mid);
		}

		if (i < fix) left.push_back(mid);
		else if (i > fix) right.push_back(mid);

		Point dd(up[i]->coord.hx() + (i - fix) * 1.0 * LINE_GAP, up[i]->coord.hy());
		if (up[i]->is_device)
		{
			DreamNodePtr mid2 = newDreamNode(dd);
			if (up[i] == now_pa)
			{
				mid->parent = mid2;
				mid2->children.push_back(mid);
			}
			else
			{
				mid2->parent = mid;
				mid->children.push_back(mid2);
			}
			mid = mid2;
		}
		else
		{
			up[i]->coord = dd;
		}
		if (up[i] == now_pa)
		{
			mid->parent = up[i];
			up[i]->children.push_back(mid);
		}
		else
		{
			up[i]->parent = mid;
			mid->children.push_back(up[i]);
		}
	}
}

vector<pair<DreamNodePtr, Point>> CableRouter::intersect_dream_tree(DreamTree tree, Segment seg)
{
	vector<pair<DreamNodePtr, Point>> res;

	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int i = 0; i < all.size(); i++)
	{
		DreamNodePtr now = all[i];

		if (!now->parent) continue;

		Segment seg_now = Segment(now->coord, now->parent->coord);
		Rectangle rec_now;

		if (seg_now.is_horizontal())
		{
			double minX = min(DOUBLE(now->coord.hx()), DOUBLE(now->parent->coord.hx()));
			double minY = DOUBLE(now->coord.hy()) - MERGE_GAP;
			double maxX = max(DOUBLE(now->coord.hx()), DOUBLE(now->parent->coord.hx()));
			double maxY = DOUBLE(now->coord.hy()) + MERGE_GAP;
			rec_now = Rectangle(minX, minY, maxX, maxY);
		}
		else if (seg_now.is_vertical())
		{
			double minY = min(DOUBLE(now->coord.hy()), DOUBLE(now->parent->coord.hy()));
			double minX = DOUBLE(now->coord.hx()) - MERGE_GAP;
			double maxY = max(DOUBLE(now->coord.hy()), DOUBLE(now->parent->coord.hy()));
			double maxX = DOUBLE(now->coord.hx()) + MERGE_GAP;
			rec_now = Rectangle(minX, minY, maxX, maxY);
		}
		else
		{
			double minX = min(DOUBLE(now->coord.hx()), DOUBLE(now->parent->coord.hx()));
			double minY = min(DOUBLE(now->coord.hy()), DOUBLE(now->parent->coord.hy()));
			double maxX = max(DOUBLE(now->coord.hx()), DOUBLE(now->parent->coord.hx()));
			double maxY = max(DOUBLE(now->coord.hy()), DOUBLE(now->parent->coord.hy()));
			rec_now = Rectangle(minX, minY, maxX, maxY);
		}

		if (CGAL::do_intersect(rec_now, seg))
		{
			CGAL::Object result = CGAL::intersection(rec_now, seg);
			Point pp;
			Segment ss;
			if (CGAL::assign(pp, result))
			{
				res.push_back(make_pair(now, pp));
			}
			else if (CGAL::assign(ss, result))
			{
				res.push_back(make_pair(now, ss.source()));
				res.push_back(make_pair(now, ss.target()));
			}
		}
	}

	return res;
}

void CableRouter::add_line_num(DreamNodePtr node)
{
	while (node->parent != NULL)
	{
		node->line_num_to_parent++;
		node = node->parent;
	}
}

void CableRouter::init_line_num(DreamTree tree)
{
	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int i = 0; i < all.size(); i++)
	{
		DreamNodePtr now = all[i];

		if (!now->parent) continue;

		if (now->is_device)
		{
			add_line_num(now);
		}
	}
}

DreamTree CableRouter::merge_to_a_tree(vector<Polyline>& paths)
{
	if (paths.size() == 0) return NULL;
	if (paths[0].size() == 0) return NULL;

	// init tree
	DreamTree tree = newDreamNode(paths[0][0]);
	DreamNodePtr parent = tree;
	for (int i = 1; i < paths[0].size(); i++)
	{
		DreamNodePtr no = newDreamNode(paths[0][i]);
		no->parent = parent;
		if (i == (int)paths[0].size() - 1)
			no->is_device = true;
		parent->children.push_back(no);
		parent = no;
	}

	// add paths to tree
	for (int i = 1; i < paths.size(); i++)
	{
		DreamNodePtr child = NULL;
		for (int j = (int)paths[i].size() - 1; j >= 1; j--)
		{
			Point old = paths[i][j];
			Point young = paths[i][j - 1];

			DreamNodePtr old_node = newDreamNode(old);
			if (j == (int)paths[i].size() - 1)
				old_node->is_device = true;
			if (child != NULL)
			{
				old_node->children.push_back(child);
				child->parent = old_node;
			}

			Segment seg(old, young);
			auto intersections = intersect_dream_tree(tree, seg);
			if (intersections.size() == 0)
			{
				child = old_node;
				continue;
			}

			int u = -1;
			double MIN = CR_INF;
			for (int k = 0; k < intersections.size(); k++)
			{
				double dis = DIST(intersections[k].second, old);
				if (dis < MIN)
				{
					u = k;
					MIN = dis;
				}
			}
			DreamNodePtr inter_ch = intersections[u].first;
			DreamNodePtr inter_pa = inter_ch->parent;
			Point inter_point = intersections[u].second;

			if (inter_point == old)
			{
				if (old == inter_ch->coord)
				{
					if (old_node->is_device)
						inter_ch->is_device = true;
					for (int c = 0; c < old_node->children.size(); c++)
					{
						old_node->children[c]->parent = inter_ch;
						inter_ch->children.push_back(old_node->children[c]);
					}
					//deleteDreamTree(old_node);
				}
				else if (old == inter_pa->coord)
				{
					if (old_node->is_device)
						inter_pa->is_device = true;
					for (int c = 0; c < old_node->children.size(); c++)
					{
						old_node->children[c]->parent = inter_pa;
						inter_pa->children.push_back(old_node->children[c]);
					}
					//deleteDreamTree(old_node);
				}
				else if (CGAL::collinear(inter_ch->coord, old, inter_pa->coord))
				{
					for (auto ch = inter_pa->children.begin(); ch != inter_pa->children.end(); ch++)
					{
						if ((*ch) == inter_ch)
						{
							inter_pa->children.erase(ch);
							break;
						}
					}
					inter_ch->parent = old_node;
					old_node->children.push_back(inter_ch);
					old_node->parent = inter_pa;
					inter_pa->children.push_back(old_node);
				}
				else
				{
					inter_point = project_point_to_segment(old, Segment(inter_ch->coord, inter_pa->coord));

					if (inter_point == inter_ch->coord)
					{
						old_node->parent = inter_ch;
						inter_ch->children.push_back(old_node);
					}
					else if (inter_point == inter_pa->coord)
					{
						old_node->parent = inter_pa;
						inter_pa->children.push_back(old_node);
					}
					else
					{
						DreamNodePtr inter = newDreamNode(inter_point);
						old_node->parent = inter;
						inter->children.push_back(old_node);
						for (auto ch = inter_pa->children.begin(); ch != inter_pa->children.end(); ch++)
						{
							if ((*ch) == inter_ch)
							{
								inter_pa->children.erase(ch);
								break;
							}
						}
						inter->parent = inter_pa;
						inter_pa->children.push_back(inter);
						inter_ch->parent = inter;
						inter->children.push_back(inter_ch);
					}
				}
			}
			else
			{
				if (inter_point == inter_ch->coord)
				{
					old_node->parent = inter_ch;
					inter_ch->children.push_back(old_node);
				}
				else if (inter_point == inter_pa->coord)
				{
					old_node->parent = inter_pa;
					inter_pa->children.push_back(old_node);
				}
				else
				{
					Point project_point = project_point_to_segment(inter_point, Segment(inter_ch->coord, inter_pa->coord));

					if (project_point == inter_ch->coord)
					{
						DreamNodePtr inter = newDreamNode(inter_point);
						old_node->parent = inter;
						inter->children.push_back(old_node);
						inter->parent = inter_ch;
						inter_ch->children.push_back(inter);
					}
					else if (project_point == inter_pa->coord)
					{
						DreamNodePtr inter = newDreamNode(inter_point);
						old_node->parent = inter;
						inter->children.push_back(old_node);
						inter->parent = inter_pa;
						inter_pa->children.push_back(inter);
					}
					else
					{
						DreamNodePtr project = newDreamNode(project_point);
						old_node->parent = project;
						project->children.push_back(old_node);
						for (auto ch = inter_pa->children.begin(); ch != inter_pa->children.end(); ch++)
						{
							if ((*ch) == inter_ch)
							{
								inter_pa->children.erase(ch);
								break;
							}
						}
						project->parent = inter_pa;
						inter_pa->children.push_back(project);
						inter_ch->parent = project;
						project->children.push_back(inter_ch);
					}
				}
			}
			break;
		}
	}
	init_line_num(tree);
	return tree;
}

vector<Segment> CableRouter::get_dream_tree_lines(DreamTree tree, bool opened)
{
	vector<Segment> res;
	if (!tree) return res;

	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int idx = 0; idx < all.size(); idx++)
	{
		DreamNodePtr now = all[idx];

		if (!now->parent) continue;

		Segment ss(now->coord, now->parent->coord);

		if (!opened || now->line_num_to_parent <= 1)
		{
			res.push_back(ss);
		}
		else if (ss.is_horizontal())
		{
			int turn_up = 0;
			int turn_down = 0;
			int go_ahead = 0;
			horizontal_count(now, turn_up, turn_down);
			go_ahead = now->line_num_to_parent - turn_up - turn_down;
			for (int i = 0; i < go_ahead; i++)
			{
				int off = i - go_ahead / 2;
				res.push_back(Segment(
					Point(now->coord.hx(), now->coord.hy() + off * LINE_GAP), 
					Point(now->parent->coord.hx(), now->parent->coord.hy() + off * LINE_GAP)
				));
			}
			for (int i = 0; i < turn_up; i++)
			{
				int off = go_ahead - go_ahead / 2 + i;
				res.push_back(Segment(
					Point(now->coord.hx(), now->coord.hy() + off * LINE_GAP),
					Point(now->parent->coord.hx(), now->parent->coord.hy() + off * LINE_GAP)
				));
			}
			for (int i = 0; i < turn_down; i++)
			{
				int off = 0 - go_ahead / 2 - (go_ahead == 0 ? 0 : 1) - i;
				//int off = 0 - go_ahead / 2 - 1 - i;
				res.push_back(Segment(
					Point(now->coord.hx(), now->coord.hy() + off * LINE_GAP),
					Point(now->parent->coord.hx(), now->parent->coord.hy() + off * LINE_GAP)
				));
			}
		}
		else if (ss.is_vertical())
		{
			int trun_left = 0;
			int trun_right = 0;
			int go_ahead = 0;
			vertical_count(now, trun_left, trun_right);
			go_ahead = now->line_num_to_parent - trun_left - trun_right;
			for (int i = 0; i < go_ahead; i++)
			{
				double off = i - go_ahead / 2;
				res.push_back(Segment(
					Point(now->coord.hx() + off * LINE_GAP, now->coord.hy()),
					Point(now->parent->coord.hx() + off * LINE_GAP, now->parent->coord.hy())
				));
			}
			for (int i = 0; i < trun_right; i++)
			{
				double off = go_ahead - go_ahead / 2 + i;
				res.push_back(Segment(
					Point(now->coord.hx() + off * LINE_GAP, now->coord.hy()),
					Point(now->parent->coord.hx() + off * LINE_GAP, now->parent->coord.hy())
				));
			}
			for (int i = 0; i < trun_left; i++)
			{
				double off = 0 - go_ahead / 2 - (go_ahead == 0 ? 0 : 1) - i;
				//double off = 0 - go_ahead / 2 - 1 - i;
				res.push_back(Segment(
					Point(now->coord.hx() + off * LINE_GAP, now->coord.hy()),
					Point(now->parent->coord.hx() + off * LINE_GAP, now->parent->coord.hy())
				));
			}
		}
		else
		{
			res.push_back(ss);
		}
	}

	return res;
}

vector<Polyline> CableRouter::get_dream_tree_paths(DreamTree tree)
{
	vector<Polyline> res;
	vector<DreamNodePtr> all = getAllNodes(tree);
	for (int i = 0; i < all.size(); i++)
	{
		DreamNodePtr now = all[i];
		
		if (!now->parent) continue;

		if (now->is_device)
		{
			Polyline pl = get_path(now);
			if (pl.size() >= 2)
			{
				res.push_back(pl);
			}
		}
	}
	return res;
}

Polyline CableRouter::get_path(DreamNodePtr node)
{
	Polyline res;
	DreamNodePtr now = node;
	do
	{
		res.push_back(now->coord);
		now = now->parent;
	} while (now != NULL && !now->is_device);
	if (now != NULL && now->is_device)
	{
		res.push_back(now->coord);
	}
	reverse(res.begin(), res.end());
	return res;
}

void CableRouter::horizontal_count(DreamNodePtr now, int& up, int& down)
{
	if (now->line_num_to_parent <= 1)
		return;

	for (int i = 0; i < now->children.size(); i++)
	{
		DreamNodePtr next = now->children[i];
		if (EQUAL(now->coord.hy(), next->coord.hy()))
		{
			horizontal_count(next, up, down);
		}
		else if (EQUAL(now->coord.hx(), next->coord.hx()))
		{
			if (next->coord.hy() > now->coord.hy())
			{
				up += next->line_num_to_parent;
			}
			else
			{
				down += next->line_num_to_parent;
			}
		}
	}
}

void CableRouter::vertical_count(DreamNodePtr now, int& left, int& right)
{
	if (now->line_num_to_parent <= 1)
		return;

	for (int i = 0; i < now->children.size(); i++)
	{
		DreamNodePtr next = now->children[i];
		if (EQUAL(now->coord.hx(), next->coord.hx()))
		{
			vertical_count(next, left, right);
		}
		else if (EQUAL(now->coord.hy(), next->coord.hy()))
		{
			if (next->coord.hx() > now->coord.hx())
			{
				right += next->line_num_to_parent;
			}
			else
			{
				left += next->line_num_to_parent;
			}
		}
	}
}