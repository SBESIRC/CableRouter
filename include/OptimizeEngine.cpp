#include "OptimizeEngine.h"
#include "RouteEngine.h"

using namespace CableRouter;

#define LINE_GAP	150
#define MAX_OVERLAP	50
#define MERGE_GAP	200
#define SMALL_TURN	500

DreamNode* CableRouter::newDreamNode(Point coord)
{
	DreamNode* n = new DreamNode();
	n->parent = NULL;
	n->line_num_to_parent = 0;
	n->coord = coord;
	n->children = vector<DreamNode*>();
	n->is_device = false;
	all_dream_nodes.push_back(n);
	return n;
}

void CableRouter::deleteAllDreamNodes()
{
	for (int i = 0; i < all_dream_nodes.size(); i++)
	{
		delete all_dream_nodes[i];
	}
	reset(all_dream_nodes);
}

//void CableRouter::deleteDreamTree(DreamTree root)
//{
//	vector<DreamNode*> chs = root->children;
//	delete root;
//	for (int i = 0; i < chs.size(); i++)
//	{
//		deleteDreamTree(chs[i]);
//	}
//}

void CableRouter::get_manhattan_tree(MapInfo* map, DreamTree tree, vector<Polyline>& exist)
{
	vector<Segment> exist_lines = get_segments_from_polylines(exist);

	queue<DreamNode*> q;
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		DreamNode* pa = now->parent;
		q.pop();


		Polyline path = manhattan_connect(map, pa->coord, now->coord, pa->dir_from_parent, exist_lines);

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
			DreamNode* ch = NULL;
			if (j + 1 == (int)path.size() - 1)
				ch = now;
			else
				ch = newDreamNode(path[j + 1]);
			pa->children.push_back(ch);
			ch->parent = pa;
			ch->dir_from_parent = Vector(path[j], path[j + 1]);
			pa = ch;
		}
				

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
		if (path.size() > 2 && LEN(exist_lines.back()) < SMALL_TURN)
		{
			vector<DreamNode*> children = now->children;
			reset(now->children);
			for (int i = 0; i < children.size(); i++)
			{
				Vector ab(now->parent->coord, now->coord);
				Vector ap(now->parent->coord, children[i]->coord);
				double abap = DOUBLE(ab * ap);
				double abab = DOUBLE(ab * ab);
				double apap = DOUBLE(ap * ap);

				if (abab == 0 ||
					apap == 0 ||
					(abap / sqrt(abab) >= 1 && abap / sqrt(abab) / sqrt(apap) > sqrt(2.0) / 2))
				{
					children[i]->parent = now;
					now->children.push_back(children[i]);
				}
				else
				{
					DreamNode* mid = newDreamNode(now->parent->coord);
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

bool compare_left_from_down_to_up(DreamNode* a, DreamNode* b)
{
	int aturn;
	if (a->is_device)
		aturn = 0;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hy() > 0)
		aturn = 1;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hy() < 0)
		aturn = -1;
	else
		aturn = 0;

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hy() > 0)
		bturn = 1;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hy() < 0)
		bturn = -1;
	else
		bturn = 0;

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hx() > b->coord.hx();

	return a->coord.hx() < b->coord.hx();
}

bool compare_right_from_down_to_up(DreamNode* a, DreamNode* b)
{
	int aturn;
	if (a->is_device)
		aturn = 0;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hy() > 0)
		aturn = 1;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hy() < 0)
		aturn = -1;
	else
		aturn = 0;

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hy() > 0)
		bturn = 1;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hy() < 0)
		bturn = -1;
	else
		bturn = 0;

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hx() < b->coord.hx();

	return a->coord.hx() > b->coord.hx();
}

bool compare_down_from_left_to_right(DreamNode* a, DreamNode* b)
{
	int aturn;
	if (a->is_device)
		aturn = 0;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hx() > 0)
		aturn = 1;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hx() < 0)
		aturn = -1;
	else
		aturn = 0;

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hx() > 0)
		bturn = 1;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hx() < 0)
		bturn = -1;
	else
		bturn = 0;

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hy() > b->coord.hy();

	return a->coord.hx() < b->coord.hx();
}

bool compare_up_from_left_to_right(DreamNode* a, DreamNode* b)
{
	int aturn;
	if (a->is_device)
		aturn = 0;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hx() > 0)
		aturn = 1;
	else if (a->children.size() > 0 && a->children[0]->dir_from_parent.hx() < 0)
		aturn = -1;
	else
		aturn = 0;

	int bturn;
	if (b->is_device)
		bturn = 0;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hx() > 0)
		bturn = 1;
	else if (b->children.size() > 0 && b->children[0]->dir_from_parent.hx() < 0)
		bturn = -1;
	else
		bturn = 0;

	if (aturn != bturn) return aturn < bturn;

	if (aturn == -1) return a->coord.hy() < b->coord.hy();

	return a->coord.hx() > b->coord.hx();
}

void CableRouter::avoid_coincidence(DreamTree tree)
{
	if (tree == NULL) return;

	queue<DreamNode*> q;
	q.push((DreamNode*)tree);
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}


		if (!now->is_device) continue;

		vector<DreamNode*> children = now->children;
		DreamNode* now_pa = now->parent;
		if (now->parent) children.push_back(now->parent);

		if (children.size() <= 1) continue;

		vector<vector<DreamNode*>> dir_nodes(4);

		for (int i = 0; i < children.size(); i++)
		{

			Vector dir(now->coord, children[i]->coord);
			//if (LEN(dir) <= MAX_OVERLAP) continue;
			if (dir.hy() == 0 && dir.hx() < 0) dir_nodes[N_LEFT].push_back(children[i]);
			if (dir.hy() == 0 && dir.hx() > 0) dir_nodes[N_RIGHT].push_back(children[i]);
			if (dir.hx() == 0 && dir.hy() < 0) dir_nodes[N_DOWN].push_back(children[i]);
			if (dir.hx() == 0 && dir.hy() > 0) dir_nodes[N_UP].push_back(children[i]);
		}

		if (children.size() > 4) {
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
						dir_nodes[N_RIGHT], now, now_pa,
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
							dir_nodes[N_RIGHT], now, now_pa,
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
						dir_nodes[N_RIGHT], now, now_pa,
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
							dir_nodes[N_RIGHT], now, now_pa,
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
						dir_nodes[N_RIGHT], now, now_pa,
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
						dir_nodes[N_RIGHT], now, now_pa,
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
			printf("route engine ƒ„’“∏ˆ∞‡…œ∞…\n");
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

	vector<rbush::TreeNode<DreamNode*>* > dream_node_nodes;

	queue<DreamNode*> q;
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

		Segment seg = Segment(now->parent->coord, now->coord);

		rbush::TreeNode<DreamNode*>* node = new rbush::TreeNode<DreamNode*>();
		node->data = new DreamNode*(now);
		node->bbox.minX = DOUBLE(seg.bbox().xmin());
		node->bbox.minY = DOUBLE(seg.bbox().ymin());
		node->bbox.maxX = DOUBLE(seg.bbox().xmax());
		node->bbox.maxY = DOUBLE(seg.bbox().ymax());
		dream_node_nodes.push_back(node);

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}

	auto dn_tree = new rbush::RBush<DreamNode*>(dream_node_nodes);

	// move non-device edges if overlap with other edges

	reset(q);
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}

		if (now->is_device || now->parent->is_device) continue;

		Segment seg_now = Segment(now->parent->coord, now->coord);
		bool overlap = false;

		vector<rbush::TreeNode<DreamNode*>* > search_ret;
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
		}
		for (auto sit = search_ret.begin(); sit != search_ret.end(); sit++)
		{
			DreamNode* sn = *(*sit)->data;
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
		delete dream_node_nodes[i];
	}
}

void CableRouter::avoid_left(
	vector<DreamNode*>& left, DreamNode* now, DreamNode* now_pa,
	vector<DreamNode*>& down, vector<DreamNode*>& up, int fix)
{
	if (left.size() <= 1) return;

	sort(left.begin(), left.end(), compare_left_from_down_to_up);
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

		DreamNode* mid = newDreamNode(Point(now->coord.hx(), now->coord.hy() + (i - fix) * 1.0 * LINE_GAP));

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
		if (i > fix) up.push_back(mid);

		Point dd(left[i]->coord.hx(), left[i]->coord.hy() + (i - fix) * 1.0 * LINE_GAP);
		if (left[i]->is_device)
		{
			DreamNode* mid2 = newDreamNode(dd);
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
	vector<DreamNode*>& right, DreamNode* now, DreamNode* now_pa,
	vector<DreamNode*>& down, vector<DreamNode*>& up, int fix)
{
	if (right.size() <= 1) return;

	sort(right.begin(), right.end(), compare_right_from_down_to_up);
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

		DreamNode* mid = newDreamNode(Point(now->coord.hx(), now->coord.hy() + (i - fix) * 1.0 * LINE_GAP));

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
		if (i > fix) up.push_back(mid);

		Point dd(right[i]->coord.hx(), right[i]->coord.hy() + (i - fix) * 1.0 * LINE_GAP);
		if (right[i]->is_device)
		{
			DreamNode* mid2 = newDreamNode(dd);
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
	vector<DreamNode*>& down, DreamNode* now, DreamNode* now_pa,
	vector<DreamNode*>& left, vector<DreamNode*>& right, int fix)
{
	if (down.size() <= 1) return;

	sort(down.begin(), down.end(), compare_down_from_left_to_right);
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

		DreamNode* mid = newDreamNode(Point(now->coord.hx() + (i - fix) * 1.0 * LINE_GAP, now->coord.hy()));

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
		if (i > fix) right.push_back(mid);

		Point dd(down[i]->coord.hx() + (i - fix) * 1.0 * LINE_GAP, down[i]->coord.hy());
		if (down[i]->is_device)
		{
			DreamNode* mid2 = newDreamNode(dd);
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
	vector<DreamNode*>& up, DreamNode* now, DreamNode* now_pa,
	vector<DreamNode*>& left, vector<DreamNode*>& right, int fix)
{
	if (up.size() <= 1) return;

	sort(up.begin(), up.end(), compare_up_from_left_to_right);
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

		DreamNode* mid = newDreamNode(Point(now->coord.hx() + (i - fix) * 1.0 * LINE_GAP, now->coord.hy()));

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
		if (i > fix) right.push_back(mid);

		Point dd(up[i]->coord.hx() + (i - fix) * 1.0 * LINE_GAP, up[i]->coord.hy());
		if (up[i]->is_device)
		{
			DreamNode* mid2 = newDreamNode(dd);
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

vector<pair<DreamNode*, Point>> CableRouter::intersect_dream_tree(DreamTree tree, Segment seg)
{
	vector<pair<DreamNode*, Point>> res;

	queue<DreamNode*> q;
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

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

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}

	return res;
}

void CableRouter::add_line_num(DreamNode* node)
{
	while (node->parent != NULL)
	{
		node->line_num_to_parent++;
		node = node->parent;
	}
}

void CableRouter::init_line_num(DreamTree tree)
{
	queue<DreamNode*> q;
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

		if (now->is_device)
		{
			add_line_num(now);
		}

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}
}

DreamTree CableRouter::merge_to_a_tree(vector<Polyline>& paths)
{
	if (paths.size() == 0) return NULL;
	if (paths[0].size() == 0) return NULL;

	// init tree
	DreamTree tree = newDreamNode(paths[0][0]);
	DreamNode* parent = tree;
	for (int i = 1; i < paths[0].size(); i++)
	{
		DreamNode* no = newDreamNode(paths[0][i]);
		no->parent = parent;
		if (i == (int)paths[0].size() - 1)
			no->is_device = true;
		parent->children.push_back(no);
		parent = no;
	}

	// add paths to tree
	for (int i = 1; i < paths.size(); i++)
	{
		DreamNode* child = NULL;
		for (int j = (int)paths[i].size() - 1; j >= 1; j--)
		{
			Point old = paths[i][j];
			Point young = paths[i][j - 1];

			DreamNode* old_node = newDreamNode(old);
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
			DreamNode* inter_ch = intersections[u].first;
			DreamNode* inter_pa = inter_ch->parent;
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
						DreamNode* inter = newDreamNode(inter_point);
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
						DreamNode* inter = newDreamNode(inter_point);
						old_node->parent = inter;
						inter->children.push_back(old_node);
						inter->parent = inter_ch;
						inter_ch->children.push_back(inter);
					}
					else if (project_point == inter_pa->coord)
					{
						DreamNode* inter = newDreamNode(inter_point);
						old_node->parent = inter;
						inter->children.push_back(old_node);
						inter->parent = inter_pa;
						inter_pa->children.push_back(inter);
					}
					else
					{
						DreamNode* project = newDreamNode(project_point);
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

	queue<DreamNode*> q;
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

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
					Point(now->coord.hx(), now->coord.hy() + off * 500), 
					Point(now->parent->coord.hx(), now->parent->coord.hy() + off * 500)
				));
			}
			for (int i = 0; i < turn_up; i++)
			{
				int off = go_ahead - go_ahead / 2 + i;
				res.push_back(Segment(
					Point(now->coord.hx(), now->coord.hy() + off * 500), 
					Point(now->parent->coord.hx(), now->parent->coord.hy() + off * 500)
				));
			}
			for (int i = 0; i < turn_down; i++)
			{
				int off = 0 - go_ahead / 2 - (go_ahead == 0 ? 0 : 1) - i;
				//int off = 0 - go_ahead / 2 - 1 - i;
				res.push_back(Segment(
					Point(now->coord.hx(), now->coord.hy() + off * 500), 
					Point(now->parent->coord.hx(), now->parent->coord.hy() + off * 500)
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
					Point(now->coord.hx() + off * 500, now->coord.hy()),
					Point(now->parent->coord.hx() + off * 500, now->parent->coord.hy())
				));
			}
			for (int i = 0; i < trun_right; i++)
			{
				double off = go_ahead - go_ahead / 2 + i;
				res.push_back(Segment(
					Point(now->coord.hx() + off * 500, now->coord.hy()),
					Point(now->parent->coord.hx() + off * 500, now->parent->coord.hy())
				));
			}
			for (int i = 0; i < trun_left; i++)
			{
				double off = 0 - go_ahead / 2 - (go_ahead == 0 ? 0 : 1) - i;
				//double off = 0 - go_ahead / 2 - 1 - i;
				res.push_back(Segment(
					Point(now->coord.hx() + off * 500, now->coord.hy()),
					Point(now->parent->coord.hx() + off * 500, now->parent->coord.hy())
				));
			}
		}
		else
		{
			res.push_back(ss);
		}

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}

	return res;
}

vector<Polyline> CableRouter::get_dream_tree_paths(DreamTree tree)
{
	vector<Polyline> res;
	queue<DreamNode*> q;
	for (int i = 0; i < tree->children.size(); i++)
	{
		q.push(tree->children[i]);
	}
	while (!q.empty())
	{
		DreamNode* now = q.front();
		q.pop();

		if (now->is_device)
		{
			Polyline pl = get_path(now);
			if (pl.size() >= 2)
			{
				res.push_back(pl);
			}
		}

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}
	return res;
}

Polyline CableRouter::get_path(DreamNode* node)
{
	Polyline res;
	DreamNode* now = node;
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

void CableRouter::horizontal_count(DreamNode* now, int& up, int& down)
{
	if (now->line_num_to_parent <= 1)
		return;

	for (int i = 0; i < now->children.size(); i++)
	{
		DreamNode* next = now->children[i];
		if (now->coord.hy() == next->coord.hy())
		{
			horizontal_count(next, up, down);
		}
		else if (now->coord.hx() == next->coord.hx())
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

void CableRouter::vertical_count(DreamNode* now, int& left, int& right)
{
	if (now->line_num_to_parent <= 1)
		return;

	for (int i = 0; i < now->children.size(); i++)
	{
		DreamNode* next = now->children[i];
		if (now->coord.hx() == next->coord.hx())
		{
			vertical_count(next, left, right);
		}
		else if (now->coord.hy() == next->coord.hy())
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
