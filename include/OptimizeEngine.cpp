#include "OptimizeEngine.h"
#include "RouteEngine.h"

using namespace CableRouter;

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
	return n;
}

void CableRouter::deleteDreamTree(DreamTree root)
{
	vector<DreamNode*> chs = root->children;
	delete root;
	for (int i = 0; i < chs.size(); i++)
	{
		deleteDreamTree(chs[i]);
	}
}

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
		for (int j = 0; j < path.size() - 1; j++)
		{
			exist_lines.push_back(Segment(path[j], path[j + 1]));
			DreamNode* ch = NULL;
			if (j + 1 == path.size() - 1)
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
			for (int i = 0; i < now->children.size(); i++)
			{
				now->children[i]->parent = now->parent;
				now->parent->children.push_back(now->children[i]);
			}
			reset(now->children);
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
	printf("init begin\n");
	DreamTree tree = newDreamNode(paths[0][0]);
	DreamNode* parent = tree;
	for (int i = 1; i < paths[0].size(); i++)
	{
		DreamNode* no = newDreamNode(paths[0][i]);
		no->parent = parent;
		if (i == paths[0].size() - 1)
			no->is_device = true;
		parent->children.push_back(no);
		parent = no;
	}
	printf("init end\n");

	// add paths to tree
	for (int i = 1; i < paths.size(); i++)
	{
		DreamNode* child = NULL;
		for (int j = paths[i].size() - 1; j >= 1; j--)
		{
			Point old = paths[i][j];
			Point young = paths[i][j - 1];

			DreamNode* old_node = newDreamNode(old);
			if (j == paths[i].size() - 1)
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
					deleteDreamTree(old_node);
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
					deleteDreamTree(old_node);
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

vector<Segment> CableRouter::get_dream_tree_lines(DreamTree tree)
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

		res.push_back(ss);

		for (int i = 0; i < now->children.size(); i++)
		{
			q.push(now->children[i]);
		}
	}

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
