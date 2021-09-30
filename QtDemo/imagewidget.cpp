#include "imagewidget.h"
#include <qpen>
#include <algorithm>
#include <set>
#include <queue>
#include <CR_IO.h>
#include <crtdbg.h>

#define INF 1000000
#define MAX_DEV_IN_GROUP 25
#define MIN_DEV_IN_GROUP 11

// FOR CONNECT
#define DIJ0 0.5

// FOR PARTITION
#define ALPHA	30		// pos relation
#define BETA	10		// size even
#define OMEGA	50		// inner cohesion
#define DELTA	30		// size big
#define GAMMA	10		// cut line len

#define DIJKSTRA_ONLY 0

// test/  7 8 9 10
// test2/ 9 10 11 12
// test3/ 0 1 2 3 4 5
const std::string FOLD_NAME = "geojson/";
const std::string FILE_NAME = "test1";
ImageWidget::ImageWidget()
{
	//_CrtSetBreakAlloc(144568);
	//_CrtSetBreakAlloc(4180561);
	//_CrtSetBreakAlloc(4159756);
	//_CrtSetBreakAlloc(353061);
	//CableRouteEngine cre;
	//ifstream f(FOLD_NAME + FILE_NAME + ".geojson");
	//stringstream ss;
	//ss << f.rdbuf();
	//f.close();
	//string datastr = ss.str();
	//string res = cre.routing(datastr);
	//printf("%s", res.c_str());
	//_CrtDumpMemoryLeaks();
	//return;
	//readFiles(FILE_NAME);
	data = read_from_geojson_file(FOLD_NAME + FILE_NAME + ".geojson");
	printf("read over\n");

	//data.holes.erase(data.holes.begin() + 235);

	range.maxX = range.maxY = 0;
	range.minX = range.minY = 10000000;

	for (int i = 0; i < data.devices.size(); ++i)
	{
		points_.push_back(data.devices[i].coord);
	}
	printf("device size: %d\n", points_.size());
	for (int i = 0; i < data.powers.size(); ++i)
	{
		power_sources_.push_back(data.powers[i].points[0]);
	}
	printf("power size: %d\n", power_sources_.size());
	for (int i = 0; i < data.centers.size(); ++i)
	{
		centers_.push_back(data.centers[i]);
	}
	printf("center line size: %d\n", centers_.size());
	for (auto i = data.area.info.boundary.vertices_begin(); i != data.area.info.boundary.vertices_end(); ++i)
	{
		area_.push_back(*i);
		range.maxX = std::max(range.maxX, i->hx());
		range.maxY = std::max(range.maxY, i->hy());
		range.minX = std::min(range.minX, i->hx());
		range.minY = std::min(range.minY, i->hy());
	}
	printf("area point size: %d\n", area_.size());
	for (int i = 0; i < data.holes.size(); ++i)
	{
		vector<Point> hole;
		for (auto j = data.holes[i].vertices_begin(); j != data.holes[i].vertices_end(); ++j)
		{
			hole.push_back(*j);
			//range.maxX = std::max(range.maxX, j->hx());
			//range.maxY = std::max(range.maxY, j->hy());
			//range.minX = std::min(range.minX, j->hx());
			//range.minY = std::min(range.minY, j->hy());
		}
		holes_.push_back(hole);
	}
	printf("hole size: %d\n", holes_.size());
	for (int i = 0; i < data.rooms.size(); ++i)
	{
		vector<Point> room;
		for (auto j = data.rooms[i].vertices_begin(); j != data.rooms[i].vertices_end(); ++j)
		{
			room.push_back(*j);
		}
		rooms_.push_back(room);
	}
	printf("room size: %d\n", rooms_.size());

}

void ImageWidget::readFiles(const std::string name)
{
	Clean();
	range.maxX = range.maxY = 0;
	range.minX = range.minY = 10000000;
	FILE* infile;
	double x, y;
	char c;
	int leftbra;
	PElement e;

	// read dev points
	infile = fopen((FOLD_NAME + "点位" + name).c_str(), "rt");
	if (infile)
	{
		while (fscanf(infile, "(%lf,%lf)", &x, &y) != EOF) {
			points_.push_back(Point(x, y));
			data.devices.push_back(Device(Point(x, y), data.devices.size()));
		}
		fclose(infile);
	}
	cout << "点位数： " << points_.size() << endl;





	std::vector<rbush::TreeNode<PElement>*> hole_nodes;
	std::vector<rbush::TreeNode<PElement>*> room_nodes;
	std::vector<rbush::TreeNode<Segment>*> area_nodes;
	std::vector<rbush::TreeNode<Segment>*> center_nodes;

	// read center lines
	infile = fopen((FOLD_NAME + "中心线" + name).c_str(), "rt");
	if (infile)
	{
		double a, b;
		while (fscanf(infile, "((%lf,%lf),(%lf,%lf))", &x, &y, &a, &b) != EOF) {
			centers_.push_back(Segment(Point(x, y), Point(a, b)));
			center_nodes.push_back(get_seg_rtree_node(&centers_.back()));
		}
		fclose(infile);
	}
	cout << "中心线数： " << centers_.size() << endl;

	// read area
	infile = fopen((FOLD_NAME + "防火分区" + name).c_str(), "rt");
	if (!infile)
		leftbra = 0;
	else if (fscanf(infile, "(((") != EOF)
		leftbra = 3;
	else
		leftbra = 0;
	while (leftbra) {
		switch (leftbra)
		{
		case 3:
			fscanf(infile, "%lf,%lf)", &x, &y);
			area_.push_back(Point(x, y));
			range.maxX = std::max(range.maxX, x);
			range.maxY = std::max(range.maxY, y);
			range.minX = std::min(range.minX, x);
			range.minY = std::min(range.minY, y);
			leftbra--;
			break;
		case 2:
			fscanf(infile, "%c", &c);
			if (c == ',')
			{
				fscanf(infile, "(");
				leftbra++;
				break;
			}
			else
			{
				e.boundary = construct_polygon(&area_);
				area_.pop_back();
				leftbra--;
				break;
			}
		case 1:
			fscanf(infile, ",%lf)", &x);
			e.weight = x;
			data.area.info = e;
			leftbra--;
			break;
		case 0:
		default:
			break;
		}
	}
	for (size_t i = 0; i < area_.size(); i++)
	{
		int next = (i + 1) % area_.size();
		area_nodes.push_back(get_seg_rtree_node(&Segment(area_[i], area_[next])));
	}
	if (infile) fclose(infile);
	cout << "防火分区点数： " << area_.size() << endl;

	// read holes
	infile = fopen((FOLD_NAME + "洞口" + name).c_str(), "rt");
	if (!infile)
		leftbra = 0;
	else if (fscanf(infile, "(((") != EOF)
		leftbra = 3;
	else
		leftbra = 0;
	vector<Point> hole;
	while (leftbra >= 0) {
		switch (leftbra)
		{
		case 3:
			fscanf(infile, "%lf,%lf)", &x, &y);
			hole.push_back(Point(x, y));
			leftbra--;
			break;
		case 2:
			fscanf(infile, "%c", &c);
			if (c == ',')
			{
				fscanf(infile, "(");
				leftbra++;
				break;
			}
			else
			{
				e.boundary = construct_polygon(&hole);
				hole.pop_back();
				holes_.push_back(hole);
				reset(hole);
				leftbra--;
				break;
			}
		case 1:
			fscanf(infile, ",%lf)", &x);
			e.weight = x;
			hole_nodes.push_back(get_rtree_node(&e));
			leftbra--;
			break;
		case 0:
			if (fscanf(infile, "(((") != EOF)
				leftbra = 3;
			else
				leftbra = -1;
		default:
			break;
		}
	}
	if (infile) fclose(infile);
	cout << "洞口个数： " << holes_.size() << endl;

	// read rooms
	infile = fopen((FOLD_NAME + "房间框线" + name).c_str(), "rt");
	if (!infile)
		leftbra = 0;
	else if (fscanf(infile, "(((") != EOF)
		leftbra = 3;
	else
		leftbra = 0;
	vector<Point> room;
	while (leftbra >= 0) {
		switch (leftbra)
		{
		case 3:
			fscanf(infile, "%lf,%lf)", &x, &y);
			room.push_back(Point(x, y));
			leftbra--;
			break;
		case 2:
			fscanf(infile, "%c", &c);
			if (c == ',')
			{
				fscanf(infile, "(");
				leftbra++;
				break;
			}
			else
			{
				e.boundary = construct_polygon(&room);
				room.pop_back();
				rooms_.push_back(room);
				reset(room);
				leftbra--;
				break;
			}
		case 1:
			fscanf(infile, ",%lf)", &x);
			e.weight = x;
			room_nodes.push_back(get_rtree_node(&e));
			leftbra--;
			break;
		case 0:
			if (fscanf(infile, "(((") != EOF)
				leftbra = 3;
			else
				leftbra = -1;
		default:
			break;
		}
	}
	if (infile) fclose(infile);
	cout << "房间个数： " << rooms_.size() << endl;

	data.cen_line_tree = new SegBush(center_nodes);
	data.area.area_edge_tree = new SegBush(area_nodes);
	data.hole_tree = new PEBush(hole_nodes);
	data.room_tree = new PEBush(room_nodes);

}

ImageWidget::~ImageWidget()
{
}

void ImageWidget::paintEvent(QPaintEvent* paintevent)
{
	QPainter painter(this);

	// draw power
	painter.setPen(QPen(Qt::red, 10));
	for (size_t i = 0; i < power_sources_.size(); i++)
	{
		painter.drawPoint(XtoCanvas(DOUBLE(power_sources_[i].hx())), YtoCanvas(DOUBLE(power_sources_[i].hy())));
	}

	// draw points
	painter.setPen(QPen(Qt::green, 8));
	for (size_t i = 0; i < points_.size(); i++)
	{
		painter.drawPoint(XtoCanvas(DOUBLE(points_[i].hx())), YtoCanvas(DOUBLE(points_[i].hy())));
	}

	// draw centers
	painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
	//painter.setPen(QPen(Qt::red, 2));
	for (size_t i = 0; i < centers_.size(); i++)
	{
		painter.drawLine(
			XtoCanvas(DOUBLE(centers_[i].source().hx())), YtoCanvas(DOUBLE(centers_[i].source().hy())),
			XtoCanvas(DOUBLE(centers_[i].target().hx())), YtoCanvas(DOUBLE(centers_[i].target().hy())));
	}

	// draw area
	painter.setPen(QPen(Qt::red, 4));
	for (size_t i = 0; i < area_.size(); i++)
	{
		int next = (i + 1) % area_.size();
		painter.drawLine(
			XtoCanvas(DOUBLE(area_[i].hx())), YtoCanvas(DOUBLE(area_[i].hy())),
			XtoCanvas(DOUBLE(area_[next].hx())), YtoCanvas(DOUBLE(area_[next].hy())));
	}

	// draw holes
	painter.setPen(QPen(Qt::yellow, 1));
	for (size_t i = 0; i < holes_.size(); i++)
	{
		for (size_t j = 0; j < holes_[i].size(); j++) {
			int next = (j + 1) % holes_[i].size();
			painter.drawLine(
				XtoCanvas(DOUBLE(holes_[i][j].hx())), YtoCanvas(DOUBLE(holes_[i][j].hy())),
				XtoCanvas(DOUBLE(holes_[i][next].hx())), YtoCanvas(DOUBLE(holes_[i][next].hy())));
		}
	}

	// draw rooms
	painter.setPen(QPen(Qt::white, 1, Qt::DashLine));
	for (size_t i = 0; i < rooms_.size(); i++)
	{
		for (size_t j = 0; j < rooms_[i].size(); j++) {
			int next = (j + 1) % rooms_[i].size();
			painter.drawLine(
				XtoCanvas(DOUBLE(rooms_[i][j].hx())), YtoCanvas(DOUBLE(rooms_[i][j].hy())),
				XtoCanvas(DOUBLE(rooms_[i][next].hx())), YtoCanvas(DOUBLE(rooms_[i][next].hy())));
		}
	}

	// draw triangulation result
	painter.setPen(QPen(Qt::magenta, 2));
	for (size_t i = 0; i < result_triangulation_.size(); i++)
	{
		//painter.setPen(QPen(Qt::darkCyan, (i + 1) * 4.0 / result_tree_.size()));
		painter.drawLine(
			XtoCanvas(DOUBLE(result_triangulation_[i].source().hx())), YtoCanvas(DOUBLE(result_triangulation_[i].source().hy())),
			XtoCanvas(DOUBLE(result_triangulation_[i].target().hx())), YtoCanvas(DOUBLE(result_triangulation_[i].target().hy())));
	}

	// draw result
	painter.setPen(QPen(Qt::cyan, 1));
	if (result_tree_.size() > 0)
	//for (size_t p = GlobalCount % result_tree_.size(); p <= GlobalCount % result_tree_.size(); p++)
	for (size_t p = 0; p < result_tree_.size(); p++)
	{
		//painter.setPen(QPen(Qt::darkCyan, (i + 1) * 4.0 / result_tree_.size()));
		vector<Segment> sss = get_segments_from_polyline(result_tree_[p]);

		for (size_t i = 0; i < sss.size(); i++)
		{
			painter.drawLine(
				XtoCanvas(DOUBLE(sss[i].source().hx())), YtoCanvas(DOUBLE(sss[i].source().hy())),
				XtoCanvas(DOUBLE(sss[i].target().hx())), YtoCanvas(DOUBLE(sss[i].target().hy())));
		}
	}
	// draw result one by one
	painter.setPen(QPen(Qt::red, 1));
	if (result_tree_.size() > 0)
	for (size_t p = GlobalCount % result_tree_.size(); p <= GlobalCount % result_tree_.size(); p++)
	//for (size_t p = 0; p < result_tree_.size(); p++)
	{
		//painter.setPen(QPen(Qt::darkCyan, (i + 1) * 4.0 / result_tree_.size()));
		vector<Segment> sss = get_segments_from_polyline(result_tree_[p]);

		for (size_t i = 0; i < sss.size(); i++)
		{
			painter.drawLine(
				XtoCanvas(DOUBLE(sss[i].source().hx())), YtoCanvas(DOUBLE(sss[i].source().hy())),
				XtoCanvas(DOUBLE(sss[i].target().hx())), YtoCanvas(DOUBLE(sss[i].target().hy())));
		}
	}

	// draw grouping result
	if (result_paths_.size() < 0)
	{
		for (size_t i = 0; i <= GlobalCount % result_paths_.size(); i++)
		{
			painter.setPen(QPen(QBrush(QColor(128 + (17 + i * 13) % 128, 128 + (67 + i * 91) % 128, 128 + (175 + i * 51) % 128)), 2));
			for (auto p = result_paths_[i].begin(); p != result_paths_[i].end(); p++)
			{
				if ((p + 1) != result_paths_[i].end())
					painter.drawLine(
						XtoCanvas(DOUBLE(p->hx())), YtoCanvas(DOUBLE(p->hy())),
						XtoCanvas(DOUBLE((p + 1)->hx())), YtoCanvas(DOUBLE((p + 1)->hy())));
			}
		}
	}


	update();
}

void ImageWidget::mousePressEvent(QMouseEvent* mouseevent)
{
	if (mouseevent->button() == Qt::LeftButton)
	{
		power_sources_.push_back(Point(CanvastoX(mouseevent->pos().rx()), CanvastoY(mouseevent->pos().ry())));
		//data.powers.push_back(Power(power_sources_.back()));
		if (power_sources_.size() >= 3)
		{
			data.powers.push_back(Power(Segment(power_sources_[1], power_sources_[2])));
		}
		update();
	}
}

void ImageWidget::mouseMoveEvent(QMouseEvent* mouseevent)
{
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* mouseevent)
{
}

void ImageWidget::Clean()
{
	//GlobalCount--;
	//return;


	clean_lines();
	//reset(points_);
	//reset(area_);
	//reset(holes_);
	//reset(rooms_);
}

void ImageWidget::ConvexHull()
{
	CableRouteEngine cre;
	ifstream f(FOLD_NAME + FILE_NAME + ".geojson");
	stringstream ss;
	ss << f.rdbuf();
	f.close();
	string datastr = ss.str();
	string res = cre.routing(datastr);
	printf("%s", res.c_str());
	return;
	//reset(points_);
	//int k = 0;
	//for (int i = 0; i <= GlobalCount % engines[k].data.devices.size(); i++)
	//{
	//	points_.push_back(engines[k].data.devices[i].coord);
	//}
	//printf("dist = %lf\n", engines[k].data.G[engines[k].data.devices.size()][GlobalCount % engines[k].data.devices.size()]);
	GlobalCount++;
}

double ImageWidget::XtoCanvas(double x)
{
	return 1.0 * (x - range.minX) / (range.maxX - range.minX) * 1500 + 40;
}

double ImageWidget::YtoCanvas(double y)
{
	return 1500 - 1.0 * (y - range.minY) / (range.maxY - range.minY) * 1500 + 40;
}

double ImageWidget::CanvastoX(double x)
{
	return 1.0 * (x - 40) / 1500 * (range.maxX - range.minX) + range.minX;
}

double ImageWidget::CanvastoY(double y)
{
	return 1.0 * (1540 - y) / 1500 * (range.maxY - range.minY) + range.minY;
}

void ImageWidget::Partition()
{
	clean_lines();
	point_n_ = points_.size();
	if (point_n_ == 0) return;

	// map points to id

	GroupEngine pe;
	GroupParam paramm;
	paramm.max_dev_size = MAX_DEV_IN_GROUP;
	paramm.min_dev_size = MIN_DEV_IN_GROUP;
	paramm.weight_pos = ALPHA;
	paramm.weight_even = BETA;
	paramm.weight_cohesion = OMEGA;
	paramm.weight_big = DELTA;
	paramm.weight_cut_len = GAMMA;

	CGAL::Timer timer;
	timer.start();

	printf("begin parse\n");
	//MapInfo map = read_from_geojson_file("test" + FILE_NAME + ".geojson");
	//for (int i = 0; i < power_sources_.size(); i++)
	//{
	//map.powers.push_back(Power(power_sources_));
	//}
	printf("begin group\n");
	vector<vector<int>> group_info = pe.grouping(&data, &paramm);

	double sec = timer.time();
	printf("用时：%lf s\n", sec);

	vector<ISData> groups = parse_groups(&data, group_info);

	reset(engines);
	for (int i = 0; i < groups.size(); i++)
	{
		ImmuneSystem re;
		re.init(&groups[i]);
		engines.push_back(re);
	}


	sec = timer.time();
	printf("用时：%lf s\n", sec);
}

bool path_compare_0(std::pair<Point, Point> a, std::pair<Point, Point> b)
{
	//int a_up = (a.second.hy() - a.first.hy()) > 0 ? 1 : -1;
	//int b_up = (b.second.hy() - b.first.hy()) > 0 ? 1 : -1;
	//int a_area = -1 * (a_right ? 1 : -2) * (a_up ? 1 : 2);
	//int b_area = -1 * (b_right ? 1 : -2) * (b_up ? 1 : 2);

	//if (a_area != b_area) return a_area > b_area;

	//if (a_right) return abs(DOUBLE(a.second.hy() - a.first.hy())) > abs(DOUBLE(b.second.hy() - b.first.hy()));

	//return abs(DOUBLE(a.second.hy() - a.first.hy())) < abs(DOUBLE(b.second.hy() - b.first.hy()));
	//return a.second.hx() < b.second.hx();
	int a_right = (a.second.hx() - a.first.hx()) > 0 ? 1 : -1;
	int b_right = (b.second.hx() - b.first.hx()) > 0 ? 1 : -1;
	if (a_right != b_right) return a_right < b_right;
	return a_right * abs(DOUBLE(a.second.hy() - a.first.hy())) >
		b_right * abs(DOUBLE(b.second.hy() - b.first.hy()));
}

void ImageWidget::Connect()
{
	//GlobalCount++; return;
	//reset(points_);
	//clean_lines();
	//for (int i = 0; i < GlobalCount; i++)
	//{
	//	points_.push_back(engines[4].data.devices[i].coord);
	//}
	//printf("dev to pwr = %lf\n", engines[4].data.G[GlobalCount - 1][engines[4].data.devices.size()]);
	//printf("pwr to dev = %lf\n", engines[4].data.G[engines[4].data.devices.size()][GlobalCount - 1]);
	//return;
	printf("Routing begin\n");
	CGAL::Timer timer;
	timer.start();
	double sec;
	reset(result_tree_);
	reset(result_triangulation_);
	reset(result_paths_);

	vector<Polyline> cables;

	vector<Polyline> power_paths(data.powers.size());
	for (int e = 0; e < engines.size(); e++)
	{
		for (int k = 0; k < 20; k++)
			engines[e].run();

		if (engines[e].globlMem.size() < 1)
		{
			printf("Group %d No result!\n", e);
			continue;
		}

		auto adj = engines[e].globlMem.rbegin()->adj;
		printf("Best value = %lf\n", engines[e].globlMem.rbegin()->value);

		vector<Device>& devices = engines[e].data.devices;
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
		printf("get_manhattan_tree begin\n");
		get_manhattan_tree(&data, path_tree, cables);
		printf("avoid_coincidence begin\n");
		avoid_coincidence(path_tree);
		printf("avoid_coincidence end\n");
		vector<Polyline> paths = get_dream_tree_paths(path_tree);
		cables.insert(cables.end(), paths.begin(), paths.end());
		//deleteDreamTree(path_tree);
	}
	deleteAllDreamNodes();
	result_tree_ = cables;
	printf("Routing end\n");
	sec = timer.time();
	printf("用时：%lf s\n", sec); 
	return;

	vector<Polyline> result_paths;
	vector<Segment> exist_lines = get_segments_from_polylines(cables);
	//for (int i = 0; i < power_paths.size(); i++)
	for (int i = 0; i < power_paths.size(); i++)
	{
		Power pwr = data.powers[i];
		for (int j = 0; j < power_paths[i].size(); j++)
		{
			Point dev = power_paths[i][j];
			printf("Power %d: Looking for path %d\n", i, j);
			Polyline pp;
			if (pwr.is_point())
				pp = obstacle_avoid_connect_p2p(&data, pwr.points[0], dev, exist_lines);
			else if (pwr.is_segment())
				pp = obstacle_avoid_connect_p2s(&data, dev, Segment(pwr.points[0], pwr.points[1]), exist_lines);
			else
				printf("invalid power\n");

			printf("Path size: %d\n", pp.size());
			result_paths.push_back(pp);
			for (int k = 0; k < (int) pp.size() - 1; k++)
			{
				result_triangulation_.push_back(Segment(pp[k], pp[k + 1]));
				//exist_lines.push_back(Segment(pp[k], pp[k + 1]));
			}
			//vector<Segment> pp_segs = get_segments_from_polyline(pp);
			//exist_lines.insert(exist_lines.end(), pp_segs.begin(), pp_segs.end());
		}
	}


	printf("Routing end\n");
	sec = timer.time();
	printf("用时：%lf s\n", sec);

	//return;

	printf("Begin merge\n");
	//DreamTree dream_tree = merge_to_a_tree(result_paths_);
	printf("Begin show tree\n");
	//result_triangulation_ = get_dream_tree_lines(dream_tree, true);
	//result_triangulation_ = get_dream_tree_lines(dream_tree);
	//reset(centers_);
	//centers_ = get_dream_tree_lines(dream_tree);



	//queue<DreamNode*> q;
	//for (int i = 0; i < dream_tree->children.size(); i++)
	//{
	//	q.push(dream_tree->children[i]);
	//}
	//while (!q.empty())
	//{
	//	DreamNode* now = q.front();
	//	q.pop();

	//	printf("line num to parent = %d\n", now->line_num_to_parent);

	//	for (int i = 0; i < now->children.size(); i++)
	//	{
	//		q.push(now->children[i]);
	//	}
	//}



	printf("Begin delete tree\n");
	//deleteDreamTree(dream_tree);
	printf("delete over\n");

}

void ImageWidget::Triangulation()
{
	Point s = data.devices[0].coord;
	Point t = data.devices[1].coord;
	CDT dt;
	set<Constraint> constraints;

	// start and end
	vector<pair<Point, VertexInfo>> vec;
	vec.push_back(make_pair(s, VertexInfo(0, true)));
	vec.push_back(make_pair(t, VertexInfo(1, true)));

	// area
	Polygon area_boundary = data.area.info.boundary;
	for (int i = 0; i < area_boundary.size(); i++)
	{
		constraints.insert(Constraint(area_boundary.vertex(i), area_boundary.vertex((i + 1) % area_boundary.size())));
	}

	// holes
	for (auto h = data.holes.begin(); h != data.holes.end(); h++)
	{
		for (int i = 0; i < h->size(); i++)
		{
			constraints.insert(Constraint(h->vertex(i), h->vertex((i + 1) % h->size())));
		}
	}

	// centers
	//for (auto c = data.centers.begin(); c != data.centers.end(); c++)
	//{
	//	Point start = c->source();
	//	Point mid = CGAL::midpoint(c->source(), c->target());
	//	if (mid != s && mid != t)
	//		vec.push_back(make_pair(mid, VertexInfo(2, true)));
	//	int num = 1 + ((int)LEN(*c)) / 2000;
	//	for (int i = 1; i <= num; i++)
	//	{
	//		Point end = Point(
	//			1.0 * i / num * DOUBLE(c->target().hx() - c->source().hx()) + DOUBLE(c->source().hx()),
	//			1.0 * i / num * DOUBLE(c->target().hy() - c->source().hy()) + DOUBLE(c->source().hy()));
	//		constraints.insert(Constraint(start, end));
	//		start = end;
	//	}
	//}

	dt.insert(vec.begin(), vec.end());

	for (auto c = constraints.begin(); c != constraints.end(); c++)
	{
		dt.insert_constraint(c->source, c->target);
	}

	mark_domains(dt);

	//printf("Begin a star\n");

	map<Face_handle, int> fid;
	ASNode* nodes = new ASNode[dt.number_of_faces()];
	vector<ASNode*> openlist;
	int n = 0;

	for (auto feit = dt.finite_edges_begin(); feit != dt.finite_edges_end(); feit++)
	{
		if (feit->first->info().not_reach() ||
			feit->first->neighbor(feit->second)->info().not_reach())
		{
			//continue;
		}
		Polyline p;
		p.push_back(feit->first->vertex(dt.cw(feit->second))->point());
		p.push_back(feit->first->vertex(dt.ccw(feit->second))->point());
		result_tree_.push_back(p);
	}

	return;





	//clean_lines();

	//if (power_sources_.size() < 2) return;

	//vector<Point> path = getFittingLine(Vector(0,1), power_sources_, 0);

	//for (int i = 0; i < path.size() - 1; i++)
	//{
	//	result_tree_.push_back(Segment(path[i], path[i + 1]));
	//}
	//return;


	if (power_sources_.size() < 3) return;


	CGAL::Timer timer;
	timer.start();
	//vector<Segment> exist_lines = result_tree_;
	//for (int l = 1; l < power_sources_.size(); l++)
	//{
	//	int off = l - power_sources_.size() / 2;
	//	Point pwr = Point(power_sources_[0].hx() + off * 400, power_sources_[0].hy());
	//	Point dev = power_sources_[l];
	//	vector<Point> line = obstacle_avoid_connect(&data, pwr, dev, exist_lines);

	//	for (int i = 0; i < line.size() - 1; i++)
	//	{
	//		Point u = line[i];
	//		Point v = line[i + 1];
	//		exist_lines.push_back(Segment(u, v));
	//		result_triangulation_.push_back(Segment(u, v));
	//	}
	//}


	Segment pwr = Segment(power_sources_[0], power_sources_[1]);
	vector<Segment> exist_lines;
	//vector<Segment> exist_lines = result_tree_;
	//result_tree_.push_back(pwr);
	for (int l = 2; l < power_sources_.size(); l++)
	{
		
		Point dev = power_sources_[l];
		Polyline line = obstacle_avoid_connect_p2s(&data, dev, pwr, exist_lines);
		for (int i = 0; i < line.size() - 1; i++)
		{
			Point u = line[i];
			Point v = line[i + 1];
			result_triangulation_.push_back(Segment(u, v));
			//exist_lines.push_back(Segment(u, v));
		}
	}


	double secc = timer.time();
	printf("用时：%lf s", secc);
}

void ImageWidget::Open()
{
	reset(result_triangulation_);

	auto h = data.holes[25];

	for (int i = GlobalCount % h.size(); i <= GlobalCount % h.size(); i++)
	{
		result_triangulation_.push_back(Segment(h.vertex(i), h.vertex((i + 1) % h.size())));
		printf("(%.15f, %.15f) -> ", h.vertex(i).hx(), h.vertex(i).hy());
		printf("(%.15f, %.15f)\n", h.vertex((i + 1) % h.size()).hx(), h.vertex((i + 1) % h.size()).hy());
	}
	GlobalCount++;
}

void ImageWidget::clean_lines()
{
	reset(result_triangulation_);
	reset(result_tree_);
	reset(result_paths_);
}
