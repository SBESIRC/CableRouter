#include "CR_IO.h"
#include "json\json.h"

using namespace CableRouter;

//Point CableRouter::cgal_point(CableRouterParse::point& p)
//{
//	return Point(p.x, p.y);
//}
//
//CableRouterParse::point CableRouter::block_point(Point& p)
//{
//	return CableRouterParse::point{ DOUBLE(p.hx()), DOUBLE(p.hy()) };
//}
//
//vector<Point> CableRouter::cgal_points(vector<CableRouterParse::point>& pts)
//{
//	vector<Point> res;
//	for (int i = 0; i < pts.size(); i++)
//	{
//		res.push_back(cgal_point(pts[i]));
//	}
//	return res;
//}
//
//vector<CableRouterParse::point> CableRouter::block_points(vector<Point>& pts)
//{
//	vector<CableRouterParse::point> res;
//	for (int i = 0; i < pts.size(); i++)
//	{
//		res.push_back(block_point(pts[i]));
//	}
//	return res;
//}
//
//MapInfo CableRouter::read_blocks(vector<CableRouterParse::Block>& data)
//{
//	MapInfo map;
//	vector<rbush::TreeNode<PElement>*> hole_nodes;
//	vector<rbush::TreeNode<PElement>*> room_nodes;
//	vector<rbush::TreeNode<Segment>* > area_nodes;
//	vector<rbush::TreeNode<Segment>* > center_nodes;
//
//	PElement e;
//
//	printf("blocks size: %d\n", data.size());
//	for (int i = 0; i < data.size(); i++)
//	{
//		string category = data[i].category;
//		//string area_id = data[i].area_id;
//		if (category.compare("Device") == 0)
//		{
//			map.devices.push_back(Device(cgal_point(data[i].coords[0]), (int)map.devices.size()));
//		}
//		if (category.compare("PowerSource") == 0)
//		{
//			map.powers.push_back(Power(cgal_points(data[i].coords)));
//		}
//		else if (category.compare("Hole") == 0)
//		{
//			e.boundary = construct_polygon(&cgal_points(data[i].coords));
//			e.weight = CR_INF;
//			hole_nodes.push_back(get_rtree_node(&e));
//		}
//		else if (category.compare("FireCompartment") == 0)
//		{
//			e.boundary = construct_polygon(&cgal_points(data[i].coords));
//			e.weight = CR_INF;
//			map.area.info = e;
//			for (auto eit = e.boundary.edges_begin(); eit != e.boundary.edges_end(); eit++)
//			{
//				area_nodes.push_back(get_seg_rtree_node(&(*eit)));
//			}
//		}
//		else if (category.compare("CenterLine") == 0)
//		{
//			center_nodes.push_back(get_seg_rtree_node(&Segment(cgal_point(data[i].coords[0]), cgal_point(data[i].coords[1]))));
//		}
//		else if (category.compare("RoomLine") == 0)
//		{
//			e.boundary = construct_polygon(&cgal_points(data[i].coords));
//			e.weight = 10;
//			room_nodes.push_back(get_rtree_node(&e));
//		}
//	}
//
//	map.cen_line_tree = new SegBush(center_nodes);
//	map.area.area_edge_tree = new SegBush(area_nodes);
//	map.hole_tree = new PEBush(hole_nodes);
//	map.room_tree = new PEBush(room_nodes);
//
//	return map;
//}

static void parse_geojson(MapInfo& map, const string& datastr)
{
    bool res;
    std::string errs;
    Json::Value root;
    Json::CharReaderBuilder readerBuilder;
    std::unique_ptr<Json::CharReader> const jsonReader(readerBuilder.newCharReader());

    res = jsonReader->parse(datastr.c_str(), datastr.c_str() + datastr.length(), &root, &errs);
    if (!res || !errs.empty())
    {
        printf("parseJson err. %s\n", errs.c_str());
    }

    size_t size = root["features"].size();


    vector<rbush::TreeNode<PElement>*> hole_nodes;
    vector<rbush::TreeNode<PElement>*> room_nodes;
    vector<rbush::TreeNode<Segment>* > area_nodes;
    vector<rbush::TreeNode<Segment>* > center_nodes;

    for (auto i = 0; i < size; i++)
    {
        auto geom = root["features"][i]["geometry"];
        auto coor = geom["coordinates"];
        auto prop = root["features"][i]["properties"];
        string gtype = geom["type"].asString();
        string cat = prop["Category"].asString();

        vector<vector<Point>> vec_pts;
        if (gtype.compare("Polygon") == 0)
        {
            for (auto j = 0; j < coor.size(); ++j)
            {
                vector<Point> pts;
                for (auto k = 0; k < coor[j].size(); ++k)
                {
                    Point p(coor[j][k][0].asDouble(), coor[j][k][1].asDouble());
                    //Point aligned_p = p.transform(it->second.align);
                    Point aligned_p = p;
                    pts.push_back(aligned_p);
                }
                vec_pts.push_back(pts);
            }
        }
        else if (gtype.compare("Point") == 0)
        {
            vector<Point> pts;
            Point p(coor[0].asDouble(), coor[1].asDouble());
            //Point aligned_p = p.transform(it->second.align);
            Point aligned_p = p;
            pts.push_back(aligned_p);
            vec_pts.push_back(pts);
        }
        else if (gtype.compare("LineString") == 0)
        {
            vector<Point> pts;
            for (auto k = 0; k < coor.size(); ++k)
            {
                Point p(coor[k][0].asDouble(), coor[k][1].asDouble());
                //Point aligned_p = p.transform(it->second.align);
                Point aligned_p = p;
                pts.push_back(aligned_p);
            }
            vec_pts.push_back(pts);
        }
        else 
        {
            printf("No such geometry type : %s\n", gtype.c_str());
        }

        if (cat.compare("WiringPosition") == 0)
        {
            map.devices.push_back(Device(vec_pts[0][0], (int)map.devices.size()));
        }
        else if (cat.compare("PowerPosition") == 0)
        {
            map.powers.push_back(Power(vec_pts[0][0]));
        }
        else if (
            cat.compare("ArchitectureWall") == 0 ||
            cat.compare("ShearWall") == 0 ||
            cat.compare("Column") == 0 ||
            cat.compare("Hole") == 0)
        {
            for (auto j = 0; j < vec_pts.size(); ++j)
            {
                PElement e;
                e.boundary = construct_polygon(&vec_pts[j]);
                e.weight = CR_INF;
                hole_nodes.push_back(get_rtree_node(&e));
                map.holes.push_back(e.boundary);
            }
        }
        else if (cat.compare("FireApart") == 0)
        {
            PElement e;
            e.boundary = construct_polygon(&vec_pts[0]);
            e.weight = CR_INF;
            map.area.info = e;
            for (auto eit = e.boundary.edges_begin(); eit != e.boundary.edges_end(); eit++)
            {
                area_nodes.push_back(get_seg_rtree_node(&(*eit)));
            }
        }
        else if (cat.compare("CenterLine") == 0)
        {
            for (auto j = 0; j < vec_pts[0].size() - 1; ++j)
            {
                center_nodes.push_back(get_seg_rtree_node(&Segment(vec_pts[0][j], vec_pts[0][j + 1])));
            }
        }
        else if (cat.compare("Room") == 0)
        {
            for (auto j = 0; j < vec_pts.size(); ++j)
            {
                PElement e;
                e.boundary = construct_polygon(&vec_pts[j]);
                e.weight = 10;
                room_nodes.push_back(get_rtree_node(&e));
                map.rooms.push_back(e.boundary);
            }
        }
        else
        {
            printf("No such category : %s\n", cat.c_str());
        }
    }

    map.cen_line_tree = new SegBush(center_nodes);
    map.area.area_edge_tree = new SegBush(area_nodes);
    map.hole_tree = new PEBush(hole_nodes);
    map.room_tree = new PEBush(room_nodes);
}

MapInfo CableRouter::read_from_geojson_string(const string& datastr)
{
	MapInfo map;
	parse_geojson(map, datastr);
	return map;
}

MapInfo CableRouter::read_from_geojson_file(const string& filename)
{
	ifstream f(filename);
	stringstream ss;
	ss << f.rdbuf();
	string datastr = ss.str();
    f.close();

	return read_from_geojson_string(datastr);
}


static Json::Value cable_to_json(const Polyline& polyline) {
    Json::Value json_linestring;
    Json::Value json_geometry;
    Json::Value json_properties;
    Json::Value json_coordinates;

    for (size_t i = 0; i < polyline.size(); ++i) {
        Json::Value json_point;
        json_point.append(DOUBLE(polyline[i].hx()));
        json_point.append(DOUBLE(polyline[i].hy()));
        json_coordinates.append(json_point);
    }

    json_geometry["type"] = Json::Value("LineString");
    json_geometry["coordinates"] = json_coordinates;
    json_properties["Category"] = Json::Value("Cable");

    json_linestring["type"] = Json::Value("Feature");
    json_linestring["geometry"] = json_geometry;
    json_linestring["properties"] = json_properties;

    return json_linestring;
}

string CableRouter::write_to_geojson_string(const vector<Polyline>& cables)
{
    Json::Value json_root;
    Json::Value json_features;

    for (auto it = cables.begin(); it != cables.end(); ++it) {
        json_features.append(cable_to_json(*it));
    }

    json_root["type"] = Json::Value("FeatureCollection");
    json_root["features"] = json_features;

    Json::StreamWriterBuilder writerBuilder;
    ostringstream os;
    writerBuilder.settings_["indentation"] = "    ";
    std::unique_ptr<Json::StreamWriter> jsonWriter(writerBuilder.newStreamWriter());
    jsonWriter->write(json_root, &os);
    return os.str();
}

void CableRouter::write_to_geojson_file(const string& filename, const vector<Polyline>& paths)
{
    string datastr = write_to_geojson_string(paths);
    ofstream f(filename);
    f << datastr << endl;
    f.close();
}