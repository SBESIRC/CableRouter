#include "CR_Parse.h"


static void CableRouterParse::splits(const string& s, vector<string>& tokens, const string& delimiters)
{
	string::size_type lastPos = s.find(delimiters, 0);
	if (lastPos == string::npos)
		return;
	lastPos += delimiters.length();
	string::size_type pos = s.find(delimiters, lastPos);
	while (tokens.push_back(s.substr(lastPos, pos-lastPos)), pos != string::npos) {
		lastPos = pos + delimiters.length();
		pos = s.find(delimiters, lastPos);
	}
}

static void CableRouterParse::splits_properties(const string& s, vector<string>& tokens, const string& delimiters)
{
	string::size_type lastPos = s.find(delimiters, 0);
	if (lastPos == string::npos)
		return;

	string new_string = s.substr(0, lastPos);
	new_string.erase(remove(new_string.begin(), new_string.end(), ' '), new_string.end());
	new_string.erase(remove(new_string.begin(), new_string.end(), '\t'), new_string.end());
	tokens.push_back(new_string);
	lastPos += delimiters.length();
	string::size_type pos = s.find(delimiters, lastPos);

	while (pos != string::npos) {
		string new_string = s.substr(lastPos, pos-lastPos);
		new_string.erase(remove(new_string.begin(), new_string.end(), ' '), new_string.end());
		new_string.erase(remove(new_string.begin(), new_string.end(), '\t'), new_string.end());
		tokens.push_back(new_string);
		lastPos = pos + delimiters.length();
		pos = s.find(delimiters, lastPos);
	}
}

CableRouterParse::Block::Block()
{
	category = "";
	name = "";
	id = "";
	area_id = "";
	group_id = "";
	isolated = true;
	direction = point{1.0, 0.0};
}

CableRouterParse::Block::Block(vector<point> coords, string category, string name)
{
	this->coords = coords;
	this->category = category;
	this->name = name;
	direction = point{1.0, 0.0};
}

vector<CableRouterParse::Block> CableRouterParse::parse_geojson(const string filename, set<string> &categories, set<string> &spacenames)
{
	vector<Block> data;
	ifstream f{ filename, std::ios::in };
	stringstream ss;
	ss << f.rdbuf();
	auto datastr = ss.str();

	auto start = datastr.find_first_of('[');
	auto end = datastr.find_last_of(']');
	datastr = datastr.substr(start, end - start + 1);

	vector<string> s;
	splits(datastr, s, "{\r\n");
	assert(s.size() % 3 == 0);

	for (int i = 0; i < s.size() / 3; ++i) {
		string s1 = s[3 * i + 1];
		auto start = s1.find_first_of('[');
		auto end = s1.find_last_of(']');
		string scoords = s1.substr(start, end - start + 1);
		regex re("(-[0-9]+(.[0-9]+)?)|([0-9]+(.[0-9]+)?)");
		sregex_iterator pos(scoords.begin(), scoords.end(), re);
		sregex_iterator end_it;
		vector<point> coords;
		for (; pos != end_it; ++pos) {
			double x = stod(pos->str(0));
			double y = stod((++pos)->str(0));
			point p{x, y};
			coords.push_back(p);
		}
		string s2 = s[3 * i + 2];
		vector<string> properties;
		splits_properties(s2, properties, "\r\n");

		Block new_block;
		new_block.coords = coords;
		for(int j = 0; j < properties.size(); j++)
		{
			string property = properties[j];
			auto start = property.find_first_of(':');
			if(start != string::npos)
			{
				string key = property.substr(0+1, start-2);
				string value = property.substr(start+1, property.length()-start-1);
				if(value[0] == '\"')
				{
					if(value[value.length()-1] == ',')
					{
						value = value.substr(1, value.length()-3);
					}
					else
					{
						value = value.substr(1, value.length()-2);
					}
				}

				if(key.compare("Id") == 0)
				{
					new_block.id = value;
				} 
				else if (key.compare("Category") == 0)
				{
					new_block.category = value;
				} 
				else if (key.compare("AreaId") == 0)
				{
					new_block.area_id = value;
				} 
				else if (key.compare("GroupId") == 0)
				{
					new_block.group_id = value;
				}
				else if (key.compare("Name") == 0)
				{
					new_block.name = value;
				}
				else if (key.compare("Isolated") == 0)
				{
					istringstream(value) >> new_block.isolated;
				}
				else if (key.compare("AlignmentVector") == 0)
				{
					auto start = value.find_first_of('[');
					auto end = value.find_last_of(']');
					value = value.substr(start+1, end-start-1);
					regex re(",");
					sregex_token_iterator pos(value.begin(), value.end(), re, -1);
					point p{stof(pos->str()), stof((++pos)->str())};
					new_block.alignment_vector = p;
				}
				else if (key.compare("Direction") == 0)
				{
					auto start = value.find_first_of('[');
					auto end = value.find_last_of(']');
					value = value.substr(start+1, end-start-1);
					regex re(",");
					sregex_token_iterator pos(value.begin(), value.end(), re, -1);
					point p{stof(pos->str()), stof((++pos)->str())};
					new_block.direction = p;
				}
				else if (key.compare("NeighborIds") == 0)
				{
					auto start = value.find_first_of('[');
					auto end = value.find_last_of(']');
					value = value.substr(start+1, end-start-1);
					if(value.length() == 0)
					{
						continue;
					}
					regex re(",");
					sregex_token_iterator pos(value.begin(), value.end(), re, -1);
					sregex_token_iterator end_it;
					vector<string> neighbor_ids;
					for(; pos != end_it; ++pos)
					{
						string neighbor = pos->str();
						neighbor = neighbor.substr(1, neighbor.length()-2);
						new_block.neighbor_ids.push_back(neighbor);
					}
				}
			}
		}
		data.push_back(new_block);
	}
	return data;
}


vector<CableRouterParse::Block> CableRouterParse::parse_geojson_string(string datastr, set<string> &categories, set<string> &spacenames)
{
	auto start = datastr.find_first_of('[');
	auto end = datastr.find_last_of(']');
	datastr = datastr.substr(start, end - start);

	vector<Block> data;
	vector<string> s;
	splits(datastr, s, "{\r\n");
	assert(s.size() % 3 == 0);

	// for(int i = 0; i < s.size(); i++)
	// {
	// 	cout << "comp" << endl;
	// 	cout << s[i] << endl;
	// }

	for (int i = 0; i < s.size() / 3; ++i) {
		string s1 = s[3 * i + 1];
		auto start = s1.find_first_of('[');
		auto end = s1.find_last_of(']');
		string scoords = s1.substr(start, end - start + 1);
		regex re("(-[0-9]+(.[0-9]+)?)|([0-9]+(.[0-9]+)?)");
		sregex_iterator pos(scoords.begin(), scoords.end(), re);
		sregex_iterator end_it;
		vector<point> coords;
		for (; pos != end_it; ++pos) {
			double x = stod(pos->str(0));
			double y = stod((++pos)->str(0));
			point p{x, y};
			coords.push_back(p);
		}
		string s2 = s[3 * i + 2];
		vector<string> properties;
		splits_properties(s2, properties, "\r\n");

		Block new_block;
		new_block.coords = coords;
		for(int j = 0; j < properties.size(); j++)
		{
			string property = properties[j];
			auto start = property.find_first_of(':');
			if(start != string::npos)
			{
				string key = property.substr(0+1, start-2);
				string value = property.substr(start+1, property.length()-start-1);
				if(value[0] == '\"')
				{
					if(value[value.length()-1] == ',')
					{
						value = value.substr(1, value.length()-3);
					}
					else
					{
						value = value.substr(1, value.length()-2);
					}
				}

				if(key.compare("Id") == 0)
				{
					new_block.id = value;
				} 
				else if (key.compare("Category") == 0)
				{
					new_block.category = value;
				} 
				else if (key.compare("AreaId") == 0)
				{
					new_block.area_id = value;
				} 
				else if (key.compare("GroupId") == 0)
				{
					new_block.group_id = value;
				}
				else if (key.compare("Name") == 0)
				{
					new_block.name = value;
				}
				else if (key.compare("Isolated") == 0)
				{
					istringstream(value) >> new_block.isolated;
				}
				else if (key.compare("AlignmentVector") == 0)
				{
					stringstream ss;
					ss << value;
					bool is_end = false;
					if(value.find("]") != string::npos)
					{
						is_end = true;
					}
					while(!is_end)
					{
						j++;
						string search_target = properties[j];
						ss << search_target;
						if(search_target.find("]") != string::npos)
						{
							is_end = true;
							break;
						}
					}
					string content = ss.str();
					regex re("(-[0-9]+(.[0-9]+)?)|([0-9]+(.[0-9]+)?)");
					sregex_iterator pos(content.begin(), content.end(), re);
					sregex_iterator end_it;
					vector<point> coords;
					double align_x = stof(pos->str(0));
					double align_y = stof((++pos)->str(0));
					point p{align_x, align_y};

					new_block.alignment_vector = p;
				}
				else if (key.compare("Direction") == 0)
				{
					stringstream ss;
					ss << value;
					bool is_end = false;
					if(value.find("]") != string::npos)
					{
						is_end = true;
					}
					while(!is_end)
					{
						j++;
						string search_target = properties[j];
						ss << search_target;
						if(search_target.find("]") != string::npos)
						{
							is_end = true;
							break;
						}
					}
					string content = ss.str();
					regex re("(-[0-9]+(.[0-9]+)?)|([0-9]+(.[0-9]+)?)");
					sregex_iterator pos(content.begin(), content.end(), re);
					sregex_iterator end_it;
					vector<point> coords;
					double dir_x = stof(pos->str(0));
					double dir_y = stof((++pos)->str(0));
					point p{dir_x, dir_y};
					new_block.direction = p;
				}
				else if (key.compare("NeighborIds") == 0)
				{
					stringstream ss;
					ss << value;
					bool is_end = false;
					if(value.find("]") != string::npos)
					{
						is_end = true;
					}
					while(!is_end)
					{
						j++;
						string search_target = properties[j];
						ss << search_target;
						if(search_target.find("]") != string::npos)
						{
							is_end = true;
							break;
						}
					}
					string content = ss.str();
					auto start = content.find_first_of('[');
					auto end = content.find_last_of(']');
					content = content.substr(start+1, end-start-1);
					if(content.length() == 0)
					{
						continue;
					}
					regex re(",");
					sregex_token_iterator pos(content.begin(), content.end(), re, -1);
					sregex_token_iterator end_it;
					vector<string> neighbor_ids;
					for(; pos != end_it; ++pos)
					{
						string neighbor = pos->str();
						neighbor = neighbor.substr(1, neighbor.length()-2);
						new_block.neighbor_ids.push_back(neighbor);
					}
				}
			}
		}
		data.push_back(new_block);
	}
	return data;
}


void CableRouterParse::analysis(vector<Block>& data, map<string, Block*>& region, map<string, vector<Block*>>& space, map<string, vector<Block*>>& column, map<string, vector<Block*>>& input, map<string, vector<Block*>>& output)
{
	for(int i = 0; i < data.size(); i++)
	{
		string category = data[i].category;
		if(category.compare("Area") == 0)
		{
			region[data[i].id] = &data[i];
		}
	}

	for(map<string, Block*>::iterator iter = region.begin(); iter != region.end(); iter++)
	{
		string id = iter->first;
		space[id] = vector<Block*>();
		column[id] = vector<Block*>();
		input[id] = vector<Block*>();
		output[id] = vector<Block*>();
	}

	for(int i = 0; i < data.size(); i++)
	{
		string category = data[i].category;
		string area_id = data[i].area_id;
		if (category.compare("Space") == 0)
		{
			space[area_id].push_back(&data[i]);
		}
		else if (category.compare("Column") == 0)
		{
			column[area_id].push_back(&data[i]);
		}
		else if (category.compare("WaterSupplyStartPoint") == 0)
		{
			input[area_id].push_back(&data[i]);
		}
		else if (category.compare("WaterSupplyPoint") == 0)
		{
			output[area_id].push_back(&data[i]);
		}
	}
}

void CableRouterParse::output_file(string filename, vector<Block>& data)
{
	ofstream fout{ filename, ios::out };
	fout << fixed;
	fout << "{\r\n";
	fout << "    \"type\": \"FeatureCollection\",\r\n";
	fout << "    \"features\": [\r\n";
	for(int i = 0; i < data.size(); i++)
	{
		output_single(fout, &data[i]);
		if(i == data.size() - 1)
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
	fout.close();
}

void CableRouterParse::output_single(ofstream& fout, Block* data)
{
	if(data->category.compare("WaterSupplyPoint") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Point\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            " << to_string(data->coords[0].x) << ",\r\n";
		fout << "            " << to_string(data->coords[0].y) << "\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Id\": \"" << data->id << "\",\r\n";
		fout << "        \"Category\": \"WaterSupplyPoint\",\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\",\r\n";
		fout << "        \"Direction\" : [";
		fout << to_string(data->direction.x) << "," << to_string(data->direction.y) << "],\r\n";
		fout << "        \"GroupId\": \"" << data->group_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if (data->category.compare("WaterSupplyStartPoint") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Point\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            " << to_string(data->coords[0].x) << ",\r\n";
		fout << "            " << to_string(data->coords[0].y) << "\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Category\": \"WaterSupplyStartPoint\",\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if (data->category.compare("Column") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Polygon\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "            ]\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Category\": \"Column\",\r\n";
		fout << "        \"Isolated\": ";
		if(data->isolated)
		{
			fout << "true,\r\n";
		} 
		else
		{
			fout << "false,\r\n";
		}
		fout << "        \"AreaId\": \"" << data->area_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if(data->category.compare("Space") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Polygon\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "            ]\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Category\": \"Space\",\r\n";
		// fout << "        \"Name\": \"" << data->name << "\",\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if(data->category.compare("Area") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Polygon\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "            ]\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Id\": \"" << data->id << "\",\r\n";
		fout << "        \"Category\": \"Area\",\r\n";
		fout << "        \"AlignmentVector\" : [";
		fout << to_string(data->alignment_vector.x) << "," << to_string(data->alignment_vector.y) << "],\r\n";
		fout << "        \"NeighborIds\": [";
		for(int j = 0; j < data->neighbor_ids.size(); j++)
		{
			fout << "\"" << data->neighbor_ids[j] << "\"";
			if(j != data->neighbor_ids.size()-1)
			{
				fout << ",";
			}
		}
		fout << "]\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if(data->category.compare("Pipe") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"LineString\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\",\r\n";
		fout << "        \"Category\": \"Pipe\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
}

void CableRouterParse::output_string(stringstream& fout, Block* data)
{
	if(data->category.compare("WaterSupplyPoint") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Point\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            " << to_string(data->coords[0].x) << ",\r\n";
		fout << "            " << to_string(data->coords[0].y) << "\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Id\": \"" << data->id << "\",\r\n";
		fout << "        \"Category\": \"WaterSupplyPoint\",\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\",\r\n";
		fout << "        \"Direction\" : [";
		fout << to_string(data->direction.x) << "," << to_string(data->direction.y) << "],\r\n";
		fout << "        \"GroupId\": \"" << data->group_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if (data->category.compare("WaterSupplyStartPoint") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Point\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            " << to_string(data->coords[0].x) << ",\r\n";
		fout << "            " << to_string(data->coords[0].y) << "\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Category\": \"WaterSupplyStartPoint\",\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if (data->category.compare("Column") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Polygon\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "            ]\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Category\": \"Column\",\r\n";
		fout << "        \"Isolated\": ";
		if(data->isolated)
		{
			fout << "true,\r\n";
		} 
		else
		{
			fout << "false,\r\n";
		}
		fout << "        \"AreaId\": \"" << data->area_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if(data->category.compare("Space") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Polygon\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "            ]\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Category\": \"Space\",\r\n";
		// fout << "        \"Name\": \"" << data->name << "\",\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if(data->category.compare("Area") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"Polygon\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		fout << "            [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "            ]\r\n";
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"Id\": \"" << data->id << "\",\r\n";
		fout << "        \"Category\": \"Area\",\r\n";
		fout << "        \"AlignmentVector\" : [";
		fout << to_string(data->alignment_vector.x) << "," << to_string(data->alignment_vector.y) << "],\r\n";
		fout << "        \"NeighborIds\": [";
		for(int j = 0; j < data->neighbor_ids.size(); j++)
		{
			fout << "\"" << data->neighbor_ids[j] << "\"";
			if(j != data->neighbor_ids.size()-1)
			{
				fout << ",";
			}
		}
		fout << "]\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
	else if(data->category.compare("Pipe") == 0)
	{
		fout << "{\r\n";
		fout << "    \"type\": \"Feature\",\r\n";
		fout << "    \"geometry\": {\r\n";
		fout << "        \"type\": \"LineString\",\r\n";
		fout << "        \"coordinates\": [\r\n";
		for(int j = 0; j < data->coords.size(); j++)
		{
			fout << "                [\r\n";
			fout << "                    " << data->coords[j].x << ",\r\n";
			fout << "                    " << data->coords[j].y << "\r\n";
			fout << "                ]";
			if(j != data->coords.size()-1)
			{
				fout << ",\r\n";
			}
			else
			{
				fout << "\r\n";
			}
		}
		fout << "        ]\r\n";
		fout << "    },\r\n";
		fout << "    \"properties\": {\r\n";
		fout << "        \"AreaId\": \"" << data->area_id << "\",\r\n";
		fout << "        \"Category\": \"Pipe\"\r\n";
		fout << "    }\r\n";
		fout << "}";
	}
}

void CableRouterParse::get_region_size(Block* region, double& min_x, double& min_y)
{
	for(int i = 0; i < region->coords.size(); i++)
	{
		if(region->coords[i].x < min_x)
		{
			min_x = region->coords[i].x;
		}
		if(region->coords[i].y < min_y)
		{
			min_y = region->coords[i].y;
		}
	}
}

void CableRouterParse::copy_block(Block* old_block, Block* new_block)
{
	for(int i = 0; i < old_block->coords.size(); i++)
	{
		new_block->coords.push_back(old_block->coords[i]);
	}

	for(int i = 0; i < old_block->neighbor_ids.size(); i++)
	{
		new_block->neighbor_ids.push_back(old_block->neighbor_ids[i]);
	}

	new_block->alignment_vector = old_block->alignment_vector;
	new_block->category = old_block->category;
	new_block->name = old_block->name;
	new_block->id = old_block->id;
	new_block->area_id = old_block->area_id;
	new_block->group_id = old_block->group_id;
	new_block->isolated = old_block->isolated;
}

void CableRouterParse::rotate_point(point& pt, double angle, double min_x, double min_y)
{
	/*

	x_new = (x - min_x) * cos(a) - (y - min_y) * sin(a) + min_x
	y_new = (x - min_x) * sin(a) + (y - min_y) * cos(a) + min_y

	*/
	double old_x = pt.x;
	double old_y = pt.y;

	pt.x = (old_x - min_x) * cos(angle) - (old_y - min_y) * sin(angle) + min_x;
	pt.y = (old_x - min_x) * sin(angle) + (old_y - min_y) * cos(angle) + min_y;
}

#pragma optimize("", off)
void CableRouterParse::align_region(double& angle, point& origin, Block* region, vector<Block*>& space, vector<Block*>& input, vector<Block*>& output, vector<Block*>& column, Block* new_region, vector<Block*>& new_space, vector<Block*>& new_input, vector<Block*>& new_output, vector<Block*>& new_column)
{
	point align_vector = region->alignment_vector;
	double min_x = MAX_LENGTH, min_y = MAX_LENGTH;
	get_region_size(region, min_x, min_y);

	// copy blocks
	copy_block(region, new_region);
	
	for(int i = 0; i < space.size(); i++)
	{
		Block * new_space_i = new Block();
		copy_block(space[i], new_space_i);
		new_space.push_back(new_space_i);
	}

	for(int i = 0; i < input.size(); i++)
	{
		Block * new_input_i = new Block();
		copy_block(input[i], new_input_i);
		new_input.push_back(new_input_i);
	}

	for(int i = 0; i < output.size(); i++)
	{
		Block * new_output_i = new Block();
		copy_block(output[i], new_output_i);
		new_output.push_back(new_output_i);
	}

	for(int i = 0; i < column.size(); i++)
	{
		Block * new_column_i = new Block();
		copy_block(column[i], new_column_i);
		new_column.push_back(new_column_i);
	}

	// determine rotation angle

	if(align_vector.x == 0 || align_vector.y == 0)
	{
		angle = 0;
		return;
	}

	angle = -atan(align_vector.y / align_vector.x);

	for(int i = 0; i < new_region->coords.size(); i++)
	{
		rotate_point(new_region->coords[i], angle, min_x, min_y);
	}
	rotate_point(new_region->alignment_vector, angle, 0, 0);

	for(int i = 0; i < new_output.size(); i++)
	{
		rotate_point(new_output[i]->coords[0], angle, min_x, min_y);
	}

	for(int i = 0; i < new_input.size(); i++)
	{
		rotate_point(new_input[i]->coords[0], angle, min_x, min_y);
	}

	for(int i = 0; i < new_space.size(); i++)
	{
		for(int j = 0; j < new_space[i]->coords.size(); j++)
		{
			rotate_point(new_space[i]->coords[j], angle, min_x, min_y);
		}
	}

	for(int i = 0; i < new_column.size(); i++)
	{
		for(int j = 0; j < new_column[i]->coords.size(); j++)
		{
			rotate_point(new_column[i]->coords[j], angle, min_x, min_y);
		}
	}

	origin.x = min_x;
	origin.y = min_y;

}


void CableRouterParse::rotate_region(double& angle, point& origin, Block* region, vector<Block*>& space, vector<Block*>& input, vector<Block*>& output, vector<Block*>& column)
{
	double min_x = origin.x, min_y = origin.y;

	for(int i = 0; i < region->coords.size(); i++)
	{
		rotate_point(region->coords[i], angle, min_x, min_y);
	}
	rotate_point(region->alignment_vector, angle, 0, 0);

	for(int i = 0; i < output.size(); i++)
	{
		rotate_point(output[i]->coords[0], angle, min_x, min_y);
	}

	for(int i = 0; i < input.size(); i++)
	{
		rotate_point(input[i]->coords[0], angle, min_x, min_y);
	}

	for(int i = 0; i < space.size(); i++)
	{
		for(int j = 0; j < space[i]->coords.size(); j++)
		{
			rotate_point(space[i]->coords[j], angle, min_x, min_y);
		}
	}

	for(int i = 0; i < column.size(); i++)
	{
		for(int j = 0; j < column[i]->coords.size(); j++)
		{
			rotate_point(column[i]->coords[j], angle, min_x, min_y);
		}
	}
}

double CableRouterParse::pp_distance(point x1, point x2)
{
	return sqrt(pow(x1.x - x2.x, 2) + pow(x1.y - x2.y, 2));
}

double CableRouterParse::point_wall_distance(point wall_p1, point wall_p2, point pt)
{
	double wall_length = pp_distance(wall_p1, wall_p2);
	point l1{pt.x - wall_p1.x, pt.y - wall_p1.y};
	point l2{wall_p2.x - wall_p1.x, wall_p2.y - wall_p1.y};
	double judge = (l1.x * l2.x + l1.y * l2.y) / (wall_length * wall_length);

	double scale = max(0.0, min(1.0, judge));
	point projection{wall_p1.x + scale * (wall_p2.x - wall_p1.x), wall_p1.y + scale * (wall_p2.y - wall_p1.y)};

	return pp_distance(projection, pt);
}

void CableRouterParse::get_space_size(Block* space, double& area, double& min_x, double& max_x, double& min_y, double& max_y)
{
	for(int i = 0; i < space->coords.size(); i++)
	{
		if(space->coords[i].x < min_x)
		{
			min_x = space->coords[i].x;
		}
		if(space->coords[i].x > max_x)
		{
			max_x = space->coords[i].x;
		}
		if(space->coords[i].y < min_y)
		{
			min_y = space->coords[i].y;
		}
		if(space->coords[i].y > max_y)
		{
			max_y = space->coords[i].y;
		}
	}
	area = (max_x - min_x) * (max_y - min_y);
}

double CableRouterParse::pp_mul(point p1, point p2, point p3)
{
	return (p2.x-p1.x)*(p3.y-p1.y)-(p3.x-p1.x)*(p2.y-p1.y);
}

void CableRouterParse::get_align_vector(Block * region, double& angle, double& min_x, double& min_y)
{
	point align_vector = region->alignment_vector;

	if(align_vector.x == 0 || align_vector.y == 0)
	{
		angle = 0;
	}
	else
	{
		angle = -atan(align_vector.y / align_vector.x);
	}

	get_region_size(region, min_x, min_y);
}