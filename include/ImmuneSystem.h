#include "CGALUtils.h"
#include "MapUtils.h"

#ifndef _CABLEROUTER_IMMUNE_SYSTEM_H_
#define _CABLEROUTER_IMMUNE_SYSTEM_H_

namespace CableRouter
{
	struct Antibody
	{
		vector<vector<int>> adj;
		vector<int> prufer_code;
		double value;
		bool operator < (const Antibody& an) const
		{
			return value < an.value;
		}
		bool operator > (const Antibody& an) const
		{
			return value > an.value;
		}
		bool operator == (const Antibody& an) const
		{
			return prufer_code == an.prufer_code;
		}
	};

	vector<int> PruferEncode(const vector<vector<int>>& adj);
	vector<vector<int>> PruferDecode(const vector<int>& code);

	struct ISData
	{
		vector<Device> devices;
		vector<Power> powers;
		vector<vector<double>> G;
	};

	vector<ISData> parse_groups(MapInfo* map, vector<vector<int>>& groups);

	class ImmuneSystem
	{
	public:
		void init(ISData* data);
		void run();
		ISData data;
		set<Antibody> globlMem;

	private:
		vector<vector<double>> pheromone;
		set<Antibody> localMem;

		double affinity(vector<vector<int>>& adj, vector<vector<double>>& G, int dn);
	};

}

#endif