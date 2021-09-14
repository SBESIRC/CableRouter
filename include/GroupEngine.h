#include "CR_CGALUtils.h"
#include "MapUtils.h"
#include "GETree.h"

#ifndef _CABLEROUTER_GROUP_ENGINE_H_
#define _CABLEROUTER_GROUP_ENGINE_H_

namespace CableRouter
{
	struct GroupParam
	{
		int max_dev_size;
		int min_dev_size;
		int weight_pos;
		int weight_even;
		int weight_cohesion;
		int weight_big;
		int weight_cut_len;
	};

	class GroupEngine
	{
	public:
		vector<vector<int>>	grouping(MapInfo* const data, const GroupParam* param);


	private:
		void			getParam(const GroupParam* param);

		void			divide(vector<GENode>& tree, int root);
		double			evaluate(vector<GENode>& tree, int root, int now);


		vector<vector<int>>	partition_result;
		int					dev_min;
		int					dev_max;
		int					w1, w2, w3, w4, w5;
	};
}

#endif