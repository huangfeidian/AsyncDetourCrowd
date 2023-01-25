#include "async_detour_crowd.h"
#include <iostream>
int main()
{
	std::string nav_map_path = "../../../data/simple_grid.navmesh";
	spiritsaway::system::navigation::async_detour_crowd cur_crowd(1024);
	std::array<dtReal_t, 3> half_extend{ 10, 10,10 };
	auto init_err = cur_crowd.init(nav_map_path, half_extend, 10);
	if (init_err)
	{
		std::cout << "fail to load navmesh with err " << init_err<<std::endl;
		return 0;
	}
	auto& cur_crowd_impl = cur_crowd.crowd_impl();
	auto m_navquery = dtAllocNavMeshQuery();
	if (dtStatusFailed(m_navquery->init(cur_crowd.navmesh(), 512)))
	{
		std::cout<<"fail to init navmesh qeury"<<std::endl;
		return 0;
	}
	std::array<dtReal_t, 3> pos{ 0,0,0 };
	std::array<dtReal_t, 3> extend{ 8,6,8 };
	dtQueryFilter default_filer;
	dtPolyRef result;
	std::array<dtReal_t, 3> nearst;
	m_navquery->findNearestPoly(pos.data(), extend.data(), &default_filer, &result, nearst.data());
	return 0;
}