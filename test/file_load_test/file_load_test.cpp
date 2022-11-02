#include "async_detour_crowd.h"
#include <iostream>
int main()
{
	std::string nav_map_path = "../../../data/simple_grid.navmesh";
	spiritsaway::system::navigation::async_detour_crowd cur_crowd(1024);
	std::array<dtReal_t, 3> half_extend{ 10, 10,10 };
	if (!cur_crowd.init(nav_map_path, half_extend, 10))
	{
		std::cout << "fail to load navmesh" << std::endl;
		return -1;
	}
	return 0;
}