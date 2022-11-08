#include "async_detour_crowd.h"
#include <cstring>

namespace
{
	static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
	static const int NAVMESHSET_VERSION = 1;
#pragma pack(push, 4)
	struct NavMeshSetHeader
	{
		int magic;
		int version;
		int numTiles;
		dtNavMeshParams params;
	};

	struct NavMeshTileHeader
	{
		dtTileRef tileRef;
		int dataSize;
	};
#pragma pack(pop)
	std::uint32_t load_nav_mesh(const char* path, dtNavMesh** navmesh)
	{
		FILE* fp = fopen(path, "rb");
		if (!fp) return __LINE__;

		// Read header.
		NavMeshSetHeader header;
		size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return __LINE__;
		}
		if (header.magic != NAVMESHSET_MAGIC)
		{
			fclose(fp);
			return __LINE__;
		}
		if (header.version != NAVMESHSET_VERSION)
		{
			fclose(fp);
			return __LINE__;
		}

		dtNavMesh* mesh = dtAllocNavMesh();
		if (!mesh)
		{
			fclose(fp);
			return __LINE__;
		}
		dtStatus status = mesh->init(&header.params);
		if (dtStatusFailed(status))
		{
			fclose(fp);
			return status;
		}

		// Read tiles.
		for (int i = 0; i < header.numTiles; ++i)
		{
			NavMeshTileHeader tileHeader;
			readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
			if (readLen != 1)
			{
				fclose(fp);
				return __LINE__;
			}

			if (!tileHeader.tileRef || !tileHeader.dataSize)
				break;

			unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
			if (!data) break;
			std::memset(data, 0, tileHeader.dataSize);
			readLen = fread(data, tileHeader.dataSize, 1, fp);
			if (readLen != 1)
			{
				dtFree(data);
				fclose(fp);
				return __LINE__;
			}

			mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
		}

		fclose(fp);
		*navmesh = mesh;
		return 0;
	}
}
namespace spiritsaway::system::navigation
{

	async_detour_crowd::async_detour_crowd(std::uint32_t max_agent_num)
		: m_max_agent_num(max_agent_num)
		, m_agents(max_agent_num)
		, m_finished_agent_vec(max_agent_num)
		, m_active_agent_vec(max_agent_num)
		, m_last_update_ms(0)
		, m_dt_update_finish(true)
	{
		m_inactive_agent_idxes.reserve(max_agent_num);
		for (std::uint32_t i = 0; i < max_agent_num; i++)
		{
			m_inactive_agent_idxes.push_back(i);
		}
		std::reverse(m_inactive_agent_idxes.begin(), m_inactive_agent_idxes.end());
	}

	std::uint32_t async_detour_crowd::init(const std::string& nav_map, const std::array<dtReal_t, 3>& half_extend, dtReal_t max_agent_radius)
	{
		if (m_navmesh)
		{
			return 100;
		}
		m_half_extend = half_extend;
		std::uint32_t load_err = load_nav_mesh(nav_map.c_str(), &m_navmesh);
		if (!m_navmesh)
		{
			return load_err;
		}
		auto init_err = m_detour_crowd.init(m_max_agent_num, max_agent_radius, m_navmesh);
		if (init_err != 0)
		{
			dtFreeNavMesh(m_navmesh);
			m_navmesh = nullptr;
			return init_err;
		}
		return 0;

	}
	void async_detour_crowd::add_agent_req(agent_req_info&& cur_req)
	{
		std::unique_lock<std::mutex> temp_lock(m_mutex);
		m_agent_reqs.push_back(std::move(cur_req));
	}
	std::optional<std::uint32_t> async_detour_crowd::add_agent(const ExtendDetourCrowdAgentParams& param, const std::array<dtReal_t, 3>& pos)
	{
		if (m_inactive_agent_idxes.empty())
		{
			return {};
		}
		std::uint32_t i = m_inactive_agent_idxes.back();
		m_inactive_agent_idxes.pop_back();
		m_agents[i].state = agent_state::idle;
		m_agents[i].req_version = 0;
		m_agents[i].pos = pos;
		m_agents[i].param = param;
		agent_req_info cur_req;
		cur_req.cmd = agent_req_cmd::add;
		cur_req.param = param;
		cur_req.req_version = 0;
		cur_req.idx = i;
		cur_req.pos = pos;
		add_agent_req(std::move(cur_req));
		return i;
	}

	bool async_detour_crowd::remove_agent(std::uint32_t agent_idx)
	{
		if (agent_idx >= m_max_agent_num)
		{
			return false;
		}
		auto& cur_agent = m_agents[agent_idx];
		switch (cur_agent.state)
		{
		case agent_state::inactive:
		{
			return false;
		}
		case agent_state::deleting:
		{
			return false;
		}
		default:
		{
			cur_agent.state = agent_state::deleting;
			agent_req_info cur_req;
			cur_req.cmd = agent_req_cmd::remove;
			cur_req.req_version = cur_agent.req_version;
			cur_req.idx = agent_idx;
			add_agent_req(std::move(cur_req));
			return true;
		}
		}
	}

	bool async_detour_crowd::move_to_pos(std::uint32_t agent_idx, const std::array<dtReal_t, 3>& dest_pos, dtReal_t radius, std::function<void()> finish_cb)
	{
		if (agent_idx >= m_max_agent_num)
		{
			return false;
		}
		auto& cur_agent = m_agents[agent_idx];
		if (cur_agent.state != agent_state::idle)
		{
			return false;
		}
		cur_agent.state = agent_state::move_to_pos;
		cur_agent.dest_pos = dest_pos;
		cur_agent.dest_radius = radius;
		cur_agent.finish_cb = finish_cb;
		cur_agent.req_version++;
		agent_req_info cur_req;
		cur_req.cmd = agent_req_cmd::move_to_pos;
		cur_req.req_version = cur_agent.req_version;
		cur_req.idx = agent_idx;
		cur_req.radius = radius;
		cur_req.pos = dest_pos;
		add_agent_req(std::move(cur_req));
		return true;
	}

	bool async_detour_crowd::move_to_agent(std::uint32_t agent_idx, std::uint32_t dest_agent, dtReal_t radius, std::function<void()> finish_cb)
	{
		if (agent_idx >= m_max_agent_num || dest_agent >= m_max_agent_num)
		{
			return false;
		}
		auto& cur_agent = m_agents[agent_idx];
		if (cur_agent.state != agent_state::idle)
		{
			return false;
		}
		auto dest_agent_state = m_agents[dest_agent].state;
		if (dest_agent_state == agent_state::inactive || dest_agent_state == agent_state::deleting)
		{
			return false;
		}

		cur_agent.state = agent_state::move_to_agent;
		cur_agent.dest_agent = dest_agent;
		cur_agent.dest_radius = radius;
		cur_agent.finish_cb = finish_cb;
		cur_agent.req_version++;
		agent_req_info cur_req;
		cur_req.cmd = agent_req_cmd::move_to_agent;
		cur_req.req_version = cur_agent.req_version;
		cur_req.idx = agent_idx;
		cur_req.dest_idx = dest_agent;
		cur_req.radius = radius;
		add_agent_req(std::move(cur_req));
		return true;
	}
	bool async_detour_crowd::cancel_move(std::uint32_t agent_idx)
	{
		if (agent_idx >= m_max_agent_num)
		{
			return false;
		}
		auto& cur_agent = m_agents[agent_idx];
		if (cur_agent.state != agent_state::move_to_pos && cur_agent.state != agent_state::move_to_agent)
		{
			return false;
		}
		cur_agent.state = agent_state::idle;
		auto pre_cb = std::move(cur_agent.finish_cb);
		cur_agent.req_version++;
		agent_req_info cur_req;
		cur_req.cmd = agent_req_cmd::cancel;
		cur_req.req_version = cur_agent.req_version;
		cur_req.idx = agent_idx;
		add_agent_req(std::move(cur_req));
		if (pre_cb)
		{
			pre_cb();
		}
		return true;
	}

	bool async_detour_crowd::change_pos(std::uint32_t agent_idx, const std::array<dtReal_t, 3>& pos)
	{
		if (agent_idx >= m_max_agent_num)
		{
			return false;
		}
		auto& cur_agent = m_agents[agent_idx];
		if (cur_agent.state != agent_state::idle)
		{
			return false;
		}
		cur_agent.req_version++;
		cur_agent.pos = pos;
		agent_req_info cur_req;
		cur_req.cmd = agent_req_cmd::change_pos;
		cur_req.req_version = cur_agent.req_version;
		cur_req.idx = agent_idx;
		cur_req.pos = pos;
		add_agent_req(std::move(cur_req));
		return true;
	}

	bool async_detour_crowd::update_param(std::uint32_t agent_idx, const ExtendDetourCrowdAgentParams& param)
	{
		if (agent_idx >= m_max_agent_num)
		{
			return false;
		}
		auto& cur_agent = m_agents[agent_idx];
		if (cur_agent.state == agent_state::inactive || cur_agent.state == agent_state::deleting)
		{
			return false;
		}
		cur_agent.req_version++;
		cur_agent.param = param;
		agent_req_info cur_req;
		cur_req.cmd = agent_req_cmd::update_param;
		cur_req.req_version = cur_agent.req_version;
		cur_req.idx = agent_idx;
		cur_req.param = param;
		add_agent_req(std::move(cur_req));
		return true;
		
	}

	void async_detour_crowd::process_acks()
	{
		m_temp_agent_ack_vec_in_main.clear();
		{
			std::unique_lock<std::mutex> temp_lock;
			std::swap(m_temp_agent_ack_vec_in_main, m_agent_acks);
		}
		for (const auto& one_ack : m_temp_agent_ack_vec_in_main)
		{
			auto& cur_agent = m_agents[one_ack.idx];
			switch (one_ack.cmd)
			{
			case agent_req_cmd::sync_pos:
			{
				if (cur_agent.state == agent_state::move_to_agent || cur_agent.state == agent_state::move_to_pos)
				{
					if (one_ack.ack_version == cur_agent.req_version)
					{
						cur_agent.pos = one_ack.pos;
						handle_agent_pos_update(one_ack.idx);
					}
				}
				break;
			}
			case agent_req_cmd::remove:
			{
				if (cur_agent.state == agent_state::deleting)
				{
					cur_agent.state = agent_state::inactive;
					m_inactive_agent_idxes.push_back(one_ack.idx);
					
				}
				break;
			}
			case agent_req_cmd::notify_move_finished:
			{
				if (cur_agent.state == agent_state::move_to_agent || cur_agent.state == agent_state::move_to_pos)
				{
					if (one_ack.ack_version == cur_agent.req_version)
					{
						cancel_move(one_ack.idx);
					}
				}
			}
			default:
				break;
			}
		}
	}
	void async_detour_crowd::handle_agent_pos_update(std::uint32_t agent_idx)
	{
		auto& cur_agent = m_agents[agent_idx];
		auto cur_dest_pos = cur_agent.dest_pos;
	}

	void async_detour_crowd::update_dt_crowd(float dt)
	{
		m_temp_agent_req_vec_in_update.clear();
		{
			std::unique_lock<std::mutex> temp_lock(m_mutex);
			std::swap(m_temp_agent_req_vec_in_update, m_agent_reqs);
		}
		m_temp_agent_ack_vec_in_update.clear();

		for (const auto& one_req : m_temp_agent_req_vec_in_update)
		{
			m_agent_ack_versions[one_req.idx] = one_req.req_version;
			agent_ack_info one_ack;
			one_ack.idx = one_req.idx;
			one_ack.ack_version = one_req.req_version;
			one_ack.cmd_result = 0xffffffff;
			switch (one_req.cmd)
			{
			case agent_req_cmd::add:
			{
				if (m_detour_crowd.addAgent(one_req.idx, one_req.pos.data(), &one_req.param))
				{
					one_ack.cmd_result = 0;
				}
				break;
			}
			case agent_req_cmd::remove:
			{
				m_detour_crowd.removeAgent(one_req.idx);
				one_ack.cmd_result = 0;
				break;
			}
			case agent_req_cmd::cancel:
			{
				m_detour_crowd.resetMoveTarget(one_req.idx);
				one_ack.cmd_result = 0;
				break;
			}
			case agent_req_cmd::move_to_agent:
			{
				m_detour_crowd.requestMoveTarget(one_req.idx, one_req.dest_idx, one_req.radius, false);
				break;
			}
			case agent_req_cmd::move_to_pos:
			{
				dtPolyRef dest_poly;
				std::array<dtReal_t, 3> dest_pos;
				one_ack.cmd_result = m_detour_crowd.getNavMeshQuery()->findNearestPoly(one_req.pos.data(), m_half_extend.data(), &m_query_filter, &dest_poly, dest_pos.data());
				if (dtStatusSucceed(one_ack.cmd_result))
				{
					m_detour_crowd.requestMoveTarget(one_req.idx, dest_poly, dest_pos.data());
				}
				break;
			}
			case agent_req_cmd::change_pos:
			{
				m_detour_crowd.updateAgentPos(one_req.idx, one_req.pos.data());
				break;
			}
			case agent_req_cmd::update_param:
			{
				one_ack.cmd_result = 0;
				m_detour_crowd.updateAgentParameters(one_req.idx, &one_req.param);
				break;
			}
			default:
				break;
			}
			m_temp_agent_ack_vec_in_update.push_back(one_ack);
		}
		m_detour_crowd.update(dt, nullptr);
		auto active_num = m_detour_crowd.getActiveAgents(m_active_agent_vec.data(), m_active_agent_vec.size());
		ExtendDetourCrowdAgent* idx_0_agent = m_detour_crowd.getEditableAgent(0);
		for (int i = 0; i < active_num; i++)
		{
			auto cur_agent = m_active_agent_vec[i];
			agent_ack_info one_ack;
			one_ack.idx = std::distance(idx_0_agent, cur_agent);
			one_ack.ack_version = m_agent_ack_versions[one_ack.idx];
			one_ack.cmd_result = 0;
			one_ack.cmd = agent_req_cmd::sync_pos;
			std::copy(cur_agent->npos, cur_agent->npos + 3, one_ack.pos.data());
			m_temp_agent_ack_vec_in_update.push_back(one_ack);
		}
		auto finish_num = m_detour_crowd.fetchAndClearArrived(m_finished_agent_vec.data(), m_finished_agent_vec.size());
		for (int i = 0; i < finish_num; i++)
		{
			agent_ack_info one_ack;
			one_ack.idx = m_finished_agent_vec[i];
			one_ack.ack_version = m_agent_ack_versions[one_ack.idx];
			one_ack.cmd_result = 0;
			one_ack.cmd = agent_req_cmd::notify_move_finished;
			m_temp_agent_ack_vec_in_update.push_back(one_ack);
		}

		{
			std::unique_lock<std::mutex> temp_lock(m_mutex);
			std::swap(m_agent_acks, m_temp_agent_ack_vec_in_update);
		}
		m_dt_update_finish.store(true);
	}
	std::function<void()> async_detour_crowd::update(std::uint64_t cur_ms)
	{
		if (m_last_update_ms == 0)
		{
			m_last_update_ms = cur_ms;
			return {};
		}
		

		if (m_dt_update_finish)
		{
			process_acks();
			m_dt_update_finish.store(false);
			auto diff_ms = cur_ms - m_last_update_ms;
			auto dt = diff_ms * 0.001;
			m_last_update_ms = cur_ms;
			return [this, dt]()
			{

				this->update_dt_crowd(dt);
			};
		}
		return {};
		
	}
}