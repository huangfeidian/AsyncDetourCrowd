#include "async_detour_crowd.h"

namespace spiritsaway::system::navigation
{
	void async_detour_crowd::add_agent_req(agent_req_info&& cur_req)
	{
		std::unique_lock<std::mutex> temp_lock(m_mutex);
		m_agent_reqs.push_back(std::move(cur_req));
	}
	std::optional<std::uint32_t> async_detour_crowd::add_agent(const dtCrowdAgentParams& param, const std::array<double, 3>& pos)
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

	bool async_detour_crowd::move_to_pos(std::uint32_t agent_idx, const std::array<double, 3>& dest_pos, double radius, std::function<void()> finish_cb)
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

	bool async_detour_crowd::move_to_agent(std::uint32_t agent_idx, std::uint32_t dest_agent, double radius, std::function<void()> finish_cb)
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

	bool async_detour_crowd::change_pos(std::uint32_t agent_idx, const std::array<double, 3>& pos)
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

	bool async_detour_crowd::update_param(std::uint32_t agent_idx, const dtCrowdAgentParams& param)
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
			default:
				break;
			}
		}
	}
	void async_detour_crowd::handle_agent_pos_update(std::uint32_t agent_idx)
	{
		auto& cur_agent = m_agents[agent_idx];
		auto cur_dest_pos = cur_agent.dest_pos;
		if (cur_agent.state == agent_state::move_to_agent)
		{
			const auto& dest_agent = m_agents[cur_agent.dest_agent];
			if (dest_agent.state == agent_state::inactive || dest_agent.state == agent_state::deleting)
			{
				cancel_move(agent_idx);
				return;
			}
			cur_dest_pos = dest_agent.pos;
		}
		std::array<float, 3> diff_pos;
		for (std::uint32_t i = 0; i < 3; i++)
		{
			diff_pos[i] = cur_dest_pos[i] - cur_agent.pos[i];
		}
		if (diff_pos[0] * diff_pos[0] + diff_pos[2] * diff_pos[2] < cur_agent.dest_radius * cur_agent.dest_radius)
		{
			cancel_move(agent_idx);
			return;
		}
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
				//TODO
				break;
			}
			case agent_req_cmd::move_to_pos:
			{
				dtPolyRef dest_poly;
				std::array<double, 3> dest_pos;
				one_ack.cmd_result = m_detour_crowd.getNavMeshQuery()->findNearestPoly(one_req.pos.data(), m_half_extend.data(), &m_query_filter, &dest_poly, dest_pos.data());
				if (dtStatusSucceed(one_ack.cmd_result))
				{
					m_detour_crowd.requestMoveTarget(one_req.idx, dest_poly, dest_pos.data());
				}
				break;
			}
			case agent_req_cmd::change_pos:
			{
				//TODO
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
		std::vector<dtCrowdAgent*> active_agent_vec(m_max_agent_num, nullptr);
		auto active_num = m_detour_crowd.getActiveAgents(active_agent_vec.data(), active_agent_vec.size());
		dtCrowdAgent* idx_0_agent = m_detour_crowd.getEditableAgent(0);
		for (std::uint32_t i = 0; i < active_num; i++)
		{
			auto cur_agent = active_agent_vec[i];
			agent_ack_info one_ack;
			one_ack.idx = std::distance(idx_0_agent, cur_agent);
			one_ack.ack_version = m_agent_ack_versions[one_ack.idx];
			one_ack.cmd_result = 0;
			one_ack.cmd = agent_req_cmd::sync_pos;
			std::copy(cur_agent->npos, cur_agent->npos + 3, one_ack.pos.data());
			m_temp_agent_ack_vec_in_update.push_back(one_ack);
		}
		{
			std::unique_lock<std::mutex> temp_lock(m_mutex);
			std::swap(m_agent_acks, m_temp_agent_ack_vec_in_update);
		}


	}
}