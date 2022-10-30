#pragma once
#include "DetourCrowd.h"
#include <cstdint>
#include <vector>
#include <array>
#include <functional>
#include <deque>
#include <mutex>
#include <optional>
namespace spiritsaway::system::navigation
{
	class async_detour_crowd
	{
	public:
		enum class agent_state
		{
			inactive = 0,
			idle,
			move_to_pos,
			move_to_agent,
			finished,
			deleting,

		};
		enum class agent_req_cmd
		{
			add = 0,
			remove,
			move_to_pos,
			move_to_agent,
			cancel,
			change_pos,
			update_param,
			sync_pos,
		};


		struct agent_req_info
		{
			agent_req_cmd cmd;
			int idx;
			std::array<double, 3> pos;
			std::uint32_t dest_idx;
			double radius;
			dtCrowdAgentParams param;
			std::uint32_t req_version;
		};
		struct agent_ack_info
		{
			agent_req_cmd cmd;
			int idx;
			std::array<double, 3> pos;
			std::uint32_t ack_version;
			std::uint32_t cmd_result;
		};
		struct agent_info
		{
			std::array<double, 3> pos;
			std::array<double, 3> dest_pos;
			double dest_radius;
			int dest_agent;
			agent_state state;
			dtCrowdAgentParams param;
			std::uint32_t req_version = 0;
			std::function<void()> finish_cb;

		};
		const std::uint32_t m_max_agent_num;
	protected:
		dtCrowd m_detour_crowd;
		
		std::vector< agent_info> m_agents;
		std::vector<std::uint32_t> m_inactive_agent_idxes;
		std::vector<std::uint32_t> m_agent_ack_versions;
		std::uint64_t m_last_update_ts;
		std::vector<agent_req_info> m_agent_reqs;
		std::vector<agent_req_info> m_temp_agent_req_vec_in_update;
		std::vector<agent_req_info> m_temp_agent_req_vec_in_main;
		std::vector<agent_ack_info> m_agent_acks;
		std::vector<agent_ack_info> m_temp_agent_ack_vec_in_update;
		std::vector<agent_ack_info> m_temp_agent_ack_vec_in_main;
		std::mutex m_mutex;
		std::vector<double> m_half_extend;
		dtQueryFilter m_query_filter;

	protected:
		void add_agent_req(agent_req_info&& cur_req);
	public:
		async_detour_crowd(std::uint32_t max_agent_num);
		bool init(const std::string& nav_map);
		std::optional<std::uint32_t> add_agent(const dtCrowdAgentParams& param, const std::array<double, 3>& pos);
		bool remove_agent(std::uint32_t agent_idx);
		bool move_to_pos(std::uint32_t agent_idx, const std::array<double, 3>& dest_pos, double radius, std::function<void()> finish_cb);
		bool move_to_agent(std::uint32_t agent_idx, std::uint32_t dest_agent, double radius, std::function<void()> finish_cb);
		bool cancel_move(std::uint32_t agent_idx);
		bool change_pos(std::uint32_t agent_idx, const std::array<double, 3>& pos);
		bool update_param(std::uint32_t agent_idx, const dtCrowdAgentParams& param);
		static dtNavMesh* load_nav_file(const std::string& nav_map);
	protected:
		void update_dt_crowd(float dt);
		void process_acks();
		virtual void handle_agent_pos_update(std::uint32_t agent_idx);

	};
}