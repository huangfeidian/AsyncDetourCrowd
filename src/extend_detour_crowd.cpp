//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <new>
#include "extend_detour_crowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"

namespace spiritsaway::system::navigation
{
	ExtendDetourCrowd* dtAllocExtendDetourCrowd()
	{
		void* mem = dtAlloc(sizeof(ExtendDetourCrowd), DT_ALLOC_PERM);
		if (!mem) return 0;
		return new(mem) ExtendDetourCrowd;
	}

	void dtFreeExtendDetourCrowd(ExtendDetourCrowd* ptr)
	{
		if (!ptr) return;
		ptr->~ExtendDetourCrowd();
		dtFree(ptr);
	}


	static const int MAX_ITERS_PER_UPDATE = 100;

	static const int MAX_PATHQUEUE_NODES = 4096;
	static const int MAX_COMMON_NODES = 512;

	inline dtReal_t tween(const dtReal_t t, const dtReal_t t0, const dtReal_t t1)
	{
		return dtClamp<dtReal_t>((t - t0) / (t1 - t0), 0.0f, 1.0f);
	}

	static void integrate(ExtendDetourCrowdAgent* ag, const dtReal_t dt)
	{
		// Fake dynamic constraint.
		const dtReal_t maxDelta = ag->params.maxAcceleration * dt;
		dtReal_t dv[3];
		dtVsub(dv, ag->nvel, ag->vel);
		dtReal_t ds = dtVlen(dv);
		if (ds > maxDelta)
			dtVscale(dv, dv, maxDelta / ds);
		dtVadd(ag->vel, ag->vel, dv);

		// Integrate
		if (dtVlen(ag->vel) > 0.0001f)
			dtVmad(ag->npos, ag->npos, ag->vel, dt);
		else
			dtVset(ag->vel, 0, 0, 0);
	}

	static bool overOffmeshConnection(const ExtendDetourCrowdAgent* ag, const dtReal_t radius)
	{
		if (!ag->ncorners)
			return false;

		const bool offMeshConnection = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
		if (offMeshConnection)
		{
			const dtReal_t distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]);
			if (distSq < radius * radius)
				return true;
		}

		return false;
	}

	static dtReal_t getDistanceToGoal(const ExtendDetourCrowdAgent* ag, const dtReal_t range)
	{
		if (!ag->ncorners)
			return range;

		const bool endOfPath = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_END) ? true : false;
		if (endOfPath)
			return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]), range);

		return range;
	}

	static void calcSmoothSteerDirection(const ExtendDetourCrowdAgent* ag, dtReal_t* dir)
	{
		if (!ag->ncorners)
		{
			dtVset(dir, 0, 0, 0);
			return;
		}

		const int ip0 = 0;
		const int ip1 = dtMin(1, ag->ncorners - 1);
		const dtReal_t* p0 = &ag->cornerVerts[ip0 * 3];
		const dtReal_t* p1 = &ag->cornerVerts[ip1 * 3];

		dtReal_t dir0[3], dir1[3];
		dtVsub(dir0, p0, ag->npos);
		dtVsub(dir1, p1, ag->npos);
		dir0[1] = 0;
		dir1[1] = 0;

		dtReal_t len0 = dtVlen(dir0);
		dtReal_t len1 = dtVlen(dir1);
		if (len1 > 0.001f)
			dtVscale(dir1, dir1, 1.0f / len1);

		dir[0] = dir0[0] - dir1[0] * len0 * 0.5f;
		dir[1] = 0;
		dir[2] = dir0[2] - dir1[2] * len0 * 0.5f;

		dtVnormalize(dir);
	}

	static void calcStraightSteerDirection(const ExtendDetourCrowdAgent* ag, dtReal_t* dir)
	{
		if (!ag->ncorners)
		{
			dtVset(dir, 0, 0, 0);
			return;
		}
		dtVsub(dir, &ag->cornerVerts[0], ag->npos);
		dir[1] = 0;
		dtVnormalize(dir);
	}

	static int addNeighbour(const int idx, const dtReal_t dist,
		dtCrowdNeighbour* neis, const int nneis, const int maxNeis)
	{
		// Insert neighbour based on the distance.
		dtCrowdNeighbour* nei = 0;
		if (!nneis)
		{
			nei = &neis[nneis];
		}
		else if (dist >= neis[nneis - 1].dist)
		{
			if (nneis >= maxNeis)
				return nneis;
			nei = &neis[nneis];
		}
		else
		{
			int i;
			for (i = 0; i < nneis; ++i)
				if (dist <= neis[i].dist)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nneis - i, maxNeis - tgt);

			dtAssert(tgt + n <= maxNeis);

			if (n > 0)
				memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour) * n);
			nei = &neis[i];
		}

		memset(nei, 0, sizeof(dtCrowdNeighbour));

		nei->idx = idx;
		nei->dist = dist;

		return dtMin(nneis + 1, maxNeis);
	}

	static int getNeighbours(const dtReal_t* pos, const dtReal_t height, const dtReal_t range,
		const ExtendDetourCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult,
		ExtendDetourCrowdAgent** agents, const int /*nagents*/, dtProximityGrid* grid)
	{
		int n = 0;

		static const int MAX_NEIS = 32;
		unsigned short ids[MAX_NEIS];
		int nids = grid->queryItems(pos[0] - range, pos[2] - range,
			pos[0] + range, pos[2] + range,
			ids, MAX_NEIS);

		for (int i = 0; i < nids; ++i)
		{
			const ExtendDetourCrowdAgent* ag = agents[ids[i]];

			if (ag == skip) continue;

			// Check for overlap.
			dtReal_t diff[3];
			dtVsub(diff, pos, ag->npos);
			if (std::abs(diff[1]) >= (height + ag->params.height) / 2.0f)
				continue;
			diff[1] = 0;
			const dtReal_t distSqr = dtVlenSqr(diff);
			if (distSqr > dtSqr(range))
				continue;

			n = addNeighbour(ids[i], distSqr, result, n, maxResult);
		}
		return n;
	}

	static int addToOptQueue(ExtendDetourCrowdAgent* newag, ExtendDetourCrowdAgent** agents, const int nagents, const int maxAgents)
	{
		// Insert neighbour based on greatest time.
		int slot = 0;
		if (!nagents)
		{
			slot = nagents;

		}
		else if (newag->topologyOptTime <= agents[nagents - 1]->topologyOptTime)
		{
			if (nagents >= maxAgents)
				return nagents;
			slot = nagents;
		}
		else
		{
			int i;
			for (i = 0; i < nagents; ++i)
				if (newag->topologyOptTime >= agents[i]->topologyOptTime)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nagents - i, maxAgents - tgt);

			dtAssert(tgt + n <= maxAgents);

			if (n > 0)
				memmove(&agents[tgt], &agents[i], sizeof(ExtendDetourCrowdAgent*) * n);
			slot = i;
		}

		agents[slot] = newag;

		return dtMin(nagents + 1, maxAgents);
	}

	static int addToPathQueue(ExtendDetourCrowdAgent* newag, ExtendDetourCrowdAgent** agents, const int nagents, const int maxAgents)
	{
		// Insert neighbour based on greatest time.
		int slot = 0;
		if (!nagents)
		{
			slot = nagents;
		}
		else if (newag->targetReplanTime <= agents[nagents - 1]->targetReplanTime)
		{
			if (nagents >= maxAgents)
				return nagents;
			slot = nagents;
		}
		else
		{
			int i;
			for (i = 0; i < nagents; ++i)
				if (newag->targetReplanTime >= agents[i]->targetReplanTime)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nagents - i, maxAgents - tgt);

			dtAssert(tgt + n <= maxAgents);

			if (n > 0)
				memmove(&agents[tgt], &agents[i], sizeof(ExtendDetourCrowdAgent*) * n);
			slot = i;
		}

		agents[slot] = newag;

		return dtMin(nagents + 1, maxAgents);
	}


	/**
	@class ExtendDetourCrowd
	@par

	This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
	of the crowd features.

	A common method for setting up the crowd is as follows:

	-# Allocate the crowd using #dtAllocExtendDetourCrowd.
	-# Initialize the crowd using #init().
	-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

	A common process for managing the crowd is as follows:

	-# Call #update() to allow the crowd to manage its agents.
	-# Retrieve agent information using #getActiveAgents().
	-# Make movement requests using #requestMoveTarget() when movement goal changes.
	-# Repeat every frame.

	Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
	agent position.  So it is not possible to update an active agent's position.  If agent position
	must be fed back into the crowd, the agent must be removed and re-added.

	Notes:

	- Path related information is available for newly added agents only after an #update() has been
	  performed.
	- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
	  #ExtendDetourCrowdAgent::active to determine if the agent is actually in use or not.
	- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.
	  So it is not meant to provide automatic pathfinding services over long distances.

	@see dtAllocExtendDetourCrowd(), dtFreeExtendDetourCrowd(), init(), ExtendDetourCrowdAgent

	*/

	ExtendDetourCrowd::ExtendDetourCrowd() :
		m_maxAgents(0),
		m_agents(0),
		m_activeAgents(0),
		m_agentAnims(0),
		//m_obstacleQuery(0),
		//m_grid(0),
		m_pathResult(0),
		m_maxPathResult(0),
		m_maxAgentRadius(0),
		m_velocitySampleCount(0),
		m_navquery(0)
	{
	}

	ExtendDetourCrowd::~ExtendDetourCrowd()
	{
		purge();
	}

	void ExtendDetourCrowd::purge()
	{
		for (int i = 0; i < m_maxAgents; ++i)
			m_agents[i].~ExtendDetourCrowdAgent();
		dtFree(m_agents);
		m_agents = 0;
		m_maxAgents = 0;

		dtFree(m_activeAgents);
		m_activeAgents = 0;

		dtFree(m_agentAnims);
		m_agentAnims = 0;

		dtFree(m_pathResult);
		m_pathResult = 0;


		dtFreeNavMeshQuery(m_navquery);
		m_navquery = 0;
	}

	/// @par
	///
	/// May be called more than once to purge and re-initialize the crowd.
	std::uint32_t ExtendDetourCrowd::init(const int maxAgents, const dtReal_t maxAgentRadius, const dtNavMesh* nav)
	{
		purge();

		m_maxAgents = maxAgents;
		m_maxAgentRadius = maxAgentRadius;

		// Larger than agent radius because it is also used for agent recovery.
		dtVset(m_agentPlacementHalfExtents, m_maxAgentRadius * 2.0f, m_maxAgentRadius * 1.5f, m_maxAgentRadius * 2.0f);

		m_grid = dtAllocProximityGrid();
		if (!m_grid)
			return false;
		if (!m_grid->init(m_maxAgents * 4, maxAgentRadius * 3))
			return false;

		m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
		if (!m_obstacleQuery)
			return false;
		if (!m_obstacleQuery->init(6, 8))
			return false;

		// Init obstacle query params.
		memset(m_obstacleQueryParams, 0, sizeof(m_obstacleQueryParams));
		for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
		{
			dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[i];
			params->velBias = 0.4f;
			params->weightDesVel = 2.0f;
			params->weightCurVel = 0.75f;
			params->weightSide = 0.75f;
			params->weightToi = 2.5f;
			params->horizTime = 2.5f;
			params->gridSize = 33;
			params->adaptiveDivs = 7;
			params->adaptiveRings = 2;
			params->adaptiveDepth = 5;
		}


		// Allocate temp buffer for merging paths.
		m_maxPathResult = 256;
		m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef) * m_maxPathResult, DT_ALLOC_PERM);
		if (!m_pathResult)
			return __LINE__;

		if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
			return __LINE__;

		m_agents = (ExtendDetourCrowdAgent*)dtAlloc(sizeof(ExtendDetourCrowdAgent) * m_maxAgents, DT_ALLOC_PERM);
		if (!m_agents)
			return __LINE__;

		m_activeAgents = (ExtendDetourCrowdAgent**)dtAlloc(sizeof(ExtendDetourCrowdAgent*) * m_maxAgents, DT_ALLOC_PERM);
		if (!m_activeAgents)
			return __LINE__;

		m_agentAnims = (ExtendDetourCrowdAgentAnimation*)dtAlloc(sizeof(ExtendDetourCrowdAgentAnimation) * m_maxAgents, DT_ALLOC_PERM);
		if (!m_agentAnims)
			return __LINE__;

		for (int i = 0; i < m_maxAgents; ++i)
		{
			new(&m_agents[i]) ExtendDetourCrowdAgent();
			m_agents[i].active = false;
			if (!m_agents[i].corridor.init(m_maxPathResult))
				return __LINE__;
		}

		for (int i = 0; i < m_maxAgents; ++i)
		{
			m_agentAnims[i].active = false;
		}

		// The navquery is mostly used for local searches, no need for large node pool.
		m_navquery = dtAllocNavMeshQuery();
		if (!m_navquery)
			return __LINE__;
		if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
			return __LINE__;

		return 0;
	}

	int ExtendDetourCrowd::getAgentCount() const
	{
		return m_maxAgents;
	}

	/// @par
	/// 
	/// Agents in the pool may not be in use.  Check #ExtendDetourCrowdAgent.active before using the returned object.
	const ExtendDetourCrowdAgent* ExtendDetourCrowd::getAgent(const int idx)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return 0;
		return &m_agents[idx];
	}

	/// 
	/// Agents in the pool may not be in use.  Check #ExtendDetourCrowdAgent.active before using the returned object.
	ExtendDetourCrowdAgent* ExtendDetourCrowd::getEditableAgent(const int idx)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return 0;
		return &m_agents[idx];
	}

	void ExtendDetourCrowd::updateAgentParameters(const int idx, const ExtendDetourCrowdAgentParams* params)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return;
		memcpy(&m_agents[idx].params, params, sizeof(ExtendDetourCrowdAgentParams));
	}

	void ExtendDetourCrowd::updateAgentPos(const int idx, const dtReal_t* pos)
	{
		auto ag = &m_agents[idx];
		if (!ag->active || ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_IDLE)
		{
			return;
		}
		// Find nearest position on navmesh and place the agent there.
		dtReal_t nearest[3];
		dtPolyRef ref = 0;
		dtVcopy(nearest, pos);
		dtStatus status = m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest);
		if (dtStatusFailed(status))
		{
			dtVcopy(nearest, pos);
			ref = 0;
		}
		dtVcopy(ag->npos, nearest);
		ag->corridor.reset(ref, nearest);
		ag->boundary.reset();
	}

	void ExtendDetourCrowd::updateAgentPolyPos(const int idx, dtPolyRef ref, const dtReal_t* pos)
	{
		auto ag = &m_agents[idx];
		if (!ag->active || ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_IDLE)
		{
			return;
		}
		// Find nearest position on navmesh and place the agent there.
		dtReal_t nearest[3];
		dtVcopy(nearest, pos);
		if (!ref)
		{
			dtStatus status = m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest);
			if (dtStatusFailed(status))
			{
				dtVcopy(nearest, pos);
				ref = 0;
			}
		}


		dtVcopy(ag->npos, nearest);
		ag->corridor.reset(ref, nearest);
		ag->boundary.reset();
	}

	void ExtendDetourCrowd::moveAlongSurface(const int idx, const dtReal_t* dis)
	{
		auto ag = &m_agents[idx];
		if (!ag->active || ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_IDLE)
		{
			return;
		}
		dtReal_t new_pos[3];
		dtVadd(new_pos, ag->npos, dis);
		if (dtVlen(dis) > ag->params.radius * 2)
		{

			updateAgentPos(idx, new_pos);
			return;
		}
		dtReal_t nearest[3];
		static const int ref_count = 8;
		dtPolyRef visited_refs[ref_count];
		int result_ref_count = 0;
		dtStatus status = m_navquery->moveAlongSurface(ag->corridor.getFirstPoly(), ag->npos, new_pos, &m_filters[ag->params.queryFilterType], nearest, visited_refs, &result_ref_count, ref_count);
		if (dtStatusSucceed(status) && !dtStatusDetail(status, DT_BUFFER_TOO_SMALL))
		{
			dtVcopy(ag->npos, nearest);
			ag->corridor.reset(visited_refs[result_ref_count - 1], nearest);
			ag->boundary.reset();
		}
		else
		{
			updateAgentPos(idx, new_pos);
		}

	}

	void ExtendDetourCrowd::addAgentImpl(const int idx, const dtReal_t* pos, const ExtendDetourCrowdAgentParams* params)
	{
		auto ag = &m_agents[idx];

		updateAgentParameters(idx, params);

		// Find nearest position on navmesh and place the agent there.
		dtReal_t nearest[3];
		dtPolyRef ref = 0;
		dtVcopy(nearest, pos);
		dtStatus status = m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest);
		if (dtStatusFailed(status))
		{
			dtVcopy(nearest, pos);
			ref = 0;
		}

		ag->corridor.reset(ref, nearest);
		ag->boundary.reset();
		ag->partial = false;

		ag->topologyOptTime = 0;
		ag->targetReplanTime = 0;
		ag->nneis = 0;

		dtVset(ag->dvel, 0, 0, 0);
		dtVset(ag->nvel, 0, 0, 0);
		dtVset(ag->vel, 0, 0, 0);
		dtVcopy(ag->npos, nearest);

		ag->desiredSpeed = 0;

		if (ref)
			ag->state = ExtendDetourCrowdAgentState::STATE_WALKING;
		else
			ag->state = ExtendDetourCrowdAgentState::STATE_INVALID;

		ag->targetState = ExtendDetourMoveRequestState::TARGET_NONE;

		if (ref)
			ag->state = ExtendDetourCrowdAgentState::STATE_WALKING;
		else
			ag->state = ExtendDetourCrowdAgentState::STATE_INVALID;
		//printf("add agent %d with state %d \n", idx, ag->state);
		ag->targetState = ExtendDetourMoveRequestState::TARGET_NONE;
		ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_IDLE;
		ag->follow_by_agent_count = 0;
		ag->target_agent = -1;
		ag->active = true;

	}

	/// @par
	///
	/// The agent's position will be constrained to the surface of the navigation mesh.
	int ExtendDetourCrowd::addAgent(const dtReal_t* pos, const ExtendDetourCrowdAgentParams* params)
	{
		// Find empty slot.
		int idx = -1;
		for (int i = 0; i < m_maxAgents; ++i)
		{
			if (!m_agents[i].active)
			{
				idx = i;
				break;
			}
		}
		if (idx == -1)
			return -1;

		addAgentImpl(idx, pos, params);
		return idx;
	}

	bool ExtendDetourCrowd::addAgent(int idx, const dtReal_t* pos, const ExtendDetourCrowdAgentParams* params)
	{
		if (idx < 0 || idx >= m_maxAgents)
		{
			return false;
		}
		if (m_agents[idx].active)
		{
			return false;
		}
		addAgentImpl(idx, pos, params);
		return true;
	}


	/// @par
	///
	/// The agent is deactivated and will no longer be processed.  Its #ExtendDetourCrowdAgent object
	/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
	void ExtendDetourCrowd::removeAgent(const int idx)
	{
		if (idx >= 0 && idx < m_maxAgents)
		{
			if (!m_agents[idx].active)
			{
				return;
			}
			if (m_agents[idx].target_agent != -1)
			{
				m_agents[m_agents[idx].target_agent].follow_by_agent_count--;
				m_agents[idx].target_agent = -1;
			}
			m_agents[idx].active = false;
			m_agents[idx].move_state = ExtendDetourCrowdAgentMoveState::STATE_IDLE;
			if (!m_agents[idx].follow_by_agent_count)
			{
				return;
			}

			m_agents[idx].follow_by_agent_count = 0;

			for (int i = 0; i < m_maxAgents; i++)
			{
				auto cur_agent = &m_agents[i];
				if (cur_agent->target_agent == idx)
				{
					resetMoveTarget(i);
					cur_agent->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;
				}

			}

		}
	}

	bool ExtendDetourCrowd::requestMoveTargetReplan(const int idx, dtPolyRef ref, const dtReal_t* pos)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return false;

		ExtendDetourCrowdAgent* ag = &m_agents[idx];

		// Initialize request.
		ag->targetRef = ref;
		dtVcopy(ag->targetPos, pos);
		ag->targetPathqRef = DT_PATHQ_INVALID;
		ag->targetReplan = true;
		if (ag->targetRef)
			ag->targetState = ExtendDetourMoveRequestState::TARGET_REQUESTING;
		else
			ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;

		return true;
	}

	void ExtendDetourCrowd::updateAgentFollowTarget(ExtendDetourCrowdAgent* ag)
	{
		ExtendDetourCrowdAgent* target_ag = &m_agents[ag->target_agent];

		dtReal_t nearest[3];
		dtVcopy(nearest, target_ag->npos);
		dtPolyRef agentPolyRef = 0;
		m_navquery->findNearestPoly(target_ag->npos, m_agentPlacementHalfExtents, &m_filters[target_ag->params.queryFilterType], &agentPolyRef, nearest);
		ag->targetRef = agentPolyRef;
		dtVcopy(ag->targetPos, nearest);
	}
	/// @par
	/// 
	/// This method is used when a new target is set.
	/// 
	/// The position will be constrained to the surface of the navigation mesh.
	///
	/// The request will be processed during the next #update().
	bool ExtendDetourCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const dtReal_t* pos, const dtReal_t target_radius)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return false;
		if (!ref)
			return false;
		if (target_radius < 0)
		{
			return false;
		}

		ExtendDetourCrowdAgent* ag = &m_agents[idx];
		if (ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_IDLE)
		{
			return false;
		}
		if (ref)
		{
			ag->targetRef = ref;
			dtVcopy(ag->targetPos, pos);

		}
		else
		{
			m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ag->targetRef, ag->targetPos);
		}
		// Initialize request.

		ag->targetPathqRef = DT_PATHQ_INVALID;
		ag->targetReplan = false;

		ag->target_radius = target_radius;
		if (ag->targetRef)
			ag->targetState = ExtendDetourMoveRequestState::TARGET_REQUESTING;
		else
			ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;
		ag->state = ExtendDetourCrowdAgentState::STATE_WALKING;
		ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_MOVING;
		return true;
	}

	bool ExtendDetourCrowd::requestMoveTarget(const int idx, const int target_idx, const dtReal_t target_radius, const bool follow)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return false;
		if (target_idx < 0 || target_idx >= m_maxAgents)
		{
			return false;
		}
		if (!m_agents[idx].active || !m_agents[target_idx].active)
		{
			return false;
		}
		if (target_radius < 0)
		{
			return false;
		}
		ExtendDetourCrowdAgent* ag = &m_agents[idx];
		if (ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_IDLE)
		{
			return false;
		}
		ag->target_agent = target_idx;
		m_agents[target_idx].follow_by_agent_count++;
		ag->continue_follow = follow;
		// Initialize request.

		updateAgentFollowTarget(ag);
		ag->targetPathqRef = DT_PATHQ_INVALID;
		ag->targetReplan = false;
		ag->state = ExtendDetourCrowdAgentState::STATE_WALKING;
		ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_MOVING;
		ag->target_radius = target_radius;
		if (ag->targetRef)
			ag->targetState = ExtendDetourMoveRequestState::TARGET_REQUESTING;
		else
			ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;

		return true;
	}

	bool ExtendDetourCrowd::requestMoveVelocity(const int idx, const dtReal_t* vel)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return false;

		ExtendDetourCrowdAgent* ag = &m_agents[idx];
		// Initialize request.
		ag->targetRef = 0;
		dtVcopy(ag->targetPos, vel);
		ag->targetPathqRef = DT_PATHQ_INVALID;
		ag->targetReplan = false;
		ag->targetState = ExtendDetourMoveRequestState::TARGET_VELOCITY;
		ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_MOVING;
		ag->state = ExtendDetourCrowdAgentState::STATE_WALKING;


		return true;
	}

	bool ExtendDetourCrowd::resetMoveTarget(const int idx)
	{
		if (idx < 0 || idx >= m_maxAgents)
			return false;

		ExtendDetourCrowdAgent* ag = &m_agents[idx];

		// Initialize request.
		ag->targetRef = 0;
		dtVset(ag->targetPos, 0, 0, 0);
		dtVset(ag->dvel, 0, 0, 0);
		ag->targetPathqRef = DT_PATHQ_INVALID;
		ag->targetReplan = false;
		ag->targetState = ExtendDetourMoveRequestState::TARGET_NONE;
		ag->state = ExtendDetourCrowdAgentState::STATE_INVALID;
		ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_IDLE;
		ag->target_radius = 0;
		if (ag->continue_follow && ag->target_agent != -1)
		{
			auto target_ag = &m_agents[ag->target_agent];
			target_ag->follow_by_agent_count--;
		}
		ag->continue_follow = false;
		ag->target_agent = -1;
		return true;
	}

	int ExtendDetourCrowd::getActiveAgents(ExtendDetourCrowdAgent** agents, const int maxAgents)
	{
		int n = 0;
		for (int i = 0; i < m_maxAgents; ++i)
		{
			if (!m_agents[i].active) continue;
			if (n < maxAgents)
				agents[n++] = &m_agents[i];
		}
		return n;
	}


	void ExtendDetourCrowd::updateMoveRequest(const dtReal_t /*dt*/)
	{
		const int PATH_MAX_AGENTS = 8;
		ExtendDetourCrowdAgent* queue[PATH_MAX_AGENTS];
		int nqueue = 0;

		// Fire off new requests.
		for (int i = 0; i < m_maxAgents; ++i)
		{
			ExtendDetourCrowdAgent* ag = &m_agents[i];
			if (!ag->active)
				continue;
			if (ag->state == ExtendDetourCrowdAgentState::STATE_INVALID)
				continue;
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
				continue;

			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_REQUESTING)
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);

				static const int MAX_RES = 32;
				dtReal_t reqPos[3];
				dtPolyRef reqPath[MAX_RES];	// The path to the request location
				int reqPathCount = 0;

				// Quick search towards the goal.
				static const int MAX_ITER = 20;
				m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filters[ag->params.queryFilterType]);
				m_navquery->updateSlicedFindPath(MAX_ITER, 0);
				dtStatus status = 0;
				if (ag->targetReplan) // && npath > 10)
				{
					// Try to use existing steady path during replan if possible.
					status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
				}
				else
				{
					// Try to move towards target when goal changes.
					status = m_navquery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
				}

				if (!dtStatusFailed(status) && reqPathCount > 0)
				{
					// In progress or succeed.
					if (reqPath[reqPathCount - 1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						status = m_navquery->closestPointOnPoly(reqPath[reqPathCount - 1], ag->targetPos, reqPos, 0);
						if (dtStatusFailed(status))
							reqPathCount = 0;
					}
					else
					{
						dtVcopy(reqPos, ag->targetPos);
					}
				}
				else
				{
					reqPathCount = 0;
				}

				if (!reqPathCount)
				{
					// Could not find path, start the request from current location.
					dtVcopy(reqPos, ag->npos);
					reqPath[0] = path[0];
					reqPathCount = 1;
				}

				ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
				ag->boundary.reset();
				ag->partial = false;

				if (reqPath[reqPathCount - 1] == ag->targetRef)
				{
					ag->targetState = ExtendDetourMoveRequestState::TARGET_VALID;
					ag->targetReplanTime = 0.0;
				}
				else
				{
					// The path is longer or potentially unreachable, full plan.
					ag->targetState = ExtendDetourMoveRequestState::TARGET_WAITING_FOR_QUEUE;
				}
			}

			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_WAITING_FOR_QUEUE)
			{
				nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
			}
		}

		for (int i = 0; i < nqueue; ++i)
		{
			ExtendDetourCrowdAgent* ag = queue[i];
			ag->targetPathqRef = m_pathq.request(ag->corridor.getLastPoly(), ag->targetRef,
				ag->corridor.getTarget(), ag->targetPos, &m_filters[ag->params.queryFilterType]);
			if (ag->targetPathqRef != DT_PATHQ_INVALID)
				ag->targetState = ExtendDetourMoveRequestState::TARGET_WAITING_FOR_PATH;
		}


		// Update requests.
		m_pathq.update(MAX_ITERS_PER_UPDATE);

		dtStatus status;

		// Process path results.
		for (int i = 0; i < m_maxAgents; ++i)
		{
			ExtendDetourCrowdAgent* ag = &m_agents[i];
			if (!ag->active)
				continue;
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
				continue;

			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_WAITING_FOR_PATH)
			{
				// Poll path queue.
				status = m_pathq.getRequestStatus(ag->targetPathqRef);
				if (dtStatusFailed(status))
				{
					// Path find failed, retry if the target location is still valid.
					ag->targetPathqRef = DT_PATHQ_INVALID;
					if (ag->targetRef)
						ag->targetState = ExtendDetourMoveRequestState::TARGET_REQUESTING;
					else
						ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;
					ag->targetReplanTime = 0.0;
				}
				else if (dtStatusSucceed(status))
				{
					const dtPolyRef* path = ag->corridor.getPath();
					const int npath = ag->corridor.getPathCount();
					dtAssert(npath);

					// Apply results.
					dtReal_t targetPos[3];
					dtVcopy(targetPos, ag->targetPos);

					dtPolyRef* res = m_pathResult;
					bool valid = true;
					int nres = 0;
					status = m_pathq.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathResult);
					if (dtStatusFailed(status) || !nres)
						valid = false;

					if (dtStatusDetail(status, DT_PARTIAL_RESULT))
						ag->partial = true;
					else
						ag->partial = false;

					// Merge result and existing path.
					// The agent might have moved whilst the request is
					// being processed, so the path may have changed.
					// We assume that the end of the path is at the same location
					// where the request was issued.

					// The last ref in the old path should be the same as
					// the location where the request was issued..
					if (valid && path[npath - 1] != res[0])
						valid = false;

					if (valid)
					{
						// Put the old path infront of the old path.
						if (npath > 1)
						{
							// Make space for the old path.
							if ((npath - 1) + nres > m_maxPathResult)
								nres = m_maxPathResult - (npath - 1);

							memmove(res + npath - 1, res, sizeof(dtPolyRef) * nres);
							// Copy old path in the beginning.
							memcpy(res, path, sizeof(dtPolyRef) * (npath - 1));
							nres += npath - 1;

							// Remove trackbacks
							for (int j = 0; j < nres; ++j)
							{
								if (j - 1 >= 0 && j + 1 < nres)
								{
									if (res[j - 1] == res[j + 1])
									{
										memmove(res + (j - 1), res + (j + 1), sizeof(dtPolyRef) * (nres - (j + 1)));
										nres -= 2;
										j -= 2;
									}
								}
							}

						}

						// Check for partial path.
						if (res[nres - 1] != ag->targetRef)
						{
							// Partial path, constrain target position inside the last polygon.
							dtReal_t nearest[3];
							status = m_navquery->closestPointOnPoly(res[nres - 1], targetPos, nearest, 0);
							if (dtStatusSucceed(status))
								dtVcopy(targetPos, nearest);
							else
								valid = false;
						}
					}

					if (valid)
					{
						// Set current corridor.
						ag->corridor.setCorridor(targetPos, res, nres);
						ag->boundary.reset();
						ag->targetState = ExtendDetourMoveRequestState::TARGET_VALID;
					}
					else
					{
						// Something went wrong.
						ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;
					}

					ag->targetReplanTime = 0.0;
				}
			}
		}

	}


	void ExtendDetourCrowd::updateTopologyOptimization(ExtendDetourCrowdAgent** agents, const int nagents, const dtReal_t dt)
	{
		if (!nagents)
			return;

		const dtReal_t OPT_TIME_THR = 0.5f; // seconds
		const int OPT_MAX_AGENTS = 1;
		ExtendDetourCrowdAgent* queue[OPT_MAX_AGENTS];
		int nqueue = 0;

		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];
			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
				continue;
			if ((ag->params.updateFlags & std::uint8_t(ExtendDetourCrowdUpdateFlags::OPTIMIZE_TOPO)) == 0)
				continue;
			ag->topologyOptTime += dt;
			if (ag->topologyOptTime >= OPT_TIME_THR)
				nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
		}

		for (int i = 0; i < nqueue; ++i)
		{
			ExtendDetourCrowdAgent* ag = queue[i];
			ag->corridor.optimizePathTopology(m_navquery, &m_filters[ag->params.queryFilterType]);
			ag->topologyOptTime = 0;
		}

	}

	void ExtendDetourCrowd::checkPathValidity(ExtendDetourCrowdAgent** agents, const int nagents, const dtReal_t dt)
	{
		static const int CHECK_LOOKAHEAD = 10;
		static const dtReal_t TARGET_REPLAN_DELAY = 1.0; // seconds

		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];

			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;

			ag->targetReplanTime += dt;

			bool replan = false;

			// First check that the current location is valid.
			const int idx = getAgentIndex(ag);
			dtReal_t agentPos[3];
			dtPolyRef agentRef = ag->corridor.getFirstPoly();
			dtVcopy(agentPos, ag->npos);
			if (!m_navquery->isValidPolyRef(agentRef, &m_filters[ag->params.queryFilterType]))
			{
				// Current location is not valid, try to reposition.
				// TODO: this can snap agents, how to handle that?
				dtReal_t nearest[3];
				dtVcopy(nearest, agentPos);
				agentRef = 0;
				m_navquery->findNearestPoly(ag->npos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &agentRef, nearest);
				dtVcopy(agentPos, nearest);

				if (!agentRef)
				{
					// Could not find location in navmesh, set state to invalid.
					ag->corridor.reset(0, agentPos);
					ag->partial = false;
					ag->boundary.reset();
					ag->state = ExtendDetourCrowdAgentState::STATE_INVALID;
					//printf("agent %d checkPathValidity state changeto invalid \n", getAgentIndex(ag));
					continue;
				}

				// Make sure the first polygon is valid, but leave other valid
				// polygons in the path so that replanner can adjust the path better.
				ag->corridor.fixPathStart(agentRef, agentPos);
				//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
				ag->boundary.reset();
				dtVcopy(ag->npos, agentPos);

				replan = true;
			}

			// If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
				continue;

			// Try to recover move request position.
			if (ag->targetState != ExtendDetourMoveRequestState::TARGET_NONE && ag->targetState != ExtendDetourMoveRequestState::TARGET_FAILED)
			{
				if (!m_navquery->isValidPolyRef(ag->targetRef, &m_filters[ag->params.queryFilterType]))
				{
					// Current target is not valid, try to reposition.
					dtReal_t nearest[3];
					dtVcopy(nearest, ag->targetPos);
					ag->targetRef = 0;
					m_navquery->findNearestPoly(ag->targetPos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ag->targetRef, nearest);
					dtVcopy(ag->targetPos, nearest);
					replan = true;
				}
				if (!ag->targetRef)
				{
					// Failed to reposition target, fail moverequest.
					ag->corridor.reset(agentRef, agentPos);
					ag->partial = false;
					ag->targetState = ExtendDetourMoveRequestState::TARGET_NONE;
				}
			}

			// If nearby corridor is not valid, replan.
			if (!ag->corridor.isValid(CHECK_LOOKAHEAD, m_navquery, &m_filters[ag->params.queryFilterType]))
			{
				// Fix current path.
	//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
	//			ag->boundary.reset();
				replan = true;
			}

			// If the end of the path is near and it is not the requested location, replan.
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_VALID)
			{
				if (ag->targetReplanTime > TARGET_REPLAN_DELAY &&
					ag->corridor.getPathCount() < CHECK_LOOKAHEAD &&
					ag->corridor.getLastPoly() != ag->targetRef)
					replan = true;
			}

			// Try to replan path to goal.
			if (replan)
			{
				if (ag->targetState != ExtendDetourMoveRequestState::TARGET_NONE)
				{
					requestMoveTargetReplan(idx, ag->targetRef, ag->targetPos);
				}
			}
		}
	}
	void ExtendDetourCrowd::updateProximity(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		// Register agents to proximity grid.
		m_grid->clear();
		for (int i = 0; i < nagents; ++i)
		{
			auto ag = agents[i];
			const dtReal_t* p = ag->npos;
			const dtReal_t r = ag->params.radius;
			m_grid->addItem((unsigned short)i, p[0] - r, p[2] - r, p[0] + r, p[2] + r);
		}

		// Get nearby navmesh segments and agents to collide with.
		for (int i = 0; i < nagents; ++i)
		{
			auto ag = agents[i];
			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING)
				continue;

			// Update the collision boundary after certain distance has been passed or
			// if it has become invalid.
			const dtReal_t updateThr = ag->params.collisionQueryRange * 0.25f;
			if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
				!ag->boundary.isValid(m_navquery, &m_filters[ag->params.queryFilterType]))
			{
				ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
					m_navquery, &m_filters[ag->params.queryFilterType]);
			}
			// Query neighbour agents
			ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
				ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
				agents, nagents, m_grid);
			for (int j = 0; j < ag->nneis; j++)
				ag->neis[j].idx = getAgentIndex(agents[ag->neis[j].idx]);
		}
	}

	void ExtendDetourCrowd::findCornerSteerTo(ExtendDetourCrowdAgent** agents, const int nagents, ExtendDetourCrowdAgentDebugInfo* debug)
	{
		const int debugIdx = debug ? debug->idx : -1;
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];

			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
				continue;

			// Find corners for steering
			ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
				DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filters[ag->params.queryFilterType]);

			// Check to see if the corner after the next corner is directly visible,
			// and short cut to there.
			if ((ag->params.updateFlags & std::uint8_t(ExtendDetourCrowdUpdateFlags::OPTIMIZE_VIS)) && ag->ncorners > 0)
			{
				const dtReal_t* target = &ag->cornerVerts[dtMin(1, ag->ncorners - 1) * 3];
				ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filters[ag->params.queryFilterType]);

				// Copy data for debug purposes.
				if (debugIdx == i)
				{
					dtVcopy(debug->optStart, ag->corridor.getPos());
					dtVcopy(debug->optEnd, target);
				}
			}
			else
			{
				// Copy data for debug purposes.
				if (debugIdx == i)
				{
					dtVset(debug->optStart, 0, 0, 0);
					dtVset(debug->optEnd, 0, 0, 0);
				}
			}
		}
	}
	void ExtendDetourCrowd::trigOffmeshconnection(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];

			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
				continue;

			// Check 
			const dtReal_t triggerRadius = ag->params.radius * 2.25f;
			if (overOffmeshConnection(ag, triggerRadius))
			{
				// Prepare to off-mesh connection.
				const int idx = (int)(ag - m_agents);
				ExtendDetourCrowdAgentAnimation* anim = &m_agentAnims[idx];

				// Adjust the path over the off-mesh connection.
				dtPolyRef refs[2];
				if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners - 1], refs,
					anim->startPos, anim->endPos, m_navquery))
				{
					dtVcopy(anim->initPos, ag->npos);
					anim->polyRef = refs[1];
					anim->active = true;
					anim->t = 0.0f;
					anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;

					ag->state = ExtendDetourCrowdAgentState::STATE_OFFMESH;
					ag->ncorners = 0;
					//ag->nneis = 0;
					continue;
				}
				else
				{
					// Path validity check will ensure that bad/blocked connections will be replanned.
				}
			}
		}
	}
	void ExtendDetourCrowd::calculateSteering(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];

			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE)
				continue;

			dtReal_t dvel[3] = { 0,0,0 };

			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
			{
				dtVcopy(dvel, ag->targetPos);
				ag->desiredSpeed = dtVlen(ag->targetPos);
			}
			else
			{
				// Calculate steering direction.
				if (ag->params.updateFlags & std::uint8_t(ExtendDetourCrowdUpdateFlags::ANTICIPATE_TURNS))
					calcSmoothSteerDirection(ag, dvel);
				else
					calcStraightSteerDirection(ag, dvel);

				// Calculate speed scale, which tells the agent to slowdown at the end of the path.
				const dtReal_t slowDownRadius = ag->params.radius * 2;	// TODO: make less hacky.
				const dtReal_t speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;

				ag->desiredSpeed = ag->params.maxSpeed;
				dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
			}
			// Separation
			if (ag->params.updateFlags & DT_CROWD_SEPARATION)
			{
				const dtReal_t separationDist = ag->params.collisionQueryRange;
				const dtReal_t invSeparationDist = 1.0f / separationDist;
				const dtReal_t separationWeight = ag->params.separationWeight;

				dtReal_t w = 0;
				dtReal_t disp[3] = { 0,0,0 };

				for (int j = 0; j < ag->nneis; ++j)
				{
					const ExtendDetourCrowdAgent* nei = &m_agents[ag->neis[j].idx];

					dtReal_t diff[3];
					dtVsub(diff, ag->npos, nei->npos);
					diff[1] = 0;

					const dtReal_t distSqr = dtVlenSqr(diff);
					if (distSqr < 0.00001f)
						continue;
					if (distSqr > dtSqr(separationDist))
						continue;
					const dtReal_t dist = std::sqrt(distSqr);
					const dtReal_t weight = separationWeight * (1.0f - dtSqr(dist * invSeparationDist));

					dtVmad(disp, disp, diff, weight / dist);
					w += 1.0f;
				}

				if (w > 0.0001f)
				{
					// Adjust desired velocity.
					dtVmad(dvel, dvel, disp, 1.0f / w);
					// Clamp desired velocity to desired speed.
					const dtReal_t speedSqr = dtVlenSqr(dvel);
					const dtReal_t desiredSqr = dtSqr(ag->desiredSpeed);
					if (speedSqr > desiredSqr)
						dtVscale(dvel, dvel, desiredSqr / speedSqr);
				}
			}
			// Set the desired velocity.
			dtVcopy(ag->dvel, dvel);
		}
	}
	void ExtendDetourCrowd::updateOffmeshConnection(ExtendDetourCrowdAgent** agents, const int nagents, ExtendDetourCrowdAgentDebugInfo* debug, const dtReal_t dt)
	{
		for (int i = 0; i < m_maxAgents; ++i)
		{
			ExtendDetourCrowdAgentAnimation* anim = &m_agentAnims[i];
			if (!anim->active)
				continue;
			ExtendDetourCrowdAgent* ag = agents[i];

			anim->t += dt;
			if (anim->t > anim->tmax)
			{
				// Reset animation
				anim->active = false;
				// Prepare agent for walking.
				ag->state = ExtendDetourCrowdAgentState::STATE_WALKING;
				continue;
			}

			// Update position
			const dtReal_t ta = anim->tmax * 0.15f;
			const dtReal_t tb = anim->tmax;
			if (anim->t < ta)
			{
				const dtReal_t u = tween(anim->t, 0.0, ta);
				dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
			}
			else
			{
				const dtReal_t u = tween(anim->t, ta, tb);
				dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
			}

			// Update velocity.
			dtVset(ag->vel, 0, 0, 0);
			dtVset(ag->dvel, 0, 0, 0);
		}
	}
	void ExtendDetourCrowd::moveCorridorPos(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];
			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;

			// Move along navmesh.
			ag->corridor.movePosition(ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);
			// Get valid constrained position back.
			dtVcopy(ag->npos, ag->corridor.getPos());

			// If not using path, truncate the corridor to just one poly.
			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_NONE || ag->targetState == ExtendDetourMoveRequestState::TARGET_VELOCITY)
			{
				ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
				ag->partial = false;
			}

		}
	}

	void ExtendDetourCrowd::velocityPlan(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];

			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				continue;
			if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
			{
				m_obstacleQuery->reset();

				// Add neighbours as obstacles.
				for (int j = 0; j < ag->nneis; ++j)
				{
					const ExtendDetourCrowdAgent* nei = &m_agents[ag->neis[j].idx];
					m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
				}

				// Append neighbour segments as obstacles.
				for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
				{
					const dtReal_t* s = ag->boundary.getSegment(j);
					if (dtTriArea2D(ag->npos, s, s + 3) < 0.0f)
						continue;
					m_obstacleQuery->addSegment(s, s + 3);
				}

				//dtObstacleAvoidanceDebugData* vod = 0;
				//if (debugIdx == i)
				//	vod = debug->vod;

				// Sample new safe velocity.
				bool adaptive = true;
				int ns = 0;

				const dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];

				if (adaptive)
				{
					ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->params.radius, ag->desiredSpeed,
						ag->vel, ag->dvel, ag->nvel, params, nullptr);
				}
				else
				{
					ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->params.radius, ag->desiredSpeed,
						ag->vel, ag->dvel, ag->nvel, params, nullptr);
				}
				m_velocitySampleCount += ns;
			}
			else
			{
				// If not using velocity planning, new velocity is directly the desired velocity.
				dtVcopy(ag->nvel, ag->dvel);
			}

		}
	}

	void ExtendDetourCrowd::intergateVelocity(ExtendDetourCrowdAgent** agents, const int nagents, const dtReal_t dt)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];
			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING || ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
			{
				continue;
			}
			integrate(ag, dt);
		}
	}

	void ExtendDetourCrowd::handleCollision(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		// Handle collisions.
		static const dtReal_t COLLISION_RESOLVE_FACTOR = 0.7f;

		for (int iter = 0; iter < 4; ++iter)
		{
			for (int i = 0; i < nagents; ++i)
			{
				ExtendDetourCrowdAgent* ag = agents[i];
				const int idx0 = getAgentIndex(ag);

				if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING)
					continue;

				dtVset(ag->disp, 0, 0, 0);

				dtReal_t w = 0;

				for (int j = 0; j < ag->nneis; ++j)
				{
					const ExtendDetourCrowdAgent* nei = &m_agents[ag->neis[j].idx];
					const int idx1 = getAgentIndex(nei);

					dtReal_t diff[3];
					dtVsub(diff, ag->npos, nei->npos);
					diff[1] = 0;

					dtReal_t dist = dtVlenSqr(diff);
					if (dist > dtSqr(ag->params.radius + nei->params.radius))
						continue;
					dist = std::sqrt(dist);
					dtReal_t pen = (ag->params.radius + nei->params.radius) - dist;
					if (dist < 0.0001f)
					{
						// Agents on top of each other, try to choose diverging separation directions.
						if (idx0 > idx1)
							dtVset(diff, -ag->dvel[2], 0, ag->dvel[0]);
						else
							dtVset(diff, ag->dvel[2], 0, -ag->dvel[0]);
						pen = 0.01f;
					}
					else
					{
						pen = (1.0f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
					}

					dtVmad(ag->disp, ag->disp, diff, pen);

					w += 1.0f;
				}

				if (w > 0.0001f)
				{
					const dtReal_t iw = 1.0f / w;
					dtVscale(ag->disp, ag->disp, iw);
				}
			}

			for (int i = 0; i < nagents; ++i)
			{
				ExtendDetourCrowdAgent* ag = agents[i];
				if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING)
					continue;

				dtVadd(ag->npos, ag->npos, ag->disp);
			}
		}

	}
	void ExtendDetourCrowd::checkArrived(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];

			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING)
			{
				continue;
			}
			if (ag->target_agent < 0)
			{
				if (ag->targetState != ExtendDetourMoveRequestState::TARGET_VALID)
				{
					continue;
				}
				auto cur_dis = dtVdist(ag->npos, ag->targetPos);
				bool close_tag = false;
				if (ag->target_radius != 0)
				{
					close_tag = cur_dis <= ag->target_radius;
				}
				else
				{
					close_tag = cur_dis <= ag->params.radius * 0.5;
				}
				if (close_tag)
				{
					ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;
					dtVset(ag->vel, 0, 0, 0);

				}
			}
			else
			{
				if (ag->targetState != ExtendDetourMoveRequestState::TARGET_VALID && ag->targetState != ExtendDetourMoveRequestState::TARGET_VELOCITY)
				{
					continue;
				}
				auto target_ag = getAgent(ag->target_agent);
				if (!target_ag)
				{
					ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;
					dtVset(ag->vel, 0, 0, 0);

					continue;
				}
				if (ag->move_state == ExtendDetourCrowdAgentMoveState::STATE_WAITING)
				{
					continue;
				}
				auto cur_dis = dtVdist(ag->npos, target_ag->npos);
				bool close_tag = false;
				if (ag->target_radius != 0)
				{
					close_tag = cur_dis <= ag->target_radius;
				}
				else
				{
					close_tag = cur_dis <= (ag->params.radius + target_ag->params.radius) * 0.5;
				}
				if (close_tag)
				{
					if (ag->continue_follow)
					{
						ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_WAITING;
						dtVset(ag->vel, 0, 0, 0);
					}
					else
					{
						ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;
						dtVset(ag->vel, 0, 0, 0);

					}
				}

			}
		}
	}

	void ExtendDetourCrowd::checkFollowStopWaiting(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];
			if (ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_WAITING)
			{
				continue;
			}
			auto target_ag = getAgent(ag->target_agent);
			if (!target_ag || !target_ag->active)
			{
				ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;

				ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;
				ag->state = ExtendDetourCrowdAgentState::STATE_INVALID;
				continue;
			}

			auto cur_dis = dtVdist(ag->npos, target_ag->npos);
			auto temp_dis = ag->params.radius + ag->target_radius + target_ag->params.radius;

			if (cur_dis < temp_dis)
			{
				continue;
			}
			updateAgentFollowTarget(ag);
			ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_MOVING;

		}
	}
	void ExtendDetourCrowd::checkFollowSteer(ExtendDetourCrowdAgent** agents, const int nagents)
	{
		for (int i = 0; i < nagents; ++i)
		{
			ExtendDetourCrowdAgent* ag = agents[i];
			if (ag->targetState != ExtendDetourMoveRequestState::TARGET_VALID && ag->targetState != ExtendDetourMoveRequestState::TARGET_VELOCITY)
			{
				continue;
			}
			if (ag->state != ExtendDetourCrowdAgentState::STATE_WALKING)
			{
				continue;
			}
			if (ag->move_state != ExtendDetourCrowdAgentMoveState::STATE_MOVING)
			{
				continue;
			}
			if (ag->target_agent == -1)
			{
				continue;
			}
			auto target_ag = getAgent(ag->target_agent);
			if (!target_ag || !target_ag->active)
			{
				ag->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;
				dtVset(ag->dvel, 0, 0, 0);
				ag->targetState = ExtendDetourMoveRequestState::TARGET_FAILED;
				continue;
			}
			auto dist_threhold = 2 * (ag->params.radius + ag->target_radius + target_ag->params.radius);
			auto cur_dis = dtVdist(ag->npos, target_ag->npos);
			dtReal_t offset[3];
			dtVsub(offset, target_ag->npos, ag->npos);
			dtVnormalize(offset);
			dtVscale(offset, offset, ag->params.maxSpeed);

			if (ag->targetState == ExtendDetourMoveRequestState::TARGET_VALID)
			{
				if (cur_dis < dist_threhold)
				{

					requestMoveVelocity(getAgentIndex(ag), offset);
					continue;
				}
			}
			else
			{
				if (cur_dis > 2 * dist_threhold)
				{
					updateAgentFollowTarget(ag);
					ag->targetState = ExtendDetourMoveRequestState::TARGET_VALID;
				}
				else
				{
					// continue to use velocity mode
					dtVcopy(ag->targetPos, offset);
					continue;
				}
			}
			auto now_to_cor_dis = dtVdist(ag->npos, ag->corridor.getTarget());
			auto cor_to_target_dis = dtVdist(ag->corridor.getTarget(), target_ag->npos);

			if (cor_to_target_dis < dist_threhold && now_to_cor_dis < 10 * cor_to_target_dis)
			{
				updateAgentFollowTarget(ag);
				ag->corridor.moveTargetPosition(target_ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);

				continue;
			}

			if (cor_to_target_dis + now_to_cor_dis > 1.5 * cur_dis)
			{
				updateAgentFollowTarget(ag);
				requestMoveTargetReplan(getAgentIndex(ag), ag->targetRef, ag->targetPos);

				continue;
			}
		}
	}



	void ExtendDetourCrowd::update(const dtReal_t dt, ExtendDetourCrowdAgentDebugInfo* debug)
	{
		m_velocitySampleCount = 0;

		// const int debugIdx = debug ? debug->idx : -1;

		ExtendDetourCrowdAgent** agents = m_activeAgents;
		int nagents = getActiveAgents(agents, m_maxAgents);

		checkFollowStopWaiting(agents, nagents);

		checkFollowSteer(agents, nagents);

		// Check that all agents still have valid paths.
		checkPathValidity(agents, nagents, dt);

		// Update async move request and path finder.
		updateMoveRequest(dt);

		// Optimize path topology.
		updateTopologyOptimization(agents, nagents, dt);

		//update proximity

		updateProximity(agents, nagents);

		// Find next corner to steer to.
		findCornerSteerTo(agents, nagents, debug);

		// Trigger off-mesh connections (depends on corners).
		trigOffmeshconnection(agents, nagents);

		// Calculate steering.
		calculateSteering(agents, nagents);

		// Velocity planning.	
		velocityPlan(agents, nagents);

		// Integrate.
		intergateVelocity(agents, nagents, dt);

		handleCollision(agents, nagents);
		moveCorridorPos(agents, nagents);


		// Update agents using off-mesh connection.
		updateOffmeshConnection(agents, nagents, debug, dt);

		checkArrived(agents, nagents);

	}

	int  ExtendDetourCrowd::fetchAndClearArrived(int* result, const int max_result)
	{
		ExtendDetourCrowdAgent** agents = m_activeAgents;
		int nagents = getActiveAgents(agents, m_maxAgents);
		int count = 0;
		for (int i = 0; i < nagents; i++)
		{
			if (agents[i]->move_state == ExtendDetourCrowdAgentMoveState::STATE_MOVING)
			{
				if (agents[i]->targetState == ExtendDetourMoveRequestState::TARGET_FAILED)
				{
					agents[i]->move_state = ExtendDetourCrowdAgentMoveState::STATE_FINISHED;
				}
			}
			if (agents[i]->move_state == ExtendDetourCrowdAgentMoveState::STATE_FINISHED)
			{
				auto cur_agent_idx = getAgentIndex(agents[i]);
				result[count] = cur_agent_idx;
				count++;
				resetMoveTarget(cur_agent_idx);
				if (count >= max_result)
				{
					break;
				}
			}
		}
		return count;
	}

}