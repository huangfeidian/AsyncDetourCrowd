#pragma once
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
#include "DetourPathQueue.h"
#include "DetourCrowd.h"
#include <unordered_set>
#include <cstdint>
namespace spiritsaway::system::navigation
{



	/// The type of navigation mesh polygon the agent is currently traversing.
	/// @ingroup crowd
	enum class ExtendDetourCrowdAgentState
	{
		STATE_INVALID,		///< The agent is not in a valid state.
		STATE_WALKING,		///< The agent is traversing a normal navigation mesh polygon.
		STATE_OFFMESH,		///< The agent is traversing an off-mesh connection.
	};

	enum class ExtendDetourCrowdAgentMoveState
	{
		STATE_IDLE,
		STATE_MOVING,
		STATE_WAITING,
		STATE_FINISHED,
	};

	/// Configuration parameters for a crowd agent.
	/// @ingroup crowd
	struct ExtendDetourCrowdAgentParams
	{
		dtReal_t radius;						///< Agent radius. [Limit: >= 0]
		dtReal_t height;						///< Agent height. [Limit: > 0]
		dtReal_t maxAcceleration;				///< Maximum allowed acceleration. [Limit: >= 0]
		dtReal_t maxSpeed;						///< Maximum allowed speed. [Limit: >= 0]

		// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
		dtReal_t collisionQueryRange;

		dtReal_t pathOptimizationRange;		///< The path visibility optimization range. [Limit: > 0]

		/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
		dtReal_t separationWeight;

		/// Flags that impact steering behavior. (See: #UpdateFlags)
		unsigned char updateFlags;

		/// The index of the avoidance configuration to use for the agent. 
		/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
		unsigned char obstacleAvoidanceType;

		/// The index of the query filter used by this agent.
		unsigned char queryFilterType;

		/// User defined data attached to the agent.
		void* userData;
	};

	enum class ExtendDetourMoveRequestState
	{
		TARGET_NONE = 0,
		TARGET_FAILED,
		TARGET_VALID,// in moving
		TARGET_REQUESTING,
		TARGET_WAITING_FOR_QUEUE,
		TARGET_WAITING_FOR_PATH,
		TARGET_VELOCITY, // move controled by speed  not target
	};

	/// Represents an agent managed by a #ExtendDetourCrowd object.
	/// @ingroup crowd
	struct ExtendDetourCrowdAgent
	{
		/// True if the agent is active, false if the agent is in an unused slot in the agent pool.
		bool active;

		/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
		ExtendDetourCrowdAgentState state;

		ExtendDetourCrowdAgentMoveState move_state = ExtendDetourCrowdAgentMoveState::STATE_IDLE;

		/// True if the agent has valid path (targetState == TARGET_VALID) and the path does not lead to the requested position, else false.
		bool partial;

		/// The path corridor the agent is using.
		dtPathCorridor corridor;

		/// The local boundary data for the agent.
		dtLocalBoundary boundary;

		/// Time since the agent's path corridor was optimized.
		dtReal_t topologyOptTime;

		/// The known neighbors of the agent.
		dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];

		/// The number of neighbors.
		int nneis;

		/// The desired speed.
		dtReal_t desiredSpeed;

		dtReal_t npos[3];		///< The current agent position. [(x, y, z)]
		dtReal_t disp[3];		///< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
		dtReal_t dvel[3];		///< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
		dtReal_t nvel[3];		///< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
		dtReal_t vel[3];		///< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

		/// The agent's configuration parameters.
		ExtendDetourCrowdAgentParams params;

		/// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
		dtReal_t cornerVerts[DT_CROWDAGENT_MAX_CORNERS * 3];

		/// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
		unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

		/// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
		dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

		/// The number of corners.
		int ncorners;

		ExtendDetourMoveRequestState targetState;			///< State of the movement request.
		dtPolyRef targetRef;				///< Target polyref of the movement request.
		dtReal_t targetPos[3];					///< Target position of the movement request (or velocity in case of TARGET_VELOCITY).
		dtPathQueueRef targetPathqRef;		///< Path finder ref.
		dtReal_t target_radius;				///< radius considered near enough to stop walk
		int32_t target_agent = -1;			///< the agent target idx 
		bool continue_follow = false;		///< shoud continue to follow agent target
		bool targetReplan;					///< Flag indicating that the current path is being replanned.
		dtReal_t targetReplanTime;				/// <Time since the agent's target was replanned.
		std::uint32_t follow_by_agent_count = 0; ///< the agents countfollowing current agent
	};

	struct ExtendDetourCrowdAgentAnimation
	{
		bool active;
		dtReal_t initPos[3], startPos[3], endPos[3];
		dtPolyRef polyRef;
		dtReal_t t, tmax;
	};

	/// Crowd agent update flags.
	/// @ingroup crowd
	/// @see ExtendDetourCrowdAgentParams::updateFlags
	enum class ExtendDetourCrowdUpdateFlags
	{
		ANTICIPATE_TURNS = 1,
		OBSTACLE_AVOIDANCE = 2,
		SEPARATION = 4,
		OPTIMIZE_VIS = 8,			///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
		OPTIMIZE_TOPO = 16,		///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
	};

	struct ExtendDetourCrowdAgentDebugInfo
	{
		int idx;
		dtReal_t optStart[3], optEnd[3];
	};

	/// Provides local steering behaviors for a group of agents. 
	/// @ingroup crowd
	class ExtendDetourCrowd
	{
		int m_maxAgents;
		ExtendDetourCrowdAgent* m_agents;
		ExtendDetourCrowdAgent** m_activeAgents;
		ExtendDetourCrowdAgentAnimation* m_agentAnims;

		dtPathQueue m_pathq;

		dtObstacleAvoidanceParams m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
		dtObstacleAvoidanceQuery* m_obstacleQuery;

		dtProximityGrid* m_grid;

		dtPolyRef* m_pathResult;
		int m_maxPathResult;

		dtReal_t m_agentPlacementHalfExtents[3];

		dtQueryFilter m_filters[DT_CROWD_MAX_QUERY_FILTER_TYPE];

		dtReal_t m_maxAgentRadius;

		int m_velocitySampleCount;

		dtNavMeshQuery* m_navquery;

		void updateTopologyOptimization(ExtendDetourCrowdAgent** agents, const int nagents, const dtReal_t dt);
		void updateMoveRequest(const dtReal_t dt);
		void checkPathValidity(ExtendDetourCrowdAgent** agents, const int nagents, const dtReal_t dt);
		void updateProximity(ExtendDetourCrowdAgent** agents, const int nagents);
		void findCornerSteerTo(ExtendDetourCrowdAgent** agents, const int nagents, ExtendDetourCrowdAgentDebugInfo* debug);
		void trigOffmeshconnection(ExtendDetourCrowdAgent** agents, const int nagents);
		void calculateSteering(ExtendDetourCrowdAgent** agents, const int nagents);
		void updateOffmeshConnection(ExtendDetourCrowdAgent** agents, const int nagents, ExtendDetourCrowdAgentDebugInfo* debug, const dtReal_t dt);
		void moveCorridorPos(ExtendDetourCrowdAgent** agents, const int nagents);
		void velocityPlan(ExtendDetourCrowdAgent** agents, const int nagents);
		void intergateVelocity(ExtendDetourCrowdAgent** agents, const int nagents, const dtReal_t dt);
		void handleCollision(ExtendDetourCrowdAgent** agents, const int nagents);
		void checkArrived(ExtendDetourCrowdAgent** agents, const int nagents);
		void checkFollowStopWaiting(ExtendDetourCrowdAgent** agents, const int nagents);
		void checkFollowSteer(ExtendDetourCrowdAgent** agents, const int nagents);
		inline int getAgentIndex(const ExtendDetourCrowdAgent* agent) const
		{
			return (int)(agent - m_agents);
		}

		bool requestMoveTargetReplan(const int idx, dtPolyRef ref, const dtReal_t* pos);
		void updateAgentFollowTarget(ExtendDetourCrowdAgent* ag);
		void purge();
		void addAgentImpl(const int idx, const dtReal_t* pos, const ExtendDetourCrowdAgentParams* params);
	public:
		ExtendDetourCrowd();
		~ExtendDetourCrowd();

		/// Initializes the crowd.  
		///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
		///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
		///  @param[in]		nav				The navigation mesh to use for planning.
		/// @return True if the initialization succeeded.
		std::uint32_t init(const int maxAgents, const dtReal_t maxAgentRadius, const dtNavMesh* nav);

		/// Gets the specified agent from the pool.
		///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return The requested agent.
		const ExtendDetourCrowdAgent* getAgent(const int idx);

		/// Gets the specified agent from the pool.
		///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return The requested agent.
		ExtendDetourCrowdAgent* getEditableAgent(const int idx);

		/// The maximum number of agents that can be managed by the object.
		/// @return The maximum number of agents.
		int getAgentCount() const;

		/// Adds a new agent to the crowd.
		///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
		///  @param[in]		params	The configutation of the agent.
		/// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
		int addAgent(const dtReal_t* pos, const ExtendDetourCrowdAgentParams* params);

		bool addAgent(int idx, const dtReal_t* pos, const ExtendDetourCrowdAgentParams* params);
		/// Updates the specified agent's configuration.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		params	The new agent configuration.
		void updateAgentParameters(const int idx, const ExtendDetourCrowdAgentParams* params);

		/// Updates the specified agent's configuration.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		pos		The new agent position.
		void updateAgentPos(const int idx, const dtReal_t* pos);

		/// Updates the specified agent's configuration.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		ref		The new agent poly.
		///  @param[in]		pos		The new agent position.
		void updateAgentPolyPos(const int idx, dtPolyRef ref, const dtReal_t* pos);

		/// Updates the specified agent's pos along current surface 
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		dis		The length along surface.
		void moveAlongSurface(const int idx, const dtReal_t* dis);

		/// Removes the agent from the crowd.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		void removeAgent(const int idx);

		/// Submits a new move request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		ref		The position's polygon reference.
		///  @param[in]		pos		The position within the polygon. [(x, y, z)]
		///  @param[in]		target_radius		distance to pos less than raget_radius considered complete
		/// @return True if the request was successfully submitted.
		bool requestMoveTarget(const int idx, dtPolyRef ref, const dtReal_t* pos, const dtReal_t target_radius = 0);

		/// Submits a new move request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		ref		The position's polygon reference.
		///  @param[in]		pos		The position within the polygon. [(x, y, z)]
		///  @param[in]		target_radius		distance to pos less than raget_radius considered complete
		/// @return True if the request was successfully submitted.
		bool requestMoveTarget(const int idx, const int target_idx, const dtReal_t target_radius = 0, const bool follow = false);

		/// Submits a new move request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		vel		The movement velocity. [(x, y, z)]
		/// @return True if the request was successfully submitted.
		bool requestMoveVelocity(const int idx, const dtReal_t* vel);

		/// Resets any request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return True if the request was successfully reseted.
		bool resetMoveTarget(const int idx);

		/// Gets the active agents int the agent pool.
		///  @param[out]	agents		An array of agent pointers. [(#ExtendDetourCrowdAgent *) * maxAgents]
		///  @param[in]		maxAgents	The size of the crowd agent array.
		/// @return The number of agents returned in @p agents.
		int getActiveAgents(ExtendDetourCrowdAgent** agents, const int maxAgents);

		/// Updates the steering and positions of all agents.
		///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
		///  @param[out]	debug	A debug object to load with debug information. [Opt]
		void update(const dtReal_t dt, ExtendDetourCrowdAgentDebugInfo* debug);

		/// get all agents that navigation is completed
		/// @return the agent ids that meet the complete standard
		int fetchAndClearArrived(int* result, const int max_result);

		/// Gets the filter used by the crowd.
		/// @return The filter used by the crowd.
		inline const dtQueryFilter* getFilter(const int i) const
		{
			return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0;
		}

		/// Gets the filter used by the crowd.
		/// @return The filter used by the crowd.
		inline dtQueryFilter* getEditableFilter(const int i)
		{
			return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0;
		}

		/// Gets the search halfExtents [(x, y, z)] used by the crowd for query operations. 
		/// @return The search halfExtents used by the crowd. [(x, y, z)]
		const dtReal_t* getQueryHalfExtents() const
		{
			return m_agentPlacementHalfExtents;
		}

		/// Same as getQueryHalfExtents. Left to maintain backwards compatibility.
		/// @return The search halfExtents used by the crowd. [(x, y, z)]
		const dtReal_t* getQueryExtents() const
		{
			return m_agentPlacementHalfExtents;
		}

		/// Gets the velocity sample count.
		/// @return The velocity sample count.
		inline int getVelocitySampleCount() const
		{
			return m_velocitySampleCount;
		}

		/// Gets the crowd's path request queue.
		/// @return The crowd's path request queue.
		const dtPathQueue* getPathQueue() const
		{
			return &m_pathq;
		}

		/// Gets the query object used by the crowd.
		const dtNavMeshQuery* getNavMeshQuery() const
		{
			return m_navquery;
		}

	private:
		// Explicitly disabled copy constructor and copy assignment operator.
		ExtendDetourCrowd(const ExtendDetourCrowd&);
		ExtendDetourCrowd& operator=(const ExtendDetourCrowd&);
	};

	/// Allocates a crowd object using the Detour allocator.
	/// @return A crowd object that is ready for initialization, or null on failure.
	///  @ingroup crowd
	ExtendDetourCrowd* dtAllocExtendDetourCrowd();

	/// Frees the specified crowd object using the Detour allocator.
	///  @param[in]		ptr		A crowd object allocated using #dtAllocCrowd
	///  @ingroup crowd
	void dtFreeExtendDetourCrowd(ExtendDetourCrowd* ptr);
}