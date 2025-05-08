#ifndef CPFA_CONTROLLER_H
#define CPFA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/Base/Pheromone.h>
// #include <source/CPFA/CPFA_loop_functions.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
//#include <cmath>

#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/datatypes/color.h>

using namespace std;
using namespace argos;

static unsigned int num_targets_collected = 0;

class CPFA_loop_functions;
#include <functional> // Required for std::hash
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

class CPFA_controller : public BaseController {

	public:

		CPFA_controller();
		void DrawEntryPath();
		// logic for spiral path
		// bool CollisionDetection() override;

		// CCI_Controller inheritence functions
		void Init(argos::TConfigurationNode &node);
		void ControlStep();
		void Reset();		
		bool IsHoldingFood();
		bool IsUsingSiteFidelity();
		bool IsInTheNest();
		argos::Real getSimTimeInSeconds();

		Real FoodDistanceTolerance;

		void SetLoopFunctions(CPFA_loop_functions* lf);
  
  size_t     GetSearchingTime();//qilu 09/26/2016
  size_t      GetTravelingTime();//qilu 09/26/2016
  string      GetStatus();//qilu 09/26/2016
  size_t      startTime;//qilu 09/26/2016
  void 		 setStatus(string status);

  Real curr_time_in_seconds; 
    Real last_time_in_seconds; 
        
		bool CollisionDetection() override;
		// # of collisions in returning state
		int collisions_in_returning = 0;

		enum CPFA_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3
		} CPFA_state;		
		bool hasCachedRLState = false;
		struct ActorState {
			float distance_to_nest;  
			int timesteps_returning;
			int collisions;
			float path_efficiency;      // optimal/actual
			float angular_deviation;    // angle between optimal and current direction
			int reached_nest; // 1 if reached nest, 0 otherwise
		};
		ActorState cachedRLState;
		// std::vector<float> robotActions;
		int actionRepeatCounter = 0;           // how many steps to keep current action
		std::deque<int> collision_history; // history of collisions
		int window_size = 200; // size of the history window for collisions
		float recent_collision_sum = 0; // sum of recent collisions
		bool moving_to_target = false;
		bool reached_nest = false;
		bool IsNearWall(Real threshold);
		bool reached_nest_during_action = false;
		size_t rl_tick_counter = 0;

	private:
  string 			controllerID;//qilu 07/26/2016
		CCI_DifferentialSteeringActuator* m_pcWheels; //defining wheels
		CPFA_loop_functions* LoopFunctions;
		argos::CRandom::CRNG* RNG;

		/* pheromone trail variables */
		std::vector<argos::CVector2> TrailToShare;
		std::vector<argos::CVector2> TrailToFollow;
		std::vector<argos::CRay3>    MyTrail;

		/* robot position variables */
		argos::CVector2 SiteFidelityPosition;
  bool			 updateFidelity; //qilu 09/07/2016
  
		vector<CRay3> myTrail;
		CColor        TrailColor;

		bool isInformed;
		bool isHoldingFood;
		bool isUsingSiteFidelity;
		bool isGivingUpSearch;
  
		size_t ResourceDensity;
		size_t RobotDensity; //qilu 06/2023
		size_t MaxTrailSize;
		size_t SearchTime;//for informed search
  
  size_t           searchingTime; //qilu 09/26
  size_t           travelingTime;//qilu 09/26
        
  
		/* iAnt CPFA state variable */
		// enum CPFA_state {
		// 	DEPARTING = 0,
		// 	SEARCHING = 1,
		// 	RETURNING = 2,
		// 	SURVEYING = 3,
		// 	FOLLOWING_ENTRY_PATH = 4,
		// 	FOLLOWING_EXIT_PATH = 5
		// } CPFA_state;

		/* iAnt CPFA state functions */
		void CPFA();
		void Departing();
		void Searching();
		void Returning();
		void Surveying();

		/* CPFA helper functions */
		void SetRandomSearchLocation();
		void SetHoldingFood();
		void SetLocalResourceDensity();
		void SetRobotDensity(); //qilu 06/2023
		
		void SetFidelityList(argos::CVector2 newFidelity);
		void SetFidelityList();
		bool SetTargetPheromone();

		argos::Real GetExponentialDecay(argos::Real value, argos::Real time, argos::Real lambda);
		argos::Real GetBound(argos::Real value, argos::Real min, argos::Real max);
		argos::Real GetPoissonCDF(argos::Real k, argos::Real lambda);

		void UpdateTargetRayList();

   		// RL state tracking
		bool first_time_returning = true;
		argos::CVector2 resource_pickup_position;
		argos::CVector2 last_position;
		float optimal_distance_to_nest = 0.0f;
		int timesteps_returning = 0;
		float total_returning_path_length = 0.0f;
		float path_efficiency = 1.0f;
		
		// Function to update RL state
		void UpdateRLState(const ActorState& state);

\
		std::vector<argos::CVector2> returning_trajectory;	



		CVector2 previous_position;

		string results_path;
		string results_full_path;
		bool isUsingPheromone;
		int reroute_attempts = 0;
		bool isCongested = false;
		unsigned int survey_count;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs; 
};

#endif /* CPFA_CONTROLLER_H */
