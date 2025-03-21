#ifndef CPFA_CONTROLLER_H
#define CPFA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/Base/Pheromone.h>
#include <source/CPFA/CPFA_loop_functions.h>
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
		void SetCongestion(bool value);
		int FindClosestPointOnPath(argos::CVector2 robotPos, std::vector<argos::CVector2> path);
		argos::CVector2 EntryPoint = {1.4, 0};
		std::vector<argos::CVector2> EntryPath = {
			{1.2, 0.0}, {0.94, 0.70}, {0.33, 1.10}, {-0.39, 1.06}, {-0.92, 0.60}, 
			{-1.08, -0.07}, {-0.80, -0.68}, {-0.23, -1.00}, {0.41, -0.92}, {0.85, -0.48}, 
			{0.95, 0.12}, {0.67, 0.65}, {0.14, 0.89}, {-0.41, 0.78}, {-0.77, 0.37}, 
			{-0.82, -0.16}, {-0.54, -0.60}, {-0.08, -0.78}, {0.39, -0.65}, {0.68, -0.28}, 
			{0.69, 0.18}, {0.43, 0.54}, {0.02, 0.66}, {-0.36, 0.52}, {-0.58, 0.19}, 
			{-0.56, -0.19}, {-0.32, -0.46}, {0.02, -0.54}, {0.32, -0.40}, {0.47, -0.12}, 
			{0.43, 0.17}, {0.23, 0.38}, {-0.04, 0.41}, {-0.26, 0.29}, {-0.36, 0.07}, 
			{-0.31, -0.15}, {-0.15, -0.28}, {0.05, -0.29}, {0.19, -0.19}, {0.24, -0.03}, 
			{0.19, 0.11}, {0.08, 0.18}, {-0.04, 0.17}, {-0.11, 0.10}, {-0.12, 0.01}, 
			{-0.08, -0.05}, {-0.03, -0.07}, {0.01, -0.05}, {0.02, -0.01}, {0.0, 0.0}
		};
		std::vector<argos::CVector2> ExitPath = {
			{0.0, 0.0}, {-0.02, 0.01}, {-0.01, 0.05}, {0.03, 0.07}, {0.08, 0.05},
			{0.12, -0.01}, {0.11, -0.10}, {0.04, -0.17}, {-0.08, -0.18}, {-0.19, -0.11},
			{-0.24, 0.03}, {-0.19, 0.19}, {-0.05, 0.29}, {0.15, 0.28}, {0.31, 0.15},
			{0.36, -0.07}, {0.26, -0.29}, {0.04, -0.41}, {-0.23, -0.38}, {-0.43, -0.17},
			{-0.47, 0.12}, {-0.32, 0.40}, {-0.02, 0.54}, {0.32, 0.46}, {0.56, 0.19},
			{0.58, -0.19}, {0.36, -0.52}, {-0.02, -0.66}, {-0.43, -0.54}, {-0.69, -0.18},
			{-0.68, 0.28}, {-0.39, 0.65}, {0.08, 0.78}, {0.54, 0.60}, {0.82, 0.16},
			{0.77, -0.37}, {0.41, -0.78}, {-0.14, -0.89}, {-0.67, -0.65}, {-0.95, -0.12},
			{-0.85, 0.48}, {-0.41, 0.92}, {0.23, 1.00}, {0.80, 0.68}, {1.08, 0.07},
			{0.92, -0.60}, {0.39, -1.06}, {-0.33, -1.10}, {-0.94, -0.70}, {-1.2, 0.0}
		};
		const argos::Real RestrictedZoneRadius = 1.3; // adjust this based on the path radius
		bool IsInRestrictedZone(argos::CVector2 position);			
		void setZoneActive(bool value);
		bool isZoneActive = false;

		int currentWaypointIndex;
		enum CPFA_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3,
			FOLLOWING_ENTRY_PATH = 4,
			FOLLOWING_EXIT_PATH = 5
		} CPFA_state;		

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
		void FollowingEntryPath();
		void FollowingExitPath();

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

		const size_t WINDOW_SIZE = 150;
		const size_t STEP_SIZE = 50;	
		std::vector<argos::CVector2> returning_trajectory;	
		bool IsInCongestion();

		CVector2 previous_position;

		string results_path;
		string results_full_path;
		bool isUsingPheromone;
		int reroute_attempts = 0;
		bool isCongested = false;
		std::unordered_map<std::string, int> dropCooldownMap; // Track when each robot last dropped a resource
		const int DROP_COOLDOWN = 75; // Time before a robot can re-collect its own drop
		// std::unordered_map<argos::CVector2, int, Vector2Hash, Vector2Equal> foodTargetCount; // Track how many robots are targeting each resource
		const int MAX_ROBOTS_PER_RESOURCE = 1; // Max robots that can target the same resource
		int resources_dapu = 0;
		unsigned int survey_count;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;

		double optimal_distance = 0.08 * 150;
		double previous_ratio_distance = -1;  
		double previous_ratio_distance_lag_1 = -1;  
	
		double previous_angle = -1;  
		double previous_angle_lag_1 = -1;  
};

#endif /* CPFA_CONTROLLER_H */
