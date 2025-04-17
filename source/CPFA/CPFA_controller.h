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
#include <deque>

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
		//cirular path
        // std::vector<argos::CVector2> EntryPath = {
        //     {1.20, 0.00}, {0.99, 0.60}, {0.52, 0.99}, {-0.06, 1.07}, {-0.58, 0.86}, 
        //     {-0.90, 0.42}, {-0.95, -0.10}, {-0.72, -0.55}, {-0.32, -0.81}, {0.13, -0.82}, 
        //     {0.51, -0.60}, {0.71, -0.24}, {0.69, 0.15}, {0.48, 0.46}, {0.17, 0.60}, 
        //     {-0.15, 0.56}, {-0.39, 0.37}, {-0.48, 0.11}, {-0.43, -0.15}, {-0.27, -0.32}, 
        //     {-0.06, -0.37}, {0.12, -0.31}, {0.23, -0.18}, {0.25, -0.03}, {0.19, 0.09}, 
        //     {0.09, 0.14}, {0.01, 0.12}, {-0.04, 0.07}, {-0.04, 0.02}, {0.00, 0.00}
        // };
		//square path
		std::vector<argos::CVector2> EntryPath = {
			{1.13, -0.90},
			{-0.90, -0.90},
			{-0.90, 0.67},
			{0.67, 0.67},
			{0.67, -0.45},
			{-0.45, -0.45},
			{-0.45, 0.23},
			{0.23, 0.23},
			{0.23, 0.00},
			{0.00, 0.00}
		};
        // std::vector<argos::CVector2> ExitPath = {
        //     {0.00, -0.00}, {0.04, -0.02}, {0.04, -0.07}, {-0.01, -0.12}, {-0.09, -0.14},
        //     {-0.19, -0.09}, {-0.25, 0.03}, {-0.23, 0.18}, {-0.12, 0.31}, {0.06, 0.37},
        //     {0.27, 0.32}, {0.43, 0.15}, {0.48, -0.11}, {0.39, -0.37}, {0.15, -0.56},
        //     {-0.17, -0.60}, {-0.48, -0.46}, {-0.69, -0.15}, {-0.71, 0.24}, {-0.51, 0.60},
        //     {-0.13, 0.82}, {0.32, 0.81}, {0.72, 0.55}, {0.95, 0.10}, {0.90, -0.42},
        //     {0.58, -0.86}, {0.06, -1.07}, {-0.52, -0.99}, {-0.99, -0.60}, {-1.20, 0.00}
        // };
		argos::CVector2 mainTarget;
		int exitPathIndex;
		const argos::Real RestrictedZoneRadius = 1.3; // adjust this based on the path radius
		bool IsInRestrictedZone(argos::CVector2 position);			
		void setZoneActive(bool value);
		bool isZoneActive = false;
		float distance_traveled = 0.0;
		float tortuosity;
		CVector2 previous_location;
		int currentWaypointIndex;
		bool goingtoentry = false;
		
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
		std::deque<argos::CVector2> returning_trajectory;	
		bool IsInCongestion();
		float ema_distance = -1.0f;

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
