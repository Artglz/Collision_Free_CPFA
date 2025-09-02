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
		CVector2 FindClosestEntryPoint();
		int FindClosestPointIndexOnPath(const std::vector<argos::CVector2>& path);
		CVector2 FindClosestNest();
		bool inCentralZone();
		bool IsInsideCircleBoundary(const argos::CVector2& pos);
		argos::CVector2 SafeTargetFromHeading(argos::CRadians base_angle, argos::Real step_size);
		bool hasLoggedCentralZone = false;
		bool runningfromcorridor = false;
		// Circle center and radius (adjust values as needed)
		const argos::CVector2 m_cCircleCenter = argos::CVector2(0.0, 0.0);
		const argos::Real m_fCircleRadius = 2.0;
		
		const argos::Real EntryPointThreshold = 0.3; // Distance threshold for entry points
		const argos::Real ExitPointThreshold = 0.1;
			 
		std::vector<argos::CVector2> entryPath1 = {
			{ 1.60,  1.10 }, { 1.60,  0.825 }, { 1.60,  0.55 }, { 1.60,  0.275 },
			{ 1.60,  0.00 }, { 1.60, -0.275 }, { 1.60, -0.55 }, { 1.60, -0.825 },
			{ 1.60, -1.10 }, { 1.51, -1.10 }, { 1.42, -1.10 }, { 1.42, -0.84375 },
			{ 1.42, -0.5875 }, { 1.42, -0.33125 }, { 1.42, -0.075 }, { 1.42,  0.18125 },
			{ 1.42,  0.4375 }, { 1.42,  0.69375 }, { 1.42,  0.95 }, { 1.33,  0.95 },
			{ 1.24,  0.95 }, { 1.24,  0.7375 }, { 1.24,  0.525 }, { 1.24,  0.3125 },
			{ 1.24,  0.10 }, { 1.24, -0.1125 }, { 1.24, -0.325 }, { 1.24, -0.5375 },
			{ 1.24, -0.75 }, { 1.15, -0.75 }, { 1.06, -0.75 }, { 1.06, -0.57625 },
			{ 1.06, -0.4025 }, { 1.06, -0.22875 }, { 1.06, -0.055 }, { 1.06,  0.11875 },
			{ 1.06,  0.2925 }, { 1.06,  0.46625 }, { 1.06,  0.64 },
			{ 0.88,  0.64 }, {0.88, 0.32}, { 0.88,  0.095 }, { 0.88, -0.26 },{ 0.88, -0.45 }, 
			{ 0.72, -0.45 }, { 0.72, -0.26 }, { 0.72, -0.07 }, { 0.72,  0.12 },
			{ 0.72,  0.31 }, { 0.675, 0.31 }, { 0.63,  0.31 }, { 0.585, 0.31 },
			{ 0.54,  0.31 }, { 0.54,  0.2325 }, { 0.54,  0.155 }, { 0.54,  0.0775 },
			{ 0.54,  0.0 },
			{ 0.36,  0.0 }, 
			{ 0.30,  0.0 }
		};
		std::vector<argos::CVector2> entryPath2 = {
			{ -1.60, -1.10 }, { -1.60, -0.825 }, { -1.60, -0.55 }, { -1.60, -0.275 },
			{ -1.60,  0.00 }, { -1.60,  0.275 }, { -1.60,  0.55 }, { -1.60,  0.825 },
			{ -1.60,  1.10 }, { -1.51,  1.10 }, { -1.42,  1.10 }, { -1.42,  0.9625 },
			{ -1.42,  0.825 }, { -1.42,  0.6875 }, { -1.42,  0.55 }, { -1.42,  0.4125 },
			{ -1.42,  0.275 }, { -1.42,  0.1375 }, { -1.42,  0.00 }, { -1.42, -0.2375 },
			{ -1.42, -0.475 }, { -1.42, -0.7125 }, { -1.42, -0.95 }, { -1.33, -0.95 },
			{ -1.24, -0.95 }, { -1.24, -0.7375 }, { -1.24, -0.525 }, { -1.24, -0.3125 },
			{ -1.24, -0.10 }, { -1.24,  0.1125 }, { -1.24,  0.325 }, { -1.24,  0.5375 },
			{ -1.24,  0.75 }, { -1.15,  0.75 }, { -1.06,  0.75 }, { -1.06,  0.62 },
			{ -1.06,  0.49 }, { -1.06,  0.36 }, { -1.06,  0.23 }, { -1.06,  0.10 },
			{ -1.06, -0.02 }, { -1.06, -0.15 }, { -1.06, -0.28 }, { -1.06, -0.41 },
			{ -1.06, -0.53 }, { -1.06, -0.64 }, { -0.97, -0.64 }, { -0.88, -0.64 },
			{ -0.88, -0.4875 }, { -0.88, -0.335 }, { -0.88, -0.1825 }, { -0.88, -0.03 },
			{ -0.88,  0.1225 }, { -0.88,  0.275 }, { -0.88,  0.3975 }, { -0.88,  0.45 },
			{ -0.80,  0.45 }, { -0.72,  0.45 }, { -0.72,  0.31625 }, { -0.72,  0.1825 },
			{ -0.72,  0.04875 }, { -0.72, -0.085 }, { -0.72, -0.21875 }, { -0.72, -0.31 },
			{ -0.63, -0.31 }, { -0.54, -0.31 }, { -0.54, -0.20667 }, { -0.54, -0.10333 },
			{ -0.54,  0.0 },
			{ -0.45,  0.0 }, { -0.36,  0.0 }, { -0.33,  0.0 }, { -0.30,  0.0 }
		};
		std::vector<argos::CVector2> entryPath3 = {
			{ -1.10,  1.60 }, { -0.825, 1.60 }, { -0.55, 1.60 }, { -0.275, 1.60 },
			{  0.00,  1.60 }, {  0.275, 1.60 }, {  0.55, 1.60 }, {  0.825, 1.60 },
			{  1.10,  1.60 }, {  1.10,  1.51 }, {  1.10,  1.42 }, {  0.8125, 1.42 },
			{  0.525, 1.42 }, {  0.2375, 1.42 }, { -0.05, 1.42 }, { -0.3375, 1.42 },
			{ -0.625, 1.42 }, { -0.9125, 1.42 }, { -0.95, 1.42 }, { -0.95, 1.33 },
			{ -0.95, 1.24 }, { -0.7375, 1.24 }, { -0.525, 1.24 }, { -0.3125, 1.24 },
			{ -0.10, 1.24 }, {  0.1125, 1.24 }, {  0.325, 1.24 }, {  0.5375, 1.24 },
			{  0.75, 1.24 }, {  0.75, 1.15 }, {  0.75, 1.06 }, {  0.60375, 1.06 },
			{  0.4575, 1.06 }, {  0.31125, 1.06 }, {  0.165, 1.06 }, {  0.01875, 1.06 },
			{ -0.1275, 1.06 }, { -0.27375, 1.06 }, { -0.42, 1.06 }, { -0.564, 1.06 },
			{ -0.64, 1.06 }, { -0.64, 0.97 }, { -0.64, 0.88 }, { -0.485, 0.88 },
			{ -0.33, 0.88 }, { -0.175, 0.88 }, { -0.02, 0.88 }, {  0.135, 0.88 },
			{  0.29, 0.88 }, {  0.45, 0.88 }, {  0.45, 0.80 }, {  0.45, 0.72 },
			{  0.31625, 0.72 }, {  0.1825, 0.72 }, {  0.04875, 0.72 }, { -0.085, 0.72 },
			{ -0.21875, 0.72 }, { -0.31, 0.72 }, { -0.31, 0.63 }, { -0.31, 0.54 },
			{ -0.20667, 0.54 }, { -0.10333, 0.54 }, {  0.0, 0.54 },
			{  0.0, 0.45 }, {  0.0, 0.36 }, {  0.0, 0.33 }, {  0.0, 0.30 }
		};
		std::vector<argos::CVector2> entryPath4 = {
			{  1.10, -1.60 }, {  0.825, -1.60 }, {  0.55, -1.60 }, {  0.275, -1.60 },
			{  0.00, -1.60 }, { -0.275, -1.60 }, { -0.55, -1.60 }, { -0.825, -1.60 },
			{ -1.10, -1.60 }, { -1.10, -1.51 }, { -1.10, -1.42 }, { -0.84375, -1.42 },
			{ -0.5875, -1.42 }, { -0.33125, -1.42 }, { -0.075, -1.42 }, {  0.18125, -1.42 },
			{  0.4375, -1.42 }, {  0.69375, -1.42 }, {  0.95, -1.42 }, {  0.95, -1.33 },
			{  0.95, -1.24 }, {  0.7375, -1.24 }, {  0.525, -1.24 }, {  0.3125, -1.24 },
			{  0.10, -1.24 }, { -0.1125, -1.24 }, { -0.325, -1.24 }, { -0.5375, -1.24 },
			{ -0.75, -1.24 }, { -0.75, -1.15 }, { -0.75, -1.06 }, { -0.57625, -1.06 },
			{ -0.4025, -1.06 }, { -0.22875, -1.06 }, { -0.055, -1.06 }, {  0.11875, -1.06 },
			{  0.2925, -1.06 }, {  0.46625, -1.06 }, {  0.64, -1.06 },
			{  0.64, -0.88 }, {  0.32, -0.88 }, {  0.095, -0.88 },  { -0.26, -0.88 },{ -0.45, -0.88 },
			{ -0.45, -0.72 }, { -0.26, -0.72 }, { -0.07, -0.72 }, {  0.12, -0.72 },
			{  0.31, -0.72 }, {  0.31, -0.675 }, {  0.31, -0.63 }, {  0.31, -0.585 },
			{  0.31, -0.54 }, {  0.2325, -0.54 }, {  0.155, -0.54 }, {  0.0775, -0.54 },
			{  0.0,   -0.54 },
			{  0.0,   -0.36 }, 
			{  0.0,   -0.30 }
		};
										
		std::vector<argos::CVector2> entryPoints = {
			entryPath1.front(),
			entryPath2.front(),
			entryPath3.front(),
			entryPath4.front()
		};				
								
										
		bool isLeft;
		int pointonpath;			
		bool goingtoexit = false;

		std::vector<argos::CVector2> exitPath1 = {{0.2, 0.2}, {1.6, 1.6}};
		std::vector<argos::CVector2> exitPath2 = {{-0.2, -0.2}, {-1.6, -1.6}};
		std::vector<argos::CVector2> exitPath3 = {{-0.2, 0.2}, {-1.6, 1.6}};
		std::vector<argos::CVector2> exitPath4 = {{0.2, -0.2}, {1.6, -1.6}};
		std::vector<argos::CVector2> exitPoints = {{1.6, 1.6}, {-1.6, -1.6}, {-1.6, 1.6}, {1.6, -1.6}}; // exit point of the paths

		std::vector<argos::CVector2> actualExitPath;
		bool followingEntryPath1 = false;
		bool followingEntryPath2 = false;
		bool followingEntryPath3 = false;
		bool followingEntryPath4 = false;
		argos::CRange<argos::Real> GoStraightAngleRangeInDegreesInRegion;

		size_t stopCounter = 0; // Counter to track how many timesteps the robot has been stopped
		size_t stopcooldownCounter = 0;
		size_t notAtTargetCounter = 0;
		size_t nestStopCounter = 0;
		size_t inCircleCounter = 0;
		size_t exitPointCounter = 0;
		size_t secondExitPointCounter = 0;
		bool hasntReachedFirstExitPoint = false;
		bool firstTimeInNest = false;
		CVector2 escapeTarget;
		std::vector<argos::CVector2> actualPath;	
		CVector2 entrypoint;
		int currentWaypointIndex = 2;
		argos::CVector2 mainTarget; // this is to make the robot take the exit path but still save its initial target(site fidelity or random search)
		bool goingtoentry = false;
		
		enum CPFA_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3,
			FOLLOWING_ENTRY_PATH = 4,
			FOLLOWING_EXIT_PATH = 5
		} CPFA_state;	

		Real GetTotalTimeInsideRedCircle();	

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

		CVector2 previous_position;

		string results_path;
		string results_full_path;
		bool isUsingPheromone;

		unsigned int survey_count;

		bool useRandomSearch = false;
		bool hasExecutedOnce = false;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;

		Real timeInsideRedCircle;
		Real totalTimeInsideRedCircle;
		bool timeSet;

};

#endif /* CPFA_CONTROLLER_H */
