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
		argos::CVector2 FindClosestPointOnPath(const std::vector<argos::CVector2>& path);
		argos::CVector2 OffsetToSide(const argos::CVector2& A, const argos::CVector2& B, Real offset, bool left);
		int FindClosestPointIndexOnPath(const std::vector<argos::CVector2>& path);
		int FindClosestForwardWaypoint(const std::vector<argos::CVector2>& path);
		CVector2 FindClosestNest();
		bool inCentralZone();
		bool IsInsideRestrictedExitCorridor(const argos::CVector2& robotPos);
		bool IsInsideCircleBoundary(const argos::CVector2& pos);
		argos::CVector2 SafeTargetFromHeading(argos::CRadians base_angle, argos::Real step_size);
		bool hasLoggedCentralZone = false;
		bool runningfromcorridor = false;
		// std::vector<argos::CVector2> entryPoints = {{2.5, 0}, {-2.5, 0}, {0, 2.5}, {0, -2.5}}; // entry point of the paths
		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.5, 0}, {2.0, 0}, {2.0, 0.5}, {1.5, 0.5},
		// 	{1.5, -0.5}, {1.0, -0.5}, {1.0, 0.5},
		// 	{0.5, 0}, {0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.5, 0}, {-2.0, 0}, {-2.0, 0.5}, {-1.5, 0.5},
		// 	{-1.5, -0.5}, {-1.0, -0.5}, {-1.0, 0.5},
		// 	{-0.5, 0}, {-0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0, 2.5}, {0, 2.0}, {0.5, 2.0}, {0.5, 1.5},
		// 	{-0.5, 1.5}, {-0.5, 1.0}, {0.5, 1.0},
		// 	{0, 0.5}, {0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0, -2.5}, {0, -2.0}, {0.5, -2.0}, {0.5, -1.5},
		// 	{-0.5, -1.5}, {-0.5, -1.0}, {0.5, -1.0}, 
		// 	{0, -0.5}, {0, -0.3}
		// };
		// Circle center and radius (adjust values as needed)
		const argos::CVector2 m_cCircleCenter = argos::CVector2(0.0, 0.0);
		const argos::Real m_fCircleRadius = 2.0;  // meters

		// std::vector<argos::CVector2> entryPoints = {{2.0, 0}, {-2.0, 0}, {0, 2.0}, {0, -2.0}}; // entry point of the paths

		
		const argos::Real EntryPointThreshold = 0.2; // Distance threshold for entry points

		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0.5}, {1.5, 0.5},
		// 	{1.5, -0.5}, {1.0, -0.5}, {1.0, 0.5},
		// 	{0.5, 0}, {0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0.5}, {-1.5, 0.5},
		// 	{-1.5, -0.5}, {-1.0, -0.5}, {-1.0, 0.5},
		// 	{-0.5, 0}, {-0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0.5, 2.0}, {0.5, 1.5},
		// 	{-0.5, 1.5}, {-0.5, 1.0}, {0.5, 1.0},
		// 	{0, 0.5}, {0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0.5, -2.0}, {0.5, -1.5},
		// 	{-0.5, -1.5}, {-0.5, -1.0}, {0.5, -1.0}, 
		// 	{0, -0.5}, {0, -0.3}
		// };


		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0.0}, {1.3, -0.9}, {1.3, 0}, {1.3, 0.9},
		// 	{0.95, 0.55}, {0.95, 0.0}, {0.95, -0.55},
		// 	{0.6, -0.2}, {0.6, 0.2}, {0.3, 0}
		// };
		
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0.0}, {-1.3, -0.9}, {-1.3, 0}, {-1.3, 0.9},
		// 	{-0.95, 0.55}, {-0.95, 0.0}, {-0.95, -0.55},
		// 	{-0.6, -0.2}, {-0.6, 0.2}, {-0.3, 0}
		// };
		
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0.0, 2.0}, {0.9, 1.3}, {0, 1.3}, {-0.9, 1.3},
		// 	{-0.55, 0.95}, {0.0, 0.95}, {0.55, 0.95},
		// 	{0.2, 0.6}, {-0.2, 0.6}, {0, 0.3}
		// };
		
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0.0, -2.0}, {-0.9, -1.3}, {0, -1.3}, {0.9, -1.3},
		// 	{0.55, -0.95}, {0.0, -0.95}, {-0.55, -0.95},
		// 	{-0.2, -0.6}, {0.2, -0.6}, {0, -0.3}
		// };


		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0}, {1.5, 0.0}, {1.5, -1.0}, {1.2, -1.0},
		// 	{1.2, 0.8}, {0.9, 0.6}, {0.9, -0.5}, {0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0}, {-1.5, 0.0}, {-1.5, -1.0}, {-1.2, -1.0},
		// 	{-1.2, 0.8}, {-0.9, 0.6}, {-0.9, -0.5}, {-0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0, 2.0}, {0.0, 1.5}, {1.0, 1.5}, {1.0, 1.2},
		// 	{-0.8, 1.2}, {-0.6, 0.9}, {0.5, 0.9}, {0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0, -2.0}, {0.0, -1.5}, {1.0, -1.5}, {1.0, -1.2},
		// 	{-0.8, -1.2}, {-0.6, -0.9}, {0.5, -0.9}, {0, -0.3}
		// };
		
		// std::vector<argos::CVector2> entryPath1 = {{2.0, 0}, {1.0, 0}, {0.3, 0}};
		// std::vector<argos::CVector2> entryPath2 = {{-2.0, 0}, {-1.0, 0}, {-0.3, 0}};
		// std::vector<argos::CVector2> entryPath3 = {{0, 2.0}, {0, 1.0}, {0, 0.3}};
		// std::vector<argos::CVector2> entryPath4 = {{0, -2.0}, {0, -1.0}, {0, -0.3}};
		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0.0}, 
		// 	{1.8, 0.5}, 
		// 	{1.5, 0.8}, 
		// 	{1.2, 0.6}, 
		// 	{1.1, 0.2},
		// 	{1.05, -0.2},
		// 	{0.8, -0.4},
		// 	{0.6, -0.2},
		// 	{0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0.0}, 
		// 	{-1.8, 0.5}, 
		// 	{-1.5, 0.8}, 
		// 	{-1.2, 0.6}, 
		// 	{-1.1, 0.2},
		// 	{-1.05, -0.2},
		// 	{-0.8, -0.4},
		// 	{-0.6, -0.2},
		// 	{-0.3, 0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0.0, 2.0}, 
		// 	{-0.5, 1.8}, 
		// 	{-0.8, 1.5}, 
		// 	{-0.6, 1.2}, 
		// 	{-0.2, 1.1},
		// 	{0.2, 1.05},
		// 	{0.4, 0.8},
		// 	{0.2, 0.6},
		// 	{0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0.0, -2.0}, 
		// 	{0.5, -1.8}, 
		// 	{0.8, -1.5}, 
		// 	{0.6, -1.2}, 
		// 	{0.2, -1.1},
		// 	{-0.2, -1.05},
		// 	{-0.4, -0.8},
		// 	{-0.2, -0.6},
		// 	{0, -0.3}
		// };
		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0.0}, {1.82, 0.35}, {1.57, 0.75}, {1.5, 0.70}, {1.5, 0.55},
		// 	{1.5, 0.30}, {1.5, -0.15}, {1.5, -0.30}, {1.5, -0.55}, {1.5, -0.70},
		// 	{1.35, -0.75}, {1.2, -0.65}, {1.2, -0.50}, {1.2, -0.35}, {1.2, -0.25},
		// 	{1.2, -0.15}, {1.2, 0.0}, {1.2, 0.20}, {1.2, 0.50}, {1.05, 0.65},
		// 	{0.9, 0.5}, {0.9, 0.25}, {0.9, 0.0}, {0.9, -0.22}, {0.80, -0.40},
		// 	{0.7, -0.35}, {0.62, -0.27}, {0.4, -0.10}, {0.3, 0.0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0.0}, {-1.82, 0.35}, {-1.57, 0.75}, {-1.5, 0.70}, {-1.5, 0.55},
		// 	{-1.5, 0.30}, {-1.5, -0.15}, {-1.5, -0.30}, {-1.5, -0.55}, {-1.5, -0.70},
		// 	{-1.35, -0.75}, {-1.2, -0.65}, {-1.2, -0.50}, {-1.2, -0.35}, {-1.2, -0.25},
		// 	{-1.2, -0.15}, {-1.2, 0.0}, {-1.2, 0.20}, {-1.2, 0.50}, {-1.05, 0.65},
		// 	{-0.9, 0.5}, {-0.9, 0.25}, {-0.9, 0.0}, {-0.9, -0.22}, {-0.80, -0.40},
		// 	{-0.7, -0.35}, {-0.62, -0.27}, {-0.4, -0.10}, {-0.3, 0.0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0.0, 2.0}, {-0.35, 1.82}, {-0.75, 1.57}, {-0.70, 1.5}, {-0.55, 1.5},
		// 	{-0.30, 1.5}, {0.15, 1.5}, {0.30, 1.5}, {0.55, 1.5}, {0.70, 1.5},
		// 	{0.75, 1.35}, {0.65, 1.2}, {0.50, 1.2}, {0.35, 1.2}, {0.25, 1.2},
		// 	{0.15, 1.2}, {0.0, 1.2}, {-0.20, 1.2}, {-0.50, 1.2}, {-0.65, 1.05},
		// 	{-0.5, 0.9}, {-0.25, 0.9}, {0.0, 0.9}, {0.22, 0.9}, {0.40, 0.80},
		// 	{0.35, 0.7}, {0.27, 0.62}, {0.10, 0.4}, {0.0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0.0, -2.0}, {0.35, -1.82}, {0.75, -1.57}, {0.70, -1.5}, {0.55, -1.5},
		// 	{0.30, -1.5}, {-0.15, -1.5}, {-0.30, -1.5}, {-0.55, -1.5}, {-0.70, -1.5},
		// 	{-0.75, -1.35}, {-0.65, -1.2}, {-0.50, -1.2}, {-0.35, -1.2}, {-0.25, -1.2},
		// 	{-0.15, -1.2}, {0.0, -1.2}, {0.20, -1.2}, {0.50, -1.2}, {0.65, -1.05},
		// 	{0.5, -0.9}, {0.25, -0.9}, {0.0, -0.9}, {-0.22, -0.9}, {-0.40, -0.80},
		// 	{-0.35, -0.7}, {-0.27, -0.62}, {-0.10, -0.4}, {0.0, -0.3}
		// };
			
		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0.0}, {1.6, 0.7}, {1.6, -0.7}, {1.3, -0.9}, {1.3, 0.0},
		// 	{1.3, 0.9}, {0.95, 0.55}, {0.95, 0.0}, {0.95, -0.55},
		// 	{0.6, -0.2}, {0.6, 0.2}, {0.3, 0.0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0.0}, {-1.6, 0.7}, {-1.6, -0.7}, {-1.3, -0.9}, {-1.3, 0.0},
		// 	{-1.3, 0.9}, {-0.95, 0.55}, {-0.95, 0.0}, {-0.95, -0.55},
		// 	{-0.6, -0.2}, {-0.6, 0.2}, {-0.3, 0.0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0.0, 2.0}, {-0.7, 1.6}, {0.7, 1.6}, {0.9, 1.3}, {0.0, 1.3},
		// 	{-0.9, 1.3}, {-0.55, 0.95}, {0.0, 0.95}, {0.55, 0.95},
		// 	{0.2, 0.6}, {-0.2, 0.6}, {0.0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0.0, -2.0}, {0.7, -1.6}, {-0.7, -1.6}, {-0.9, -1.3}, {0.0, -1.3},
		// 	{0.9, -1.3}, {0.55, -0.95}, {0.0, -0.95}, {-0.55, -0.95},
		// 	{-0.2, -0.6}, {0.2, -0.6}, {0.0, -0.3}
		// };
								
		// std::vector<argos::CVector2> entryPath1 = {
		// 	{2.0, 0.0}, {1.8, 0.32}, {1.6, 0.58}, {1.6, 0.24}, {1.6, -0.04}, {1.6, -0.32}, {1.6, -0.9},
		// 	{1.3, -0.75}, {1.3, -0.6}, {1.3, -0.4}, {1.3, -0.2}, {1.3, 0.0},
		// 	{1.3, 0.225}, {1.3, 0.45}, {1.3, 0.675}, {1.3, 0.9},
		// 	{0.95, 0.55}, {0.95, 0.4125}, {0.95, 0.275}, {0.95, 0.1375}, {0.95, 0.0},
		// 	{0.95, -0.1375}, {0.95, -0.275}, {0.95, -0.4125}, {0.95, -0.55},
		// 	{0.6, -0.2}, {0.6, -0.1}, {0.6, 0.0}, {0.6, 0.1}, {0.6, 0.2},
		// 	{0.525, 0.15}, {0.45, 0.1}, {0.375, 0.05}, {0.3, 0.0}
		// };
		
		
		// std::vector<argos::CVector2> entryPath2 = {
		// 	{-2.0, 0.0}, {-1.8, 0.32}, {-1.6, 0.58}, {-1.6, 0.24}, {-1.6, -0.04}, {-1.6, -0.32}, {-1.6, -0.9},
		// 	{-1.3, -0.75}, {-1.3, -0.6}, {-1.3, -0.4}, {-1.3, -0.2}, {-1.3, 0.0},
		// 	{-1.3, 0.225}, {-1.3, 0.45}, {-1.3, 0.675}, {-1.3, 0.9},
		// 	{-0.95, 0.55}, {-0.95, 0.4125}, {-0.95, 0.275}, {-0.95, 0.1375}, {-0.95, 0.0},
		// 	{-0.95, -0.1375}, {-0.95, -0.275}, {-0.95, -0.4125}, {-0.95, -0.55},
		// 	{-0.6, -0.2}, {-0.6, -0.1}, {-0.6, 0.0}, {-0.6, 0.1}, {-0.6, 0.2},
		// 	{-0.525, 0.15}, {-0.45, 0.1}, {-0.375, 0.05}, {-0.3, 0.0}
		// };
		
		
		// std::vector<argos::CVector2> entryPath3 = {
		// 	{0.0, 2.0}, {-0.32, 1.8}, {-0.58, 1.6}, {-0.24, 1.6}, {0.04, 1.6}, {0.32, 1.6}, {0.9, 1.6},
		// 	{0.75, 1.3}, {0.6, 1.3}, {0.4, 1.3}, {0.2, 1.3}, {0.0, 1.3},
		// 	{-0.225, 1.3}, {-0.45, 1.3}, {-0.675, 1.3}, {-0.9, 1.3},
		// 	{-0.55, 0.95}, {-0.4125, 0.95}, {-0.275, 0.95}, {-0.1375, 0.95}, {0.0, 0.95},
		// 	{0.1375, 0.95}, {0.275, 0.95}, {0.4125, 0.95}, {0.55, 0.95},
		// 	{0.2, 0.6}, {0.1, 0.6}, {0.0, 0.6}, {-0.1, 0.6}, {-0.2, 0.6},
		// 	{-0.15, 0.525}, {-0.1, 0.45}, {-0.05, 0.375}, {0.0, 0.3}
		// };
		// std::vector<argos::CVector2> entryPath4 = {
		// 	{0.0, -2.0}, {0.32, -1.8}, {0.58, -1.6}, {0.24, -1.6}, {-0.04, -1.6}, {-0.32, -1.6}, {-0.9, -1.6},
		// 	{-0.75, -1.3}, {-0.6, -1.3}, {-0.4, -1.3}, {-0.2, -1.3}, {0.0, -1.3},
		// 	{0.225, -1.3}, {0.45, -1.3}, {0.675, -1.3}, {0.9, -1.3},
		// 	{0.55, -0.95}, {0.4125, -0.95}, {0.275, -0.95}, {0.1375, -0.95}, {0.0, -0.95},
		// 	{-0.1375, -0.95}, {-0.275, -0.95}, {-0.4125, -0.95}, {-0.55, -0.95},
		// 	{-0.2, -0.6}, {-0.1, -0.6}, {0.0, -0.6}, {0.1, -0.6}, {0.2, -0.6},
		// 	{0.15, -0.525}, {0.1, -0.45}, {0.05, -0.375}, {0.0, -0.3}
		// };				
		std::vector<argos::CVector2> entryPath1 = {
			{ 1.60,  1.1}, { 1.60, -1.1},
			{ 1.45, -1.1}, { 1.45,  1.05},
			{ 1.30,  1.05}, { 1.30, -0.85},
			{ 1.15, -0.85}, { 1.15,  0.74},
			{ 1.00,  0.74}, { 1.00, -0.55},
			{ 0.85, -0.55}, { 0.85,  0.45},
			{ 0.70,  0.45}, { 0.70, -0.3},
			{ 0.55, -0.3}, { 0.55,  0.26},
			{ 0.30,  0.0}
		};
		
		std::vector<argos::CVector2> entryPath2 = {
			{-1.60,  1.1}, {-1.60, -1.1},
			{-1.45, -1.1}, {-1.45,  1.05},
			{-1.30,  1.05}, {-1.30, -0.85},
			{-1.15, -0.85}, {-1.15,  0.74},
			{-1.00,  0.74}, {-1.00, -0.55},
			{-0.85, -0.55}, {-0.85,  0.45},
			{-0.70,  0.45}, {-0.70, -0.3},
			{-0.55, -0.3}, {-0.55,  0.26},
			{-0.30,  0.0}
		};
		
		
		std::vector<argos::CVector2> entryPath3 = {
			{ 1.1,  1.60}, {-1.1,  1.60},
			{-1.1,  1.45}, { 1.05,  1.45},
			{ 1.05,  1.30}, {-0.85,  1.30},
			{-0.85,  1.15}, { 0.74,  1.15},
			{ 0.74,  1.00}, {-0.55,  1.00},
			{-0.55,  0.85}, { 0.45,  0.85},
			{ 0.45,  0.70}, {-0.3,  0.70},
			{-0.3,  0.55}, { 0.26,  0.55},
			{ 0.0,  0.30}
		};
		
		std::vector<argos::CVector2> entryPath4 = {
			{ 1.1, -1.60}, {-1.1, -1.60},
			{-1.1, -1.45}, { 1.05, -1.45},
			{ 1.05, -1.30}, {-0.85, -1.30},
			{-0.85, -1.15}, { 0.74, -1.15},
			{ 0.74, -1.00}, {-0.55, -1.00},
			{-0.55, -0.85}, { 0.45, -0.85},
			{ 0.45, -0.70}, {-0.3, -0.70},
			{-0.3, -0.55}, { 0.26, -0.55},
			{ 0.0, -0.30}
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
		// std::vector<argos::CVector2> exitPath1 = {{0.3, 0.3}, {1.4, 1.4}};
		// std::vector<argos::CVector2> exitPath2 = {{-0.3, -0.3}, {-1.4, -1.4}};
		// std::vector<argos::CVector2> exitPath3 = {{-0.3, 0.3}, {-1.4, 1.4}};
		// std::vector<argos::CVector2> exitPath4 = {{0.3, -0.3}, {1.4, -1.4}};
		std::vector<argos::CVector2> exitPath1 = {{0.3, 0.3}, {1.6, 1.6}};
		std::vector<argos::CVector2> exitPath2 = {{-0.3, -0.3}, {-1.6, -1.6}};
		std::vector<argos::CVector2> exitPath3 = {{-0.3, 0.3}, {-1.6, 1.6}};
		std::vector<argos::CVector2> exitPath4 = {{0.3, -0.3}, {1.6, -1.6}};
		std::vector<argos::CVector2> exitPoints = {{1.6, 1.6}, {-1.6, -1.6}, {-1.6, 1.6}, {1.6, -1.6}}; // exit point of the paths

		std::vector<argos::CVector2> actualExitPath;
		bool followingEntryPath1 = false;
		bool followingEntryPath2 = false;
		bool followingEntryPath3 = false;
		bool followingEntryPath4 = false;
		argos::CRange<argos::Real> GoStraightAngleRangeInDegreesInRegion;
		argos::CRange<argos::Real> GoStraightAngleRangeInDegreesGoingToRegion;
		argos::CRange<argos::Real> GoStraightAngleRangeInDegreesLeftSide;
		argos::CRange<argos::Real> GoStraightAngleRangeInDegreesRightSide;

		size_t stopCounter = 0; // Counter to track how many timesteps the robot has been stopped
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


		bool IsLeftOfLine(const argos::CVector2& A, const argos::CVector2& B, const argos::CVector2& P);
		Real DistanceFromPointToSegment(const argos::CVector2& P, const argos::CVector2& A, const argos::CVector2& B);
		bool IsLeftOfPath(const std::vector<argos::CVector2>& path, const argos::CVector2& pos);
};

#endif /* CPFA_CONTROLLER_H */
