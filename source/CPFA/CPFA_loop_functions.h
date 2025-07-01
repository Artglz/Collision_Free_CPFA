#ifndef CPFA_LOOP_FUNCTIONS_H
#define CPFA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/CPFA/CPFA_controller.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm> 
#include <json/json.h>

#pragma push_macro("slots")
#undef slots
// #include "Python.h"
#pragma pop_macro("slots")

using namespace argos;
using namespace std;

static const size_t GENOME_SIZE = 7; // There are 7 parameters to evolve


class CPFA_loop_functions : public argos::CLoopFunctions
{

	friend class CPFA_controller;
	friend class CPFA_qt_user_functions;

	public:
		void UpdateInCircleCounter(size_t counter);
		CPFA_loop_functions();
		void Init(argos::TConfigurationNode &t_tree);
		void Reset();
		void PreStep();
		void PostStep();
		bool IsExperimentFinished();
		void PostExperiment();
		argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

		// GA Functions
		
		/* Configures the robot controller from the genome */
		void ConfigureFromGenome(Real* pf_genome);
		/* Calculates the performance of the robot in a trial */
		Real Score();
	
		/**
		 * Returns the current trial.
		 */
		UInt32 GetTrial() const;
	
		/**
		 * Sets the current trial.
		 * @param un_trial The trial number.
		 */
		void SetTrial(UInt32 un_trial);
	
		/* public helper functions */
		void UpdatePheromoneList();
		void SetFoodDistribution();

		argos::Real getSimTimeInSeconds();

		std::vector<argos::CColor>   TargetRayColorList;


		unsigned int getNumberOfRobots();
        void increaseNumDistributedFoodByOne();
		double getProbabilityOfSwitchingToSearching();
		double getProbabilityOfReturningToNest();
		double getUninformedSearchVariation();
		double getRateOfInformedSearchDecay();
		double getRateOfSiteFidelity();
		double getRateOfLayingPheromone();
		double getRateOfPheromoneDecay();
		// std::vector<argos::CVector2> entryPoints = {{2.5, 0}, {-2.5, 0}, {0, 2.5}, {0, -2.5}}; // entry point of the paths
		// std::vector<argos::CVector2> entryPoints = {{2.0, 0}, {-2.0, 0}, {0, 2.0}, {0, -2.0}};
		// std::vector<argos::CVector2> entryPoints = {    
		// 	{2.0, 0.0}, {1.82, 0.35}, {1.57, 0.75}, {1.5, 0.70}, {1.5, 0.55},
		// 	{1.5, 0.30}, {1.5, -0.15}, {1.5, -0.30}, {1.5, -0.55}, {1.5, -0.70},
		// 	{1.35, -0.75}, {1.2, -0.65}, {1.2, -0.50}, {1.2, -0.35}, {1.2, -0.25},
		// 	{1.2, -0.15}, {1.2, 0.0}, {1.2, 0.20}, {1.2, 0.50}, {1.05, 0.65},
		// 	{0.9, 0.5}, {0.9, 0.25}, {0.9, 0.0}, {0.9, -0.22}, {0.80, -0.40},
		// 	{0.7, -0.35}, {0.62, -0.27}, {0.4, -0.10}, {0.3, 0.0}	};
		// std::vector<argos::CVector2> entryPoints = {
		// 	{2.0, 0.0},
		// 	{1.933, 0.167},
		// 	{1.867, 0.333},
		// 	{1.8, 0.5},
		// 	{1.7, 0.6},
		// 	{1.6, 0.7},
		// 	{1.5, 0.8},
		// 	{1.4, 0.733},
		// 	{1.3, 0.667},
		// 	{1.2, 0.6},
		// 	{1.167, 0.467},
		// 	{1.133, 0.333},
		// 	{1.1, 0.2},
		// 	{1.067, 0.067},
		// 	{1.033, -0.067},
		// 	{1.05, -0.2},
		// 	{0.967, -0.267},
		// 	{0.883, -0.333},
		// 	{0.8, -0.4},
		// 	{0.733, -0.333},
		// 	{0.667, -0.267},
		// 	{0.6, -0.2},
		// 	{0.5, -0.133},
		// 	{0.4, -0.067},
		// 	{0.3, 0}
		// };


		// std::vector<argos::CVector2> entryPoints = {{}}
		// std::vector<argos::CVector2> entryPoints = {{2.0, 0}};

		// std::vector<argos::CVector2> entryPoints = {{2.0, 0}, {1.5, 0.0}, {1.5, -1.0}, {1.2, -1.0}, {1.2, 0.8}, {0.9, 0.6}, {0.9, -0.5}, {.3, 0}}; // entry point of the paths
		// std::vector<argos::CVector2> exitPoints = {{0.3, 0.3},{-0.3, -0.3},{-0.3, 0.3}, {0.3, -0.3}}; // exit point of the paths
		// std::vector<argos::CVector2> entryPoints = {{2.0, 0.0}, {1.3, -1.0}, {1.3, 1.0}, {0.95, 0.65}, {0.95, -0.65}, {.6, -0.3}, {.6, 0.3}, {.3, 0}}; // exit point of the paths
		// std::vector<argos::CVector2> entryPoints = {			{2.0, 0.0}, {1.8, 0.32},
		// {1.6, 0.58}, {1.6, 0.24}, {1.6, -0.04}, {1.6, -0.32}, {1.6, -0.9},
		// {1.3, -0.75},
		// {1.3, -0.6}, {1.3, -0.4}, {1.3, -0.2}, {1.3, 0.0},
		// {1.3, 0.225}, {1.3, 0.45}, {1.3, 0.675}, {1.3, 0.9},
		//  {0.95, 0.55},
		// {0.95, 0.4125}, {0.95, 0.275}, {0.95, 0.1375}, {0.95, 0.0},
		// {0.95, -0.1375}, {0.95, -0.275}, {0.95, -0.4125}, {0.95, -0.55}, {0.6, -0.2},
		// {0.6, -0.1}, {0.6, 0.0}, {0.6, 0.1}, {0.6, 0.2},
		// {0.525, 0.15}, {0.45, 0.1}, {0.375, 0.05}, {0.3, 0.0}};
		// std::vector<argos::CVector2> entryPoints = {{1.6,  0.9}, {1.6, -0.9}, {1.5, -0.9}, {1.5,  0.9}, {1.4,  0.9}, {1.4, -0.9},
		// {1.3, -0.9}, {1.3,  0.9}, {1.2, -0.9}, {1.2,  0.9}, {1.1,  0.9}, {1.1, -0.75},
		// {1.0, -0.75}, {1.0,  0.7}, {0.9,  0.7}, {0.9, -0.55}, {0.8, -0.55}, {0.8,  0.45},
		// {0.7,  0.45}, {0.7, -0.3}, {0.6, -0.3}, {0.6,  0.26}, {0.5,  0.26}, {0.5, -0.2},
		// {0.4, -0.2}, {0.3,  0.0}};
		std::vector<argos::CVector2> entryPoints = {	
			// { 1.60,  1.10 }, { 1.60,  0.825 }, { 1.60,  0.55 }, { 1.60,  0.275 },
			// { 1.60,  0.00 }, { 1.60, -0.275 }, { 1.60, -0.55 }, { 1.60, -0.825 },
			// { 1.60, -1.10 }, { 1.51, -1.10 }, { 1.42, -1.10 }, { 1.42, -0.84375 },
			// { 1.42, -0.5875 }, { 1.42, -0.33125 }, { 1.42, -0.075 }, { 1.42,  0.18125 },
			// { 1.42,  0.4375 }, { 1.42,  0.69375 }, { 1.42,  0.95 }, { 1.33,  0.95 },
			// { 1.24,  0.95 }, { 1.24,  0.7375 }, { 1.24,  0.525 }, { 1.24,  0.3125 },
			// { 1.24,  0.10 }, { 1.24, -0.1125 }, { 1.24, -0.325 }, { 1.24, -0.5375 },
			// { 1.24, -0.75 }, { 1.15, -0.75 }, { 1.06, -0.75 }, { 1.06, -0.57625 },
			// { 1.06, -0.4025 }, { 1.06, -0.22875 }, { 1.06, -0.055 }, { 1.06,  0.11875 },
			// { 1.06,  0.2925 }, { 1.06,  0.46625 }, { 1.06,  0.64 },
			// { 0.88,  0.64 }, {0.88, 0.32}, { 0.88,  0.095 }, { 0.88, -0.45 }, { 0.88, -0.26 },
			// { 0.72, -0.45 }, { 0.72, -0.26 }, { 0.72, -0.07 }, { 0.72,  0.12 },
			// { 0.72,  0.31 }, { 0.675, 0.31 }, { 0.63,  0.31 }, { 0.585, 0.31 },
			// { 0.54,  0.31 }, { 0.54,  0.2325 }, { 0.54,  0.155 }, { 0.54,  0.0775 },
			// { 0.54,  0.0 },
			// { 0.36,  0.0 }, 
			// { 0.30,  0.0 }
			
		};
		// 	{2.0, 0.0}, 
		// 	{1.8, 0.5}, 
		// 	{1.5, 0.8}, 
		// 	{1.2, 0.6}, 
		// 	{1.1, 0.2},
		// 	{1.05, -0.2},
		// 	{0.8, -0.4},
		// 	{0.6, -0.2}
		// };
		
		//initialize 4 queues data type for each entry path
		// std::queue<argos::CVector2> entryQueue1;
		// std::queue<argos::CVector2> entryQueue2;
		// std::queue<argos::CVector2> entryQueue3;
		// std::queue<argos::CVector2> entryQueue4;


	protected:
		std::vector<size_t> InCircleCounters;
		void setScore(double s);

		argos::CRandom::CRNG* RNG;
                size_t NumDistributedFood;
		size_t MaxSimTime;
		size_t ResourceDensityDelay;
		size_t RandomSeed;
		size_t SimCounter;
		size_t MaxSimCounter;
		size_t VariableFoodPlacement;
		size_t OutputData;
		size_t DrawDensityRate;
		size_t DrawIDs;
		size_t DrawTrails;
		size_t DrawTargetRays;
		size_t FoodDistribution;
		size_t FoodItemCount;
		size_t PowerlawFoodUnitCount;
		size_t NumberOfClusters;
		size_t ClusterWidthX;
		size_t ClusterWidthY;
		size_t PowerRank;
                size_t ArenaWidth;
                size_t SimTime; 
                Real curr_time_in_minutes; 
                Real last_time_in_minutes; 
  
		/* CPFA variables */
		argos::Real ProbabilityOfSwitchingToSearching;
		argos::Real ProbabilityOfReturningToNest;
		argos::CRadians UninformedSearchVariation;
		argos::Real RateOfInformedSearchDecay;
		argos::Real RateOfSiteFidelity;
		argos::Real RateOfLayingPheromone;
		argos::Real RateOfPheromoneDecay;
		
		/* physical robot & world variables */
		argos::Real FoodRadius;
		argos::Real FoodRadiusSquared;
		argos::Real NestRadius;
		argos::Real NestRadiusSquared;
		argos::Real NestElevation;
		argos::Real SearchRadiusSquared;
		argos::Real CameraRadiusSquared;
		
		/* list variables for food & pheromones */
		std::vector<argos::CVector2> FoodList;
		std::vector<argos::CColor>   FoodColoringList;
		vector<argos::CVector2> CollectedFoodList;
                map<string, argos::CVector2> FidelityList; 
		std::vector<Pheromone>   PheromoneList; 
		std::vector<argos::CRay3>    TargetRayList;
		map<string, std::vector<CVector2> >  Trajectory;
		
		argos::CRange<argos::Real>   ForageRangeX;
		argos::CRange<argos::Real>   ForageRangeY;
		map<string, argos::CVector2> robotPosList; //qilu 06/2023
		//vector<argos::CVector2> robotPosList; //qilu 06/2023
		map<string, vector<argos::CVector2>> robotPosList3;

		
                Real   CollisionTime;
                size_t currCollisionTime; 
                size_t lastCollisionTime; 
                size_t lastNumCollectedFood;
                size_t currNumCollectedFood;
                size_t Num_robots;
      
                vector<size_t>		ForageList;
		argos::CVector2 NestPosition;
		std::vector<argos::CVector2> NestPositions;
	private:			


		/* private helper functions */
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();


        bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		double score;
		int PrintFinalScore;
};

#endif /* CPFA_LOOP_FUNCTIONS_H */
