#include "CPFA_loop_functions.h"

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
        SimTime(0),
	//MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()),
    MaxSimTime(0),//qilu 02/05/2021
        CollisionTime(0), 
        lastNumCollectedFood(0),
        currNumCollectedFood(0),
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	FoodDistribution(2),
	FoodItemCount(256),
	PowerlawFoodUnitCount(256),
	NumberOfClusters(4),
	ClusterWidthX(8),
	ClusterWidthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.12),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	CameraRadiusSquared(2.25),
	NumDistributedFood(0),
	score(0),
	PrintFinalScore(0)
{}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {	
 
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");

	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	
	argos::GetNodeAttribute(CPFA_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");
	
	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/05/2021 dyn2d error
	argos::GetNodeAttribute(settings_node, "MaxSimCounter", MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", DrawTargetRays);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", FoodDistribution);
	argos::GetNodeAttribute(settings_node, "FoodItemCount", FoodItemCount);
	argos::GetNodeAttribute(settings_node, "PowerlawFoodUnitCount", PowerlawFoodUnitCount);
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterWidthY", ClusterWidthY);
	argos::GetNodeAttribute(settings_node, "FoodRadius", FoodRadius);
    argos::GetNodeAttribute(settings_node, "NestRadius", NestRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", NestElevation);
    argos::GetNodeAttribute(settings_node, "NestPosition", NestPosition);
    FoodRadiusSquared = FoodRadius*FoodRadius;
    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
    }
    else{
        NumDistributedFood = FoodItemCount;  
    }
    
	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

        ArenaWidth = ArenaSize[0];
        
       /* if(abs(NestPosition.GetX()) < -1) //quad arena
        {
            NestRadius *= sqrt(1 + log(ArenaWidth)/log(2));
        }
        else
        {
            NestRadius *= sqrt(log(ArenaWidth)/log(2));
        } */
        
        //argos::LOG<<"NestRadius="<<NestRadius<<endl;
	   // Send a pointer to this loop functions object to each controller.
	   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    Num_robots = footbots.size();
    argos::LOG<<"Number of robots="<<Num_robots<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        c2.SetLoopFunctions(this);
	    }
     
     
   NestRadiusSquared = NestRadius*NestRadius;
	
    SetFoodDistribution();
  
	ForageList.clear(); 
	last_time_in_minutes=0;
	SetupPythonEnvironment();
 
}

void CPFA_loop_functions::RegisterActorState(string robot_id, const CPFA_controller::ActorState& state) {
	m_localStates[robot_id] = state;
}

void CPFA_loop_functions::Reset() {
	   if(VariableFoodPlacement == 0) {
		      RNG->Reset();
	   }

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    MaxSimCounter = SimCounter;
    SimCounter = 0;
    score = 0;
   
    FoodList.clear();
    CollectedFoodList.clear();
    FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
    TargetRayList.clear();
    Trajectory.clear();
    
    SetFoodDistribution();
    
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;
   
    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    c2.Reset();
    }
}

void CPFA_loop_functions::PreStep() {
    SimTime++;
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    if(curr_time_in_minutes - last_time_in_minutes==1){
		      
        ForageList.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
    }
    UpdatePheromoneList();
	//print timestep
	//argos::LOG << "timestep: " << GetSpace().GetSimulationClock() << std::endl;
	if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
      for(size_t i = 0; i < FoodColoringList.size(); i++) {
            FoodColoringList[i] = argos::CColor::BLACK;
      }
	}
	argos::CVector2 position;
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    
    robotPosList.clear();
    for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
      argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
      position = c2.GetPosition();
      robotPosList[c2.GetId()] = position;
      //robotPosList.push_back(position);
    }
    
    //for(map<string, CVector2>::iterator it= robotPosList.begin(); it!=robotPosList.end(); ++it) {
	//	argos::LOG << "pos["<< it->first <<"]="<< it->second << endl;
	//}
         
    if(FoodList.size() == 0) {
	FidelityList.clear();
	PheromoneList.clear();
        TargetRayList.clear();
        Trajectory.clear();
    }
}

void CPFA_loop_functions::PostStep() {

	size_t N = Num_robots;
	float sum_efficiency = 0.0f;
	float sum_collisions = 0.0f;
	size_t near_nest_count = 0;

	for (auto const& kv : m_localStates) {
		const auto& st = kv.second;
		sum_efficiency += st.path_efficiency;
		sum_collisions += static_cast<float>(st.collisions);
		if (st.distance_to_nest <= 1.00f) {
			++near_nest_count;
		}
	}

	//check if a robot is within

	CriticState cstate;
	// if (N > 0) {
		cstate.mean_path_efficiency   = sum_efficiency / static_cast<float>(m_localStates.size());
		cstate.nest_congestion_index  = static_cast<float>(near_nest_count) / static_cast<float>(N);
		cstate.total_collisions  = sum_collisions / m_localStates.size();
	// } else {
	// 	cstate.mean_path_efficiency   = 0.0f;
	// 	cstate.nest_congestion_index  = 0.0f;
	// 	cstate.total_collisions  = 0.0f;
	// }

	//log m_localStates
	// for(auto it = m_localStates.begin(); it != m_localStates.end(); ++it) {
	// 	argos::LOG << "robot["<< it->first <<"]="<< it->second.distance_to_nest << ", "<< it->second.timesteps_returning << ", "<< it->second.collisions << ", "<< it->second.path_efficiency << ", "<< it->second.angular_deviation << endl;
	// }

	//log cstate
	//argos::LOG << "cstate="<< cstate.mean_path_efficiency << ", "<< cstate.nest_congestion_index << ", "<< cstate.total_collisions << endl;

	m_mapRobotActions = CallPythonTrainStep(m_localStates, cstate);
	
	// CallPythonTrainStep(m_localStates, cstate);
	//log m_mapRobotActions
	// for(auto it = m_mapRobotActions.begin(); it != m_mapRobotActions.end(); ++it) {
	// 	argos::LOG << "robot["<< it->first <<"]="<< it->second[0] << ", "<< it->second[1] << endl;
	// }

	m_localStates.clear();

	// if robot id from m_mapRobotActions matches with id of cpfa_controller robot, then store the action in std::vector<float> robotActions variable from cpfa_controller
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	for (auto it = footbots.begin(); it != footbots.end(); ++it) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		string robot_id = c2.GetId();
		
		if (m_mapRobotActions.find(robot_id) != m_mapRobotActions.end()) {
			//access std::vector<float> robotActions from cpfa_controller
			c2.robotActions = m_mapRobotActions[robot_id];
		}
	}
}

bool CPFA_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
		isFinished = true;
	}
    //set to collected 88% food and then stop
    if(score >= NumDistributedFood){
		isFinished = true;
		}
         
         
    
	if(isFinished == true && MaxSimCounter > 1) {
		size_t newSimCounter = SimCounter + 1;
		size_t newMaxSimCounter = MaxSimCounter - 1;
        argos::LOG<< "time out..."<<endl; 
		PostExperiment();
		Reset();

		SimCounter    = newSimCounter;
		MaxSimCounter = newMaxSimCounter;
		isFinished    = false;
	}

	return isFinished;
}

void CPFA_loop_functions::PostExperiment() {
	  
    //  printf("%f, %f, %lu\n", score, getSimTimeInSeconds(), RandomSeed);
    //  printf("%f\n", score);  
	// argos::LOG << resources_dropped << " resources dropped" << std::endl;

	// argos::LOG << totalResourcesPickedUp << " resources picked up" << std::endl;	

    // if (PrintFinalScore == 1) {
    //     string type="";
    //     if (FoodDistribution == 0) type = "random";
    //     else if (FoodDistribution == 1) type = "cluster";
    //     else type = "powerlaw";
            
    //     ostringstream num_tag;
    //     num_tag << FoodItemCount; 
              
    //     ostringstream num_robots;
    //     num_robots <<  Num_robots;
   
    //     ostringstream arena_width;
    //     arena_width << ArenaWidth;
        
    //     ostringstream quardArena;
    //     if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
    //          quardArena << 1;
    //      }
    //      else{
    //          quardArena << 0;
    //     }
        
    //     string header = "./results/"+ type+"_CPFA_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_" + quardArena.str() +"_";
       
    //     unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
       
    //     /* Real total_travel_time=0;
    //     Real total_search_time=0;
    //     ofstream travelSearchTimeDataOutput((header+"TravelSearchTimeData.txt").c_str(), ios::app);
    //     */
        
        
    //     argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
    //     for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
    //         argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
    //         BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
    //         CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
    //         CollisionTime += c2.GetCollisionTime();
            
    //         /*if(c2.GetStatus() == "SEARCHING"){
    //             total_search_time += SimTime-c2.GetTravelingTime();
    //             total_travel_time += c2.GetTravelingTime();
	//     }
    //         else {
	// 	total_search_time += c2.GetSearchingTime();
	// 	total_travel_time += SimTime-c2.GetSearchingTime();
    //         } */        
    //     }
    //     //travelSearchTimeDataOutput<< total_travel_time/ticks_per_second<<", "<<total_search_time/ticks_per_second<<endl;
    //     //travelSearchTimeDataOutput.close();   
             
    //     ofstream dataOutput( (header+ "iAntTagDa.txt").c_str(), ios::app);
    //     // output to file
    //     if(dataOutput.tellp() == 0) {
    //         dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
    //     }
    
    //     //dataOutput <<data.CollisionTime/16.0<<", "<< time_in_minutes << ", " << data.RandomSeed << endl;
    //     //dataOutput << Score() << ", "<<(CollisionTime-16*Score())/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
    //     dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", " << totalResourcesPickedUp << ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
    //     dataOutput.close();

	// 	/*
    //     ofstream densityOutput( ("./results/densities.txt"), ios::app);
    //     densityOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
    //     densityOutput.close();
	// 	*/
    //     ofstream forageDataOutput((header+"ForageData.txt").c_str(), ios::app);
    //     if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
    //     for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
    //     forageDataOutput<<"\n";
    //     forageDataOutput.close();
        
    //     ofstream trajOutput( (header+ "iAntTrajData.txt").c_str(), ios::app);
    //     // output to file
    //     //if(trajOutput.tellp() == 0) {
    //         trajOutput << "trajs\n";//qilu 11/2023
    //     //}
        
    //     for(map<string, std::vector<CVector2>>::iterator it= Trajectory.begin(); it!= Trajectory.end(); ++it) {
			
	// 		for(size_t j = 0; j < it->second.size(); j++) {
	// 			trajOutput << it->second[j]<<"; ";
	// 		}
	// 		trajOutput << "\n";
		
	// 	}
        
	// 	trajOutput.close();
   
    //   }  


	CallPythonSaveModels();
}


argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void CPFA_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	//ofstream log_output_stream;
	//log_output_stream.open("time.txt", ios::app);
	//log_output_stream << t << ", " << GetSpace().GetSimulationClock() << ", " << GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() << endl;
	//log_output_stream.close();
	    for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);
		if(PheromoneList[i].IsActive()) {
			new_p_list.push_back(PheromoneList[i]);
		}
      }
     	PheromoneList = new_p_list;
	new_p_list.clear();
}
void CPFA_loop_functions::SetFoodDistribution() {
	switch(FoodDistribution) {
		case 0:
			RandomFoodDistribution();
			break;
		case 1:
			ClusterFoodDistribution();
			break;
		case 2:
			PowerLawFoodDistribution();
			break;
		default:
			argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
	}
}

void CPFA_loop_functions::RandomFoodDistribution() {
	FoodList.clear();
        FoodColoringList.clear();
	argos::CVector2 placementPosition;

	for(size_t i = 0; i < FoodItemCount; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		FoodList.push_back(placementPosition);
		FoodColoringList.push_back(argos::CColor::BLACK);
	}
}

 
void CPFA_loop_functions::ClusterFoodDistribution() {
        FoodList.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterWidthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	FoodItemCount = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterWidthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < ClusterWidthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}


void CPFA_loop_functions::PowerLawFoodDistribution() {
 FoodList.clear();
    FoodColoringList.clear();
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
		}
	FoodItemCount = foodPlaced;
}
 
bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
	argos::CVector2 placementPosition = p;

	argos::Real foodOffset   = 3.0 * FoodRadius;
	argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
	argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

	argos::Real x_min = p.GetX() - FoodRadius;
	argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

	argos::Real y_min = p.GetY() - FoodRadius;
	argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

	if((x_min < (ForageRangeX.GetMin() + FoodRadius))
			|| (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
			(y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
			(y_max > (ForageRangeY.GetMax() - FoodRadius)))
	{
		return true;
	}

	for(size_t j = 0; j < length; j++) {
		for(size_t k = 0; k < width; k++) {
			if(IsCollidingWithFood(placementPosition)) return true;
			if(IsCollidingWithNest(placementPosition)) return true;
			placementPosition.SetX(placementPosition.GetX() + foodOffset);
		}

		placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
		placementPosition.SetY(placementPosition.GetY() + foodOffset);
	}

	return false;
}

  
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

      return ( (p - NestPosition).SquareLength() < NRPB_squared) ;
}

bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int CPFA_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double CPFA_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double CPFA_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double CPFA_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double CPFA_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double CPFA_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double CPFA_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double CPFA_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real CPFA_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void CPFA_loop_functions::SetTrial(unsigned int v) {
}

void CPFA_loop_functions::setScore(double s) {
	score = s;
    
	if (score >= NumDistributedFood) {
		PostExperiment();
	}
}

double CPFA_loop_functions::Score() {	
	return score;
}

void CPFA_loop_functions::increaseNumDistributedFoodByOne(){
    NumDistributedFood++;
}

void CPFA_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

bool CPFA_loop_functions::SetupPythonEnvironment(){

	Py_Initialize();
	if(Py_IsInitialized()){
		LOG << "Python version: " << Py_GetVersion() << endl;
		return 1;
	} else {
		LOGERR << "ERROR: Python failed to initialize." << endl;
		return 0;	
	}

	
	PyObject *sys = PyImport_ImportModule("sys");
	PyObject *path = PyObject_GetAttrString(sys, "path");
	PyList_Append(path, PyUnicode_FromString("/home/arturo/src/argos3/build_simulator/Collision_Free_CPFA/source/CPFA"));

	Py_DECREF(path);
	Py_DECREF(sys);

	// // Load the module
	// pyFileName = PyUnicode_FromString("congestion");
	// if (pyFileName == NULL) {
	// 	LOG << "Error converting module name to PyUnicode" << std::endl;
	// 	Py_Finalize();
	// 	return 0;
	// }

	// pyModule = PyImport_Import(pyFileName);
	// Py_DECREF(pyFileName);

	// if (pyModule == NULL) {
	// 	LOG << "Failed to load Python module" << std::endl;
	// 	Py_Finalize();
	// 	return 0;
	// }

	// // Load the function from the module
	// pyCongestion = PyObject_GetAttrString(pyModule, "run_congestion_logic");
	// Py_DECREF(pyModule);


    // if (!pyCongestion || !PyCallable_Check(pyCongestion)) {
    //     LOGERR << "ERROR: Python function 'run_congestion_logic' is not callable." << std::endl;
    //     if (PyErr_Occurred()) PyErr_Print();
    //     Py_XDECREF(pyCongestion);
    //     Py_Finalize();
    //     return false;
    // }

    // LOG << "Python environment successfully initialized." << std::endl;
    return true;

}

std::map<std::string, std::vector<float>> CPFA_loop_functions::CallPythonTrainStep(
    const std::map<std::string, CPFA_controller::ActorState>& actorStates,
    const CriticState& gstate) {
    
    std::map<std::string, std::vector<float>> actions;
    
    if (!Py_IsInitialized()) {
        Py_Initialize();
    }

    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyUnicode_FromString("/home/arturo/src/argos3/build_simulator/Collision_Free_CPFA/source/CPFA"));
    PyList_Append(path, PyUnicode_FromString("/home/arturo/venvs/rl/lib/python3.10/site-packages"));
    Py_DECREF(path);
    Py_DECREF(sys);

    PyGILState_STATE gil = PyGILState_Ensure();

    PyObject* pName = PyUnicode_FromString("rltrainer");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        std::cerr << "Failed to load rltrainer.py" << std::endl;
        PyGILState_Release(gil);
        return actions;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, "get_actions");
    if (!pFunc || !PyCallable_Check(pFunc)) {
        std::cerr << "Cannot find function 'train_step'" << std::endl;
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        PyGILState_Release(gil);
        return actions;
    }

    // Build the actor_states Python dictionary
    PyObject* pActorDict = PyDict_New();
    for (auto const& [robot_id, st] : actorStates) {
        PyObject* pList = PyList_New(6);
        PyList_SetItem(pList, 0, PyFloat_FromDouble(st.distance_to_nest));
        PyList_SetItem(pList, 1, PyLong_FromLong(st.timesteps_returning));
        PyList_SetItem(pList, 2, PyLong_FromLong(st.collisions));
        PyList_SetItem(pList, 3, PyFloat_FromDouble(st.path_efficiency));
        PyList_SetItem(pList, 4, PyFloat_FromDouble(st.angular_deviation));
		PyList_SetItem(pList, 5, PyFloat_FromDouble(st.reached_nest));
        PyDict_SetItem(pActorDict, PyUnicode_FromString(robot_id.c_str()), pList);
        Py_DECREF(pList);
    }

    // Build the global_state Python list
    PyObject* pGlobalList = PyList_New(3);
    PyList_SetItem(pGlobalList, 0, PyFloat_FromDouble(gstate.nest_congestion_index));
    PyList_SetItem(pGlobalList, 1, PyFloat_FromDouble(gstate.mean_path_efficiency));
    PyList_SetItem(pGlobalList, 2, PyFloat_FromDouble(gstate.total_collisions));

    // Build argument tuple
    PyObject* pArgs = PyTuple_Pack(2, pActorDict, pGlobalList);

    // Call Python function
    PyObject* pReturn = PyObject_CallObject(pFunc, pArgs);

    if (!pReturn) {
        PyErr_Print();
        std::cerr << "Python function call failed!" << std::endl;
    } else {
        // std::cout << "Python function call succeeded!" << std::endl;

        if (PyDict_Check(pReturn)) {
            PyObject *key, *value;
            Py_ssize_t pos = 0;

            while (PyDict_Next(pReturn, &pos, &key, &value)) {
                if (!PyUnicode_Check(key) || !PyList_Check(value)) {
                    continue;
                }

                std::string robot_id = PyUnicode_AsUTF8(key);
                std::vector<float> robot_actions;
                Py_ssize_t list_size = PyList_Size(value);

                for (Py_ssize_t i = 0; i < list_size; ++i) {
                    PyObject* item = PyList_GetItem(value, i);  // Borrowed reference
                    PyObject* float_obj = PyNumber_Float(item); // New reference
                    if (float_obj) {
                        float val = static_cast<float>(PyFloat_AsDouble(float_obj));
                        robot_actions.push_back(val);
                        Py_DECREF(float_obj);
                    } else {
                        PyErr_Print();
                        std::cerr << "[WARNING] Failed to convert action item to float for robot " << robot_id << std::endl;
                    }
                }

                actions[robot_id] = robot_actions;
            }
        } else {
            std::cerr << "[ERROR] Python return is not a dictionary.\n";
        }

        Py_DECREF(pReturn);
    }

    // Clean up
    Py_DECREF(pArgs);
    Py_DECREF(pActorDict);
    Py_DECREF(pGlobalList);
    Py_DECREF(pFunc);
    Py_DECREF(pModule);

    PyGILState_Release(gil);

    return actions;
}

void CPFA_loop_functions::CallPythonSaveModels() {
    if (!Py_IsInitialized()) {
        Py_Initialize();
    }

    PyGILState_STATE gil = PyGILState_Ensure();

    // Import path
    PyObject* sys = PyImport_ImportModule("sys");
    PyObject* path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyUnicode_FromString("/home/arturo/src/argos3/build_simulator/Collision_Free_CPFA/source/CPFA"));
    PyList_Append(path, PyUnicode_FromString("/home/arturo/venvs/rl/lib/python3.10/site-packages"));
    Py_DECREF(path);
    Py_DECREF(sys);

    // Import module
    PyObject* pName = PyUnicode_FromString("rltrainer");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        std::cerr << "[ERROR] Failed to import Python module 'rltrainer'\n";
        PyGILState_Release(gil);
        return;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, "save_models");
    if (!pFunc || !PyCallable_Check(pFunc)) {
        std::cerr << "[ERROR] Python function 'save_models' not found or not callable\n";
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        PyGILState_Release(gil);
        return;
    }

    // Call Python function
    PyObject* pReturn = PyObject_CallObject(pFunc, NULL);

    if (!pReturn) {
        PyErr_Print();
        std::cerr << "[ERROR] Python function 'save_models' call failed\n";
    } else {
        std::cout << "[INFO] Python function 'save_models' executed successfully\n";
        Py_DECREF(pReturn);
    }

    // Cleanup
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
    PyGILState_Release(gil);
}

REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
