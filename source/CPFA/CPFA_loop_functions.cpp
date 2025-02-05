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
    
	m_collision_threshold = 0.1f;
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
	// SetupPythonEnvironment();
 
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


// Euclidean distance between two points
double CPFA_loop_functions::euclideanDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Calculate sigmoid function
double CPFA_loop_functions::sigmoid(double z) {
    return 1.0 / (1.0 + exp(-z));
}

// Function to calculate features and predict congestion
bool CPFA_loop_functions::predictCongestion(size_t start_index, size_t end_index, const std::vector<argos::CVector2>& coordinates) {
    // Validate indices
    if (start_index >= end_index || end_index >= coordinates.size() || coordinates.size() < 50) {
        std::cerr << "Error: Invalid indices or insufficient coordinates (expected at least 50).\n";
        return false;
    }
    // Load model parameters from JSON
    std::ifstream file("/home/arturo/src/argos3/build_simulator/Collision_Free_CPFA/source/CPFA/logistic_model.json");
    if (!file.is_open()) {
        std::cerr << "Error: Could not open logistic_model.json.\n";
        return false;
    }
    Json::Value modelParams;
    file >> modelParams;

	// if (!modelParams["intercept"].isDouble()) {
	// 	std::cerr << "Error: 'intercept' is not a double. Value: " << modelParams["intercept"] << std::endl;
	// }
    // Extract coefficients
    double intercept = modelParams["intercept"][0].asDouble();
    double coef_start_index = modelParams["coefficients"][0].asDouble();
    double coef_end_index = modelParams["coefficients"][1].asDouble();
    double coef_ratio_distance = modelParams["coefficients"][2].asDouble();
    double coef_angle = modelParams["coefficients"][3].asDouble();

    // Calculate optimal distance based on a constant velocity (e.g., 0.08 units per step)
    double optimal_distance = 0.08 * (end_index - start_index);

    // Calculate start-to-end distance
    double start_to_end_distance = euclideanDistance(
        coordinates[start_index].GetX(), coordinates[start_index].GetY(),
        coordinates[end_index].GetX(), coordinates[end_index].GetY()
    );

    // Calculate ratio_distance
    double ratio_distance = (optimal_distance != 0) ? start_to_end_distance / optimal_distance : std::numeric_limits<double>::infinity();

    // Calculate angle using the angle calculator logic
    if (end_index - start_index < 3) {
        std::cerr << "Error: Insufficient points to calculate angles.\n";
        return false; // Need at least 3 points for angle calculation
    }

    size_t middle_index = (start_index + end_index) / 2;
    argos::CVector2 p1 = coordinates[start_index];
    argos::CVector2 p2 = coordinates[middle_index];
    argos::CVector2 p3 = coordinates[end_index];

    // Vectors
    double dx1 = p2.GetX() - p1.GetX();
    double dy1 = p2.GetY() - p1.GetY();
    double dx2 = p3.GetX() - p2.GetX();
    double dy2 = p3.GetY() - p2.GetY();

    // Dot product and magnitudes
    double dot_product = dx1 * dx2 + dy1 * dy2;
    double mag_v1 = sqrt(dx1 * dx1 + dy1 * dy1);
    double mag_v2 = sqrt(dx2 * dx2 + dy2 * dy2);

    double angle = 0.0;
    if (mag_v1 > 0 && mag_v2 > 0) {
        double cosine_angle = dot_product / (mag_v1 * mag_v2);
        cosine_angle = std::clamp(cosine_angle, -1.0, 1.0); // Clamp for stability
        angle = acos(cosine_angle) * (180.0 / M_PI); // Convert to degrees
    }

    // Logistic regression probability
    double z = intercept +
               (coef_start_index * static_cast<double>(start_index)) +
               (coef_end_index * static_cast<double>(end_index)) +
               (coef_ratio_distance * ratio_distance) +
               (coef_angle * angle);

    double probability = sigmoid(z); // Ensure sigmoid function is defined

    // Predict if the robot is congested
    return probability >= 0.5;
}

// drop resource function
void CPFA_loop_functions::dropResource(string robot_id) {
	argos::LOG << "Robot " << robot_id << " has dropped a resource due to congestion." << std::endl;
}


void CPFA_loop_functions::PostStep() {

	// get x,y coordinate of each robot
	argos::CVector2 position;
	vector<argos::CVector2> robotPosList2;
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	//dictionary that holds the robot id as a key and the trajectory of the robot that has dropped a resource up to the last 50 values.
	//std::map<std::string, std::vector<argos::CVector2>> dropped_trajectories;
	const size_t WINDOW_SIZE = 300;
	const size_t STEP_SIZE = 100;

	robotPosList.clear();
	//robotPosList2.clear();

	bool dropped_resource = false;
	//print timestep
	//argos::LOG << "timestep: " << GetSpace().GetSimulationClock() << std::endl;
	size_t counter_nest = 0;
	for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
	  argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
	  BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
	  CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
	  //get distance from position to nest(0,0)
	  position = c2.GetPosition();
	  double distance = sqrt(pow(position.GetX(), 2) + pow(position.GetY(), 2));
	  //if distance is less than 1 increment counter
	  if(distance < 5){
		counter_nest++;
	  }	
	}
	//collisions occured
	size_t collision_count = 0;
	for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
	  argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
	  BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
	  CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		const argos::CCI_FootBotProximitySensor::TReadings& tProxReads = c2.GetProximitySensorReadings();

		for(size_t i = 0; i < tProxReads.size(); ++i) {
			if(tProxReads[i].Value > m_collision_threshold) { // Define a threshold to consider it a collision
				// Handle collision
				//argos::LOG << "Collision detected for robot " << c2.GetId() << std::endl;
				collision_count++;
				break;
			}
		}

	  position = c2.GetPosition();
	  robotPosList[c2.GetId()] = position;

	  //robotPosList2.push_back(position);
	  robotPosList3[c2.GetId()].push_back(position);
		// if(c2.GetStatus() == "DROPPED"){
		// 	dropped_resource = true;
		// 	argos::LOG << "Robot " << c2.GetId() << " has dropped a resource" << std::endl;
		// 	std::vector<argos::CVector2> traj;
		// 	traj.assign(robotPosList3[c2.GetId()].end() - 100, robotPosList3[c2.GetId()].end());

		// 	dropped_trajectories[c2.GetId()].push_back(traj);
		// 	counter_nest_history.push_back(counter_nest);
		// }

		// if(c2.IsHoldingFood() == true){
			if(c2.GetStatus() == "FOUND"){
				// argos::LOG << "Robot " << c2.GetId() << " has found a resource" << std::endl;
				temp_trajectories[c2.GetId()].push_back(c2.GetPosition());
			}
			else if(c2.GetStatus() == "DROPPED"){
				dropped_resource = true;
				std::vector<argos::CVector2> traj;
				// argos::LOG << "Robot " << c2.GetId() << " has dropped a resource" << std::endl;			
				// for (const auto& pos : temp_trajectories[c2.GetId()]) {
				// 	argos::LOG << "(" << pos.GetX() << ", " << pos.GetY() << "), ";
				// }
				// argos::LOG << std::endl;
				traj = temp_trajectories[c2.GetId()];
				dropped_trajectories[c2.GetId()].push_back(traj);
				counter_nest_history.push_back(counter_nest);
				temp_trajectories.erase(c2.GetId()); // remove the trajectory from temp_trajectories
			}
			else {
				// If the robot is not in the "FOUND" or "DROPPED" state but temp_trajectories contains its ID,
				// it means the robot is moving towards the nest with a resource. Add its current position to the trajectory.
				if(temp_trajectories.count(c2.GetId()) > 0) {
					temp_trajectories[c2.GetId()].push_back(c2.GetPosition());
					size_t trajectory_size = temp_trajectories[c2.GetId()].size();
					// Check congestion every x positions in the trajectory
					if (trajectory_size >= WINDOW_SIZE && (trajectory_size - WINDOW_SIZE) % STEP_SIZE == 0) {
						// Determine the start and end indices for the current window
						size_t start_index = trajectory_size - WINDOW_SIZE;
						size_t end_index = trajectory_size - 1;

						// Call predictCongestion with the windowed trajectory
						bool drop = predictCongestion(start_index, end_index, temp_trajectories[c2.GetId()]);
						c2.SetCongestion(drop);
						// debugging
						if (drop) {
							argos::LOG << "Robot " << c2.GetId() << " is congested with indexes: " << start_index << " - " << end_index << std::endl;
							argos::LOG << "Trajectory Size: " << trajectory_size << std::endl;
						}
					}
				}
			}
		
	}
	if (dropped_resource) {
		collision_history.push_back(collision_count);
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
                  
    if (PrintFinalScore == 1) {
        string type="";
        if (FoodDistribution == 0) type = "random";
        else if (FoodDistribution == 1) type = "cluster";
        else type = "powerlaw";
            
        ostringstream num_tag;
        num_tag << FoodItemCount; 
              
        ostringstream num_robots;
        num_robots <<  Num_robots;
   
        ostringstream arena_width;
        arena_width << ArenaWidth;
        
        ostringstream quardArena;
        if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
             quardArena << 1;
         }
         else{
             quardArena << 0;
        }
        
        string header = "./results/"+ type+"_CPFA_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_" + quardArena.str() +"_";
       
        unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
       
        /* Real total_travel_time=0;
        Real total_search_time=0;
        ofstream travelSearchTimeDataOutput((header+"TravelSearchTimeData.txt").c_str(), ios::app);
        */
        
        
        argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
        for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
            argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
            BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
            CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
            CollisionTime += c2.GetCollisionTime();
            
            /*if(c2.GetStatus() == "SEARCHING"){
                total_search_time += SimTime-c2.GetTravelingTime();
                total_travel_time += c2.GetTravelingTime();
	    }
            else {
		total_search_time += c2.GetSearchingTime();
		total_travel_time += SimTime-c2.GetSearchingTime();
            } */        
        }
        //travelSearchTimeDataOutput<< total_travel_time/ticks_per_second<<", "<<total_search_time/ticks_per_second<<endl;
        //travelSearchTimeDataOutput.close();   
             
        ofstream dataOutput( (header+ "iAntTagDa.txt").c_str(), ios::app);
        // output to file
        if(dataOutput.tellp() == 0) {
            dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
        }
    
        //dataOutput <<data.CollisionTime/16.0<<", "<< time_in_minutes << ", " << data.RandomSeed << endl;
        //dataOutput << Score() << ", "<<(CollisionTime-16*Score())/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        dataOutput.close();

		/*
        ofstream densityOutput( ("./results/densities.txt"), ios::app);
        densityOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
        densityOutput.close();
		*/
        ofstream forageDataOutput((header+"ForageData.txt").c_str(), ios::app);
        if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
        for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
        forageDataOutput<<"\n";
        forageDataOutput.close();
        
        ofstream trajOutput( (header+ "iAntTrajData.txt").c_str(), ios::app);
        // output to file
        //if(trajOutput.tellp() == 0) {
            trajOutput << "trajs\n";//qilu 11/2023
        //}
        
        for(map<string, std::vector<CVector2>>::iterator it= Trajectory.begin(); it!= Trajectory.end(); ++it) {
			
			for(size_t j = 0; j < it->second.size(); j++) {
				trajOutput << it->second[j]<<"; ";
			}
			trajOutput << "\n";
		
		}
        
		trajOutput.close();

		std::ofstream droppedtrajOutput((header + "iAntDroppedTrajData.txt").c_str(), std::ios::app);

		// Check if the file was successfully opened
		if (!trajOutput) {
			std::cerr << "Error opening file for writing dropped trajectories." << std::endl;
			return;
		}

		// Write a header or label for the data (if required)
		droppedtrajOutput << "Dropped Trajectories\n";

		// Iterate through each robot's dropped trajectories
		for (std::map<std::string, std::vector<std::vector<argos::CVector2>>>::iterator it = dropped_trajectories.begin(); it != dropped_trajectories.end(); ++it) {
			const std::string& robotId = it->first;  // Robot ID
			const std::vector<std::vector<argos::CVector2>>& trajectories = it->second;  // Vector of trajectories

			// Iterate through each trajectory for the current robot
			for (size_t i = 0; i < trajectories.size(); ++i) {
				const std::vector<argos::CVector2>& trajectory = trajectories[i];

				// Write the robot ID and trajectory index (optional for clarity)
				droppedtrajOutput << "Robot: " << robotId << ", Nest Counter: " << counter_nest_history[i] << ", Collision Counter: " << collision_history[i] <<", Trajectory " << i + 1 << ":\n";

				// Iterate through the positions in the trajectory
				for (size_t j = 0; j < trajectory.size(); ++j) {
					droppedtrajOutput << trajectory[j] << "; ";  // Write each position (CVector2)
				}
				droppedtrajOutput << "\n";  // Newline after each trajectory
			}
		}
		
		// droppedtrajOutput << "\nRobots near the nest at each timestep:\n";

		// for (size_t i = 0; i < counter_nest_history.size(); ++i) {
		// 	droppedtrajOutput << "Timestep " << i << ": " << counter_nest_history[i] << "\n";
		// }

		// Close the file after writing
		droppedtrajOutput.close();       
      }  

	// get food collected for each robot at each timestep
	ofstream foodOutput( "./results/foodData.txt", ios::app);
	if(foodOutput.tellp() == 0) {
	   foodOutput << "food_collected\n";
	   //foodOutput << CollectedFoodList.size() << endl;
	}
	for(size_t i = 0; i < CollectedFoodList.size(); i++) {
	   foodOutput << CollectedFoodList[i] << ", ";
	}
	foodOutput << endl;
	foodOutput.close();


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


REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
