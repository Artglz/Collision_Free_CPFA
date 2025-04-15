#include "CPFA_controller.h"
#include <unistd.h>
#include <fstream>

//#include <argos3/core/utility/logging/argos_log.h>




CPFA_controller::CPFA_controller() :
	RNG(argos::CRandom::CreateRNG("argos")),
	isInformed(false),
	isHoldingFood(false),
	isUsingSiteFidelity(false),
	isGivingUpSearch(false),
	ResourceDensity(0),
	MaxTrailSize(50),
	SearchTime(0),
	CPFA_state(DEPARTING),
	LoopFunctions(NULL),
	survey_count(0),
	m_pcWheels(NULL),
	isUsingPheromone(0),
    SiteFidelityPosition(1000, 1000), 
        searchingTime(0),
        travelingTime(0),
        startTime(0),
    m_pcLEDs(NULL),
    TrailColor(CColor::BLUE),
        updateFidelity(false),
        last_time_in_seconds(0)
{
}

void CPFA_controller::Init(argos::TConfigurationNode &node) {
	compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
	wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
	argos::TConfigurationNode settings = argos::GetNode(node, "settings");

	argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
	argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestDistanceTolerance", NestDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestAngleTolerance",    NestAngleTolerance);
	argos::GetNodeAttribute(settings, "TargetAngleTolerance",    TargetAngleTolerance);
	argos::GetNodeAttribute(settings, "SearchStepSize",          SearchStepSize);
	argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
	argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);
	argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      results_path);
	argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
	argos::GetNodeAttribute(settings, "PositionNoiseStdev",      PositionNoiseStdev);
	m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering"); // adding wheels
	argos::CVector2 p(GetPosition());
	SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
	
	FoodDistanceTolerance *= FoodDistanceTolerance;
	SetIsHeadingToNest(true);
	//qilu 10/21/2016 Let robots start to search immediately
	SetTarget(p);
        controllerID= GetId();
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
    controllerID= GetId();//qilu 07/26/2016
		m_pcLEDs->SetAllColors(CColor::GREEN);
}

void CPFA_controller::ControlStep() {
	/*
	ofstream log_output_stream;
	log_output_stream.open("cpfa_log.txt", ios::app);

	// depart from nest after food drop off or simulation start
	if (isHoldingFood) log_output_stream << "(Carrying) ";
	
	switch(CPFA_state)  {
		case DEPARTING:
			if (isUsingSiteFidelity) {
				log_output_stream << "DEPARTING (Fidelity): "
					<< GetTarget().GetX() << ", " << GetTarget().GetY()
					<< endl;
			} else if (isInformed) {
				log_output_stream << "DEPARTING (Waypoint): "
				<< GetTarget().GetX() << ", " << GetTarget().GetY() << endl;
			} else {
				log_output_stream << "DEPARTING (Searching): "
				<< GetTarget().GetX() << ", " << GetTarget().GetY() << endl;
			}
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			if (isInformed) log_output_stream << "SEARCHING: Informed" << endl;     
			else log_output_stream << "SEARCHING: UnInformed" << endl;
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			log_output_stream << "RETURNING" << endl;
			break;
		case SURVEYING:
			log_output_stream << "SURVEYING" << endl;
			break;
		default:
			log_output_stream << "Unknown state" << endl;
	}
	*/

	// Add line so we can draw the trail
	curr_time_in_seconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond()); 
     
	if(curr_time_in_seconds - last_time_in_seconds >= 0)
	{
		CVector2 position2d(GetPosition().GetX(), GetPosition().GetY());
		
		CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
		CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
		CRay3 targetRay(target3d, position3d);
		myTrail.push_back(targetRay);
		LoopFunctions->Trajectory[controllerID].push_back(position2d);
		//since it costs a lot of memeory, I commented it. qilu 06/2023. You can uncomment it if you want to show the trails.
		LoopFunctions->TargetRayList.push_back(targetRay);
		LoopFunctions->TargetRayColorList.push_back(TrailColor);
		//argos::LOG<< "TargetRayList size =" << LoopFunctions->TargetRayList.size() <<endl;
		previous_position = GetPosition();
		last_time_in_seconds = curr_time_in_seconds;
     }
	//UpdateTargetRayList();
	CPFA();
	Move();
}

void CPFA_controller::Reset() {
 num_targets_collected =0;
 isHoldingFood   = false;
    isInformed      = false;
    SearchTime      = 0;
    ResourceDensity = 0;
    RobotDensity = 0;
    collisionDelay = 0;
    
  	LoopFunctions->CollisionTime=0; //qilu 09/26/2016
    
    
    /* Set LED color */
    /* m_pcLEDs->SetAllColors(CColor::BLACK); //qilu 09/04 */
    SetTarget(LoopFunctions->NestPosition); //qilu 09/08
    updateFidelity = false;
    TrailToShare.clear();
    TrailToFollow.clear();
    MyTrail.clear();

	myTrail.clear();

	isInformed = false;
	isHoldingFood = false;
	isUsingSiteFidelity = false;
	isGivingUpSearch = false;
}

bool CPFA_controller::IsHoldingFood() {
		return isHoldingFood;
}

bool CPFA_controller::IsUsingSiteFidelity() {
		return isUsingSiteFidelity;
}

void CPFA_controller::CPFA() {
	
	switch(CPFA_state) {
		// depart from nest after food drop off or simulation start
		case DEPARTING:
			//argos::LOG << "DEPARTING" << std::endl;
			//SetIsHeadingToNest(false);
			Departing();
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING: 
			//argos::LOG << "SEARCHING" << std::endl;
			//SetIsHeadingToNest(false);
			if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
				Searching();
			}
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			//argos::LOG << "RETURNING" << std::endl;
			//SetIsHeadingToNest(true);
			Returning();
			break;
		case SURVEYING:
			//argos::LOG << "SURVEYING" << std::endl;
			//SetIsHeadingToNest(false);
			Surveying();
			break;
		case FOLLOWING_ENTRY_PATH:
			//argos::LOG << "FOLLOWING_ENTRY_PATH" << std::endl;
			FollowingEntryPath();
			break;
		case FOLLOWING_EXIT_PATH:
			//argos::LOG << "FOLLOWING_EXIT_PATH" << std::endl;
			FollowingExitPath();
			break;
	}
}

bool CPFA_controller::IsInTheNest() {
    
	return ((GetPosition() - LoopFunctions->NestPosition).SquareLength()
		< LoopFunctions->NestRadiusSquared);
}

bool CPFA_controller::IsInRestrictedZone(argos::CVector2 position) {

	// get the nest position and check if the robot is in the restricted zone
	argos::CVector2 RestrictedZoneCenter = LoopFunctions->NestPosition;
    return (position - RestrictedZoneCenter).SquareLength() < (RestrictedZoneRadius * RestrictedZoneRadius);
}

void CPFA_controller::setZoneActive(bool value) {
	isZoneActive = value;
}


bool CPFA_controller::CollisionDetection() {
	//log current CPFA State
	argos::CVector2 collisionVector = GetCollisionVector();
	argos::Real collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
	bool isCollisionDetected = false;


	// Here we are checking if a robot that is not on the path is in the restricted zone
	// If it is, we want to turn away from the restricted zone
	// static size_t lastRestrictedZoneCheck = 0;
	// size_t currentTick = SimulationTick();

	// if (currentTick - lastRestrictedZoneCheck >= 50 * SimulationTicksPerSecond()) {
	// 	if (IsInRestrictedZone(GetPosition()) && (GetStatus() != "FOLLOWING_ENTRY_PATH" && GetStatus() != "FOLLOWING_EXIT_PATH") && isZoneActive) {
	// 		Stop();
	// 		isCollisionDetected = true;
	// 		collision_counter++;
	// 		while (MovementStack.size() > 0) MovementStack.pop();

	// 		PushMovement(FORWARD, SearchStepSize);

	// 		Real randomNumber = RNG->Uniform(CRange<Real>(0.5, 1.0));
	// 		collisionDelay = currentTick + (size_t)(randomNumber * SimulationTicksPerSecond());

	// 		// **Compute detour direction**
	// 		argos::CVector2 toCenter = LoopFunctions->NestPosition - GetPosition(); // Vector pointing to the center
	// 		argos::CRadians heading = GetHeading(); // Robots current heading
	// 		argos::CRadians toCenterAngle = toCenter.Angle(); // Angle toward the center

	// 		// Compute the difference between robot's heading and the center of the restricted zone
	// 		argos::CRadians angleDifference = toCenterAngle - heading;

	// 		//argos::LOG << "Robot " << GetId() << " is in the restricted zone." << std::endl;
	// 		if (angleDifference.GetValue() > 0) {
	// 			SetRightTurn(angleDifference.GetValue()); // Turn right away from the restricted zone
	// 			argos::LOG << "Robot " << GetId() << " turning RIGHT to avoid restricted zone with angle: " << angleDifference.GetValue() << std::endl;
	// 		} else {
	// 			SetLeftTurn(angleDifference.GetValue()); // Turn left away from the restricted zone
	// 			argos::LOG << "Robot " << GetId() << " turning LEFT to avoid restricted zone with angle: " << angleDifference.GetValue() << std::endl;
	// 		}

	// 		lastRestrictedZoneCheck = currentTick;
	// 		return isCollisionDetected;
	// 	}
	// }
	
	// this is the normal collision logic where a robot determines a where it is colliding with another robot and the turn it must take.
	if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
		 && collisionVector.Length() > 0.0) {

		// if a robot is following a path, we dont want to avoid collisions since paths dont overlap. So there will be no
		// collisions as longs other robots don't interfere.
		isCollisionDetected = true;
		collision_counter++;
		if(GetStatus() == "FOLLOWING_ENTRY_PATH") {
			// argos::Real randomDecision = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
			// if (randomDecision < 0.05) { // 50% chance to stop
			// 	Real randomStopTime = RNG->Uniform(CRange<Real>(0.5, 1.0)); // Random stop time between 0.5 and 2 seconds
			// 	collisionDelay = SimulationTick() + (size_t)(randomStopTime * SimulationTicksPerSecond());
			// 	Stop();
			// }
			return isCollisionDetected;
		}	 
		Stop();

   
		while(MovementStack.size() > 0) MovementStack.pop();

		PushMovement(FORWARD, SearchStepSize);

		Real randomNumber = RNG->Uniform(CRange<Real>(0.5, 1.0));
        collisionDelay = SimulationTick() + (size_t)(randomNumber*SimulationTicksPerSecond());//qilu 10/26/2016	

		if(collisionAngle <= 0.0)  {
			//argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;
			SetLeftTurn(collisionAngle); //qilu 09/24/2016
		} else {
			//argos::LOG << collisionAngle << std::endl << collisionVector << std::endl << std::endl;
			SetRightTurn(collisionAngle); //qilu 09/24/2016
		}

	}

	return isCollisionDetected;
}

int CPFA_controller::FindClosestPointOnPath(argos::CVector2 robotPos, std::vector<argos::CVector2> path) {
    int closestPoint = 0;
    argos::Real minDistance = (robotPos - path[0]).SquareLength();

    for (size_t i = 1; i < path.size(); i++) {
        argos::Real distance = (robotPos - path[i]).SquareLength();
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = i;
        }
    }
    return closestPoint;
}
//make a function 

void CPFA_controller::SetCongestion(bool value){
	// if num == 0{
	// 	isCongested = false;
	// }
	// else{
	// 	isCongested = true;
	// }
	isCongested = value;
}

bool CPFA_controller::IsInCongestion(){
	return isCongested;
}

void CPFA_controller::SetLoopFunctions(CPFA_loop_functions* lf) {
	LoopFunctions = lf;

	// Initialize the SiteFidelityPosition

	// Create the output file here because it needs LoopFunctions
		
	// Name the results file with the current time and date
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	stringstream ss;

	char hostname[1024];                                                   
	hostname[1023] = '\0';                    
	gethostname(hostname, 1023);  

	/* ss << "CPFA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-"
		<< hostname << '-'
		<< getpid() << '-'
		<< (now->tm_year) << '-'
		<< (now->tm_mon + 1) << '-'
		<<  now->tm_mday << '-'
		<<  now->tm_hour << '-'
		<<  now->tm_min << '-'
		<<  now->tm_sec << ".csv";

		string results_file_name = ss.str();
		results_full_path = results_path+"/"+results_file_name;
         */
         
	// Only the first robot should do this:	 
	if (GetId().compare("CPFA_0") == 0) {
		/*
		ofstream results_output_stream;
		results_output_stream.open(results_full_path, ios::app);
		results_output_stream << "NumberOfRobots, "
			<< "TargetDistanceTolerance, "
			<< "TargetAngleTolerance, "
			<< "FoodDistanceTolerance, "
			<< "RobotForwardSpeed, "
			<< "RobotRotationSpeed, "
			<< "RandomSeed, "
			<< "ProbabilityOfSwitchingToSearching, "
			<< "ProbabilityOfReturningToNest, "
			<< "UninformedSearchVariation, "   
			<< "RateOfInformedSearchDecay, "   
			<< "RateOfSiteFidelity, "          
			<< "RateOfLayingPheromone, "       
			<< "RateOfPheromoneDecay" << endl
			<< LoopFunctions->getNumberOfRobots() << ", "
			<< CSimulator::GetInstance().GetRandomSeed() << ", "  
			<< TargetDistanceTolerance << ", "
			<< TargetAngleTolerance << ", "
			<< FoodDistanceTolerance << ", "
			<< RobotForwardSpeed << ", "
			<< RobotRotationSpeed << ", "
			<< LoopFunctions->getProbabilityOfSwitchingToSearching() << ", "
			<< LoopFunctions->getProbabilityOfReturningToNest() << ", "
			<< LoopFunctions->getUninformedSearchVariation() << ", "
			<< LoopFunctions->getRateOfInformedSearchDecay() << ", "
			<< LoopFunctions->getRateOfSiteFidelity() << ", "
			<< LoopFunctions->getRateOfLayingPheromone() << ", "
			<< LoopFunctions->getRateOfPheromoneDecay()
			<< endl;
				
			results_output_stream.close();
		*/
	}

}

// Trying to modfiy state to prioritize dropped resources around the nest.
void CPFA_controller::Departing()
{
     //LOG<<"Departing..."<<endl;
    argos::Real distanceToTarget = (GetPosition() - GetTarget()).Length();
    argos::Real randomNumber = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

	/* When not informed, continue to travel until randomly switching to the searching state. */
    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
       if(isInformed == false){
           if(SimulationTick()%(5*SimulationTicksPerSecond())==0 && randomNumber < LoopFunctions->ProbabilityOfSwitchingToSearching){
			 //LOG<<"Switch to search..."<<endl;
                 Stop();
                 SearchTime = 0;
                 CPFA_state = SEARCHING;
                 travelingTime+=SimulationTick()-startTime;//qilu 10/22
                 startTime = SimulationTick();//qilu 10/22
            
                 argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
                 argos::Real rand = RNG->Gaussian(USV);
                 argos::CRadians rotation(rand);
                 argos::CRadians angle1(rotation.UnsignedNormalize());
                 argos::CRadians angle2(GetHeading().UnsignedNormalize());
                 argos::CRadians turn_angle(angle1 + angle2);
                 argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                 SetIsHeadingToNest(false);
                 SetTarget(turn_vector + GetPosition());
		   }
		   else if(distanceToTarget < TargetDistanceTolerance){
			 SetRandomSearchLocation();
		   }
	   }
	 } 
		 
     /* Are we informed? I.E. using site fidelity or pheromones. */	
     if(isInformed && distanceToTarget < TargetDistanceTolerance) {
          SearchTime = 0;
          CPFA_state = SEARCHING;
          travelingTime+=SimulationTick()-startTime;//qilu 10/22
          startTime = SimulationTick();//qilu 10/22

          if(isUsingSiteFidelity) {
               isUsingSiteFidelity = false;
               SetFidelityList();
          }
     }
     else{ // based on the density of robots, decide to do random search
		 if(isInformed == true && SimulationTick()% SimulationTicksPerSecond() ==0 ){
			 //LOG<<"Departing..."<<endl;
			
	      }
	 }


}

void CPFA_controller::FollowingEntryPath() {
	
	// Log id and position where robot is heading
	//argos::LOG << "Robot ID: " << controllerID << " is heading to: " << GetTarget() << std::endl;

	if (IsInTheNest()) {
		//argos::LOG << "Executed " << currentWaypointIndex << " out of " << EntryPath.size() << " Waypoints" << std::endl;
		if (isHoldingFood) {
			num_targets_collected++;
			LoopFunctions->currNumCollectedFood++;
			LoopFunctions->setScore(num_targets_collected);
		}

		//Decide next search strategy (pheromone/site fidelity/random)
		if (updateFidelity && GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity) > RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0))) {
		    SetIsHeadingToNest(false);
		    SetTarget(SiteFidelityPosition);
		    isInformed = true;
		}
		else if (SetTargetPheromone()) {
		    isInformed = true;
		    isUsingSiteFidelity = false;
		}
		else {
		    SetRandomSearchLocation();
		    isInformed = false;
		    isUsingSiteFidelity = false;
		}

		// Choose the best intermediate step (closest of 4-adjacent)
		mainTarget = GetTarget();
		argos::CVector2 currentPos = GetPosition();

		std::vector<argos::CVector2> candidates = {
			argos::CVector2(currentPos.GetX() + 1.0, currentPos.GetY()),
			argos::CVector2(currentPos.GetX() - 1.0, currentPos.GetY()),
			argos::CVector2(currentPos.GetX(), currentPos.GetY() + 1.0),
			argos::CVector2(currentPos.GetX(), currentPos.GetY() - 1.0)
		};

		argos::CVector2 bestStep = candidates[0];
		Real bestDistance = (mainTarget - bestStep).Length();

		for (size_t i = 1; i < candidates.size(); ++i) {
			Real dist = (mainTarget - candidates[i]).Length();
			if (dist < bestDistance) {
				bestDistance = dist;
				bestStep = candidates[i];
			}
		}

		// Set intermediate target first
		SetTarget(bestStep);
		CPFA_state = FOLLOWING_EXIT_PATH;
		isGivingUpSearch = false;
		isHoldingFood = false;
		travelingTime += SimulationTick() - startTime;
		startTime = SimulationTick();

		return;
	}

	if (IsAtTarget()) {
		// argos::LOG << "Robot " << GetId() << " is at waypoint " << currentWaypointIndex << std::endl;
		currentWaypointIndex++;
		// argos::LOG << "Next target is: " << EntryPath[currentWaypointIndex] << std::endl;
		SetTarget(EntryPath[currentWaypointIndex]);
		
	}
	
	// argos::LOG << GetId() <<  " - Current target is " << GetTarget() << std::endl;
	//argos::LOG << "IN entry path..." << std::endl;
}

void CPFA_controller::FollowingExitPath() {
    
    // if (IsAtTarget() && exitPathIndex < ExitPath.size()) { //will keep following the path by setting each waypoint in the spiral as target
    //     SetTarget(ExitPath[exitPathIndex]);
    //     exitPathIndex++;
    // } 
	// else if (exitPathIndex >= ExitPath.size()) { // Once that is done, we will check if the robot will do site fidelity, pheromone trail, or random search
	// 	if (updateFidelity && GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity) > RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0))) {
    //         SetIsHeadingToNest(false);
    //         SetTarget(SiteFidelityPosition); // Use site fidelity
    //         isInformed = true;
    //     }
    //     else if (SetTargetPheromone()) {
    //         isInformed = true;
    //         isUsingSiteFidelity = false; // Follow pheromone waypoints
    //     }
    //     else {
    //         SetRandomSearchLocation(); // Perform a random search
    //         isInformed = false;
    //         isUsingSiteFidelity = false;
    //     }

	// 	argos::LOG << "done with exit path... depating" << std::endl;
    //     // Update robot state
    //     isGivingUpSearch = false;
	// 	CPFA_state = DEPARTING;
    //     isHoldingFood = false;
    //     travelingTime += SimulationTick() - startTime;
    //     startTime = SimulationTick();
    // }
	//argos::LOG << GetId() << " is going to " << GetTarget() << std::endl;
	if (IsAtTarget()) {
		//argos::LOG << "Reached intermediate target on exit path..." << std::endl;
        SetTarget(mainTarget);
        CPFA_state = DEPARTING;  // Or your default movement/search state
    }
}


void CPFA_controller::Searching() {
 //LOG<<"Searching..."<<endl;
	// "scan" for food only every half of a second
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		SetHoldingFood();
	}
	// When not carrying food, calculate movement.
	if(IsHoldingFood() == false) {
		   argos::CVector2 distance = GetPosition() - GetTarget();
		   argos::Real     random   = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
     
       // If we reached our target search location, set a new one. The 
       // new search location calculation is different based on whether
       // we are currently using informed or uninformed search.
       if(distance.SquareLength() < TargetDistanceTolerance) {
         // randomly give up searching
         if(SimulationTick()% (5*SimulationTicksPerSecond())==0 && random < LoopFunctions->ProbabilityOfReturningToNest) {
             
             SetFidelityList();
	         TrailToShare.clear();
             SetIsHeadingToNest(true);
             SetTarget(LoopFunctions->NestPosition);
             isGivingUpSearch = true;
	         LoopFunctions->FidelityList.erase(controllerID);
             isUsingSiteFidelity = false; 
             updateFidelity = false; 
			 CPFA_state = RETURNING;
             //CPFA_state = GAVE_UP;
             searchingTime+=SimulationTick()-startTime;
             startTime = SimulationTick();

             /*
             ofstream log_output_stream;
             log_output_stream.open("giveup.txt", ios::app);
             log_output_stream << "Give up: " << SimulationTick() / SimulationTicksPerSecond() << endl;
             log_output_stream.close();
             */
     
             return; 
             
         }
         argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
         argos::Real rand = RNG->Gaussian(USCV);

         // uninformed search
         if(isInformed == false) {
          argos::CRadians rotation(rand);
          argos::CRadians angle1(rotation);
          argos::CRadians angle2(GetHeading());
          argos::CRadians turn_angle(angle1 + angle2);
          argos::CVector2 turn_vector(SearchStepSize, turn_angle);
      
          //argos::LOG << "UNINFORMED SEARCH: rotation: " << angle1 << std::endl;
          //argos::LOG << "UNINFORMED SEARCH: old heading: " << angle2 << std::endl;
      
          /*
          ofstream log_output_stream;
          log_output_stream.open("uninformed_angle1.log", ios::app);
          log_output_stream << angle1.GetValue() << endl;
          log_output_stream.close();
      
          log_output_stream.open("uninformed_angle2.log", ios::app);
          log_output_stream << angle2.GetValue() << endl;
          log_output_stream.close();
      
          log_output_stream.open("uninformed_turning_angle.log", ios::app);
          log_output_stream << turn_angle.GetValue() << endl;
          log_output_stream.close();
          */
          SetIsHeadingToNest(false);
          SetTarget(turn_vector + GetPosition());
         }
         // informed search
         else{
          
              SetIsHeadingToNest(false);
              
              if(IsAtTarget()) {
                  size_t          t           = SearchTime++;
                  argos::Real     twoPi       = (argos::CRadians::TWO_PI).GetValue();
                  argos::Real     pi          = (argos::CRadians::PI).GetValue();
                  argos::Real     isd         = LoopFunctions->RateOfInformedSearchDecay;
	                  /*argos::Real     correlation = GetExponentialDecay((2.0 * twoPi) - LoopFunctions->UninformedSearchVariation.GetValue(), t, isd);
	                  argos::Real     rand = RNG->Gaussian(correlation + LoopFunctions->UninformedSearchVariation.GetValue());
	                       */ //qilu 09/24/2016
	                  Real correlation = GetExponentialDecay(rand, t, isd);
	                  //argos::CRadians rotation(GetBound(rand, -pi, pi));
	                  argos::CRadians rotation(GetBound(correlation, -pi, pi));//qilu 09/24/2016
                  argos::CRadians angle1(rotation);
                  argos::CRadians angle2(GetHeading());
                  argos::CRadians turn_angle(angle2 + angle1);
                  argos::CVector2 turn_vector(SearchStepSize, turn_angle);
          
                  //argos::LOG << "INFORMED SEARCH: rotation: " << angle1 << std::endl;
                  //argos::LOG << "INFORMED SEARCH: old heading: " << angle2 << std::endl;
          
                  /*
                  ofstream log_output_stream;
                  log_output_stream.open("informed_angle1.log", ios::app);
                  log_output_stream << angle1.GetValue() << endl;
                  log_output_stream.close();
          
                  log_output_stream.open("informed_angle2.log", ios::app);
                  log_output_stream << angle2.GetValue() << endl;
                  log_output_stream.close();
          
                  log_output_stream.open("informed_turning_angle.log", ios::app);
                  log_output_stream << turn_angle.GetValue() << endl;
                  log_output_stream.close();
                  */
                  SetTarget(turn_vector + GetPosition());
              }
         }
	  } //not reach the target location
	  else {
			 //argos::LOG << "SEARCH: Haven't reached destination. " << GetPosition() << "," << GetTarget() << std::endl;
		
			 
	  }
    }
	else {
		   //argos::LOG << "SEARCH: Carrying food." << std::endl;
	}

	// Food has been found, change state to RETURNING and go to the nest
	//else {
	//	SetTarget(LoopFunctions->NestPosition);
	//	CPFA_state = RETURNING;
	//}
}

// Cause the robot to rotate in place as if surveying the surrounding targets
// Turns 36 times by 10 degrees
void CPFA_controller::Surveying() {
 //LOG<<"Surveying..."<<endl;
	if (survey_count <= 4) { 
		CRadians rotation(survey_count*3.14/2); // divide by 10 so the vector is small and the linear motion is minimized
		argos::CVector2 turn_vector(SearchStepSize, rotation.SignedNormalize());
		
		SetIsHeadingToNest(true); // Turn off error for this
		SetTarget(turn_vector + GetPosition());
		
		if(fabs((GetHeading() - rotation).SignedNormalize().GetValue()) < TargetAngleTolerance.GetValue()) survey_count++;
			//else Keep trying to reach the turning angle
	}
	// Set the survey countdown
	else {
		SetIsHeadingToNest(false); // Turn on error for this
		SetTarget(LoopFunctions->NestPosition); 
		CPFA_state = RETURNING;
		survey_count = 0; // Reset
        searchingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22
            
	}
}

// void CPFA_controller::Returning() {
	

//     // Check if the robot is either in the nest or in a congested area
//     if (IsInTheNest()) {
// 		returning_trajectory.clear();
//         // Handle normal nest drop logic
//         if (isHoldingFood) {
//             num_targets_collected++; // Increment collected resource count
//             LoopFunctions->currNumCollectedFood++; // Update current collected food count
//             LoopFunctions->setScore(num_targets_collected); // Update the score
// 			//argos::LOG << "Resource collected by robot " << GetId() << " at tick " << SimulationTick() << std::endl;
//             // Determine if pheromone should be placed
//             argos::Real poissonCDF_pLayRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
//             argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
//             if (poissonCDF_pLayRate > r1 && updateFidelity) {
//                 TrailToShare.push_back(LoopFunctions->NestPosition); // Add nest position to trail
//                 argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
//                 Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
//                 LoopFunctions->PheromoneList.push_back(sharedPheromone); // Add pheromone to the list
//                 sharedPheromone.Deactivate(); // Ensure it won't get re-added later
//             }
//             TrailToShare.clear();
//         }else{
// 			//argos::LOG << "Robot " << GetId() << " reached the nest with no food" << std::endl;
// 		}

//         // Decide next task: Site fidelity, pheromones, or random search
//         if (updateFidelity && GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity) > RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0))) {
//             SetIsHeadingToNest(false);
//             SetTarget(SiteFidelityPosition); // Use site fidelity
//             isInformed = true;
//         }
//         else if (SetTargetPheromone()) {
//             isInformed = true;
//             isUsingSiteFidelity = false; // Follow pheromone waypoints
//         }
//         else {
//             SetRandomSearchLocation(); // Perform a random search
//             isInformed = false;
//             isUsingSiteFidelity = false;
//         }

//         // Update robot state
//         isGivingUpSearch = false;
//         // CPFA_state = DROPPED;
// 		CPFA_state = DEPARTING;
//         isHoldingFood = false;
//         travelingTime += SimulationTick() - startTime;
//         startTime = SimulationTick();
//     }
//     // else if (IsInCongestion()) {
//     //    // Handle congestion-specific drop logic
// 	//    returning_trajectory.clear();
//     //     if (isHoldingFood) {			

// 	// 		// Find the closest segment on the spiral path
// 	// 		currentWaypointIndex = FindClosestPointOnPath(GetPosition(), EntryPath);	
// 	// 		// Set target to the closest point on the path and start following it
// 	// 		argos::LOG << "Robot " << GetId() << " joining congestion path: " << EntryPath[currentWaypointIndex] <<std::endl;
// 	// 		CPFA_state = FOLLOWING_ENTRY_PATH;
// 	// 		SetTarget(EntryPath[currentWaypointIndex]);

// 	// 		isGivingUpSearch = false;
// 	// 		isCongested = false; // fixes problem where robot gets detected as congestion even thought it is not
// 	// 		travelingTime += SimulationTick() - startTime;
// 	// 		// startTime = SimulationTick();

//     //     // } else{
// 	// 	// 	// If the robot is in congestion but has no food, resume searching and stop going to nest
// 	// 	// 	//argos::LOG << "Robot " << GetId() << " is in congestion but has no food! Resuming search." << std::endl;
// 	// 	// 	// argos::LOG << "Congested with no resource at: " <<  SimulationTick() << std::endl;
// 	// 	// 	CPFA_state = SEARCHING;
// 	// 	// 	isCongested = false;
// 	// 	// 	SetRandomSearchLocation();
// 	// 	}
//     // }
//     else {
//         // // If not in the nest or congestion, proceed towards the target

// 		// // check if returning_trajectory is empty
// 		// if (returning_trajectory.empty()) {
// 		// 	// Handle the case where the trajectory is empty
// 		// 	// argos::LOG << "Returning trajectory is empty for robot " << GetId() << std::endl;
// 		// 	previous_ratio_distance_lag_1 = -1;
// 		// 	previous_ratio_distance = -1;
// 		// 	previous_angle_lag_1 = -1;
// 		// 	previous_angle = -1;
// 		// }
// 		// //Start logging the trajectory for congestion prediction
// 		// returning_trajectory.push_back(GetPosition());
// 		// if(returning_trajectory.size() >= WINDOW_SIZE && (returning_trajectory.size() - WINDOW_SIZE) % STEP_SIZE == 0){
// 		// 	//call predict congestion function
// 		// 	size_t start_index = returning_trajectory.size() - 150;
// 		// 	size_t end_index = returning_trajectory.size();		
// 		// 	size_t middle_index = (start_index + end_index) / 2;
// 		// 	std::vector<argos::CVector2> trajectory_segment(returning_trajectory.begin() + start_index, returning_trajectory.begin() + end_index);

// 		// 	// Calculate start-to-end distance
// 		// 	double start_to_end_distance = LoopFunctions->euclideanDistance(
// 		// 		returning_trajectory[start_index].GetX(), returning_trajectory[start_index].GetY(),
// 		// 		returning_trajectory[end_index].GetX(), returning_trajectory[end_index].GetY()
// 		// 	);

// 		// 	int indexes = start_index + end_index;

// 		// 	if(previous_ratio_distance_lag_1 != -1 && previous_angle_lag_1 != -1) {
// 		// 		// argos::LOG << "Robot " << GetId() << " is predicting congestion. " << start_index << " : " << end_index << std::endl;

// 		// 		bool drop = LoopFunctions->predictCongestion(indexes, trajectory_segment, 
// 		// 			previous_ratio_distance, previous_ratio_distance_lag_1, 
// 		// 			previous_angle, previous_angle_lag_1);
// 		// 		if(drop && (GetPosition() - LoopFunctions->NestPosition).Length() < 1.0){
// 		// 			SetCongestion(true);
// 		// 			//reset all variables
// 		// 			argos::LOG << "Robot " << GetId() << " dropped a resource. " << start_index << "-" << end_index << " at " << GetPosition() << std::endl;
// 		// 			previous_ratio_distance_lag_1 = -1;
// 		// 			previous_ratio_distance = -1;
// 		// 			previous_angle_lag_1 = -1;
// 		// 			previous_angle = -1;
// 		// 			returning_trajectory.clear();
// 		// 		}
				
// 		// 	}

// 		// 	previous_ratio_distance_lag_1 = previous_ratio_distance; // -1 , 20
// 		// 	previous_ratio_distance = start_to_end_distance / optimal_distance; // 20 , 22

// 		// 	previous_angle_lag_1 = previous_angle;
// 		// 	previous_angle = LoopFunctions->calculateAngle(returning_trajectory[start_index], returning_trajectory[middle_index], returning_trajectory[end_index]);		
// 		//  }
//         if (IsAtTarget()) {
//             // Perform random search adjustment if the target is reached
//             argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
//             argos::Real rand = RNG->Gaussian(USCV);

//             argos::CRadians rotation(rand);
//             argos::CRadians angle1(rotation);
//             argos::CRadians angle2(GetHeading());
//             argos::CRadians turn_angle(angle1 + angle2);
//             argos::CVector2 turn_vector(SearchStepSize, turn_angle);

//             SetIsHeadingToNest(false);
//             SetTarget(turn_vector + GetPosition());
//         }
//     }
// }

void CPFA_controller::Returning() {
	
	if(goingtoentry){
		if (IsAtTarget()){
			currentWaypointIndex += 1;
			SetTarget(EntryPath[currentWaypointIndex]);
			CPFA_state = FOLLOWING_ENTRY_PATH;
			goingtoentry = false;
			return;
		}
	}

	if (IsInTheNest()) {
		returning_trajectory.clear();
        // Handle normal nest drop logic
		argos::LOG << "Reach Nest" << std::endl;
        if (isHoldingFood) {
            num_targets_collected++; // Increment collected resource count
            LoopFunctions->currNumCollectedFood++; // Update current collected food count
            LoopFunctions->setScore(num_targets_collected); // Update the score
			//argos::LOG << "Resource collected by robot " << GetId() << " at tick " << SimulationTick() << std::endl;
            // Determine if pheromone should be placed
            argos::Real poissonCDF_pLayRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
            argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
            if (poissonCDF_pLayRate > r1 && updateFidelity) {
                TrailToShare.push_back(LoopFunctions->NestPosition); // Add nest position to trail
                argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
                Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
                LoopFunctions->PheromoneList.push_back(sharedPheromone); // Add pheromone to the list
                sharedPheromone.Deactivate(); // Ensure it won't get re-added later
            }
            TrailToShare.clear();
        }

        // Decide next task: Site fidelity, pheromones, or random search
        if (updateFidelity && GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity) > RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0))) {
            SetIsHeadingToNest(false);
            SetTarget(SiteFidelityPosition); // Use site fidelity
            isInformed = true;
        }
        else if (SetTargetPheromone()) {
            isInformed = true;
            isUsingSiteFidelity = false; // Follow pheromone waypoints
        }
        else {
            SetRandomSearchLocation(); // Perform a random search
            isInformed = false;
            isUsingSiteFidelity = false;
        }

        // Update robot state
        isGivingUpSearch = false;
		CPFA_state = DEPARTING;
        isHoldingFood = false;
        travelingTime += SimulationTick() - startTime;
        startTime = SimulationTick();
    }
	else{
		// **Force all robots to follow the entry path!**
		// if (CPFA_state != FOLLOWING_ENTRY_PATH) {
		// 	// static bool firstTime = true;
		// 	// if (firstTime) {
		// 	// 	argos::LOG << " Robot " << GetId() << " is headed towards the entry point" << std::endl;
		// 	// 	firstTime = false;
		// 	// }
			
		// 	// Find the closest waypoint in the entry path
		// 	currentWaypointIndex = FindClosestPointOnPath(GetPosition(), EntryPath);
		// 	SetTarget(EntryPath[currentWaypointIndex]);
		// 	if (IsAtTarget()){
		// 		currentWaypointIndex += 1;
		// 		SetTarget(EntryPath[currentWaypointIndex]);
		// 		CPFA_state = FOLLOWING_ENTRY_PATH;
		// 	}

		// 	// return;  // Exit early so the robot starts path-following
		// }

		returning_trajectory.push_back(GetPosition());
		if (returning_trajectory.size() > 100) {
			returning_trajectory.erase(returning_trajectory.begin()); // slide the window
		}
		// if (returning_trajectory.size() == 100) {
		// 	argos::CVector2 start = returning_trajectory.front();
		// 	argos::CVector2 end = returning_trajectory.back();
		// 	float euclidean_distance = (end - start).Length();
		
		// 	// Initialize EMA if first time
		// 	if (ema_distance < 0.0f) {
		// 		ema_distance = euclidean_distance;
		// 	}
		
		// 	// Update EMA using smoothing factor (alpha)
		// 	float alpha = 0.2f;
		// 	ema_distance = alpha * euclidean_distance + (1.0f - alpha) * ema_distance;
		
		// 	// Congestion detection: if distance is significantly less than EMA
		// 	if (euclidean_distance < ema_distance * 0.75f && GetPosition().Length() < 3.0f) {
		// 		// argos::LOG << "[Robot " << GetId() << "] CONGESTION DETECTED — "
		// 		// 		   << "Distance: " << euclidean_distance
		// 		// 		   << ", EMA: " << ema_distance << std::endl;
		// 		// argos::LOG << "Robot " << GetId() << " is in congestion at going to entry spiral" << std::endl;
		// 		currentWaypointIndex = FindClosestPointOnPath(GetPosition(), EntryPath);
		// 		SetTarget(EntryPath[currentWaypointIndex]);	
		// 		SetIsHeadingToNest(false);
		// 		CPFA_state = FOLLOWING_ENTRY_PATH;
		// 		// Reset state for next detection cycle
		// 		returning_trajectory.clear();
		// 		ema_distance = -1.0f;
		// 	}
		// }
		/*------------
		This is Trajectory Tortuosity Method
		-------------*/

		// If not the very first step, update distance_traveled
		if (returning_trajectory.size() > 1) {
			distance_traveled += (GetPosition() - previous_location).Length();
		}
		previous_location = GetPosition();
		//argos::LOG << "Distance traveled: " << distance_traveled << std::endl;
		// Then, once your window is full, start calculating tortuosity
		if (returning_trajectory.size() == 100) {
			argos::Real euclidean_distance = 
				(returning_trajectory.back() - returning_trajectory.front()).Length();

			argos::Real tortuosity = distance_traveled / euclidean_distance;
			//argos::LOG << "Distance traveled: " << distance_traveled << " - " << "Euclidean distance: " << euclidean_distance << std::endl;
			// the higher the threshold the less strict the algorithm is
			if (tortuosity > 3.0 && (GetPosition().Length() < 2.0)) {
				currentWaypointIndex = FindClosestPointOnPath(GetPosition(), EntryPath);
				SetTarget(EntryPath[currentWaypointIndex]);	
				SetIsHeadingToNest(false);
				goingtoentry = true;
				argos::LOG << GetId() <<  " - Current target is " << GetTarget() << std::endl;
				//CPFA_state = FOLLOWING_ENTRY_PATH;
				// Reset state for next detection cycle
				returning_trajectory.clear();
				distance_traveled = 0.0;
			}
		}
		// Slide the window forward after it's full
		if (returning_trajectory.size() >= 100) {
			// Subtract oldest segment before removing the point
			distance_traveled -= (returning_trajectory[1] - returning_trajectory[0]).Length();
			returning_trajectory.erase(returning_trajectory.begin());
		}

		/*------------
		End of Trajectory Tortuosity Method
		-------------*/
	}

}


void CPFA_controller::SetRandomSearchLocation() {
	argos::Real random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
	argos::Real x = 0.0, y = 0.0;

	/* north wall */
	if(random_wall < 0.25) {
		x = RNG->Uniform(ForageRangeX);
		y = ForageRangeY.GetMax();
	}
	/* south wall */
	else if(random_wall < 0.5) {
		x = RNG->Uniform(ForageRangeX);
		y = ForageRangeY.GetMin();
	}
	/* east wall */
	else if(random_wall < 0.75) {
		x = ForageRangeX.GetMax();
		y = RNG->Uniform(ForageRangeY);
	}
	/* west wall */
	else {
		x = ForageRangeX.GetMin();
		y = RNG->Uniform(ForageRangeY);
	}
		
	SetIsHeadingToNest(true); // Turn off error for this
	SetTarget(argos::CVector2(x, y));
}

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
void CPFA_controller::SetHoldingFood() {
	// Is the iAnt already holding food?
	if(IsHoldingFood() == false) {
		// No, the iAnt isn't holding food. Check if we have found food at our
		// current position and update the food list if we have.

		    std::vector<argos::CVector2> newFoodList;
		    std::vector<argos::CColor> newFoodColoringList;
		    size_t i = 0, j = 0;
			int currentTime = SimulationTick();  // Get current simulation time
		    //argos::LOG<<"LoopFunctions->FoodList size =" <<LoopFunctions->FoodList.size() << endl;
		    //argos::LOG<<"LoopFunctions->FoodColoringList size =" <<LoopFunctions->FoodColoringList.size() << endl;
 
	         for(i = 0; i < LoopFunctions->FoodList.size(); i++) {
	            if((GetPosition() - LoopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance ) {
					// if (dropCooldownMap.find(GetId()) != dropCooldownMap.end() &&
					// 	(currentTime - dropCooldownMap[GetId()] < DROP_COOLDOWN)) {
						
					// 	argos::LOG << "Robot " << GetId() << " is still in cooldown and cannot pick up its own dropped resource at: " 
					// 			<< LoopFunctions->FoodList[i] << " (Tick: " << currentTime << ")" << std::endl;
						
					// 	continue;  // Skip picking up its own recently dropped food
					// }					
					// We found food! Calculate the nearby food density.
					 isHoldingFood = true;
                     CPFA_state = SURVEYING;
					 //CPFA_state = FOUND;
					 j = i + 1;
					 searchingTime+=SimulationTick()-startTime;
					 startTime = SimulationTick();
					//increment totalResourcesPickedUp from loop functions
					LoopFunctions->totalResourcesPickedUp++;


					// Check if this food was previously dropped
					for (size_t k = 0; k < LoopFunctions->CongestionDropList.size(); k++) {
						if ((LoopFunctions->FoodList[i] - LoopFunctions->CongestionDropList[k]).SquareLength() < FoodDistanceTolerance) {
							// argos::LOG << "🚨 Robot " << GetId() << " picked up a previously dropped resource at: " 
							// 		<< LoopFunctions->FoodList[i] << " at: " << SimulationTick() << std::endl;
							LoopFunctions->totalResourcesPickedUp--;  // decrement the count since we dont count resources that were prevoiusly dropped as pickedup
							// Remove the food from the dropped list
							LoopFunctions->CongestionDropList.erase(LoopFunctions->CongestionDropList.begin() + k);
							break;  // Stop checking after the first match
						}
					}
				   //distribute a new food 
			       /*  argos::CVector2 placementPosition;
			         placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
			          
			         while(LoopFunctions->IsOutOfBounds(placementPosition, 1, 1)){
			             placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
			         }
			         newFoodList.push_back(placementPosition);
					 newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
                    LoopFunctions->increaseNumDistributedFoodByOne(); //the total number of cubes in the arena should be updated. qilu 11/15/2018
					 //end
					 */
                     break; 
		         } else {
				   //Return this unfound-food position to the list
				   newFoodList.push_back(LoopFunctions->FoodList[i]);
				   newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
				 }
			 }
    
      if(j>0){
          for(; j < LoopFunctions->FoodList.size(); j++) {
              newFoodList.push_back(LoopFunctions->FoodList[j]);
              newFoodColoringList.push_back(LoopFunctions->FoodColoringList[j]);
          }
      }
		//argos::LOG<<"newFoodList size =" << newFoodList.size() << endl;
      // We picked up food. Update the food list minus what we picked up.
      if(IsHoldingFood()) {
         //SetIsHeadingToNest(true);
         //SetTarget(LoopFunctions->NestPosition);
         LoopFunctions->FoodList = newFoodList;
         LoopFunctions->FoodColoringList = newFoodColoringList; //qilu 09/12/2016
         SetLocalResourceDensity();
        
      }
      newFoodList.clear();
     newFoodColoringList.clear();
	}
	 
		
	// This shouldn't be checked here ---
	// Drop off food: We are holding food and have reached the nest.
	//else if((GetPosition() - LoopFunctions->NestPosition).SquareLength() < LoopFunctions->NestRadiusSquared) {
	//    isHoldingFood = false;
	// }

	// We are carrying food and haven't reached the nest, keep building up the
	// pheromone trail attached to this found food item.
  /*if(IsHoldingFood() && SimulationTick() % LoopFunctions->DrawDensityRate == 0) {
        TrailToShare.push_back(GetPosition());
  }*/
}

void CPFA_controller::SetRobotDensity() {
	argos::CVector2 vect1, vect2;
	RobotDensity = 0;
    argos::Real  lowerPt, upperPt;
    bool split; // the angle is in [-pi, pi]. If the camera range on both sides of the angle pi, we need to split the range into two ranges.
    argos::Real neighborAngle, targetAngle; 
    /* Calculate resource density based on the global food list positions. */
	
	for(map<string, CVector2>::iterator it= LoopFunctions->robotPosList.begin(); it!=LoopFunctions->robotPosList.end(); ++it){
		if(controllerID.compare(it->first) != 0){
		  vect1 = it->second - GetPosition();
		  //argos::LOG<<controllerID<<" current position= "<< GetPosition()<<endl;
		  //argos::LOG<<"Heading = "<< GetHeading().GetValue()<< endl;
		  
		  //argos::CRadians angle(GetHeading());
		  //lowerRange = (GetHeading().UnsignedNormalize() - (argos::CRadians::PI)/4.0).GetValue();  
		  split=false;    
	      lowerPt = (GetHeading() - (argos::CRadians::PI)/4.0).GetValue();      
	      upperPt = (GetHeading() + (argos::CRadians::PI)/4.0).GetValue();
	      
	      neighborAngle = atan2(vect1.GetY(), vect1.GetX());
	      
	      //argos::LOG<<"lowerPt ="<<lowerPt<<endl;
	      //argos::LOG<<"upperPt ="<<upperPt<<endl;
	      if(lowerPt < -3.1415){
			  lowerPt += 2*3.1415;
			  split = true;
			  //argos::LOG<<"*** lowerPt************************ ***" <<endl;
			  //the two ranges are [-3.1415, upperPt] and [lowerPt, 3.1415]
	      }
	      if(upperPt > 3.1415){
			  upperPt -= 2*3.1415;
			  split = true;
			  //argos::LOG<<"*** upperPt************************ ***" <<endl; 
			  //the two ranges are [-3.1415, upperPt] and [lowerPt, 3.1415]
	      }
	      
	      //argos::LOG<<it->first<< " robot position= " << it->second <<endl; 
		  //argos::LOG<<"vect1 ="<< vect1<< ", angle=" << neighborAngle << endl;
		    
		  vect2 = GetTarget() - GetPosition();
		  
		  targetAngle = atan2(vect2.GetY(), vect2.GetX());
		  
		  //argos::LOG<<"GetHeading().GetValue()-targetAngle = "<< fabs(GetHeading().GetValue()-targetAngle) <<endl;
		  if(vect1.SquareLength() < LoopFunctions->CameraRadiusSquared && fabs(GetHeading().GetValue()-targetAngle)<= 0.35){ // 0.35 = 20 degree
			//argos::LOG<<"GetTarget()="<<GetTarget()<< endl;
		    //argos::LOG<<"GetPosition()=" << GetPosition() <<endl;
		    //argos::LOG<<"vect2 = " << vect2 << endl;     
		  
		    if(split){
			  if( (neighborAngle >= -3.1415 && neighborAngle <= upperPt) || (neighborAngle >= lowerPt && neighborAngle <= 3.1415) ){
				 RobotDensity++;  
				 //argos::LOG<<"*** in the splitted ranges ***" <<endl;
			  }
			}
		    else if(neighborAngle >= lowerPt && neighborAngle <= upperPt) {
		      RobotDensity++;
		      //argos::LOG<<"*** in the range ***" <<endl;
		    }
		    argos::LOG << controllerID<< " detects " << RobotDensity<< " robots."<<endl;			
			/*
			ofstream density_file;
			density_file.open("./results/densities.txt", ios::app); 
			density_file << controllerID << " detects " << RobotDensity << " robots.\n";
			density_file.close();
			*/
		  }
	  }
	}
	//LoopFunctions->robotDensities[controllerID] = RobotDensity;
}

/*****
 * If the robot has just picked up a food item, this function will be called
 * so that the food density in the local region is analyzed and saved. This
 * helps facilitate calculations for pheromone laying.
 *
 * Ideally, given that: [*] is food, and [=] is a robot
 *
 * [*] [*] [*] | The maximum resource density that should be calculated is
 * [*] [=] [*] | equal to 9, counting the food that the robot just picked up
 * [*] [*] [*] | and up to 8 of its neighbors.
 *
 * That being said, the random and non-grid nature of movement will not
 * produce the ideal result most of the time. This is especially true since
 * item detection is based on distance calculations with circles.
 *****/
void CPFA_controller::SetLocalResourceDensity() {
	argos::CVector2 distance;

	// remember: the food we picked up is removed from the foodList before this function call
	// therefore compensate here by counting that food (which we want to count)
	ResourceDensity = 1;

	/* Calculate resource density based on the global food list positions. */
	for(size_t i = 0; i < LoopFunctions->FoodList.size(); i++) {
	  distance = GetPosition() - LoopFunctions->FoodList[i];

	  if(distance.SquareLength() < LoopFunctions->SearchRadiusSquared*2) { //multiply 2 to use the diagonal distance
	    ResourceDensity++;
		LoopFunctions->FoodColoringList[i] = argos::CColor::ORANGE;
		LoopFunctions->ResourceDensityDelay = SimulationTick() + SimulationTicksPerSecond() * 10;
	  }
	}
 
	/* Set the fidelity position to the robot's current position. */
    SiteFidelityPosition = GetPosition();
    isUsingSiteFidelity = true;
    updateFidelity = true; 
    TrailToShare.push_back(SiteFidelityPosition);
    LoopFunctions->FidelityList[controllerID] = SiteFidelityPosition;
    /* Delay for 4 seconds (simulate iAnts scannning rotation). */
	//  Wait(4); // This function is broken. It causes the rover to move in the wrong direction after finishing its local resource density test 

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "(Survey): " << ResourceDensity << endl;
	//log_output_stream << "SiteFidelityPosition: " << SiteFidelityPosition << endl;
	//log_output_stream.close();
}

/*****
 * Update the global site fidelity list for graphics display and add a new fidelity position.
 *****/
void CPFA_controller::SetFidelityList(argos::CVector2 newFidelity) {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/*for(size_t i = 0; i < LoopFunctions->FidelityList.size(); i++) {
  if((LoopFunctions->FidelityList[i] - SiteFidelityPosition).SquareLength() != 0.0) {
			newFidelityList.push_back(LoopFunctions->FidelityList[i]);
		}
	} */


	/* Update the global fidelity list. */
	//LoopFunctions->FidelityList = newFidelityList;

        LoopFunctions->FidelityList[controllerID] = newFidelity;
	/* Add the robot's new fidelity position to the global fidelity list. */
	//LoopFunctions->FidelityList.push_back(newFidelity);
 

	/* Update the local fidelity position for this robot. */
	SiteFidelityPosition = newFidelity;
 
  updateFidelity = true;
}

/*****
 * Update the global site fidelity list for graphics display and remove the old fidelity position.
 *****/
void CPFA_controller::SetFidelityList() {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/* Update the global fidelity list. */
        LoopFunctions->FidelityList.erase(controllerID);
 SiteFidelityPosition = CVector2(10000, 10000);
 updateFidelity = true; 
}

/*****
 * Update the pheromone list and set the target to a pheromone position.
 * return TRUE:  pheromone was successfully targeted
 *        FALSE: pheromones don't exist or are all inactive
 *****/
bool CPFA_controller::SetTargetPheromone() {
	argos::Real maxStrength = 0.0, randomWeight = 0.0;
	bool isPheromoneSet = false;

 if(LoopFunctions->PheromoneList.size()==0) return isPheromoneSet; //the case of no pheromone.
	/* update the pheromone list and remove inactive pheromones */

	/* default target = nest; in case we have 0 active pheromones */
	//SetIsHeadingToNest(true);
	//SetTarget(LoopFunctions->NestPosition);
	/* Calculate a maximum strength based on active pheromone weights. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		if(LoopFunctions->PheromoneList[i].IsActive()) {
			maxStrength += LoopFunctions->PheromoneList[i].GetWeight();
		}
	}

	/* Calculate a random weight. */
	randomWeight = RNG->Uniform(argos::CRange<argos::Real>(0.0, maxStrength));

	/* Randomly select an active pheromone to follow. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		   if(randomWeight < LoopFunctions->PheromoneList[i].GetWeight()) {
			       /* We've chosen a pheromone! */
			       SetIsHeadingToNest(false);
          SetTarget(LoopFunctions->PheromoneList[i].GetLocation());
          TrailToFollow = LoopFunctions->PheromoneList[i].GetTrail();
          isPheromoneSet = true;
          /* If we pick a pheromone, break out of this loop. */
          break;
     }

     /* We didn't pick a pheromone! Remove its weight from randomWeight. */
     randomWeight -= LoopFunctions->PheromoneList[i].GetWeight();
	}

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "Found: " << LoopFunctions->PheromoneList.size()  << " waypoints." << endl;
	//log_output_stream << "Follow waypoint?: " << isPheromoneSet << endl;
	//log_output_stream.close();

	return isPheromoneSet;
}

/*****
 * Calculate and return the exponential decay of "value."
 *****/
argos::Real CPFA_controller::GetExponentialDecay(argos::Real w, argos::Real time, argos::Real lambda) {
	/* convert time into units of haLoopFunctions-seconds from simulation frames */
	//time = time / (LoopFunctions->TicksPerSecond / 2.0);

	//LOG << "time: " << time << endl;
	//LOG << "correlation: " << (value * exp(-lambda * time)) << endl << endl;

	//return (value * std::exp(-lambda * time));
    Real     twoPi       = (CRadians::TWO_PI).GetValue();
    return w + (twoPi-w)* exp(-lambda * time);
}

/*****
 * Provides a bound on the value by rolling over a la modulo.
 *****/
argos::Real CPFA_controller::GetBound(argos::Real value, argos::Real min, argos::Real max) {
	/* Calculate an offset. */
	argos::Real offset = std::abs(min) + std::abs(max);

	/* Increment value by the offset while it's less than min. */
	while (value < min) {
			value += offset;
	}

	/* Decrement value by the offset while it's greater than max. */
	while (value > max) {
			value -= offset;
	}

	/* Return the bounded value. */
	return value;
}

size_t CPFA_controller::GetSearchingTime(){//qilu 10/22
    return searchingTime;
}
size_t CPFA_controller::GetTravelingTime(){//qilu 10/22
    return travelingTime;
}

string CPFA_controller::GetStatus(){//qilu 10/22
    //DEPARTING, SEARCHING, RETURNING 
    if (CPFA_state == DEPARTING) return "DEPARTING";
    else if (CPFA_state ==SEARCHING)return "SEARCHING";
    else if (CPFA_state == RETURNING)return "RETURNING";
    else if (CPFA_state == SURVEYING) return "SURVEYING";
	else if (CPFA_state == FOLLOWING_ENTRY_PATH) return "FOLLOWING_ENTRY_PATH";
	else if (CPFA_state == FOLLOWING_EXIT_PATH) return "FOLLOWING_EXIT_PATH";
    //else if (MPFA_state == INACTIVE) return "INACTIVE";
    else return "SHUTDOWN";
    
}

void CPFA_controller::setStatus(string status){
	if(status == "DEPARTING") CPFA_state = DEPARTING;
	else if(status == "SEARCHING") CPFA_state = SEARCHING;
	else if(status == "RETURNING") CPFA_state = RETURNING;
	else if(status == "SURVEYING") CPFA_state = SURVEYING;
	//else if(status == "INACTIVE") MPFA_state = INACTIVE;
}

/*****
 * Return the Poisson cumulative probability at a given k and lambda.
 *****/
argos::Real CPFA_controller::GetPoissonCDF(argos::Real k, argos::Real lambda) {
	argos::Real sumAccumulator       = 1.0;
	argos::Real factorialAccumulator = 1.0;

	for (size_t i = 1; i <= floor(k); i++) {
		factorialAccumulator *= i;
		sumAccumulator += pow(lambda, i) / factorialAccumulator;
	}

	return (exp(-lambda) * sumAccumulator);
}

void CPFA_controller::UpdateTargetRayList() {
	if(SimulationTick() % LoopFunctions->DrawDensityRate == 0 && LoopFunctions->DrawTargetRays == 1) {
		/* Get position values required to construct a new ray */
		argos::CVector2 t(GetTarget());
		argos::CVector2 p(GetPosition());
		argos::CVector3 position3d(p.GetX(), p.GetY(), 0.02);
		argos::CVector3 target3d(t.GetX(), t.GetY(), 0.02);

		/* scale the target ray to be <= searchStepSize */
		argos::Real length = std::abs(t.Length() - p.Length());

		if(length > SearchStepSize) {
			MyTrail.clear();
		} else {
			/* add the ray to the robot's target trail */
			argos::CRay3 targetRay(target3d, position3d);
			MyTrail.push_back(targetRay);

			/* delete the oldest ray from the trail */
			if(MyTrail.size() > MaxTrailSize) {
				MyTrail.erase(MyTrail.begin());
			}

			LoopFunctions->TargetRayList.insert(LoopFunctions->TargetRayList.end(), MyTrail.begin(), MyTrail.end());
			// loopFunctions.TargetRayList.push_back(myTrail);
		}
	}
}

REGISTER_CONTROLLER(CPFA_controller, "CPFA_controller")
