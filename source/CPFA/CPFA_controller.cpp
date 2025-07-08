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
        last_time_in_seconds(0),
		timeSet(false),
		timeInsideRedCircle(0.0),
		totalTimeInsideRedCircle(0.0)
{
	GoStraightAngleRangeInDegreesInRegion.Set(-40.0, 40.0);
	GoStraightAngleRangeInDegreesGoingToRegion.Set(-55.0, 55.0);
	GoStraightAngleRangeInDegreesLeftSide.Set(-90.0, -30.0);
	GoStraightAngleRangeInDegreesRightSide.Set(30.0, 90.0);
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
    // Update the trail color based on the state
    if (CPFA_state == FOLLOWING_ENTRY_PATH) {
        TrailColor = CColor::RED; // Red for FOLLOWING_ENTRY_PATH
    } else if (CPFA_state == FOLLOWING_EXIT_PATH) {
        TrailColor = CColor::GREEN; // Green for FOLLOWING_EXIT_PATH
    } else {
        TrailColor = CColor::BLUE; // Default to blue for other states
    }

    // Add line to draw the trail only in the specified states
    curr_time_in_seconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
    if (curr_time_in_seconds - last_time_in_seconds >= 0) {
        // if (CPFA_state == FOLLOWING_ENTRY_PATH || CPFA_state == FOLLOWING_EXIT_PATH) {
        //     CVector2 position2d(GetPosition().GetX(), GetPosition().GetY());
        //     CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
        //     CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
        //     CRay3 targetRay(target3d, position3d);
        //     myTrail.push_back(targetRay);
        //     LoopFunctions->Trajectory[controllerID].push_back(position2d);

        //     // Add the ray to the global trail list
        //     LoopFunctions->TargetRayList.push_back(targetRay);
        //     LoopFunctions->TargetRayColorList.push_back(TrailColor);
        // }

        // Update the previous position and time
        previous_position = GetPosition();
        last_time_in_seconds = curr_time_in_seconds;
    }

	// change led colors based on cpfa state returning, following entry path, and following exit path
	if(CPFA_state == RETURNING) {
		m_pcLEDs->SetAllColors(CColor::RED);
	} else if(CPFA_state == FOLLOWING_ENTRY_PATH) {
		m_pcLEDs->SetAllColors(CColor::PURPLE);
	} else if(CPFA_state == FOLLOWING_EXIT_PATH) {
		m_pcLEDs->SetAllColors(CColor::YELLOW);
	} 


	//UpdateTargetRayList();
	CPFA();
	Move();

	// if (GetTarget().GetX() == 0.0 && GetTarget().GetY() == 0.0) {
	// 	argos::LOG << "ERobot: " << GetId() << " is at target (0,0) on exit path with status " << GetStatus() << std::endl;
	// }

	// Check if the robot is within 2.0 units from the origin (0,0)
	if(IsInTheNest() && GetStatus() == "RETURNING" && isHoldingFood && firstTimeInNest) {
		// send inCircleCounter to loopfunctions
		LoopFunctions->UpdateInCircleCounter(inCircleCounter);
		inCircleCounter = 0;
	}
	else if ((GetPosition() - argos::CVector2(0.0, 0.0)).Length() <= 2.0 && GetStatus() == "RETURNING" && isHoldingFood) {
		inCircleCounter++;
	}
	// if ((GetPosition() - argos::CVector2(0.0, 0.0)).Length() <= 1.1 && GetStatus() != "RETURNING" && GetStatus() != "FOLLOWING_EXIT_PATH" && GetStatus() != "FOLLOWING_ENTRY_PATH") {
	// 	// argos::LOG << GetId() << " is within 0.7 distance of the center. " << GetStatus() <<std::endl;
	// 	// CPFA_state = RETURNING;
	// 	// Set to closest exit path
		
	// }
	// if(GetId() == "F19"){
	// 	argos::LOG << GetId() << " has target: " << GetTarget() << std::endl;
	// }
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
    SetTarget(LoopFunctions->NestPositions[1]); //qilu 09/08          ///////////
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
    for (const auto& nest_position : LoopFunctions->NestPositions) {
        if ((GetPosition() - nest_position).SquareLength() < LoopFunctions->NestRadiusSquared) {
            return true; // The robot is in one of the nests
        }
    }
    return false; // The robot is not in any nest
}



bool CPFA_controller::CollisionDetection() {
	//log current CPFA State
	argos::CVector2 collisionVector = GetCollisionVector();
	argos::Real collisionAngle = ToDegrees(collisionVector.Angle()).GetValue();
	bool isCollisionDetected = false;

	// if (GetStatus() == "RETURNING" && inCentralZone()) {
    //     if (GoStraightAngleRangeInDegreesInRegion.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
    //         && collisionVector.Length() > 0.0) {
	// 		// If a collision is detected for more than 32 timesteps (1 second) in a row, then keep moving to prevent deadlock.
	// 		if(stopCounter < 32){
	// 			Stop();

	// 		};
	// 		// stopCounter++;
	// 		return true;
	// 	}else{
	// 		// stopCounter = 0;
	// 		return false;
	// 	}
	// }

	// if robot is following exit path then ignore collisions.
	if(GetStatus() == "FOLLOWING_EXIT_PATH"){
		if (GoStraightAngleRangeInDegreesInRegion.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
		&& collisionVector.Length() > 0.0) {
		return true;
		} else {
			return false;
		}
	}

    if (GetStatus() == "FOLLOWING_ENTRY_PATH" || isWaitingForCollision) {
        if (GoStraightAngleRangeInDegreesInRegion.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
            && collisionVector.Length() > 0.0) {
				collision_counter++;
			// if in collision for more than 10 timesteps, ignore collison
            stopcooldownCounter++; // Increment the stop counter
            if (stopcooldownCounter > 10) {
                stopcooldownCounter = 0; // Reset the counter
				return true;
			}
			else{
				Stop();
				return true;
			}
        } else{
			return false;
		}

    }

	else{
		if(GoStraightAngleRangeInDegrees.WithinMinBoundIncludedMaxBoundIncluded(collisionAngle)
			&& collisionVector.Length() > 0.0) {

			isCollisionDetected = true;
			collision_counter++;	 
			
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
			if(isHoldingFood){
				SetTarget(entrypoint);
			}

		}

		return isCollisionDetected;
	}
}


/* Attempting to perform avoidance logic for robots to find open slot in path*/

bool CPFA_controller::IsLeftOfLine(const argos::CVector2& A, const argos::CVector2& B, const argos::CVector2& P) {
	// argos::LOG << "IsLeftOfLine called with A: " << A << ", B: " << B << ", P: " << P << std::endl;
    Real side = (B.GetX() - A.GetX()) * (P.GetY() - A.GetY()) -
                (B.GetY() - A.GetY()) * (P.GetX() - A.GetX());
    return side > 0; // left = true, right = false
}
Real CPFA_controller::DistanceFromPointToSegment(const argos::CVector2& P, const argos::CVector2& A, const argos::CVector2& B) {
	// argos::LOG << "DistanceFromPointToSegment called with P: " << P << ", A: " << A << ", B: " << B << std::endl;
    argos::CVector2 AB = B - A;
    argos::CVector2 AP = P - A;

	Real t = (AB.DotProduct(AP)) / (AB.SquareLength());
	t = std::max(0.0, std::min(1.0, t)); // Clamp to segment

    argos::CVector2 closest = A + AB * t;
    return (P - closest).Length();
}
bool CPFA_controller::IsLeftOfPath(const std::vector<argos::CVector2>& path, const argos::CVector2& pos) {
	// argos::LOG << "IsLeftOfPath called with path size: " << path.size() << std::endl;

    Real min_dist = std::numeric_limits<Real>::max();
    size_t best_index = 0;

    for(size_t i = 0; i < path.size() - 1; ++i) {
        Real dist = DistanceFromPointToSegment(pos, path[i], path[i+1]);
        if(dist < min_dist) {
            min_dist = dist;
            best_index = i;
        }
    }

    return IsLeftOfLine(path[best_index], path[best_index + 1], pos);
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
	m_pcLEDs->SetAllColors(CColor::BLACK);
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
	
	if (SimulationTick() % 40 == 0) {
		// argos::LOG << GetId() << " is stopping." << std::endl;
		Stop();
	}

	if (IsInTheNest()) {
		//argos::LOG << "Executed " << currentWaypointIndex << " out of " << EntryPath.size() << " Waypoints" << std::endl;

		if(nestStopCounter == 0){
			firstTimeInNest = true;
		}else{
			firstTimeInNest = false;
		}

		if(nestStopCounter < 160){
			Stop();
			nestStopCounter++;
			return;
		}

		if (isHoldingFood) {
			num_targets_collected++;
			LoopFunctions->currNumCollectedFood++;
			LoopFunctions->setScore(num_targets_collected);
		}

		if (updateFidelity && GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity) > RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0))) {
		    SetIsHeadingToNest(false);
		    SetTarget(SiteFidelityPosition);
			// LOG << GetId() << " is using site fidelity at position: " << SiteFidelityPosition << std::endl;
		    isInformed = true;
		}
		else if (SetTargetPheromone()) {
			// LOG << GetId() << " is using pheromone at position: " << GetTarget() << std::endl;
		    isInformed = true;
		    isUsingSiteFidelity = false;
		}
		else {
		    SetRandomSearchLocation();
		    isInformed = false;
		    isUsingSiteFidelity = false;
			useRandomSearch = true;
			// LOG << GetId() << " is using random search at position: " << GetTarget() << std::endl;
		}


		mainTarget = GetTarget();


		// if (followingEntryPath1) {
        //     SetTarget(exitPath1[0]);
        //     followingEntryPath1 = false;
        //     actualExitPath = exitPath1;
        // } else if (followingEntryPath2) {
        //     SetTarget(exitPath2[0]);
        //     followingEntryPath2 = false;
        //     actualExitPath = exitPath2;
        // } else if (followingEntryPath3) {
        //     SetTarget(exitPath3[0]);
        //     followingEntryPath3 = false;
        //     actualExitPath = exitPath3;
        // } else if (followingEntryPath4) {
        //     SetTarget(exitPath4[0]);
        //     followingEntryPath4 = false;
        //     actualExitPath = exitPath4;
        // }

		/* 
		This is for choosing the exit path that is closest to next destination.
		*/		

		argos::CVector2 bestStep = exitPoints[0];
		Real bestDistance = (mainTarget - bestStep).Length();

		for (size_t i = 1; i < exitPoints.size(); ++i) {
			Real dist = (mainTarget - exitPoints[i]).Length();
			if (dist < bestDistance) {
				bestDistance = dist;
				bestStep = exitPoints[i];
			}
		}

		if(bestStep == exitPath1[1]){
			actualExitPath = exitPath1;
			followingEntryPath1 = false;
		} else if(bestStep == exitPath2[1]) {
			actualExitPath = exitPath2;
			followingEntryPath2 = false;
		} else if(bestStep == exitPath3[1]) {
			actualExitPath = exitPath3;
			followingEntryPath3 = false;
		} else if(bestStep == exitPath4[1]) {
			actualExitPath = exitPath4;
			followingEntryPath4 = false;
		}
		// argos::LOG << "Robot: " << GetId() << " reached the nest and is now following the exit path to: "
			// << actualExitPath[0] << std::endl;
		SetTarget(actualExitPath[0]);

		isHoldingFood = false; 
		isGivingUpSearch = false;
        travelingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22		
		CPFA_state = FOLLOWING_EXIT_PATH;
		currentWaypointIndex = 2;
		nestStopCounter = 0;
		actualPath.clear();
		return;
	}

	if (IsAtTarget()) {
		// argos::LOG << "Robot " << GetId() << " is at waypoint " << currentWaypointIndex << std::endl;
		// argos::LOG << "Next target is: " << EntryPath[currentWaypointIndex] << std::endl;
		//check if is in bounds
		if (currentWaypointIndex >= actualPath.size()) {
			argos::LOG << "Robot " << GetId() << " has reached the end of the entry path. waypoint: "<< currentWaypointIndex << " out of " << actualPath.size() << std::endl;
			// SetTarget(LoopFunctions->NestPositions[1]);
			// set target to last point in actualPath
			SetTarget(actualPath.back());
			// CPFA_state = FOLLOWING_EXIT_PATH;
			return;
		}else{
			SetTarget(actualPath[currentWaypointIndex]);
			currentWaypointIndex++;
		}
		notAtTargetCounter = 0;
	}else{
		notAtTargetCounter++;
	}

	// if robot hasnt reached the target after 300 ticks, find closest point on path and set target to next point
	if(notAtTargetCounter > 300){
		currentWaypointIndex = FindClosestPointIndexOnPath(actualPath);
		currentWaypointIndex += 1; // increment to next point
		if (currentWaypointIndex >= actualPath.size()) {
			SetTarget(actualPath.back());
		}
		else{
			SetTarget(actualPath[currentWaypointIndex]);
		}
		notAtTargetCounter = 0;
	}

}

void CPFA_controller::FollowingExitPath() {

	if (SimulationTick() % 100 == 0) {
		// argos::LOG << GetId() << " is stopping." << std::endl;
		Stop();
	}
	
	// check if target is 0,0, the print
	// if (GetTarget().GetX() == 0.0 && GetTarget().GetY() == 0.0) {
	// 	argos::LOG << "EXIT Robot: " << GetId() << " is at target (0,0) on exit path." << std::endl;
	// }


	if ((GetPosition() - GetTarget()).Length() < EntryPointThreshold) {
		//argos::LOG << "Reached intermediate target on exit path..." << std::endl;
        SetTarget(actualExitPath[1]);
		actualExitPath.clear();
		goingtoexit = true;
		exitPointCounter = 0;
		hasntReachedFirstExitPoint = true;
		secondExitPointCounter = 0;
	}
	if(!hasntReachedFirstExitPoint){
		exitPointCounter++;
	}else{
		secondExitPointCounter++;
	}

	if(exitPointCounter > 150 || secondExitPointCounter > 1300) {
		// if the robot is not at the target after 100 ticks, choose a random new exit path from exitPath1, exitPath2, exitPath3, exitPath4
		// argos::LOG << "Robot: " << GetId() << " is not at the target after 150 ticks, choosing a new exit path." << std::endl;
		int randomExitPath = RNG->Uniform(argos::CRange<int>(0, 3));
		if (randomExitPath == 0) {
			actualExitPath = exitPath1;
			SetTarget(exitPath1[0]);
		} else if (randomExitPath == 1) {
			actualExitPath = exitPath2;
			SetTarget(exitPath2[0]);
		} else if (randomExitPath == 2) {
			actualExitPath = exitPath3;
			SetTarget(exitPath3[0]);
		} else if (randomExitPath == 3) {
			actualExitPath = exitPath4;
			SetTarget(exitPath4[0]);
		}
		exitPointCounter = 0;
		secondExitPointCounter = 0;
	}

	if(goingtoexit && ((GetPosition() - GetTarget()).Length() < EntryPointThreshold)){
		exitPointCounter = 0;
		secondExitPointCounter = 0;
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

		// argos::LOG << GetId() <<  " Going to main target: " << mainTarget << std::endl;

		
		// if (GetTarget().GetX() == 0.0 && GetTarget().GetY() == 0.0) {
		// 	argos::LOG << "EXIT Robot: " << GetId() << " is at " << GetTarget() << " on exit path." << std::endl;
		// 	SetRandomSearchLocation();
		// 	argos::LOG << "Robot: " << GetId() << " is using random search at position: " << GetTarget() << std::endl;
		// }else{
		// 	SetTarget(mainTarget);
		// }

		CPFA_state = DEPARTING; 		
		goingtoexit = false;
		hasntReachedFirstExitPoint = false;
	}
}


void CPFA_controller::Searching() {

	// if(inCentralZone()){
	// 	// argos::LOG << GetId() << " reached the radius and is now doing random search..." << std::endl;

	// 	Stop();
	// 	SearchTime = 0;
	// 	travelingTime+=SimulationTick()-startTime;//qilu 10/22
	// 	startTime = SimulationTick();//qilu 10/22
   
	// 	argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
	// 	argos::Real rand = RNG->Gaussian(USV);
	// 	argos::CRadians rotation(rand);
	// 	argos::CRadians angle1(rotation.UnsignedNormalize());
	// 	argos::CRadians angle2(GetHeading().UnsignedNormalize());
	// 	argos::CRadians turn_angle(angle1 + angle2);
	// 	argos::CVector2 turn_vector(SearchStepSize, turn_angle);
	// 	SetIsHeadingToNest(false);
	// 	SetTarget(turn_vector + GetPosition());
	// 	return;
	// }
 //LOG<<"Searching..."<<endl;
	// "scan" for food only every half of a second
	m_pcLEDs->SetAllColors(CColor::BLACK);
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		SetHoldingFood();
	}

	if(IsInsideCircleBoundary(GetPosition())){
		// Stop();
		argos::CVector2 direction = GetPosition() - m_cCircleCenter;
		direction.Normalize();
		argos::CVector2 escape_target = GetPosition() + direction * SearchStepSize;
		
		SetTarget(escape_target);
		// argos::LOG << GetId() << " is inside the circle boundary and escaping to: " 
		// 	<< escape_target.GetX() << ", " << escape_target.GetY() << std::endl;
		return; 
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
			//  entrypoint = FindClosestEntryPoint();
			 entrypoint = FindClosestNest();
			//  argos::LOG << "Robot: " << GetId() << " is giving up searching and going to entry point: "
			// 	<< entrypoint.GetX() << ", " << entrypoint.GetY() << std::endl;
			 if (entrypoint == LoopFunctions->NestPositions[1]) {
				actualPath = entryPath1;
				// SetTarget(entryPath1[1]);
				followingEntryPath1 = true;
				//SetTarget(LoopFunctions->NestPositions[1]);
			} else if (entrypoint == LoopFunctions->NestPositions[3]) {
				// SetTarget(entryPath2[1]);
				actualPath = entryPath2;
				followingEntryPath2 = true;
				// SetTarget(LoopFunctions->NestPositions[3]);
			} else if (entrypoint == LoopFunctions->NestPositions[0]) {
				// SetTarget(entryPath3[1]);
				actualPath = entryPath3;
				followingEntryPath3 = true;
				//SetTarget(LoopFunctions->NestPositions[0]);
			} else if (entrypoint == LoopFunctions->NestPositions[2]) {
				// SetTarget(entryPath4[1]);
				actualPath = entryPath4;
				followingEntryPath4 = true;
				// SetTarget(LoopFunctions->NestPositions[2]);
			}
			//  goingtonest = true;
			 SetTarget(entrypoint);
			 
             isGivingUpSearch = true;
	         LoopFunctions->FidelityList.erase(controllerID);
             isUsingSiteFidelity = false; 
             updateFidelity = false; 
			 CPFA_state = RETURNING;
			//  argos::LOG << "Robot: " << GetId() << " is at returning state after giving up searching." << std::endl;
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
          //SetTarget(turn_vector + GetPosition());
		  //argos::LOG << GetId() << " is turning away from nest" << std::endl;
		//   Stop();
		  SetTarget(SafeTargetFromHeading(turn_angle, SearchStepSize));

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
                  log_output_stream.close();x
                  */
                  //SetTarget(turn_vector + GetPosition());
				//   Stop();
				  SetTarget(SafeTargetFromHeading(turn_angle, SearchStepSize));

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
		// entrypoint = FindClosestEntryPoint();
		entrypoint = FindClosestNest();
		// argos::LOG << "Robot: " << GetId() << " is finished surveying and going to entry point: "
		// 	<< entrypoint.GetX() << ", " << entrypoint.GetY() << std::endl;
		if (entrypoint == LoopFunctions->NestPositions[1]) {
			// argos::LOG << "Robot: " << GetId() << " set following entry path 1." << std::endl;
			actualPath = entryPath1;
			// SetTarget(entryPath1[1]);
			followingEntryPath1 = true;
			//SetTarget(LoopFunctions->NestPositions[1]);
		} else if (entrypoint == LoopFunctions->NestPositions[3]) {
			// argos::LOG << "Robot: " << GetId() << " set following entry path 2." << std::endl;
			// SetTarget(entryPath2[1]);
			actualPath = entryPath2;
			followingEntryPath2 = true;
			// SetTarget(LoopFunctions->NestPositions[3]);
		} else if (entrypoint == LoopFunctions->NestPositions[0]) {
			// argos::LOG << "Robot: " << GetId() << " set following entry path 3." << std::endl;
			// SetTarget(entryPath3[1]);
			actualPath = entryPath3;
			followingEntryPath3 = true;
			//SetTarget(LoopFunctions->NestPositions[0]);
		} else if (entrypoint == LoopFunctions->NestPositions[2]) {
			// argos::LOG << "Robot: " << GetId() << " set following entry path 4." << std::endl;
			// SetTarget(entryPath4[1]);
			actualPath = entryPath4;
			followingEntryPath4 = true;
			// SetTarget(LoopFunctions->NestPositions[2]);
		}
		// goingtonest = true;
		SetTarget(entrypoint);
		// SetTarget(LoopFunctions->NestPositions[0]); // Go to the first nest position
		CPFA_state = RETURNING;
		//argos::LOG << "Robot: " << GetId() << " is returning to the nest after surveying." << std::endl;
		survey_count = 0; // Reset
        searchingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22
            
	}
}

Real CPFA_controller::GetTotalTimeInsideRedCircle() {
	return totalTimeInsideRedCircle;
}

void CPFA_controller::Returning() {

	// if (SimulationTick() % 100 == 0) {
	// 	// argos::LOG << GetId() << " is stopping." << std::endl;
	// 	Stop();
	// }

	if(runningfromcorridor){
		SetTarget(escapeTarget);
		
		if(IsAtTarget()){
			hasExecutedOnce = true;
			runningfromcorridor = false;
			SetTarget(entrypoint);
		}
		if (IsInTheNest()) {
			if(nestStopCounter == 0){
				firstTimeInNest = true;
			}else{
				firstTimeInNest = false;
			}
	
			//stop for 160 timesteps
			if(nestStopCounter < 160){
				Stop();
				nestStopCounter++;
				return;
			}
	
			if (isHoldingFood) {
				num_targets_collected++;
				LoopFunctions->currNumCollectedFood++;
				LoopFunctions->setScore(num_targets_collected);
			}
	
			if (followingEntryPath1) {
				SetTarget(exitPath1[0]);
				followingEntryPath1 = false;
				actualExitPath = exitPath1;
			} else if (followingEntryPath2) {
				SetTarget(exitPath2[0]);
				followingEntryPath2 = false;
				actualExitPath = exitPath2;
			} else if (followingEntryPath3) {
				SetTarget(exitPath3[0]);
				followingEntryPath3 = false;
				actualExitPath = exitPath3;
			} else if (followingEntryPath4) {
				SetTarget(exitPath4[0]);
				followingEntryPath4 = false;
				actualExitPath = exitPath4;
			}

	
			isHoldingFood = false; 
			isGivingUpSearch = false;
			travelingTime+=SimulationTick()-startTime;//qilu 10/22
			startTime = SimulationTick();//qilu 10/22		
			CPFA_state = FOLLOWING_EXIT_PATH;
			hasExecutedOnce = false;
			currentWaypointIndex = 2;
			nestStopCounter = 0;
			runningfromcorridor = false;
			actualPath.clear();
		}
	}

	/* Avoid Exit Path */
	else if(IsInsideRestrictedExitCorridor(GetPosition())) {
		Stop();
		SetIsHeadingToNest(false);
	
		// LOG << "Robot: " << GetId() << " is in restricted corridor. Original target: "
		// 	<< GetTarget().GetX() << ", " << GetTarget().GetY() << std::endl;
	
		// Determine deflection direction based on robot's position
		CRadians baseAngle = (CVector2(0, 0) - GetPosition()).Angle(); // Angle from robot to center
		argos::CVector2 robotPos = GetPosition();
		if(followingEntryPath1){
			isLeft = IsLeftOfPath(exitPath1Straight, robotPos);
		}else if(followingEntryPath2){
			isLeft = IsLeftOfPath(exitPath2Straight, robotPos);
		}else if(followingEntryPath3){
			isLeft = IsLeftOfPath(exitPath3Straight, robotPos);
		}else if(followingEntryPath4){
			isLeft = IsLeftOfPath(exitPath4Straight, robotPos);
		}
		CRadians deflection = isLeft ? CRadians::PI / 4 : -CRadians::PI / 4;
	
		CRadians deflectedAngle = baseAngle + deflection;
		CVector2 escapeVec(SearchStepSize * 2.5, deflectedAngle);
		escapeTarget = GetPosition() + escapeVec;
		SetTarget(escapeTarget);
		runningfromcorridor = true;
		return;
	}

	// if robot is not holding food, do not enter
	if(!isHoldingFood && inCentralZone()){
		// argos::LOG << GetId() << " reached the radius and is now doing random search..." << std::endl;
		CPFA_state = SEARCHING; 
		isGivingUpSearch = false;
		isInformed = false;
		isUsingSiteFidelity = false;
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
		return;
	}

	if(!timeSet && inCentralZone()) {
		timeInsideRedCircle = LoopFunctions->getSimTimeInSeconds();
		timeSet = true;
	}

	

	/* This is in case the robot actually reaches the nest directly (No Path) */
	if (IsInTheNest()) {
		if(nestStopCounter == 0){
			firstTimeInNest = true;
		}else{
			firstTimeInNest = false;
		}

		if(timeSet && isHoldingFood) {
			totalTimeInsideRedCircle += (LoopFunctions->getSimTimeInSeconds() - timeInsideRedCircle);
			timeSet = false;
		}

		//stop for 160 timesteps
		if(nestStopCounter < 160){
			Stop();
			nestStopCounter++;
			return;
		}

		if (isHoldingFood) {
			num_targets_collected++;
			LoopFunctions->currNumCollectedFood++;
			LoopFunctions->setScore(num_targets_collected);
		}

		if (followingEntryPath1) {
            SetTarget(exitPath1[0]);
            followingEntryPath1 = false;
            actualExitPath = exitPath1;
        } else if (followingEntryPath2) {
            SetTarget(exitPath2[0]);
            followingEntryPath2 = false;
            actualExitPath = exitPath2;
        } else if (followingEntryPath3) {
            SetTarget(exitPath3[0]);
            followingEntryPath3 = false;
            actualExitPath = exitPath3;
        } else if (followingEntryPath4) {
            SetTarget(exitPath4[0]);
            followingEntryPath4 = false;
            actualExitPath = exitPath4;
        }
		// argos::LOG << "Robot: " << GetId() << " reached the nest and is now following the exit path to: "
		// 	<< GetTarget().GetX() << ", " << GetTarget().GetY() << std::endl;

		isHoldingFood = false; 
		isGivingUpSearch = false;
        travelingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22		
		CPFA_state = FOLLOWING_EXIT_PATH;
		hasExecutedOnce = false;
		currentWaypointIndex = 2;
		nestStopCounter = 0;
		runningfromcorridor = false;
		actualPath.clear();
		return;
	}

	// If we detect a collision, calculate the closest point on the entry path and set it as the target.
	if(CollisionDetection() && inCentralZone()){
		if(!isWaitingForCollision) {
			timeCollided = collision_counter;
			isWaitingForCollision = true;
		}
		else{
			// argos::LOG << "Robot: " << GetId() << " is waiting for collision to end." << std::endl;
			size_t collision = collision_counter - timeCollided;
			if(collision > 50) {
				if(followingEntryPath1){
					pointonpath = FindClosestPointIndexOnPath(entryPath1);
				} else if (followingEntryPath2) {
					pointonpath = FindClosestPointIndexOnPath(entryPath2);
				} else if (followingEntryPath3) {
					pointonpath = FindClosestPointIndexOnPath(entryPath3);
				} else if (followingEntryPath4) {
					pointonpath = FindClosestPointIndexOnPath(entryPath4);
				}
						// stop for 1 second to enter path
		// if(stopCounter > 32){

			// only do pointonpath+1 if in bounds actualPath
			if(pointonpath >= actualPath.size()){
				pointonpath -= 1;
			}
			SetTarget(actualPath[pointonpath]);
			// LOG << GetId() << " is going to entry path." << GetTarget() <<std::endl;
			CPFA_state = FOLLOWING_ENTRY_PATH;
			hasExecutedOnce = false;
			currentWaypointIndex = pointonpath;	
			isWaitingForCollision = false;	
		// }
			}
		}




		return;
	}else {
		if(isWaitingForCollision) {
			isWaitingForCollision = false;
			timeCollided = 0;
		}
	}

	/* ******** Following Entry Path Method ******** */

	// if ((GetPosition() - entrypoint).Length() < EntryPointThreshold){

	// 	if (entrypoint == entryPoints[0]) {
	// 		actualPath = entryPath1;
	// 		SetTarget(entryPath1[1]);
	// 		followingEntryPath1 = true;
	// 		//SetTarget(LoopFunctions->NestPositions[1]);
	// 	} else if (entrypoint == entryPoints[1]) {
	// 		SetTarget(entryPath2[1]);
	// 		actualPath = entryPath2;
	// 		followingEntryPath2 = true;
	// 		// SetTarget(LoopFunctions->NestPositions[3]);
	// 	} else if (entrypoint == entryPoints[2]) {
	// 		SetTarget(entryPath3[1]);
	// 		actualPath = entryPath3;
	// 		followingEntryPath3 = true;
	// 		//SetTarget(LoopFunctions->NestPositions[0]);
	// 	} else if (entrypoint == entryPoints[3]) {
	// 		SetTarget(entryPath4[1]);
	// 		actualPath = entryPath4;
	// 		followingEntryPath4 = true;
	// 		// SetTarget(LoopFunctions->NestPositions[2]);
	// 	}

	// 	goingtoentry = false;
	// 	CPFA_state = FOLLOWING_ENTRY_PATH;
	// }
}

bool CPFA_controller::IsInsideRestrictedExitCorridor(const argos::CVector2& robotPos) {
    Real corridorRadius = 0.21;

    // for(const auto& exitPoint : exitPoints) {
    //     argos::CVector2 toOrigin = argos::CVector2(0, 0) - exitPoint;
    //     argos::CVector2 AP = robotPos - exitPoint;

	// 	Real t = (toOrigin.DotProduct(AP)) / toOrigin.SquareLength();
    //     t = std::max(0.0, std::min(1.0, t));  // Clamp to segment

    //     argos::CVector2 closestPoint = exitPoint + t * toOrigin;
    //     Real distance = (robotPos - closestPoint).Length();

    //     if(distance <= corridorRadius)
    //         return true;
    // }

    // return false;

    for (const auto& exitPath : {exitPath1, exitPath2, exitPath3, exitPath4}) {
        // Iterate over each segment in the exit path
        for (size_t i = 0; i < exitPath.size() - 1; ++i) {
            argos::CVector2 toOrigin = exitPath[i + 1] - exitPath[i]; // Vector from start to end of the segment
            argos::CVector2 AP = robotPos - exitPath[i];             // Vector from start of the segment to the robot

            // Project the robot's position onto the segment
            Real t = (toOrigin.DotProduct(AP)) / toOrigin.SquareLength();
            t = std::max(0.0, std::min(1.0, t)); // Clamp to the segment

            // Find the closest point on the segment
            argos::CVector2 closestPoint = exitPath[i] + t * toOrigin;

            // Calculate the distance from the robot to the closest point
            Real distance = (robotPos - closestPoint).Length();

            // Check if the robot is within the corridor radius
            if (distance <= corridorRadius) {
                return true;
            }
        }
    }

    return false;
}


argos::CVector2 CPFA_controller::OffsetToSide(const argos::CVector2& A, const argos::CVector2& B, Real offset, bool left) {
    argos::CVector2 direction = B - A;
    direction.Normalize();
    CRadians angle = left ? CRadians::PI_OVER_TWO : -CRadians::PI_OVER_TWO;
    argos::CVector2 normal(offset, angle);
    normal.Rotate(direction.Angle());
    return A + normal;
}



bool CPFA_controller::inCentralZone(){
	argos::CVector2 currentPosition = GetPosition();
	argos::CVector2 center = LoopFunctions->NestPosition;
	argos::Real distanceToCenter = (currentPosition - center).Length();

	// Check if the robot is within the central zone radius
	return distanceToCenter < 2.1;

}  



bool CPFA_controller::IsInsideCircleBoundary(const argos::CVector2& pos) {
    return (pos - m_cCircleCenter).Length() < m_fCircleRadius;
}

argos::CVector2 CPFA_controller::SafeTargetFromHeading(argos::CRadians base_angle, argos::Real step_size) {
    argos::CVector2 target_vec(step_size, base_angle);
    argos::CVector2 candidate = GetPosition() + target_vec;

    if(IsInsideCircleBoundary(candidate)) {
        // Flip angle and recompute
        argos::CVector2 avoid_vec(step_size, base_angle + argos::CRadians::PI);
        return GetPosition() + avoid_vec;
    }

    return candidate;
}


//make a function that will find the closest point on the path given the path as a parameter and return the index int
int CPFA_controller::FindClosestPointIndexOnPath(const std::vector<argos::CVector2>& path) {
	argos::CVector2 currentPosition = GetPosition();
	int closestIndex = 0;
	argos::Real minDistance = (currentPosition - path[0]).SquareLength();

	for (size_t i = 1; i < path.size(); ++i) {
		argos::Real distance = (currentPosition - path[i]).SquareLength();
		if (distance < minDistance) {
			minDistance = distance;
			closestIndex = i;
		}
	}
	return closestIndex + 1;
}



//make a function that will find closest point given the path as a parameter
argos::CVector2 CPFA_controller::FindClosestPointOnPath(const std::vector<argos::CVector2>& path) {
	argos::CVector2 currentPosition = GetPosition();
	argos::CVector2 closestPoint = path[0];
	argos::Real minDistance = (currentPosition - path[0]).SquareLength();

	for (size_t i = 1; i < path.size(); ++i) {
		argos::Real distance = (currentPosition - path[i]).SquareLength();
		if (distance < minDistance) {
			minDistance = distance;
			closestPoint = path[i];
		}
	}
	return closestPoint;
}

CVector2 CPFA_controller::FindClosestEntryPoint() {
    argos::CVector2 currentPosition = GetPosition();
    argos::CVector2 closestEntryPoint = entryPoints[0];
    argos::Real minDistance = (currentPosition - entryPoints[0]).SquareLength();

    for (size_t i = 1; i < entryPoints.size(); ++i) {
        argos::Real distance = (currentPosition - entryPoints[i]).SquareLength();
        if (distance < minDistance) {
            minDistance = distance;
            closestEntryPoint = entryPoints[i];
        }
    }
    // Set the closest entry point as the target
	return closestEntryPoint;
}

//make a function that finds the closest point on the path, takes the path as input and returns the index of the point in the path
int CPFA_controller::FindClosestForwardWaypoint(const std::vector<argos::CVector2>& path) {
    CVector2 currentPos = GetPosition();
    CVector2 nestPos(0.0, 0.0);  // Nest assumed at origin

    Real currentDistToNest = (currentPos - nestPos).SquareLength();
    int closestIndex = -1;
    Real minDistToRobot = std::numeric_limits<Real>::max();

    for (size_t i = 0; i < path.size(); ++i) {
        Real waypointDistToNest = (path[i] - nestPos).SquareLength();

        // Only consider waypoints that are farther from nest than current position
        if (waypointDistToNest > currentDistToNest) {
            Real distToRobot = (path[i] - currentPos).SquareLength();
            if (distToRobot < minDistToRobot) {
                minDistToRobot = distToRobot;
                closestIndex = i;
            }
        }
    }

    return closestIndex;  // Returns -1 if no valid forward waypoint found
}


CVector2 CPFA_controller::FindClosestNest() {
	argos::CVector2 currentPosition = GetPosition();
	argos::CVector2 closestNest = LoopFunctions->NestPositions[0];
	argos::Real minDistance = (currentPosition - LoopFunctions->NestPositions[0]).SquareLength();

	for (size_t i = 1; i < LoopFunctions->NestPositions.size(); ++i) {
		argos::Real distance = (currentPosition - LoopFunctions->NestPositions[i]).SquareLength();
		if (distance < minDistance) {
			minDistance = distance;
			closestNest = LoopFunctions->NestPositions[i];
		}
	}
	// Set the closest nest as the target
	return closestNest;
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
	// argos::LOG << GetId() << " Random search location set to: " << GetTarget() << std::endl;
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
		    //argos::LOG<<"LoopFunctions->FoodList size =" <<LoopFunctions->FoodList.size() << endl;
		    //argos::LOG<<"LoopFunctions->FoodColoringList size =" <<LoopFunctions->FoodColoringList.size() << endl;
 
	         for(i = 0; i < LoopFunctions->FoodList.size(); i++) {
		            
	            if((GetPosition() - LoopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance ) {
					// We found food! Calculate the nearby food density.
					 isHoldingFood = true;
                     CPFA_state = SURVEYING;
					 j = i + 1;
					 searchingTime+=SimulationTick()-startTime;
					 startTime = SimulationTick();
					 
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
