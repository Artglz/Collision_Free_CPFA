#include "CPFA_qt_user_functions.h"

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
CPFA_qt_user_functions::CPFA_qt_user_functions() :
	loopFunctions(dynamic_cast<CPFA_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
	RegisterUserFunction<CPFA_qt_user_functions, CFootBotEntity>(&CPFA_qt_user_functions::DrawOnRobot);
	RegisterUserFunction<CPFA_qt_user_functions, CFloorEntity>(&CPFA_qt_user_functions::DrawOnArena);
}

void CPFA_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
	CPFA_controller& c = dynamic_cast<CPFA_controller&>(entity.GetControllableEntity().GetController());
	//draw the simulated camera view
	/*std::vector<argos::CVector2> points;
	points.push_back(CVector2(0,0));
	points.push_back(CVector2(0.71,0.71));
	
	points.push_back(CVector2(0.83,0.55));
	
	points.push_back(CVector2(0.92, 0.38));
	points.push_back(CVector2(0.98,0.2));
	points.push_back(CVector2(1,0));
	
	points.push_back(CVector2(0.98,-0.2));
	
	points.push_back(CVector2(0.92, -0.38));
	
	points.push_back(CVector2(0.83, -0.55));
	points.push_back(CVector2(0.71, -0.71));
	 */

/* 
 points.push_back(CVector2(1.065,1.065));
	
	points.push_back(CVector2(1.245,0.825));
	
	points.push_back(CVector2(1.38, 0.57));
	points.push_back(CVector2(1.47,0.3));
	points.push_back(CVector2(1.5,0));
	
	points.push_back(CVector2(1.47,-0.3));
	
	points.push_back(CVector2(1.38, -0.57));
	
	points.push_back(CVector2(1.245, -0.825));
	points.push_back(CVector2(1.065, -1.065));

 */
	//DrawPolygon(CVector3(0, 0, 0.002), CQuaternion(), points, argos::CColor::RED, false);
	//points.clear();
	
	if(c.IsHoldingFood()) {
		DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	}

	if(loopFunctions.DrawIDs == 1) {
		/* Disable lighting, so it does not interfere with the chosen text color */
		glDisable(GL_LIGHTING);
		/* Disable face culling to be sure the text is visible from anywhere */
		glDisable(GL_CULL_FACE);
		/* Set the text color */
		CColor cColor(CColor::BLACK);
		glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());

		/* The position of the text is expressed wrt the reference point of the footbot
		 * For a foot-bot, the reference point is the center of its base.
		 * See also the description in
		 * $ argos3 -q foot-bot
		 */
		
		// Disable for now
		//GetOpenGLWidget().renderText(0.0, 0.0, 0.5,             // position
		//			     entity.GetId().c_str()); // text
		
			DrawText(CVector3(0.0, 0.0, 0.3),   // position
            entity.GetId().c_str()); // text
		/* Restore face culling */
		glEnable(GL_CULL_FACE);
		/* Restore lighting */
		glEnable(GL_LIGHTING);
	}
}
 
void CPFA_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
	DrawFood();
	DrawFidelity();
	DrawPheromones();
	DrawNest();
	DrawEntryPoint();
	DrawCircleOnArena();

	DrawPaths();
    DrawParallelExitPaths();
    DrawConnectingLines();
	// DrawExitPath2();

	if(loopFunctions.DrawTargetRays == 1) DrawTargetRays();
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
// void CPFA_qt_user_functions::DrawNest() {
// 	/* 2d cartesian coordinates of the nest */
// 	Real x_coordinate = loopFunctions.NestPosition.GetX();
// 	Real y_coordinate = loopFunctions.NestPosition.GetY();

// 	/* required: leaving this 0.0 will draw the nest inside of the floor */
// 	Real elevation = loopFunctions.NestElevation;

// 	/* 3d cartesian coordinates of the nest */
// 	CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

// 	/* Draw the nest on the arena. */
// 	//DrawCircle(nest_3d, CQuaternion(), loopFunctions.NestRadius, CColor::RED);
//     DrawCylinder(nest_3d, CQuaternion(), loopFunctions.NestRadius, 0.008, CColor::GREEN);
// }

void CPFA_qt_user_functions::DrawNest() {
    for (const auto& nest_position : loopFunctions.NestPositions) {
        /* 2D cartesian coordinates of the nest */
        Real x_coordinate = nest_position.GetX();
        Real y_coordinate = nest_position.GetY();

        /* Required: leaving this 0.0 will draw the nest inside of the floor */
        Real elevation = loopFunctions.NestElevation;

        /* 3D cartesian coordinates of the nest */
        CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

        /* Draw the nest on the arena */
        DrawCylinder(nest_3d, CQuaternion(), loopFunctions.NestRadius, 0.008, CColor::GREEN);
    }
}

void CPFA_qt_user_functions::DrawExitPath2() {
    // Iterate over the points in exitPath2
    for (const auto& point : exitPath2) {
        // Get the coordinates of the point
        Real x_coordinate = point.GetX();
        Real y_coordinate = point.GetY();
        Real elevation = 0.01; // Slight elevation above the floor

        // Define the 3D position of the point
        CVector3 point_3d(x_coordinate, y_coordinate, elevation);

        // Draw the point as a cylinder with radius 0.25
        DrawCylinder(point_3d, CQuaternion(), 0.17, 0.01, CColor::BLUE);
    }
}

void CPFA_qt_user_functions::DrawPaths() {
    // Define colors for each path
    CColor path1Color = CColor::BLUE;
    CColor path2Color = CColor::BLUE;
    CColor path3Color = CColor::BLUE;
    CColor path4Color = CColor::BLUE;

    // Draw entryPath1
    for (size_t i = 0; i < entryPath1.size() - 1; ++i) {
        CRay3 ray(CVector3(entryPath1[i].GetX(), entryPath1[i].GetY(), 0.01),
                  CVector3(entryPath1[i + 1].GetX(), entryPath1[i + 1].GetY(), 0.01));
        DrawRay(ray, path1Color, 1.0);
    }

    // Draw entryPath2
    for (size_t i = 0; i < entryPath2.size() - 1; ++i) {
        CRay3 ray(CVector3(entryPath2[i].GetX(), entryPath2[i].GetY(), 0.01),
                  CVector3(entryPath2[i + 1].GetX(), entryPath2[i + 1].GetY(), 0.01));
        DrawRay(ray, path2Color, 1.0);
    }

    // Draw entryPath3
    for (size_t i = 0; i < entryPath3.size() - 1; ++i) {
        CRay3 ray(CVector3(entryPath3[i].GetX(), entryPath3[i].GetY(), 0.01),
                  CVector3(entryPath3[i + 1].GetX(), entryPath3[i + 1].GetY(), 0.01));
        DrawRay(ray, path3Color, 1.0);
    }

    // Draw entryPath4
    for (size_t i = 0; i < entryPath4.size() - 1; ++i) {
        CRay3 ray(CVector3(entryPath4[i].GetX(), entryPath4[i].GetY(), 0.01),
                  CVector3(entryPath4[i + 1].GetX(), entryPath4[i + 1].GetY(), 0.01));
        DrawRay(ray, path4Color, 1.0);
    }
}

void CPFA_qt_user_functions::DrawConnectingLines() {
    // Define color for connecting lines
    CColor connectingLineColor = CColor::GREEN;

    // Helper function to draw a line between two points
    auto drawLine = [&](const CVector2& start, const CVector2& end) {
        CRay3 ray(CVector3(start.GetX(), start.GetY(), 0.01),
                  CVector3(end.GetX(), end.GetY(), 0.01));
        DrawRay(ray, connectingLineColor, 1.0);
    };

    // Draw line from last entryPoint1 to first exitPath1
    if (!entryPath1.empty() && !exitPath1.empty()) {
        drawLine(entryPath1.back(), {0.205, 0.135});
    }

    // Draw line from last entryPoint2 to first exitPath2
    if (!entryPath2.empty() && !exitPath2.empty()) {
        drawLine(entryPath2.back(), {-0.205, -0.135});
    }

    // Draw line from last entryPoint3 to first exitPath3
    if (!entryPath3.empty() && !exitPath3.empty()) {
        drawLine(entryPath3.back(), {-0.135, 0.205});
    }

    // Draw line from last entryPoint4 to first exitPath4
    if (!entryPath4.empty() && !exitPath4.empty()) {
        drawLine(entryPath4.back(), { 0.135, -0.205});
    }
}



void CPFA_qt_user_functions::DrawParallelExitPaths() {
    // Define offset for parallel lines
    Real offset = 0.05;

    // Define color for parallel lines
    CColor parallelColor = CColor::GREEN;

    // Helper function to calculate offset points
    auto calculateOffsetPoint = [](const CVector2& point, const CVector2& direction, Real offset) {
        CVector2 mutableDirection = direction; // Create a mutable copy
        CVector2 normalizedDirection = mutableDirection.Normalize();
        CVector2 perpendicular(-normalizedDirection.GetY(), normalizedDirection.GetX());
        return point - perpendicular * offset; // Offset in the opposite direction
    };

    // Function to draw parallel lines for a given path
    auto drawParallelLines = [&](const std::vector<CVector2>& path) {
        for (size_t i = 0; i < path.size() - 1; ++i) {
            CVector2 direction = path[i + 1] - path[i];
            CVector2 offsetStart = calculateOffsetPoint(path[i], direction, offset);
            CVector2 offsetEnd = calculateOffsetPoint(path[i + 1], direction, offset);

            // Draw the parallel line
            CRay3 ray(CVector3(offsetStart.GetX(), offsetStart.GetY(), 0.01),
                      CVector3(offsetEnd.GetX(), offsetEnd.GetY(), 0.01));
            DrawRay(ray, parallelColor, 1.0);
        }
    };

    // Draw parallel lines for all exit paths
    drawParallelLines(exitPath1);
    drawParallelLines(exitPath2);
    drawParallelLines(exitPath3);
    drawParallelLines(exitPath4);
}


void CPFA_qt_user_functions::DrawCircleOnArena() {
    /* Define the center of the circle */
    Real x_coordinate = 0.0; // X-coordinate of the circle's center
    Real y_coordinate = 0.0; // Y-coordinate of the circle's center
    Real elevation = 0.01;   // Elevation above the floor

    // Real radius = 2.5; // Radius of the circle
    Real radius = 2; // Radius of the circle

    CColor circleColor = CColor::RED;

    DrawCircle(CVector3(x_coordinate, y_coordinate, elevation), CQuaternion(), radius, circleColor, false);

    CColor xColor = CColor::RED;

    /* Adjust the endpoints of the "X" to fit within the circle */
    Real diagonal_offset = radius * 0.7071; // sqrt(2)/2 ensures the endpoints lie within the circle

    /* Draw the "X" using two diagonal rays */
    CVector3 start1(x_coordinate - diagonal_offset, y_coordinate - diagonal_offset, elevation); // Bottom-left corner
    CVector3 end1(x_coordinate + diagonal_offset, y_coordinate + diagonal_offset, elevation);   // Top-right corner
    DrawRay(CRay3(start1, end1), xColor);

    CVector3 start2(x_coordinate - diagonal_offset, y_coordinate + diagonal_offset, elevation); // Top-left corner
    CVector3 end2(x_coordinate + diagonal_offset, y_coordinate - diagonal_offset, elevation);   // Bottom-right corner
    DrawRay(CRay3(start2, end2), xColor);
}

// draw entry point from cpfa controller
void CPFA_qt_user_functions::DrawEntryPoint() {
    /* Iterate over the entryPoints list in loopFunctions */
    for (const auto& entry_point : loopFunctions.entryPoints) {
        /* Get the coordinates of the entry point */
        Real x_coordinate = entry_point.GetX();
        Real y_coordinate = entry_point.GetY();
        Real elevation = loopFunctions.NestElevation; // Slight elevation above the floor

        /* Define the 3D position of the entry point */
        CVector3 entry_point_3d(x_coordinate, y_coordinate, elevation);

        /* Draw the entry point as a cylinder */
        DrawCylinder(entry_point_3d, CQuaternion(), 0.08, 0.008, CColor::ORANGE);
    }
}

void CPFA_qt_user_functions::DrawFood() {

	Real x, y;

	for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
		x = loopFunctions.FoodList[i].GetX();
		y = loopFunctions.FoodList[i].GetY();
		DrawCylinder(CVector3(x, y, 0.002), CQuaternion(), loopFunctions.FoodRadius, 0.025, loopFunctions.FoodColoringList[i]);
	}
 
	 //draw food in nests
	 /*for (size_t i=0; i< loopFunctions.CollectedFoodList.size(); i++)
	 { 
	        x = loopFunctions.CollectedFoodList[i].GetX();
	        y = loopFunctions.CollectedFoodList[i].GetY();
	        DrawCylinder(CVector3(x, y, 0.002), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	  } */ 
}

void CPFA_qt_user_functions::DrawFidelity() {

	   Real x, y;
        for(map<string, CVector2>::iterator it= loopFunctions.FidelityList.begin(); it!=loopFunctions.FidelityList.end(); ++it) {
            x = it->second.GetX();
            y = it->second.GetY();
            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::YELLOW);
        }
}

void CPFA_qt_user_functions::DrawPheromones() {

	Real x, y, weight;
	vector<CVector2> trail;
	CColor trailColor = CColor::GREEN, pColor = CColor::GREEN;

	    for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
		       x = loopFunctions.PheromoneList[i].GetLocation().GetX();
		       y = loopFunctions.PheromoneList[i].GetLocation().GetY();

		       if(loopFunctions.DrawTrails == 1) {
			          trail  = loopFunctions.PheromoneList[i].GetTrail();
			          weight = loopFunctions.PheromoneList[i].GetWeight();
                

             if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                 pColor = trailColor = CColor::GREEN;
             else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                 pColor = trailColor = CColor::YELLOW;
             else                                      // [   5.0% ,  0.0% ]
                 pColor = trailColor = CColor::RED;
      
             CRay3 ray;
             size_t j = 0;
      
             for(j = 1; j < trail.size(); j++) {
                 ray = CRay3(CVector3(trail[j - 1].GetX(), trail[j - 1].GetY(), 0.01),
		CVector3(trail[j].GetX(), trail[j].GetY(), 0.01));
                 
                 DrawRay(ray, trailColor, 1.0);
             }

	 DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
		       } 
         else {
			          weight = loopFunctions.PheromoneList[i].GetWeight();

             if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                 pColor = CColor::GREEN;
             else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                 pColor = CColor::YELLOW;
             else                                      // [   5.0% ,  0.0% ]
                 pColor = CColor::RED;
      
             DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
         }
 }
}

void CPFA_qt_user_functions::DrawTargetRays() {
	//size_t tick = loopFunctions.GetSpace().GetSimulationClock();
	//size_t tock = loopFunctions.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() / 8;

	//if(tock == 0) tock = 1;

	//if(tick % tock == 0) {
			
	for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
		DrawRay(loopFunctions.TargetRayList[j], loopFunctions.TargetRayColorList[j]);
	}
		
	//}	
}

/*
void CPFA_qt_user_functions::DrawTargetRays() {

	CColor c = CColor::BLUE;

	for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j],c);
	}

	//if(loopFunctions.SimTime % (loopFunctions.TicksPerSecond * 10) == 0) {
		// comment out for DSA, uncomment for CPFA
		loopFunctions.TargetRayList.clear();
	//}
}
*/

REGISTER_QTOPENGL_USER_FUNCTIONS(CPFA_qt_user_functions, "CPFA_qt_user_functions")
