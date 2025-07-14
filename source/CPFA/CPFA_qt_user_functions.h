#ifndef CPFA_QT_USER_FUNCTIONS_H
#define CPFA_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/ray3.h>
#include <source/CPFA/CPFA_loop_functions.h>
#include <source/CPFA/CPFA_controller.h>

using namespace std;
using namespace argos;

class CPFA_loop_functions;

class CPFA_qt_user_functions : public argos::CQTOpenGLUserFunctions {

	public:

		CPFA_qt_user_functions();

		/* interface functions between QT and ARGoS */
		void DrawOnRobot(argos::CFootBotEntity& entity);
		void DrawOnArena(argos::CFloorEntity& entity);

	private:

		/* private helper drawing functions */
		void DrawNest();
		void DrawFood();
		void DrawFidelity();
		void DrawPheromones();
		void DrawTargetRays();
		void DrawEntryPoint();
		void DrawCircleOnArena();
		void DrawPaths();
		void DrawExitPath2();
		void DrawParallelExitPaths();
		void DrawConnectingLines();
		CPFA_loop_functions& loopFunctions;

		// std::vector<argos::CVector2> entryPath1 = {	
		// 	{ 1.60,  1.1}, { 1.60, -1.1},
		// 	{ 1.40, -1.1}, { 1.40,  0.9},
		// 	{ 1.20,  0.9}, { 1.20, -0.7},
		// 	{ 1.00, -0.7}, { 1.00,  0.50},
		// 	{ 0.80,  0.50}, { 0.80, -0.3},
		// 	{ 0.60, -0.3}, { 0.60,  0.0},
		// 	{ 0.30,  0.0}
		// };
		// std::vector<argos::CVector2> entryPath2 = {	
		// 	{ -1.60,  1.1}, { -1.60, -1.1},
		// 	{ -1.40, -1.1}, { -1.40,  0.9},
		// 	{ -1.20,  0.9}, { -1.20, -0.7},
		// 	{ -1.00, -0.7}, { -1.00,  0.50},
		// 	{ -0.80,  0.50}, { -0.80, -0.3},
		// 	{ -0.60, -0.3}, { -0.60,  0.0},
		// 	{ -0.30,  0.0}
		// };
		// std::vector<argos::CVector2> entryPath3 = {	
		// 	{  1.1,  1.60}, { -1.1,  1.60},
		// 	{ -1.1,  1.40}, {  0.9,  1.40},
		// 	{  0.9,  1.20}, { -0.7,  1.20},
		// 	{ -0.7,  1.00}, {  0.5,  1.00},
		// 	{  0.5,  0.80}, { -0.3,  0.80},
		// 	{ -0.3,  0.60}, {  0.0,  0.60},
		// 	{  0.0,  0.30}
		// };
		// std::vector<argos::CVector2> entryPath4 = {	
		// 	{  1.1, -1.60}, { -1.1, -1.60},
		// 	{ -1.1, -1.40}, {  0.9, -1.40},
		// 	{  0.9, -1.20}, { -0.7, -1.20},
		// 	{ -0.7, -1.00}, {  0.5, -1.00},
		// 	{  0.5, -0.80}, { -0.3, -0.80},
		// 	{ -0.3, -0.60}, {  0.0, -0.60},
		// 	{  0.0, -0.30}
		// };
		
		std::vector<argos::CVector2> entryPath1 = {
			{ 1.60,  1.1}, { 1.60, -1.1},
			{ 1.42, -1.1}, { 1.42,  .95},
			{ 1.24,  .95}, { 1.24, -0.75},
			{ 1.06, -0.75}, { 1.06,  0.64},
			{ 0.88,  0.64}, { 0.88, -0.45},
			{ 0.72, -0.45}, { 0.72,  0.31},
			{ 0.54,  0.31}, { 0.54, 0.0},
			{ 0.36, 0.0},
			{ 0.30,  0.0}
		};
		
		std::vector<argos::CVector2> entryPath2 = {
			{ -1.60, -1.1}, { -1.60,  1.1},
			{ -1.42,  1.1}, { -1.42, -0.95},
			{ -1.24, -0.95}, { -1.24,  0.75},
			{ -1.06,  0.75}, { -1.06, -0.64},
			{ -0.88, -0.64}, { -0.88,  0.45},
			{ -0.72,  0.45}, { -0.72, -0.31},
			{ -0.54, -0.31}, { -0.54,  0.0},
			{ -0.36,  0.0},
			{ -0.30,  0.0}
			
		};
		
		
		std::vector<argos::CVector2> entryPath3 = {
			{ -1.1,  1.60}, {  1.1,  1.60},
			{  1.1,  1.42}, { -0.95,  1.42},
			{ -0.95,  1.24}, {  0.75,  1.24},
			{  0.75,  1.06}, { -0.64,  1.06},
			{ -0.64,  0.88}, {  0.45,  0.88},
			{  0.45,  0.72}, { -0.31,  0.72},
			{ -0.31,  0.54}, { -0.0,  0.54},
			{ -0.0,  0.36},
			{ -0.0,  0.30}			
		};
	
		std::vector<argos::CVector2> entryPath4 = {
			{  1.1, -1.60}, { -1.1, -1.60},
			{ -1.1, -1.42}, {  0.95, -1.42},
			{  0.95, -1.24}, { -0.75, -1.24},
			{ -0.75, -1.06}, {  0.64, -1.06},
			{  0.64, -0.88}, { -0.45, -0.88},
			{ -0.45, -0.72}, {  0.31, -0.72},
			{  0.31, -0.54}, {  0.0, -0.54},
			{  0.0, -0.36},
			{  0.0, -0.30}
		};
			

		std::vector<argos::CVector2> exitPath1 = {{0.172, 0.172}, {1.4, 1.4}};
		std::vector<argos::CVector2> exitPath2 = {{-0.172, -0.172}, {-1.4, -1.4}};
		std::vector<argos::CVector2> exitPath3 = {{-0.172, 0.172}, {-1.4, 1.4}};
		std::vector<argos::CVector2> exitPath4 = {{0.172, -0.172}, {1.4, -1.4}};
};

#endif /* CPFA_QT_USER_FUNCTIONS_H */
