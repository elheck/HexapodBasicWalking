/*
 * RobotConstants.hpp
 *
 *  Created on: Mar 6, 2018
 *      Author: aron
 */

#ifndef ROBOT_ROBOTCONSTANTS_HPP_
#define ROBOT_ROBOTCONSTANTS_HPP_

#include <stdint.h>

namespace robot{

	namespace body{

		static constexpr float32_t coxaLength = 52.0; //mm
		static constexpr float32_t femurLength = 65.0; //mm
		static constexpr float32_t tibiaLength = 135.0; //mm
		static constexpr float32_t leftFrontXDistance = 60.0;//mm
		static constexpr float32_t leftFrontYDistance = 120.0;//mm
		static constexpr float32_t leftMiddleXDistance = 100.0;//mm
		static constexpr float32_t leftMiddleYDistance = 0.0;//mm
		static constexpr float32_t leftBackXDistance = 60.0;//mm
		static constexpr float32_t leftBackYDistance = 120.0;//mm
		static constexpr float32_t rightFrontXDistance = 60.0;//mm
		static constexpr float32_t rightFrontYDistance = 120.0;//mm
		static constexpr float32_t rightMiddleXDistance = 100.0;//mm
		static constexpr float32_t rightMiddleYDistance = 0.0;//mm
		static constexpr float32_t rightBackXDistance = 60.0;//mm
		static constexpr float32_t rightBackYDistance = 120.0;//mm
		static constexpr float32_t leftFrontAngleOffset = 135.0;//degree
		static constexpr float32_t leftMiddleAngleOffset = 180.0;//degree
		static constexpr float32_t leftBackAngleOffset = 225.0;//degree
		static constexpr float32_t rightFrontAngleOffset = 45.0;//degree
		static constexpr float32_t rightMiddleAngleOffset = 0.0;//degree
		static constexpr float32_t rightBackAngleOffset = 315.0;//degree
	}

	namespace motor{
		static constexpr uint16_t leftCoxaMaxAngle = 270;
		static constexpr uint16_t leftCoxaMinAngle = 60;
		static constexpr uint16_t leftFemurMaxAngle = 250;
		static constexpr uint16_t leftFemurMinAngle = 50;
		static constexpr uint16_t leftTibiaMaxAngle = 270;
		static constexpr uint16_t leftTibiaMinAngle = 60;
		static constexpr uint16_t rightCoxaMaxAngle = 240;
		static constexpr uint16_t rightCoxaMinAngle = 30;
		static constexpr uint16_t rightFemurMaxAngle = 250;
		static constexpr uint16_t rightFemurMinAngle = 50;
		static constexpr uint16_t rightTibiaMaxAngle = 280;
		static constexpr uint16_t rightTibiaMinAngle = 60;
		static constexpr int16_t rightCoxaOffsetAngle = 150;
		static constexpr int16_t leftCoxaOffsetAngle = 150;
		static constexpr int16_t rightFemurOffsetAngle = 150;
		static constexpr int16_t leftFemurOffsetAngle = 150;
		static constexpr int16_t rightTibiaOffsetAngle = 15;
		static constexpr int16_t leftTibiaOffsetAngle = 285;
	}

	namespace basic{
		static constexpr uint16_t seventhOrderMatrixDimension = 7*7;
		static constexpr uint16_t seventhOrderVectorDimension = 7;
		static constexpr uint16_t seventhOrderM = 7;
		static constexpr uint16_t seventhOrderN = 7;
		static constexpr uint16_t thirdOrderVectorDimension = 3;
		static constexpr uint16_t thirdOrderMatrixDimension = 3*3;
		static constexpr uint16_t thirdOrderM = 3;
		static constexpr uint16_t thirdOrderN = 3;
		static constexpr uint16_t rotationMaxAngle = 30;
		static constexpr uint16_t trajectoryIncrements = 80;
		static constexpr int startXAir = 150;
		static constexpr int startYAir = 0;
		static constexpr int startZAir = 100;
		static constexpr int startXGround = 110;
		static constexpr int startYGround = 0;
		static constexpr int startZGround = -100;
		static constexpr int maxX = 240;
		static constexpr int minX = 100;
		static constexpr int maxY = 30;
		static constexpr int minY = -30;
	}
}


#endif /* ROBOT_ROBOTCONSTANTS_HPP_ */
