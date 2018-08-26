/*
 * RuleBasedGait.cpp
 *
 *  Created on: May 26, 2018
 *      Author: aron
 */

#include "RuleBasedGait.hpp"
#include "robot/RobotConstants.hpp"
#include <algorithm>
#include <arm_math.h>
#include <math.h>

using namespace robot::basic;

RuleBasedGait::RuleBasedGait(Leg* lFront, Leg* lMiddle, Leg* lBack, Leg* rFront, Leg* rMiddle, Leg* rBack):Gait(){
	this->legs.push_back(rFront);
	this->legs.push_back(rMiddle);
	this->legs.push_back(rBack);
	this->legs.push_back(lFront);
	this->legs.push_back(lMiddle);
	this->legs.push_back(lBack);
	this->initialBodyX = 0;
	this->initialBodyY = 0;
	this->currentBodyX = 0;
	this->currentBodyY = 0;
}

float RuleBasedGait::get_duty_cycle(){
	return this->dutyCycle;
}

uint16_t RuleBasedGait::get_step_size(){
	return this->stepSize;
}

uint16_t RuleBasedGait::get_step_height(){
	return this->stepHeight;
}

void RuleBasedGait::cycle(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	this->reset_recent_usage();
	while(this->body_is_at_desired_position(newXLeft, newYLeft) != true){
		std::sort(this->legs.begin(), this->legs.end(), [](Leg* a, Leg* b){return a->get_usage() < b->get_usage();});
		this->cycle_legs(newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	}
}

bool RuleBasedGait::body_is_at_desired_position(int x, int y){
	bool isAtDesiredX = false;
	bool isAtDesiredY = false;
	if(x < 0){
		isAtDesiredX = (this->currentBodyX <= (this->initialBodyX + x));
	}else if(x == 0){
		isAtDesiredX = true;
	}else if(x > 0){
		isAtDesiredX = (this->currentBodyX >= (this->initialBodyX + x));
	}
	if(y < 0){
		isAtDesiredY = (this->currentBodyY <= (this->initialBodyY + y));
	}else if(y == 0){
		isAtDesiredY = true;
	}else if(y > 0){
		isAtDesiredY = (this->currentBodyY >= (this->initialBodyY + y));
	}
	return (isAtDesiredX == true && isAtDesiredY == true);
}

void RuleBasedGait::reset_recent_usage(){
	for(auto& leg : this->legs){
		leg->reset_usage();
	}
}

void RuleBasedGait::cycle_legs(int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	LegInterpolation leg0 = this->get_leg_interpolation_situation(this->legs.at(0), newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	LegInterpolation leg1 = this->get_leg_interpolation_situation(this->legs.at(1), newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	LegInterpolation leg2 = this->get_leg_interpolation_situation(this->legs.at(2), newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	LegInterpolation leg3 = this->get_leg_interpolation_situation(this->legs.at(3), newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	LegInterpolation leg4 = this->get_leg_interpolation_situation(this->legs.at(4), newXRight, newYRight, newXLeft, newYLeft, bodyHeight);
	LegInterpolation leg5 = this->get_leg_interpolation_situation(this->legs.at(5), newXRight, newYRight, newXLeft, newYLeft, bodyHeight);

	int trajectoryStepIncrements = (this->incrementPerStep * trajectoryIncrements);
	int midTime = trajectoryStepIncrements/2;
	for(int i=0; i<trajectoryStepIncrements; i++){


		leg0.generate_current_angle((i/float(trajectoryStepIncrements)));
		leg1.generate_current_angle((i/float(trajectoryStepIncrements)));
		leg2.generate_current_angle((i/float(trajectoryStepIncrements)));
		leg3.generate_current_angle((i/float(trajectoryStepIncrements)));
		leg4.generate_current_angle((i/float(trajectoryStepIncrements)));
		leg5.generate_current_angle((i/float(trajectoryStepIncrements)));

		leg0.set_current_angles();
		leg1.set_current_angles();
		leg2.set_current_angles();
		leg3.set_current_angles();
		leg4.set_current_angles();
		leg5.set_current_angles();
	}
}

LegInterpolation RuleBasedGait::get_leg_interpolation_situation(Leg* leg, int newXRight, int newYRight, int newXLeft, int newYLeft, int bodyHeight){
	LegInterpolation legInterpolation = LegInterpolation(leg);
	int yAdded;
	int xAdded;
	int zAdded;
	bool isLeft = (leg->get_position() == LegPosition::LEFT_BACK ||
			leg->get_position() == LegPosition::LEFT_FRONT ||
			leg->get_position() == LegPosition::LEFT_MIDDLE);
	float anteriorDistanceRestrictedness = 1 - this->get_anterior_euclidean_distance_restrictedness(leg);
	float posteriorDistanceRestrictedness = 1 -  this->get_posterior_euclidean_distance_restrictedness(leg);
	int yDirection = newYRight/ abs(newYRight);

	if(leg->get_leg_phase() == LegPhase::SWING){
		leg->currentSwingIncrement ++;
		leg->currentStanceIncrement = 0;

		if(isLeft){
			yAdded =  (this->incrementPerStep * newYLeft);
			xAdded =  (this->incrementPerStep * newXLeft);
		}else{
			yAdded =  (this->incrementPerStep * newYRight);
			xAdded =  (this->incrementPerStep * newXRight);
		}
		zAdded = leg->currentSwingIncrement * (this->incrementPerStep * stepHeight);

		if(yDirection >=0){ //going forward
			if(anteriorDistanceRestrictedness < this->maxEuclideanRestrictednessThreshold){ //can move forward
				legInterpolation.generate_mid_via_and_end(xAdded, yAdded, (zAdded/2), (-bodyHeight + zAdded));
			}else if(anteriorDistanceRestrictedness >= this->maxEuclideanRestrictednessThreshold){//in swing but too restricted
				legInterpolation.generate_mid_via_and_end(0, 0, (bodyHeight+leg->get_z())/2, -bodyHeight);
				leg->update_leg_phase(LegPhase::SUPPORT);
			}

		}else if(yDirection < 0){ // going backwards
			if(posteriorDistanceRestrictedness < this->maxEuclideanRestrictednessThreshold){ //unrestricted
				legInterpolation.generate_mid_via_and_end(xAdded, yAdded, (zAdded/2), (-bodyHeight + zAdded));
			}else if(posteriorDistanceRestrictedness >= maxEuclideanRestrictednessThreshold){//cant move needs to set down
				legInterpolation.generate_mid_via_and_end(0, 0, (bodyHeight+leg->get_z())/2, -bodyHeight);
				leg->update_leg_phase(LegPhase::SUPPORT);
			}
		}
		if (leg->currentSwingIncrement >= this->emergencyCycleKill){
			legInterpolation.generate_mid_via_and_end(0, 0, (bodyHeight+leg->get_z())/2, -bodyHeight);
			leg->update_leg_phase(LegPhase::SUPPORT);
		}

	}else if(leg->get_leg_phase() == LegPhase::SUPPORT){
		leg->currentStanceIncrement++;
		leg->currentSwingIncrement = 0;
		if(isLeft){
			yAdded =  -this->distanceIncrement * yDirection;
			xAdded =  -this->distanceIncrement * (newXLeft/abs(newXLeft));
		}else{
			yAdded =  -this->distanceIncrement * yDirection;
			xAdded =  -this->distanceIncrement * (newXRight/abs(newXRight));
		}
		if (leg->currentStanceIncrement >= this->emergencyCycleKill){
			if(this->get_posterior_leg(leg)->get_leg_phase() == LegPhase::SUPPORT && this->get_anterior_leg(leg)->get_leg_phase() == LegPhase::SUPPORT){
				leg->update_leg_phase(LegPhase::SWING);
			}
			legInterpolation.generate_mid_via_and_end(xAdded, yAdded, 0, -bodyHeight);
		}
		if(yDirection >=0){
			if(posteriorDistanceRestrictedness < this->maxEuclideanRestrictednessThreshold){ // can retract leg
				legInterpolation.generate_mid_via_and_end(xAdded, yAdded, 0, -bodyHeight);
			}else if(posteriorDistanceRestrictedness >= this->maxEuclideanRestrictednessThreshold){//restricted can't retract
				if(this->get_posterior_leg(leg)->get_leg_phase() == LegPhase::SUPPORT && this->get_anterior_leg(leg)->get_leg_phase() == LegPhase::SUPPORT){
					leg->update_leg_phase(LegPhase::SWING);
				}
				legInterpolation.generate_mid_via_and_end(0, 0, 0, -bodyHeight);
			}

		}else if(yDirection < 0){
			if(anteriorDistanceRestrictedness < this->maxEuclideanRestrictednessThreshold){
				legInterpolation.generate_mid_via_and_end(xAdded, yAdded, 0, -bodyHeight);
			}else if(anteriorDistanceRestrictedness >= this->maxEuclideanRestrictednessThreshold){
				if(this->get_posterior_leg(leg)->get_leg_phase() == LegPhase::SUPPORT && this->get_anterior_leg(leg)->get_leg_phase() == LegPhase::SUPPORT){
					leg->update_leg_phase(LegPhase::SWING);
				}
				legInterpolation.generate_mid_via_and_end(xAdded, yAdded, 0, -bodyHeight);
			}
		}
	}
	legInterpolation.generate_trajectory_angles();
	return legInterpolation;
}

float RuleBasedGait::get_anterior_euclidean_distance_restrictedness(Leg* leg){
	Leg* anteriorLeg = this->get_anterior_leg(leg);
	float currentXDifference = this->get_anterior_current_x_difference(leg, anteriorLeg);
	float currentYDifference = this->get_anterior_current_y_difference(leg, anteriorLeg);
	float anteriorEuclideanDistance;
	arm_sqrt_f32((powf(currentXDifference,2) + powf(currentYDifference,2)), &anteriorEuclideanDistance);
	float anteriorEuclideanRestrictedness = this->solve_parameter_restrictedness(this->minEuclideanDistance, this->maxEuclideanDistance, anteriorEuclideanDistance);
	return anteriorEuclideanRestrictedness;
}

float RuleBasedGait::get_anterior_current_x_difference(Leg* leg, Leg* anteriorLeg){
	float currentXDistance;
	float constantXDistance = this->get_anterior_constant_x_distance(leg->get_position());
	if(leg->get_position() == LegPosition::LEFT_FRONT || leg->get_position() == LegPosition::RIGHT_FRONT){
		currentXDistance = leg->get_x() + constantXDistance + anteriorLeg->get_x();
	}else{
		currentXDistance = leg->get_x() + constantXDistance - anteriorLeg->get_x();
	}
	return currentXDistance;
}

float RuleBasedGait::get_anterior_current_y_difference(Leg* leg, Leg* anteriorLeg){
	float currentYDistance;
	float constantYDistance = this->get_anterior_constant_y_distance(leg->get_position());
	if(leg->get_position() == LegPosition::LEFT_FRONT || leg->get_position() == LegPosition::RIGHT_FRONT){
		currentYDistance = leg->get_y() - anteriorLeg->get_y();
	}else{
		currentYDistance = constantYDistance - leg->get_y() + anteriorLeg->get_y();
	}
	return currentYDistance;
}

float RuleBasedGait::get_anterior_constant_x_distance(LegPosition position){
	float xDistance;
	switch(position){
	case LegPosition::LEFT_FRONT:
		xDistance = this->frontXDistance;
		break;
	case LegPosition::RIGHT_FRONT:
		xDistance = this->frontXDistance;
		break;
	case LegPosition::LEFT_MIDDLE:
		xDistance = this->ipsilateralXDistance;
		break;
	case LegPosition::RIGHT_MIDDLE:
		xDistance = this->ipsilateralXDistance;
		break;
	case LegPosition::LEFT_BACK:
		xDistance = - this->ipsilateralXDistance;
		break;
	case LegPosition::RIGHT_BACK:
		xDistance = - this->ipsilateralXDistance;
		break;
	}
	return xDistance;
}

float RuleBasedGait::get_anterior_constant_y_distance(LegPosition position){
	float yDistance;
	switch(position){
	case LegPosition::LEFT_FRONT:
		yDistance = this->frontYDistance;
		break;
	case LegPosition::RIGHT_FRONT:
		yDistance = this->frontYDistance;
		break;
	case LegPosition::LEFT_MIDDLE:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::RIGHT_MIDDLE:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::LEFT_BACK:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::RIGHT_BACK:
		yDistance = this->ipsilateralYDistance;
		break;
	}
	return yDistance;
}

Leg* RuleBasedGait::get_anterior_leg(Leg* currentLeg){
	Leg* leg;
	switch(currentLeg->get_position()){
	case LegPosition::RIGHT_FRONT:
		leg = this->get_leg_with_position(LegPosition::LEFT_FRONT);
		break;
	case LegPosition::RIGHT_MIDDLE:
		leg = this->get_leg_with_position(LegPosition::RIGHT_FRONT);
		break;
	case LegPosition::RIGHT_BACK:
		leg = this->get_leg_with_position(LegPosition::RIGHT_MIDDLE);
		break;
	case LegPosition::LEFT_FRONT:
		leg = this->get_leg_with_position(LegPosition::RIGHT_FRONT);
		break;
	case LegPosition::LEFT_MIDDLE:
		leg = this->get_leg_with_position(LegPosition::LEFT_FRONT);
		break;
	case LegPosition::LEFT_BACK:
		leg = this->get_leg_with_position(LegPosition::LEFT_MIDDLE);
		break;
	}
	return leg;
}

Leg* RuleBasedGait::get_posterior_leg(Leg* currentLeg){
	Leg* leg;
	switch(currentLeg->get_position()){
	case LegPosition::RIGHT_FRONT:
		leg = this->get_leg_with_position(LegPosition::RIGHT_MIDDLE);
		break;
	case LegPosition::RIGHT_MIDDLE:
		leg = this->get_leg_with_position(LegPosition::RIGHT_BACK);
		break;
	case LegPosition::RIGHT_BACK:
		leg = this->get_leg_with_position(LegPosition::LEFT_BACK);
		break;
	case LegPosition::LEFT_FRONT:
		leg = this->get_leg_with_position(LegPosition::LEFT_MIDDLE);
		break;
	case LegPosition::LEFT_MIDDLE:
		leg = this->get_leg_with_position(LegPosition::LEFT_BACK);
		break;
	case LegPosition::LEFT_BACK:
		leg = this->get_leg_with_position(LegPosition::RIGHT_BACK);
		break;
	}
	return leg;
}

Leg* RuleBasedGait::get_leg_with_position(LegPosition position){
	Leg* returnLeg;
	for(auto& leg : this->legs){
		if(leg->get_position() == position){
			returnLeg = leg;
			break;
		}
	}
	return returnLeg;
}

float RuleBasedGait::get_posterior_euclidean_distance_restrictedness(Leg* leg){
	Leg* posteriorLeg = this->get_posterior_leg(leg);
	float currentXDifference = this->get_posterior_current_x_difference(leg, posteriorLeg);
	float currentYDifference = this->get_posterior_current_y_difference(leg, posteriorLeg);
	float posteriorEuclideanDistance;
	arm_sqrt_f32((powf(currentXDifference,2) + powf(currentYDifference,2)), &posteriorEuclideanDistance);
	float posteriorEuclideanRestrictedness = this->solve_parameter_restrictedness(this->minEuclideanDistance, this->maxEuclideanDistance, posteriorEuclideanDistance);
	return posteriorEuclideanRestrictedness;
}

float RuleBasedGait::get_posterior_current_x_difference(Leg* leg, Leg* posteriorLeg){
	float currentXDistance;
	float constantXDistance = this->get_posterior_constant_x_distance(leg->get_position());
	if(leg->get_position() == LegPosition::LEFT_BACK || leg->get_position() == LegPosition::RIGHT_BACK){
		currentXDistance = leg->get_x() + constantXDistance + posteriorLeg->get_x();
	}else{
		currentXDistance = leg->get_x() + constantXDistance - posteriorLeg->get_x();
	}
	return currentXDistance;
}

float RuleBasedGait::get_posterior_current_y_difference(Leg* leg, Leg* posteriorLeg){
	float currentYDistance;
	float constantYDistance = this->get_posterior_constant_y_distance(leg->get_position());
	if(leg->get_position() == LegPosition::LEFT_BACK || leg->get_position() == LegPosition::RIGHT_BACK){
		currentYDistance = leg->get_y() - posteriorLeg->get_y();
	}else{
		currentYDistance = constantYDistance + leg->get_y() - posteriorLeg->get_y();
	}
	return currentYDistance;
}

float RuleBasedGait::get_posterior_constant_x_distance(LegPosition position){
	float xDistance;
	switch(position){
	case LegPosition::LEFT_FRONT:
		xDistance = - this->ipsilateralXDistance;
		break;
	case LegPosition::RIGHT_FRONT:
		xDistance = - this->ipsilateralXDistance;
		break;
	case LegPosition::LEFT_MIDDLE:
		xDistance = this->ipsilateralXDistance;
		break;
	case LegPosition::RIGHT_MIDDLE:
		xDistance = this->ipsilateralXDistance;
		break;
	case LegPosition::LEFT_BACK:
		xDistance = this->frontXDistance;
		break;
	case LegPosition::RIGHT_BACK:
		xDistance = this->frontXDistance;
		break;
	}
	return xDistance;
}

float RuleBasedGait::get_posterior_constant_y_distance(LegPosition position){
	float yDistance;
	switch(position){
	case LegPosition::LEFT_FRONT:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::RIGHT_FRONT:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::LEFT_MIDDLE:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::RIGHT_MIDDLE:
		yDistance = this->ipsilateralYDistance;
		break;
	case LegPosition::LEFT_BACK:
		yDistance = this->frontYDistance;
		break;
	case LegPosition::RIGHT_BACK:
		yDistance = this->frontYDistance;
		break;
	}
	return yDistance;
}

float RuleBasedGait::solve_parameter_restrictedness(float minValue, float maxValue, float currentValue){
	float correctedCurrentValue = (currentValue == 0)? currentValue + 1 : currentValue;
	float epsilon = logf(this->smoothingFactor) / correctedCurrentValue;
	float restricted = expf(-epsilon * (currentValue - maxValue));
	return restricted;
}


