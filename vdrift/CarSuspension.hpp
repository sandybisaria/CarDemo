#pragma once

#include "MathVector.hpp"
#include "LinearInterp.hpp"

class CarSuspension {
public:
	// Default makes an S2000-like car
	CarSuspension()
		: springConstant(50000.0), bounce(2588), rebound(2600), travel(0.19),
		  antiRollK(8000), damperFactors(1), springFactors(1),
		  camber(-0.5), caster(0.28), toe(0),
		  overTravel(0), displacement(0), velocity(0), force(0) {}

	double getAntiRollK() const { return antiRollK; }
	void setAntiRollK(double antiRollK) { this->antiRollK = antiRollK; }

	double getBounce() const { return bounce; }
	void setBounce(double bounce) { this->bounce = bounce; }

	double getCamber() const { return camber; }
	void setCamber(double camber) { this->camber = camber; }

	double getCaster() const { return caster; }
	void setCaster(double caster) { this->caster = caster; }

	double getDisplacement() const { return displacement; }
	double getDisplacementPercent() const { return displacement / travel; } // 0.0 = fully extended; 1.0 = fully compressed

	double getForce() const { return force; }
	const double getForce(double displacement, double velocity) {
		double damping = bounce;
		if (velocity < 0)
			damping = rebound;

		double dampFactor = damperFactors.interpolate(std::abs(velocity));
		double springFactor = springFactors.interpolate(displacement);

		// Compressed -> spring force pushes car in positive z direction
		double springForce = -displacement * springConstant * springFactor;
		// Increasing compression -> damp force pushes car in positive z direction;
		double dampForce = -velocity * damping * dampFactor;

		double force = springForce + dampForce;
		return force;
	}

	const MathVector<double, 3>& getHinge() const { return hinge; }
	void setHinge(const MathVector<double, 3>& hinge) { this->hinge = hinge; }

	double getOverTravel() const { return overTravel; }

	double getRebound() const { return rebound; }
	void setRebound(double rebound) { this->rebound = rebound; }

	double getSpringConstant() const { return springConstant; }
	void setSpringConstant(double springConstant) { this->springConstant = springConstant; }

	double getToe() const { return toe; }
	void setToe(double toe) { this->toe = toe; }

	double getTravel() const { return travel; }
	void setTravel(double travel) { this->travel = travel; }

	double getVelocity() const { return velocity; }

	// Compute suspension force for given time interval and external displacement
	double update(double dt, double extDisplacement) {
		overTravel = extDisplacement - travel;
		overTravel = std::max(overTravel, 0.);

		extDisplacement = std::max(std::min(extDisplacement, travel), 0.);
		double newDisplacement = extDisplacement;

		velocity = (newDisplacement - displacement) / dt;
		velocity = std::max(std::min(velocity, 5.), -5.);

		displacement = newDisplacement;
		force = getForce(displacement, velocity);

		return -force;
	}

	void setDamperFactorPoints(std::vector<std::pair<double, double> >& curve) {
		for (std::vector<std::pair<double, double> >::iterator i = curve.begin(); i != curve.end(); ++i) {
			damperFactors.addPoint(i->first, i->second);
		}
	}
	void setSpringFactorPoints(std::vector<std::pair<double, double> >& curve) {
		for (std::vector<std::pair<double, double> >::iterator i = curve.begin(); i != curve.end(); ++i) {
			springFactors.addPoint(i->first, i->second);
		}
	}

	// Variables
	double overTravel; // Amount past travel that suspension was requested to compress
	double displacement; // Linear repr of suspension displacement, actually the displacement about the arc formed by the hinge
	double velocity;
	double force;

private:
	// Constants
	MathVector<double, 3> hinge; // Point that wheels are rotated around as suspension compresses
	double springConstant;
	double bounce; // Suspension compression damping
	double rebound; // Suspension decompression damping
	double travel; // How far suspension can travel from zero-g fully-extended position around hinge arc before wheel travel is stopped
	double antiRollK; // Spring constant for anti-roll bar
	LinearInterp<double> damperFactors;
	LinearInterp<double> springFactors;

	double camber; // Camber angle in degrees
	double caster; // Caster angle in degrees
	double toe; // Toe angle in degrees
};
