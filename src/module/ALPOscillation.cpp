
#include "crpropa/module/ALPOscillation.h"
#include "crpropa/Units.h"
#include <random>
#include <iostream>

namespace crpropa {

/**
 * @brief Returns true with probability p when called.
 * @param p The probability (0 to 1).
 * @return Whether the event was triggered.
 */
static bool triggerWithProbability(double p) {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	
	std::discrete_distribution<> d({p, 1 - p});
	int value = d(gen);
	return (value == 0);
}

/**
 * @brief Returns the transverse magnetic field in Teslas for a particle.
 * @param candidate The particle in question.
 * @param bField The current magnetic field.
 * @return The component of the magnetic field perpendicular to the trajectory of the particle.
 */
static double getTransverseBfield(Candidate *candidate, ref_ptr<MagneticField> bField) {
	Vector3d velocity = candidate->current.getVelocity();
	Vector3d field = bField->getField(candidate->current.getPosition()); 
	Vector3d perpendicular = velocity.getPerpendicularTo(field);
	return perpendicular.getR();
}

/**
 * @brief Returns the angular frequency in rad/s for a particle.
 * @param candidate The particle in question.
 * @return The angular frequency of the particle in rad / s.
 */
static double getAngularFrequency(Candidate *candidate) {
	static const double reducedPlanckConstant = 6.582119514e-16 * eV; // In J * s / rad
	double energy = candidate->current.getEnergy(); // In J
	return energy / reducedPlanckConstant; // J / (J * s / rad) -> rad / s
}

/**
 * @brief Finds the probability that a gamma ray will oscillate into an axion-like particle.
 * @param candidate The particle in question.
 * @param bField The current magnetic field.
 * @return The probability that the particle will oscillate. (0 to 1).
 */
static double getOscillationProbability(Candidate *candidate, ref_ptr<MagneticField> bField) {
	return 0.0;
}

ALPOscillation::ALPOscillation(ref_ptr<MagneticField> _Bfield) :
	Bfield(_Bfield) {
}

void ALPOscillation::process(Candidate *candidate) const {
	static double L = 500.0 * Mpc; // Chance of disappearance in 500 Mpc.
	// Find the trajectory length step.
	double lengthStep = candidate->getCurrentStep();
	double probability = getOscillationProbability(candidate, Bfield);
	double stepProbability = lengthStep / L * probability;
	if (triggerWithProbability(stepProbability) == true) {
		candidate->setActive(false);
	}
}

};

