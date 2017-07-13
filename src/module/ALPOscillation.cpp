
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
 * @brief Calculates the Delta term as described in the paper.
 * @param candidate The particle in question.
 * @param L The magnetic domain length [m].
 * @param ne The electron density [cm^-3].
 * @return The Delta term.
 */
static double getDelta(Candidate *candidate, double L, double ne) {
	return 0.053 * (ne / 0.001)
		* (1000 * eV / getAngularFrequency(candidate)) // Perhaps, omega means energy?
							// dimensions would make more sense that way.
							// * (1000 * eV) / (candidate->current.getEnergy())
		* (L / kpc);
}

/**
 * @brief Calculates the tan(2*theta) term as described in the paper.
 * @param candidate The particle in question.
 * @param bField The current magnetic field.
 * @param M The mass density [J].
 * @param ne The electron density [cm^-3].
 * @return The tan(2*theta) term.
 */
static double getTan2Theta(Candidate *candidate,
                           ref_ptr<MagneticField> bField,
                           double M, double ne) {
    return 2.8 *
            10 * TeV // 10^-3 * 10^-3 / 1 keV * 10^13 GeV = 10^13 eV = 10 * TeV
            / muG
            * getAngularFrequency(candidate)
            / ne
            / M
            * getTransverseBfield(candidate, bField);
}

/**
 * @brief Finds the probability that a gamma ray will oscillate into an axion-like particle.
 * @param candidate The particle in question.
 * @param bField The current magnetic field.
 * @param L The length of the magnetic field domain [m].
 * @param ne The electron density in [cm^-3].
 * @param M The mass density in [J]
 * @return The probability that the particle will oscillate. (0 to 1).
 */
static double getOscillationProbability(Candidate *candidate,
                                        ref_ptr<MagneticField> bField,
                                        double L, double ne, double M) {
    double tan2th = getTan2Theta(candidate, bField, M, ne);
    double theta2 = atan(tan2th);
    double Delta = getDelta(candidate, L, ne);
    double cosTheta2 = cos(theta2);
    double sinTheta2 = sin(theta2);
    return pow(sinTheta2, 2) * pow(sin(Delta / cosTheta2), 2);
}

ALPOscillation::ALPOscillation(ref_ptr<MagneticField> _Bfield,
		double _L,
		double _ne,
		double _M) :
	Bfield(_Bfield),
	L(_L),
	ne(_ne),
	M(_M)
{
}

void ALPOscillation::process(Candidate *candidate) const {
	// Find the trajectory length step.
	double lengthStep = candidate->getCurrentStep();
	double probability = getOscillationProbability(candidate, Bfield,
			L, ne, M);
	double stepProbability = lengthStep / L * probability;
	if (triggerWithProbability(stepProbability) == true) {
		candidate->setActive(false);
	}
}

}

