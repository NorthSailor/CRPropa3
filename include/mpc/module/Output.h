#ifndef OUTPUT_H_
#define OUTPUT_H_

#include "mpc/Module.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

namespace mpc {

std::string getOutputString(ParticleState particle) {
	std::stringstream ss;
	ss << particle.getId() << ", ";
	ss << particle.getEnergy() / EeV << ", ";
	Vector3 pos = particle.getPosition() / Mpc;
	ss << pos.x() << ", ";
	ss << pos.y() << ", ";
	ss << pos.z() << ", ";
	Vector3 dir = particle.getDirection();
	ss << dir.x() << ", ";
	ss << dir.y() << ", ";
	ss << dir.z() << "\n";
	return ss.str();
}

/**
 @class TrajectoryOutput
 @brief Saves trajectories to CSV file.
 */
class TrajectoryOutput: public Module {
private:
	std::ofstream outfile;

public:
	std::string getDescription() const {
		return "TrajectoryOutput";
	}

	TrajectoryOutput(std::string name) {
		outfile.open(name.c_str());
		outfile << "# Age, HepId, E, posX, posY, posZ, dirX, dirY, dirZ\n";
	}

	~TrajectoryOutput() {
		outfile.close();
	}

	void process(Candidate *candidate) {
		outfile << candidate->getTrajectoryLength() / Mpc << ", "
				<< getOutputString(candidate->current);
	}
};

/**
 @class FinishedOutput
 @brief Saves finished particles to a CSV file.
 */
class FinishedOutput: public Module {
private:
	std::ofstream outfile;

public:
	std::string getDescription() const {
		return "FinishedOutput";
	}

	FinishedOutput(std::string name) {
		outfile.open(name.c_str());
		outfile
				<< "# initial: Age, HepId, E, posX, posY, posZ, dirX, dirY, dirZ\n";
		outfile
				<< "# final: Age, HepId, E, posX, posY, posZ, dirX, dirY, dirZ\n";
	}

	~FinishedOutput() {
		outfile.close();
	}

	void process(Candidate *candidate) {
		if (candidate->getStatus() == Candidate::Active)
			return;
//		// initial state
		outfile << "0, ";
		outfile << getOutputString(candidate->initial);
//		// final state
		outfile << candidate->getTrajectoryLength() / Mpc << ", ";
		outfile << getOutputString(candidate->current);
	}
};

/**
 @class ShellOutput
 @brief Output of the candidate to the shell.
 */
class ShellOutput: public Module {
public:
	std::string getDescription() const {
		return "ShellOutput";
	}

	void process(Candidate *candidate) {
		std::cout << std::fixed << std::showpoint << std::setprecision(2)
				<< std::setw(6);
		std::cout << candidate->getTrajectoryLength() / Mpc << " Mpc,  ";
		std::cout << candidate->current.getId() << ",  ";
		std::cout << candidate->current.getEnergy() / EeV << " EeV,  ";
		std::cout << candidate->current.getPosition() / Mpc << " Mpc";
		std::cout << std::endl;
	}
};

} // namespace mpc

#endif /* OUTPUT_H_ */
