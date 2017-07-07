#ifndef CRPROPA3_ALP_OSCILLATION_H
#define CRPROPA3_ALP_OSCILLATION_H

#include "crpropa/Module.h"
#include "crpropa/magneticField/MagneticField.h"

namespace crpropa {

class ALPOscillation : public Module {
private:
	ref_ptr<MagneticField> Bfield;
	double L; // The magnetic domain length [m].
	double ne; // The electron density [cm^-3];
	double M; // The mass scale [J];
public:
	ALPOscillation(ref_ptr<MagneticField> _Bfield, double _L, double _ne, double _M);
	void process(Candidate *candidate) const;
};

}

#endif /* CRPROPA3_ALP_OSCILLATION_H */
