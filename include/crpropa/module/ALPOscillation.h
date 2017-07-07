#ifndef CRPROPA3_ALP_OSCILLATION_H
#define CRPROPA3_ALP_OSCILLATION_H

#include "crpropa/Module.h"
#include "crpropa/magneticField/MagneticField.h"

namespace crpropa {

class ALPOscillation : public Module {
private:
	ref_ptr<MagneticField> Bfield;
public:
	ALPOscillation(ref_ptr<MagneticField> _Bfield);
	void process(Candidate *candidate) const;
};

}

#endif /* CRPROPA3_ALP_OSCILLATION_H */
