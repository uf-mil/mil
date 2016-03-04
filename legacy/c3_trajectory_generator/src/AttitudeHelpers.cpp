#include "AttitudeHelpers.h"

using namespace subjugator;
using namespace Eigen;
using namespace std;


Matrix3d AttitudeHelpers::EulerToRotation(const Vector3d& rpy)
{
	double sphi = sin(rpy(0));
	double cphi = cos(rpy(0));

	double stheta = sin(rpy(1));
	double ctheta = cos(rpy(1));

	double spsi = sin(rpy(2));
	double cpsi = cos(rpy(2));

	return (Matrix3d() <<
		cpsi*ctheta, -spsi*cphi + cpsi*stheta*sphi, spsi*sphi + cphi*cphi*stheta,
		spsi*ctheta, cpsi*cphi + sphi*stheta*spsi, -cpsi*sphi + stheta*spsi*cphi,
	        -stheta, ctheta*sphi, ctheta*cphi).finished();

}
