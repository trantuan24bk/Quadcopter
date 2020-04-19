//heading -> Yaw
//attitude -> Pitch
//bank -> Roll
#include <math.h>
const float PI = 3.14159265358979323846;
extern float Roll,Pitch,Yaw;
void QuatoE_Math(float x,float y,float z,float w) {
    float sqw = w*w;
    float sqx = x*x;
    float sqy = y*y;
    float sqz = z*z;
		float heading,attitude,bank;

	float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	float test = x*y + w*z;
	if (test > 0.499*unit) { // singularity at north pole
		heading = 2 * atan2(x,w);
		attitude = PI/2;
		bank = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		heading = -2 * atan2(x,w);
		attitude = -PI/2;
		bank = 0;
		return;
	}
  heading = atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw);
	attitude = asin(2*test/unit);
	bank = atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw);
	Yaw=heading*57.295779513082320876798154814105f;
	Roll = bank*57.295779513082320876798154814105f;
	Pitch = attitude*57.295779513082320876798154814105f;
}
void quaternion2EulerRad(float q0,float q1,float q2,float q3)
{
float gx, gy, gz; // estimated gravity direction
float q[4];
float ypr[3];
q[0]=q0;q[1]=q1;q[2]=q2;q[3]=q3;
gx = 2 * (q[1]*q[3] - q[0]*q[2]);
gy = 2 * (q[0]*q[1] + q[2]*q[3]);
gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
ypr[1] = atan2(gx, sqrt(gy*gy + gz*gz));
ypr[2] = atan2(gy, sqrt(gx*gx + gz*gz));

Yaw = ypr[0]; // yaw
Pitch = ypr[1]; // pitch
Roll = ypr[2]; // roll

}