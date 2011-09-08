// CustomRayCastCar.cpp: implementation of the CustomRayCastCar class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "./CustomRayCastCar.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CustomRayCastCar::CustomRayCastCar(int maxTireCount, const dMatrix& cordenateSytem, NewtonBody* carBody, const dVector& gravity)
	:NewtonCustomJoint(0, carBody, NULL), m_gravity (gravity)
{
	dVector com;
	dMatrix tmp;

	m_tiresRollSide = 0;

	// set the chassis matrix at the center of mass
	NewtonBodyGetCentreOfMass(m_body0, &com[0]);
	com.m_w = 1.0f;

	// set the joint reference point at the center of mass of the body
	dMatrix chassisMatrix (cordenateSytem);
//	chassisMatrix.m_posit += chassisMatrix.RotateVector(com);
//	CalculateLocalMatrix (chassisMatrix, m_localFrame, tmp);
	chassisMatrix.m_posit += com;
	m_localFrame = chassisMatrix;


	// allocate space for the tires;
	m_tiresCount = 0;
	m_tires = new Tire[maxTireCount];

	m_curSpeed = 0.0f;
	m_aerodynamicDrag = 0.1f;
	m_aerodynamicDownForce = 0.1f;

	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBodyGetMassMatrix(m_body0, &m_mass, &Ixx, &Iyy, &Izz);

	// TODO uncomment
	m_gravity.m_x *= m_mass;
	m_gravity.m_y *= m_mass;
	m_gravity.m_z *= m_mass;
	// register the callback for tire integration
//	NewtonUserJointSetFeedbackCollectorCallback (m_joint, IntegrateTires);
}


CustomRayCastCar::~CustomRayCastCar()
{
	NewtonWorld *world;

	world = NewtonBodyGetWorld (m_body0);
	for (int i = 0; i < m_tiresCount; i ++) {
		NewtonReleaseCollision (world, m_tires[i].m_shape);
	}

	if(m_tires) {
		delete[] m_tires;
	}
}


int CustomRayCastCar::GetTiresCount() const
{
	return m_tiresCount;
}




void CustomRayCastCar::GetInfo (NewtonJointRecord* info) const
{
}

//this function is to be overloaded by a derive class
void CustomRayCastCar::SetSteering (dFloat angle)
{
}

//this function is to be overloaded by a derive class
void CustomRayCastCar::SetBrake (dFloat torque)
{
}

//this function is to be overloaded by a derive class
void CustomRayCastCar::SetTorque (dFloat torque)
{
}

dFloat CustomRayCastCar::GetSpeed() const
{
	return m_curSpeed;
}


void CustomRayCastCar::SetTireMaxRPS (int tireIndex, dFloat maxTireRPS)
{
	m_tires[tireIndex].m_maxTireRPS = maxTireRPS;
}

CustomRayCastCar::Tire& CustomRayCastCar::GetTire (int index) const
{
	return m_tires[index];
}

dFloat CustomRayCastCar::GetParametricPosition (int index) const
{
	return m_tires[index].m_posit / m_tires[index].m_suspensionLength;
}

void CustomRayCastCar::SetTireSteerAngle (int index, dFloat angle, dFloat turnforce)
{
   m_tires[index].m_steerAngle = angle;
   m_tires[index].m_localAxis.m_z = dCos (angle);
   m_tires[index].m_localAxis.m_x = dSin (angle);
	if (m_tiresRollSide==0) {
	  m_tires[index].m_turnforce = turnforce;
	} else {
	  m_tires[index].m_turnforce = -turnforce;
	}
}

void CustomRayCastCar::SetTireTorque (int index, dFloat torque)
{
//torque=-600.0f;
	m_tires[index].m_torque = torque;
}

void CustomRayCastCar::SetTireBrake (int index, dFloat torque)
{
	m_tires[index].m_breakTorque = torque;
}


void CustomRayCastCar::AddSingleSuspensionTire (
	void *userData,
	const dVector& localPosition,
	dFloat mass,
	dFloat radius,
	dFloat width,
	dFloat suspensionLength,
	dFloat springConst,
	dFloat springDamper,
	int castMode)
{
	// calculate the tire local base pose matrix
	dMatrix bodyMatrix;
	m_tires[m_tiresCount].m_contactPoint = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	m_tires[m_tiresCount].m_tireAxelPosit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	m_tires[m_tiresCount].m_localAxelPosit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	m_tires[m_tiresCount].m_tireAxelVeloc = dVector (0.0f, 0.0f, 0.0f, 1.0f);
	m_tires[m_tiresCount].m_torque = 0.0f;
	m_tires[m_tiresCount].m_turnforce = 0.0f;
	m_tires[m_tiresCount].m_harpoint = m_localFrame.UntransformVector(localPosition);
	m_tires[m_tiresCount].m_localAxis = m_localFrame.UnrotateVector(dVector (0.0f, 0.0f, 1.0f, 0.0f));
	m_tires[m_tiresCount].m_localAxis.m_w = 0.0f;
	m_tires[m_tiresCount].m_userData = userData;
	m_tires[m_tiresCount].m_angularVelocity = 0.0f;
	m_tires[m_tiresCount].m_spinAngle = 0.0f;
	m_tires[m_tiresCount].m_steerAngle = 0.0f;

	m_tires[m_tiresCount].m_posit = suspensionLength;
	m_tires[m_tiresCount].m_suspensionLength = suspensionLength;
	m_tires[m_tiresCount].m_tireLoad = 0.0f;
	m_tires[m_tiresCount].m_breakTorque = 0.0f;
	m_tires[m_tiresCount].m_localSuspentionSpeed = 0.0f;

	m_tires[m_tiresCount].m_springConst = springConst;
	m_tires[m_tiresCount].m_springDamper = springDamper;
	m_tires[m_tiresCount].m_groundFriction = 2.0f;

	m_tires[m_tiresCount].m_tireUseConvexCastMode = castMode;
//	m_tires[m_tiresCount].m_tireJacobianRowIndex = -1;

	// make a convex shape to represent the tire collision
	#define TIRE_SHAPE_SIZE 12
	dVector shapePoints[TIRE_SHAPE_SIZE * 2];
	for (int i = 0; i < TIRE_SHAPE_SIZE; i ++) {
		shapePoints[i].m_x = -width * 0.5f;
		shapePoints[i].m_y = radius * dCos (2.0f * 3.1416 * dFloat(i)/ dFloat(TIRE_SHAPE_SIZE));
		shapePoints[i].m_z = radius * dSin (2.0f * 3.1416 * dFloat(i)/ dFloat(TIRE_SHAPE_SIZE));
		shapePoints[i + TIRE_SHAPE_SIZE].m_x = -shapePoints[i].m_x;
		shapePoints[i + TIRE_SHAPE_SIZE].m_y = shapePoints[i].m_y;
		shapePoints[i + TIRE_SHAPE_SIZE].m_z = shapePoints[i].m_z;
	}
	m_tires[m_tiresCount].m_shape = NewtonCreateConvexHull (m_world, TIRE_SHAPE_SIZE * 2, &shapePoints[0].m_x, sizeof (dVector), 0.0f, 0, NULL);

	// calculate the tire geometrical parameters
	m_tires[m_tiresCount].m_radius = radius;
//	m_tires[m_tiresCount].m_radiusInv  = 1.0f / m_tires[m_tiresCount].m_radius;
	m_tires[m_tiresCount].m_mass = mass;
	m_tires[m_tiresCount].m_massInv = 1.0f / m_tires[m_tiresCount].m_mass;
	m_tires[m_tiresCount].m_Ixx = mass * radius * radius / 2.0f;
	m_tires[m_tiresCount].m_IxxInv = 1.0f / m_tires[m_tiresCount].m_Ixx;
	SetTireMaxRPS (m_tiresCount, 150.0f / radius);

	m_tiresCount ++;
}


const dMatrix& CustomRayCastCar::GetChassisMatrixLocal () const
{
	return m_localFrame;
}

dMatrix CustomRayCastCar::CalculateSuspensionMatrix (int tireIndex, dFloat distance) const
{
	const Tire& tire = m_tires[tireIndex];

	dMatrix matrix;
	// calculate the steering angle matrix for the axis of rotation
	matrix.m_front = tire.m_localAxis;
	matrix.m_up    = dVector (0.0f, 1.0f, 0.0f, 0.0f);
	matrix.m_right = dVector (-tire.m_localAxis.m_z, 0.0f, tire.m_localAxis.m_x, 0.0f);
	matrix.m_posit = tire.m_harpoint - m_localFrame.m_up.Scale (distance);
	return matrix;
}

dMatrix CustomRayCastCar::CalculateTireMatrix (int tireIndex) const
{
	const Tire& tire = m_tires[tireIndex];

	// calculate the rotation angle matrix
	dMatrix angleMatrix (dPitchMatrix(tire.m_spinAngle));

	// get the tire body matrix
	dMatrix bodyMatrix;
	NewtonBodyGetMatrix(m_body0, &bodyMatrix[0][0]);
	return angleMatrix * CalculateSuspensionMatrix (tireIndex, tire.m_posit) * m_localFrame * bodyMatrix;

}


unsigned CustomRayCastCar::ConvexCastPrefilter(const NewtonBody* body, const NewtonCollision* collision, void* userData)
{
	NewtonBody* me;
	me = (NewtonBody*) userData;
	// do no cast myself
	return (me != body);
}


void CustomRayCastCar::CalculateTireCollision (Tire& tire, const dMatrix& suspensionMatrixInGlobalSpace) const
{
  // make a data structure to collect the information returned by the ray cast
  struct RayCastInfo
  {
    RayCastInfo(const NewtonBody* body)
	{
	  m_param = 1.0f;
	  m_me = body;
	  m_hitBody = NULL;
	}
	static dFloat RayCast (const NewtonBody* body, const dFloat* normal, int collisionID, void* userData, dFloat intersetParam)
	{
	  RayCastInfo& caster = *((RayCastInfo*) userData);
	  // if this body is not the vehicle, see if a close hit
	  if (body != caster.m_me) {
	    if (intersetParam < caster.m_param) {
	      // this is a close hit, record the information.
		  caster.m_param = intersetParam;
		  caster.m_hitBody = body;
		  caster.m_contactID = collisionID;
		  caster.m_normal = dVector (normal[0], normal[1], normal[2], 1.0f);
		}
	}
	return intersetParam;
  }
    dFloat m_param;
    dVector m_normal;
    const NewtonBody* m_me;
    const NewtonBody* m_hitBody;
    int m_contactID;
  };
  RayCastInfo info (m_body0);
  // extend the ray by the radius of the tire
  dFloat dist (tire.m_suspensionLength + tire.m_radius);
  dVector destination (suspensionMatrixInGlobalSpace.TransformVector(m_localFrame.m_up.Scale (-dist)));
  // cast a ray to the world ConvexCastPrefilter
  NewtonWorldRayCast(m_world, &suspensionMatrixInGlobalSpace.m_posit[0], &destination[0], RayCastInfo::RayCast, &info, &ConvexCastPrefilter);
  // if the ray hit something, it means the tire has some traction
  if (info.m_hitBody) {
    dFloat intesectionDist;

  tire.m_contactPoint = suspensionMatrixInGlobalSpace.m_posit + (destination - suspensionMatrixInGlobalSpace.m_posit).Scale (info.m_param);
  tire.m_contactNormal = info.m_normal;
  // TO DO: get the material properties for tire frictions on different roads

  intesectionDist = dist * info.m_param - tire.m_radius;
  if (intesectionDist > tire.m_suspensionLength) {
    intesectionDist = tire.m_suspensionLength;
  } else if (intesectionDist < dFloat (0.0f)) {
    intesectionDist = dFloat (0.0f);
  }
  tire.m_posit = intesectionDist;
  switch (info.m_contactID)
  {
    case 0:
	{
	  // normal ground friction
	  tire.m_groundFriction = 3.0f;
	  break;
	}
	default:
	{
	  // default ground friction
	  tire.m_groundFriction = 3.0f;
	  break;
	}
  }
  } else {
    tire.m_posit = tire.m_suspensionLength;
	tire.m_groundFriction = 0.0f;
  }
}

void CustomRayCastCar::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dFloat invTimestep;
	dMatrix bodyMatrix;
	dMatrix suspensionMatrices[VEHICLE_MAX_TIRE_COUNT];

	// get the simulation time
	invTimestep = 1.0f / timestep ;

	// get the vehicle global matrix, and use it in several calculations
	NewtonBodyGetMatrix(m_body0, &bodyMatrix[0][0]);
	dMatrix chassisMatrix (m_localFrame * bodyMatrix);

	// calculate all suspension matrices in global space and tire collision
	for (int i = 0; i < m_tiresCount; i ++) {
		Tire& tire = m_tires[i];
		dMatrix suspensionMatrix = suspensionMatrices[i];

		// calculate this suspension matrix and save it for future used
		suspensionMatrix = CalculateSuspensionMatrix (i, 0.0f) * chassisMatrix;

		// calculate the tire collision
		CalculateTireCollision (tire, suspensionMatrix);
	}


	// calculate all suspension forces due to spring and damper
	dVector m_chassisForce (0.0f, 0.0f, 0.0f, 0.0f);
	dVector m_chassisTorque (0.0f, 0.0f, 0.0f, 0.0f);

	// get the chassis instantaneous linear and angular velocity in the local space of the chassis
	int longitidunalForceIndex;
	longitidunalForceIndex = 0;
	NewtonBodyGetVelocity(m_body0, &m_chassisVelocity[0]);
	NewtonBodyGetOmega(m_body0, &m_chassisOmega[0]);
	for (int i = 0; i < m_tiresCount; i ++) {
		Tire& tire = m_tires[i];
		// calculate the linear velocity of the tire at the ground contact
		tire.m_tireAxelPosit = (chassisMatrix.TransformVector(tire.m_harpoint - m_localFrame.m_up.Scale (tire.m_posit)));
	    tire.m_localAxelPosit = (tire.m_tireAxelPosit - chassisMatrix.m_posit);
		tire.m_tireAxelVeloc = (m_chassisVelocity + m_chassisOmega * tire.m_localAxelPosit);


	    dVector lateralPin (chassisMatrix.RotateVector (tire.m_localAxis));
	    dVector longitudinalPin (chassisMatrix.m_up * lateralPin);
	    tire.m_longitudinalDir = longitudinalPin;
	    tire.m_lateralDir = lateralPin;
		if (tire.m_posit < tire.m_suspensionLength)  {
			dFloat speed;
            // TO DO: need to calculate the velocity if the other body at the point

			// for now assume the ground is a static body
			dVector hitBodyVeloc (0, 0, 0, 0);

			// calculate the relative velocity
			dVector relVeloc (tire.m_tireAxelVeloc - hitBodyVeloc);
			speed = -(relVeloc % chassisMatrix.m_up);

			// now calculate the tire load at the contact point
			tire.m_tireLoad = - NewtonCalculateSpringDamperAcceleration (timestep, tire.m_springConst, tire.m_suspensionLength - tire.m_posit, tire.m_springDamper, speed)*0.5f;

			if (tire.m_tireLoad < 0.0f) {
				// since the tire is not a body with real mass it can only push the chassis.
				tire.m_tireLoad = 0.0f;
			} else {
				//this suspension is applying a normalize force to the car chassis, need to scales by the mass of the car
				tire.m_tireLoad *= m_mass;

				// apply the tire model to these wheel
			}

			// convert the tire load force magnitude to a torque and force.
			dVector tireForce (chassisMatrix.m_up.Scale (tire.m_tireLoad));

			// accumulate the force and torque form this suspension
			m_chassisForce = tireForce;

            m_chassisForce += chassisMatrix.m_front.Scale(tire.m_torque*-40);
			if (dAbs(m_curSpeed)!=0.0f) {
              m_chassisForce += chassisMatrix.m_right.Scale(tire.m_turnforce*60);
			}
			if (tire.m_groundFriction!=0) {
			  ApplyTractionAndSteer(m_chassisForce,tire.m_tireAxelPosit);
			  ApplyTireFrictionModel(chassisMatrix, timestep);
			}
		} else {
			//tire is on the air  not force applied to the vehicle.
			tire.m_tireLoad = dFloat (0.0f);
//			tire.m_tireJacobianRowIndex = -1;
			dFloat torque;
			torque = tire.m_torque - tire.m_angularVelocity * tire.m_Ixx * 0.1f;
			tire.m_angularVelocity  += torque * tire.m_IxxInv * timestep;
		}
		ApplyTiresTorqueVisual(tire,timestep,threadIndex);
	}
	NewtonBodyAddForce(m_body0, &m_gravity[0]);
	// set the current vehicle speed
	m_curSpeed = bodyMatrix[0] % m_chassisVelocity;
	if (m_curSpeed>0) {
      m_tiresRollSide = 0;
	} else {
      m_tiresRollSide = 1;
	}
}

// I get a problem with the visual.
// If the tire get so much presure from the side the visual can disappear.
void CustomRayCastCar::ApplyTiresTorqueVisual(Tire& tire, dFloat timestep, int threadIndex)
{
	dFloat timestepInv;
	// get the simulation time
	timestepInv = 1.0f / timestep;
    dVector tireRadius (tire.m_contactPoint - tire.m_tireAxelPosit);
    //check if any engine torque or brake torque is applied to the tire
	if (dAbs(tire.m_torque) < 1.0e-3f){
      //tire is coasting, calculate the tire zero slip angular velocity
	  // this is the velocity that satisfy the constraint equation
	  // V % dir + W * R % dir = 0
	  // where V is the tire Axel velocity
	  // W is the tire local angular velocity
	  // R is the tire radius
	  // dir is the longitudinal direction of of the tire.
	  dFloat tireLinearSpeed;
	  dFloat tireContactSpeed;
	  tireLinearSpeed = tire.m_tireAxelVeloc % tire.m_longitudinalDir;
	  tireContactSpeed = (tire.m_lateralDir * tireRadius) % tire.m_longitudinalDir;
	  tire.m_angularVelocity = - tireLinearSpeed / tireContactSpeed;
	} else {
	  // tire is under some power, need to do the free body integration to apply the net torque
	  dFloat tireLinearSpeed;
	  dFloat tireContactSpeed;
	  tireLinearSpeed = tire.m_tireAxelVeloc % tire.m_longitudinalDir;
	  tireContactSpeed = (tire.m_lateralDir * tireRadius) % tire.m_longitudinalDir;
	  dFloat nettorque = - tireLinearSpeed / tireContactSpeed;
	  //tire.m_angularVelocity = - tireLinearSpeed / tireContactSpeed;

	  dFloat torque;
	  torque = tire.m_torque - nettorque - tire.m_angularVelocity * tire.m_Ixx * 0.1f;
	  tire.m_angularVelocity  += torque * tire.m_IxxInv * timestep;
	}
	// integrate tire angular velocity and rotation
	tire.m_spinAngle = dMod (tire.m_spinAngle + tire.m_angularVelocity * timestep, 3.1416f * 2.0f);
	// reset tire torque to zero after integration;
	tire.m_torque = 0.0f;
}


void CustomRayCastCar::ApplyTireFrictionModel(const dMatrix& chassisMatrix, dFloat timestep)
{
  ApplyVelocityCorrection(chassisMatrix);
  ApplyOmegaCorrection();

  NewtonBodySetVelocity(m_body0,&m_chassisVelocity[0]);
  NewtonBodySetOmega(m_body0,&m_chassisOmega[0]);
}

void CustomRayCastCar::ApplyOmegaCorrection()
{
  // can surely need a fix to make lose friction when the vehicle turn from steer for simulate rotational effect.
  // this is only a quick hack in waitting a better solution.
  m_chassisOmega = m_chassisOmega.Scale(0.975f);
}

// friction need a fix to make lose friction on the torque tires at pos tires.
// because currently the friction is only lose in directional mode by the hack.
void CustomRayCastCar::ApplyVelocityCorrection(const dMatrix& chassisMatrix)
{
  dFloat vl = 0;
  dVector vlo = m_chassisVelocity;
  dVector vDir = chassisMatrix.m_right;
  dVector vt = dVector(0.0f, 0.0f, 0.0f, 0.0f);
  //
  dFloat speed;
  // add some lose friction.
  // can surely need a fix to make lose friction when the vehicle turn from steer.
  // this is only a quick hack in waitting a better solution.
  speed = dAbs (GetSpeed())*0.03f;
  if (speed>0.295f) {
    speed = 0.295f;
  }
  vl = (vlo % vDir.Scale((0.3f-speed)));
  //
  vt.m_x = (vlo.m_x) - vl * vDir.m_x;
  vt.m_y = (vlo.m_y) - vl * vDir.m_y;
  vt.m_z = (vlo.m_z) - vl * vDir.m_z;

  m_chassisVelocity = vt;
}

void CustomRayCastCar::ApplyTractionAndSteer(const dVector& vForce, const dVector& vPoint)
{
  dVector Torque;
  dMatrix M;
  dVector com;
  NewtonBodyGetCentreOfMass(m_body0,&com[0]);
  NewtonBodyGetMatrix(m_body0,&M[0][0]);
  Torque = ( vPoint - M.TransformVector( dVector(com.m_x, com.m_y, com.m_z, 1.0f ))) * vForce;
  NewtonBodyAddForce(m_body0,&vForce[0]);
  NewtonBodyAddTorque(m_body0,&Torque[0]);
}
