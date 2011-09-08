#if !defined(AFX_CUSTOM_RAYCASTCAR_INCLUDED)
#define AFX_CUSTOM_RAYCASTCAR_INCLUDED



#include "NewtonCustomJoint.h"


#define VEHICLE_MAX_TIRE_COUNT 16

#define USE_MASTER_KOOM_VARIANCE

class JOINTLIBRARY_API CustomRayCastCar: public NewtonCustomJoint
{
public:
	struct Tire
	{
		// constant data
		dVector m_harpoint;				   // attachment point of this tire to the chassis
		dVector m_localAxis;               // tire local axis of rotation
		dVector m_contactPoint;			   // contact point in global space
		dVector m_contactNormal;           // contact normal in global space
		dVector m_lateralDir;
		dVector m_longitudinalDir;
		dVector m_tireRadius;
		dVector m_tireAxelPosit;
		dVector m_localAxelPosit;
		dVector m_tireAxelVeloc;

		NewtonCollision* m_shape;          // collision shape of this tire
		void* m_userData;                  // user data pointing to the visual tire
		dFloat m_mass;					   // tire Mass matrix
		dFloat m_Ixx;					   // axis inertia
		dFloat m_massInv;				   // tire Mass matrix
		dFloat m_IxxInv;                   // axis inertia
		dFloat m_radius;				   // tire Radius
		dFloat m_maxTireRPS;

		dFloat m_localLateralSpeed;
		dFloat m_localSuspentionSpeed;
		dFloat m_localLongitudinalSpeed;
		dFloat m_currentSlipVeloc;		   // the tire sleep acceleration
		dFloat m_springConst;			   // normalized spring Ks
		dFloat m_springDamper;			   // normalized spring Damper Kc
		dFloat m_suspensionLength;         // max suspension length
		dFloat m_groundFriction;		   // coefficient of friction of the ground surface
		dFloat m_posit;					   // parametric position for this tire ( always positive value between 0 and m_suspensionLength)
		dFloat m_torque;				   // tire torque
		dFloat m_turnforce;				   // tire turnforce
		dFloat m_breakTorque;			   // tire break torque
		dFloat m_tireLoad;				   // force generate by the suspension compression (must be always positive)
		dFloat m_steerAngle;               // current tire steering angle
		dFloat m_spinAngle;                // current tire spin angle
		dFloat m_angularVelocity;          // current tire spin velocity
		int	m_tireUseConvexCastMode;      // default to false (can be set to true for fast LOD cars)
	};


	CustomRayCastCar(int maxTireCount, const dMatrix& chassisMatrix, NewtonBody* carBody, const dVector& gravity = dVector (0.0f, -9.8f, 0.0f, 0.0f));
	virtual ~CustomRayCastCar();

	dFloat GetSpeed() const;
	int GetTiresCount() const;
	Tire& GetTire (int index) const;
	dFloat GetParametricPosition (int index) const;

	virtual void SetBrake (dFloat torque) = 0;
	virtual void SetTorque (dFloat torque) = 0;
	virtual void SetSteering (dFloat angle) = 0;


	void AddSingleSuspensionTire (void* userData, const dVector& localPosition,
								  dFloat mass, dFloat radius, dFloat with,
								  dFloat suspensionLength, dFloat springConst, dFloat springDamper,
								  int castMode);

	void SetTireMaxRPS (int tireIndex, dFloat maxTireRPS);

	const dMatrix& GetChassisMatrixLocal () const;
	dMatrix CalculateTireMatrix (int tire) const;

protected:
	void SetTireBrake (int index, dFloat torque);
	void SetTireTorque (int index, dFloat torque);
	void SetTireSteerAngle (int index, dFloat angle, dFloat turnforce);

	static unsigned ConvexCastPrefilter(const NewtonBody* body, const NewtonCollision* collision, void* userData);
	dFloat CalculateNormalizeForceVsSlipAngle (const Tire& tire, float slipAngle) const;
	void CalculateTireCollision (Tire& tire, const dMatrix& suspensionMatrixInGlobalSpace) const;

	virtual void SubmitConstraints (dFloat timestep, int threadIndex);

	virtual void GetInfo (NewtonJointRecord* info) const;

	void ApplyTireForces (const dMatrix& chassisMatrix, dFloat tiemStep) const;
	void ApplySuspensionForces (const dMatrix& chassisMatrix, dFloat tiemStep) const;
	void ApplyTireFrictionModel(const dMatrix& chassisMatrix, dFloat timestep);
    void ApplyOmegaCorrection();
    void ApplyVelocityCorrection(const dMatrix& chassisMatrix);
	void ApplyTractionAndSteer(const dVector& vForce, const dVector& vPoint);
	void ApplyTiresTorqueVisual(Tire& tire, dFloat timestep, int threadIndex);

    dVector m_chassisOmega;         // chassis omega correction
    dVector m_chassisVelocity;      // chassis velocity correction
    dVector m_chassisForce;         // chassis Force Global
    dVector m_chassisTorque;        // chassis Torque Global
	int m_tiresRollSide;            // visual rolling side

	dMatrix m_localFrame;			// local coordinate system of the vehicle
	dFloat m_mass;
	dFloat m_curSpeed;
	int m_tiresCount;				// current number of tires
	dVector m_gravity;
	Tire* m_tires;					// tires array

private:
	dMatrix CalculateSuspensionMatrix (int tire, dFloat param) const;
};

#endif
