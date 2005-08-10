/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005

  IMU header

  Project Apollo is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Project Apollo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Project Apollo; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  See http://nassp.sourceforge.net/license/ for more details.

  **************************** Revision History ****************************
  *	$Log$
  **************************************************************************/

// IMU

class ApolloGuidance;

typedef union {      // 3 vector
	double data[3];
	struct { double x, y, z; };
} IMU_Vector3;

typedef union {      // 3x3 matrix
	double data[9];
	struct { double m11, m12, m13, m21, m22, m23, m31, m32, m33; };
} IMU_Matrix3;

class IMU {

public:
	IMU(ApolloGuidance &comp);
	~IMU();

	void Init();
	void ChannelOutput(int address, int value);
	void Timestep(double simt);
	void TurnOn();
	void TurnOff();
	void DriveGimbals(double x, double y, double z);
	void SetVessel(VESSEL *v) { OurVessel = v; };

	void LoadState(FILEHANDLE scn);
	void SaveState(FILEHANDLE scn);

protected:

	VECTOR3 IMU::CalculateAccelerations(double deltaT);
	void DriveCDUX();
	void DriveCDUY();
	void DriveCDUZ();
	void DriveCDU(int index, int RegCDU, int RegCDUCMD);
	void DriveGimbalX(double angle);
	void DriveGimbalY(double angle);
	void DriveGimbalZ(double angle);
	void DriveGimbal(int index, int RegCDU, double angle, int changeCDU);
	void PulsePIPA(int RegPIPA, int pulses);
	void SetOrbiterAttitudeReference();
	void ZeroIMUCDUs();

	//
	// Logging.
	//

	void LogInit();
	void LogState(int channel, char *device, int value);
	void LogTimeStep(long time);
	void LogVector(char* message, IMU_Vector3 v);
	void LogMessage(char* s);

	//
	// Maths.
	//

	IMU_Matrix3 getRotationMatrixX(double angle);
	IMU_Matrix3 getRotationMatrixY(double angle);
	IMU_Matrix3 getRotationMatrixZ(double angle);
	IMU_Matrix3 multiplyMatrix(IMU_Matrix3 a, IMU_Matrix3 b);
	IMU_Vector3 multiplyMatrixByVector(IMU_Matrix3 m, IMU_Vector3 v);
	IMU_Vector3 getRotationAnglesXZY(IMU_Matrix3 m);
	IMU_Vector3 getRotationAnglesZYX(IMU_Matrix3 m);
	IMU_Vector3 VECTOR3ToIMU_Vector3(VECTOR3 v);
	VECTOR3 IMU_Vector3ToVECTOR3(IMU_Vector3 iv);
	IMU_Matrix3 MATRIX3ToIMU_Matrix3(MATRIX3 m);

	double degToRad(double angle);
	double radToDeg(double angle);
	double gyroPulsesToRad(int pulses);
	int radToGyroPulses(double angle);

	IMU_Matrix3 getNavigationBaseToOrbiterLocalTransformation();
	IMU_Matrix3 getOrbiterLocalToNavigationBaseTransformation();

	ApolloGuidance &agc;
	VESSEL *OurVessel;

	bool Operate;
	bool TurnedOn;
	bool Initialized;

	union {
		struct {
			double X;	//outer gimbal angle in rad
			double Y;	//inner gimbal angle in rad
			double Z;	//middle gimbal angle in rad
		} Gimbal;
		double Gimbals[3];
	};

	struct {
		double X;
		double Y;
		double Z;
	} RemainingPIPA;

	struct {
		struct {
			double X;
			double Y;
			double Z;
		} Attitude;
		struct {
			double X;
			double Y;
			double Z;
		} LastAttitude;

		IMU_Matrix3 AttitudeReference;
	} Orbiter;

	VECTOR3 Velocity;

	double LastTime;	// in seconds
};

//
// Strings for state saving.
//

#define IMU_START_STRING	"IMU_BEGIN"
#define IMU_END_STRING		"IMU_END"
