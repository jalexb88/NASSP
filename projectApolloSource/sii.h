/***************************************************************************
  This file is part of Project Apollo - NASSP
  Copyright 2004-2005 Jean-Luc Rocca-Serra, Mark Grant

  ORBITER vessel module: SII class

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
  *	Revision 1.2  2006/01/26 03:07:50  movieman523
  *	Quick hack to support low-res mesh.
  *	
  *	Revision 1.1  2005/11/23 01:43:13  movieman523
  *	Added SII stage DLL.
  *	
  *	
  *	
  **************************************************************************/

//
// Data structure passed from main vessel to SII to configure stage.
//

typedef struct {

	int SettingsType;
	int VehicleNo;
	int Realism;
	int RetroNum;

	double THRUST_SECOND_VAC;
	double ISP_SECOND_VAC;
	double ISP_SECOND_SL;
	double MissionTime;
	double EmptyMass;
	double MainFuelKg;
	double CurrentThrust;

	bool LowRes;

} SIISettings;

//
// Which parts of the structure are active.
//

#define SII_SETTINGS_MASS		0x1
#define SII_SETTINGS_FUEL		0x2
#define SII_SETTINGS_GENERAL	0x4
#define SII_SETTINGS_ENGINES	0x8

//
// Stage states.
//

#define SII_STATE_SHUTTING_DOWN		0
#define SII_STATE_WAITING			1

///
/// This code simulates the seperated SII stage. Basically it simulates thrust decay if there is any fuel 
/// left, fires any retro rockets to push it away from the Saturn and then sits around waiting to be deleted.
///
/// \brief SII stage simulation.
/// \ingroup SepStages
///
class SII : public VESSEL2 {

public:
	SII (OBJHANDLE hObj, int fmodel);
	virtual ~SII();

	void clbkSaveState (FILEHANDLE scn);
	void clbkPreStep(double simt, double simdt, double mjd);
	void clbkLoadStateEx (FILEHANDLE scn, void *status);
	void clbkSetClassCaps (FILEHANDLE cfg);
	void clbkDockEvent(int dock, OBJHANDLE connected);

	//
	// Must be virtual so it can be called from other DLLs.
	//
	virtual void SetState(SIISettings &state);

protected:

	void SetSII();
	void InitSII();
	void AddEngines();

	int GetMainState();
	void SetMainState(int s);

	int MissionNo;
	int VehicleNo;
	int State;
	int Realism;
	int RetroNum;

	bool RetrosFired;
	bool LowRes;

	double EmptyMass;
	double PayloadMass;
	double MainFuel;

	double MissionTime;
	double NextMissionEventTime;
	double LastMissionEventTime;

	double THRUST_SECOND_VAC;
	double ISP_SECOND_VAC;
	double ISP_SECOND_SL;
	double CurrentThrust;

	SURFHANDLE SMMETex;

	THRUSTER_HANDLE th_retro[4], th_main[5];
	THGROUP_HANDLE thg_retro, thg_main;
	PROPELLANT_HANDLE ph_retro, ph_main;
};
