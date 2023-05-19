/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2023

RTCC Calculations for Mission J3

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

**************************************************************************/

#include "Orbitersdk.h"
#include "soundlib.h"
#include "apolloguidance.h"
#include "saturn.h"
#include "saturnv.h"
#include "LVDC.h"
#include "iu.h"
#include "LEM.h"
#include "../src_rtccmfd/OrbMech.h"
#include "mcc.h"
#include "rtcc.h"

bool RTCC::CalculationMTP_J3(int fcn, LPVOID &pad, char * upString, char * upDesc, char * upMessage)
{
	char uplinkdata[1024 * 3];
	bool preliminary = false;
	bool scrubbed = false;

	double NominalLaunchGMT = OrbMech::HHMMSSToSS(2, 53, 0); //Nominal launch GMT

	switch (fcn) {
	case 1: //GENERIC CMC CSM STATE VECTOR UPDATE
	case 3: //GENERIC LGC CSM STATE VECTOR UPDATE
	{
		EphemerisData sv;
		char buffer1[1000];
		int veh;

		sv = StateVectorCalcEphem(calcParams.src); //State vector for uplink
		if (fcn == 1)
		{
			veh = RTCC_MPT_CSM;
		}
		else
		{
			veh = RTCC_MPT_LM;
		}

		AGCStateVectorUpdate(buffer1, veh, RTCC_MPT_CSM, sv);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector");
		}
	}
	break;
	case 2: //GENERIC CMC LM STATE VECTOR UPDATE
	case 4: //GENERIC LGC LM STATE VECTOR UPDATE
	{
		EphemerisData sv;
		char buffer1[1000];
		int veh;

		sv = StateVectorCalcEphem(calcParams.tgt); //State vector for uplink
		if (fcn == 2)
		{
			veh = RTCC_MPT_CSM;
		}
		else
		{
			veh = RTCC_MPT_LM;
		}

		AGCStateVectorUpdate(buffer1, veh, RTCC_MPT_LM, sv);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "LM state vector");
		}
	}
	break;
	case 5: //GENERIC CMC CSM STATE VECTOR UPDATE WITH V66
	case 6: //GENERIC LGC LM STATE VECTOR UPDATE WITH V66
	{
		EphemerisData sv;
		char buffer1[1000];
		int veh, vehs;

		if (fcn == 5)
		{
			sv = StateVectorCalcEphem(calcParams.src); //State vector for uplink
			veh = RTCC_MPT_CSM;
			vehs = RTCC_MPT_CSM;
		}
		else
		{
			sv = StateVectorCalcEphem(calcParams.tgt); //State vector for uplink
			veh = RTCC_MPT_LM;
			vehs = RTCC_MPT_LM;
		}

		AGCStateVectorUpdate(buffer1, veh, vehs, sv, true);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			if (fcn == 5)
			{
				sprintf(upDesc, "CSM state vector and V66");
			}
			else
			{
				sprintf(upDesc, "LM state vector and V66");
			}
		}
	}
	break;
	case 7: //CSM DAP DATA
	{
		AP10DAPDATA * form = (AP10DAPDATA *)pad;

		CSMDAPUpdate(calcParams.src, *form, false);
	}
	break;
	case 8: //LM DAP DATA (ASCENT STAGE)
	case 9: //LM DAP DATA
	{
		AP10DAPDATA * form = (AP10DAPDATA *)pad;

		if (fcn == 9)
		{
			LMDAPUpdate(calcParams.tgt, *form, false);
		}
		else
		{
			LMDAPUpdate(calcParams.tgt, *form, false, true);
		}
	}
	break;
	case 10: //GROUND LIFTOFF TIME UPDATE
	{
		char Buff[128];

		//P80 MED: mission initialization
		sprintf_s(Buff, "P80,1,CSM,%d,%d,%d;", GZGENCSN.MonthofLiftoff, GZGENCSN.DayofLiftoff, GZGENCSN.Year);
		GMGMED(Buff);

		//P10 MED: Enter actual liftoff time
		double TEPHEM0, tephem_scal;
		Saturn *cm = (Saturn *)calcParams.src;

		//Get TEPHEM
		TEPHEM0 = 41133.;
		tephem_scal = GetTEPHEMFromAGC(&cm->agc.vagc);
		double LaunchMJD = (tephem_scal / 8640000.) + TEPHEM0;
		LaunchMJD = (LaunchMJD - SystemParameters.GMTBASE)*24.0;

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(LaunchMJD*3600.0, hh, mm, ss);

		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P12: CSM GRR and Azimuth
		SaturnV *SatV = (SaturnV*)cm;
		LVDCSV *lvdc = (LVDCSV*)SatV->iu->GetLVDC();
		double Azi = lvdc->Azimuth*DEG;
		double T_GRR = lvdc->T_L;

		sprintf_s(Buff, "P12,CSM,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//P15: CMC, LGC and AGS clock zero
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);
		sprintf_s(Buff, "P15,LGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);
		GMGMED("P15,AGS,,110:00:00;");

		//P12: IU GRR and Azimuth
		OrbMech::SStoHHMMSS(T_GRR, hh, mm, ss);
		sprintf_s(Buff, "P12,IU1,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//Get actual liftoff REFSMMAT from telemetry
		BZSTLM.CMC_REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		BZSTLM.CMCRefsPresent = true;
		EMSGSUPP(1, 1);
		//Make telemetry matrix current
		GMGMED("G00,CSM,TLM,CSM,CUR;");
	}
	break;
	case 11: //TLI SIMULATION
	{
		if (PZMPTCSM.ManeuverNum > 0)
		{
			//Delete maneuvers from MPT
			GMGMED("M62,CSM,1,D;");
		}

		//Config and mass update
		med_m55.Table = RTCC_MPT_CSM;
		MPTMassUpdate(calcParams.src, med_m50, med_m55, med_m49);
		PMMWTC(55);
		med_m50.Table = RTCC_MPT_CSM;
		med_m50.WeightGET = GETfromGMT(RTCCPresentTimeGMT());
		PMMWTC(50);

		//Trajectory Update
		StateVectorTableEntry sv0;
		sv0.Vector = StateVectorCalcEphem(calcParams.src);
		sv0.LandingSiteIndicator = false;
		sv0.VectorCode = "APIC001";

		PMSVCT(4, RTCC_MPT_CSM, &sv0);

		//Add TLI to MPT
		if (GETEval2(3.0*3600.0))
		{
			//Second opportunity
			GMGMED("M68,CSM,2;");
		}
		else
		{
			//First opportunity
			GMGMED("M68,CSM,1;");
		}

		//Add separation maneuver to MPT
		GMGMED("M40,P1,0.0;");
		med_m66.Table = RTCC_MPT_CSM;
		med_m66.ReplaceCode = 0; //Don't replace
		med_m66.GETBI = GETfromGMT(PZMPTCSM.mantable[0].GMT_BO) + 15.0*60.0;
		med_m66.Thruster = RTCC_ENGINETYPE_CSMRCSPLUS4;
		med_m66.AttitudeOpt = RTCC_ATTITUDE_INERTIAL;
		med_m66.BurnParamNo = 1;
		med_m66.CoordInd = 0; //LVLH
		med_m66.Att = _V(41.6*RAD, 120.8*RAD, 131.9*RAD); //Make this launch day specific?
		med_m66.ConfigChangeInd = RTCC_CONFIGCHANGE_UNDOCKING;
		med_m66.FinalConfig = "CL";

		//Dummy data
		std::vector<std::string> str;
		PMMMED("66", str);

		//Save TLI time
		TimeofIgnition = GETfromGMT(PZMPTCSM.mantable[0].GMT_BI);
		calcParams.TLI = GETfromGMT(PZMPTCSM.mantable[0].GMT_BO);
	}
	break;
	case 12: //TLI+90 MANEUVER PAD
	{
		EntryOpt entopt;
		EntryResults res;
		AP11ManPADOpt opt;
		double TLIBase, TIG;
		EphemerisData sv;
		SV sv1;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);

		TLIBase = calcParams.TLI;
		TIG = TLIBase + 90.0*60.0;
		entopt.ATPLine = 2; //AOL
		entopt.r_rbias = PZREAP.RRBIAS;

		sv1.mass = PZMPTCSM.mantable[1].CommonBlock.CSMMass;
		sv1.gravref = hEarth;
		sv1.MJD = OrbMech::MJDfromGET(PZMPTCSM.mantable[1].GMT_BO, SystemParameters.GMTBASE);
		sv1.R = PZMPTCSM.mantable[1].R_BO;
		sv1.V = PZMPTCSM.mantable[1].V_BO;

		entopt.entrylongmanual = false;
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.TIGguess = TIG;
		entopt.t_Z = OrbMech::HHMMSSToSS(24.0, 38.0, 09.0);
		entopt.type = 1;
		entopt.vessel = calcParams.src;
		entopt.RV_MCC = sv1;

		EntryTargeting(&entopt, &res); //Target Load for uplink

		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.TIG = res.P30TIG;
		opt.vessel = calcParams.src;
		opt.vesseltype = 0;
		opt.useSV = true;
		opt.RV_MCC = sv1;

		AP11ManeuverPAD(&opt, *form);
		form->lat = res.latitude*DEG;
		form->lng = res.longitude*DEG;
		form->RTGO = res.RTGO;
		form->VI0 = res.VIO / 0.3048;
		form->Weight = PZMPTCSM.mantable[1].CommonBlock.CSMMass / 0.45359237;
		form->GET05G = res.GET05G;

		sprintf(form->purpose, "TLI+90");
	}
	break;
	case 13: //LIFTOFF+9 PAD
	{
		EntryOpt entopt;
		EntryResults res;
		SV sv1;

		P37PAD * form = (P37PAD *)pad;

		sv1.mass = PZMPTCSM.mantable[1].CommonBlock.CSMMass;
		sv1.gravref = hEarth;
		sv1.MJD = OrbMech::MJDfromGET(PZMPTCSM.mantable[1].GMT_BO, SystemParameters.GMTBASE);
		sv1.R = PZMPTCSM.mantable[1].R_BO;
		sv1.V = PZMPTCSM.mantable[1].V_BO;

		entopt.entrylongmanual = false;
		entopt.ATPLine = 0; //MPL
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.type = 1;
		entopt.vessel = calcParams.src;
		entopt.r_rbias = PZREAP.RRBIAS;

		entopt.TIGguess = form->GETI[0] = OrbMech::HHMMSSToSS(9, 0, 0);
		entopt.t_Z = OrbMech::HHMMSSToSS(33.0, 49.0, 0.0);
		entopt.RV_MCC = sv1;
		EntryTargeting(&entopt, &res);
		form->dVT[0] = length(res.dV_LVLH) / 0.3048;
		form->GET400K[0] = res.GET05G;
		form->lng[0] = round(res.longitude*DEG);
	}
	break;
	case 14: //TLI PAD
	{
		SV sv;

		TLIPAD * form = (TLIPAD *)pad;

		//DMT for TLI
		GMGMED("U20,CSM,1;");

		form->TB6P = DMTBuffer[0].GETI - 9.0*60.0 - 38.0;
		form->IgnATT = DMTBuffer[0].IMUAtt;
		form->BurnTime = DMTBuffer[0].DT_B;
		form->dVC = DMTBuffer[0].DVC;
		form->VI = length(PZMPTCSM.mantable[0].V_BO) / 0.3048;
		form->type = 2;

		//DMT for sep maneuver
		GMGMED("U20,CSM,2;");
		form->SepATT = DMTBuffer[0].IMUAtt;
		form->ExtATT = OrbMech::imulimit(_V(300.0 - form->SepATT.x, form->SepATT.y + 180.0, 360.0 - form->SepATT.z));

		//Add Ordeal data (fix me)
		double R2 = 306.0; //R2 Ign, must be calculated dynamically based on launch delay
		int yaw = 0; // Opportunity 1 = 0, Opportunity 2 = 3
		sprintf(form->remarks, "\nXX%04.1f R2 Align\nXX%04.1f R2 Ign\nXX57:10 ORDEAL Start\nXXX00%d YAW", R2, R2 + 6.0, yaw);

		//Delete maneuvers from MPT
		GMGMED("M62,CSM,1,D;");

		//Set anchor vector time to 0, so that no trajectory updates are done anymore
		EZANCHR1.AnchorVectors[9].Vector.GMT = 0.0;
	}
	break;
	case 15: //TLI Evaluation
	{
		SaturnV *SatV = (SaturnV*)calcParams.src;
		LVDCSV *lvdc = (LVDCSV*)SatV->iu->GetLVDC();

		if (lvdc->LVDC_Timebase == 5)
		{
			scrubbed = true;
		}
	}
	break;
	case 16: //Block Data 1
	case 17: //Block Data 2
	{
		EntryOpt entopt;
		EntryResults res;
		SV sv1, sv2;

		P37PAD * form = (P37PAD *)pad;

		sv1 = StateVectorCalc(calcParams.src);

		if (length(DeltaV_LVLH) > 0.0)
		{
			sv2 = ExecuteManeuver(sv1, TimeofIgnition, DeltaV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);
		}
		else
		{
			sv2 = sv1;
		}

		entopt.entrylongmanual = false;
		entopt.ATPLine = 0; //MPL
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.type = 1;
		entopt.vessel = calcParams.src;
		entopt.r_rbias = PZREAP.RRBIAS;

		if (fcn == 16)
		{
			entopt.TIGguess = form->GETI[0] = OrbMech::HHMMSSToSS(15, 0, 0);
			entopt.t_Z = OrbMech::HHMMSSToSS(57.0, 56.0, 0.0);
			entopt.RV_MCC = sv1;
			EntryTargeting(&entopt, &res);
			form->dVT[0] = length(res.dV_LVLH) / 0.3048;
			form->GET400K[0] = res.GET05G;
			form->lng[0] = round(res.longitude*DEG);

			entopt.TIGguess = form->GETI[1] = OrbMech::HHMMSSToSS(25, 0, 0);
			entopt.t_Z = OrbMech::HHMMSSToSS(57.0, 25.0, 0.0);
			entopt.RV_MCC = sv1;
			EntryTargeting(&entopt, &res);
			form->dVT[1] = length(res.dV_LVLH) / 0.3048;
			form->GET400K[1] = res.GET05G;
			form->lng[1] = round(res.longitude*DEG);
		}
		else
		{
			entopt.TIGguess = form->GETI[0] = OrbMech::HHMMSSToSS(35, 0, 0);
			entopt.t_Z = OrbMech::HHMMSSToSS(81.0, 39.0, 0.0);
			entopt.RV_MCC = sv1;
			EntryTargeting(&entopt, &res);
			form->dVT[0] = length(res.dV_LVLH) / 0.3048;
			form->GET400K[0] = res.GET05G;
			form->lng[0] = round(res.longitude*DEG);

			entopt.TIGguess = form->GETI[1] = OrbMech::HHMMSSToSS(45, 0, 0);
			entopt.t_Z = OrbMech::HHMMSSToSS(81.0, 18.0, 0.0);
			entopt.RV_MCC = sv2;
			EntryTargeting(&entopt, &res);
			form->dVT[1] = length(res.dV_LVLH) / 0.3048;
			form->GET400K[1] = res.GET05G;
			form->lng[1] = round(res.longitude*DEG);

			entopt.TIGguess = form->GETI[2] = OrbMech::HHMMSSToSS(55, 0, 0);
			entopt.t_Z = OrbMech::HHMMSSToSS(105.0, 30.0, 0.0);
			entopt.RV_MCC = sv2;
			EntryTargeting(&entopt, &res);
			form->dVT[2] = length(res.dV_LVLH) / 0.3048;
			form->GET400K[2] = res.GET05G;
			form->lng[2] = round(res.longitude*DEG);

			entopt.TIGguess = form->GETI[3] = OrbMech::HHMMSSToSS(65, 0, 0);
			entopt.t_Z = OrbMech::HHMMSSToSS(129.0, 40.0, 0.0);
			entopt.RV_MCC = sv2;
			EntryTargeting(&entopt, &res);
			form->dVT[3] = length(res.dV_LVLH) / 0.3048;
			form->GET400K[3] = res.GET05G;
			form->lng[3] = round(res.longitude*DEG);
		}
	}
	break;
	case 18: //PTC REFSMMAT
	{
		char buffer[1000];
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;

		refsopt.REFSMMATopt = 6;
		refsopt.REFSMMATTime = 41668.18229177778;  //241:29:30 GET of nominal mission

		REFSMMAT = REFSMMATCalc(&refsopt);

		//Store in LCV slot
		EMGSTSTM(RTCC_MPT_CSM, REFSMMAT, RTCC_REFSMMAT_TYPE_LCV, RTCCPresentTimeGMT());

		AGCDesiredREFSMMATUpdate(buffer, REFSMMAT);
		sprintf(uplinkdata, "%s", buffer);

		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "PTC REFSMMAT");
		}
	}
	break;
	case 19: //MCC-1 EVALUATION
	case 21: //MCC-1 CALCULATION AND UPDATE
	case 20: //MCC-2 EVALUATION
	case 22: //MCC-2 CALCULATION AND UPDATE
	{
		double P30TIG, MCC1GET, MCC2GET, CSMmass, LMmass, F23time;
		int engine, mccnum;
		VECTOR3 dV_LVLH;
		EphemerisData sv;
		char Buff[128];

		int hh, mm;
		double ss;

		double TLIbase = calcParams.TLI - 5.0*60.0 - 45.0; //Approximate TLI ignition

		double LOIGET = GETfromGMT(320138.0 + NominalLaunchGMT);

		MCC1GET = TLIbase + 5.0*3600.0 + 24.0*60.0;
		MCC2GET = TLIbase + 32.0*3600.0 + 9.0*60.0;

		sv = StateVectorCalcEphem(calcParams.src);

		PZMCCPLN.MidcourseGET = MCC2GET;
		PZMCCPLN.Config = true;
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 1;
		PZMCCPLN.Mode = 4;
		CSMmass = calcParams.src->GetMass();
		LMmass = calcParams.tgt->GetMass();

		sprintf_s(Buff, "F23,0.0:0.0:0.0,0.0:0.0:0.0;");
		GMGMED(Buff);

		TranslunarMidcourseCorrectionProcessor(sv, CSMmass, LMmass);

		//Iterate node GET for MCC-2
		bool init = true;
		F23time = LOIGET - 11.0*60.0;
		while (PZMCCDIS.data[0].GET_LOI < LOIGET - 5.0 || init)
		{
			OrbMech::SStoHHMMSS(F23time, hh, mm, ss);
			sprintf_s(Buff, "F23,%d:%d:%.2lf,%d:%d:%.2lf;", hh, mm, ss, hh, mm + 10, ss);
			GMGMED(Buff);
			TranslunarMidcourseCorrectionProcessor(sv, CSMmass, LMmass);
			F23time = F23time + 5.0;
			if (init) init = false;
		}

		if (fcn == 19 || fcn == 21) // MCC-1
		{
			mccnum = 1;

			if (length(PZMCCDIS.data[0].DV_MCC) < 30.0*0.3048) // Can we wait until MCC-2? // To Do: Check TLC DV budget for Apollo 17 
			{
				scrubbed = true;
			}
			else
			{
				PZMCCPLN.MidcourseGET = MCC1GET;

				TranslunarMidcourseCorrectionProcessor(sv, CSMmass, LMmass);

				//Re-iterate node GET for MCC-1
				bool init = true;
				F23time = LOIGET - 11.0*60.0;
				while (PZMCCDIS.data[0].GET_LOI < LOIGET - 5.0 || init)
				{
					OrbMech::SStoHHMMSS(F23time, hh, mm, ss);
					sprintf_s(Buff, "F23,%d:%d:%.2lf,%d:%d:%.2lf;", hh, mm, ss, hh, mm + 10, ss);
					GMGMED(Buff);
					TranslunarMidcourseCorrectionProcessor(sv, CSMmass, LMmass);
					F23time = F23time + 5.0;
					if (init) init = false;
				}
			}
		}
		else // MCC-2
		{
			mccnum = 2;

			if (length(PZMCCDIS.data[0].DV_MCC) < 1.0*0.3048)
			{
				scrubbed = true;
			}
		}

		calcParams.LOI = PZMCCDIS.data[0].GET_LOI;
		//Transfer MCC plan to skeleton flight plan table
		GMGMED("F30,1;");

		if (!scrubbed)
		{
			engine = SPSRCSDecision(SPS_THRUST / (CSMmass + LMmass), PZMCCDIS.data[0].DV_MCC);
			PoweredFlightProcessor(sv, CSMmass, PZMCCPLN.MidcourseGET, engine, LMmass, PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V, false, P30TIG, dV_LVLH);

			//Save burn data for block data calculation
			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
		}
		else
		{
			sprintf(upMessage, "MCC-%d has been scrubbed.", mccnum);
			DeltaV_LVLH = _V(0.0,0.0,0.0);
		}

		if (fcn > 20)
		{
			if (scrubbed)
			{
				char buffer1[1000];

				sprintf(upDesc, "CSM state vector and V66");

				AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);

				sprintf(uplinkdata, "%s", buffer1);
				if (upString != NULL) {
					// give to mcc
					strncpy(upString, uplinkdata, 1024 * 3);
				}
			}
			else
			{
				char buffer1[1000];
				char buffer2[1000];
				AP11ManPADOpt manopt;

				AP11MNV * form = (AP11MNV *)pad;

				manopt.dV_LVLH = dV_LVLH;
				manopt.enginetype = SPSRCSDecision(SPS_THRUST / (CSMmass + LMmass), dV_LVLH);
				manopt.HeadsUp = true;
				manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
				manopt.TIG = P30TIG;
				manopt.vessel = calcParams.src;
				manopt.vesseltype = 1;

				AP11ManeuverPAD(&manopt, *form);
				sprintf(form->purpose, "MCC-%d", mccnum);
				sprintf(form->remarks, "LM weight is %.0f.", form->LMWeight);

				AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
				CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

				sprintf(uplinkdata, "%s%s", buffer1, buffer2);
				if (upString != NULL) {
					// give to mcc
					strncpy(upString, uplinkdata, 1024 * 3);
					sprintf(upDesc, "CSM state vector and V66, target load");
				}
			}
		}
	}
	break;
	case 23: //Lunar Flyby PAD
	{
		RTEMoonOpt entopt;
		EntryResults res;
		AP11ManPADOpt opt;
		SV sv;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		entopt.SMODE = 14;
		entopt.RV_MCC = sv;
		entopt.TIGguess = calcParams.LOI - 5.0*3600.0;
		entopt.vessel = calcParams.src;
		entopt.t_zmin = 153.0*3600.0;
		entopt.entrylongmanual = false;
		entopt.ATPLine = 0; //MPL

		RTEMoonTargeting(&entopt, &res);

		SV sv_peri = FindPericynthion(res.sv_postburn);
		double h_peri = length(sv_peri.R) - OrbMech::R_Moon;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = SPSRCSDecision(SPS_THRUST / (calcParams.src->GetMass() + calcParams.tgt->GetMass()), res.dV_LVLH);
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.TIG = res.P30TIG;
		opt.vessel = calcParams.src;
		opt.vesseltype = 1;

		AP11ManeuverPAD(&opt, *form);

		sprintf(form->purpose, "Flyby");
		sprintf(form->remarks, "Height of pericynthion is %.0f NM", h_peri / 1852.0);
		form->lat = res.latitude*DEG;
		form->lng = res.longitude*DEG;
		form->RTGO = res.RTGO;
		form->VI0 = res.VIO / 0.3048;
		form->GET05G = res.GET05G;
	}
	break;
	case 24: //MCC-3
	{
		AP11ManPADOpt manopt;
		VECTOR3 dV_LVLH, dv;
		EphemerisData sv;
		double P30TIG, tig, CSMmass, LMmass;
		int engine;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);

		//Calculate MCC-3
		PZMCCPLN.MidcourseGET = calcParams.LOI - 22.0*3600.0;
		PZMCCPLN.Config = true;
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 2;
		PZMCCPLN.Mode = 1;
		CSMmass = calcParams.src->GetMass();
		LMmass = calcParams.tgt->GetMass();

		TranslunarMidcourseCorrectionProcessor(sv, CSMmass, LMmass);

		tig = GETfromGMT(PZMCCXFR.sv_man_bef[0].GMT);
		dv = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;

		//DV smaller than 3 ft/s? This is a simplified logic. The 3 ft/s check should be for MCC-4 and even if it is larger than that MCC-3 might not have been done
		if (length(dv) < 3.0*0.3048)
		{
			scrubbed = true;
		}

		if (scrubbed)
		{
			char buffer1[1000];

			sprintf(upMessage, "MCC-3 has been scrubbed");
			sprintf(upDesc, "CSM state vector and V66");

			AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);

			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
			}
		}
		else
		{
			char buffer1[1000];
			char buffer2[1000];

			calcParams.LOI = PZMCCDIS.data[0].GET_LOI;

			engine = SPSRCSDecision(SPS_THRUST / (CSMmass + LMmass), dv);
			PoweredFlightProcessor(sv, CSMmass, tig, engine, LMmass, dv, false, P30TIG, dV_LVLH);

			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = engine;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			manopt.TIG = P30TIG;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 1;

			AP11ManeuverPAD(&manopt, *form);
			sprintf(form->purpose, "MCC-3");

			AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load");
			}
		}
	}
	break;
	case 25: //MCC-4
	{
		VECTOR3 dV_LVLH;
		EphemerisData sv;
		double P30TIG, h_peri, h_peri_tgt, h_node, h_node_tgt, CSMmass, LMmass, ApsidRot;
		int engine, loisol;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);

		bool loierr = PMMLRBTI(sv);

		h_peri_tgt = PZSFPTAB.blocks[1].h_pc2 / 1852.0;
		h_node_tgt = PZSFPTAB.blocks[1].h_nd / 1852.0;

		if (loierr == false)
		{
			//Choose solution with the correct HPC
			double solht1, solht2;
			solht1 = PZLRBTI.sol[1].H_PC - PZLRBTI.HPLOI1;
			solht2 = PZLRBTI.sol[2].H_PC - PZLRBTI.HPLOI1;

			if (abs(solht1) > abs(solht2))
			{
				loisol = 2;
			}
			else
			{
				loisol = 1;
			}

			h_peri = PZLRBTI.h_pc;
			h_node = PZLRBTI.sol[loisol].H_ND;
			ApsidRot = PZLRBTI.sol[loisol].f_ND_E - PZLRBTI.sol[loisol].f_ND_H;
			while (ApsidRot > 180.0)
			{
				ApsidRot -= 360.0;
			}
			while (ApsidRot < -180.0)
			{
				ApsidRot += 360.0;
			}

			//Maneuver execution criteria
			if (h_peri > h_peri_tgt - 3.0 && h_peri < h_peri_tgt + 3.0)
			{
				if (h_node > h_node_tgt - 10.0 && h_node < h_node_tgt + 15.0)
				{
					if (abs(ApsidRot) < 45.0)
					{
						scrubbed = true;
						calcParams.LOI = PZLRBTI.sol[loisol].GETLOI;
					}
				}
			}
		}

		if (scrubbed)
		{
			char buffer1[1000];

			sprintf(upMessage, "MCC-4 has been scrubbed");
			sprintf(upDesc, "CSM state vector and V66");

			AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);

			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
			}

			DeltaV_LVLH = _V(0.0, 0.0, 0.0);
		}
		else
		{
			AP11ManPADOpt manopt;
			char buffer1[1000];
			char buffer2[1000];
			double tig;
			VECTOR3 dv;

			PZMCCPLN.MidcourseGET = calcParams.LOI - 5.0*3600.0;
			PZMCCPLN.Config = true;
			PZMCCPLN.Column = 1;
			PZMCCPLN.SFPBlockNum = 2;
			PZMCCPLN.Mode = 1;
			CSMmass = calcParams.src->GetMass();
			LMmass = calcParams.tgt->GetMass();

			TranslunarMidcourseCorrectionProcessor(sv, CSMmass, LMmass);
			calcParams.LOI = PZMCCDIS.data[0].GET_LOI;

			tig = GETfromGMT(PZMCCXFR.sv_man_bef[0].GMT);
			dv = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;

			engine = SPSRCSDecision(SPS_THRUST / (CSMmass + LMmass), dv);
			PoweredFlightProcessor(sv, CSMmass, tig, engine, LMmass, dv, false, P30TIG, dV_LVLH);

			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = engine;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			manopt.TIG = P30TIG;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 1;

			AP11ManeuverPAD(&manopt, *form);
			sprintf(form->purpose, "MCC-4");

			AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load");
			}

			//Save burn data for PC+2 calculation
			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
		}
	}
	break;
	case 26: //PC+2 UPDATE
	{
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		RTEMoonOpt entopt;
		EntryResults res;
		AP11ManPADOpt opt;
		SV sv, sv1;
		double tig, P30TIG, CSMmass, LMmass, GET_SV1;
		int loisol;
		VECTOR3 dV_LVLH, dv;

		EphemerisData sv_e;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src);
		CSMmass = calcParams.src->GetMass();
		LMmass = calcParams.tgt->GetMass();

		//Simulate MCC-4 if not scrubbed
		if (length(DeltaV_LVLH) > 0.0)
		{
			sv1 = ExecuteManeuver(sv, TimeofIgnition, DeltaV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);
		}
		else
		{
			sv1 = sv;
		}

		GET_SV1 = OrbMech::GETfromMJD(sv1.MJD, CalcGETBase());

		sv_e.GMT = GMTfromGET(GET_SV1);
		sv_e.R = sv1.R;
		sv_e.V = sv1.V;
		sv_e.RBI = BODY_MOON;

		//Calculate LOI REFSMMAT
		PMMLRBTI(sv_e);

		//Choose solution with the correct HPC
		double solht1, solht2;
		solht1 = PZLRBTI.sol[1].H_PC - PZLRBTI.HPLOI1;
		solht2 = PZLRBTI.sol[2].H_PC - PZLRBTI.HPLOI1;

		if (abs(solht1) > abs(solht2))
		{
			loisol = 2;
		}
		else
		{
			loisol = 1;
		}

		tig = GETfromGMT(PZLRBELM.sv_man_bef[loisol].GMT);
		dv = PZLRBELM.V_man_after[loisol] - PZLRBELM.sv_man_bef[loisol].V;
		PoweredFlightProcessor(sv_e, CSMmass, tig, RTCC_ENGINETYPE_CSMSPS, LMmass, dv, false, P30TIG, dV_LVLH);

		refsopt.REFSMMATopt = 0;
		refsopt.dV_LVLH = dV_LVLH;
		refsopt.REFSMMATTime = P30TIG;
		refsopt.HeadsUp = false;
		refsopt.vessel = calcParams.src;
		refsopt.csmlmdocked = true;

		REFSMMAT = REFSMMATCalc(&refsopt);

		entopt.returnspeed = 2;
		entopt.SMODE = 14;
		entopt.RV_MCC = sv1;
		entopt.vessel = calcParams.src;
		entopt.TIGguess = calcParams.LOI + 2.0*3600.0;
		PZREAP.VRMAX = 37500.0;
		entopt.entrylongmanual = false;
		entopt.ATPLine = 0; //MPL

		RTEMoonTargeting(&entopt, &res);

		//Reset to default
		PZREAP.VRMAX = 36323.0;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = false;
		opt.REFSMMAT = REFSMMAT; //LOI REFSMMAT
		opt.TIG = res.P30TIG;
		opt.vessel = calcParams.src;
		opt.vesseltype = 1;

		AP11ManeuverPAD(&opt, *form);
		sprintf(form->remarks, "Assumes LOI REFSMMAT and docked");

		if (!REFSMMATDecision(form->Att*RAD))
		{
			refsopt.dV_LVLH = res.dV_LVLH;
			refsopt.REFSMMATTime = res.P30TIG;
			refsopt.REFSMMATopt = 0;

			REFSMMAT = REFSMMATCalc(&refsopt);

			opt.HeadsUp = true;
			opt.REFSMMAT = REFSMMAT;
			AP11ManeuverPAD(&opt, *form);

			sprintf(form->remarks, "Docked, requires realignment to preferred REFSMMAT");
		}
		sprintf(form->purpose, "PC+2");
		form->lat = res.latitude*DEG;
		form->lng = res.longitude*DEG;
		form->RTGO = res.RTGO;
		form->VI0 = res.VIO / 0.3048;
		form->GET05G = res.GET05G;
		form->type = 2;

		//Also calculate and save Landing Site REFSMMAT here
		refsopt.LSAzi = calcParams.LSAzi;
		refsopt.LSLat = BZLAND.lat[RTCC_LMPOS_BEST];
		refsopt.LSLng = BZLAND.lng[RTCC_LMPOS_BEST];
		refsopt.REFSMMATopt = 8;
		refsopt.REFSMMATTime = CZTDTGTU.GETTD;

		REFSMMAT = REFSMMATCalc(&refsopt);

		//Store as LM LLD matrix
		EMGSTSTM(RTCC_MPT_LM, REFSMMAT, RTCC_REFSMMAT_TYPE_LLD, RTCCPresentTimeGMT());
		//Also move to CSM LCV
		GMGMED("G00,LEM,LLD,CSM,LCV;");
	}
	break;
	case 27: // LOI MANEUVER (PRELIMINARY)
	case 28: // LOI MANEUVER
	{
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		AP11ManPADOpt manopt;
		double P30TIG, tig, CSMmass, LMmass;
		VECTOR3 dV_LVLH, dv;
		int loisol;
		EphemerisData sv;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src); //State vector for uplink
		CSMmass = calcParams.src->GetMass();
		LMmass = calcParams.tgt->GetMass();

		PMMLRBTI(sv);

		//Choose solution with the correct HPC
		double solht1, solht2;
		solht1 = PZLRBTI.sol[1].H_PC - PZLRBTI.HPLOI1;
		solht2 = PZLRBTI.sol[2].H_PC - PZLRBTI.HPLOI1;

		if (abs(solht1) > abs(solht2))
		{
			loisol = 2;
		}
		else
		{
			loisol = 1;
		}

		tig = GETfromGMT(PZLRBELM.sv_man_bef[loisol].GMT);
		dv = PZLRBELM.V_man_after[loisol] - PZLRBELM.sv_man_bef[loisol].V;
		PoweredFlightProcessor(sv, CSMmass, tig, RTCC_ENGINETYPE_CSMSPS, LMmass, dv, false, P30TIG, dV_LVLH);

		refsopt.REFSMMATopt = 0;
		refsopt.dV_LVLH = dV_LVLH;
		refsopt.REFSMMATTime = P30TIG;
		refsopt.HeadsUp = false;
		refsopt.vessel = calcParams.src;
		refsopt.csmlmdocked = true;

		REFSMMAT = REFSMMATCalc(&refsopt);

		manopt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		manopt.dV_LVLH = dV_LVLH;
		manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		manopt.HeadsUp = false;
		manopt.TIG = P30TIG;
		manopt.vessel = calcParams.src;
		manopt.vesseltype = 1;

		if (fcn == 27)
		{
			manopt.REFSMMAT = REFSMMAT;
		}
		else
		{
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		}

		AP11ManeuverPAD(&manopt, *form);
		sprintf(form->purpose, "LOI");
		sprintf(form->remarks, "LM weight is %.0f", form->LMWeight);

		TimeofIgnition = P30TIG;
		DeltaV_LVLH = dV_LVLH;

		calcParams.LOI = P30TIG;

		char buffer1[1000];
		char buffer2[1000];

		AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

		if (fcn == 27)
		{
			char buffer3[1000];

			AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

			sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load, desired orientation (LOI)");
			}
		}
		else
		{
			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load");
			}
		}
	}
	break;
	case 29: //DOI-1 calculation
	case 30: //DOI-1 update
	{
		double P30TIG, SV_GET, Peri_GET, LONGP_tgt, HA_tgt, HA_max, HA_min, dlp, dlp_last, TIG_imp, t_P;
		VECTOR3 dV_LVLH, dV_imp;
		SV sv1, sv2, sv_peri;
		EphemerisData sv_e;
		GMPOpt orbopt;
		PMMMPTInput in;

		sv_e = StateVectorCalcEphem(calcParams.src);
		sv1 = StateVectorCalc(calcParams.src); //State vector for uplink

		//Find pre DOI-1 periapsis GET
		double revsfromLOI = 2.0;
		SV_GET = OrbMech::GETfromMJD(sv1.MJD, CalcGETBase());
		t_P = OrbMech::period(sv1.R, sv1.V, OrbMech::mu_Moon);
		sv2 = coast(sv1, (t_P*revsfromLOI - (SV_GET - calcParams.LOI)));
		sv_peri = FindPericynthion(sv2);
		Peri_GET = OrbMech::GETfromMJD(sv_peri.MJD, CalcGETBase());

		LONGP_tgt = BZLAND.lng[RTCC_LMPOS_BEST]*DEG + 0.61; //Periapsis 10 degrees west of landing site at PDI
		HA_tgt = 59.5*1852.0;
		HA_max = HA_tgt + 1.0*1852.0;
		HA_min = HA_tgt - 1.0*1852.0;

		orbopt.ManeuverCode = RTCC_GMP_HBT;
		orbopt.sv_in = sv_e;
		orbopt.H_A = HA_tgt;
		orbopt.H_P = 13.17*1852.0;
		orbopt.TIG_GET = Peri_GET - 2.0*60.0 - 30.0;

		GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);

		dlp = dlp_last = OrbMech::calculateDifferenceBetweenAngles(LONGP_tgt*RAD, PZGPMDIS.long_P);

		if (abs(dlp*DEG) < 20.0)
		{
			//Iterations to find desired periapsis longitude
			int n = 0, nmax = 100;
			double incr = 0.0;

			orbopt.TIG_GET = orbopt.TIG_GET - 30.0;

			//Iterate on TIG
			while (abs(dlp*DEG) > 0.1 && abs(dlp) <= abs(dlp_last) && orbopt.TIG_GET < Peri_GET - 5.0)
			{
				dlp_last = OrbMech::calculateDifferenceBetweenAngles(LONGP_tgt*RAD, PZGPMDIS.long_P);

				incr = dlp * DEG / abs(dlp*DEG);

				orbopt.TIG_GET += incr;

				GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);

				dlp = OrbMech::calculateDifferenceBetweenAngles(LONGP_tgt*RAD, PZGPMDIS.long_P);
			}

			//Iterate on apoapsis height
			while (abs(dlp*DEG) > 0.1 && PZGPMDIS.HA < HA_max && PZGPMDIS.HA > HA_min && n < nmax)
			{
				incr = -dlp * DEG / (abs(dlp*DEG) * 10.0);

				orbopt.H_A += incr * 1852.0;

				GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);

				dlp = OrbMech::calculateDifferenceBetweenAngles(LONGP_tgt*RAD, PZGPMDIS.long_P);

				n++;
			}
		}

		in.CONFIG = 13; //CSM+LM
		in.CSMWeight = calcParams.src->GetMass();
		in.sv_before = PZGPMELM.SV_before;
		in.V_aft = PZGPMELM.V_after;
		in.DETU = 15.0; //Ullage
		in.UT = true; //4 jets
		in.IgnitionTimeOption = false;
		in.IterationFlag = true;
		in.LMWeight = calcParams.tgt->GetMass();
		in.Thruster = RTCC_ENGINETYPE_CSMSPS;
		in.VC = RTCC_MANVEHICLE_CSM;
		in.VehicleArea = PZMPTCSM.ConfigurationArea;
		in.HeadsUpIndicator = true;

		double GMT_TIG;
		PoweredFlightProcessor(in, GMT_TIG, dV_LVLH);
		P30TIG = GETfromGMT(GMT_TIG);

		ExecuteManeuver(sv1, P30TIG, dV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);

		TimeofIgnition = P30TIG;
		DeltaV_LVLH = dV_LVLH;

		if (fcn == 30)
		{
			AP11ManPADOpt manopt;
			AP11MNV * form = (AP11MNV *)pad;

			char buffer1[1000];
			char buffer2[1000];

			manopt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			manopt.sxtstardtime = -40.0*60.0;
			manopt.TIG = P30TIG;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 1;

			AP11ManeuverPAD(&manopt, *form);
			sprintf(form->purpose, "DOI-1");

			AGCStateVectorUpdate(buffer1, sv1, true, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load");
			}
		}
	}
	break;
	case 31: //LS REFSMMAT UPLINK (post LOI)
	case 32: //LS REFSMMAT UPLINK (post DOI-1)
	{
		MATRIX3 REFSMMAT;
		SV sv0, sv1;
		REFSMMATOpt opt;
		char buffer1[1000];

		sv0 = StateVectorCalc(calcParams.src);

		if (fcn == 31)
		{
			sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);
		}
		else
		{
			sv1 = sv0;
		}

		//MED K17
		GZGENCSN.LDPPPoweredDescentSimFlag = false;
		GZGENCSN.LDPPDwellOrbits = 0;
		//MED K16
		med_k16.Mode = 4;
		med_k16.Sequence = 1;
		med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.LOI + 22.5*3600.0;

		LunarDescentPlanningProcessor(sv1);

		calcParams.DOI = GETfromGMT(PZLDPELM.sv_man_bef[0].GMT);
		calcParams.PDI = PZLDPDIS.PD_GETIG;
		CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;

		opt.LSLat = BZLAND.lat[RTCC_LMPOS_BEST];
		opt.LSLng = BZLAND.lng[RTCC_LMPOS_BEST];
		opt.REFSMMATopt = 5;
		opt.REFSMMATTime = CZTDTGTU.GETTD;
		opt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&opt);
		//Store as LM LLD
		EMGSTSTM(RTCC_MPT_LM, REFSMMAT, RTCC_REFSMMAT_TYPE_LLD, RTCCPresentTimeGMT());
		//Also move to CSM LCV
		GMGMED("G00,LEM,LLD,CSM,LCV;");

		AGCDesiredREFSMMATUpdate(buffer1, REFSMMAT);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "LS REFSMMAT");
		}
	}
	break;
	case 33: //LOI EVALUATION
	{
		SV sv;
		OELEMENTS coe;

		sv = StateVectorCalc(calcParams.src);

		coe = OrbMech::coe_from_sv(sv.R, sv.V, OrbMech::mu_Moon);

		if (coe.e > 0.7)
		{
			sprintf(upMessage, "LOI scrubbed, burn PC+2 PAD");
			scrubbed = true;
			mcc->AbortMode = 6;
		}
	}
	break;
	case 35: //LGC ACTIVATION UPDATE
	{
		SV sv;
		MATRIX3 REFSMMAT;
		char buffer1[1000];
		char buffer2[1000];

		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		REFSMMAT = EZJGMTX3.data[RTCC_REFSMMAT_TYPE_LLD - 1].REFSMMAT;

		AGCStateVectorUpdate(buffer1, sv, false, true);
		AGCREFSMMATUpdate(buffer2, REFSMMAT, false);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "LM state vector and V66, LS REFSMMAT");
		}
	}
	break;
	case 36: //ABORT CONSTANTS UPDATE
	{
		/*AP11AGSACT *form = (AP11AGSACT*)pad;

		SV svcm0, svcm1, svlm0, svlm1;
		double t_sunrise, t_TPI;
		int emem[14];
		char buffer1[1000];

		PDAPOpt opt;
		PDAPResults res;

		svcm0 = StateVectorCalc(calcParams.src);
		svlm0 = StateVectorCalc(calcParams.tgt);

		if (mcc->MoonRev >= 13)
		{
			svcm1 = svcm0;
			svlm1 = svlm0;
		}
		else
		{
			svcm1 = ExecuteManeuver(svcm0, TimeofIgnition, DeltaV_LVLH, 0.0, RTCC_ENGINETYPE_CSMSPS);
			svlm1 = ExecuteManeuver(svlm0, calcParams.TIGSTORE1, calcParams.DVSTORE1, 0, RTCC_ENGINETYPE_LMRCSMINUS4);
		}

		t_sunrise = calcParams.PDI + 3.0*3600.0;
		t_TPI = FindOrbitalSunrise(svcm1, t_sunrise) - 16.0*60.0;

		opt.dt_stage = 999999.9;
		opt.W_TAPS = 4711.0;
		opt.W_TDRY = 6874.3;
		opt.dt_step = 20.0;
		opt.t_TPI = t_TPI;
		opt.IsTwoSegment = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		opt.sv_A = svlm1;
		opt.sv_P = svcm1;
		opt.TLAND = CZTDTGTU.GETTD;

		PoweredDescentAbortProgram(opt, res);

		form->DEDA224 = (int)(res.DEDA224 / 0.3048 / 100.0);
		form->DEDA225 = (int)(res.DEDA225 / 0.3048 / 100.0);
		form->DEDA226 = (int)(res.DEDA226 / 0.3048 / 100.0);
		form->DEDA227 = OrbMech::DoubleToDEDA(res.DEDA227 / 0.3048*pow(2, -20), 14);

		emem[0] = 16;
		emem[1] = 2550;
		emem[2] = OrbMech::DoubleToBuffer(res.J1, 23, 1);
		emem[3] = OrbMech::DoubleToBuffer(res.J1, 23, 0);
		emem[4] = OrbMech::DoubleToBuffer(res.K1*PI2, 23, 1);
		emem[5] = OrbMech::DoubleToBuffer(res.K1*PI2, 23, 0);
		emem[6] = OrbMech::DoubleToBuffer(res.J2, 23, 1);
		emem[7] = OrbMech::DoubleToBuffer(res.J2, 23, 0);
		emem[8] = OrbMech::DoubleToBuffer(res.K2*PI2, 23, 1);
		emem[9] = OrbMech::DoubleToBuffer(res.K2*PI2, 23, 0);
		emem[10] = OrbMech::DoubleToBuffer(res.Theta_LIM / PI2, 0, 1);
		emem[11] = OrbMech::DoubleToBuffer(res.Theta_LIM / PI2, 0, 0);
		emem[12] = OrbMech::DoubleToBuffer(res.R_amin, 24, 1);
		emem[13] = OrbMech::DoubleToBuffer(res.R_amin, 24, 0);

		V7XUpdate(71, buffer1, emem, 14);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "LGC abort constants");
		}*/

		sprintf(upMessage, "Use abort constants in the checklist");
	}
	break;
	case 37: //UNDOCK & SEPARATION MANEUVER
	{
		AP11ManPADOpt opt;
		SV sv;
		VECTOR3 dV_LVLH;
		double t_Sep, t_lng, ttoLNG;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		t_lng = OrbMech::P29TimeOfLongitude(sv.R, sv.V, sv.MJD, GetGravref(BODY_MOON), 136.0*RAD);
		ttoLNG = (t_lng - sv.MJD)*24.0 * 3600.0;
		t_Sep = (sv.MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;

		calcParams.SEP = t_Sep;
		dV_LVLH = _V(0, 0, -1)*0.3048;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSPLUS4;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.TIG = t_Sep;
		opt.vessel = calcParams.src;
		opt.vesseltype = 0;

		AP11ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "CSM SEP");
		form->type = 2;
	}
	break;
	case 38: //DOI-2
	{
		AP11LMManPADOpt opt;
		SV sv;
		AP11LMMNV * form = (AP11LMMNV *)pad;
		double P30TIG;
		VECTOR3 dV_LVLH;

		sv = StateVectorCalc(calcParams.tgt);

		//MED K17
		GZGENCSN.LDPPPoweredDescentSimFlag = false;
		GZGENCSN.LDPPDwellOrbits = 0;
		//MED K16
		med_k16.Mode = 4;
		med_k16.Sequence = 1;
		med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.LOI + 22.5*3600.0;

		LunarDescentPlanningProcessor(sv);

		calcParams.DOI = GETfromGMT(PZLDPELM.sv_man_bef[0].GMT);
		calcParams.PDI = PZLDPDIS.PD_GETIG;
		CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;

		PoweredFlightProcessor(sv, calcParams.DOI, RTCC_ENGINETYPE_LMRCSMINUS4, 0.0, PZLDPELM.V_man_after[0] - PZLDPELM.sv_man_bef[0].V, false, P30TIG, dV_LVLH);

		calcParams.TIGSTORE1 = P30TIG;
		calcParams.DVSTORE1 = dV_LVLH;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.csmlmdocked = false;
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_LMRCSMINUS4;
		opt.HeadsUp = false;
		opt.REFSMMAT = EZJGMTX3.data[RTCC_REFSMMAT_TYPE_LLD - 1].REFSMMAT;
		opt.TIG = P30TIG;
		opt.vessel = calcParams.tgt;

		double AGSTIG = P30TIG - GETfromGMT(GetAGSClockZero());

		AP11LMManeuverPAD(&opt, *form);
		sprintf(form->purpose, "DOI-2");
		sprintf(form->remarks, "TIG AGS 373: %+05.1f", AGSTIG / 60.0);
	}
	break;
	case 39: //CSM CIRC calculation
	case 40: //CSM CIRC update
	{
		SV sv = StateVectorCalc(calcParams.src);

		if (fcn == 39)
		{
			double P30TIG, TIG_imp;
			VECTOR3 dV_LVLH, dV_imp;
			EphemerisData sv_e;

			GMPOpt orbopt;
			PMMMPTInput in;

			sv_e = StateVectorCalcEphem(calcParams.src);

			orbopt.sv_in = sv_e;
			orbopt.TIG_GET = calcParams.DOI - 1.0*3600.0;

			//Nominal targets
			orbopt.ManeuverCode = RTCC_GMP_HBL;
			orbopt.H_A = 70.3*1852.0;
			orbopt.H_P = 54.3*1852.0;
			orbopt.long_D = -142.0*RAD;

			//Circularize at 60 NM
			//orbopt.ManeuverCode = RTCC_GMP_CRH;
			//orbopt.H_D = 60.0*1852.0;

			GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);

			in.CONFIG = 1; //CSM
			in.CSMWeight = calcParams.src->GetMass();
			in.sv_before = PZGPMELM.SV_before;
			in.V_aft = PZGPMELM.V_after;
			in.DETU = 12.0; //Ullage
			in.UT = true; //4 jets
			in.IgnitionTimeOption = false;
			in.IterationFlag = true;
			in.LMWeight = 0.0;
			in.Thruster = RTCC_ENGINETYPE_CSMSPS;
			in.VC = RTCC_MANVEHICLE_CSM;
			in.VehicleArea = PZMPTCSM.ConfigurationArea;
			in.HeadsUpIndicator = true;

			double GMT_TIG;
			PoweredFlightProcessor(in, GMT_TIG, dV_LVLH);
			P30TIG = GETfromGMT(GMT_TIG);

			ExecuteManeuver(sv, P30TIG, dV_LVLH, 0, RTCC_ENGINETYPE_CSMSPS);

			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
		}
		else
		{
			AP11ManPADOpt manopt;
			AP11MNV * form = (AP11MNV *)pad;

			char buffer1[1000];
			char buffer2[1000];

			manopt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
			manopt.dV_LVLH = DeltaV_LVLH;
			manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
			manopt.HeadsUp = true;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			//manopt.sxtstardtime = -40.0*60.0;
			manopt.TIG = TimeofIgnition;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 0;

			AP11ManeuverPAD(&manopt, *form);
			sprintf(form->purpose, "CIRC");
			sprintf(form->remarks, "Four-jet ullage for 12 seconds");

			//Coast SV to CIRC-10min
			double svt = TimeofIgnition - 10.0*60.0;
			double SV_GET = OrbMech::GETfromMJD(sv.MJD, CalcGETBase());
			SV svc = coast(sv, svt - SV_GET);

			AGCStateVectorUpdate(buffer1, svc, true);
			CMCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector (CIRC-10), target load");
			}
		}
	}
	break;
	case 41: //TEI-4 UPDATE (PRE LOI)
	case 42: //TEI-5 UPDATE (PRE DOI-1)
	case 43: //TEI-12 UPDATE
	case 44: //TEI-19 UPDATE (PRE CIRC)
	case 45: //TEI-26 UPDATE
	case 46: //TEI-38 UPDATE
	case 47: //TEI-49 UPDATE
	case 48: //TEI-55 UPDATE PRE LOPC
	case 49: //TEI-65 UPDATE
	case 50: //TEI-72 UPDATE
	case 51: //TEI-75 UPDATE (PRELIM)
	case 52: //TEI-75 UPDATE (FINAL)
	case 53: //TEI-76 UPDATE
	{
		AP11ManPADOpt opt;
		double GET_SV1, returnspeed, DT_TEI_EI, AbortGuess, MoonRevTEI;
		SV sv0, sv1;
		char manname[8];
		EphemerisData sv_e;

		AP11MNV * form = (AP11MNV *)pad;

		sv0 = StateVectorCalc(calcParams.src); //State vector for uplink

		GMGMED("F79,0;"); //Clear abort scan table

		//Simulate the maneuver preceeding TEI
		if ((fcn == 41 || fcn == 42 || fcn == 44 || fcn == 48))
		{
			sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);
		}
		else
		{
			sv1 = sv0;
		}

		GET_SV1 = OrbMech::GETfromMJD(sv1.MJD, CalcGETBase());

		sv_e.GMT = GMTfromGET(GET_SV1);
		sv_e.R = sv1.R;
		sv_e.V = sv1.V;
		sv_e.RBI = BODY_MOON;

		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		returnspeed = 1;
		med_f75_f77.Inclination = 0.0;

		std::string site = med_f77.Site;

		if (fcn == 41)
		{
			sprintf(manname, "TEI-4");
			MoonRevTEI = 4.25;
			opt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT; //Landing site REFSMMAT
			returnspeed = 0;
		}
		else if (fcn == 42)
		{
			sprintf(manname, "TEI-5");
			MoonRevTEI = 4.875;
			returnspeed = 0;
		}
		else if (fcn == 43)
		{
			sprintf(manname, "TEI-12");
			MoonRevTEI = 11.5;
			opt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT; //Landing site REFSMMAT
			returnspeed = 0;
		}
		else if (fcn == 44)
		{
			sprintf(manname, "TEI-19");
			MoonRevTEI = 18.5;
			opt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT; //Landing site REFSMMAT
			returnspeed = 0;
		}
		else if (fcn == 45)
		{
			sprintf(manname, "TEI-26");
			MoonRevTEI = 25.5;
			returnspeed = 0;
		}
		else if (fcn == 46)
		{
			sprintf(manname, "TEI-38");
			MoonRevTEI = 37.5;
			returnspeed = 0;
		}
		else if (fcn == 47)
		{
			sprintf(manname, "TEI-49");
			MoonRevTEI = 48.3;
		}
		else if (fcn == 48)
		{
			sprintf(manname, "TEI-55");
			MoonRevTEI = 54.0;
			returnspeed = 0;
		}
		else if (fcn == 49)
		{
			sprintf(manname, "TEI-65");
			MoonRevTEI = 64.5;
			returnspeed = 0;
		}
		else if (fcn == 50)
		{
			sprintf(manname, "TEI-72");
			MoonRevTEI = 70.9;
		}
		else if (fcn == 51) //Preliminary
		{
			sprintf(manname, "TEI-75");
			MoonRevTEI = 73.9;
			site = PZREAP.PTPSite[0];
			med_f75_f77.Inclination = 66.5*RAD; //From SCOT (72-FM-187) pg 4-146
		}
		else if (fcn == 52) //Final
		{
			sprintf(manname, "TEI-%d", mcc->MoonRev);
			MoonRevTEI = ((double)mcc->MoonRev) - 1.1;
			site = PZREAP.PTPSite[0];
			med_f75_f77.Inclination = 66.5*RAD; //From SCOT (72-FM-187) pg 4-146
		}
		else if (fcn == 53)
		{
			sprintf(manname, "TEI-%d", mcc->MoonRev + 1);
			MoonRevTEI = ((double)mcc->MoonRev + 1) - 1.1;
			site = PZREAP.PTPSite[0];
			med_f75_f77.Inclination = 66.5*RAD; //From SCOT (72-FM-187) pg 4-146
		}

		AbortGuess = calcParams.LOI + (MoonRevTEI * 2.0*3600.0);

		DT_TEI_EI = 62.0*3600.0;

		if (returnspeed == 0)
		{
			DT_TEI_EI += 24.0*3600.0;
		}
		else if (returnspeed == 2)
		{
			DT_TEI_EI -= 24.0*3600.0;
		}

		VEHDATABUF.csmmass = calcParams.src->GetMass();
		VEHDATABUF.lmascmass = 0.0;
		VEHDATABUF.lmdscmass = 0.0;
		VEHDATABUF.sv = sv_e;
		VEHDATABUF.config = "C";

		med_f75_f77.T_0_min = AbortGuess;
		med_f75_f77.T_Z = AbortGuess + DT_TEI_EI;

		DetermineRTESite(site);

		PZREAP.RTEVectorTime = GMTfromGET(med_f75_f77.T_V) / 3600.0;
		PZREAP.RTET0Min = GMTfromGET(med_f75_f77.T_0_min) / 3600.0;
		PZREAP.RTETimeOfLanding = GMTfromGET(med_f75_f77.T_Z) / 3600.0;
		PZREAP.RTEPTPMissDistance = med_f77.MissDistance;

		PMMREAST(77, &sv_e);

		med_f80.ASTCode = PZREAP.AbortScanTableData[0].ASTCode;
		med_f80.ManeuverCode = "CSUX";
		med_f80.REFSMMAT = "TEI";
		med_f80.HeadsUp = false;
		med_f80.NumQuads = 4;
		med_f80.UllageDT = 12.0;

		PMMREDIG(false);

		if (fcn == 51)
		{
			GMGMED("G11,CSM,REP;");
			GMGMED("G00,CSM,DOD,CSM,LCV;");

			opt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT;
		}

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = PZREAP.RTEDTable[0].DV_XDV;;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = false;
		opt.RV_MCC = sv1;
		opt.TIG = PZREAP.RTEDTable[0].GETI;
		opt.useSV = true;
		opt.vessel = calcParams.src;
		opt.vesseltype = 0;

		AP11ManeuverPAD(&opt, *form);

		RMMYNIInputTable entin;
		RMMYNIOutputTable entout;
		EphemerisData2 sv_EI_ECT;

		ELVCNV(PZREAP.AbortScanTableData[0].sv_EI, 0, 1, sv_EI_ECT);

		entin.R0 = sv_EI_ECT.R;
		entin.V0 = sv_EI_ECT.V;
		entin.GMT0 = sv_EI_ECT.GMT;
		entin.lat_T = PZREAP.RTEDTable[0].lat_imp_tgt;
		entin.lng_T = PZREAP.RTEDTable[0].lng_imp_tgt;
		entin.KSWCH = 3;

		RMMYNI(entin, entout);

		sprintf(form->purpose, manname);
		form->lat = PZREAP.RTEDTable[0].lat_imp_tgt*DEG;
		form->lng = PZREAP.RTEDTable[0].lng_imp_tgt*DEG;
		form->RTGO = entout.R_EMS / 1852.0;
		form->VI0 = entout.V_EMS / 0.3048;
		form->GET05G = GETfromGMT(entout.t_05g);
		form->type = 2;

		sprintf(form->remarks, "Four-jet ullage for 12 seconds");

		if (fcn == 41)
		{
			sprintf(form->remarks, "%s,\nUndocked, assumes no DOI-1", form->remarks);
		}
		else if (fcn == 42)
		{
			sprintf(form->remarks, "%s,\nUndocked, assumes DOI-1", form->remarks);
		}
		else if (fcn == 43)
		{
			sprintf(form->remarks, "%s,\nUndocked, assumes no CIRC", form->remarks);
		}
		else if (fcn == 44)
		{
			sprintf(form->remarks, "%s,\nUndocked, assumes CIRC", form->remarks);
		}
		else if (fcn == 48)
		{
			sprintf(form->remarks, "%s,\nAssumes LOPC", form->remarks);
		}
		else if (fcn == 51 || fcn == 52)
		{
			form->type = 1;
		}

		if (fcn != 53)
		{
			//Save parameters for further use
			SplashLatitude = PZREAP.RTEDTable[0].lat_imp_tgt;
			SplashLongitude = PZREAP.RTEDTable[0].lng_imp_tgt;
			calcParams.TEI = PZREAP.RTEDTable[0].GETI;
			calcParams.EI = PZREAP.RTEDTable[0].ReentryPET;
		}

		if (fcn == 52)
		{
			char buffer1[1000], buffer2[1000];
			TimeofIgnition = PZREAP.RTEDTable[0].GETI;
			DeltaV_LVLH = PZREAP.RTEDTable[0].DV_XDV;

			AGCStateVectorUpdate(buffer1, sv1, true, true);
			CMCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load");
			}
		}
	}
	break;
	case 55: //K-FACTOR
	{
		GENERICPAD * form = (GENERICPAD *)pad;

		double KFactor, agsclockzero;

		int hh, mm;
		double ss;

		LEM *l = (LEM*)calcParams.tgt;

		bool res_k = CalculateAGSKFactor(&l->agc.vagc, &l->aea.vags, KFactor);
		if (res_k)
		{
			SystemParameters.MCGZSS = SystemParameters.MCGZSL + KFactor / 3600.0;
		}

		agsclockzero = GETfromGMT(GetAGSClockZero());
		OrbMech::SStoHHMMSS(agsclockzero, hh, mm, ss);

		sprintf(form->paddata, "K-FACTOR: %03d:%02d:%.2lf", hh, mm, ss);//
	}
	break;
	case 56: //REV 12 LS TCA
	{
		GENERICPAD * form = (GENERICPAD *)pad;

		double t_lng, ttoLNG, GET;
		SV sv;

		int hh, mm;
		double ss;

		sv = StateVectorCalc(calcParams.tgt);

		t_lng = OrbMech::P29TimeOfLongitude(sv.R, sv.V, sv.MJD, GetGravref(BODY_MOON), BZLAND.lng[RTCC_LMPOS_BEST]);
		ttoLNG = (t_lng - sv.MJD)*24.0 * 3600.0;
		GET = (sv.MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;
		OrbMech::SStoHHMMSS(GET, hh, mm, ss);

		sprintf(form->paddata, "REV 12 LS TCA: %d:%d:%.2lf", hh, mm, ss);
	}
	break;
	case 57: //P76 PAD for DOI-2
	case 58: //P76 PAD for CIRC
	{
		AP11P76PAD * form = (AP11P76PAD*)pad;

		if (fcn == 53)
		{
			form->entries = 1;
			sprintf(form->purpose[0], "DOI-2");
			form->TIG[0] = calcParams.TIGSTORE1;
			form->DV[0] = calcParams.DVSTORE1 / 0.3048;
		}
		else
		{
			form->entries = 1;
			sprintf(form->purpose[0], "CSM CIRC");
			form->TIG[0] = TimeofIgnition;
			form->DV[0] = DeltaV_LVLH / 0.3048;
		}
	}
	break;
	case 59: //LM P22 ACQUISITION TIME
	{
		LMP22ACQPAD * form = (LMP22ACQPAD*)pad;

		SV sv0, sv1;
		VECTOR3 R_P;
		double alt, lat, lng, dt2, LmkTime, MJDguess, LmkRange;

		sv0 = StateVectorCalc(calcParams.src);

		LmkTime = calcParams.LunarLiftoff - 2 * 3600.0 - 30.0*60.0;

		alt = BZLAND.rad[RTCC_LMPOS_BEST] - OrbMech::R_Moon;;
		lat = BZLAND.lat[RTCC_LMPOS_BEST];
		lng = BZLAND.lng[RTCC_LMPOS_BEST];
		MJDguess = CalcGETBase() + LmkTime / 24.0 / 3600.0;
		sv1 = coast(sv0, (MJDguess - sv0.MJD)*24.0*3600.0);
		R_P = unit(_V(cos(lng)*cos(lat), sin(lng)*cos(lat), sin(lat)))*(oapiGetSize(sv1.gravref) + alt);
		dt2 = OrbMech::findelev_gs(sv1.R, sv1.V, R_P, MJDguess, 152.0*RAD, sv1.gravref, LmkRange);

		form->P22_ACQ_GET = dt2 + (MJDguess - CalcGETBase()) * 24.0 * 60.0 * 60.0;
	}
	break;
	case 62: //LANDMARK TRACKING PAD (J3)
	case 63: //LANDMARK TRACKING PAD (17-X)
	case 64: //LANDMARK TRACKING PAD (17-1)
	case 65: //LANDMARK TRACKING PAD (RP-3)
	case 66: //LANDMARK TRACKING PAD (F-1 & 17-1)
	{
		LMARKTRKPADOpt opt;
		SV sv0;
		double GET_SV;

		AP11LMARKTRKPAD * form = (AP11LMARKTRKPAD *)pad;

		sv0 = StateVectorCalc(calcParams.src);

		opt.sv0 = sv0;

		GET_SV = OrbMech::GETfromMJD(sv0.MJD, CalcGETBase());

		if (fcn == 62) //LANDMARK TRACKING PAD (J3)
		{
			SV sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);

			opt.sv0 = sv1;
			sprintf(form->LmkID[0], "J-3");
			opt.alt[0] = 0.0;
			opt.lat[0] = 19.948*RAD;
			opt.LmkTime[0] = TimeofIgnition + 10.0*60.0;
			opt.lng[0] = 40.102*RAD;
			opt.entries = 1;
		}
		else if (fcn == 63) //LANDMARK TRACKING PAD (17-X)
		{
			SV sv_peri = FindPericynthion(sv0);
			double h_peri = length(sv_peri.R) - OrbMech::R_Moon;

			opt.LmkTime[0] = calcParams.SEP + 30.0*60.0;
			opt.entries = 1;

			if (h_peri < 10.0*1852.0)
			{
				//int r = rand() % 10;

				//if (r < 5)
				//{
					sprintf(form->LmkID[0], "17-2");
					opt.alt[0] = -1.97*1852.0;
					opt.lat[0] = 20.02*RAD;
					opt.lng[0] = 30.804*RAD;
				//}
				//else
				//{
					//sprintf(form->LmkID[0], "17-3");
					//opt.alt[0] = -1.89*1852.0;
					//opt.lat[0] = 20.272*RAD;
					//opt.lng[0] = 30.7*RAD;
				//}
			}
			else
			{
				sprintf(form->LmkID[0], "17-1");
				opt.alt[0] = -1.96*1852.0;
				opt.lat[0] = 20.16*RAD;
				opt.lng[0] = 30.809*RAD;
			}

			//Updated landing site vector after processing landmark tracking data
			//BZLAND.rad[RTCC_LMPOS_BEST] = 937.08*1852.0;
			//BZLAND.lat[RTCC_LMPOS_BEST] = 20.164*RAD;
			//BZLAND.lng[RTCC_LMPOS_BEST] = 30.750*RAD;
		}
		else if (fcn == 64) //LANDMARK TRACKING PAD (17-1)
		{
			opt.LmkTime[0] = calcParams.PDI;
			opt.entries = 1;
			sprintf(form->LmkID[0], "17-1");
			opt.alt[0] = -1.96*1852.0;
			opt.lat[0] = 20.16*RAD;
			opt.lng[0] = 30.809*RAD;
		}
		else if (fcn == 65) //LANDMARK TRACKING PAD (RP-3)
		{
			SV sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, 0.0, RTCC_ENGINETYPE_CSMSPS);

			opt.sv0 = sv1;
			opt.LmkTime[0] = TimeofIgnition + 25.0*60.0;
			opt.entries = 1;
			sprintf(form->LmkID[0], "RP-3");
			opt.alt[0] = 0.0;
			opt.lat[0] = -3.694*RAD;
			opt.lng[0] = 131.912*RAD;
		}
		else if (fcn == 66) //LANDMARK TRACKING PAD (F-1 & 17-1)
		{
			opt.entries = 2;

			sprintf(form->LmkID[0], "F-1");
			opt.LmkTime[0] = calcParams.LunarLiftoff - 2.0*3600.0 - 40.0*60.0;
			opt.alt[0] = 0.0;
			opt.lat[0] = 1.863*RAD;
			opt.lng[0] = 88.25*RAD;

			sprintf(form->LmkID[1], "17-1");
			opt.LmkTime[1] = calcParams.LunarLiftoff - 2.0*3600.0 - 30.0*60.0;
			opt.alt[1] = -1.96*1852.0;
			opt.lat[1] = 20.16*RAD;
			opt.lng[1] = 30.809*RAD;
		}

		LandmarkTrackingPAD(&opt, *form);
	}
	break;
	case 70: //PDI PAD
	{
		AP11PDIPAD * form = (AP11PDIPAD *)pad;

		PDIPADOpt opt;
		SV sv;

		sv = StateVectorCalc(calcParams.tgt);

		if (mcc->MoonRev >= 13)
		{
			opt.direct = true;
		}
		else
		{
			opt.direct = false;
			opt.dV_LVLH = calcParams.DVSTORE1;
			opt.P30TIG = calcParams.TIGSTORE1;
		}
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		opt.sv0 = sv;
		opt.t_land = CZTDTGTU.GETTD;
		opt.vessel = calcParams.tgt;

		PDI_PAD(&opt, *form);

		form->Att = _V(form->Att.x, form->Att.y, 290.0);
	}
	break;
	case 71: //PDI ABORT PAD
	{
		AP12PDIABORTPAD * form = (AP12PDIABORTPAD *)pad;

		SV sv1;
		double t_sunrise1, t_sunrise2, t_TPI;

		sv1 = StateVectorCalc(calcParams.src);

		t_sunrise1 = calcParams.PDI + 3.0*3600.0;
		t_sunrise2 = calcParams.PDI + 5.0*3600.0;

		if (mcc->MoonRev >= 13)
		{
			//PDI-2
			t_TPI = FindOrbitalSunrise(sv1, t_sunrise2) - 16.0*60.0;
			form->T_TPI_Pre10Min = round(t_TPI);
			t_TPI = FindOrbitalSunrise(sv1, t_sunrise1) - 16.0*60.0;
			form->T_TPI_Post10Min = round(t_TPI);
		}
		else
		{
			//PDI-1
			SV sv2 = ExecuteManeuver(sv1, TimeofIgnition, DeltaV_LVLH, 0, RTCC_ENGINETYPE_CSMSPS);
			t_TPI = FindOrbitalSunrise(sv2, t_sunrise1) - 16.0*60.0;
			form->T_TPI_Pre10Min = round(t_TPI);
			t_TPI = FindOrbitalSunrise(sv2, t_sunrise2) - 16.0*60.0;
			form->T_TPI_Post10Min = round(t_TPI);
		}
	}
	break;
	case 72: //No PDI+12 PAD
	{
		AP11LMManPADOpt opt;
		SV svlm0, svlm1, svcsm0, svcsm1;
		VECTOR3 dV_LVLH;
		double P30TIG, t_sunrise, t_CSI, t_TPI, DH, TIG, dvx1, dvx2;

		AP11LMMNV * form = (AP11LMMNV *)pad;

		svcsm0 = StateVectorCalc(calcParams.src);
		svlm0 = StateVectorCalc(calcParams.tgt);

		TIG = calcParams.PDI + 12.0*60.0;
		dvx1 = dvx2 = 103.0;

		svcsm1 = ExecuteManeuver(svcsm0, TimeofIgnition, DeltaV_LVLH, 0, RTCC_ENGINETYPE_CSMSPS);
		svlm1 = ExecuteManeuver(svlm0, calcParams.TIGSTORE1, calcParams.DVSTORE1, 0, RTCC_ENGINETYPE_LMRCSMINUS4);

		t_sunrise = calcParams.PDI + 3.0*3600.0;
	    t_TPI = FindOrbitalSunrise(svcsm1, t_sunrise) - 16.0*60.0;
	    //Round to next 30 seconds
	    t_TPI = round(t_TPI / 30.0)*30.0;

		SPQOpt coeopt;
		SPQResults coeres;

		coeopt.DH = 15.0*1852.0;
		coeopt.E = 26.6*RAD;
		coeopt.sv_P = svcsm1;
		coeopt.K_CDH = 0;
		coeopt.t_CSI = TIG + 55.0*60.0;
		coeopt.t_TPI = t_TPI;

		int n = 0, nmax = 100;

		//Find solution for no DOI-2 case
		coeopt.sv_A = ExecuteManeuver(svlm0, TIG, _V(dvx2, 0.0, -50.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
		ConcentricRendezvousProcessor(coeopt, coeres);
		DH = coeopt.DH - coeres.DH;
		while (abs(DH) > 0.1*1852.0 && n < nmax)
		{
			dvx2 += DH/ (abs(DH) * 10.0);

			coeopt.sv_A = ExecuteManeuver(svlm0, TIG, _V(dvx2, 0.0, -50.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
			ConcentricRendezvousProcessor(coeopt, coeres);
			DH = coeopt.DH - coeres.DH;

			n++;
		}

		n = 0;

		//Find nominal solution (with DOI-2)
		coeopt.sv_A = ExecuteManeuver(svlm1, TIG, _V(dvx1, 0.0, -50.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
		ConcentricRendezvousProcessor(coeopt, coeres);
		DH = coeopt.DH - coeres.DH;
		while (abs(DH) > 0.1*1852.0 && n < nmax)
		{
			dvx1 += DH / (abs(DH) * 10.0);

			coeopt.sv_A = ExecuteManeuver(svlm1, TIG, _V(dvx1, 0.0, -50.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
			ConcentricRendezvousProcessor(coeopt, coeres);
			DH = coeopt.DH - coeres.DH;

			n++;
		}

		PoweredFlightProcessor(svlm1, TIG, RTCC_ENGINETYPE_LMDPS, 0.0, _V(dvx1, 0.0, -50.0)*0.3048, true, P30TIG, dV_LVLH);
		t_CSI = coeres.t_CSI;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_LMDPS;
		opt.HeadsUp = false;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.TIG = P30TIG;
		opt.vessel = calcParams.tgt;
		opt.csmlmdocked = false;
		opt.useSV = true;
		opt.RV_MCC = svlm1;

		double AGSTIG = P30TIG - GETfromGMT(GetAGSClockZero());

		AP11LMManeuverPAD(&opt, *form);
		sprintf(form->purpose, "No PDI +12 ABORT");
		sprintf(form->remarks, "TIG 373: %+05.1f\nDVX NO DOI-2: %+07.1f", AGSTIG / 60.0, dvx2);
		form->type = 1;
		form->t_CSI = round(t_CSI);
		form->t_TPI = round(t_TPI);
	}
	break;
	case 73: //LUNAR SURFACE DATA CARD
	{
		AP12LunarSurfaceDataCard * form = (AP12LunarSurfaceDataCard*)pad;

		SV sv0, sv_CSM, sv_Ins, sv_IG;
		VECTOR3 R_LS;
		double T2, m0, v_LH, v_LV, t_sunrise, t_TPI, dv;

		sv0 = StateVectorCalc(calcParams.src);
		
		LEM *l = (LEM*)calcParams.tgt;
		m0 = l->GetAscentStageMass();

		v_LH = 5515.7*0.3048;
		v_LV = 19.5*0.3048;

		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		if (mcc->MoonRev >= 13)
		{
			sv_CSM = sv0;
			T2 = calcParams.PDI + 22.0*60.0 + 19.0;
			t_sunrise = calcParams.PDI + 5.0*3600.0;
		}
		else
		{
			sv_CSM = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, 0, RTCC_ENGINETYPE_CSMSPS);
			T2 = calcParams.PDI + 24.0*60.0 + 45.0;
			t_sunrise = calcParams.PDI + 7.0*3600.0;
		}

		t_TPI = FindOrbitalSunrise(sv_CSM, t_sunrise) - 16.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;

		form->T2_TIG = T2;
		form->T2_t_TPI = round(t_TPI);

		//T3
		LunarLiftoffTimeOpt opt;
		SV sv_CSM2, sv_CSM_over;
		double MJD_over, theta_1, dt_1, t_L, t_CSI;

		//Calculate TPI time for T3
		t_sunrise = calcParams.PDI + 5.0*3600.0;
		t_TPI = FindOrbitalSunrise(sv_CSM, t_sunrise) - 23.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;

		//Set all the options
		opt.I_BURN = 1;
		opt.I_TPI = 2;
		opt.I_CDH = 1;
		opt.I_SRCH = 0;
		opt.I_OS = 0;
		opt.M = 2;
		opt.P = 1;
		opt.L_DH = 3;
		opt.t_max = 9.0*3600.0;
		opt.H_S = 5.0*1852.0;
		opt.DV_MAX[0] = opt.DV_MAX[1] = 430.0*0.3048;
		opt.DH[0] = 10.0*1852.0;
		opt.DH[1] = 15.0*1852.0;
		opt.DH[2] = 20.0*1852.0;
		opt.v_LH = 5535.6*0.3048;
		opt.v_LV = 32.0*0.3048;
		opt.h_BO = 60000.0*0.3048;
		opt.Y_S = 0.5*RAD;
		opt.DT_B = 50.0*60.0;
		opt.theta_F = 130.0*RAD;
		opt.E = 26.6*RAD;
		opt.DH_OFF = 0.0;
		opt.dTheta_OFF = 0.0;
		opt.t_hole = GMTfromGET(t_TPI);
		opt.lat = BZLAND.lat[0];
		opt.lng = BZLAND.lng[0];
		opt.R_LLS = BZLAND.rad[0];
		opt.sv_CSM = sv_CSM;
		opt.theta_1 = 9.9588*RAD;
		opt.dt_1 = 447.0;

		//Initial pass through the processor
		LunarLaunchWindowProcessor(opt);
		//Refine ascent parameters
		LunarAscentProcessor(R_LS, m0, sv_CSM, PZLRPT.data[1].GETLO, opt.v_LH, opt.v_LV, theta_1, dt_1, dv, sv_IG, sv_Ins);
		opt.theta_1 = theta_1;
		opt.dt_1 = dt_1;
		//Final pass through
		LunarLaunchWindowProcessor(opt);
		t_L = PZLRPT.data[1].GETLO;
		t_CSI = PZLRPT.data[1].T_CSI;
		t_TPI = PZLRPT.data[1].T_TPI;

		sv_CSM2 = coast(sv_CSM, calcParams.PDI - OrbMech::GETfromMJD(sv_CSM.MJD, CalcGETBase()));
		MJD_over = OrbMech::P29TimeOfLongitude(sv_CSM2.R, sv_CSM2.V, sv_CSM2.MJD, sv_CSM2.gravref, BZLAND.lng[RTCC_LMPOS_BEST]);
		sv_CSM_over = coast(sv_CSM2, (MJD_over - sv_CSM2.MJD)*24.0*3600.0);

		form->T3_TIG = round(t_L);
	}
	break;
	case 74: //CMC RLS UPDATE
    case 75: //LGC RLS UPDATE
	{
		char buffer[100];

		if (fcn == 74)
		{
			LandingSiteUplink(buffer, RTCC_MPT_CSM);
		}
		else
		{
			LandingSiteUplink(buffer, RTCC_MPT_LM);
		}

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "RLS");
		}
	}
	break;
	case 76: //LGC TLAND UPDATE
	{
		char TLANDbuffer[64];

		TLANDUpdate(TLANDbuffer, CZTDTGTU.GETTD);

		sprintf(uplinkdata, "%s", TLANDbuffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Descent target load");
		}
	}
	break;
	case 77: //DOI EVALUATION
	{
		sprintf(upMessage, "Go for PDI");
	}
	break;
	case 78: //PDI EVALUATION
	{
		SV sv;
		double apo, peri;

		sv = StateVectorCalc(calcParams.tgt);

		OrbMech::periapo(sv.R, sv.V, OrbMech::mu_Moon, apo, peri);

		if (peri - OrbMech::R_Moon > 0.)
		{
			sprintf(upMessage, "Recycle to next PDI attempt");
			scrubbed = true;
		}
	}
	break;
	case 79: //LANDING EVALUATION
	{
		sprintf(upMessage, "Okay, Houston. The Challenger has landed!");
	}
	break;
	case 80: //STAY FOR T1
	{
		sprintf(upMessage, "Stay for T1");
	}
	break;
	case 81: //STAY FOR T2
	{
		sprintf(upMessage, "Stay for T2");
	}
	break;
	case 82: //STAY FOR T3
	{
		sprintf(upMessage, "Stay for T3. IMU parking angles: +295.86, +000.00, +084.14");

		//Update landing site
		calcParams.tgt->GetEquPos(BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
	}
	break;
	case 85: //LIFTOFF TIME UPDATE REV 15 THRU 20
	case 86: //LIFTOFF TIME UPDATE REV 21 THRU 25
	case 87: //LIFTOFF TIME UPDATE REV 26 THRU 32
	case 88: //LIFTOFF TIME UPDATE REV 33 THRU 37
	case 89: //LIFTOFF TIME UPDATE REV 38 THRU 43
	case 90: //LIFTOFF TIME UPDATE REV 44 THRU 50
	{
		LIFTOFFTIMES * form = (LIFTOFFTIMES*)pad;

		LunarLiftoffTimeOpt opt;
		SV sv_CSM, sv_CSM_upl, sv_Ins, sv_IG;
		VECTOR3 R_LS;
		double m0, theta_1, dt_1, dv, t_TPI_guess, GET_CSM;

		sv_CSM = StateVectorCalc(calcParams.src);

		LEM *l = (LEM*)calcParams.tgt;
		m0 = l->GetAscentStageMass();

		GET_CSM = OrbMech::GETfromMJD(sv_CSM.MJD, CalcGETBase());

		//Set all the options
		opt.I_BURN = 1;
		opt.I_TPI = 2;
		opt.I_CDH = 1;
		opt.I_SRCH = 0;
		opt.I_OS = 0;
		opt.M = 2;
		opt.P = 1;
		opt.L_DH = 3;
		opt.t_max = 9.0*3600.0;
		opt.H_S = 5.0*1852.0;
		opt.DV_MAX[0] = opt.DV_MAX[1] = 430.0*0.3048;
		opt.DH[0] = 10.0*1852.0;
		opt.DH[1] = 15.0*1852.0;
		opt.DH[2] = 20.0*1852.0;
		opt.v_LH = 5535.6*0.3048; //5509.5?
		opt.v_LV = 32.0*0.3048; //19.5?
		opt.h_BO = 60000.0*0.3048;
		opt.Y_S = 0.5*RAD;
		opt.DT_B = 50.0*60.0;
		opt.theta_F = 130.0*RAD;
		opt.E = 26.6*RAD;
		opt.DH_OFF = 0.0;
		opt.dTheta_OFF = 0.0;
		opt.lat = BZLAND.lat[0];
		opt.lng = BZLAND.lng[0];
		opt.R_LLS = BZLAND.rad[0];
		opt.sv_CSM = sv_CSM;
		opt.theta_1 = 9.9588*RAD;
		opt.dt_1 = 447.0;

		if (fcn == 85) //LIFTOFF TIME UPDATE REVS 15 THRU 20
		{
			t_TPI_guess = GET_CSM + 6.0*3600.0;
			form->entries = 6;
			form->startdigit = mcc->MoonRev + 2; //REV 15
		}
		else if (fcn == 86) //LIFTOFF TIME UPDATE REV 21 THRU 25
		{
			t_TPI_guess = GET_CSM + 5.5*3600.0;
			form->entries = 5;
			form->startdigit = mcc->MoonRev + 1; //REV 21
		}
		else if (fcn == 87) //LIFTOFF TIME UPDATE REV 26 THRU 32
		{
			t_TPI_guess = GET_CSM + 5.0*3600.0;
			form->entries = 7;
			form->startdigit = mcc->MoonRev + 1; //REV 26
		}
		else if (fcn == 88) //LIFTOFF TIME UPDATE REV 33 THRU 37
		{
			t_TPI_guess = GET_CSM + 6.5*3600.0;
			form->entries = 5;
			form->startdigit = mcc->MoonRev + 2; //REV 33
		}
		else if (fcn == 89) //LIFTOFF TIME UPDATE REV 38 THRU 43
		{
			t_TPI_guess = GET_CSM + 5.0*3600.0;
			form->entries = 6;
			form->startdigit = mcc->MoonRev + 1; //REV 38
		}
		else //LIFTOFF TIME UPDATE REV 44 THRU 50
		{
			t_TPI_guess = GET_CSM + 5.0*3600.0;
			form->entries = 7;
			form->startdigit = mcc->MoonRev + 1; //REV 44
		}

		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		double t_TPI;
		t_TPI = FindOrbitalSunrise(sv_CSM, t_TPI_guess) - 16.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;
		opt.t_hole = GMTfromGET(t_TPI);

		//Initial pass through the processor
		LunarLaunchWindowProcessor(opt);
		//Refine ascent parameters
		LunarAscentProcessor(R_LS, m0, sv_CSM, PZLRPT.data[1].GETLO, opt.v_LH, opt.v_LV, theta_1, dt_1, dv, sv_IG, sv_Ins);
		opt.theta_1 = theta_1;
		opt.dt_1 = dt_1;

		for (int i = 0;i < form->entries;i++)
		{
			LunarLaunchWindowProcessor(opt);
			form->TIG[i] = PZLRPT.data[1].GETLO;

			t_TPI_guess += 2.0*3600.0;
			t_TPI = FindOrbitalSunrise(sv_CSM, t_TPI_guess) - 16.0*60.0;
			//Round to next 30 seconds
			t_TPI = round(t_TPI / 30.0)*30.0;
			opt.t_hole = GMTfromGET(t_TPI);
		}
	}
	break;
	case 94: //PLANE CHANGE CALCULATION
	case 95: //PLANE CHANGE UPDATE
	{
		SV sv;

		sv = StateVectorCalc(calcParams.src);

		med_k16.GETTH1 = calcParams.LunarLiftoff - 6.0*3600.0;
		med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.LunarLiftoff - 0.5*3600.0;
		med_k16.Mode = 7;
		med_k16.Sequence = 1;
		med_k16.Vehicle = RTCC_MPT_CSM;

		GZGENCSN.LDPPAzimuth = 0.0;

		LunarDescentPlanningProcessor(sv);

		PoweredFlightProcessor(sv, PZLDPDIS.GETIG[0], RTCC_ENGINETYPE_CSMSPS, 0.0, PZLDPDIS.DVVector[0] * 0.3048, true, TimeofIgnition, DeltaV_LVLH);

		if (fcn == 95)
		{
			AP11MNV * form = (AP11MNV*)pad;

			AP11ManPADOpt manopt;

			REFSMMATOpt refsopt;
			MATRIX3 REFSMMAT;

			char buffer1[1000], buffer2[1000], buffer3[1000], updesc[128];

			refsopt.dV_LVLH = DeltaV_LVLH;
			refsopt.HeadsUp = true;
			refsopt.REFSMMATTime = TimeofIgnition;
			refsopt.REFSMMATopt = 0;
			refsopt.vessel = calcParams.src;
			refsopt.vesseltype = 0;

			REFSMMAT = REFSMMATCalc(&refsopt);

			manopt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
			manopt.dV_LVLH = DeltaV_LVLH;
			manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
			manopt.HeadsUp = true;
			manopt.REFSMMAT = REFSMMAT;
			manopt.TIG = TimeofIgnition;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 0;

			AP11ManeuverPAD(&manopt, *form);

			sprintf(form->purpose, "LOPC");
			sprintf(form->remarks, "Four-jet ullage for 12 seconds");
			sprintf(updesc, "CSM state vector, target load, PC REFSMMAT");
			AGCStateVectorUpdate(buffer1, sv, true);

			CMCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);
			AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

			sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);

			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, updesc);
			}
		}
	}
	break;
	case 96: //CMC LUNAR LIFTOFF REFSMMAT UPLINK
	{
		REFSMMATOpt opt;
		MATRIX3 REFSMMAT;
		char buffer[1000];

		opt.LSLat = BZLAND.lat[RTCC_LMPOS_BEST];
		opt.LSLng = BZLAND.lng[RTCC_LMPOS_BEST];
		opt.REFSMMATopt = 5;
		opt.REFSMMATTime = calcParams.LunarLiftoff;
		opt.vessel = calcParams.src;

		REFSMMAT = REFSMMATCalc(&opt);
		AGCDesiredREFSMMATUpdate(buffer, REFSMMAT);

		//Store as LM LLD
		EMGSTSTM(RTCC_MPT_LM, REFSMMAT, RTCC_REFSMMAT_TYPE_LLD, RTCCPresentTimeGMT());
		//Also move to CSM LCV
		GMGMED("G00,LEM,LLD,CSM,LCV;");

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Liftoff REFSMMAT");
		}
	}
	break;
	case 98: //P57 Liftoff time
	{
		GENERICPAD * form = (GENERICPAD *)pad;

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(calcParams.LunarLiftoff, hh, mm, ss);

		sprintf(form->paddata, "P57 Liftoff time: %03d:%02d:%0.2lf", hh, mm, ss);
	}
	break;
	case 99: //LIFTOFF TIME INITIAL CALCULATION
	{
		SV sv;
		int CurRev, NomAscentRev;
		double CurRevTime, t_P, DT_toascent, ASC_GET;

		sv = StateVectorCalc(calcParams.src);

		CurRev = mcc->MoonRev;
		CurRevTime = mcc->MoonRevTime;
		NomAscentRev = 51;
		t_P = OrbMech::period(sv.R, sv.V, OrbMech::mu_Moon);
		DT_toascent = (t_P * (NomAscentRev - CurRev)) - CurRevTime + 45.0*60.0; //Nominal ascent is ~45 minutes into REV 51
		ASC_GET = OrbMech::GETfromMJD(sv.MJD, CalcGETBase()) + DT_toascent;

		if (mcc->MoonRev < 51)
		{
			calcParams.LunarLiftoff = ASC_GET;
		}
		else
		{
			calcParams.LunarLiftoff = ASC_GET + (CurRev + 1 - NomAscentRev)*2.0*3600.0;
		}
	}
	break;
	case 100: //NOMINAL INSERTION TARGET (DIRECT ASCENT RENDEZVOUS)
	{
		LLTPOpt opt;
		SV sv_CSM, sv_LM_upl, sv_Ins, sv_IG;
		EphemerisData sv_CSM_e;
		VECTOR3 R_LS;
		double m0, theta_1, dt_1, dv, t_TPI_guess, t_TPI;

		sv_CSM = StateVectorCalc(calcParams.src);
		sv_CSM_e = StateVectorCalcEphem(calcParams.src);

		LEM *l = (LEM*)calcParams.tgt;
		m0 = l->GetAscentStageMass();

		t_TPI_guess = calcParams.LunarLiftoff + 55.0*60.0;
		t_TPI = FindOrbitalSunrise(sv_CSM, t_TPI_guess) - 16.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;

		//Set all the options
		opt.sv_CSM = sv_CSM_e;
		opt.Y_S = PZLTRT.YawSteerCap;
		opt.V_Z_NOM = 32.0*0.3048;
		opt.T_TH = GMTfromGET(calcParams.LunarLiftoff - 30.0*60.0);
		opt.R_LS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.lat_LS = BZLAND.lat[RTCC_LMPOS_BEST];
		opt.lng_LS = BZLAND.lng[RTCC_LMPOS_BEST];
		opt.alpha_PF = PZLTRT.PoweredFlightArc;
		opt.dt_PF = PZLTRT.PoweredFlightTime;
		opt.dt_INS_TPI = PZLTRT.DT_Ins_TPI;
		opt.h_INS = PZLTRT.InsertionHeight;
		opt.DH_TPI = PZLTRT.DT_DH;
		opt.dTheta_TPI = PZLTRT.DT_Theta_i;
		opt.WT = PZLTRT.TerminalPhaseTravelAngle;

		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		//Initial direct ascent calculation
		LunarLiftoffTimePredictionDT(opt, PZLLTT);
		LunarAscentProcessor(R_LS, m0, sv_CSM, PZLLTT.GETLOR, PZLLTT.VH, opt.V_Z_NOM, theta_1, dt_1, dv, sv_IG, sv_Ins);
		opt.alpha_PF = theta_1;
		opt.dt_PF = dt_1;
		LunarLiftoffTimePredictionDT(opt, PZLLTT);

		//Iterate DT to find correct TPI time
		int n = 0, nmax = 100;
		double TPI_diff = t_TPI - PZLLTT.GET_TPI;
		while (abs(TPI_diff) > 20.0 && n < nmax)
		{
			opt.dt_INS_TPI += ((TPI_diff / abs(TPI_diff * 10.0)) * 1852.0);
			LunarLiftoffTimePredictionDT(opt, PZLLTT);
			TPI_diff = t_TPI - PZLLTT.GET_TPI;
			n++;
		}

		LunarAscentProcessor(R_LS, m0, sv_CSM, PZLLTT.GETLOR, PZLLTT.VH, opt.V_Z_NOM, theta_1, dt_1, dv, sv_IG, sv_Ins);

		calcParams.LunarLiftoff = PZLLTT.GETLOR;
		calcParams.Insertion = OrbMech::GETfromMJD(sv_Ins.MJD, CalcGETBase());
		calcParams.TPI = PZLLTT.GET_TPI;

		//Store for Ascent PAD
		TimeofIgnition = calcParams.LunarLiftoff;
		DeltaV_LVLH = _V(PZLLTT.VH, opt.V_Z_NOM, 0);
		calcParams.TIGSTORE1 = calcParams.TPI;

		calcParams.SVSTORE1 = sv_Ins;
	}
	break;
	case 101: //COELLIPTIC RENDEZVOUS TARGET
	{
		LunarLiftoffTimeOpt opt;
		SV sv_CSM, sv_LM_upl, sv_Ins, sv_IG;
		VECTOR3 R_LS;
		double m0, theta_1, dt_1, dv, t_TPI_guess, t_TPI;

		sv_CSM = StateVectorCalc(calcParams.src);

		LEM *l = (LEM*)calcParams.tgt;
		m0 = l->GetAscentStageMass();

		//One rev after direct ascent TPI
		t_TPI_guess = calcParams.TPI + 2.0*3600.0;
		t_TPI = FindOrbitalSunrise(sv_CSM, t_TPI_guess) - 16.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;

		//Set all the options
		opt.I_BURN = 1;
		opt.I_TPI = 2;
		opt.I_CDH = 1;
		opt.I_SRCH = 0;
		opt.I_OS = 0;
		opt.M = 2;
		opt.P = 1;
		opt.L_DH = 3;
		opt.t_max = 9.0*3600.0;
		opt.H_S = 5.0*1852.0;
		opt.DV_MAX[0] = opt.DV_MAX[1] = 430.0*0.3048;
		opt.DH[0] = 10.0*1852.0;
		opt.DH[1] = 15.0*1852.0;
		opt.DH[2] = 20.0*1852.0;
		opt.v_LH = 5539.0*0.3048;
		opt.v_LV = 38.0*0.3048;
		opt.h_BO = 60000.0*0.3048;
		opt.Y_S = 0.5*RAD;
		opt.DT_B = 50.0*60.0;
		opt.theta_F = 130.0*RAD;
		opt.E = 26.6*RAD;
		opt.DH_OFF = 0.0;
		opt.dTheta_OFF = 0.0;
		opt.lat = BZLAND.lat[0];
		opt.lng = BZLAND.lng[0];
		opt.R_LLS = BZLAND.rad[0];
		opt.sv_CSM = sv_CSM;
		opt.theta_1 = 9.9588*RAD;
		opt.dt_1 = 447.0;
		opt.t_hole = GMTfromGET(t_TPI);

		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		//Initial pass through the processor
		LunarLaunchWindowProcessor(opt);
		//Refine ascent parameters
		LunarAscentProcessor(R_LS, m0, sv_CSM, PZLRPT.data[1].GETLO, opt.v_LH, opt.v_LV, theta_1, dt_1, dv, sv_IG, sv_Ins);
		opt.theta_1 = theta_1;
		opt.dt_1 = dt_1;
		//Final pass through
		LunarLaunchWindowProcessor(opt);

		calcParams.CSI = PZLRPT.data[1].T_CSI;
		calcParams.CDH = PZLRPT.data[1].T_CDH;

		//Store for Ascent PAD
		TimeofIgnition = PZLRPT.data[1].GETLO;
		DeltaV_LVLH = _V(opt.v_LH, opt.v_LV, 0);
		calcParams.TIGSTORE1 = PZLRPT.data[1].T_TPI;

		//Store for CSI PAD
		calcParams.DVSTORE1 = _V(PZLRPT.data[1].DVCSI*0.3048, 0, 0);
	}
	break;
	case 102: //CMC INSERTION STATE VECTORS
	case 103: //LGC CSM STATE VECTORS + RLS
	{
		SV sv_LM_upl, sv_CSM, sv_CSM_upl;
		char buffer1[100], buffer2[100];

		sv_CSM = StateVectorCalc(calcParams.src);

		sv_CSM_upl = coast(sv_CSM, calcParams.LunarLiftoff - OrbMech::GETfromMJD(sv_CSM.MJD, CalcGETBase()));
		sv_LM_upl = coast(calcParams.SVSTORE1, calcParams.Insertion + 5.0*60.0 - OrbMech::GETfromMJD(calcParams.SVSTORE1.MJD, CalcGETBase()));

		AGCStateVectorUpdate(buffer1, sv_CSM_upl, true);
		AGCStateVectorUpdate(buffer2, sv_LM_upl, false);

		if (fcn == 102)
		{
			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM and LM state vectors");
			}
		}
		else
		{
			char buffer3[100];
			LandingSiteUplink(buffer3, RTCC_MPT_LM);

			sprintf(uplinkdata, "%s%s", buffer1, buffer3);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector, RLS");
			}
		}
	}
	break;
	case 104: //NOMINAL LM ASCENT PAD
	{
		AP12LMASCPAD * form = (AP12LMASCPAD*)pad;

		SV sv_CSM;
		MATRIX3 Rot, Rot2, Rot_VL;
		VECTOR3 R_LS;
		double m0, m1, TIG, v_LH, v_LV;

		//Definitions
		TIG = TimeofIgnition;
		v_LH = DeltaV_LVLH.x;
		v_LV = DeltaV_LVLH.y;

		sv_CSM = StateVectorCalc(calcParams.src);
		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		LEM *l = (LEM*)calcParams.tgt;
		m0 = l->GetAscentStageMass();
		m1 = calcParams.src->GetMass();

		calcParams.tgt->GetRotationMatrix(Rot);
		oapiGetRotationMatrix(sv_CSM.gravref, &Rot2);

		Rot_VL = OrbMech::GetVesselToLocalRotMatrix(Rot, Rot2);

		// Calculate ascent PAD
		SV sv_Ins, sv_CSM_TIG;
		MATRIX3 Rot_LG, Rot_VG;
		VECTOR3 R_LSP, Q, U_R, Z_B, X_AGS, Y_AGS, Z_AGS;
		double CR, MJD_TIG, SMa, R_D, delta_L, sin_DL, cos_DL, AGSTPI, apo, peri;

		AGSTPI = calcParams.TIGSTORE1 - GETfromGMT(GetAGSClockZero());

		OrbMech::periapo(sv_CSM.R, sv_CSM.V, OrbMech::mu_Moon, apo, peri);

		MJD_TIG = OrbMech::MJDfromGET(TIG, CalcGETBase());
		sv_CSM_TIG = coast(sv_CSM, TIG - OrbMech::GETfromMJD(sv_CSM.MJD, CalcGETBase()));

		ELVCNV(R_LS, GMTfromGET(TIG), 1, 3, 2, R_LSP);

		Q = unit(crossp(sv_CSM_TIG.V, sv_CSM_TIG.R));
		U_R = unit(R_LSP);
		R_D = length(R_LSP) + 60000.0*0.3048;
		CR = -R_D * asin(dotp(U_R, Q));

		sv_Ins.R = _V(R_D, 0, 0);
		sv_Ins.V = _V(v_LV, v_LH, 0);
		sv_Ins.gravref = sv_CSM_TIG.gravref;
		SMa = GetSemiMajorAxis(sv_Ins);

		Rot_LG = OrbMech::GetRotationMatrix(BODY_MOON, MJD_TIG);
		Rot_VG = OrbMech::GetVesselToGlobalRotMatrix(Rot_VL, Rot_LG);
		Z_B = mul(Rot_VG, _V(0, 0, 1));
		Z_B = unit(_V(Z_B.x, Z_B.z, Z_B.y));
		X_AGS = U_R;
		Z_AGS = unit(crossp(-Q, X_AGS));
		Y_AGS = unit(crossp(Z_AGS, X_AGS));

		delta_L = atan2(dotp(Z_B, Z_AGS), dotp(Z_B, Y_AGS));
		sin_DL = sin(delta_L);
		cos_DL = cos(delta_L);

		form->CR = CR / 1852.0;
		form->TIG = TIG;
		form->V_hor = v_LH / 0.3048;
		form->V_vert = v_LV / 0.3048;
		form->DEDA047 = OrbMech::DoubleToDEDA(sin_DL, 14);
		form->DEDA053 = OrbMech::DoubleToDEDA(cos_DL, 14);
		form->DEDA225_226 = SMa / 0.3048 / 100.0;
		form->DEDA231 = length(R_LS) / 0.3048 / 100.0;
		form->DEDA465 = v_LV / 0.3048;
		form->TIG_2 = calcParams.TIGSTORE1;
		form->LMWeight = m0 / 0.45359237;
		form->CSMWeight = m1 / 0.45359237;
		sprintf(form->remarks, "\n373 TPI: %+05.1f\nCSM HA: %+05.1f\nCSM HP: %+05.1f", AGSTPI / 60.0, (apo - OrbMech::R_Moon) / 1852.0, (peri - OrbMech::R_Moon) / 1852.0);
	}
	break;
	case 106: //NOMINAL CSI PAD
	{
		AP10CSI * form = (AP10CSI*)pad;

		AP10CSIPADOpt opt;

		opt.dV_LVLH = calcParams.DVSTORE1;
		opt.enginetype = RTCC_ENGINETYPE_LMRCSPLUS4;
		opt.KFactor = GETfromGMT(GetAGSClockZero());
		opt.REFSMMAT = EZJGMTX3.data[RTCC_REFSMMAT_TYPE_LLD - 1].REFSMMAT;
		opt.sv0 = calcParams.SVSTORE1;
		opt.t_CSI = calcParams.CSI;
		opt.t_TPI = calcParams.TIGSTORE1;

		AP10CSIPAD(&opt, *form);
		form->type = 1;
	}
	break;
	case 107: //LM LIFTOFF EVALUATION
	{
		if (calcParams.tgt->GroundContact())
		{
			sprintf(upMessage, "Launch scrubbed, recycle for next rev");
			scrubbed = true;
		}
	}
	break;
	case 108: //ASCENT DETERMINATON
	{
		if (calcParams.tgt->GroundContact())
		{
			sprintf(upMessage, "Nominal liftoff TIG passed. Fly coelliptic ascent");
			calcParams.LunarLiftoff = TimeofIgnition;
			calcParams.Insertion = calcParams.LunarLiftoff + 7.0*60.0;
			calcParams.TPI = calcParams.TIGSTORE1;
			scrubbed = true;
		}
	}
	break;
	case 109: //TWEAK BURN
	{
		LambertMan opt;
		TwoImpulseResuls res;
		SV sv_LM, sv_CSM;

		sv_CSM = StateVectorCalc(calcParams.src);
		sv_LM = StateVectorCalc(calcParams.tgt);

		opt.mode = 2;
		opt.axis = RTCC_LAMBERT_MULTIAXIS;
		opt.DH = 15.0*1852.0;
		opt.ElevationAngle = 26.6*RAD;
		opt.N = 0;
		opt.Perturbation = RTCC_LAMBERT_PERTURBED;
		opt.PhaseAngle = 1.7*RAD;
		opt.sv_A = sv_LM;
		opt.sv_P = sv_CSM;
		opt.T1 = calcParams.Insertion + 3.0*60.0;
		opt.T2 = calcParams.TPI;

		LambertTargeting(&opt, res);

		if (length(res.dV_LVLH) > 5.0 * 0.3048)
		{
			sprintf(upMessage, "Tweak Burn: VX %+2.1f VY %+2.1f VZ %+2.1f", res.dV_LVLH.x / 0.3048, res.dV_LVLH.y / 0.3048, res.dV_LVLH.z / 0.3048);
		}
		else
		{
			sprintf(upMessage, "Tweak Burn scrubbed");
		}
	}
	break;
	case 110: //LM JETT PAD + CMC SV UPLINK
	{
		GENERICPAD * form = (GENERICPAD *)pad;

		SV sv0, sv_upl;
		char buffer[100];

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(calcParams.SEP, hh, mm, ss);
		sprintf(form->paddata, "Jettison PAD\nGET %d:%d:%.2lf\nR 020\nP 007\nY 349", hh, mm, ss);

		sv0 = StateVectorCalc(calcParams.src);
		sv_upl = coast(sv0, (calcParams.SEP - 10.0*60.0) - OrbMech::GETfromMJD(sv0.MJD, CalcGETBase()));

		AGCStateVectorUpdate(buffer, sv_upl, true);

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector (CSM SEP-10)");
		}
	}
	break;
	case 111: //LM IMPACT BURN PAD + UPLINK
	{
		AP11LMManPADOpt opt;

		SV sv0, sv1, sv_upl;
		double t_lng, ttoLNG;
		char buffer1[1000];
		char buffer2[1000];

		AP11LMMNV * form = (AP11LMMNV *)pad;

		sv0 = StateVectorCalc(calcParams.tgt);

		sv1 = coast(sv0, 1.5*3600.0);

		//Find time of SEP
		t_lng = OrbMech::P29TimeOfLongitude(sv1.R, sv1.V, sv1.MJD, GetGravref(BODY_MOON), 35.3*RAD);
		ttoLNG = (t_lng - sv1.MJD)*24.0 * 3600.0;
		calcParams.SEP = (sv1.MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;

		sv1 = coast(sv0, 3.0*3600.0);

		//Find impact burn TIG
		t_lng = OrbMech::P29TimeOfLongitude(sv1.R, sv1.V, sv1.MJD, GetGravref(BODY_MOON), 87.1*RAD);
		ttoLNG = (t_lng - sv1.MJD)*24.0 * 3600.0;
		TimeofIgnition = (sv1.MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;

		DeltaV_LVLH = _V(-219.2, 56.0, 168.0)*0.3048;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.csmlmdocked = false;
		opt.dV_LVLH = DeltaV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_LMRCSPLUS4;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.TIG = TimeofIgnition;
		opt.vessel = calcParams.tgt;

		AP11LMManeuverPAD(&opt, *form);
		sprintf(form->purpose, "IMPACT PAD");
		form->Att = _V(049.0, 139.0, 075.0);

		//Save cutoff time for MCC
		calcParams.TIGSTORE1 = TimeofIgnition + form->burntime;

		sv_upl = coast(sv0, (TimeofIgnition - 10.0*60.0) - OrbMech::GETfromMJD(sv0.MJD, CalcGETBase()));

		AGCStateVectorUpdate(buffer1, sv_upl, false);
		LGCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "LM state vector, Impact burn target load");
		}
	}
	break;
	case 112: //CSM P76 PAD
	{
		AP11P76PAD * form = (AP11P76PAD*)pad;

		form->entries = 1;
		sprintf(form->purpose[0], "CSM P76");
		form->TIG[0] = TimeofIgnition;
		form->DV[0] = DeltaV_LVLH / 0.3048;
	}
	break;
	case 120: //P99 LM deorbit uplink
	{
		char buffer1[1000], buffer2[1000], buffer3[1000], buffer4[1000];
		int emem1[20], emem2[10], emem3[15], emem4[13];

		emem1[0] = 24;
		emem1[1] = 3404;
		emem1[2] = 1450;
		emem1[3] = 12324;
		emem1[4] = 5520;
		emem1[5] = 161;
		emem1[6] = 1400;
		emem1[7] = 12150;
		emem1[8] = 5656;
		emem1[9] = 3667;
		emem1[10] = 74066;
		emem1[11] = 12404;
		emem1[12] = 12433;
		emem1[13] = 1406;
		emem1[14] = 5313;
		emem1[15] = 143;
		emem1[16] = 36266;
		emem1[17] = 54333;
		emem1[18] = 6060;
		emem1[19] = 77634;

		V7XUpdate(71, buffer1, emem1, 20);

		emem2[0] = 12;
		emem2[1] = 3734;
		emem2[2] = 26;
		emem2[3] = 30605;
		emem2[4] = 151;
		emem2[5] = 5214;
		emem2[6] = 0;
		emem2[7] = 0;
		emem2[8] = 15400;
		emem2[9] = 0;

		V7XUpdate(71, buffer2, emem2, 10);

		emem3[0] = 17;
		emem3[1] = 3400;
		emem3[2] = 5520;
		emem3[3] = 3401;
		emem3[4] = 312;
		emem3[5] = 3402;
		emem3[6] = 5263;
		emem3[7] = 3426;
		emem3[8] = 10636;
		emem3[9] = 3427;
		emem3[10] = 56246;
		emem3[11] = 3430;
		emem3[12] = 77650;
		emem3[13] = 3431;
		emem3[14] = 75202;

		V7XUpdate(72, buffer3, emem3, 15);

		emem4[0] = 15;
		emem4[1] = 3455;
		emem4[2] = 1404;
		emem4[3] = 1250;
		emem4[4] = 0;
		emem4[5] = 3515;
		emem4[6] = 4;
		emem4[7] = 2371;
		emem4[8] = 13001;
		emem4[9] = 2372;
		emem4[10] = 1420;
		emem4[11] = 2373;
		emem4[12] = 12067;

		V7XUpdate(72, buffer4, emem4, 13);

		sprintf(uplinkdata, "%s%s%s%s", buffer1, buffer2, buffer3, buffer4);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "P99 LM deorbit");
		}
	}
	break;
	case 121: //GROUND COMMAND: V96
	case 122: //GROUND COMMAND: V30
	case 123: //GROUND COMMAND: V33
	case 124: //GROUND COMMAND: ENT
	case 125: //GROUND COMMAND: P00
	case 126: //GROUND COMMAND: V5N26
	{
		char updesc[128];

		if (fcn == 121)
		{
			sprintf(uplinkdata, "V96ER");
			sprintf(updesc, "");
		}
		else if (fcn == 122)
		{
			sprintf(uplinkdata, "V30ER");
			sprintf(updesc, "P99 initiated");
		}
		else if (fcn == 123)
		{
			sprintf(uplinkdata, "V33ER");
			sprintf(updesc, "");
		}
		else if (fcn == 124)
		{
			sprintf(uplinkdata, "ER");
			sprintf(updesc, "");
		}
		else if (fcn == 125)
		{
			sprintf(uplinkdata, "00ER");
			sprintf(updesc, "P99 completed");
		}
		else if (fcn == 126)
		{
			sprintf(uplinkdata, "V5N26ER");
			sprintf(updesc, "");
		}

		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, updesc);
		}
	}
	break;
	case 128: //REV 1 MAP UPDATE
	case 129: //REV 3 MAP UPDATE
	case 130: //REV 76 MAP UPDATE
	{
		SV2 sv0, sv1, sv2;
		AP10MAPUPDATE upd_hyper, upd_ellip;

		AP10MAPUPDATE * form = (AP10MAPUPDATE *)pad;

		sv0 = StateVectorCalc2(calcParams.src);
		LunarOrbitMapUpdate(sv0.sv, upd_hyper);

		sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, RTCC_ENGINETYPE_CSMSPS);
		sv2.sv = coast(sv1.sv, -30.0*60.0);
		LunarOrbitMapUpdate(sv2.sv, upd_ellip);

		if (fcn == 128)
		{
			form->Rev = 1;
		}
		else if (fcn == 129)
		{
			form->Rev = 3;
		}
		else
		{
			form->Rev = mcc->MoonRev + 1;
		}

		form->type = 2;
		form->AOSGET = upd_hyper.AOSGET;
		form->LOSGET = upd_hyper.LOSGET;
		form->PMGET = upd_ellip.AOSGET;
	}
	break;
	case 132: //TEI REFSMMAT UPLINK
	{
		MATRIX3 REFSMMAT;

		char buffer1[1000];

		REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT;

		AGCDesiredREFSMMATUpdate(buffer1, REFSMMAT, true);
		sprintf(uplinkdata, "%s", buffer1);

		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "TEI REFSMMAT");
		}
	}
	break;
	case 133: //PDI-2 calculation
	{
		//Recalculate PDI and landing times
		SV sv;
		double GET_SV;

		AP11AGSACT *form = (AP11AGSACT*)pad;

		sv = StateVectorCalc(calcParams.tgt);

		GET_SV = OrbMech::GETfromMJD(sv.MJD, CalcGETBase());

		//MED K17
		GZGENCSN.LDPPPoweredDescentSimFlag = false;
		GZGENCSN.LDPPDwellOrbits = 0;
		//MED K16
		med_k16.Mode = 4;
		med_k16.Sequence = 1;
		med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = GET_SV + 0.5*3600.0;

		LunarDescentPlanningProcessor(sv);

		calcParams.PDI = PZLDPDIS.PD_GETIG;
		CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;
	}
	break;
	case 134: //No PDI2+12 PAD
	{
		AP11LMManPADOpt opt;
		SV svlm, svcsm, svman, svboost;
		VECTOR3 dV_LVLH;
		double P30TIG, t_sunrise, t_CSI, t_TPI, DH, TIG, dvx;

		AP11LMMNV * form = (AP11LMMNV *)pad;

		svcsm = StateVectorCalc(calcParams.src);
		svlm = StateVectorCalc(calcParams.tgt);

		TIG = calcParams.PDI + 12.0*60.0;
		dvx = 116.0;

		t_sunrise = calcParams.PDI + 5.0*3600.0;
		t_TPI = FindOrbitalSunrise(svcsm, t_sunrise) - 16.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;

		svman = ExecuteManeuver(svlm, TIG, _V(dvx, 0.0, -50.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
		svboost = ExecuteManeuver(svman, TIG + 1.0*3600.0, _V(10.0, 0.0, 0.0)*0.3048, 0, RTCC_ENGINETYPE_LMRCSPLUS4);

		SPQOpt coeopt;
		SPQResults coeres;

		coeopt.DH = 15.0*1852.0;
		coeopt.E = 26.6*RAD;
		coeopt.sv_P = svcsm;
		coeopt.K_CDH = 0;
		coeopt.t_CSI = TIG + 3.0*3600.0;
		coeopt.t_TPI = t_TPI;

		int n = 0, nmax = 100;

		//Find solution
		coeopt.sv_A = svboost;
		ConcentricRendezvousProcessor(coeopt, coeres);
		DH = coeopt.DH - coeres.DH;
		while (abs(DH) > 0.1*1852.0 && n < nmax)
		{
			dvx += DH / (abs(DH) * 10.0);

			svman = ExecuteManeuver(svlm, TIG, _V(dvx, 0.0, -50.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
			svboost = ExecuteManeuver(svman, TIG + 1.0*3600.0, _V(10.0, 0.0, 0.0)*0.3048, 0, RTCC_ENGINETYPE_LMRCSPLUS4);

			coeopt.sv_A = svboost;
			ConcentricRendezvousProcessor(coeopt, coeres);
			DH = coeopt.DH - coeres.DH;

			n++;
		}

		PoweredFlightProcessor(svlm, TIG, RTCC_ENGINETYPE_LMDPS, 0.0, _V(dvx, 0.0, -50.0)*0.3048, true, P30TIG, dV_LVLH);
		t_CSI = coeres.t_CSI;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_LMDPS;
		opt.HeadsUp = false;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.TIG = P30TIG;
		opt.vessel = calcParams.tgt;
		opt.csmlmdocked = false;

		double AGSTIG = P30TIG - GETfromGMT(GetAGSClockZero());

		AP11LMManeuverPAD(&opt, *form);
		sprintf(form->purpose, "No PDI +12 ABORT");
		sprintf(form->remarks, "TIG 373: %+05.1f", AGSTIG / 60.0);
		form->type = 1;
		form->t_CSI = round(t_CSI);
		form->t_TPI = round(t_TPI);
	}
	break;
	case 136: //LGC LM SV update (PDI-10)
	{
		SV sv0, sv1;
		char buffer[1000];

		sv0 = StateVectorCalc(calcParams.tgt);

		double SV_GET = OrbMech::GETfromMJD(sv0.MJD, CalcGETBase());
		sv1 = coast(sv0, (calcParams.PDI - 10.0*60.0) - SV_GET);

		AGCStateVectorUpdate(buffer, sv1, false);

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "LM state vector (PDI-10)");
		}
	}
	break;
	case 137: //CMC pre PDI uplinks
	{
		SV sv_CSM1, sv_LM1, sv_CSM2, sv_LM2;
		char buffer1[1000];
		char buffer2[1000];

		//State vectors
		sv_CSM1 = StateVectorCalc(calcParams.src);
		sv_LM1 = StateVectorCalc(calcParams.tgt);

		double SV_GET = OrbMech::GETfromMJD(sv_LM1.MJD, CalcGETBase());

		sv_CSM2 = coast(sv_CSM1, calcParams.PDI - SV_GET);
		sv_LM2 = coast(sv_LM1, (calcParams.PDI - 10.0*60.0) - SV_GET);

		AGCStateVectorUpdate(buffer1, sv_CSM2, true);
		AGCStateVectorUpdate(buffer2, sv_LM2, false);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM SV (P24 T2 ACQ), LM SV (PDI-10)");
		}
	}
	break;
	case 138: //LGC CSM SV update (155:15)
	{
		EphemerisData sv0, sv1;
		char buffer1[1000];

		sv0 = StateVectorCalcEphem(calcParams.src);
		sv1 = coast(sv0, OrbMech::HHMMSSToSS(155, 15, 0) - GETfromGMT(sv0.GMT));

		AGCStateVectorUpdate(buffer1, RTCC_MPT_LM, RTCC_MPT_CSM, sv1);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector (155:15)");
		}
	}
	break;
	case 140: //TLC lunar pericynthion evaluation
	{
		SV sv_peri = FindPericynthion(StateVectorCalc(calcParams.src));
		double h_peri = length(sv_peri.R) - OrbMech::R_Moon;

		if ((h_peri / 1852.0) > 20.0 )
		{
			scrubbed = true;
		}
	}
	break;
	case 141: //Liftoff Time Evaluation (TLC)
	{
		double dt = GetCMCClockZero() - NominalLaunchGMT;

		if (abs(dt) < 60.0)
		{
			scrubbed = true;
		}

		if (!scrubbed)
		{
			int hh, mm;
			double ss;

			calcParams.LOI += dt;

			calcParams.TLI = dt;

			OrbMech::SStoHHMMSS(dt, hh, mm, ss);

			sprintf(upMessage, "Liftoff time update required\nAdjust times by: %d:%d:%.2lf", hh, mm, ss);
		}
	}
	break;
	case 142: //Liftoff time Evaluation REV 26
	case 143: //Liftoff time Evaluation REV 66
	{
		double t_lng, ttoLNG, dt, GETfp, GETm, revs;
		SV sv0, sv1, sv2;

		sv0 = StateVectorCalc(calcParams.src);

		if (fcn == 142)
		{
			sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, 0, RTCC_ENGINETYPE_CSMSPS);
			revs = 12.5;
			GETfp = OrbMech::HHMMSSToSS(137, 49, 3);
		}
		else
		{
			sv1 = sv0;
			revs = 3.5;
			GETfp = OrbMech::HHMMSSToSS(216, 55, 5);
		}

		double t_P = OrbMech::period(sv1.R, sv1.V, OrbMech::mu_Moon);
		sv2 = coast(sv1, t_P*revs);

		t_lng = OrbMech::P29TimeOfLongitude(sv2.R, sv2.V, sv2.MJD, GetGravref(BODY_MOON), 180.0*RAD);
		ttoLNG = (t_lng - sv2.MJD)*24.0 * 3600.0;
		GETm = (sv2.MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;

		dt = GETfp - GETm;

		if (abs(dt) < 120.0)
		{
			scrubbed = true;
		}

		if (!scrubbed)
		{
			int hh, mm;
			double ss;

			calcParams.LOI += dt;
			calcParams.DOI += dt;
			calcParams.PDI += dt;
			CZTDTGTU.GETTD += dt;
			TimeofIgnition += dt;

			calcParams.TLI = dt;

			OrbMech::SStoHHMMSS(dt, hh, mm, ss);

			sprintf(upMessage, "Liftoff time update required\nAdjust times by: %d:%d:%.2lf", hh, mm, ss);
		}
	}
	break;
	case 144: //CMC Liftoff Time Update
	{
		char buffer[128];

		IncrementAGCLiftoffTime(buffer, RTCC_MPT_CSM, -calcParams.TLI);

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Liftoff time update");
		}
	}
	break;
	case 145: //RTCC Liftoff Time Update
	{
		char Buff[128];

		//P10 MED: Enter actual liftoff time
		double TEPHEM0, tephem_scal;
		Saturn *cm = (Saturn *)calcParams.src;

		//Get TEPHEM
		TEPHEM0 = 41133.;
		tephem_scal = GetTEPHEMFromAGC(&cm->agc.vagc);
		double LaunchMJD = (tephem_scal / 8640000.) + TEPHEM0;
		LaunchMJD = (LaunchMJD - SystemParameters.GMTBASE)*24.0;

		int hh, mm;
		double ss;

		OrbMech::SStoHHMMSS(LaunchMJD*3600.0, hh, mm, ss);

		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//P15: CMC, LGC and AGS clock zero
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);
		sprintf_s(Buff, "P15,LGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		if (mcc->MoonRev < 50)
		{
			GMGMED("P15,AGS,,110:00:00;");
		}
		else
		{
			GMGMED("P15,AGS,,180:00:00;");
		}
	}
	break;
	case 146: //CMC clock sync evaluation
	case 147: //LGC clock sync evaluation
	{
		double TEPHEM0, tephem, t_AGC, t_actual, dt;

		if (fcn == 146)
		{
			Saturn *cm = (Saturn *)calcParams.src;
			tephem = GetTEPHEMFromAGC(&cm->agc.vagc);
			t_AGC = GetClockTimeFromAGC(&cm->agc.vagc) / 100.0;
		}
		else
		{
			LEM *lem = (LEM *)calcParams.tgt;
			tephem = GetTEPHEMFromAGC(&lem->agc.vagc);
			t_AGC = GetClockTimeFromAGC(&lem->agc.vagc) / 100.0;
		}

		TEPHEM0 = 41133.;
		tephem = (tephem / 8640000.) + TEPHEM0;
		t_actual = (oapiGetSimMJD() - tephem) * 86400.;
		dt = t_actual - t_AGC;

		if (abs(dt) < 0.005)
		{
			scrubbed = true;
		}

		if (!scrubbed)
		{
			calcParams.TLI = dt;
		}
	}
	break;
	case 148: //CMC clock sync uplink
	case 149: //LGC clock sync uplink
	{
		char buffer[128];

		if (fcn == 148)
		{
			IncrementAGCTime(buffer, RTCC_MPT_CSM, calcParams.TLI);
		}
		else
		{
			IncrementAGCTime(buffer, RTCC_MPT_LM, calcParams.TLI);
		}

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Clock sync");
		}
	}
	break;
	case 150: //Pan camera photo PAD (90:45)
	case 151: //Pan camera photo PAD (91:20)
	case 152: //Pan camera photo PAD (114:10)
	case 153: //Pan camera photo PAD (116:20)
	case 154: //Earthshine photo PAD (121:05)
	case 155: //Zodiacal light photo PAD (SR)
	case 156: //Solar corona photo PAD (137:33)
	case 157: //Pan camera photo PAD (141:50)
	case 158: //Lunar sounder PAD (156:50)
	case 159: //Lunar sounder PAD (158:40)
	case 160: //UV solar atmosphere PAD (160:38)
	case 161: //Zodiacal light photo PAD (163:13)
	case 162: //Lunar sounder PAD (166:40)
	case 163: //Pan camera photo PAD(183:45)
	case 164: //Zodiacal light photo PAD (SR)
	case 165: //Lunar sounder T-STOP (196:20)
	case 166: //Solar corona photo PAD (SS)
	case 167: //Pan camera photo PAD (209:15)
	case 168: //Pan camera photo PAD
	case 169: //Lunar sounder PAD (213:15)
	case 170: //Lunar sounder PAD (213:38)
	case 171: //Lunar sounder PAD (230:55)
	case 172: //Lunar sounder PAD
	case 173: //Lunar sounder PAD (232:20)
	case 174: //Pan camera photo PAD (233:20)
	case 175: //Pan camera photo PAD (233:40)
	{
		GENERICPAD * form = (GENERICPAD *)pad;

		double MJD;
		OBJHANDLE gravref;
		double GET1, GET2, coast_t, lng1, lng2, dt;
		EphemerisData sv1, sv2;
		int padtype = 0; //Times relative to:  0 = one longitude (start), 1 = one longitude (stop), 2 = two longitudes, 3 = SR, 4 = SS, 5 = SS to SR

		int hh[2], mm[2];
		double ss[2];

		char purpose[128];

		sv1 = StateVectorCalcEphem(calcParams.src);

		lng1 = lng2 = dt = 0.0;
		coast_t = 1.0*3600.0;

		if (fcn == 150) //Pan camera photo PAD (90:45)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = -152.0*RAD;
			lng2 = 144.0*RAD;
		}
		else if (fcn == 151) //Pan camera photo PAD (91:20)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 123.0*RAD;
			lng2 = 95.0*RAD;
		}
		else if (fcn == 152) //Pan camera photo PAD (114:10)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = -172.0*RAD;
			lng2 = 100.0*RAD;
		}
		else if (fcn == 153) //Pan camera photo PAD (116:20)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 102.0*RAD;
			lng2 = 14.0*RAD;
		}
		else if (fcn == 154) //Earthshine photo PAD (121:05)
		{
			sprintf(purpose, "EARTHSHINE PHOTO PAD");
			padtype = 4;
		}
		else if (fcn == 155) //Zodiacal light photo PAD (SR)
		{
			sprintf(purpose, "ZODIACAL LIGHT PHOTO PAD (SR)");
			padtype = 3;
			dt = -15.0*60.0;
			coast_t = 10.0*60.0;
		}
		else if (fcn == 156) //Solar corona photo PAD (137:33)
		{
			sprintf(purpose, "SOLAR CORONA PHOTO PAD");
			padtype = 3;
			dt = -7.0*60.0;
			coast_t = 30.0*60.0;
		}
		else if (fcn == 157) //Pan camera photo PAD (141:50)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 155.0*RAD;
			lng2 = 85.0*RAD;
		}
		else if (fcn == 158) //Lunar sounder PAD (156:50)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = -49.0*RAD;
			lng2 = -64.0*RAD;
			coast_t = 30.0*60.0;
		}
		else if (fcn == 159) //Lunar sounder PAD (158:40)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = -49.0*RAD;
			lng2 = -64.0*RAD;
			coast_t = 140.0*60.0;
		}
		else if (fcn == 160) //UV solar atmosphere PAD (160:38)
		{
			sprintf(purpose, "UV SOLAR ATMOSPHERE PAD");
			padtype = 5;
			coast_t = 35.0*60.0;
		}
		else if (fcn == 161) //Zodiacal light photo PAD (163:13)
		{
			sprintf(purpose, "ZODIACAL LIGHT PHOTO PAD (SR)");
			padtype = 3;
			dt = -15.0*60.0;
			coast_t = 25.0*60.0;
		}
		else if (fcn == 162) //Lunar sounder PAD (166:40)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = -45.0*RAD;
			lng2 = -60.0*RAD;
			coast_t = 40.0*60.0;
		}
		else if (fcn == 163) //Pan camera photo PAD(183:45)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 80.0*RAD;
			lng2 = 26.0*RAD;
			coast_t = 55.0*60.0;
		}
		else if (fcn == 164) //Zodiacal light photo PAD (SR)
		{
			sprintf(purpose, "ZODIACAL LIGHT PHOTO PAD (SR)");
			padtype = 3;
			dt = -15.0*60.0;
			coast_t = 30.0*60.0;
		}
		else if (fcn == 165) //Lunar sounder T-STOP (196:20)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 1;
			lng1 = -39.0*RAD;
			coast_t = 15.0*60.0;
		}
		else if (fcn == 166) //Solar corona photo PAD (SS)
		{
			sprintf(purpose, "SOLAR CORONA PHOTO PAD (SS)");
			padtype = 4;
			dt = -5.0*60.0;
			coast_t = 5.0*60.0;
		}
		else if (fcn == 167) //Pan camera photo PAD (209:15)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 133.0*RAD;
			lng2 = 90.0*RAD;
		}
		else if (fcn == 168) //Pan camera photo PAD
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 33.0*RAD;
			lng2 = 27.0*RAD;
			coast_t = 1.0*60.0;
		}
		else if (fcn == 169) //Lunar sounder PAD (213:15)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = 110.0*RAD;
			lng2 = 98.0*RAD;
		}
		else if (fcn == 170) //Lunar sounder PAD (213:38)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = 49.0*RAD;
			lng2 = -8.0*RAD;
			coast_t = 80.0*60.0;
		}
		else if (fcn == 171) //Lunar sounder PAD (230:55)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = 135.0*RAD;
			lng2 = 117.0*RAD;
			coast_t = 45.0*60.0;
		}
		else if (fcn == 172) //Lunar sounder PAD
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = 58.0*RAD;
			lng2 = -8.0*RAD;
			coast_t = 5.0*60.0;
		}
		else if (fcn == 173) //Lunar sounder PAD (232:20)
		{
			sprintf(purpose, "LUNAR SOUNDER PAD");
			padtype = 2;
			lng1 = -117.0*RAD;
			lng2 = -144.0*RAD;
			coast_t = 40.0*60.0;
		}
		else if (fcn == 174) //Pan camera photo PAD (233:20)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = 67.0*RAD;
			lng2 = 25.0*RAD;
			coast_t = 90.0*60.0;
		}
		else if (fcn == 175) //Pan camera photo PAD (233:40)
		{
			sprintf(purpose, "PAN CAMERA PHOTO PAD");
			padtype = 2;
			lng1 = -5.0*RAD;
			lng2 = -45.0*RAD;
			coast_t = 10.0*60.0;
		}

		sv2 = coast(sv1, coast_t);

		MJD = OrbMech::MJDfromGET(sv2.GMT, GetGMTBase());
		gravref = GetGravref(sv2.RBI);

		if (padtype < 3)
		{
			double t_lng, ttoLNG;

			t_lng = OrbMech::P29TimeOfLongitude(sv2.R, sv2.V, MJD, gravref, lng1);
			ttoLNG = (t_lng - MJD)*24.0 * 3600.0;
			GET1 = (MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;

			t_lng = OrbMech::P29TimeOfLongitude(sv2.R, sv2.V, MJD, gravref, lng2);
			ttoLNG = (t_lng - MJD)*24.0 * 3600.0;
			GET2 = (MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;

			OrbMech::SStoHHMMSS(GET1, hh[0], mm[0], ss[0]);
			OrbMech::SStoHHMMSS(GET2, hh[1], mm[1], ss[1]);

			if (padtype < 1) //time over 1 longitude (start)
			{
				sprintf(form->paddata, "%s\nT-START: %03d:%02d:%2.0lf", purpose, hh[0], mm[0], ss[0]);
			}
			else if (padtype < 2) //time over 1 longitude (stop)
			{
				sprintf(form->paddata, "%s\nT-STOP: %03d:%02d:%2.0lf", purpose, hh[0], mm[0], ss[0]);
			}
			else //time over 2 longitudes
			{
				sprintf(form->paddata, "%s\nT-START: %03d:%02d:%2.0lf\nT-STOP: %03d:%02d:%2.0lf", purpose, hh[0], mm[0], ss[0], hh[1], mm[1], ss[1]);
			}
		}
		else
		{
			double ttoSR, ttoSS;

			OBJHANDLE hSun = oapiGetObjectByName("Sun");

			ttoSR = OrbMech::sunrise(sv2.R, sv2.V, MJD, gravref, hSun, 1, 0, true);
			ttoSS = OrbMech::sunrise(sv2.R, sv2.V, MJD, gravref, hSun, 0, 0, true);

			GET1 = (MJD - CalcGETBase())*24.0*3600.0 + ttoSR;
			GET2 = (MJD - CalcGETBase())*24.0*3600.0 + ttoSS;

			if (padtype < 4) //relative to SR
			{
				GET1 += dt;
				OrbMech::SStoHHMMSS(GET1, hh[0], mm[0], ss[0]);
				sprintf(form->paddata, "%s\nT-START: %03d:%02d:%2.0lf", purpose, hh[0], mm[0], ss[0]);
			}
			else if (padtype < 5) //relative to SS
			{
				GET2 += dt;
				OrbMech::SStoHHMMSS(GET2, hh[0], mm[0], ss[0]);
				sprintf(form->paddata, "%s\nT-START: %03d:%02d:%2.0lf", purpose, hh[0], mm[0], ss[0]);
			}
			else //SS to SR
			{
				OrbMech::SStoHHMMSS(GET1, hh[0], mm[0], ss[0]);
				OrbMech::SStoHHMMSS(GET2, hh[1], mm[1], ss[1]);
				sprintf(form->paddata, "%s\nT-START: %03d:%02d:%2.0lf\nT-STOP: %03d:%02d:%2.0lf", purpose, hh[1], mm[1], ss[1], hh[0], mm[0], ss[0]);
			}
		}
	}
	break;
	case 200: //TEI EVALUATION
	{
		SV sv;
		OELEMENTS coe;

		sv = StateVectorCalc(calcParams.src);
		coe = OrbMech::coe_from_sv(sv.R, sv.V, OrbMech::mu_Moon);

		if (coe.e < 0.5)
		{
			scrubbed = true;
		}
	}
	break;
	case 205: //EI Evaluation
	case 210: //MCC-5 UPDATE
	case 212: //MCC-6 UPDATE
	case 213: //MCC-7 DECISION
	case 214: //MCC-7 UPDATE
	case 300: //GENERIC MCC
	{
		EntryOpt entopt;
		EntryResults res;
		double MCCtime;
		char manname[8];
		SV sv;

		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		if (fcn == 210)
		{
			MCCtime = calcParams.TEI + 17.0*3600.0;
			sprintf(manname, "MCC-5");
		}
		else if (fcn == 211 || fcn == 212)
		{
			MCCtime = calcParams.EI - 22.0*3600.0;
			sprintf(manname, "MCC-6");
		}
		else if (fcn == 213 || fcn == 214)
		{
			MCCtime = calcParams.EI - 3.0*3600.0;
			sprintf(manname, "MCC-7");
		}
		else if (fcn == 300)
		{
			MCCtime = calcParams.TEI + 5.0*3600.0;
			sprintf(manname, "MCC");
		}
		else
		{
			if (AGCGravityRef(calcParams.src) == oapiGetObjectByName("Moon"))
			{
				sv = coast(sv, 20.0*3600.0);
			}

			MCCtime = OrbMech::GETfromMJD(sv.MJD, CalcGETBase());
		}

		if (mcc->MoonRev >= 76)
		{
			entopt.entrylongmanual = true;
			entopt.lng = PZREAP.PTPLongitude[0]; //EOM
		}
		else
		{
			entopt.entrylongmanual = false;
			entopt.ATPLine = 0; //MPL
		}

		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.RV_MCC = sv;
		entopt.TIGguess = MCCtime;
		entopt.vessel = calcParams.src;
		entopt.r_rbias = PZREAP.RRBIAS;

		if (calcParams.src->DockingStatus(0) == 1)
		{
			entopt.csmlmdocked = true;
		}
		else
		{
			entopt.csmlmdocked = false;
		}

		entopt.type = 3;
		EntryTargeting(&entopt, &res);

		if (fcn != 205)
		{
			AP11ManPADOpt opt;
			MATRIX3 REFSMMAT;

			AP11MNV * form = (AP11MNV *)pad;

			//Longitude control before EI-24h
			if (MCCtime < res.GET400K - 24.0*3600.0)
			{
				entopt.type = 1;
				entopt.t_Z = res.GET400K;
				EntryTargeting(&entopt, &res);
			}

			//Apollo 11 Mission Rules
			if (MCCtime > res.GET400K - 50.0*3600.0)
			{
				if (length(res.dV_LVLH) < 1.0*0.3048)
				{
					scrubbed = true;
				}
			}
			else
			{
				if (length(res.dV_LVLH) < 2.0*0.3048)
				{
					scrubbed = true;
				}
			}

			if (fcn != 213)
			{
				if (fcn == 214)
				{
					REFSMMATOpt refsopt;
					refsopt.REFSMMATopt = 3;
					refsopt.vessel = calcParams.src;
					refsopt.useSV = true;
					refsopt.RV_MCC = res.sv_postburn;

					REFSMMAT = REFSMMATCalc(&refsopt);
				}
				else
				{
					REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
				}

				if (scrubbed)
				{
					//Entry prediction without maneuver
					EntryUpdateCalc(sv, PZREAP.RRBIAS, true, &res);

					res.dV_LVLH = _V(0, 0, 0);
					res.P30TIG = entopt.TIGguess;
				}
				else
				{
					opt.dV_LVLH = res.dV_LVLH;
					opt.enginetype = SPSRCSDecision(SPS_THRUST / (calcParams.src->GetMass() + GetDockedVesselMass(calcParams.src)), res.dV_LVLH);
					opt.HeadsUp = false;
					opt.REFSMMAT = REFSMMAT;
					opt.TIG = res.P30TIG;
					opt.vessel = calcParams.src;

					if (calcParams.src->DockingStatus(0) == 1)
					{
						opt.vesseltype = 1;
					}
					else
					{
						opt.vesseltype = 0;
					}

					AP11ManeuverPAD(&opt, *form);
					sprintf(form->purpose, manname);
					form->lat = res.latitude*DEG;
					form->lng = res.longitude*DEG;
					form->RTGO = res.RTGO;
					form->VI0 = res.VIO / 0.3048;
					form->GET05G = res.GET05G;
				}
			}

			if (scrubbed)
			{
				//Scrubbed MCC-5 and MCC-6
				if (fcn == 210 || fcn == 212)
				{
					char buffer1[1000];
					char buffer2[1000];

					sprintf(upMessage, "%s has been scrubbed", manname);
					sprintf(upDesc, "CSM state vector and V66, entry target");

					AGCStateVectorUpdate(buffer1, sv, true, true);
					CMCEntryUpdate(buffer2, res.latitude, res.longitude);

					sprintf(uplinkdata, "%s%s", buffer1, buffer2);
					if (upString != NULL) {
						// give to mcc
						strncpy(upString, uplinkdata, 1024 * 3);
					}
				}
				//MCC-6 and 7 decision
				else if (fcn == 211 || fcn == 213 || fcn == 300)
				{
					sprintf(upMessage, "%s has been scrubbed", manname);
				}
				//Scrubbed MCC-7
				else if (fcn == 214)
				{
					char buffer1[1000];
					char buffer2[1000];
					char buffer3[1000];

					sprintf(upMessage, "%s has been scrubbed", manname);
					sprintf(upDesc, "CSM state vector and V66, entry target, Entry REFSMMAT");

					AGCStateVectorUpdate(buffer1, sv, true, true);
					CMCEntryUpdate(buffer2, res.latitude, res.longitude);
					AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

					sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
					if (upString != NULL) {
						// give to mcc
						strncpy(upString, uplinkdata, 1024 * 3);
					}
				}
			}
			else
			{
				//MCC-5 or MCC-6 or Generic MCC
				if (fcn == 210 || fcn == 212 || fcn == 300)
				{
					char buffer1[1000];
					char buffer2[1000];

					AGCStateVectorUpdate(buffer1, sv, true, true);
					CMCRetrofireExternalDeltaVUpdate(buffer2, res.latitude, res.longitude, res.P30TIG, res.dV_LVLH);

					sprintf(uplinkdata, "%s%s", buffer1, buffer2);
					if (upString != NULL) {
						// give to mcc
						strncpy(upString, uplinkdata, 1024 * 3);
						sprintf(upDesc, "CSM state vector and V66, target load");
					}
				}
				//MCC-6 and 7 decision
				else if (fcn == 211 || fcn == 213)
				{
					sprintf(upMessage, "%s will be executed", manname);
				}
				//MCC-7
				else if (fcn == 214)
				{
					char buffer1[1000];
					char buffer2[1000];
					char buffer3[1000];

					AGCStateVectorUpdate(buffer1, sv, true, true);
					CMCRetrofireExternalDeltaVUpdate(buffer2, res.latitude, res.longitude, res.P30TIG, res.dV_LVLH);
					AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

					sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
					if (upString != NULL) {
						// give to mcc
						strncpy(upString, uplinkdata, 1024 * 3);
						sprintf(upDesc, "CSM state vector and V66, target load, Entry REFSMMAT");
					}
				}
			}

			//Save for further use
			DeltaV_LVLH = res.dV_LVLH;
			TimeofIgnition = res.P30TIG;
			SplashLatitude = res.latitude;
			SplashLongitude = res.longitude;
			calcParams.SVSTORE1 = res.sv_postburn;
		}

		calcParams.EI = res.GET400K;
	}
	break;
	case 216: //ENTRY PAD (ASSUMES MCC-6)
	case 217: //ENTRY PAD (ASSUMES MCC-7)
	case 218: //FINAL LUNAR ENTRY PAD
	{
		AP11ENT * form = (AP11ENT *)pad;

		SV sv;
		LunarEntryPADOpt entopt;
		MATRIX3 REFSMMAT;

		sv = StateVectorCalc(calcParams.src);

		//Burn scrubbed or not
		if (length(DeltaV_LVLH) != 0.0 && fcn != 218)
		{
			entopt.direct = false;
		}
		else
		{
			entopt.direct = true;
		}

		if (fcn == 218)
		{
			REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		}
		else
		{
			REFSMMATOpt refsopt;

			refsopt.REFSMMATopt = 3;
			refsopt.vessel = calcParams.src;
			refsopt.useSV = true;
			refsopt.RV_MCC = calcParams.SVSTORE1;

			REFSMMAT = REFSMMATCalc(&refsopt);
		}

		entopt.dV_LVLH = DeltaV_LVLH;
		entopt.lat = SplashLatitude;
		entopt.lng = SplashLongitude;
		entopt.P30TIG = TimeofIgnition;
		entopt.REFSMMAT = REFSMMAT;
		entopt.sv0 = sv;
		entopt.SxtStarCheckAttitudeOpt = false;

		LunarEntryPAD(&entopt, *form);
		sprintf(form->Area[0], "MIDPAC");
		if (entopt.direct == false)
		{
			if (fcn == 216 && length(DeltaV_LVLH) != 0.0)
			{
				sprintf(form->remarks[0], "Assumes MCC6");
			}
			else if (fcn == 217 && length(DeltaV_LVLH) != 0.0)
			{
				sprintf(form->remarks[0], "Assumes MCC7");
			}
		}

		if (fcn == 218)
		{
			char buffer1[1000];
			char buffer2[1000];

			AGCStateVectorUpdate(buffer1, sv, true, true);
			CMCEntryUpdate(buffer2, SplashLatitude, SplashLongitude);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, entry update");
			}
		}
	}
	break;
	}

	return scrubbed;
}