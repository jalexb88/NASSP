/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2023

RTCC Calculations for Mission JX (Generic MCC for custom J missions)

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

//This mission profile is based on Apollo 15 and has a CSM flown DOI.
//See "MCC for custom missions.txt" in Doc\Project Apollo - NASSP\Programmers Notes

bool RTCC::CalculationMTP_JX(int fcn, LPVOID &pad, char * upString, char * upDesc, char * upMessage)
{
	char uplinkdata[1024 * 3];
	bool preliminary = false;
	bool scrubbed = false;

	srand((unsigned int)time(NULL));

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
	case 10: //MISSION INITIALIZATION
	{
		GENERICPAD * form = (GENERICPAD *)pad;

		Saturn *cm = (Saturn *)calcParams.src;
		SaturnV *SatV = (SaturnV*)cm;

		char Buff[128];

		int hh, mm;
		double ss;

		//P80 MED: mission initialization
		sprintf_s(Buff, "P80,1,CSM,%d,%d,%d;", GZGENCSN.MonthofLiftoff, GZGENCSN.DayofLiftoff, GZGENCSN.Year);
		GMGMED(Buff);

		//Predict liftoff time and simulate launch
		double LOpred = RTCCPresentTimeGMT() + abs(SatV->MissionTime);
		OrbMech::SStoHHMMSS(LOpred, hh, mm, ss);
		sprintf_s(Buff, "P10,CSM,%d:%d:%.2lf,TRAJ;", hh, mm, ss);
		GMGMED(Buff);

		//Initialize SFP
		sprintf_s(Buff, "F62,,1,%.3lf;", GZLTRA.Azimuth * DEG);
		GMGMED(Buff);

		//Set mission parameters
		calcParams.LOI = calcParams.Phasing = GETfromGMT(PZSFPTAB.blocks[0].GMT_pc2);
		PZMCCPLN.SITEROT = -GZGENCSN.LDPPDescentFlightArc;
		PZLOIPLN.DW = PZMCCPLN.SITEROT * DEG;
		GZGENCSN.LDPPDwellOrbits = PZMCCPLN.REVS2;

		sprintf(form->paddata, "LAUNCH AZIMUTH: %.2f", GZLTRA.Azimuth * DEG);
	}
	break;
	case 11: //GROUND LIFTOFF TIME UPDATE
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

		//P12: CSM GRR and Azimuth
		SaturnV *SatV = (SaturnV*)cm;
		LVDCSV *lvdc = (LVDCSV*)SatV->iu->GetLVDC();
		double Azi = lvdc->Azimuth*DEG;
		double T_GRR = lvdc->T_L;

		sprintf_s(Buff, "P12,CSM,%d:%d:%.2lf,%.2lf;", hh, mm, ss, Azi);
		GMGMED(Buff);

		//P15: CMC, LGC
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);
		sprintf_s(Buff, "P15,LGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//AGS clock zero
		double t_AGS = round((CZTDTGTU.GETTD - 18000.0) / 36000.0)*36000.0;
		OrbMech::SStoHHMMSS(t_AGS, hh, mm, ss);
		sprintf_s(Buff, "P15,AGS,,%03d:00:00;", hh);
		GMGMED(Buff);

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
	case 12: //TLI SIMULATION
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
		if (calcParams.CSI < 100.0)
		{
			//First opportunity
			GMGMED("M68,CSM,1;");
		}
		else
		{
			//Second opportunity
			GMGMED("M68,CSM,2;");
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
	case 13: //EPO SV UPDATE
	{
		double GMTSV;
		EphemerisData sv, sv_uplink;
		SV sv1;
		char buffer1[1000];

		sv = StateVectorCalcEphem(calcParams.src);

		GMTSV = PZMPTCSM.TimeToBeginManeuver[0] - 10.0*60.0; //10 minutes before TB6
		sv_uplink = coast(sv, GMTSV - sv.GMT, RTCC_MPT_CSM); //Coast with venting and drag taken into account

		AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv_uplink, true);

		sprintf(uplinkdata, "%s", buffer1);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM state vector and V66");
		}

		//Delete maneuvers from MPT
		GMGMED("M62,CSM,1,D;");

		//Set anchor vector time to 0, so that no trajectory updates are done anymore
		EZANCHR1.AnchorVectors[9].Vector.GMT = 0.0;
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

		//Add Ordeal data
		int yaw = 0;

		sprintf(form->remarks, "\nXX%04.1f R2 Align\nXX%04.1f R2 Ign\nXX56:45 ORDEAL Start\nXXX00%d YAW", form->IgnATT.y, form->IgnATT.y - 7.0, yaw);
	}
	break;
	case 15: //TLI Evaluation
	{
		SaturnV *SatV = (SaturnV*)calcParams.src;
		LVDCSV *lvdc = (LVDCSV*)SatV->iu->GetLVDC();

		if (lvdc->LVDC_Timebase == 5)
		{
			scrubbed = true;

			char Buff[128];

			//F62: Interpolate SFP for 2nd opportunity
			sprintf_s(Buff, "F62,,2,%.3lf;", lvdc->Azimuth*DEG);
			GMGMED(Buff);

			//Save LOI time
			calcParams.LOI = calcParams.Phasing = GETfromGMT(PZSFPTAB.blocks[0].GMT_pc2);

			calcParams.CSI = 9999.0; //Use as flag for TLI-2
		}
		else
		{
			calcParams.CSI = 0.0;
		}
	}
	break;
	case 18: //PTC REFSMMAT
	{
		char buffer[1000];
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;

		refsopt.REFSMMATopt = 6;
		refsopt.REFSMMATTime = OrbMech::MJDfromGET(864000.0, CalcGETBase()); //10 days into mission

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

		double LOIGET = calcParams.Phasing;

		LOIGET -= 195.0;

		MCC1GET = TLIbase + 9.0*3600.0;
		MCC2GET = TLIbase + 28.0*3600.0;

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
		calcParams.TEI = PZMCCDIS.data[0].GET_TEI;
		calcParams.EI = PZMCCDIS.data[0].GET_LC;

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
		int engine, sol;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);

		if (PZMCCPLN.REVS1 >= 2.0)
		{
			sol = 0;
		}
		else
		{
			sol = 1;
		}

		bool loierr = PMMLRBTI(sv);

		h_peri_tgt = PZSFPTAB.blocks[1].h_pc2 / 1852.0;
		h_node_tgt = PZSFPTAB.blocks[1].h_nd / 1852.0;

		if (loierr == false)
		{
			h_peri = PZLRBTI.h_pc;
			h_node = PZLRBTI.sol[sol].H_ND;
			ApsidRot = PZLRBTI.sol[sol].f_ND_E - PZLRBTI.sol[sol].f_ND_H;
			while (ApsidRot > 180.0)
			{
				ApsidRot -= 360.0;
			}
			while (ApsidRot < -180.0)
			{
				ApsidRot += 360.0;
			}

			//Maneuver execution criteria
			if (h_peri > h_peri_tgt - 5.0 && h_peri < h_peri_tgt + 5.0)
			{
				if (h_node > h_node_tgt - 10.0 && h_node < h_node_tgt + 15.0)
				{
					if (abs(ApsidRot) < 45.0)
					{
						scrubbed = true;
						calcParams.LOI = PZLRBTI.sol[sol].GETLOI;
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
	case 27: // LOI MANEUVER (PRELIMINARY)
	case 28: // LOI MANEUVER
	{
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		AP11ManPADOpt manopt;
		double P30TIG, tig, CSMmass, LMmass;
		VECTOR3 dV_LVLH, dv;
		EphemerisData sv;

		int sol;

		AP11MNV * form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src); //State vector for uplink
		CSMmass = calcParams.src->GetMass();
		LMmass = calcParams.tgt->GetMass();

		if (PZMCCPLN.REVS1 >= 2.0)
		{
			sol = 0;
		}
		else
		{
			sol = 1;
		}

		PMMLRBTI(sv);

		tig = GETfromGMT(PZLRBELM.sv_man_bef[sol].GMT);
		dv = PZLRBELM.V_man_after[sol] - PZLRBELM.sv_man_bef[sol].V;
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
	case 29: //LS REFSMMAT UPLINK (post LOI)
	case 30: //DOI update
	{
		SV sv;

		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		//MED K17
		GZGENCSN.LDPPPoweredDescentSimFlag = false;

		//MED K16
		med_k16.Mode = 4;
		med_k16.Sequence = 1;
		med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.LOI + 3.5*3600.0;

		LunarDescentPlanningProcessor(sv);

		calcParams.DOI = GETfromGMT(PZLDPELM.sv_man_bef[0].GMT);
		calcParams.PDI = PZLDPDIS.PD_GETIG;
		CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;
		calcParams.TPI = CZTDTGTU.GETTD + (double(PZMCCPLN.LOPC_M) * 7124.0); //TPI time temporarily used for LOPC time
		calcParams.LunarLiftoff = calcParams.TPI + (double(PZMCCPLN.LOPC_N) * 7124.0);

		if (fcn == 29)
		{
			MATRIX3 REFSMMAT;
			REFSMMATOpt opt;

			char buffer1[1000];

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
		else
		{
			double P30TIG;
			VECTOR3 dV_LVLH;

			AP11ManPADOpt manopt;

			char buffer1[1000];
			char buffer2[1000];

			AP11MNV * form = (AP11MNV *)pad;

			PoweredFlightProcessor(sv, PZLDPDIS.GETIG[0], RTCC_ENGINETYPE_CSMSPS, 0.0, PZLDPDIS.DVVector[0] * 0.3048, true, P30TIG, dV_LVLH);

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
			sprintf(form->purpose, "DOI");
			sprintf(form->remarks, "Four-jet ullage for 16 seconds");

			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;

			AGCStateVectorUpdate(buffer1, sv, true, true);
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
	case 31: //DOI trim evaluation
	case 32: //DOI trim update
	{
		SV sv;

		sv = StateVectorCalc(calcParams.src); //State vector for uplink

		if (fcn == 31)
		{
			double apo, peri;
			OrbMech::periapo(sv.R, sv.V, OrbMech::mu_Moon, apo, peri);

			peri -= OrbMech::R_Moon;

			if (peri >= GZGENCSN.LDPPHeightofPDI - (20000.0*0.3048) && peri <= GZGENCSN.LDPPHeightofPDI + (20000.0*0.3048))
			{
				scrubbed = true;
				sprintf(upMessage, "DOI trim scrubbed");
			}
		}
		else
		{
			MATRIX3 REFSMMAT;
			REFSMMATOpt opt;

			double P30TIG;
			VECTOR3 dV_LVLH;
			int engine;

			AP11ManPADOpt manopt;
			AP11MNV * form = (AP11MNV *)pad;

			char buffer1[1000];
			char buffer2[1000];
			char buffer3[1000];

			//MED K17
			GZGENCSN.LDPPPoweredDescentSimFlag = false;
			GZGENCSN.LDPPDwellOrbits = 4;

			//MED K16
			med_k16.Mode = 4;
			med_k16.Sequence = 1;
			med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.PDI - 9.0*3600.0;

			LunarDescentPlanningProcessor(sv);

			calcParams.DOI = GETfromGMT(PZLDPELM.sv_man_bef[0].GMT);
			calcParams.PDI = PZLDPDIS.PD_GETIG;
			CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;
			calcParams.TPI = CZTDTGTU.GETTD + (double(PZMCCPLN.LOPC_M) * 7124.0); //TPI time temporarily used for LOPC time
			calcParams.LunarLiftoff = calcParams.TPI + (double(PZMCCPLN.LOPC_N) * 7124.0);

			engine = SPSRCSDecision(SPS_THRUST / (calcParams.src->GetMass() + calcParams.tgt->GetMass()), PZLDPDIS.DVVector[0]);
			PoweredFlightProcessor(sv, PZLDPDIS.GETIG[0], engine, 0.0, PZLDPDIS.DVVector[0] * 0.3048, true, P30TIG, dV_LVLH);

			opt.dV_LVLH = dV_LVLH;
			opt.REFSMMATTime = P30TIG;
			opt.REFSMMATopt = 0;
			opt.vessel = calcParams.src;

			REFSMMAT = REFSMMATCalc(&opt);

			manopt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = engine;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = REFSMMAT;
			manopt.sxtstardtime = -40.0*60.0;
			manopt.TIG = P30TIG;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 1;

			AP11ManeuverPAD(&manopt, *form);
			sprintf(form->purpose, "DOI trim");

			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;

			AGCStateVectorUpdate(buffer1, sv, true, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);
			AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);

			sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector and V66, target load, desired orientation (DOI trim)");
			}
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
			sprintf(upMessage, "LOI scrubbed, burn PC+2");
			scrubbed = true;
			mcc->AbortMode = 6;
		}
	}
	break;
	case 34: //LS REFSMMAT UPLINK (post DOI trim)
	{
		MATRIX3 REFSMMAT;
		REFSMMATOpt opt;

		char buffer1[1000];

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
		GENERICPAD * form = (GENERICPAD *)pad;

		SV svcm0, svcm1, svlm0, svlm1;
		double t_sunrise, t_TPI, TimeToPDI;

		PDAPOpt opt;
		PDAPResults res;

		LEM *l = (LEM*)calcParams.tgt;

		svcm0 = StateVectorCalc(calcParams.src);
		svlm0 = StateVectorCalc(calcParams.tgt);

		TimeToPDI = OrbMech::GETfromMJD(svlm0.MJD, CalcGETBase()) - calcParams.PDI;

		if (abs(TimeToPDI) < 2.0*3600.0) //PDI-2
		{
			svcm1 = svcm0;
			svlm1 = svlm0;

			opt.dt_CAN = 60.0*60.0;
			opt.dt_CSI = 180.0*60.0;
			opt.dt_2CSI = -125.0*60.0;
			opt.dt_2TPI = -7117.0;
			opt.h_amin = 127.8*1852.0;
			t_sunrise = calcParams.PDI + 5.0*3600.0;
			opt.LongProfileFirst = true;
			opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		}
		else
		{
			svcm1 = ExecuteManeuver(svcm0, TimeofIgnition, DeltaV_LVLH, 0.0, RTCC_ENGINETYPE_CSMSPS);
			svlm1 = svlm0;

			opt.dt_CAN = 60.0*60.0;
			opt.dt_CSI = 180.0*60.0;
			opt.dt_2CSI = -125.0*60.0;
			opt.dt_2TPI = -7117.0;
			opt.h_amin = 128.5*1852.0;
			t_sunrise = calcParams.PDI + 5.0*3600.0;
			opt.LongProfileFirst = true;
			opt.REFSMMAT = EZJGMTX3.data[RTCC_REFSMMAT_TYPE_LLD - 1].REFSMMAT;
		}

		t_TPI = FindOrbitalSunrise(svcm1, t_sunrise) - 16.0*60.0;

		opt.W_TAPS = l->GetAscentStageMass();
		opt.W_TDRY = opt.sv_A.mass - calcParams.tgt->GetPropellantMass(calcParams.tgt->GetPropellantHandleByIndex(0));
		opt.dt_stage = 999999.9;
		opt.IsTwoSegment = true;
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		opt.sv_A = svlm1;
		opt.sv_P = svcm1;
		opt.TLAND = CZTDTGTU.GETTD;
		opt.t_TPI = t_TPI;
		opt.dt_step = 20.0;

		PoweredDescentAbortProgram(opt, res);

		int emem[14], DEDA224, DEDA225, DEDA226, DEDA305, DEDA662, DEDA673;
		char buffer1[1000];

		DEDA662 = DEDA673 = 0;
		if (res.K1 < 0.0)
			DEDA662 = -77777;
		if (res.K2 < 0.0)
			DEDA673 = -77777;

		if (res.Theta_LIM < 0.0)
		{
			DEDA224 = (int)(res.J2 / 0.3048 / 100.0);
			DEDA226 = (int)(res.J1 / 0.3048 / 100.0);
		}
		else
		{
			DEDA224 = (int)(res.J1 / 0.3048 / 100.0);
			DEDA226 = (int)(res.J2 / 0.3048 / 100.0);
		}

		DEDA225 = (int)(res.R_amin / 2.0 / 0.3048 / 100.0);
		DEDA305 = (int)((res.Theta_LIM * DEG) * 100.0);
		DEDA662 += OrbMech::decimal_octal((int)(abs(res.K1 / 0.3048) * 2.0)) / 100;
		DEDA673 += OrbMech::decimal_octal((int)(abs(res.K2 / 0.3048) * 2.0)) / 100;

		sprintf(form->paddata, "AGS ABORT CONSTANTS\n224 %+06d\n225 %+06d\n226 %+06d\n305 %+06d\n662 %+06d\n673 %+06d", DEDA224, DEDA225, DEDA226, DEDA305, DEDA662, DEDA673);

		//Update 254 & 373 for PDI-2
		if (abs(TimeToPDI) < 2.0*3600.0)
		{
			double AGSTIG = calcParams.PDI - GETfromGMT(GetAGSClockZero());
			double AGST2 = calcParams.PDI + 22.0*60.0 + 19.0 - GETfromGMT(GetAGSClockZero());
			sprintf(form->paddata, "%s\n254 %+06d\n373 %+06d", form->paddata, (int)(AGST2 / 60.0 * 10.0), (int)(AGSTIG / 60.0 * 10.0));
		}

		emem[0] = 16;
		emem[1] = 2545;
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
		}
	}
	break;
	case 37: //UNDOCK & SEPARATION MANEUVER
	{
		AP11ManPADOpt opt;
		VECTOR3 dV_LVLH;

		AP11MNV * form = (AP11MNV *)pad;

		calcParams.SEP = CZTDTGTU.GETTD - 4.45 * 3600.0;
		dV_LVLH = _V(0, 0, -1)*0.3048;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSPLUS4;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.TIG = calcParams.SEP;
		opt.vessel = calcParams.src;
		opt.vesseltype = 0;

		AP11ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "CSM SEP");
		form->type = 2;
	}
	break;
	case 38: //PDI-0 PAD
	{
		AP11LMManPADOpt opt;
		SV svlm, svcsm0, svcsm1;
		VECTOR3 dV_LVLH;
		double P30TIG, t_sunrise, t_CSI, t_TPI, DH, TIG, dvx;

		AP11LMMNV * form = (AP11LMMNV *)pad;

		svcsm0 = StateVectorCalc(calcParams.src);
		svlm = StateVectorCalc(calcParams.tgt);

		//Find PDI-0 TIG (perilune GET)
		double SV_GET = OrbMech::GETfromMJD(svlm.MJD, CalcGETBase());
		//t_P = OrbMech::period(sv1.R, sv1.V, OrbMech::mu_Moon);
		SV sv2 = coast(svlm, 1.0*3600.0);
		double dt = OrbMech::timetoperi(sv2.R, sv2.V, OrbMech::mu_Moon);
		SV sv_peri = coast(sv2, dt);
		double Peri_GET = OrbMech::GETfromMJD(sv_peri.MJD, CalcGETBase());

		TIG = Peri_GET;
		dvx = 100.0;

		svcsm1 = ExecuteManeuver(svcsm0, TimeofIgnition, DeltaV_LVLH, 0, RTCC_ENGINETYPE_CSMSPS);

		t_sunrise = calcParams.PDI + 0.8*3600.0;
		t_TPI = FindOrbitalSunrise(svcsm1, t_sunrise) - 16.0*60.0;
		//Round to next 30 seconds
		t_TPI = round(t_TPI / 30.0)*30.0;

		SPQOpt coeopt;
		SPQResults coeres;

		coeopt.DH = 15.0*1852.0;
		coeopt.E = 26.6*RAD;
		coeopt.sv_P = svcsm1;
		coeopt.K_CDH = 0;
		coeopt.t_CSI = TIG + 60.0*60.0;
		coeopt.t_TPI = t_TPI;

		int n = 0, nmax = 100;

		//Iterate to find DVX for PDI-0
		coeopt.sv_A = ExecuteManeuver(svlm, TIG, _V(dvx, 0.0, 0.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
		ConcentricRendezvousProcessor(coeopt, coeres);
		DH = coeopt.DH - coeres.DH;
		while (abs(DH) > 0.1*1852.0 && n < nmax)
		{
			dvx += DH / (abs(DH) * 10.0);

			coeopt.sv_A = ExecuteManeuver(svlm, TIG, _V(dvx, 0.0, 0.0)*0.3048, 0, RTCC_ENGINETYPE_LMDPS);
			ConcentricRendezvousProcessor(coeopt, coeres);
			DH = coeopt.DH - coeres.DH;

			n++;
		}

		PoweredFlightProcessor(svlm, TIG, RTCC_ENGINETYPE_LMDPS, 0.0, _V(dvx, 0.0, 0.0)*0.3048, true, P30TIG, dV_LVLH);
		t_CSI = coeres.t_CSI;

		calcParams.Phasing = P30TIG;

		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_LMDPS;
		opt.HeadsUp = false;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.TIG = P30TIG;
		opt.vessel = calcParams.tgt;
		opt.csmlmdocked = false;
		opt.useSV = true;
		opt.RV_MCC = svlm;

		double AGSTIG = P30TIG - GETfromGMT(GetAGSClockZero());

		AP11LMManeuverPAD(&opt, *form);
		sprintf(form->purpose, "PDI-0 ABORT");
		sprintf(form->remarks, "\n%+06d TIG 373", (int)(AGSTIG / 60.0 * 10.0));
		form->type = 1;
		form->t_CSI = round(t_CSI);
		form->t_TPI = round(t_TPI);
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
			orbopt.TIG_GET = calcParams.PDI - 3.5*3600.0;

			//Circularize at 60 NM
			orbopt.ManeuverCode = RTCC_GMP_CRH;
			orbopt.H_D = 60.0*1852.0;

			GeneralManeuverProcessor(&orbopt, dV_imp, TIG_imp);

			in.CONFIG = 1; //CSM
			in.CSMWeight = calcParams.src->GetMass();
			in.sv_before = PZGPMELM.SV_before;
			in.V_aft = PZGPMELM.V_after;
			in.DETU = 14.0; //Ullage
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

			//To fix issue that seems to be caused by circular orbit
			DeltaV_LVLH.x += 0.3;
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
			manopt.sxtstardtime = -40.0*60.0;
			manopt.TIG = TimeofIgnition;
			manopt.vessel = calcParams.src;
			manopt.vesseltype = 0;

			AP11ManeuverPAD(&manopt, *form);
			sprintf(form->purpose, "CIRC");
			sprintf(form->remarks, "Four-jet ullage for 14 seconds");

			AGCStateVectorUpdate(buffer1, sv, true);
			CMCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector, target load");
			}
		}
	}
	break;
	case 51: //TEI UPDATE (PRELIM)
	case 52: //TEI UPDATE (FINAL)
	case 53: //TEI UPDATE (NEXT REV)
	{
		AP11ManPADOpt opt;
		double GET_SV1, AbortGuess;
		SV sv1;
		char manname[8];
		EphemerisData sv_e;

		AP11MNV * form = (AP11MNV *)pad;

		sv1 = StateVectorCalc(calcParams.src); //State vector for uplink

		GMGMED("F79,0;"); //Clear abort scan table

		GET_SV1 = OrbMech::GETfromMJD(sv1.MJD, CalcGETBase());

		sv_e.GMT = GMTfromGET(GET_SV1);
		sv_e.R = sv1.R;
		sv_e.V = sv1.V;
		sv_e.RBI = BODY_MOON;

		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		med_f75_f77.Inclination = 0.0;

		std::string site = med_f77.Site;

		site = PZREAP.PTPSite[0];

		if (fcn == 51 || fcn == 52)
		{
			sprintf(manname, "TEI");
			AbortGuess = calcParams.TEI;
		}
		else if (fcn == 53)
		{
			sprintf(manname, "TEI-%d", mcc->MoonRev + 1);
			AbortGuess = calcParams.TEI + 2.0*3600.0;
		}

		VEHDATABUF.csmmass = calcParams.src->GetMass();
		VEHDATABUF.lmascmass = 0.0;
		VEHDATABUF.lmdscmass = 0.0;
		VEHDATABUF.sv = sv_e;
		VEHDATABUF.config = "C";

		med_f75_f77.T_0_min = AbortGuess;
		med_f75_f77.T_Z = calcParams.EI;

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
		med_f80.UllageDT = 13.0;

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

		sprintf(form->remarks, "Four-jet ullage for 13 seconds");

		if (fcn == 51 || fcn == 52)
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
			CMCRetrofireExternalDeltaVUpdate(buffer2, SplashLatitude, SplashLongitude, TimeofIgnition, DeltaV_LVLH);

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

		t_lng = OrbMech::P29TimeOfLongitude(SystemParameters.MAT_J2000_BRCS, sv.R, sv.V, sv.MJD, GetGravref(BODY_MOON), BZLAND.lng[RTCC_LMPOS_BEST]);
		ttoLNG = (t_lng - sv.MJD)*24.0 * 3600.0;
		GET = (sv.MJD - CalcGETBase())*24.0*3600.0 + ttoLNG;
		OrbMech::SStoHHMMSS(GET, hh, mm, ss);

		sprintf(form->paddata, "REV 12 LS TCA: %d:%d:%.2lf", hh, mm, ss);
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
		dt2 = OrbMech::findelev_gs(SystemParameters.AGCEpoch, SystemParameters.MAT_J2000_BRCS, sv1.R, sv1.V, R_P, MJDguess, 152.0*RAD, sv1.gravref, LmkRange);

		form->P22_ACQ_GET = dt2 + (MJDguess - CalcGETBase()) * 24.0 * 60.0 * 60.0;
	}
	break;
	case 62: //LANDMARK TRACKING PAD REV 12 (XX-1)
	case 63: //LANDMARK TRACKING PAD REV 13 (XX-1)
	{
		LMARKTRKPADOpt opt;
		SV sv;

		AP11LMARKTRKPAD * form = (AP11LMARKTRKPAD *)pad;

		sv = StateVectorCalc(calcParams.src);

		if (fcn == 62)
		{
			opt.LmkTime[0] = calcParams.PDI - 4.0*3600.0;
		}
		else
		{
			opt.LmkTime[0] = calcParams.PDI - 2.0*3600.0;
		}

		sprintf(form->LmkID[0], "XX-1");
		opt.sv0 = sv;
		opt.alt[0] = BZLAND.rad[RTCC_LMPOS_BEST] - OrbMech::R_Moon;
		opt.lat[0] = BZLAND.lat[RTCC_LMPOS_BEST];
		opt.lng[0] = BZLAND.lng[RTCC_LMPOS_BEST];
		opt.entries = 1;

		LandmarkTrackingPAD(&opt, *form);
	}
	break;
	case 70: //PDI PAD
	{
		AP11PDIPAD * form = (AP11PDIPAD *)pad;

		PDIPADOpt opt;
		SV sv;

		sv = StateVectorCalc(calcParams.tgt);

		opt.direct = true;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		opt.sv0 = sv;
		opt.t_land = CZTDTGTU.GETTD;

		PDI_PAD(&opt, *form);

		calcParams.PDI = form->GETI;
	}
	break;
	case 71: //PDI ABORT PAD
	{
		AP12PDIABORTPAD * form = (AP12PDIABORTPAD *)pad;

		SV sv;
		double t_sunrise1, t_sunrise2, t_TPI;

		sv = StateVectorCalc(calcParams.src);

		t_sunrise1 = calcParams.PDI + 3.0*3600.0;
		t_sunrise2 = calcParams.PDI + 5.0*3600.0;

		t_TPI = FindOrbitalSunrise(sv, t_sunrise2) - 16.0*60.0;
		form->T_TPI_Pre10Min = round(t_TPI);
		t_TPI = FindOrbitalSunrise(sv, t_sunrise1) - 16.0*60.0;
		form->T_TPI_Post10Min = round(t_TPI);
	}
	break;
	case 72: //No PDI+12 PAD
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
		sprintf(form->remarks, "\n%+06d TIG 373", (int)(AGSTIG / 60.0 * 10.0));
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

		v_LH = 5515.8*0.3048;
		v_LV = 19.5*0.3048;

		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		if (calcParams.CSI < 100.0) //PDI-1
		{
			sv_CSM = sv0;
			T2 = calcParams.PDI + 21.0*60.0 + 26.0;
			t_sunrise = calcParams.PDI + 5.0*3600.0;
		}
		else //PDI-2
		{
			sv_CSM = sv0;
			T2 = calcParams.PDI + 19.0*60.0 + 40.0;
			t_sunrise = calcParams.PDI + 3.0*3600.0;
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
		t_TPI = FindOrbitalSunrise(sv_CSM, t_sunrise) - 16.0*60.0;
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
		MJD_over = OrbMech::P29TimeOfLongitude(SystemParameters.MAT_J2000_BRCS, sv_CSM2.R, sv_CSM2.V, sv_CSM2.MJD, sv_CSM2.gravref, BZLAND.lng[RTCC_LMPOS_BEST]);
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
		int r = rand() % 20;

		if (r < 5 && calcParams.CSI < 100.0 && calcParams.tgt->GetDamageModel() > 0)
		{
			sprintf(upMessage, "NO-GO for PDI");
		}
		else
		{
			sprintf(upMessage, "GO for PDI");
		}
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
			calcParams.CSI = 9999.0; //Use as flag for PDI-2
		}
	}
	break;
	case 79: //LANDING EVALUATION
	{
		sprintf(upMessage, "We copy you down!");
	}
	break;
	case 80: //STAY FOR T1
	{
		sprintf(upMessage, "STAY for T1");
	}
	break;
	case 81: //STAY FOR T2
	{
		sprintf(upMessage, "STAY for T2");
	}
	break;
	case 82: //STAY FOR T3
	{
		sprintf(upMessage, "STAY for T3");

		//Update landing site
		calcParams.tgt->GetEquPos(BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
	}
	break;
	case 94: //PLANE CHANGE EVALUATION
	{
		SV sv_CSM, sv_Liftoff;
		VECTOR3 R_LS;
		double TIG_nom, MJD_TIG_nom, dt1, LmkRange;

		sv_CSM = StateVectorCalc(calcParams.src);

		//Initial guess for liftoff time
		TIG_nom = calcParams.LunarLiftoff;
		MJD_TIG_nom = OrbMech::MJDfromGET(TIG_nom, CalcGETBase());
		sv_Liftoff = coast(sv_CSM, (MJD_TIG_nom - sv_CSM.MJD)*24.0*3600.0);

		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		dt1 = OrbMech::findelev_gs(SystemParameters.AGCEpoch, SystemParameters.MAT_J2000_BRCS, sv_Liftoff.R, sv_Liftoff.V, R_LS, MJD_TIG_nom, 180.0*RAD, sv_Liftoff.gravref, LmkRange);

		if (abs(LmkRange) < 8.0*1852.0)
		{
			sprintf(upMessage, "Plane Change has been scrubbed");
			scrubbed = true;
		}
	}
	break;
	case 95: //PLANE CHANGE UPDATE
	{
		SV sv;

		sv = StateVectorCalc(calcParams.src);

		med_k16.GETTH1 = calcParams.TPI - 0.5*3600.0;
		med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.LunarLiftoff - 0.5*3600.0;
		med_k16.Mode = 7;
		med_k16.Sequence = 1;
		med_k16.Vehicle = RTCC_MPT_CSM;

		GZGENCSN.LDPPAzimuth = 0.0;

		LunarDescentPlanningProcessor(sv);

		PoweredFlightProcessor(sv, PZLDPDIS.GETIG[0], RTCC_ENGINETYPE_CSMSPS, 0.0, PZLDPDIS.DVVector[0] * 0.3048, true, TimeofIgnition, DeltaV_LVLH);

		AP11MNV * form = (AP11MNV*)pad;

		AP11ManPADOpt manopt;

		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;

		char buffer1[1000], buffer2[1000], updesc[128];

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
		sprintf(updesc, "Target load, PC REFSMMAT");

		CMCExternalDeltaVUpdate(buffer1, TimeofIgnition, DeltaV_LVLH);
		AGCDesiredREFSMMATUpdate(buffer2, REFSMMAT);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);

		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, updesc);
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

		//Also update AGS time here
		char Buff[128];

		double t_AGS = round((calcParams.LunarLiftoff - 3600.0 - 18000.0) / 36000.0)*36000.0;

		OrbMech::SStoHHMMSS(t_AGS, hh, mm, ss);

		sprintf_s(Buff, "P15,AGS,,%03d:00:00;", hh);

		GMGMED(Buff);
	}
	break;
	case 99: //LIFTOFF TIME RE-CALCULATION (AFTER LAUNCH SCRUB)
	{
		calcParams.LunarLiftoff = calcParams.Insertion + 1.0*3600.0 + 40.0*60.0;
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
		opt.T_TH = GMTfromGET(t_TPI - 85.0*60.0);
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
		PZLTRT.DT_Ins_TPI = opt.dt_INS_TPI;

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
		sv_LM_upl = coast(calcParams.SVSTORE1, calcParams.Insertion + 18.0*60.0 - OrbMech::GETfromMJD(calcParams.SVSTORE1.MJD, CalcGETBase()));

		AGCStateVectorUpdate(buffer1, sv_CSM_upl, true);
		AGCStateVectorUpdate(buffer2, sv_LM_upl, false);

		if (fcn == 102)
		{
			sprintf(uplinkdata, "%s%sV45ER", buffer1, buffer2);
			if (upString != NULL) {
				// give to mcc
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM and LM state vectors, Reset surface flag");
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

		ASCPADOpt opt;
		double m0, m1;

		//Definitions
		opt.TIG = TimeofIgnition;
		opt.v_LH = DeltaV_LVLH.x;
		opt.v_LV = DeltaV_LVLH.y;

		opt.sv_CSM = StateVectorCalcEphem(calcParams.src);
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);

		LEM *l = (LEM*)calcParams.tgt;
		m0 = l->GetAscentStageMass();
		m1 = calcParams.src->GetMass();

		opt.Rot_VL = OrbMech::GetVesselToLocalRotMatrix(calcParams.tgt);

		// Calculate ascent PAD
		double AGSTPI, apo, peri;

		AGSTPI = calcParams.TIGSTORE1 - GETfromGMT(GetAGSClockZero());

		SV sv_CSM = StateVectorCalc(calcParams.src);
		OrbMech::periapo(sv_CSM.R, sv_CSM.V, OrbMech::mu_Moon, apo, peri);

		AP11LMASCPAD pad2;

		LunarAscentPAD(opt, pad2);

		form->CR = pad2.CR;
		form->TIG = pad2.TIG;
		form->V_hor = pad2.V_hor;
		form->V_vert = form->DEDA465 = pad2.V_vert;
		form->DEDA047 = pad2.DEDA047;
		form->DEDA053 = pad2.DEDA053;
		form->DEDA225_226 = pad2.DEDA225_226;
		form->DEDA231 = pad2.DEDA231;
		form->TIG_2 = calcParams.TIGSTORE1;
		form->LMWeight = m0 / 0.45359237;
		form->CSMWeight = m1 / 0.45359237;
		sprintf(form->remarks, "\n%+06d 373 TPI\n%+05.1f CSM HA\n%+05.1f CSM HP", (int)(AGSTPI / 60.0 * 10.0), (apo - OrbMech::R_Moon) / 1852.0, (peri - OrbMech::R_Moon) / 1852.0);
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
	case 110: //SEP BURN & LM JETT PAD + CMC SV UPLINK
	{
		AP11ManPADOpt opt;
		SV sv_CSM, sv_LM;
		VECTOR3 dV_LVLH;
		char buffer1[100], buffer2[100];

		int hh, mm;
		double ss;

		AP11MNV * form = (AP11MNV *)pad;

		dV_LVLH = _V(0, 0, -1.0)*0.3048; //Use this initially to find jettison pitch attitude (radial up)
		opt.R_LLS = BZLAND.rad[RTCC_LMPOS_BEST];
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSPLUS4;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.TIG = calcParams.SEP - 5.0*60.0;
		opt.vessel = calcParams.src;
		opt.vesseltype = 0;

		AP11ManeuverPAD(&opt, *form);
		sprintf(form->purpose, "SEP BURN");

		OrbMech::SStoHHMMSS(form->GETI, hh, mm, ss);
		sprintf(form->remarks, "Jettison GET: %d:%d:%.2lf", hh, mm, ss);

		form->GETI = calcParams.SEP;
		form->dV = _V(-1.0, 0, 0);
		//form->Att = _V(014.0, form->Att.y, 343.0); //Apollo 15 jettison R and Y

		form->type = 2;

		sv_CSM = StateVectorCalc(calcParams.src);
		sv_LM = StateVectorCalc(calcParams.tgt);

		AGCStateVectorUpdate(buffer1, sv_CSM, true);
		AGCStateVectorUpdate(buffer2, sv_LM, false);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM and LM state vectors");
		}
	}
	break;
	case 111: //LM IMPACT BURN PAD + UPLINK
	{
		AP11LMManPADOpt opt;

		SV sv0, sv_upl;
		char buffer1[1000];
		char buffer2[1000];

		AP11LMMNV * form = (AP11LMMNV *)pad;

		sv0 = StateVectorCalc(calcParams.tgt);

		//Find time of SEP
		calcParams.SEP = calcParams.TPI + 4.0*3600.0 + 56.0*60.0;

		//Find impact burn TIG
		TimeofIgnition = calcParams.TPI + 6.0*3600.0 + 37.0*60.0;

		DeltaV_LVLH = _V(-180.0, 0.0, 0.0)*0.3048;
		//DeltaV_LVLH = _V(-161.1, 57.3, 94.6)*0.3048; //Apollo 15 deorbit burn

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

		double AGSTIG = form->GETI - GETfromGMT(GetAGSClockZero());
		sprintf(form->remarks, "\n%+06d TIG 373", (int)(AGSTIG / 60.0 * 10.0));

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
	case 115: //TLI GO/NO-GO
	{
		int r = rand() % 20;

		if (r < 5 && calcParams.CSI < 100.0 && calcParams.src->GetDamageModel() > 0)
		{
			sprintf(upMessage, "You are NO-GO for TLI");
		}
		else
		{
			sprintf(upMessage, "You are GO for TLI");
		}
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
	case 130: //TEI MAP UPDATE
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
	case 138: //Pre PDI-0 SV uplinks
	{
		SV sv_CSM1, sv_LM1, sv_CSM2, sv_LM2;
		char buffer1[1000];
		char buffer2[1000];

		//State vectors
		sv_CSM1 = StateVectorCalc(calcParams.src);
		sv_LM1 = StateVectorCalc(calcParams.tgt);

		double SV_GET = OrbMech::GETfromMJD(sv_LM1.MJD, CalcGETBase());

		sv_CSM2 = coast(sv_CSM1, calcParams.Phasing - SV_GET);
		sv_LM2 = coast(sv_LM1, calcParams.Phasing - SV_GET);

		AGCStateVectorUpdate(buffer1, sv_CSM2, true);
		AGCStateVectorUpdate(buffer2, sv_LM2, false);

		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "CSM SV (P24 T2 ACQ), LM SV (P24 T2 ACQ)");
		}
	}
	break;
	case 141: //Liftoff Time Evaluation (TLC)
	{
		double dt = CZTDTGTU.GETTD - (GETfromGMT(PZSFPTAB.blocks[1].GMT_pc2) + PZSFPTAB.blocks[1].dt_lls);

		if (abs(dt) < 60.0)
		{
			scrubbed = true;
		}

		if (!scrubbed)
		{
			int hh, mm;
			double ss;

			calcParams.LOI += dt;
			calcParams.TEI += dt;
			calcParams.EI += dt;
			TimeofIgnition += dt;

			calcParams.Phasing = dt;

			char sign[8];

			if (dt >= 0.0)
			{
				sprintf(sign, "+");
			}
			else
			{
				sprintf(sign, "-");
			}

			OrbMech::SStoHHMMSS(abs(dt), hh, mm, ss);

			sprintf(upMessage, "Liftoff time update required\nAdjust GET by: %s%03d:%02d:%0.2lf", sign, hh, mm, ss);
		}

		calcParams.TIGSTORE1 = CZTDTGTU.GETTD;
	}
	break;
	case 142: //Liftoff time Evaluation REV 14
	{
		double dt = calcParams.TIGSTORE1 - CZTDTGTU.GETTD;

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
			calcParams.TPI += dt;
			calcParams.LunarLiftoff += dt;
			calcParams.TEI += dt;
			calcParams.EI += dt;
			TimeofIgnition += dt;

			calcParams.Phasing = dt;

			char sign[8];

			if (dt >= 0.0)
			{
				sprintf(sign, "+");
			}
			else
			{
				sprintf(sign, "-");
			}

			OrbMech::SStoHHMMSS(abs(dt), hh, mm, ss);

			sprintf(upMessage, "Liftoff time update required\nAdjust GET by: %s%03d:%02d:%0.2lf", sign, hh, mm, ss);
		}
	}
	break;
	case 144: //CMC Liftoff Time Update
	{
		char buffer[128];

		IncrementAGCLiftoffTime(buffer, RTCC_MPT_CSM, -calcParams.Phasing);

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

		//P15: CMC, LGC
		sprintf_s(Buff, "P15,AGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);
		sprintf_s(Buff, "P15,LGC,%d:%d:%.2lf;", hh, mm, ss);
		GMGMED(Buff);

		//AGS clock zero
		double t_AGS = round((CZTDTGTU.GETTD - 18000.0) / 36000.0)*36000.0;
		OrbMech::SStoHHMMSS(t_AGS, hh, mm, ss);
		sprintf_s(Buff, "P15,AGS,,%03d:00:00;", hh);
		GMGMED(Buff);
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
			calcParams.Phasing = dt;
		}
	}
	break;
	case 148: //CMC clock sync uplink
	case 149: //LGC clock sync uplink
	{
		char buffer[128];

		if (fcn == 148)
		{
			IncrementAGCTime(buffer, RTCC_MPT_CSM, calcParams.Phasing);
		}
		else
		{
			IncrementAGCTime(buffer, RTCC_MPT_LM, calcParams.Phasing);
		}

		sprintf(uplinkdata, "%s", buffer);
		if (upString != NULL) {
			// give to mcc
			strncpy(upString, uplinkdata, 1024 * 3);
			sprintf(upDesc, "Clock sync");
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
			calcParams.TEI = TimeofIgnition + 2.0*3600.0;
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
			MCCtime = calcParams.TEI + 15.0*3600.0;
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

		entopt.entrylongmanual = true;
		entopt.lng = PZREAP.PTPLongitude[0]; //EOM

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