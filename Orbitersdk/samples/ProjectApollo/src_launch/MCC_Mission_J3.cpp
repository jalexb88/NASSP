/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2023

MCC sequencing for Mission J3

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
#include "saturn.h"
#include "sivb.h"
#include "mcc.h"
#include "rtcc.h"
#include "MCC_Mission_J3.h"
#include "iu.h"
#include "nassputils.h"

using namespace nassp;

void MCC::MissionSequence_J3()
{
	switch (MissionState)
	{
	case MST_J3_INSERTION: //Ground liftoff time update to TLI Simulation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(1.0*3600.0 + 30.0*60.0), 10, MST_J3_EPO1);
		break;
	case MST_J3_EPO1: //TLI Simulation to TLI+90 PAD
		UpdateMacro(UTP_NONE, PT_NONE, true, 11, MST_J3_EPO2);
		break;
	case MST_J3_EPO2: //TLI+90 Maneuver PAD to L/O+9 P37 PAD
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 12, MST_J3_EPO3);
		break;
	case MST_J3_EPO3: //L/O+9 P37 PAD to TLI PAD
		UpdateMacro(UTP_PADONLY, PT_P37PAD, rtcc->GETEval2(rtcc->calcParams.TLI - 35.0*60.0), 13, MST_J3_EPO4);
		break;
	case MST_J3_EPO4: //TLI PAD to SV Update
		UpdateMacro(UTP_PADONLY, PT_TLIPAD, SubStateTime > 5.0*60.0, 14, MST_J3_EPO5);
		break;
	case MST_J3_EPO5: //SV Update to TLI Evaluation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TLI + 18.0), 5, MST_J3_TRANSLUNAR1);
		break;
	case MST_J3_TRANSLUNAR1: //TLI Evaluation to SIVB Evasive Maneuver
		UpdateMacro(UTP_NONE, PT_NONE, true, 15, MST_J3_TRANSLUNAR2, scrubbed, rtcc->GETEval2(rtcc->calcParams.TLI + 30.0*60.0), MST_J3_EPO1);
		break;
	case MST_J3_TRANSLUNAR2:
		switch (SubState) {
		case 0:
		{
			addMessage("TLI");
			MissionPhase = MMST_TL_COAST;
			setSubState(1);
		}
		break;
		case 1:
		{
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 3600.0 + 51.0*60.0))
			{
				SlowIfDesired();
				setState(MST_J3_TRANSLUNAR3);
			}
		}
		break;
		}
		break;
	case MST_J3_TRANSLUNAR3: //SIVB Evasive Maneuver to TB8 Enable
		switch (SubState) {
		case 0:
		{
			if (cm->GetStage() >= CSM_LEM_STAGE)
			{
				setSubState(1);
			}
		}
		break;
		case 1:
		{
			if (SubStateTime > 2.0*60.0)
			{
				setSubState(2);
			}
		}
		break;
		case 2:
		{
			if (sivb == NULL)
			{
				VESSEL *v;
				OBJHANDLE hLV;
				hLV = oapiGetObjectByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);

					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB *)v;
					}
				}
			}

			sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_EVASIVE_MANEUVER_ENABLE, NULL);
			setSubState(3);
		}
		break;
		case 3:
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 2.0*3600.0 + 1.0*60.0))
			{
				SlowIfDesired();
				setState(MST_J3_TRANSLUNAR4);
			}
			break;
		}
		break;
	case MST_J3_TRANSLUNAR4:  //TB8 enable to SIVB MCC-1
		switch (SubState) {
		case 0:
		{
			if (cm->GetStage() >= CSM_LEM_STAGE)
			{
				setSubState(1);
			}
		}
		break;
		case 1:
		{
			if (SubStateTime > 2.0*60.0)
			{
				setSubState(2);
			}
		}
		break;
		case 2:
		{
			if (sivb == NULL)
			{
				VESSEL *v;
				OBJHANDLE hLV;
				hLV = oapiGetObjectByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);

					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB *)v;
					}
				}
			}

			sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_TIMEBASE_8_ENABLE, NULL);
			setSubState(3);
		}
		break;
		case 3:
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 2.0*3600.0 + 53.0*60.0))
			{
				SlowIfDesired();
				setState(MST_J3_TRANSLUNAR5);
			}
			break;
		}
		break;
	case MST_J3_TRANSLUNAR5:  //SIVB MCC-1 to PTC REFSMMAT update
		switch (SubState) {
		case 0:
		{
			if (cm->GetStage() >= CSM_LEM_STAGE)
			{
				setSubState(1);
			}
		}
		break;
		case 1:
		{
			if (SubStateTime > 2.0*60.0)
			{
				setSubState(2);
			}
		}
		break;
		case 2:
		{
			/*if (sivb == NULL)
			{
				VESSEL *v;
				OBJHANDLE hLV;
				hLV = oapiGetObjectByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);

					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB *)v;
					}
				}
			}

			sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_TIMEBASE_8_ENABLE, NULL);*/
			setSubState(3);
		}
		break;
		case 3:
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 4.0*3600.0 + 30.0*60.0))
			{
				SlowIfDesired();
				setState(MST_J3_TRANSLUNAR6);
			}
			break;
		}
		break;
	case MST_J3_TRANSLUNAR6: //MCC-1 Evaluation to MCC-1 Update
		UpdateMacro(UTP_NONE, PT_NONE, true, 19, MST_J3_TRANSLUNAR7, scrubbed, true, MST_J3_TRANSLUNAR8);
		break;
	case MST_J3_TRANSLUNAR7: //MCC-1 update to PTC REFSMMAT
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->TimeofIgnition + 5.0*60.0), 21, MST_J3_TRANSLUNAR8);
		break;
	case MST_J3_TRANSLUNAR8: //PTC REFSMMAT update to Block Data 1
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TLI + 4.0*3600.0 + 55.0*60.0), 18, MST_J3_TRANSLUNAR9);
		break;
	case MST_J3_TRANSLUNAR9: //Block Data 1 to Pericynthion Evaluation
		UpdateMacro(UTP_PADONLY, PT_P37PAD, rtcc->GETEval2(rtcc->calcParams.TLI + 6.0*3600.0), 16, MST_J3_TRANSLUNAR10);
		break;
	case MST_J3_TRANSLUNAR10: //Pericynthion Evaluation to Early flyby PAD or MCC-2 Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, true, 140, MST_J3_TRANSLUNAR11, scrubbed, rtcc->GETEval2(rtcc->calcParams.TLI + 13.0*3600.0 + 10.0*60.0),  MST_J3_TRANSLUNAR12);
		break;
	case MST_J3_TRANSLUNAR11: //Early flyby PAD to MCC-2 Evaluation
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.TLI + 13.0*3600.0 + 10.0*60.0), 23, MST_J3_TRANSLUNAR12);
		break;
	case MST_J3_TRANSLUNAR12: //MCC-2 Evaluation to Block Data 2
		UpdateMacro(UTP_NONE, PT_NONE, true, 20, MST_J3_TRANSLUNAR13);
		break;
	case MST_J3_TRANSLUNAR13: //Block Data 2 to MCC-2 update
		UpdateMacro(UTP_PADONLY, PT_P37PAD, rtcc->GETEval2(rtcc->calcParams.TLI + 31.0*3600.0), 17, MST_J3_TRANSLUNAR14);
		break;
	case MST_J3_TRANSLUNAR14: //MCC-2 update to Lunar Flyby PAD
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.TLI + 37.0*3600.0 + 30.0*60.0), 22, MST_J3_TRANSLUNAR15);
		break;
	case MST_J3_TRANSLUNAR15: //Lunar Flyby PAD to MCC-3 Update
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 23.5*3600.0), 23, MST_J3_TRANSLUNAR16);
		break;
	case MST_J3_TRANSLUNAR16: //MCC-3 Update to Liftoff time Evaluation
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 21.5*3600.0), 24, MST_J3_TRANSLUNAR17);
		break;
	case MST_J3_TRANSLUNAR17: //Liftoff time Evaluation to CMC Liftoff time Update or MCC-4 Update
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 141, MST_J3_TRANSLUNAR18, scrubbed, rtcc->GETEval2(rtcc->calcParams.LOI - 6.5*3600.0), MST_J3_TRANSLUNAR20);
		break;
	case MST_J3_TRANSLUNAR18: //CMC Liftoff time Update to RTCC Liftoff time update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 144, MST_J3_TRANSLUNAR19);
		break;
	case MST_J3_TRANSLUNAR19: //RTCC Liftoff time update to MCC-4 Update
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LOI - 6.5*3600.0), 145, MST_J3_TRANSLUNAR20);
		break;
	case MST_J3_TRANSLUNAR20: //MCC-4 update to PC+2 update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 25, MST_J3_TRANSLUNAR21);
		break;
	case MST_J3_TRANSLUNAR21: //PC+2 update to LOI update (preliminary)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 4.0*3600.0), 26, MST_J3_TRANSLUNAR22);
		break;
	case MST_J3_TRANSLUNAR22: //LOI update (preliminary) to TEI-4 update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 27, MST_J3_TRANSLUNAR23);
		break;
	case MST_J3_TRANSLUNAR23: //TEI-4 update to LOI update (final)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 1.0*3600.0 - 15.0*60.0), 41, MST_J3_TRANSLUNAR24);
		break;
	case MST_J3_TRANSLUNAR24: //LOI update (final) to Rev 1 Map Update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 28, MST_J3_TRANSLUNAR25);
		break;
	case MST_J3_TRANSLUNAR25: //Rev 1 Map Update to LOI Evaluation
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, rtcc->GETEval2(rtcc->TimeofIgnition + 7.0*60.0), 128, MST_J3_TRANSLUNAR26);
		break;
	case MST_J3_TRANSLUNAR26: //LOI Evaluation to Lunar orbit phase begin or PC+2
		UpdateMacro(UTP_NONE, PT_NONE, true, 33, MST_J3_LUNAR_ORBIT_LOI_DAY_1, scrubbed, true, MST_J3_ABORT);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_1: //Lunar orbit phase begin
		switch (SubState) {
		case 0:
			MissionPhase = MMST_LUNAR_ORBIT;
			setSubState(1);
			break;
		case 1:
			if (SubStateTime > 5.0*60.0)
			{
				SlowIfDesired();
				setState(MST_J3_LUNAR_ORBIT_LOI_DAY_2);
			}
			break;
		}
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_2: //DOI-1 calculation to Photo PAD (90:45)
		UpdateMacro(UTP_NONE, PT_NONE, MoonRev >= 1 && MoonRevTime > 25.0*60.0, 29, MST_J3_LUNAR_ORBIT_LOI_DAY_3);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_3: //Photo PAD (90:45) to Photo PAD (91:20)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 150, MST_J3_LUNAR_ORBIT_LOI_DAY_4);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_4: //Photo PAD (91:20) to landing site REFSMMAT
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 151, MST_J3_LUNAR_ORBIT_LOI_DAY_5);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_5: //landing site REFSMMAT to DOI-1 Update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 2 && MoonRevTime > 40.0*60.0, 31, MST_J3_LUNAR_ORBIT_LOI_DAY_6);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_6: //DOI-1 Update to Rev 3 Map Update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, true, 30, MST_J3_LUNAR_ORBIT_LOI_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_7: //Rev 3 Map Update to J3 Landmark Tracking PAD
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, SubStateTime > 3.0*60.0, 129, MST_J3_LUNAR_ORBIT_LOI_DAY_8);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_8: //J3 Landmark Tracking PAD to TEI-5 update
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 5.0*60.0, 62, MST_J3_LUNAR_ORBIT_LOI_DAY_9);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_9: //TEI-5 update to SV update
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, MoonRev >= 4 && MoonRevTime > 30.0*60.0, 42, MST_J3_LUNAR_ORBIT_LOI_DAY_10);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_10: //SV update to CIRC burn calculation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 5, MST_J3_LUNAR_ORBIT_LOI_DAY_11);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_11: //CIRC burn calculation to Liftoff time Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 39, MST_J3_LUNAR_ORBIT_LOI_DAY_12);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_12: //Liftoff time Evaluation to CMC Liftoff time Update or landing site REFSMMAT
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 142, MST_J3_LUNAR_ORBIT_LOI_DAY_13, scrubbed, SubStateTime > 5.0, MST_J3_LUNAR_ORBIT_LOI_DAY_15);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_13: //CMC Liftoff time Update to RTCC Liftoff time update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 144, MST_J3_LUNAR_ORBIT_LOI_DAY_14);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_14: //RTCC Liftoff time update to landing site REFSMMAT
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 145, MST_J3_LUNAR_ORBIT_LOI_DAY_15);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_15: //landing site REFSMMAT to TEI-12 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 32, MST_J3_LUNAR_ORBIT_LOI_DAY_16);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_16: //TEI-12 update to TEI-19 update
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 43, MST_J3_LUNAR_ORBIT_LOI_DAY_17);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_17: //TEI-19 update to LM DAP load
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 44, MST_J3_LUNAR_ORBIT_LOI_DAY_18);
		break;
	case MST_J3_LUNAR_ORBIT_LOI_DAY_18: //LM DAP load to CSM SV update
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, MoonRev >= 10 && MoonRevTime > 35.0*60.0, 9, MST_J3_LUNAR_ORBIT_PDI_DAY_1);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_1: //CSM SV update to DOI-2 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 11 && MoonRevTime > 30.0*60.0, 5, MST_J3_LUNAR_ORBIT_PDI_DAY_2);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_2: //DOI-2 update to Abort constants update
		UpdateMacro(UTP_PADONLY, PT_AP11LMMNV, SubStateTime > 5.0*60.0, 38, MST_J3_LUNAR_ORBIT_PDI_DAY_3);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_3: //Abort constants update to LGC activation update
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_GENERIC, SubStateTime > 5.0*60.0, 36, MST_J3_LUNAR_ORBIT_PDI_DAY_4);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_4: //LGC activation update to LGC clock sync elvaluation
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 35, MST_J3_LUNAR_ORBIT_PDI_DAY_5);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_5: //LGC clock sync elvaluation to LGC clock sync uplink or CSM SV update
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 147, MST_J3_LUNAR_ORBIT_PDI_DAY_6, scrubbed, true, MST_J3_LUNAR_ORBIT_PDI_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_6: //LGC clock sync uplink to CSM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 149, MST_J3_LUNAR_ORBIT_PDI_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_7: //CSM SV update to CSM DAP load
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 5, MST_J3_LUNAR_ORBIT_PDI_DAY_8);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_8: //CSM DAP load to Undock/sep PAD
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 5.0*60.0, 7, MST_J3_LUNAR_ORBIT_PDI_DAY_9);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_9: //Undock/sep PAD to P24 track PAD (LDMK 17-x)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 37, MST_J3_LUNAR_ORBIT_PDI_DAY_10);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_10: //P24 track PAD (LDMK 17-x) to P76 PAD for DOI-2
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 5.0*60.0, 63, MST_J3_LUNAR_ORBIT_PDI_DAY_11);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_11: //P76 PAD for DOI-2 to AGS K-factor
		UpdateMacro(UTP_PADONLY, PT_AP11P76PAD, rtcc->GETEval2(rtcc->calcParams.SEP + 25.0*60.0), 57, MST_J3_LUNAR_ORBIT_PDI_DAY_12);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_12: //AGS K-factor to REV 12 LS TCA
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 2.0*60.0, 55, MST_J3_LUNAR_ORBIT_PDI_DAY_13);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_13: //REV 12 LS TCA to CIRC burn calculation
		UpdateMacro(UTP_PADONLY, PT_GENERIC, rtcc->GETEval2(rtcc->calcParams.DOI - 60.0*60.0), 56, MST_J3_LUNAR_ORBIT_PDI_DAY_14);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_14: //CIRC burn calculation to P76 PAD for CIRC
		UpdateMacro(UTP_NONE, PT_NONE, true, 39, MST_J3_LUNAR_ORBIT_PDI_DAY_15);
		break;
	case MST_J3_LUNAR_ORBIT_PDI_DAY_15: //P76 PAD for CIRC to No PDI+12-1 PAD
		UpdateMacro(UTP_PADONLY, PT_AP11P76PAD, SubStateTime > 3.0*60.0, 58, MST_J3_LUNAR_ORBIT_PRE_DOI_1);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_1: //No PDI+12 PAD to PDI PAD
		UpdateMacro(UTP_PADONLY, PT_AP11LMMNV, SubStateTime > 3.0*60.0, 72, MST_J3_LUNAR_ORBIT_PRE_DOI_2);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_2: //PDI PAD to PDI abort PAD
		UpdateMacro(UTP_PADONLY, PT_AP11PDIPAD, SubStateTime > 3.0*60.0, 70, MST_J3_LUNAR_ORBIT_PRE_DOI_3);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_3: //PDI abort PAD to Lunar Surface PAD
		UpdateMacro(UTP_PADONLY, PT_AP12PDIABORTPAD, SubStateTime > 3.0*60.0, 71, MST_J3_LUNAR_ORBIT_PRE_DOI_4);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_4: //Lunar Surface PAD to CIRC burn update
		UpdateMacro(UTP_PADONLY, PT_AP12LUNSURFPAD, SubStateTime > 3.0*60.0, 73, MST_J3_LUNAR_ORBIT_PRE_DOI_6);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_6: //CIRC burn update to P24 track PAD (LDMK RP-3)
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 3.0*60.0, 40, MST_J3_LUNAR_ORBIT_PRE_DOI_7);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_7: //P24 track PAD (LDMK RP-3) to LGC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0*60.0, 65, MST_J3_LUNAR_ORBIT_PRE_DOI_8);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_8: //LGC CSM SV update to LGC LM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 3, MST_J3_LUNAR_ORBIT_PRE_DOI_9);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_9: //LGC LM SV update to LGC descent target update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 4, MST_J3_LUNAR_ORBIT_PRE_DOI_10);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_DOI_10: //LGC descent target update to LGC LM SV update (PDI-10)
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 15.0*60.0), 76, MST_J3_LUNAR_ORBIT_PRE_PDI_1);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_PDI_1: //LGC LM SV update (PDI-10) to LGC RLS update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 136, MST_J3_LUNAR_ORBIT_PRE_PDI_2);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_PDI_2: //LGC RLS update to P24 track PAD (LDMK 17-1)
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 75, MST_J3_LUNAR_ORBIT_PRE_PDI_3);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_PDI_3: //P24 track PAD (LDMK 17-1) to CMC pre PDI uplinks
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0*60.0, 64, MST_J3_LUNAR_ORBIT_PRE_LANDING_1);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_LANDING_1: //CMC pre PDI uplinks to DOI evaluation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 5.0*60.0), 137, MST_J3_LUNAR_ORBIT_PRE_LANDING_2);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_LANDING_2: //DOI evaluation to PDI evaluation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 2.0*60.0), 77, MST_J3_LUNAR_ORBIT_PRE_LANDING_3);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_LANDING_3: //PDI Evaluation to landing confirmation or PDI-2 calulation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->calcParams.tgt->GroundContact(), 78, MST_J3_LUNAR_ORBIT_POST_LANDING_1, scrubbed, SubStateTime > 10.0*60.0, MST_J3_LUNAR_ORBIT_NO_PDI_1);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_1: //PDI-2 calulation to No PDI2+12 PAD
		UpdateMacro(UTP_NONE, PT_NONE, true, 133, MST_J3_LUNAR_ORBIT_NO_PDI_2);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_2: //No PDI2+12 PAD PAD to PDI-2 PAD
		UpdateMacro(UTP_PADONLY, PT_AP11LMMNV, SubStateTime > 3.0*60.0, 134, MST_J3_LUNAR_ORBIT_NO_PDI_3);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_3: //PDI-2 PAD to PDI-2 abort PAD
		UpdateMacro(UTP_PADONLY, PT_AP11PDIPAD, SubStateTime > 3.0*60.0, 70, MST_J3_LUNAR_ORBIT_NO_PDI_4);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_4: //PDI-2 abort PAD to Lunar Surface PAD
		UpdateMacro(UTP_PADONLY, PT_AP12PDIABORTPAD, SubStateTime > 3.0*60.0, 71, MST_J3_LUNAR_ORBIT_NO_PDI_5);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_5: //Lunar Surface PAD to Abort constants update
		UpdateMacro(UTP_PADONLY, PT_AP12LUNSURFPAD, SubStateTime > 3.0*60.0, 73, MST_J3_LUNAR_ORBIT_NO_PDI_6);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_6: //Abort constants update to LGC descent target update
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_GENERIC, SubStateTime > 3.0*60.0, 36, MST_J3_LUNAR_ORBIT_NO_PDI_7);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_7: //LGC descent target update to LGC LM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 76, MST_J3_LUNAR_ORBIT_NO_PDI_8);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_8: //LGC LM SV update to LGC CSM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 4, MST_J3_LUNAR_ORBIT_NO_PDI_9);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_9: //LGC CSM SV update to LGC LM SV update (PDI-10)
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 15.0*60.0), 3, MST_J3_LUNAR_ORBIT_NO_PDI_10);
		break;
	case MST_J3_LUNAR_ORBIT_NO_PDI_10: //LGC LM SV update (PDI-10) to CMC pre PDI uplinks
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 136, MST_J3_LUNAR_ORBIT_PRE_LANDING_1);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_1: //Landing confirmation to T1
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 15.0*60.0), 79, MST_J3_LUNAR_ORBIT_POST_LANDING_2);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_2: //T1 to T2
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 21.0*60.0 + 30.0), 80, MST_J3_LUNAR_ORBIT_POST_LANDING_3);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_3: //T2 to Photo PAD (114:10)
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 34.0*60.0), 81, MST_J3_LUNAR_ORBIT_POST_LANDING_4);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_4: //Photo PAD (114:10) to T3
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 152, MST_J3_LUNAR_ORBIT_POST_LANDING_5);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_5: //T3 to Liftoff times REVS 15-20
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 15.0*60.0, 82, MST_J3_LUNAR_ORBIT_POST_LANDING_6);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_6: //Liftoff times REV 15-20 to Photo PAD (116:20)
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, MoonRev >= 14 && MoonRevTime > 40.0*60.0, 85, MST_J3_LUNAR_ORBIT_POST_LANDING_7);
		break;
	case MST_J3_LUNAR_ORBIT_POST_LANDING_7: //Photo PAD (116:20) to TEI-26 update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 16 && MoonRevTime > 40.0*60.0, 153, MST_J3_LUNAR_ORBIT_EVA_DAY_1);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_1: //TEI-26 update to Earthshine photo PAD (121:05)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 45, MST_J3_LUNAR_ORBIT_EVA_DAY_2);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_2: //Earthshine photo PAD (121:05) to Liftoff times REVS 21-25
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 20 && MoonRevTime > 5.0*60.0, 154, MST_J3_LUNAR_ORBIT_EVA_DAY_3);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_3: //Liftoff times REVS 21-25 to LGC CSM SV update (155:15)
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, MoonRev >= 20 && MoonRevTime > 95.0*60.0, 86, MST_J3_LUNAR_ORBIT_EVA_DAY_4);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_4: //LGC CSM SV update (155:15) to Zodiacal light photo PAD (SR)
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, MoonRev >= 23 && MoonRevTime > 85.0*60.0, 3, MST_J3_LUNAR_ORBIT_EVA_DAY_5);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_5: //Zodiacal light photo PAD (SR) to CMC CSM CV update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 155, MST_J3_LUNAR_ORBIT_EVA_DAY_6);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_6: //CMC CSM CV update to Liftoff times REVS 26-32
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 25 && MoonRevTime > 25.0*60.0, 1, MST_J3_LUNAR_ORBIT_EVA_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_7: //Liftoff times REVS 26-32 to TEI-38 update
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, MoonRev >= 25 && MoonRevTime > 70.0*60.0, 87, MST_J3_LUNAR_ORBIT_EVA_DAY_8);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_8: //TEI-38 update to Solar corona photo PAD (137:33)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 46, MST_J3_LUNAR_ORBIT_EVA_DAY_9);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_9: //Solar corona photo PAD (137:33) to Pan camera photo PAD (141:50)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 27 && MoonRevTime > 50.0*60.0, 156, MST_J3_LUNAR_ORBIT_EVA_DAY_10);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_10: //Pan camera photo PAD (141:50) to TEI-49 update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 29 && MoonRevTime > 30.0*60.0, 157, MST_J3_LUNAR_ORBIT_EVA_DAY_11);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_11: //TEI-49 update to CMC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 55.0*60.0, 47, MST_J3_LUNAR_ORBIT_EVA_DAY_12);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_12: //CMC CSM SV update to Liftoff times REVS 33-37
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 31 && MoonRevTime > 50.0*60.0, 1, MST_J3_LUNAR_ORBIT_EVA_DAY_13);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_13: //Liftoff times REVS 33-37 to Lunar sounder PAD (156:50)
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, MoonRev >= 35 && MoonRevTime > 35.0*60.0, 88, MST_J3_LUNAR_ORBIT_EVA_DAY_14);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_14: //Lunar sounder PAD (156:50) to Lunar sounder PAD (158:40)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 158, MST_J3_LUNAR_ORBIT_EVA_DAY_15);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_15: //Lunar sounder PAD (158:40) to CMC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 7.0*60.0, 159, MST_J3_LUNAR_ORBIT_EVA_DAY_16);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_16: //CMC CSM SV update to Liftoff times REVS 38-43
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 37 && MoonRevTime > 20.0*60.0, 1, MST_J3_LUNAR_ORBIT_EVA_DAY_17);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_17: //Liftoff times REVS 38-43 to UV solar atmosphere PAD (160:38)
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, SubStateTime > 10.0*60.0, 89, MST_J3_LUNAR_ORBIT_EVA_DAY_18);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_18: //UV solar atmosphere PAD (160:38) to Zodiacal light photo PAD (163:13)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 38 && MoonRevTime > 80.0*60.0, 160, MST_J3_LUNAR_ORBIT_EVA_DAY_19);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_19: //Zodiacal light photo PAD (163:13) to Lunar sounder PAD (166:40)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 40 && MoonRevTime > 35.0*60.0, 161, MST_J3_LUNAR_ORBIT_EVA_DAY_20);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_20: //Lunar sounder PAD (166:40) to CMC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 42 && MoonRevTime > 60.0*60.0, 162, MST_J3_LUNAR_ORBIT_EVA_DAY_21);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_21: //CMC CSM SV update to Liftoff time calculation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 1, MST_J3_LUNAR_ORBIT_EVA_DAY_22);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_22: //Liftoff time calculation to Plane change calculation
		UpdateMacro(UTP_NONE, PT_NONE, true, 99, MST_J3_LUNAR_ORBIT_EVA_DAY_23);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_23: //Plane change calculation to TEI-55 PAD
		UpdateMacro(UTP_NONE, PT_NONE, true, 94, MST_J3_LUNAR_ORBIT_EVA_DAY_24);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_24: //TEI-55 PAD to Liftoff times REVS 44-50
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, MoonRev >= 43 && MoonRevTime > 30.0*60.0, 48, MST_J3_LUNAR_ORBIT_EVA_DAY_25);
		break;
	case MST_J3_LUNAR_ORBIT_EVA_DAY_25: //Liftoff times REVS 44-50 to LOPC update
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, MoonRev >= 47 && MoonRevTime > 55.0*60.0, 90, MST_J3_LUNAR_ORBIT_ASCENT_DAY_1);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_1: //LOPC update to Nominal liftoff calculation
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->TimeofIgnition + 5.0*60.0), 95, MST_J3_LUNAR_ORBIT_ASCENT_DAY_2);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_2: //Nominal liftoff calculation to CMC RLS uplink
		UpdateMacro(UTP_NONE, PT_NONE, true, 100, MST_J3_LUNAR_ORBIT_ASCENT_DAY_3);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_3: //CMC RLS uplink to CMC Liftoff REFSMMAT uplink
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 74, MST_J3_LUNAR_ORBIT_ASCENT_DAY_4);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_4: //CMC Liftoff REFSMMAT uplink to Pan camera photo PAD (183:45)
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 10.0*60.0, 96, MST_J3_LUNAR_ORBIT_ASCENT_DAY_5);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_5: //Pan camera photo PAD (183:45) to CMC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 49 && MoonRevTime > 35.0*60.0, 163, MST_J3_LUNAR_ORBIT_ASCENT_DAY_6);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_6: //CMC CSM SV update to P57 Liftoff time
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 10.0*60.0, 1, MST_J3_LUNAR_ORBIT_ASCENT_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_7: //P57 Liftoff time to P22 Acq time
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 98, MST_J3_LUNAR_ORBIT_ASCENT_DAY_8);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_8: //P22 Acq time to LGC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_LMP22ACQPAD, SubStateTime > 3.0*60.0, 59, MST_J3_LUNAR_ORBIT_ASCENT_DAY_9);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_9: //LGC CSM SV update to LGC RLS update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 3, MST_J3_LUNAR_ORBIT_ASCENT_DAY_10);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_10: //LGC RLS update to Zodiacal light photo PAD
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, MoonRev >= 49 && MoonRevTime > 70.0*60.0, 75, MST_J3_LUNAR_ORBIT_ASCENT_DAY_11);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_11: //Zodiacal light photo PAD to P24 track PADS (F-1 & 17-1)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 164, MST_J3_LUNAR_ORBIT_ASCENT_DAY_12);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_12: //P24 track PADS (F-1 & 17-1) to RTCC Liftoff time update
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, MoonRev >= 50 && MoonRevTime > 40.0*60.0, 66, MST_J3_LUNAR_ORBIT_ASCENT_DAY_13);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_13: //RTCC Liftoff time update to Direct liftoff calculation
		UpdateMacro(UTP_NONE, PT_NONE, true, 145, MST_J3_LUNAR_ORBIT_ASCENT_DAY_14);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_14: //Direct liftoff calculation to Ascent PAD (direct)
		UpdateMacro(UTP_NONE, PT_NONE, true, 100, MST_J3_LUNAR_ORBIT_ASCENT_DAY_15);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_15: //Ascent PAD (direct) to Coelliptic liftoff calculation
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_J3_LUNAR_ORBIT_ASCENT_DAY_16);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_16: //Coelliptic liftoff calculation to Ascent PAD (coelliptic)
		UpdateMacro(UTP_NONE, PT_NONE, true, 101, MST_J3_LUNAR_ORBIT_ASCENT_DAY_17);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_17: //Ascent PAD (coelliptic) to CSI PAD
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_J3_LUNAR_ORBIT_ASCENT_DAY_18);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_18: //CSI PAD to CMC insertion state vectors
		UpdateMacro(UTP_PADONLY, PT_AP10CSI, SubStateTime > 15.0*60.0, 106, MST_J3_LUNAR_ORBIT_ASCENT_DAY_19);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_19: //CMC insertion state vectors to CSM DAP DATA
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 102, MST_J3_LUNAR_ORBIT_ASCENT_DAY_20);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_20: //CSM DAP DATA to LGC CSM state vector + RLS
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 47.0*60.0), 7, MST_J3_LUNAR_ORBIT_ASCENT_DAY_21);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_21: //LGC CSM state vector + RLS to AGS K-factor
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, SubStateTime > 13.0*60.0, 103, MST_J3_LUNAR_ORBIT_ASCENT_DAY_22);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_22: //AGS K-factor + RLS to Ascent determination
		UpdateMacro(UTP_PADONLY, PT_GENERIC, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff + 10.0), 55, MST_J3_LUNAR_ORBIT_ASCENT_DAY_23);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_23: //Ascent determination to LM Liftoff Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 30.0), 108, MST_J3_LUNAR_ORBIT_ASCENT_DAY_24, scrubbed, rtcc->GETEval2(rtcc->calcParams.Insertion + 30.0), MST_J3_LUNAR_ORBIT_ASCENT_COELLIPTIC_1);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_24: //LM Liftoff Evaluation to Tweak burn or Ascent recycle
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 60.0), 107, MST_J3_LUNAR_ORBIT_ASCENT_DAY_25, scrubbed, true, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_1);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_25: //Tweak burn to CMC LM SV update
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TPI - 38.0*60.0), 109, MST_J3_LUNAR_ORBIT_ASCENT_DAY_26);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_DAY_26: //CMC LM SV update to Impact burn update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TPI + 3.0*3600.0 + 3.0*60.0), 2, MST_J3_LUNAR_ORBIT_POST_DOCKING_1);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_COELLIPTIC_1: //LM Liftoff Evaluation (coelliptic timeline) to CMC LM SV update or Ascent recycle
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 7.0*60.0), 107, MST_J3_LUNAR_ORBIT_ASCENT_DAY_26, scrubbed, true, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_1);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_1: //Liftoff time calculation to Direct liftoff calculation
		UpdateMacro(UTP_NONE, PT_NONE, true, 99, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_2);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_2: //Direct liftoff calculation to Ascent PAD (direct)
		UpdateMacro(UTP_NONE, PT_NONE, true, 100, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_3);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_3: //Ascent PAD (direct) to Coelliptic liftoff calculation
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_4);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_4: //Coelliptic liftoff calculation to Ascent PAD (coelliptic)
		UpdateMacro(UTP_NONE, PT_NONE, true, 101, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_5);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_5: //Ascent PAD (coelliptic) to CSI PAD
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_6);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_6: //CSI PAD to CMC insertion state vectors
		UpdateMacro(UTP_PADONLY, PT_AP10CSI, SubStateTime > 5.0*60.0, 106, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_7);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_7: //CMC insertion state vectors to LGC CSM state vector + RLS
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 47.0*60.0), 102, MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_8);
		break;
	case MST_J3_LUNAR_ORBIT_ASCENT_RECYCLE_8: //LGC CSM state vector + RLS to Ascent determination
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff + 10.0), 103, MST_J3_LUNAR_ORBIT_ASCENT_DAY_23);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_1: //Impact burn update to P99 uplink
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_AP11LMMNV, SubStateTime > 5.0*60.0, 111, MST_J3_LUNAR_ORBIT_POST_DOCKING_2);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_2: //P99 uplink to LM DAP DATA
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 120, MST_J3_LUNAR_ORBIT_POST_DOCKING_3);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_3: //LM DAP DATA to CSM DAP DATA
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 3.0*60.0, 9, MST_J3_LUNAR_ORBIT_POST_DOCKING_4);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_4: //CSM DAP DATA to LM Jettison PAD
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 3.0*60.0, 7, MST_J3_LUNAR_ORBIT_POST_DOCKING_5);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_5: //LM Jettison PAD to CSM state vector
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_GENERIC, rtcc->GETEval2(rtcc->calcParams.SEP + 30.0*60.0), 110, MST_J3_LUNAR_ORBIT_POST_DOCKING_6);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_6: //CSM state vector to Ground command V96
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->TimeofIgnition - 10.0*60.0), 5, MST_J3_LUNAR_ORBIT_POST_DOCKING_7);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_7: //Ground command V96 to Ground command V5N26
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 10.0, 121, MST_J3_LUNAR_ORBIT_POST_DOCKING_8);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_8: //Ground command V5N26 to Ground command V30
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 126, MST_J3_LUNAR_ORBIT_POST_DOCKING_9);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_9: //Ground command V30 to Ground command V33
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 122, MST_J3_LUNAR_ORBIT_POST_DOCKING_10);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_10: //Ground command V33 to Ground command ENT
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, rtcc->GETEval2(rtcc->TimeofIgnition - 2.0*60.0), 123, MST_J3_LUNAR_ORBIT_POST_DOCKING_11);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_11: //Ground command ENT to Ground command V33
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TIGSTORE1 + 30.0), 124, MST_J3_LUNAR_ORBIT_POST_DOCKING_12);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_12: //Ground command V33 to Ground command V33
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 123, MST_J3_LUNAR_ORBIT_POST_DOCKING_13);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_13: //Ground command V33 to Ground command P00
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 123, MST_J3_LUNAR_ORBIT_POST_DOCKING_14);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_14: //Ground command P00 to Lunar sounder T-STOP (196:20)
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, rtcc->GETEval2(rtcc->calcParams.SEP + 110.0*60.0), 125, MST_J3_LUNAR_ORBIT_POST_DOCKING_15);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_15: //Lunar sounder T-STOP (196:20) to TEI-65 update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 165, MST_J3_LUNAR_ORBIT_POST_DOCKING_16);
		break;
	case MST_J3_LUNAR_ORBIT_POST_DOCKING_16: //TEI-65 update to Solar corona photo PAD (SS)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, MoonRev >= 61 && MoonRevTime > 60.0*60.0, 49, MST_J3_LUNAR_ORBIT_FINAL_DAY_1);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_1: //Solar corona photo PAD (SS) to Pan camera photo PAD (209:15)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 166, MST_J3_LUNAR_ORBIT_FINAL_DAY_2);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_2: //Pan camera photo PAD (209:15) to CSM state vector
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 62 && MoonRevTime > 38.0*60.0, 167, MST_J3_LUNAR_ORBIT_FINAL_DAY_3);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_3: //CSM state vector to Liftoff time Evaluation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 5, MST_J3_LUNAR_ORBIT_FINAL_DAY_4);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_4: //Liftoff time Evaluation to CMC Liftoff time Update or Pan camera photo PAD
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 143, MST_J3_LUNAR_ORBIT_FINAL_DAY_5, scrubbed, SubStateTime > 5.0, MST_J3_LUNAR_ORBIT_FINAL_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_5: //CMC Liftoff time Update to RTCC Liftoff time update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 144, MST_J3_LUNAR_ORBIT_FINAL_DAY_6);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_6: //RTCC Liftoff time update to Pan camera photo PAD
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 145, MST_J3_LUNAR_ORBIT_FINAL_DAY_7);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_7: //Pan camera photo PAD to Lunar sounder PAD (213:15)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 63 && MoonRevTime > 65.0*60.0, 168, MST_J3_LUNAR_ORBIT_FINAL_DAY_8);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_8: //Lunar sounder PAD (213:15) to Lunar sounder PAD (213:38)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 169, MST_J3_LUNAR_ORBIT_FINAL_DAY_9);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_9: //Lunar sounder PAD (213:38) to TEI-72 update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 64 && MoonRevTime > 40.0*60.0, 170, MST_J3_LUNAR_ORBIT_FINAL_DAY_10);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_10: //TEI-72 update to CSM state vector
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, MoonRev >= 67 && MoonRevTime > 67.0*60.0, 50, MST_J3_LUNAR_ORBIT_FINAL_DAY_11);
		break;
	case MST_J3_LUNAR_ORBIT_FINAL_DAY_11: //CSM state vector to CSM state vector
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 72 && MoonRevTime > 68.0*60.0, 5, MST_J3_LUNAR_ORBIT_PRE_TEI_1);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_1: //CSM state vector to Lunar sounder PAD (230:55)
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 5, MST_J3_LUNAR_ORBIT_PRE_TEI_2);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_2: //Lunar sounder PAD (230:55) to TEI-75 PAD (preliminary)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 171, MST_J3_LUNAR_ORBIT_PRE_TEI_3);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_3: //TEI-75 PAD (preliminary) to Lunar sounder PAD
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, MoonRev >= 73 && MoonRevTime > 27.0*60.0, 51, MST_J3_LUNAR_ORBIT_PRE_TEI_4);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_4: //Lunar sounder PAD to Lunar sounder PAD (232:20)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 73 && MoonRevTime > 45.0*60.0, 172, MST_J3_LUNAR_ORBIT_PRE_TEI_5);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_5: //Lunar sounder PAD (232:20) to Pan camera photo PAD (233:20)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 173, MST_J3_LUNAR_ORBIT_PRE_TEI_6);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_6: //Pan camera photo PAD (233:20) to Pan camera photo PAD (233:40)
		UpdateMacro(UTP_PADONLY, PT_GENERIC, MoonRev >= 74 && MoonRevTime > 35.0*60.0, 174, MST_J3_LUNAR_ORBIT_PRE_TEI_7);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_7: //Pan camera photo PAD (233:40) to TEI REFSMMAT uplink
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 175, MST_J3_LUNAR_ORBIT_PRE_TEI_8);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_8: //TEI REFSMMAT uplink to TEI-75 update (final)
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, MoonRev >= 75 && MoonRevTime > 46.0*60.0, 132, MST_J3_LUNAR_ORBIT_PRE_TEI_9);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_9: //TEI-75 update (final) to TEI-76 update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 52, MST_J3_LUNAR_ORBIT_PRE_TEI_10);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_10: //TEI-76 update to Map update REV 76
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 53, MST_J3_LUNAR_ORBIT_PRE_TEI_11);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_11: //Map update REV 76 to TEI Evaluation
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, rtcc->GETEval2(rtcc->calcParams.TEI + 300.0), 130, MST_J3_LUNAR_ORBIT_PRE_TEI_12);
		break;
	case MST_J3_LUNAR_ORBIT_PRE_TEI_12: //TEI Evaluation to TEI or TEI recycle
		UpdateMacro(UTP_NONE, PT_NONE, true, 200, MST_J3_TRANSEARTH_1, scrubbed, rtcc->GETEval2(rtcc->calcParams.TEI + 50.0*60.0), MST_J3_LUNAR_ORBIT_PRE_TEI_9);
		break;
	case MST_J3_TRANSEARTH_1: //TEI to PTC REFSMMAT
		switch (SubState)
		{
		case 0:
			MissionPhase = MMST_TE_COAST;
			setSubState(1);
			break;
		case 1:
			if (rtcc->GETEval2(rtcc->calcParams.TEI + 75.0*60.0))
			{
				SlowIfDesired();
				setState(MST_J3_TRANSEARTH_2);
			}
			break;
		}
		break;
	case MST_J3_TRANSEARTH_2: //PTC REFSMMAT to CSM SV update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TEI + 4.0*3600.0 + 5.0*60.0), 18, MST_J3_TRANSEARTH_3);
		break;
	case MST_J3_TRANSEARTH_3: //CSM SV update to MCC-5 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TEI + 15.0*3600.0 + 25.0*60.0), 5, MST_J3_TRANSEARTH_4);
		break;
	case MST_J3_TRANSEARTH_4: //MCC-5 update to CSM SV update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.TEI + 26.0*3600.0 + 40.0*60.0), 210, MST_J3_TRANSEARTH_5);
		break;
	case MST_J3_TRANSEARTH_5: //CSM SV update to MCC-6 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.EI - 23.0 * 3600.0 - 20.0 * 60.0), 5, MST_J3_TRANSEARTH_6);
		break;
	case MST_J3_TRANSEARTH_6: //MCC-6 update to Entry PAD update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 212, MST_J3_TRANSEARTH_7);
		break;
	case MST_J3_TRANSEARTH_7: //Entry PAD update to MCC-7 decision update
		UpdateMacro(UTP_PADONLY, PT_AP11ENT, rtcc->GETEval2(rtcc->calcParams.EI - 4.0*3600.0 - 30.0*60.0), 216, MST_J3_TRANSEARTH_8);
		break;
	case MST_J3_TRANSEARTH_8: //MCC-7 decision update to MCC-7 update
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0*60.0, 213, MST_J3_TRANSEARTH_9);
		break;
	case MST_J3_TRANSEARTH_9: //MCC-7 update to Entry PAD update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 214, MST_J3_TRANSEARTH_10);
		break;
	case MST_J3_TRANSEARTH_10: //Entry PAD update to final entry update
		UpdateMacro(UTP_PADONLY, PT_AP11ENT, rtcc->GETEval2(rtcc->calcParams.EI - 45.0*60.0), 217, MST_J3_TRANSEARTH_11);
		break;
	case MST_J3_TRANSEARTH_11: //Final entry update to CM/SM separation
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11ENT, cm->GetStage() == CM_STAGE, 218, MST_ENTRY);
		break;
	case MST_ENTRY:
		switch (SubState) {
		case 0:
		{
			MissionPhase = MMST_ENTRY;
			setSubState(1);
		}
		break;
		case 1:
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
		break;
		}
		break;
		//Alternative sequences
	case MST_J3_ABORT_ORBIT:
	{
		if (AbortMode == 5) //Earth Orbit Abort
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
	}
	break;
	case MST_J3_ABORT:
		if (AbortMode == 6)	//Translunar Coast
		{
			switch (SubState) {
			case 0:
			{
				if (rtcc->GETEval2(OrbMech::HHMMSSToSS(65, 0, 0)))
				{
					setSubState(13);//Flyby
				}
				else
				{
					if (rtcc->GETEval2(OrbMech::HHMMSSToSS(55, 0, 0))) //LO+65
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(65, 0, 0);
					}
					if (rtcc->GETEval2(OrbMech::HHMMSSToSS(45, 0, 0))) //LO+55
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(55, 0, 0);
					}
					else if (rtcc->GETEval2(OrbMech::HHMMSSToSS(35, 0, 0))) //LO+45
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(45, 0, 0);
					}
					else if (rtcc->GETEval2(OrbMech::HHMMSSToSS(25, 0, 0))) //LO+35
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(35, 0, 0);
					}
					else if (rtcc->GETEval2(OrbMech::HHMMSSToSS(15, 0, 0))) //LO+25
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(25, 0, 0);
					}
					else if (rtcc->GETEval2(OrbMech::HHMMSSToSS(9, 0, 0))) //LO+15
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(15, 0, 0);
					}
					else if (rtcc->GETEval2(rtcc->calcParams.TLI + 90.0*60.0)) //LO+9
					{
						rtcc->calcParams.TEI = OrbMech::HHMMSSToSS(9, 0, 0);
					}
					else //TLI+90
					{
						rtcc->calcParams.TEI = rtcc->calcParams.TLI + 90.0*60.0;
					}

					setSubState(1);
				}
			}
			break;
			case 1:
			{
				if (rtcc->GETEval2(rtcc->calcParams.TEI + 10.0*60.0))
				{
					startSubthread(205, UTP_NONE); //Evaluate EI conditions
					setSubState(2);
				}
			}
			break;
			case 2:
				if (rtcc->calcParams.TEI > rtcc->calcParams.EI - 12.0 * 60 * 60)
				{
					setSubState(3);//Skip directly to normal entry procedures
				}
				else
				{
					setSubState(4);	//Include another course correction
				}
				break;
			case 3:
			{
				if (rtcc->GETEval2(rtcc->calcParams.EI - 4.0*3600.0 - 35.0*60.0))
				{
					SlowIfDesired();
					setState(MST_J3_TRANSEARTH_9);
				}
			}
			break;
			case 4:
			{
				if (rtcc->GETEval2(rtcc->calcParams.TEI + 4.0 * 60 * 60))
				{
					SlowIfDesired();
					setSubState(5);
				}
			}
			break;

			case 5:
				allocPad(8); // Allocate AP11 Maneuver Pad
				if (padForm != NULL) {
					// If success
					startSubthread(300, UTP_PADWITHCMCUPLINK); // Start subthread to fill PAD
				}
				else {
					// ERROR STATE
				}
				setSubState(6);
				// FALL INTO
			case 6: // Await pad read-up time (however long it took to compute it and give it to capcom)
				if (SubStateTime > 1 && padState > -1) {
					if (scrubbed)
					{
						if (upMessage[0] != 0)
						{
							addMessage(upMessage);
						}
						freePad();
						scrubbed = false;
						setSubState(11);
					}
					else
					{

						addMessage("You can has PAD");
						if (padAutoShow == true && padState == 0) { drawPad(); }
						// Completed. We really should test for P00 and proceed since that would be visible to the ground.
						addMessage("Ready for uplink?");
						sprintf(PCOption_Text, "Ready for uplink");
						PCOption_Enabled = true;
						setSubState(7);
					}
				}
				break;
			case 7: // Awaiting user response
			case 8: // Negative response / not ready for uplink
				break;
			case 9: // Ready for uplink
				if (SubStateTime > 1 && padState > -1) {
					// The uplink should also be ready, so flush the uplink buffer to the CMC
					this->CM_uplink_buffer();
					// uplink_size = 0; // Reset
					PCOption_Enabled = false; // No longer needed
					if (upDescr[0] != 0)
					{
						addMessage(upDescr);
					}
					setSubState(10);
				}
				break;
			case 10: // Await uplink completion
				if (cm->pcm.mcc_size == 0) {
					addMessage("Uplink completed!");
					NCOption_Enabled = true;
					sprintf(NCOption_Text, "Repeat uplink");
					setSubState(11);
				}
				break;
			case 11: // Await burn
				if (rtcc->GETEval2(rtcc->calcParams.EI - 4.0*3600.0 - 35.0*60.0))
				{
					SlowIfDesired();
					setState(MST_J3_TRANSEARTH_9);
				}
				break;
			case 12: //Repeat uplink
			{
				NCOption_Enabled = false;
				setSubState(5);
			}
			break;
			case 13:
			{
				//Wait until LOI + 2.5 hours, then calculate Pericynthion time and EI conditions for return trajectory
				if (rtcc->GETEval2(rtcc->calcParams.LOI + 2.5*3600.0))
				{
					rtcc->calcParams.LOI = rtcc->PericynthionTime(cm);
					startSubthread(205, UTP_NONE);
					setSubState(14);
				}
			}
			break;
			case 14: //Flyby, go to nominal TEC procedures
			{
				if (rtcc->calcParams.EI - rtcc->calcParams.LOI > 45.0*3600.0) //Is this a flyby or fast PC+2?
				{
					rtcc->calcParams.TEI = rtcc->calcParams.LOI;
					setState(MST_J3_TRANSEARTH_1);
				}
				else
				{
					rtcc->calcParams.TEI = rtcc->calcParams.EI - 30.0*3600.0;
					setSubState(4); //Go to genric MCC. Not enough time for the full transearth timeline.
				}
			}
			break;
			}
		}
		else if (AbortMode == 7) //Lunar Orbit
		{
			switch (SubState) {
			case 0:
			{
				if (MoonRevTime > 900.0)
				{
					setSubState(1);
				}
			}
			break;
			case 1:
			{
				SV sv;
				OELEMENTS coe;

				sv = rtcc->StateVectorCalc(rtcc->calcParams.src);

				coe = OrbMech::coe_from_sv(sv.R, sv.V, OrbMech::mu_Moon);

				if (coe.e > 0.7)
				{
					rtcc->calcParams.TEI = rtcc->PericynthionTime(cm);
					setState(MST_J3_TRANSEARTH_1);
				}
				else
				{
					setSubState(2);
				}
			}
			break;
			case 2:
			{
				if (MoonRevTime < 100.0)
				{
					setSubState(0);
				}
			}
			break;
			}
		}
		else if (AbortMode == 8)
		{
			//How to Abort?
		}
	}
}