/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2023

MCC sequencing for Mission JX (Generic MCC for custom J missions)

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
#include "MCC_Mission_JX.h"
#include "iu.h"
#include "nassputils.h"

using namespace nassp;

//This mission profile is based on Apollo 15 and has a CSM flown DOI.
//See "MCC for custom missions.txt" in Doc\Project Apollo - NASSP\Programmers Notes

void MCC::MissionSequence_JX()
{
	switch (MissionState)
	{
	case MST_JX_PRELAUNCH: //Scenario start to prelaunch
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 5.0*60.0, 10, MST_SV_PRELAUNCH);
		break;
	case MST_JX_INSERTION: //Ground liftoff time update to TLI Simulation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(1.0*3600.0 + 30.0*60.0), 11, MST_JX_EPO1);
		break;
	case MST_JX_EPO1: //TLI Simulation to TLI PAD
		UpdateMacro(UTP_NONE, PT_NONE, true, 12, MST_JX_EPO2);
		break;
	case MST_JX_EPO2: //TLI PAD to SV Update
		UpdateMacro(UTP_PADONLY, PT_TLIPAD, SubStateTime > 5.0*60.0, 14, MST_JX_EPO3);
		break;
	case MST_JX_EPO3: //SV Update to TLI GO/NO-GO
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TLI - 20.0*60.0), 13, MST_JX_EPO4);
		break;
	case MST_JX_EPO4: //TLI GO/NO-GO to TLI Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TLI + 18.0), 115, MST_JX_TRANSLUNAR1);
		break;
	case MST_JX_TRANSLUNAR1: //TLI Evaluation to SIVB Evasive Maneuver
		UpdateMacro(UTP_NONE, PT_NONE, true, 15, MST_JX_TRANSLUNAR2, scrubbed, rtcc->GETEval2(rtcc->calcParams.TLI + 30.0*60.0), MST_JX_EPO1);
		break;
	case MST_JX_TRANSLUNAR2:
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
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 3600.0 + 31.0*60.0))
			{
				SlowIfDesired();
				setState(MST_JX_TRANSLUNAR3);
			}
		}
		break;
		}
		break;
	case MST_JX_TRANSLUNAR3: //SIVB Evasive Maneuver to TB8 Enable
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
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 3600.0 + 41.0*60.0))
			{
				SlowIfDesired();
				setState(MST_JX_TRANSLUNAR4);
			}
			break;
		}
		break;
	case MST_JX_TRANSLUNAR4:  //TB8 enable to MCC-1 Evaluation
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
			if (rtcc->GETEval2(rtcc->calcParams.TLI + 2.0*3600.0 + 14.0*60.0))
			{
				SlowIfDesired();
				setState(MST_JX_TRANSLUNAR6); //Skip SIVB MCC-1 for now
			}
			break;
		}
		break;
	case MST_JX_TRANSLUNAR5:  //SIVB MCC-1 to PTC REFSMMAT update
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
				setState(MST_JX_TRANSLUNAR6);
			}
			break;
		}
		break;
	case MST_JX_TRANSLUNAR6: //MCC-1 Evaluation to MCC-1 Update
		UpdateMacro(UTP_NONE, PT_NONE, true, 19, MST_JX_TRANSLUNAR7, scrubbed, true, MST_JX_TRANSLUNAR8);
		break;
	case MST_JX_TRANSLUNAR7: //MCC-1 update to PTC REFSMMAT
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->TimeofIgnition + 5.0*60.0), 21, MST_JX_TRANSLUNAR8);
		break;
	case MST_JX_TRANSLUNAR8: //PTC REFSMMAT update to MCC-2 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TLI + 26.0*3600.0 + 30.0*60.0), 18, MST_JX_TRANSLUNAR14);
		break;
	case MST_JX_TRANSLUNAR14: //MCC-2 update to Liftoff time Evaluation
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2((rtcc->calcParams.LOI - rtcc->calcParams.TLI) * 0.667), 22, MST_JX_TRANSLUNAR15);
		break;
	case MST_JX_TRANSLUNAR15: //Liftoff time Evaluation to CMC Liftoff time Update or MCC-3 Update
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 141, MST_JX_TRANSLUNAR16, scrubbed, rtcc->GETEval2(rtcc->calcParams.LOI - 23.5*3600.0), MST_JX_TRANSLUNAR18);
		break;
	case MST_JX_TRANSLUNAR16: //CMC Liftoff time Update to RTCC Liftoff time update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 144, MST_JX_TRANSLUNAR17);
		break;
	case MST_JX_TRANSLUNAR17: //RTCC Liftoff time update to MCC-3 Update
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LOI - 23.5*3600.0), 145, MST_JX_TRANSLUNAR18);
		break;
	case MST_JX_TRANSLUNAR18: //MCC-3 Update to MCC-4 update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 6.0*3600.0), 24, MST_JX_TRANSLUNAR19);
		break;
	case MST_JX_TRANSLUNAR19: //MCC-4 update to LOI update (preliminary)
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 3.5*3600.0), 25, MST_JX_TRANSLUNAR22);
		break;
	case MST_JX_TRANSLUNAR22: //LOI update (preliminary) to LOI update (final)
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.LOI - 1.0*3600.0), 27, MST_JX_TRANSLUNAR24);
		break;
	case MST_JX_TRANSLUNAR24: //LOI update (final) to Map Update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 28, MST_JX_TRANSLUNAR25);
		break;
	case MST_JX_TRANSLUNAR25: //Map Update to LOI Evaluation
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, rtcc->GETEval2(rtcc->TimeofIgnition + 7.0*60.0), 128, MST_JX_TRANSLUNAR26);
		break;
	case MST_JX_TRANSLUNAR26: //LOI Evaluation to Lunar orbit phase begin or PC+2
		UpdateMacro(UTP_NONE, PT_NONE, true, 33, MST_JX_LUNAR_ORBIT_LOI_DAY_1, scrubbed, true, MST_JX_ABORT);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_1: //Lunar orbit phase begin
		switch (SubState) {
		case 0:
			MissionPhase = MMST_LUNAR_ORBIT;
			setSubState(1);
			break;
		case 1:
			if (MoonRev >= 1 && MoonRevTime > 60.0*60.0)
			{
				SlowIfDesired();
				setState(MST_JX_LUNAR_ORBIT_LOI_DAY_2);
			}
			break;
		}
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_2: //landing site REFSMMAT to DOI Update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.DOI - 1.0*3600.0), 29, MST_JX_LUNAR_ORBIT_LOI_DAY_3);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_3: //DOI Update to SV update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, MoonRev >= 4 && MoonRevTime > 60.0*60.0, 30, MST_JX_LUNAR_ORBIT_LOI_DAY_4);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_4: //SV update to DOI trim evaluation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 10.0*3600.0), 5, MST_JX_LUNAR_ORBIT_LOI_DAY_5);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_5: //DOI trim evaluation to DOI trim update or SV update
		UpdateMacro(UTP_NONE, PT_NONE, true, 31, MST_JX_LUNAR_ORBIT_LOI_DAY_6, scrubbed, rtcc->GETEval2(rtcc->calcParams.PDI - 8.0*3600.0), MST_JX_LUNAR_ORBIT_LOI_DAY_7);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_6: //DOI trim update to SV update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->TimeofIgnition + 20.0*60.0), 32, MST_JX_LUNAR_ORBIT_LOI_DAY_7);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_7: //SV update to CIRC burn calculation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 5, MST_JX_LUNAR_ORBIT_LOI_DAY_8);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_8: //CIRC burn calculation to Liftoff time Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 39, MST_JX_LUNAR_ORBIT_LOI_DAY_9);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_9: //Liftoff time Evaluation to CMC Liftoff time Update or landing site REFSMMAT
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 142, MST_JX_LUNAR_ORBIT_LOI_DAY_10, scrubbed, SubStateTime > 5.0, MST_JX_LUNAR_ORBIT_LOI_DAY_12);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_10: //CMC Liftoff time Update to RTCC Liftoff time update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0, 144, MST_JX_LUNAR_ORBIT_LOI_DAY_11);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_11: //RTCC Liftoff time update to landing site REFSMMAT
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 145, MST_JX_LUNAR_ORBIT_LOI_DAY_12);
		break;
	case MST_JX_LUNAR_ORBIT_LOI_DAY_12: //landing site REFSMMAT to SV update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 6.0*3600.0), 34, MST_JX_LUNAR_ORBIT_PDI_DAY_1);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_1: //SV update to CSM DAP load
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 5, MST_JX_LUNAR_ORBIT_PDI_DAY_2);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_2: //CSM DAP load to Undock/sep PAD
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 3.0*60.0, 7, MST_JX_LUNAR_ORBIT_PDI_DAY_3);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_3: //Undock/sep PAD to P24 track PAD (LDMK XX-1)
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 3.0*60.0, 37, MST_JX_LUNAR_ORBIT_PDI_DAY_4);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_4: //P24 track PAD (LDMK XX-1) to LM DAP load
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0*60.0, 62, MST_JX_LUNAR_ORBIT_PDI_DAY_5);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_5: //LM DAP load to Abort constants update
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 3.0*60.0, 9, MST_JX_LUNAR_ORBIT_PDI_DAY_6);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_6: //Abort constants update to LGC activation update
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_GENERIC, SubStateTime > 3.0*60.0, 36, MST_JX_LUNAR_ORBIT_PDI_DAY_7);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_7: //LGC activation update to LGC clock sync elvaluation
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 35, MST_JX_LUNAR_ORBIT_PDI_DAY_8);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_8: //LGC clock sync elvaluation to LGC clock sync uplink or REV 12 LS TCA
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0, 147, MST_JX_LUNAR_ORBIT_PDI_DAY_9, scrubbed, rtcc->GETEval2(rtcc->calcParams.SEP + 10.0*60.0), MST_JX_LUNAR_ORBIT_PDI_DAY_10);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_9: //LGC clock sync uplink to REV 12 LS TCA
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.SEP + 10.0*60.0), 149, MST_JX_LUNAR_ORBIT_PDI_DAY_10);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_10: //REV 12 LS TCA to LGC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 56, MST_JX_LUNAR_ORBIT_PDI_DAY_11);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_11: //LGC CSM SV update to CIRC burn update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 3.5*3600.0), 3, MST_JX_LUNAR_ORBIT_PDI_DAY_12);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_12: //CIRC burn update to PDI-0 PAD
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 3.0*60.0, 40, MST_JX_LUNAR_ORBIT_PDI_DAY_13);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_13: //PDI-0 PAD to AGS K-factor
		UpdateMacro(UTP_PADONLY, PT_AP11LMMNV, SubStateTime > 20.0*60.0, 38, MST_JX_LUNAR_ORBIT_PDI_DAY_14);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_14: //AGS K-factor to CMC CSM/LM SV updates
		UpdateMacro(UTP_PADONLY, PT_GENERIC, rtcc->GETEval2(rtcc->calcParams.PDI - 2.0*3600.0 - 16.0*60.0), 55, MST_JX_LUNAR_ORBIT_PDI_DAY_15);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_15: //CMC CSM/LM SV updates to P24 track PAD (LDMK XX-1)
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 138, MST_JX_LUNAR_ORBIT_PDI_DAY_16);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_16: //P24 track PAD (LDMK XX-1) to LGC CSM/LM SV updates
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0*60.0, 63, MST_JX_LUNAR_ORBIT_PDI_DAY_17);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_17: //LGC CSM/LM SV updates to LGC descent target update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 138, MST_JX_LUNAR_ORBIT_PDI_DAY_18);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_18: //LGC descent target update to No PDI+12 PAD
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 76, MST_JX_LUNAR_ORBIT_PDI_DAY_19);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_19: //No PDI+12 PAD to PDI PAD
		UpdateMacro(UTP_PADONLY, PT_AP11LMMNV, SubStateTime > 3.0*60.0, 72, MST_JX_LUNAR_ORBIT_PDI_DAY_20);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_20: //PDI PAD to PDI abort PAD
		UpdateMacro(UTP_PADONLY, PT_AP11PDIPAD, SubStateTime > 3.0*60.0, 70, MST_JX_LUNAR_ORBIT_PDI_DAY_21);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_21: //PDI abort PAD to Lunar Surface PAD
		UpdateMacro(UTP_PADONLY, PT_AP12PDIABORTPAD, SubStateTime > 3.0*60.0, 71, MST_JX_LUNAR_ORBIT_PDI_DAY_22);
		break;
	case MST_JX_LUNAR_ORBIT_PDI_DAY_22: //Lunar Surface PAD to LGC LM SV update (PDI-10)
		UpdateMacro(UTP_PADONLY, PT_AP12LUNSURFPAD, rtcc->GETEval2(rtcc->calcParams.PDI - 20.0*60.0), 73, MST_JX_LUNAR_ORBIT_PRE_PDI_1);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_PDI_1: //LGC LM SV update (PDI-10) to LGC RLS update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 136, MST_JX_LUNAR_ORBIT_PRE_PDI_2);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_PDI_2: //LGC RLS update to CMC pre PDI uplinks
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 75, MST_JX_LUNAR_ORBIT_PRE_LANDING_1);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_LANDING_1: //CMC pre PDI uplinks to DOI evaluation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 3.0*60.0), 137, MST_JX_LUNAR_ORBIT_PRE_LANDING_2);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_LANDING_2: //DOI evaluation to PDI Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 2.0*60.0), 77, MST_JX_LUNAR_ORBIT_PRE_LANDING_3);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_LANDING_3: //PDI Evaluation to landing confirmation or PDI-2 calulation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->calcParams.tgt->GroundContact(), 78, MST_JX_LUNAR_ORBIT_POST_LANDING_1, scrubbed, SubStateTime > 10.0*60.0, MST_JX_LUNAR_ORBIT_NO_PDI_1);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_1: //PDI-2 calulation to No PDI2+12 PAD
		UpdateMacro(UTP_NONE, PT_NONE, true, 133, MST_JX_LUNAR_ORBIT_NO_PDI_2);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_2: //No PDI2+12 PAD PAD to PDI-2 PAD
		UpdateMacro(UTP_PADONLY, PT_AP11LMMNV, SubStateTime > 3.0*60.0, 72, MST_JX_LUNAR_ORBIT_NO_PDI_3);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_3: //PDI-2 PAD to PDI-2 abort PAD
		UpdateMacro(UTP_PADONLY, PT_AP11PDIPAD, SubStateTime > 3.0*60.0, 70, MST_JX_LUNAR_ORBIT_NO_PDI_4);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_4: //PDI-2 abort PAD to Lunar Surface PAD
		UpdateMacro(UTP_PADONLY, PT_AP12PDIABORTPAD, SubStateTime > 3.0*60.0, 71, MST_JX_LUNAR_ORBIT_NO_PDI_5);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_5: //Lunar Surface PAD to Abort constants update
		UpdateMacro(UTP_PADONLY, PT_AP12LUNSURFPAD, SubStateTime > 3.0*60.0, 73, MST_JX_LUNAR_ORBIT_NO_PDI_6);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_6: //Abort constants update to LGC descent target update
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_GENERIC, SubStateTime > 3.0*60.0, 36, MST_JX_LUNAR_ORBIT_NO_PDI_7);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_7: //LGC descent target update to LGC LM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 76, MST_JX_LUNAR_ORBIT_NO_PDI_8);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_8: //LGC LM SV update to LGC CSM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 4, MST_JX_LUNAR_ORBIT_NO_PDI_9);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_9: //LGC CSM SV update to LGC LM SV update (PDI-10)
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI - 20.0*60.0), 3, MST_JX_LUNAR_ORBIT_NO_PDI_10);
		break;
	case MST_JX_LUNAR_ORBIT_NO_PDI_10: //LGC LM SV update (PDI-10) to CMC pre PDI uplinks
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 136, MST_JX_LUNAR_ORBIT_PRE_LANDING_1);
		break;
	case MST_JX_LUNAR_ORBIT_POST_LANDING_1: //Landing confirmation to T1
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 15.0*60.0), 79, MST_JX_LUNAR_ORBIT_POST_LANDING_2);
		break;
	case MST_JX_LUNAR_ORBIT_POST_LANDING_2: //T1 to T2
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 18.0*60.0 + 45.0), 80, MST_JX_LUNAR_ORBIT_POST_LANDING_3);
		break;
	case MST_JX_LUNAR_ORBIT_POST_LANDING_3: //T2 to T3
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.PDI + 60.0*60.0), 81, MST_JX_LUNAR_ORBIT_POST_LANDING_4);
		break;
	case MST_JX_LUNAR_ORBIT_POST_LANDING_4: //T3 to LGC CSM SV update
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->CZTDTGTU.GETTD + (((rtcc->calcParams.TPI - 90.0*60.0) - rtcc->CZTDTGTU.GETTD) / 2.0)), 82, MST_JX_LUNAR_ORBIT_EVA_DAY_1);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_1: //LGC CSM SV update to CMC CSM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 3, MST_JX_LUNAR_ORBIT_EVA_DAY_2);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_2: //CMC CSM SV update to LGC CSM SV update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TPI - 90.0*60.0), 1, MST_JX_LUNAR_ORBIT_EVA_DAY_3);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_3: //LGC CSM SV update to CMC CSM SV update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 3, MST_JX_LUNAR_ORBIT_EVA_DAY_4);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_4: //CMC CSM SV update to Plane change evaluation
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 1, MST_JX_LUNAR_ORBIT_EVA_DAY_5);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_5: //Plane change evaluation to Plane change update or Nominal liftoff calculation
		UpdateMacro(UTP_NONE, PT_NONE, true, 94, MST_JX_LUNAR_ORBIT_EVA_DAY_6, scrubbed, rtcc->GETEval2(rtcc->calcParams.TPI), MST_JX_LUNAR_ORBIT_EVA_DAY_7);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_6: //Plane change update to Nominal liftoff calculation
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->TimeofIgnition + 5.0*60.0), 95, MST_JX_LUNAR_ORBIT_EVA_DAY_7);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_7: //Nominal liftoff calculation to CMC RLS uplink
		UpdateMacro(UTP_NONE, PT_NONE, true, 100, MST_JX_LUNAR_ORBIT_EVA_DAY_8);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_8: //CMC RLS uplink to CMC Liftoff REFSMMAT uplink
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 74, MST_JX_LUNAR_ORBIT_EVA_DAY_9);
		break;
	case MST_JX_LUNAR_ORBIT_EVA_DAY_9: //CMC Liftoff REFSMMAT uplink to P57 Liftoff time
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 2.0*3600.0 - 55.0*60.0), 96, MST_JX_LUNAR_ORBIT_ASCENT_DAY_7);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_7: //P57 Liftoff time to P22 Acq time
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 3.0*60.0, 98, MST_JX_LUNAR_ORBIT_ASCENT_DAY_8);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_8: //P22 Acq time to LGC CSM SV update
		UpdateMacro(UTP_PADONLY, PT_LMP22ACQPAD, SubStateTime > 3.0*60.0, 59, MST_JX_LUNAR_ORBIT_ASCENT_DAY_9);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_9: //LGC CSM SV update to LGC RLS update
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 3, MST_JX_LUNAR_ORBIT_ASCENT_DAY_10);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_10: //LGC RLS update to Direct liftoff calculation
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 1.0*3600.0 - 50.0*60.0), 75, MST_JX_LUNAR_ORBIT_ASCENT_DAY_14);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_14: //Direct liftoff calculation to Ascent PAD (direct)
		UpdateMacro(UTP_NONE, PT_NONE, true, 100, MST_JX_LUNAR_ORBIT_ASCENT_DAY_15);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_15: //Ascent PAD (direct) to Coelliptic liftoff calculation
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_JX_LUNAR_ORBIT_ASCENT_DAY_16);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_16: //Coelliptic liftoff calculation to Ascent PAD (coelliptic)
		UpdateMacro(UTP_NONE, PT_NONE, true, 101, MST_JX_LUNAR_ORBIT_ASCENT_DAY_17);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_17: //Ascent PAD (coelliptic) to CSI PAD
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_JX_LUNAR_ORBIT_ASCENT_DAY_18);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_18: //CSI PAD to CMC insertion state vectors
		UpdateMacro(UTP_PADONLY, PT_AP10CSI, SubStateTime > 3.0*60.0, 106, MST_JX_LUNAR_ORBIT_ASCENT_DAY_19);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_19: //CMC insertion state vectors to CSM DAP DATA
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, true, 102, MST_JX_LUNAR_ORBIT_ASCENT_DAY_20);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_20: //CSM DAP DATA to AGS K-factor
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 50.0*60.0), 7, MST_JX_LUNAR_ORBIT_ASCENT_DAY_21);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_21: //AGS K-factor + RLS to LGC CSM state vector + RLS
		UpdateMacro(UTP_PADONLY, PT_GENERIC, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 35.0*60.0), 55, MST_JX_LUNAR_ORBIT_ASCENT_DAY_22);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_22: //LGC CSM state vector + RLS to Ascent determination
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff + 10.0), 103, MST_JX_LUNAR_ORBIT_ASCENT_DAY_23);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_23: //Ascent determination to LM Liftoff Evaluation
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 30.0), 108, MST_JX_LUNAR_ORBIT_ASCENT_DAY_24, scrubbed, rtcc->GETEval2(rtcc->calcParams.Insertion + 30.0), MST_JX_LUNAR_ORBIT_ASCENT_COELLIPTIC_1);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_24: //LM Liftoff Evaluation to Tweak burn or Ascent recycle
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 60.0), 107, MST_JX_LUNAR_ORBIT_ASCENT_DAY_25, scrubbed, true, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_1);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_25: //Tweak burn to CMC LM SV update
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 5.0*60.0), 109, MST_JX_LUNAR_ORBIT_ASCENT_DAY_26);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_DAY_26: //CMC LM SV update to Impact burn update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->calcParams.src->DockingStatus(0), 2, MST_JX_LUNAR_ORBIT_POST_DOCKING_1);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_COELLIPTIC_1: //LM Liftoff Evaluation (coelliptic timeline) to CMC LM SV update or Ascent recycle
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->GETEval2(rtcc->calcParams.Insertion + 7.0*60.0), 107, MST_JX_LUNAR_ORBIT_ASCENT_DAY_26, scrubbed, true, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_1);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_1: //Liftoff time re-calculation to Direct liftoff calculation
		UpdateMacro(UTP_NONE, PT_NONE, true, 99, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_2);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_2: //Direct liftoff calculation to Ascent PAD (direct)
		UpdateMacro(UTP_NONE, PT_NONE, true, 100, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_3);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_3: //Ascent PAD (direct) to Coelliptic liftoff calculation
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_4);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_4: //Coelliptic liftoff calculation to Ascent PAD (coelliptic)
		UpdateMacro(UTP_NONE, PT_NONE, true, 101, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_5);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_5: //Ascent PAD (coelliptic) to CSI PAD
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, SubStateTime > 5.0*60.0, 104, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_6);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_6: //CSI PAD to CMC insertion state vectors
		UpdateMacro(UTP_PADONLY, PT_AP10CSI, SubStateTime > 5.0*60.0, 106, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_7);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_7: //CMC insertion state vectors to LGC CSM state vector + RLS
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff - 35.0*60.0), 102, MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_8);
		break;
	case MST_JX_LUNAR_ORBIT_ASCENT_RECYCLE_8: //LGC CSM state vector + RLS to Ascent determination
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.LunarLiftoff + 10.0), 103, MST_JX_LUNAR_ORBIT_ASCENT_DAY_23);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_1: //Impact burn update to P99 uplink
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_AP11LMMNV, SubStateTime > 5.0*60.0, 111, MST_JX_LUNAR_ORBIT_POST_DOCKING_2);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_2: //P99 uplink to LM DAP DATA
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, true, 120, MST_JX_LUNAR_ORBIT_POST_DOCKING_3);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_3: //LM DAP DATA to CSM DAP DATA
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 3.0*60.0, 9, MST_JX_LUNAR_ORBIT_POST_DOCKING_4);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_4: //CSM DAP DATA to LM Jettison & CSM SEP PAD
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, rtcc->GETEval2(rtcc->calcParams.SEP - 1.0*3600.0 - 45.0*60.0), 7, MST_JX_LUNAR_ORBIT_POST_DOCKING_5);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_5: //LM Jettison & CSM SEP PAD to Ground command V96
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->TimeofIgnition - 10.0*60.0), 110, MST_JX_LUNAR_ORBIT_POST_DOCKING_7);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_7: //Ground command V96 to Ground command V5N26
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 10.0, 121, MST_JX_LUNAR_ORBIT_POST_DOCKING_8);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_8: //Ground command V5N26 to Ground command V30
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 126, MST_JX_LUNAR_ORBIT_POST_DOCKING_9);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_9: //Ground command V30 to Ground command V33
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 122, MST_JX_LUNAR_ORBIT_POST_DOCKING_10);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_10: //Ground command V33 to Ground command ENT
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, rtcc->GETEval2(rtcc->TimeofIgnition - 90.0), 123, MST_JX_LUNAR_ORBIT_POST_DOCKING_11);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_11: //Ground command ENT to Ground command V33
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TIGSTORE1 + 30.0), 124, MST_JX_LUNAR_ORBIT_POST_DOCKING_12);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_12: //Ground command V33 to Ground command V33
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 123, MST_JX_LUNAR_ORBIT_POST_DOCKING_13);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_13: //Ground command V33 to Ground command P00
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, SubStateTime > 30.0, 123, MST_JX_LUNAR_ORBIT_POST_DOCKING_14);
		break;
	case MST_JX_LUNAR_ORBIT_POST_DOCKING_14: //Ground command P00 to SV update
		UpdateMacro(UTP_LGCUPLINKDIRECT, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TIGSTORE1 + (((rtcc->calcParams.TEI - 3.0*3600.0) - rtcc->calcParams.TIGSTORE1) / 2.0)), 125, MST_JX_LUNAR_ORBIT_PRE_TEI_1);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_1: //SV update to TEI PAD (preliminary)
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TEI - 3.0*3600.0), 5, MST_JX_LUNAR_ORBIT_PRE_TEI_3);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_3: //TEI PAD (preliminary) to TEI REFSMMAT uplink
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 51, MST_JX_LUNAR_ORBIT_PRE_TEI_8);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_8: //TEI REFSMMAT uplink to TEI update (final)
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TEI - 1.0*3600.0), 132, MST_JX_LUNAR_ORBIT_PRE_TEI_9);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_9: //TEI update (final) to TEI (next rev)
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 52, MST_JX_LUNAR_ORBIT_PRE_TEI_10);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_10: //TEI (next rev) update update to Map update
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0*60.0, 53, MST_JX_LUNAR_ORBIT_PRE_TEI_11);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_11: //Map update to TEI Evaluation
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, rtcc->GETEval2(rtcc->calcParams.TEI + 300.0), 130, MST_JX_LUNAR_ORBIT_PRE_TEI_12);
		break;
	case MST_JX_LUNAR_ORBIT_PRE_TEI_12: //TEI Evaluation to TEI or TEI recycle
		UpdateMacro(UTP_NONE, PT_NONE, true, 200, MST_JX_TRANSEARTH_1, scrubbed, rtcc->GETEval2(rtcc->calcParams.TEI - 1.0*3600.0), MST_JX_LUNAR_ORBIT_PRE_TEI_9);
		break;
	case MST_JX_TRANSEARTH_1: //TEI to PTC REFSMMAT
		switch (SubState)
		{
		case 0:
			MissionPhase = MMST_TE_COAST;
			setSubState(1);
			break;
		case 1:
			if (rtcc->GETEval2(rtcc->calcParams.TEI + 50.0*60.0))
			{
				SlowIfDesired();
				setState(MST_JX_TRANSEARTH_2);
			}
			break;
		}
		break;
	case MST_JX_TRANSEARTH_2: //PTC REFSMMAT to CSM SV update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TEI + 4.0*3600.0 + 5.0*60.0), 18, MST_JX_TRANSEARTH_3);
		break;
	case MST_JX_TRANSEARTH_3: //CSM SV update to MCC-5 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.TEI + 13.0*3600.0 + 30.0*60.0), 5, MST_JX_TRANSEARTH_4);
		break;
	case MST_JX_TRANSEARTH_4: //MCC-5 update to CSM SV update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, rtcc->GETEval2(rtcc->calcParams.TEI + 26.0*3600.0 + 40.0*60.0), 210, MST_JX_TRANSEARTH_5);
		break;
	case MST_JX_TRANSEARTH_5: //CSM SV update to MCC-6 update
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->GETEval2(rtcc->calcParams.EI - 23.0 * 3600.0 - 20.0 * 60.0), 5, MST_JX_TRANSEARTH_6);
		break;
	case MST_JX_TRANSEARTH_6: //MCC-6 update to Entry PAD update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 212, MST_JX_TRANSEARTH_7);
		break;
	case MST_JX_TRANSEARTH_7: //Entry PAD update to MCC-7 decision update
		UpdateMacro(UTP_PADONLY, PT_AP11ENT, rtcc->GETEval2(rtcc->calcParams.EI - 4.0*3600.0 - 30.0*60.0), 216, MST_JX_TRANSEARTH_8);
		break;
	case MST_JX_TRANSEARTH_8: //MCC-7 decision update to MCC-7 update
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0*60.0, 213, MST_JX_TRANSEARTH_9);
		break;
	case MST_JX_TRANSEARTH_9: //MCC-7 update to Entry PAD update
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0*60.0, 214, MST_JX_TRANSEARTH_10);
		break;
	case MST_JX_TRANSEARTH_10: //Entry PAD update to final entry update
		UpdateMacro(UTP_PADONLY, PT_AP11ENT, rtcc->GETEval2(rtcc->calcParams.EI - 45.0*60.0), 217, MST_JX_TRANSEARTH_11);
		break;
	case MST_JX_TRANSEARTH_11: //Final entry update to CM/SM separation
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
	case MST_JX_ABORT_ORBIT:
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
	case MST_JX_ABORT:
		if (AbortMode == 6)	//Translunar Coast
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
		else if (AbortMode == 7) //Lunar Orbit
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
		else if (AbortMode == 8)
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
	}
}