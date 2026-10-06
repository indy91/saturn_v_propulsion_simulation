/****************************************************************************
  This file is part of the Saturn V Propulsion Simulation

  Copyright (C) 2026 Niklas Beug

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
  **************************************************************************/

#include <fstream>
#include "saturn_v_propulsion_simulation.h"

#include <wx/fileconf.h>

static const double LBM_TO_KG = 0.45359237;
static const double LBF_TO_N = 4.4482216152605;

void SetThrusterLevel(ThrustSpec* ts, double level)
{
	if (ts->tank && ts->tank->mass)
	{
		ts->level = level;
	}
}

double GetThrusterThrust(ThrustSpec* ts)
{
	return ts->level * ts->maxth0;
}

void SetThrusterIsp(ThrustSpec* ts, double isp)
{
	ts->isp0 = isp;
}

void SetThrusterMax0(ThrustSpec* ts, double thrust)
{
	ts->maxth0 = thrust;
}

void GetJ2ISP(double ratio, double& isp, double& ThrustAdjust)
{
	const double G = 9.81; // This is the value Orbiter uses

#define MR_STATS 5

	static double MixtureRatios[MR_STATS] = { 6.0, 5.5, 5.0, 4.3, 4.0 };
	//static double MRISP[MR_STATS] = { 416*G, 418*G, 421*G, 427*G, 432*G };
	static double MRISP[MR_STATS] = { 421.4 * G, 423.4 * G, 426.5 * G, 432.6 * G, 437.6 * G };
	static double MRThrust[MR_STATS] = { 1.1, 1.0, .898, .7391, .7 };


	isp = 421 * G;

	// From Usenet:
	// It had roughly three stops. 178,000 lbs at 425s Isp and an O/F of 4.5,
	// 207,000 lbs at 421s Isp and an O/F of 5.0, and 230,500 lbs at 418s Isp
	// and an O/F of 5.5.

	for (int i = 0; i < MR_STATS; i++) {
		if (ratio >= MixtureRatios[i]) {
			double delta = (ratio - MixtureRatios[i]) / (MixtureRatios[i - 1] - MixtureRatios[i]);

			isp = MRISP[i] + ((MRISP[i - 1] - MRISP[i]) * delta);
			ThrustAdjust = MRThrust[i] + ((MRThrust[i - 1] - MRThrust[i]) * delta);

			return;
		}
	}
}

LiquidEngine::LiquidEngine()
{
	Reset();
}

void LiquidEngine::Reset()
{
	EngineStart = false;
	EngineStop = false;
	EngineRunning = false;
	ThrustTimer = 0.0;
	ThrustLevel = 0.0;
}

F1Engine::F1Engine()
{

}

void F1Engine::Timestep(double simdt)
{
	if (EngineStop)
	{
		if (ThrustLevel > 0.0)
		{
			EngineStart = false;
			EngineRunning = false;

			double tm_1, tm_2, tm_3;

			ThrustTimer += simdt;

			tm_1 = 0.1;
			tm_2 = 0.3;
			tm_3 = 0.7;

			if (ThrustTimer >= tm_1)
			{
				if (ThrustTimer < tm_2)
				{
					ThrustLevel = std::min(ThrustLevel, 1.0 - 3.35 * (ThrustTimer - tm_1));
				}
				else
				{
					if (ThrustTimer < tm_3)
					{
						ThrustLevel = std::min(ThrustLevel, 0.33 - 0.825 * (ThrustTimer - tm_2));
					}
					else
					{
						ThrustLevel = 0;
					}
				}
			}
			else
			{
				ThrustLevel = std::min(ThrustLevel, 1.0);
			}
		}
	}
	else if (EngineStart && !EngineRunning)
	{
		double tm_1, tm_2, tm_3, tm_4;

		ThrustTimer += simdt;

		tm_1 = -3.315 + 7.0;
		tm_2 = tm_1 + 0.3;  // Start of 1st rise
		tm_3 = tm_2 + 0.85; // Start of 2nd rise
		tm_4 = tm_3 + 0.75; // End of 2nd rise
		if (ThrustTimer >= tm_1) {
			// Light CC
			if (ThrustTimer < tm_2) {
				// Idle at 2.5% thrust
				ThrustLevel = 0.025;
			}
			else {
				if (ThrustTimer < tm_3) {
					// Rise to 93% at a rate of 106 percent per second
					ThrustLevel = 0.025 + (1.06 * (ThrustTimer - tm_2));
				}
				else {
					if (ThrustTimer < tm_4) {
						// Rise to 100% at a rate of 9 percent per second.
						ThrustLevel = 0.93 + (0.09 * (ThrustTimer - tm_3));
					}
					else {
						// Hold 100%
						ThrustLevel = 1;
						EngineRunning = true;
					}
				}
			}
		}
	}
	else if (ThrustTimer > 0.0)
	{
		ThrustTimer = 0.0;
	}
}

SICSystems::SICSystems()
{
	main_tank = NULL;
	thrusters = NULL;
	retrorockets = NULL;
	num_retro = 0;
	fireRockets = false;
}

void SICSystems::Init(TankSpec* mt, ThrustSpec* ts, ThrustSpec* retro, int num_r)
{
	main_tank = mt;
	thrusters = ts;
	retrorockets = retro;
	num_retro = num_r;
}

void SICSystems::Timestep(double simt, double simdt)
{
	// Preflight
	if (simt < 0.0)
	{
		if (simt > -7.0)
		{
			engines[4].SetEngineStart(true);
		}
		if (simt > -6.72)
		{
			engines[0].SetEngineStart(true);
			engines[2].SetEngineStart(true);
		}
		if (simt > -6.3)
		{
			engines[1].SetEngineStart(true);
			engines[3].SetEngineStart(true);
		}
	}

	for (int i = 0; i < 5; i++)
	{
		engines[i].Timestep(simdt);
	}

	// Sync thrust levels
	for (int i = 0; i < 5; i++)
	{
		SetThrusterLevel(&thrusters[i], engines[i].GetThrustLevel());
	}
	if (fireRockets)
	{
		for (int i = 0; i < num_retro; i++)
		{
			SetThrusterLevel(&retrorockets[i], 1.0);
		}
	}
	
}

void SICSystems::FireRetrorockets()
{
	fireRockets = true;
}

void SICSystems::InboardEngineCutoff()
{
	engines[4].SetEngineStop(true);
}

void SICSystems::OutboardEnginesCutoff()
{
	engines[0].SetEngineStop(true);
	engines[1].SetEngineStop(true);
	engines[2].SetEngineStop(true);
	engines[3].SetEngineStop(true);
}

bool SICSystems::PropellantLowLevel()
{
	if (main_tank->mass < 30000.0)
	{
		return true;
	}
	return false;
}

J2Engine::J2Engine()
{

}

void J2Engine::Timestep(double simdt)
{
	if (EngineStop)
	{
		if (ThrustLevel > 0.0)
		{
			ThrustTimer += simdt;

			// Cutoff transient thrust
			if (ThrustTimer < 0.25) {
				// 95% of thrust dies in the first .25 second
				ThrustLevel = 1.0 - (ThrustTimer * 3.3048);
			}
			else {
				if (ThrustTimer < 1.5) {
					// The remainder dies over the next 1.25 second
					ThrustLevel = 0.1738 - ((ThrustTimer - 0.25) * 0.1390);
				}
				else {
					// Engine is completely shut down at 1.5 second
					ThrustLevel = 0.0;
					EngineRunning = false;
					ThrustTimer = 0.0;
				}
			}
		}
	}
	else if (EngineStart && !EngineRunning)
	{
		ThrustTimer += simdt;

		if (ThrustTimer < 1.0)
		{
			ThrustLevel = 0.0;
		}
		else if (ThrustTimer < 2.7) // was 3.2
		{
			ThrustLevel = (ThrustTimer - 1.0) * 0.5882; // Was 0.45
		}
		else
		{
			ThrustLevel = 1.0;
			EngineRunning = true;
			ThrustTimer = 0.0;
		}
	}
	else if (ThrustTimer > 0.0)
	{
		ThrustTimer = 0.0;
	}
}

SIISystems::SIISystems()
{
	fireUllageRockets = false;
	fireRetroRockets = false;
	num_ullage = 0;
	num_retro = 0;
	thrusters = NULL;
	ullagerockets = NULL;
	retrorockets = NULL;
	J2DefaultThrust = 0.0;
	main_tank = NULL;
}

void SIISystems::Init(TankSpec* mt, double def_thrust, ThrustSpec* ts, ThrustSpec* ullage, int num_u, ThrustSpec* retro, int num_r)
{
	J2DefaultThrust = def_thrust;
	main_tank = mt;
	thrusters = ts;
	ullagerockets = ullage;
	num_ullage = num_u;
	retrorockets = retro;
	num_retro = num_r;
}

void SIISystems::Timestep(double simdt)
{
	int i;

	for (i = 0; i < 5; i++)
	{
		engines[i].Timestep(simdt);
	}

	if (fireUllageRockets)
	{
		for (i = 0; i < num_ullage; i++)
		{
			SetThrusterLevel(&ullagerockets[i], 1.0);
		}
	}
	if (fireRetroRockets)
	{
		for (i = 0; i < num_retro; i++)
		{
			SetThrusterLevel(&retrorockets[i], 1.0);
		}
	}

	// Sync thrust levels
	for (i = 0; i < 5; i++)
	{
		SetThrusterLevel(&thrusters[i], engines[i].GetThrustLevel());
	}
}

void SIISystems::EngineStartOn()
{
	for (int i = 0; i < 5; i++)
	{
		engines[i].SetEngineStart(true);
	}
}

void SIISystems::FireUllageRockets()
{
	fireUllageRockets = true;
}

void SIISystems::FireRetroRockets()
{
	fireRetroRockets = true;
}

void SIISystems::SetPUValve(int state)
{
	double MixtureRatio;

	if (state == 0)
	{
		MixtureRatio = 5.5;
	}
	else if (state == 2)
	{
		MixtureRatio = 4.5;
	}
	else
	{
		MixtureRatio = 5.0;
	}

	SetMixtureRatio(MixtureRatio);
}

void SIISystems::CenterEngineCutoff()
{
	engines[4].SetEngineStop(true);
}

void SIISystems::AllEnginesCutoff()
{
	for (int i = 0; i < 5; i++)
	{
		engines[i].SetEngineStop(true);
	}
}

bool SIISystems::PropellantLowLevel()
{
	if (main_tank->mass < 7368.0)
	{
		return true;
	}
	return false;
}

void SIISystems::SetMixtureRatio(double ratio)
{
	double isp, thrust, ThrustAdjust;

	GetJ2ISP(ratio, isp, ThrustAdjust);
	thrust = J2DefaultThrust * ThrustAdjust;

	for (int i = 0; i < 5; i++) {
		SetThrusterIsp(&thrusters[i], isp);
		SetThrusterMax0(&thrusters[i], thrust);
	}
}

SIVBSystems::SIVBSystems()
{
	ullagerockets = NULL;
	num_ullage = 0;
	fireUllageRockets = false;
	thruster = NULL;
	J2DefaultThrust = 0.0;
}

void SIVBSystems::Init(double def_thrust, ThrustSpec* ts, ThrustSpec* ullage, int num_u)
{
	J2DefaultThrust = def_thrust;
	thruster = ts;
	ullagerockets = ullage;
	num_ullage = num_u;
}

void SIVBSystems::Timestep(double simdt)
{
	int i;

	engine.Timestep(simdt);

	SetThrusterLevel(thruster, engine.GetThrustLevel());

	if (fireUllageRockets)
	{
		for (i = 0; i < num_ullage; i++)
		{
			SetThrusterLevel(&ullagerockets[i], 1.0);
		}
	}
}

void SIVBSystems::SetPUValve(int state)
{
	double MixtureRatio;

	if (state == 0)
	{
		MixtureRatio = 5.5;
	}
	else if (state == 2)
	{
		MixtureRatio = 4.5;
	}
	else
	{
		MixtureRatio = 4.946;
	}

	SetMixtureRatio(MixtureRatio);
}

void SIVBSystems::EngineStartOn()
{
	engine.SetEngineStart(true);
}

void SIVBSystems::FireUllageRockets()
{
	fireUllageRockets = true;
}

void SIVBSystems::Reset()
{
	fireUllageRockets = false;
	engine.Reset();
}

void SIVBSystems::SetMixtureRatio(double ratio)
{
	double isp, thrust, ThrustAdjust;

	GetJ2ISP(ratio, isp, ThrustAdjust);
	thrust = J2DefaultThrust * ThrustAdjust;

	SetThrusterIsp(thruster, isp);
	SetThrusterMax0(thruster, thrust);
}

SaturnVPropulsionSimulationInputs::SaturnVPropulsionSimulationInputs()
{
	SI_EmptyMass = 133602.0;
	SII_EmptyMass = 39869.0;
	SIVB_EmptyMass = 13439.0;

	//FirstStageInitialMass = 2906291.0;
	//SecondStageInitialMass = 659185.0;
	//ThirdStageInitialMass = 166427.0;

	SICMainPropellantMass = 2146040.0;
	SICRetroRocketPropellantMass = 1027.0 / 8.0;

	SIIMainPropellantMass = 443500.0;
	SIIInterstageMass = 3969.0;
	SIIUllageRocketPropellantMass = 336.0 * 0.45359237;
	SIIRetroRocketPropellantMass = 121.65;

	SIVBMainPropellantMass = 107428.0;
	SIVBUllageRocketPropellantMass = 58.8 * LBM_TO_KG;
	SIVBUllageCaseEmptyMass = 62.0;

	LETMass = 4051.0;
	S4PL_Mass = 45743.0;

	TotalJ2FuelLeadMass = 134446.75;

	THRUST_FIRST_VAC = 7835000.0;
	ISP_FIRST_VAC = 2979.4;
	THRUST_SECOND_VAC = 1023000.0;
	THRUST_THIRD_VAC = 1023000.0;
	F_SIC_Retro = 75800.0 * 4.4482216152605;
	ISP_SIC_Retro = 2300.6;
	F_SII_Ullage = 23000.0 * 4.4482216152605;
	ISP_SII_Ullage = 2517.3;
	F_SII_Retro = 34810.0 * LBF_TO_N;
	ISP_SII_Retro = 2517.3;
	F_SIVB_Ullage = 3390.0 * 4.4482216152605;
	ISP_SIVB_Ullage = 2517.3;

	t_Liftoff = 0.398;
	t_SIC_CECO = 134.6;
	t_SII_Ullage = 0.5;
	t_SIC_Sep_Cmd = 0.7;
	t_SIC_Sep_Delay = 0.07;
	t_SII_Engine_Start = 1.4;
	t_SII_90P_Thrust = 3.0;
	t_SII_PU_Valve = 6.9;
	t_SII_InterstageJettison = 30.7;
	t_LET_Jettison = 5.7;
	t_IGM = 40.6;
	t_SII_CECO = 299.0;
	t_SII_MRS = 287.1;
	t_SIVB_Ullage = 0.7;
	t_SII_Sep_Cmd = 0.8;
	t_SII_Sep_Delay = 0.075;
	t_SIVB_Engine_Ignition = 4.0;
	t_SIVB_90P_Thrust = 2.5;

	t_TLI_Fuel_Lead = 570.0;
	t_TLI_APS_Ullage_Off = 573.0;
	t_TLI_Reignition = 578.0;
	t_TLI_IGM = 584.0;
	t_TLI_MRS_1st = 111.0;
	t_TLI_MRS_2nd = 1.0;

	SI_RetroNum =  8;
	SII_UllageNum = 4;
	SII_RetroNum = 4;
	bDoSII_CECO = true;
	SIVB_UllageNum = 2;

	DT_MASS_PRINT = 5.0;
	DT_PROP_PRINT_NOMINAL = 10.0;
	DT_PROP_PRINT_DENSE = 0.1;
}

SaturnVPropulsionSimulation::SaturnVPropulsionSimulation()
{
	TotalMass = 0.0;
	TB1 = TB3 = TB4 = 0.0;
	T = 0.0;
	ISTAGE = 0;
	TLAST_MASS_PRINT = TLAST_PROP_PRINT = DT_MASS_PRINT = TNEXT_MASS_PRINT = DT_PROP_PRINT = TNEXT_PROP_PRINT = 0.0;
	bSpecialStorage = IEND = false;
	EventCounter = 0;
	LETMass = SIIInterstageMass = 0.0;
	FirstOpportunity = false;
	IsTLI = false;
}

void SaturnVPropulsionSimulation::Calc(const SaturnVPropulsionSimulationInputs& in)
{
	inp = in;

	CreateSystems();

	double DT, STEP, TE, TI;
	bool SICPropellantDepletion, SIIPropellantDepletion;

	T = -7.0;
	STEP = 0.01;
	ISTAGE = 1;
	EventCounter = 0;
	DT_MASS_PRINT = inp.DT_MASS_PRINT;
	DT_PROP_PRINT = inp.DT_PROP_PRINT_NOMINAL;
	SICPropellantDepletion = false;
	SIIPropellantDepletion = false;
	TLAST_PROP_PRINT = TLAST_MASS_PRINT = -1000000000.0;
	bSpecialStorage = false;
	IsTLI = false;
	IEND = false;

	// Normal end of step
	TE = T + STEP;
	// First event, first motion
	TI = 0.0;
	TNEXT_MASS_PRINT = TNEXT_PROP_PRINT = 0.0;

	// Store initial data
	StoreData();

	while (IEND == false)
	{
		// Determine step length
		// Print?
		if (TE > TNEXT_MASS_PRINT)
		{
			TE = TNEXT_MASS_PRINT;
		}
		if (TE > TNEXT_PROP_PRINT)
		{
			TE = TNEXT_PROP_PRINT;
		}
		if (TI < TE)
		{
			TE = TI;
		}

		DT = TE - T;

		// System simulation
		SystemSimulation(T, DT);
		T += DT;

		// Time for a print?
		if (bSpecialStorage)
		{
			StoreData();
			bSpecialStorage = false;
		}

		if (abs(T - TNEXT_MASS_PRINT) < EPS)
		{
			StoreMassData();
			TNEXT_MASS_PRINT = TNEXT_MASS_PRINT + DT_MASS_PRINT;
		}
		if (abs(T - TNEXT_PROP_PRINT) < EPS)
		{
			StorePropulsionData();
			TNEXT_PROP_PRINT = TNEXT_PROP_PRINT + DT_PROP_PRINT;
		}

		// Additional event detection
		if (ISTAGE == 1 && SICPropellantDepletion == false && SIC.PropellantLowLevel())
		{
			TI = T;
			SICPropellantDepletion = true;
		}
		if (ISTAGE == 2 && SIIPropellantDepletion == false && SII.PropellantLowLevel())
		{
			TI = T;
			SIIPropellantDepletion = true;
		}

		// Event
		if (abs(TI - T) < EPS)
		{
			EventSequence(EventCounter, TI);
			EventCounter++;
		}

		TE = T + STEP;
	}

	WritePropellantTape();
	WriteMassDataCards();

	// Also collect inputs for LVSP
	WriteLVSPInputs();
}

void SaturnVPropulsionSimulation::SystemSimulation(double simt, double simdt)
{
	if (ISTAGE <= 1)
	{
		SIC.Timestep(simt, simdt);
	}
	if (ISTAGE <= 2)
	{
		SII.Timestep(simdt);
	}
	SIVB.Timestep(simdt);

	// Update thruster/propellant
	UpdatePropellants(simdt);
}

void SaturnVPropulsionSimulation::UpdatePropellants(double simdt)
{
	int i;

	if (ISTAGE <= 1)
	{
		// S-IC
		for (i = 0; i < 5; i++)
		{
			UpdateThruster(&SICF1Engines[i], simdt);
		}
		for (i = 0; i < inp.SI_RetroNum; i++)
		{
			UpdateThruster(&SICRetroRockets[i], simdt);
		}
	}
	if (ISTAGE <= 2)
	{
		// S-II
		for (i = 0; i < 5; i++)
		{
			UpdateThruster(&SIIJ2Engines[i], simdt);
		}
		for (i = 0; i < inp.SII_UllageNum; i++)
		{
			UpdateThruster(&SIIUllageRockets[i], simdt);
		}
		for (i = 0; i < inp.SII_RetroNum; i++)
		{
			UpdateThruster(&SIIRetroRockets[i], simdt);
		}
	}
	if (ISTAGE <= 3)
	{
		UpdateThruster(&SIVBJ2Engine, simdt);
		for (i = 0; i < inp.SIVB_UllageNum; i++)
		{
			UpdateThruster(&SIVBUllageRockets[i], simdt);
		}
	}
}

void SaturnVPropulsionSimulation::RecalculateTotalMass()
{
	int i;

	// Third stage
	TotalMass = inp.SIVB_EmptyMass + SIVBMainPropellant.mass + SIVBUllageCaseEmptyMass + LETMass + inp.S4PL_Mass;
	for (i = 0; i < inp.SIVB_UllageNum; i++)
	{
		TotalMass += SIVBUllageRocketPropellant[i].mass;
	}

	if (ISTAGE <= 2)
	{
		// Second stage
		TotalMass += inp.SII_EmptyMass + SIIInterstageMass + SIIMainPropellant.mass;
		
		for (i = 0; i < inp.SII_RetroNum; i++)
		{
			TotalMass += SIIRetroRocketPropellant[i].mass;
		}
		for (i = 0; i < inp.SII_UllageNum; i++)
		{
			TotalMass += SIIUllageRocketPropellant[i].mass;
		}
	}
	if (ISTAGE <= 1)
	{
		// First stage
		TotalMass += inp.SI_EmptyMass;
		TotalMass += VariableFirstStageMasses();
	}
}

double SaturnVPropulsionSimulation::VariableFirstStageMasses() const
{
	double dTemp;

	dTemp = SICMainPropellant.mass;

	for (int i = 0; i < inp.SI_RetroNum; i++)
	{
		dTemp += SICRetroRocketPropellant[i].mass;
	}
	return dTemp;
}

void SaturnVPropulsionSimulation::UpdateThruster(ThrustSpec* thruster, double simdt)
{
	TankSpec* ts;
	double th;

	if (thruster->level = std::max(0.0, std::min(1.0, thruster->level)))
	{
		if ((ts = thruster->tank) && ts->mass) // fuel available?
		{
			th = thruster->maxth0 * thruster->level;
			ts->mass -= th / thruster->isp0 * simdt;
			if (ts->mass < 0.0) ts->mass = 0.0;
		}
		else thruster->level = 0.0; // No fuel
	}
}

void SaturnVPropulsionSimulation::CreateSystems()
{	
	int i;

	// Initial jettisonable masses
	SIIInterstageMass = inp.SIIInterstageMass;
	SIVBUllageCaseEmptyMass = inp.SIVBUllageCaseEmptyMass;
	LETMass = inp.LETMass;

	// Propellants
	TankSpec temp;
	// S-IC
	temp.mass = inp.SICMainPropellantMass;
	SICMainPropellant = temp;

	temp.mass = inp.SICRetroRocketPropellantMass;
	for (i = 0; i < inp.SI_RetroNum; i++)
	{
		SICRetroRocketPropellant[i] = temp;
	}

	// S-II
	temp.mass = inp.SIIMainPropellantMass;
	SIIMainPropellant = temp;

	temp.mass = inp.SIIUllageRocketPropellantMass;
	for (i = 0; i < inp.SII_UllageNum; i++)
	{
		SIIUllageRocketPropellant[i] = temp;
	}

	temp.mass = inp.SIIRetroRocketPropellantMass;
	for (i = 0; i < inp.SII_RetroNum; i++)
	{
		SIIRetroRocketPropellant[i] = temp;
	}

	// S-IVB
	temp.mass = inp.SIVBMainPropellantMass;
	SIVBMainPropellant = temp;

	temp.mass = inp.SIVBUllageRocketPropellantMass;
	for (i = 0; i < inp.SIVB_UllageNum; i++)
	{
		SIVBUllageRocketPropellant[i] = temp;
	}

	// Thrusters
	for (i = 0; i < 5; i++)
	{
		SICF1Engines[i].maxth0 = inp.THRUST_FIRST_VAC;
		SICF1Engines[i].isp0 = inp.ISP_FIRST_VAC;
		SICF1Engines[i].level = 0.0;
		SICF1Engines[i].tank = &SICMainPropellant;
	}
	for (i = 0; i < inp.SI_RetroNum; i++)
	{
		SICRetroRockets[i].maxth0 = inp.F_SIC_Retro;
		SICRetroRockets[i].isp0 = inp.ISP_SIC_Retro;
		SICRetroRockets[i].level = 0.0;
		SICRetroRockets[i].tank = &SICRetroRocketPropellant[i];
	}

	double isp, ThrustAdjust;
	GetJ2ISP(5.0, isp, ThrustAdjust);

	for (int i = 0; i < 5; i++)
	{
		SIIJ2Engines[i].maxth0 = inp.THRUST_SECOND_VAC * ThrustAdjust;
		SIIJ2Engines[i].isp0 = isp;
		SIIJ2Engines[i].level = 0.0;
		SIIJ2Engines[i].tank = &SIIMainPropellant;
	}
	for (i = 0; i < inp.SII_UllageNum; i++)
	{
		SIIUllageRockets[i].maxth0 = inp.F_SII_Ullage;
		SIIUllageRockets[i].isp0 = inp.ISP_SII_Ullage;
		SIIUllageRockets[i].level = 0.0;
		SIIUllageRockets[i].tank = &SIIUllageRocketPropellant[i];
	}
	for (i = 0; i < inp.SII_RetroNum; i++)
	{
		SIIRetroRockets[i].maxth0 = inp.F_SII_Retro;
		SIIRetroRockets[i].isp0 = inp.ISP_SII_Retro;
		SIIRetroRockets[i].level = 0.0;
		SIIRetroRockets[i].tank = &SIIRetroRocketPropellant[i];
	}

	for (i = 0; i < inp.SIVB_UllageNum; i++)
	{
		SIVBUllageRockets[i].maxth0 = inp.F_SIVB_Ullage;
		SIVBUllageRockets[i].isp0 = inp.ISP_SIVB_Ullage;
		SIVBUllageRockets[i].level = 0.0;
		SIVBUllageRockets[i].tank = &SIVBUllageRocketPropellant[i];
	}

	GetJ2ISP(4.93, isp, ThrustAdjust); // TBD: Make this an input?
	SIVBJ2Engine.maxth0 = inp.THRUST_THIRD_VAC * ThrustAdjust;
	SIVBJ2Engine.isp0 = isp;
	SIVBJ2Engine.level = 0.0;
	SIVBJ2Engine.tank = &SIVBMainPropellant;

	// Initialize stage systems
	SIC.Init(&SICMainPropellant, SICF1Engines, SICRetroRockets, inp.SI_RetroNum);
	SII.Init(&SIIMainPropellant, inp.THRUST_SECOND_VAC, SIIJ2Engines, SIIUllageRockets, inp.SII_UllageNum, SIIRetroRockets, inp.SII_RetroNum);
	SIVB.Init(inp.THRUST_THIRD_VAC, &SIVBJ2Engine, SIVBUllageRockets, inp.SIVB_UllageNum);
}

void SaturnVPropulsionSimulation::Staging(int NEWSTAGE)
{
	ISTAGE = NEWSTAGE;

	if (ISTAGE == 2)
	{
		double t_90P;
		t_90P = TB3 + inp.t_SII_Engine_Start + inp.t_SII_90P_Thrust;

		TNEXT_MASS_PRINT = t_90P;
	}
	else if (ISTAGE == 3)
	{
		double t_90P;
		t_90P = TB4 + inp.t_SII_Engine_Start + inp.t_SII_90P_Thrust;

		TNEXT_MASS_PRINT = t_90P;
	}
}

void SaturnVPropulsionSimulation::ResetForTLI(bool first)
{
	T = 0.0;
	SIIInterstageMass = SIVBUllageCaseEmptyMass = LETMass = 0.0;
	for (int i = 0; i < 4; i++)
	{
		SIVBUllageRocketPropellant[i].mass = 0.0;
	}
	ISTAGE = 3;

	// Calculate total mass
	RecalculateTotalMass();

	// Calculate difference to predicted
	double Delta_Mass = TotalMass - inp.TotalJ2FuelLeadMass;

	// Set difference into S-IVB mass
	SIVBMainPropellant.mass -= Delta_Mass;

	TLAST_PROP_PRINT = TLAST_MASS_PRINT = -1000000000.0;
	bSpecialStorage = false;
	DT_MASS_PRINT = inp.DT_MASS_PRINT;
	DT_PROP_PRINT = inp.DT_PROP_PRINT_DENSE;
	TNEXT_MASS_PRINT = TNEXT_PROP_PRINT = 0.0;

	FirstOpportunity = first;
	IsTLI = true;

	SIVB.Reset();
}

void SaturnVPropulsionSimulation::EventSequence(int event, double& TI)
{
	switch (event)
	{
	case 0: // First motion

		// Next event, liftoff (TB1)
		TI = inp.t_Liftoff;
		break;
	case 1: // Liftoff
		// Next event, S-IC Center Engine Cutoff
		TB1 = T;
		TI = inp.t_Liftoff + inp.t_SIC_CECO;
		break;
	case 2: // S-IC Center Engine Cutoff
		StoreData();
		SIC.InboardEngineCutoff();
		SetPropDataPrintInterval(true);

		// Next event, S-IC Center Engine thrust decay
		TI = T + 0.7;
		break;
	case 3: // S-IC Center Engine thrust decay
		StoreData();
		SetPropDataPrintInterval(false);

		// Next event, OECO
		TI = 1e9;
		break;
	case 4: // S-IC Outer Engine Cutoff
		StoreData();
		SIC.OutboardEnginesCutoff();
		TB3 = T;
		SetPropDataPrintInterval(true);

		// Next event, S-II Ullage Ignition
		TI = TB3 + inp.t_SII_Ullage;
		break;
	case 5: // S-II Ullage Ignition
		SII.FireUllageRockets();
		StorePropulsionData();

		// Next event, S-IC/S-II Separation Command and Retrorocket Motors
		TI = TB3 + inp.t_SIC_Sep_Cmd;
		break;
	case 6: // Sep Command and Retrorocket Motors
		SIC.FireRetrorockets();
		StorePropulsionData();

		// Next event, S-IC/S-II Separation
		TI = TB3 + inp.t_SIC_Sep_Cmd + inp.t_SIC_Sep_Delay;
		break;
	case 7: // S-IC/S-II Separation
		StoreData();
		Staging(2);
		TLAST_PROP_PRINT = TLAST_MASS_PRINT = -100000.0;
		StoreData();

		// Next event, S-II ignition
		TI = TB3 + inp.t_SII_Engine_Start;
		break;
	case 8: // S-II Ignition
		StoreData();

		SII.EngineStartOn();

		// Next event, S-II 90% thrust
		TI = TB3 + inp.t_SII_Engine_Start + inp.t_SII_90P_Thrust;
		break;
	case 9: // S-II 90% thrust
		StorePropulsionData();

		// Next event, S-II PU valve to 5.5
		TI = TB3 + inp.t_SII_PU_Valve;
		break;
	case 10: // S-II PU valve to 5.5
		StoreData();
		SII.SetPUValve(0);
		SetPropDataPrintInterval(false);
		bSpecialStorage = true;

		// Next event, S-II interstage jettison
		TI = TB3 + inp.t_SII_InterstageJettison;
		break;
	case 11: // S-II Interstage Jettison
		StoreData();
		SIIInterstageMass = 0.0;
		bSpecialStorage = true;

		// Next event, LET Jettison
		TI = TB3 + inp.t_SII_InterstageJettison + inp.t_LET_Jettison;
		break;
	case 12: // LET Jettison
		StoreData();
		LETMass = 0.0;
		bSpecialStorage = true;

		if (inp.bDoSII_CECO)
		{
			// Next event, S-II Center Engine Cutoff
			TI = TB3 + inp.t_SII_CECO;
		}
		else
		{
			// Next event, S-II MRS
			EventCounter += 2;
			TI = TB3 + inp.t_IGM + inp.t_SII_MRS;
		}
		break;
	case 13: // S-II CECO
		StoreData();
		SII.CenterEngineCutoff();
		SetPropDataPrintInterval(true);

		// Next event, S-II CECO end of tailoff
		TI = T + 1.5;
		break;
	case 14: // S-II CECO end of tailoff
		SetPropDataPrintInterval(false);
		// Next event, MRS
		TI = TB3 + inp.t_IGM + inp.t_SII_MRS;
		break;
	case 15: // S-II MRS
		StoreData();
		SII.SetPUValve(2);
		bSpecialStorage = true;

		// Next event, cutoff
		TI = 1e9;
		break;
	case 16: // S-II Cutoff
		StoreData();
		SII.AllEnginesCutoff();
		TB4 = T;
		SetPropDataPrintInterval(true);

		// Next event, S-IVB ullage ignition
		TI = TB4 + inp.t_SIVB_Ullage;
		break;
	case 17: // S-IVB ullage ignition
		StoreData();
		SIVB.FireUllageRockets();

		// Next event, sep and retrorockets cmd
		TI = TB4 + inp.t_SII_Sep_Cmd;
		break;
	case 18: // S-IVB sep cmd and retrorockets cmd
		StoreData();
		SII.FireRetroRockets();

		// Next event, S-II/S-IVB separation
		TI = TB4 + inp.t_SII_Sep_Cmd + inp.t_SII_Sep_Delay;
		break;
	case 19: // S-II/S-IVB Separation
		StoreData();
		Staging(3);
		TLAST_PROP_PRINT = TLAST_MASS_PRINT = -100000.0;
		StoreData();

		// Next event, S-IVB ignition
		TI = TB4 + inp.t_SIVB_Engine_Ignition;
		break;
	case 20: // S-IVB ignition
		SIVB.EngineStartOn();
		StorePropulsionData();

		// Next event, 90% thrust
		TI = TB4 + inp.t_SIVB_Engine_Ignition + inp.t_SIVB_90P_Thrust;
		break;
	case 21: // S-IVB 90% thrust
		StorePropulsionData();
		
		// Next event, 100% thrust
		TI = T + 1.0; // 1 second beyond 90% thrust
		break;
	case 22: // S-IVB 100% thrust
		StorePropulsionData();
		SetPropDataPrintInterval(false);

		// Next event, end of data
		TI = T + 200.0;
		break;
	case 23: // End of launch data

		// Set up for TLI sim
		EventCounter = 99;
		ResetForTLI(true);

		// Next event, APS ullage engine cutoff
		TI = inp.t_TLI_APS_Ullage_Off - inp.t_TLI_Fuel_Lead;
		break;
	case 100: // APS ullage engine cutoff

		// Next event, reignition
		TI = inp.t_TLI_Reignition - inp.t_TLI_Fuel_Lead;
		break;
	case 101: // S-IVB reignition
		StoreData();
		SIVB.SetPUValve(2);
		SIVB.EngineStartOn();

		// Next event, 90% thrust
		TI = inp.t_TLI_Reignition - inp.t_TLI_Fuel_Lead + inp.t_SIVB_90P_Thrust;
		break;
	case 102: // S-IVB 90% thrust
		StoreData();
		
		// Next event, 100% thrust
		TI = T + 1.0; // 1 second beyond 90% thrust
		break;
	case 103: // 100% thrust
		SetPropDataPrintInterval(false);

		// Next event, MRS
		TI = inp.t_TLI_IGM - inp.t_TLI_Fuel_Lead + (FirstOpportunity ? inp.t_TLI_MRS_1st : inp.t_TLI_MRS_2nd);
		break;
	case 104: // S-IVB TLI MRS
		StoreData();
		SIVB.SetPUValve(1);
		bSpecialStorage = true;

		// Next event, end of TLI data
		TI = T + SIVBMainPropellant.mass / (SIVBJ2Engine.maxth0 / SIVBJ2Engine.isp0) - 0.001; // Near burnout
		break;
	case 105: // End of TLI data
		StoreData();
		// Fake 100 extra burntime
		T = T + 100.0;
		SIVBMainPropellant.mass -= ((SIVBJ2Engine.maxth0 / SIVBJ2Engine.isp0) * 100.0);
		StoreData();

		if (FirstOpportunity)
		{
			// Set up for 2nd TLI sim
			EventCounter = 99;
			ResetForTLI(false);

			// Next event, APS ullage engine cutoff
			TI = inp.t_TLI_APS_Ullage_Off - inp.t_TLI_Fuel_Lead;
		}
		else
		{
			TI = 1e9;
			IEND = true;
		}

		break;
	}
}

void SaturnVPropulsionSimulation::SetPropDataPrintInterval(bool dense)
{
	if (dense)
	{
		DT_PROP_PRINT = inp.DT_PROP_PRINT_DENSE;
	}
	else
	{
		DT_PROP_PRINT = inp.DT_PROP_PRINT_NOMINAL;
	}
	TNEXT_PROP_PRINT = T + DT_PROP_PRINT;
}

void SaturnVPropulsionSimulation::StoreData()
{
	StorePropulsionData();
	StoreMassData();
}

void SaturnVPropulsionSimulation::StorePropulsionData()
{
	if (abs(TLAST_PROP_PRINT - T) < EPS) return;
	TLAST_PROP_PRINT = T;

	unsigned int i;

	if (ISTAGE == 1)
	{
		// S-IC
		PropulsionTapeFiles12Record temp;

		temp.T = T;
		for (i = 0; i < 5; i++)
		{
			temp.F[i] = GetThrusterThrust(&SICF1Engines[i]);
		}

		// Add S-IC retro rockets and S-II ullage rockets thrust to engine 5
		for (i = 0; i < inp.SI_RetroNum; i++)
		{
			temp.F[4] -= GetThrusterThrust(&SICRetroRockets[i]);
		}
		for (i = 0; i < inp.SII_UllageNum; i++)
		{
			temp.F[4] += GetThrusterThrust(&SIIUllageRockets[i]);
		}

		// Convert to pounds
		for (i = 0; i < 5; i++)
		{
			temp.F[i] /= LBF_TO_N;
		}
		SICPropulsionData.Records.push_back(temp);
	}
	else if (ISTAGE == 2)
	{
		// S-II
		PropulsionTapeFiles12Record temp;

		double t_90P;
		t_90P = TB3 + inp.t_SII_Engine_Start + inp.t_SII_90P_Thrust;
		temp.T = T - t_90P;

		for (i = 0; i < 5; i++)
		{
			temp.F[i] = GetThrusterThrust(&SIIJ2Engines[i]);
		}

		// Add S-II ullage and retro rockets and S-IVB ullage rockets thrust to engine 5
		for (i = 0; i < inp.SII_UllageNum; i++)
		{
			temp.F[4] += GetThrusterThrust(&SIIUllageRockets[i]);
		}
		for (i = 0; i < inp.SII_RetroNum; i++)
		{
			temp.F[4] -= GetThrusterThrust(&SIIRetroRockets[i]);
		}
		for (i = 0; i < inp.SIVB_UllageNum; i++)
		{
			temp.F[4] += GetThrusterThrust(&SIVBUllageRockets[i]);
		}

		// Convert to pounds
		for (i = 0; i < 5; i++)
		{
			temp.F[i] /= LBF_TO_N;
		}
		SIIPropulsionData.Records.push_back(temp);
	}
	else
	{
		// S-IVB
		PropulsionTapeFiles34Record temp;

		if (IsTLI)
		{
			temp.T = T;
		}
		else
		{
			double t_90P;
			t_90P = TB4 + inp.t_SIVB_Engine_Ignition + inp.t_SIVB_90P_Thrust;
			temp.T = T - t_90P;
		}

		temp.F = GetThrusterThrust(&SIVBJ2Engine);
		for (i = 0; i < inp.SIVB_UllageNum; i++)
		{
			temp.F += GetThrusterThrust(&SIVBUllageRockets[i]);
		}

		// Convert to pounds
		temp.F /= LBF_TO_N;

		if (IsTLI)
		{
			if (FirstOpportunity)
			{
				SIVB1stTLIPropulsionData.Records.push_back(temp);
			}
			else
			{
				SIVB2ndTLIPropulsionData.Records.push_back(temp);
			}
		}
		else
		{
			SIVBPropulsionData.Records.push_back(temp);
		}
	}
}

void SaturnVPropulsionSimulation::StoreMassData()
{
	if (abs(TLAST_MASS_PRINT - T) < EPS) return;
	TLAST_MASS_PRINT = T;

	MassData massTemp;

	RecalculateTotalMass();

	if (ISTAGE == 1)
	{
		// S-IC
		massTemp.T = T;
		massTemp.M = TotalMass / LBM_TO_KG;

		SICMassData.Data.push_back(massTemp);
	}
	else if (ISTAGE == 2)
	{
		// S-II

		double t_90P;
		t_90P = TB3 + inp.t_SII_Engine_Start + inp.t_SII_90P_Thrust;

		massTemp.T = T - t_90P;
		massTemp.M = TotalMass / LBM_TO_KG;

		SIIMassData.Data.push_back(massTemp);
	}
	else
	{
		// S-IVB

		if (IsTLI)
		{
			massTemp.T = T;
		}
		else
		{
			double t_90P;
			t_90P = TB4 + inp.t_SIVB_Engine_Ignition + inp.t_SIVB_90P_Thrust;

			massTemp.T = T - t_90P;
		}

		massTemp.M = TotalMass / LBM_TO_KG;

		if (IsTLI)
		{
			if (FirstOpportunity)
			{
				SIVB1stTLIMassData.Data.push_back(massTemp);
			}
			else
			{
				SIVB2ndTLIMassData.Data.push_back(massTemp);
			}
		}
		else
		{
			SIVBMassData.Data.push_back(massTemp);
		}
	}
}

void SaturnVPropulsionSimulation::WritePropellantTape()
{
	std::ofstream myfile;

	myfile.open("../output/PropulsionTape.txt");

	if (myfile.is_open() == false) return;

	unsigned int i;
	char Buffer[256];

	// S-IC
	for (i = 0; i < SICPropulsionData.Records.size(); i++)
	{
		sprintf(Buffer, "%lf %lf %lf %lf %lf %lf",
			SICPropulsionData.Records[i].T, SICPropulsionData.Records[i].F[0], SICPropulsionData.Records[i].F[1], SICPropulsionData.Records[i].F[2],
			SICPropulsionData.Records[i].F[3], SICPropulsionData.Records[i].F[4], SICPropulsionData.Records[i].F[5]);

		myfile << Buffer << std::endl;
	}

	//EOF
	myfile << "-1" << std::endl;

	// S-II
	for (i = 0; i < SIIPropulsionData.Records.size(); i++)
	{
		sprintf(Buffer, "%lf %lf %lf %lf %lf %lf",
			SIIPropulsionData.Records[i].T, SIIPropulsionData.Records[i].F[0], SIIPropulsionData.Records[i].F[1], SIIPropulsionData.Records[i].F[2],
			SIIPropulsionData.Records[i].F[3], SIIPropulsionData.Records[i].F[4], SIIPropulsionData.Records[i].F[5]);

		myfile << Buffer << std::endl;
	}
	//EOF
	myfile << "-1" << std::endl;

	// S-IVB
	for (i = 0; i < SIVBPropulsionData.Records.size(); i++)
	{
		sprintf(Buffer, "%lf %lf", SIVBPropulsionData.Records[i].T, SIVBPropulsionData.Records[i].F);
		myfile << Buffer << std::endl;
	}
	//EOF
	myfile << "-1" << std::endl;

	for (i = 0; i < SIVB1stTLIPropulsionData.Records.size(); i++)
	{
		sprintf(Buffer, "%lf %lf", SIVB1stTLIPropulsionData.Records[i].T, SIVB1stTLIPropulsionData.Records[i].F);
		myfile << Buffer << std::endl;
	}
	//EOF
	myfile << "-1" << std::endl;

	for (i = 0; i < SIVB2ndTLIPropulsionData.Records.size(); i++)
	{
		sprintf(Buffer, "%lf %lf", SIVB2ndTLIPropulsionData.Records[i].T, SIVB2ndTLIPropulsionData.Records[i].F);
		myfile << Buffer << std::endl;
	}
	//EOF
	myfile << "-1" << std::endl;

	myfile.close();
}

void SaturnVPropulsionSimulation::WriteMassDataCards()
{
	std::ofstream myfile;

	myfile.open("../output/MassDataCards.txt");

	if (myfile.is_open() == false) return;

	MassDataCards* pMasses;
	unsigned int i, j, k, siz, strlen;
	char Buffer[256];

	for (k = 0; k < 5; k++)
	{
		if (k == 0)
		{
			pMasses = &SICMassData;
		}
		else if (k == 1)
		{
			pMasses = &SIIMassData;
		}
		else if(k == 2)
		{
			pMasses = &SIVBMassData;
		}
		else if (k == 3)
		{
			pMasses = &SIVB1stTLIMassData;
		}
		else
		{
			pMasses = &SIVB2ndTLIMassData;
		}

		siz = pMasses->Data.size();
		sprintf(Buffer, "%5d%5d%5d%5d", 37, k + 1, 2, siz);
		myfile << Buffer << std::endl;
		j = 0;
		strlen = 0;

		for (i = 0; i < siz; i++)
		{
			strlen += sprintf(Buffer + strlen, "%10.5lf%10.1lf", pMasses->Data[i].T, pMasses->Data[i].M);
			j++;

			if (j == 3 || (i + 1) == siz)
			{
				myfile << Buffer << std::endl;
				j = 0;
				strlen = 0;
			}
		}
	}

	myfile.close();
}

void SaturnVPropulsionSimulation::WriteLVSPInputs()
{
	double dt_SII_Engines_Start_to_Ignition, dt_SII_Ullage, dt_SIVB_Ullage, t_SIC_CECO, t_TB3_Sep, t_TB3_90P, t_TB4_Sep, t_TB4_90P, t_TB3_LET_Jett;
	double EVAR31, EVAR33, EVAR34, EVAR35;

	dt_SII_Engines_Start_to_Ignition = 1.0;

	// Calculate ullage burn times
	dt_SII_Ullage = inp.SIIUllageRocketPropellantMass / (inp.F_SII_Ullage / inp.ISP_SII_Ullage);
	dt_SIVB_Ullage = inp.SIVBUllageRocketPropellantMass / (inp.F_SIVB_Ullage / inp.ISP_SIVB_Ullage);

	t_SIC_CECO = inp.t_SIC_CECO + inp.t_Liftoff;
	t_TB3_Sep = inp.t_SIC_Sep_Cmd + inp.t_SIC_Sep_Delay;				// Time in TB3 of stage separation
	t_TB3_90P = inp.t_SII_Engine_Start + inp.t_SII_90P_Thrust;			// Time in TB3 of S-II 90% thrust
	t_TB4_Sep = inp.t_SII_Sep_Cmd + inp.t_SII_Sep_Delay;				// Time in TB4 of stage separation
	t_TB4_90P = inp.t_SIVB_Engine_Ignition + inp.t_SIVB_90P_Thrust;		// Time in TB4 of S-IVB 90% thrust
	t_TB3_LET_Jett = inp.t_SII_InterstageJettison + inp.t_LET_Jettison;	// Time in TB3 of LET jettison

	EVAR31 = inp.t_TLI_Fuel_Lead;
	EVAR33 = inp.t_TLI_Reignition;
	EVAR34 = inp.t_TLI_APS_Ullage_Off;
	EVAR35 = (inp.t_TLI_Reignition - inp.t_TLI_Fuel_Lead) + inp.t_SIVB_90P_Thrust;

	wxFileConfig config("Saturn V Propulsion", wxEmptyString, "../output/LVSPInputs.ini", wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

	config.Write("/Basic/DELTSK(2)", inp.t_SII_InterstageJettison - t_TB3_Sep);
	config.Write("/Basic/DTCOST(1)", 0.0);
	config.Write("/Basic/DTCOST(2)", t_TB3_90P - t_TB3_Sep);
	config.Write("/Basic/DTCOST(3)", t_TB4_90P - t_TB4_Sep);
	config.Write("/Basic/DTLES", t_TB3_LET_Jett - t_TB3_Sep);
	config.Write("/Basic/EVAR2", inp.t_Liftoff);
	config.Write("/Basic/EVAR6", t_TB3_Sep);
	config.Write("/Basic/EVAR7", t_TB3_Sep - inp.t_SII_Ullage);
	config.Write("/Basic/EVAR9", inp.t_SIC_Sep_Delay);
	config.Write("/Basic/EVAR10", inp.t_SII_Engine_Start + dt_SII_Engines_Start_to_Ignition - t_TB3_Sep);
	config.Write("/Basic/EVAR11", inp.t_SII_Ullage + dt_SII_Ullage - t_TB3_Sep);
	config.Write("/Basic/EVAR17", t_TB4_Sep);
	config.Write("/Basic/EVAR18", t_TB4_Sep - inp.t_SIVB_Ullage);
	config.Write("/Basic/EVAR20", inp.t_SII_Sep_Delay);
	config.Write("/Basic/EVAR22", inp.t_SIVB_Engine_Ignition - t_TB4_Sep);
	config.Write("/Basic/EVAR23", inp.t_SIVB_Ullage + dt_SIVB_Ullage - t_TB4_Sep);
	config.Write("/Basic/EVAR31", EVAR31);
	config.Write("/Basic/EVAR33", EVAR33);
	config.Write("/Basic/EVAR34", EVAR34);
	config.Write("/Basic/EVAR35", EVAR35);
	config.Write("/Basic/TBURNI(1)", TB3 + t_TB3_Sep);
	config.Write("/Basic/TBURNI(2)", TB4 + t_TB4_Sep);
	config.Write("/MotorData/VARIIN(4)", t_SIC_CECO);
}