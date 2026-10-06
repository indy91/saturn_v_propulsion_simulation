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

#pragma once

#include <vector>

typedef struct
{
	double mass;	// current propellant mass
} TankSpec;

typedef struct
{
	double maxth0;
	double isp0;
	double level;
	TankSpec* tank;	// Propellant resource for this thruster
} ThrustSpec;

class LiquidEngine
{
public:
	LiquidEngine();

	virtual void Timestep(double simdt) = 0;

	void SetEngineStart(bool s) { EngineStart = s; }
	void SetEngineStop(bool s) { EngineStop = s; }

	double GetThrustLevel() { return ThrustLevel; }

	void Reset();
protected:
	bool EngineStart;
	bool EngineStop;
	bool EngineRunning;
	double ThrustTimer;
	double ThrustLevel;
};

class F1Engine : public LiquidEngine
{
public:
	F1Engine();

	void Timestep(double simdt);
};

class SICSystems
{
public:
	SICSystems();

	void Init(TankSpec* mt, ThrustSpec* ts, ThrustSpec* retro, int num_r);

	void Timestep(double simt, double simdt);

	void FireRetrorockets();
	void InboardEngineCutoff();
	void OutboardEnginesCutoff();

	bool PropellantLowLevel();
protected:
	F1Engine engines[5];
	TankSpec* main_tank;
	ThrustSpec* thrusters;
	ThrustSpec* retrorockets;
	int num_retro;

	bool fireRockets;
};

class J2Engine : public LiquidEngine
{
public:
	J2Engine();

	void Timestep(double simdt);
};

class SIISystems
{
public:
	SIISystems();

	void Init(TankSpec* mt, double def_thrust, ThrustSpec* ts, ThrustSpec* ullage, int num_u, ThrustSpec* retro, int num_r);

	void Timestep(double simdt);

	void EngineStartOn();
	void FireUllageRockets();
	void FireRetroRockets();
	void SetPUValve(int state);
	void CenterEngineCutoff();
	void AllEnginesCutoff();

	bool PropellantLowLevel();
protected:
	void SetMixtureRatio(double ratio);

	J2Engine engines[5];
	TankSpec* main_tank;
	ThrustSpec* thrusters;
	ThrustSpec* ullagerockets;
	int num_ullage;
	ThrustSpec* retrorockets;
	int num_retro;

	double J2DefaultThrust;
	bool fireUllageRockets;
	bool fireRetroRockets;
};

class SIVBSystems
{
public:
	SIVBSystems();

	void Init(double def_thrust, ThrustSpec* ts, ThrustSpec* ullage, int num_u);

	void Timestep(double simdt);

	void SetPUValve(int state);
	void EngineStartOn();
	void FireUllageRockets();

	void Reset();
protected:
	void SetMixtureRatio(double ratio);

	J2Engine engine;
	ThrustSpec* thruster;
	ThrustSpec* ullagerockets;
	int num_ullage;
	bool fireUllageRockets;
	double J2DefaultThrust;
};

struct SaturnVPropulsionSimulationInputs
{
	SaturnVPropulsionSimulationInputs();

	// MASSES
	double SI_EmptyMass;					// S-IC empty mass, kg
	double SII_EmptyMass;					// S-II empty mass, kg
	double SIVB_EmptyMass;					// S-IVB empty mass (including IU and SLA), kg

	double SICMainPropellantMass;			// kg
	double SICRetroRocketPropellantMass;	// Retro Rocket propellant per rocket, kg

	double SIIMainPropellantMass;			// kg
	double SIIInterstageMass;
	double SIIUllageRocketPropellantMass;
	double SIIRetroRocketPropellantMass;

	double SIVBMainPropellantMass;
	double SIVBUllageRocketPropellantMass;
	double SIVBUllageCaseEmptyMass;

	double TotalJ2FuelLeadMass;				// Approximate total mass of the vehicle at TLI J-2 fuel lead time, kg

	double LETMass;							// Mass of launch escape tower, kg
	double S4PL_Mass;						// Payload mass (CSM + LM), kilograms

	// THRUSTERS
	double THRUST_FIRST_VAC;	// Vacuum thrust of F-1 engine, Newtons
	double ISP_FIRST_VAC;		// 
	double THRUST_SECOND_VAC;	// Vacuum thrust of J-2 engine with 5.0 mixture ratio, Newtons
	double THRUST_THIRD_VAC;	// Vacuum thrust of J-2 engine with 5.0 mixture ratio, Newtons

	double F_SIC_Retro;			// Vacuum thrust of one S-IC retrorocket, Newtons
	double ISP_SIC_Retro;		// Specific impulse of S-IC retrorocket, m/sec
	double F_SII_Ullage;		// Vacuum thrust of one S-II ullage rocket, Newtons
	double ISP_SII_Ullage;		// Specific impulse of S-II ullage rocket, m/sec
	double F_SII_Retro;			// Vacuum thrust of one S-II retrorocket, Newtons
	double ISP_SII_Retro;		// Specific impulse of S-II retrorocket, m/sec
	double F_SIVB_Ullage;		// Vacuum thrust of one S-IVB ullage rocket, Newtons
	double ISP_SIVB_Ullage;		// Specific impulse of S-IVB ullage rocket, m/sec

	// TIMING
	double t_Liftoff;					// Time from first motion to liftoff (TB1)
	double t_SIC_CECO;					// Time from TB1 to S-IC Center-Engine Cutoff
	double t_SII_Ullage;				// Time from TB3 of S-II Ullage Ignition
	double t_SIC_Sep_Cmd;				// Time from TB3 of S-IC/S-II Separation and Retrorocket Motors Command
	double t_SIC_Sep_Delay;				// Time from Separation Command to Separation
	double t_SII_Engine_Start;			// Time from TB3 to S-II Engine Start (Switch Selector channel 33)
	double t_SII_90P_Thrust;			// Time from S-II engines start to S-II engines at 90% thrust
	double t_SII_PU_Valve;				// Time from TB3 to S-II P.U. Valve Commanded to 5.5 Mixture Ratio
	double t_SII_InterstageJettison;	// Time from TB3 to S-IC/S-II Interstage Jettison
	double t_LET_Jettison;				// Time from Interstage Jettison to LET Jettison
	double t_IGM;						// Time from TB3 when Iterative Guidance Mode is started
	double t_SII_CECO;					// Time from TB3 to S-II Center Engine Cutoff
	double t_SII_MRS;					// Time from IGM start when the S-II MRS is commanded
	double t_SIVB_Ullage;				// Time from TB4 to S-IVB ullage ignition
	double t_SII_Sep_Cmd;				// Time from TB4 to S-II/S-IVB Sep and Retrorockets Cmd
	double t_SII_Sep_Delay;				// Time from S-II/S-IVB Separation Command to Separation
	double t_SIVB_Engine_Ignition;		// Time from TB4 to S-IVB engine ignition (Switch Selector channel 11)
	double t_SIVB_90P_Thrust;			// Time from S-IVB ignition to S-IVB engines at 90% thrust

	double t_TLI_Fuel_Lead;				// Time from TB6 to J-2 fuel lead
	double t_TLI_APS_Ullage_Off;		// Time from TB6 when the APS ullage engines are turned off
	double t_TLI_Reignition;			// Time from TB6 of reignition (Switch Selector channel 11)
	double t_TLI_IGM;					// Time from TB6 when IGM is initiated
	double t_TLI_MRS_1st;				// Time from IGM start when TLI MRS is initiated (1st opportunity)
	double t_TLI_MRS_2nd;				// Time from IGM start when TLI MRS is initiated (2nd opportunity)

	// OPTIONS
	int SI_RetroNum;					// Number of S-IC retro rockets (up to 8)
	int SII_UllageNum;					// Number of S-II ullage rockets (up to 8)
	int SII_RetroNum;					// Number of S-II retro rockets (up to 8)
	bool bDoSII_CECO;					// Do the S-II center engine cutoff
	int SIVB_UllageNum;					// Number of S-IVB ullage rockets (up to 4)

	// PRINT OPTIONS
	double DT_MASS_PRINT;				// Nominal time interval between mass data points (sec)
	double DT_PROP_PRINT_NOMINAL;		// Nominal time interval between propulsion data points (sec)
	double DT_PROP_PRINT_DENSE;			// Dense time interval between propulsion data points used during thrust buildup and tailoff (sec)
};

struct PropulsionTapeFiles12Record
{
	// Time (in seconds) from 90-percent thrust
	double T;
	// Vacuum thrust of engines, Newtons
	double F[5];
};

struct PropulsionTapeFiles12
{
	std::vector<PropulsionTapeFiles12Record> Records;
};

struct PropulsionTapeFiles34Record
{
	// Time (in seconds) from 90-percent thrust
	double T;
	// Vacuum thrust of engine, Newtons
	double F;
};

struct PropulsionTapeFiles34
{
	std::vector<PropulsionTapeFiles34Record> Records;
};

struct MassData
{
	double T;
	double M;
};

struct MassDataCards
{
	std::vector<MassData> Data;
};

struct PropulsionTape
{
	// Engine 5 data contains vacuum thrust of engine 5, ullage engines and retro engines. Thrust decay is also included.
	PropulsionTapeFiles12 SIC;
	// Engine 5 data contains vacuum thrust of engine 5, ullage engines and retro engines. Thrust decay is also included.
	PropulsionTapeFiles12 SII;
	// Thrust decay is not included. Includes time history that extends past the nominal first-guidance cutoff signal time GCS1
	PropulsionTapeFiles34 SIVB_FirstBurn;
	// Thrust decay is not included. Includes time history that extends past the nominal second-guidance cutoff signal time GCS2
	PropulsionTapeFiles34 SIVB_SecondBurn;
};

class SaturnVPropulsionSimulation
{
public:
	SaturnVPropulsionSimulation();

	void Calc(const SaturnVPropulsionSimulationInputs& in);
protected:
	void SystemSimulation(double simt, double simdt);

	void UpdatePropellants(double simdt);
	void UpdateThruster(ThrustSpec* ts, double simdt);
	void RecalculateTotalMass();
	double VariableFirstStageMasses() const;

	void CreateSystems();
	void Staging(int NEWSTAGE);
	void ResetForTLI(bool first);

	void EventSequence(int event, double& TI);
	void SetPropDataPrintInterval(bool dense);
	void StoreData();
	void StorePropulsionData();
	void StoreMassData();

	void WritePropellantTape();
	void WriteMassDataCards();
	void WriteLVSPInputs();

	SaturnVPropulsionSimulationInputs inp;

	SICSystems SIC;
	SIISystems SII;
	SIVBSystems SIVB;

	// MASSES
	// These get set to zero at the appropiate time
	double SIIInterstageMass;
	double SIVBUllageCaseEmptyMass;
	double LETMass;

	// PROPELLANTS
	// S-IC
	TankSpec SICMainPropellant;
	TankSpec SICRetroRocketPropellant[8];

	// S-II
	TankSpec SIIMainPropellant;
	TankSpec SIIUllageRocketPropellant[8];
	TankSpec SIIRetroRocketPropellant[8];

	// S-IVB
	TankSpec SIVBMainPropellant;
	TankSpec SIVBUllageRocketPropellant[4];

	// THRUSTERS
	ThrustSpec SICF1Engines[5];
	ThrustSpec SICRetroRockets[8];
	ThrustSpec SIIUllageRockets[8];
	ThrustSpec SIIJ2Engines[5];
	ThrustSpec SIIRetroRockets[8];
	ThrustSpec SIVBUllageRockets[4];
	ThrustSpec SIVBJ2Engine;

	// SIMULATION STATE
	double T;	// Time since first motion
	int ISTAGE;	// 1 = S-IC, 2 = S-II, 3 = S-IVB
	double TotalMass;
	double TB1;
	double TB3;
	double TB4;
	double TLAST_MASS_PRINT;
	double TLAST_PROP_PRINT;
	double DT_MASS_PRINT;
	double TNEXT_MASS_PRINT;
	double DT_PROP_PRINT;
	double TNEXT_PROP_PRINT;
	bool bSpecialStorage;
	int EventCounter;
	bool IEND;
	bool FirstOpportunity;
	bool IsTLI;

	// OUTPUT DATA
	PropulsionTapeFiles12 SICPropulsionData;
	PropulsionTapeFiles12 SIIPropulsionData;
	PropulsionTapeFiles34 SIVBPropulsionData;
	PropulsionTapeFiles34 SIVB1stTLIPropulsionData;
	PropulsionTapeFiles34 SIVB2ndTLIPropulsionData;
	MassDataCards SICMassData;
	MassDataCards SIIMassData;
	MassDataCards SIVBMassData;
	MassDataCards SIVB1stTLIMassData;
	MassDataCards SIVB2ndTLIMassData;

	// CONSTANTS
	const double EPS = 1e-10;
};