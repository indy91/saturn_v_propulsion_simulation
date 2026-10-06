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

#include "saturn_v_propulsion_simulation_gui.h"
#include <wx/valgen.h>
#include <wx/fileconf.h>

enum
{
	ID_Button_Calculate = 1,
	ID_TEXTCTRL,
	ID_CHECKBOX
};

BEGIN_EVENT_TABLE(SaturnPropulsionSimFrame, wxFrame)
EVT_MENU(wxID_OPEN, SaturnPropulsionSimFrame::OnOpen)
EVT_MENU(wxID_SAVE, SaturnPropulsionSimFrame::OnSave)
EVT_BUTTON(ID_Button_Calculate, SaturnPropulsionSimFrame::OnButtonCalculate)
END_EVENT_TABLE()

bool SaturnPropulsionSimApp::OnInit()
{
	// Create the main application window
	SaturnPropulsionSimFrame* frame = new SaturnPropulsionSimFrame("Saturn V Propulsion Simulation");
	// Show it
	frame->Show(true);
	// Start the event loop
	return true;
}

SaturnPropulsionSimFrame::SaturnPropulsionSimFrame(const wxString& title)
	: wxFrame(NULL, wxID_ANY, title)
{
	// Create a menu bar
	wxMenu* fileMenu = new wxMenu;

	fileMenu->Append(wxID_OPEN, "&Open...\tCtrl+O", "");
	fileMenu->Append(wxID_SAVE, "&Save...\tCtrl+S", "");

	// Now append the freshly created menu to the menu bar...
	wxMenuBar* menuBar = new wxMenuBar();
	menuBar->Append(fileMenu, "&File");
	// ... and attach this menu bar to the frame
	SetMenuBar(menuBar);

	// Create a top-level panel to hold all the contents of the frame
	wxPanel* panel = new wxPanel(this, wxID_ANY);

	// Set up the sizer for the panel
	//wxBoxSizer* panelSizer = new wxBoxSizer(wxHORIZONTAL);
	//panel->SetSizer(panelSizer);

	// Set up the sizer for the frame and resize the frame
	// according to its contents
	//wxBoxSizer* topSizer = new wxBoxSizer(wxHORIZONTAL);
	//topSizer->SetMinSize(1366, 768);
	//topSizer->Add(panel, 1, wxEXPAND);
	//SetSizerAndFit(topSizer);

	CreateStatusBar(1);

	// Four columns for the whole page
	wxGridSizer* gridSizer = new wxGridSizer(4, 0, 0);
	panel->SetSizer(gridSizer);

	// Column 1
	wxBoxSizer* column = new wxBoxSizer(wxVERTICAL);

	column->Add(new wxStaticText(panel, wxID_ANY, "MASSES"), 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);
	column->Add(new wxStaticText(panel, wxID_ANY, "S-IC"), 0, wxALIGN_LEFT | wxALL, 5);

	wxFlexGridSizer* flexGridSizer = new wxFlexGridSizer(3, 5, 5);

	CreateTextCtrl(panel, flexGridSizer, "Empty Mass", "kg", "S-IC empty mass at ignition in kg", &inputs.SI_EmptyMass);
	CreateTextCtrl(panel, flexGridSizer, "Main Propellant Mass", "kg", "S-IC propellant mass at ignition in kg", &inputs.SICMainPropellantMass);
	CreateTextCtrl(panel, flexGridSizer, "Retro Propellant Mass", "kg", "S-IC retro rocket propellant per rocket", &inputs.SICRetroRocketPropellantMass);
	column->Add(flexGridSizer);

	column->Add(new wxStaticText(panel, wxID_ANY, "S-II"), 0, wxALIGN_LEFT | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);

	CreateTextCtrl(panel, flexGridSizer, "Empty Mass", "kg", "S-II empty mass at liftoff in kg", &inputs.SII_EmptyMass);
	CreateTextCtrl(panel, flexGridSizer, "S-IC/S-II Interstage", "kg", "Interstage mass in kg (without ullage propellant)", &inputs.SIIInterstageMass);
	CreateTextCtrl(panel, flexGridSizer, "Propellant Mass", "kg", "S-II propellant mass at liftoff in kg", &inputs.SIIMainPropellantMass);
	CreateTextCtrl(panel, flexGridSizer, "Ullage Propellant Mass", "kg", "S-II ullage propellant mass in kg per rocket", &inputs.SIIUllageRocketPropellantMass);
	CreateTextCtrl(panel, flexGridSizer, "Retro Propellant Mass", "kg", "S-II retro rocket propellant mass in kg per rocket", &inputs.SIIRetroRocketPropellantMass);
	column->Add(flexGridSizer);

	column->Add(new wxStaticText(panel, wxID_ANY, "S-IVB"), 0, wxALIGN_LEFT | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);

	CreateTextCtrl(panel, flexGridSizer, "Empty Mass", "kg", "S-IVB empty mass in kg (including IU and SLA)", &inputs.SIVB_EmptyMass);
	CreateTextCtrl(panel, flexGridSizer, "Payload Mass", "kg", "Payload mass in kg (including CSM and LM)", &inputs.S4PL_Mass);
	CreateTextCtrl(panel, flexGridSizer, "Propellant Mass", "kg", "S-IVB main propellant mass in kg", &inputs.SIVBMainPropellantMass);
	CreateTextCtrl(panel, flexGridSizer, "Ullage Propellant Mass", "kg", "S-IVB ullage propellant mass in kg per rocket", &inputs.SIVBUllageRocketPropellantMass);
	CreateTextCtrl(panel, flexGridSizer, "Ullage Case Mass", "kg", "S-IVB ullage case empty mass in kg per rocket", &inputs.SIVBUllageCaseEmptyMass);
	CreateTextCtrl(panel, flexGridSizer, "TLI Weight", "kg", "Approximate total mass of the vehicle at TLI J-2 fuel lead time, kg", &inputs.TotalJ2FuelLeadMass);
	column->Add(flexGridSizer);

	column->Add(new wxButton(panel, ID_Button_Calculate, "Calculate"), 0, wxALIGN_LEFT | wxALL, 5);

	gridSizer->Add(column);

	column = new wxBoxSizer(wxVERTICAL);

	column->Add(new wxStaticText(panel, wxID_ANY, "THRUST"), 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);
	column->Add(new wxStaticText(panel, wxID_ANY, "S-IC"), 0, wxALIGN_LEFT | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "Vacuum Thrust", "N", "Vacuum thrust per F-1 engine in Newton", &inputs.THRUST_FIRST_VAC);
	CreateTextCtrl(panel, flexGridSizer, "Vacuum ISP", "m/s", "Vacuum specific impulse of the F-1 engine in meters per second", &inputs.ISP_FIRST_VAC);
	column->Add(flexGridSizer);

	gridSizer->Add(column);

	column->Add(new wxStaticText(panel, wxID_ANY, "S-II"), 0, wxALIGN_LEFT | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "Vacuum Thrust", "N", "Vacuum thrust per S-II J-2 engine at 5.0 mixture ratio in Newton", &inputs.THRUST_SECOND_VAC);
	column->Add(flexGridSizer);

	column->Add(new wxStaticText(panel, wxID_ANY, "S-IVB"), 0, wxALIGN_LEFT | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "Vacuum Thrust", "N", "Vacuum thrust per S-IVB J-2 engine at 5.0 mixture ratio in Newton", &inputs.THRUST_THIRD_VAC);
	column->Add(flexGridSizer);
	
	column = new wxBoxSizer(wxVERTICAL);
	column->Add(new wxStaticText(panel, wxID_ANY, "LAUNCH TIMES"), 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);

	CreateTextCtrl(panel, flexGridSizer, "Liftoff", "sec", "Time from first motion to liftoff detection (Timebase 1)", &inputs.t_Liftoff);
	CreateTextCtrl(panel, flexGridSizer, "S-IC CECO", "sec", "Time from TB1 to S-IC Center-Engine Cutoff", &inputs.t_SIC_CECO);
	CreateTextCtrl(panel, flexGridSizer, "S-II Ullage", "sec", "Time from TB3 of S-II Ullage Ignition", &inputs.t_SII_Ullage);
	CreateTextCtrl(panel, flexGridSizer, "S-IC Sep Cmd", "sec", "Time from TB3 of S-IC/S-II Separation and Retrorocket Motors Command", &inputs.t_SIC_Sep_Cmd);
	CreateTextCtrl(panel, flexGridSizer, "S-IC Sep Delay", "sec", "Time from Separation Command to Separation", &inputs.t_SIC_Sep_Delay);
	CreateTextCtrl(panel, flexGridSizer, "S-II Engine Start", "sec", "Time from TB3 to S-II Engine Start (Switch Selector channel 33)", &inputs.t_SII_Engine_Start);
	CreateTextCtrl(panel, flexGridSizer, "S-II 90% Thrust", "sec", "Time from S-II ignition to S-II engines at 90% thrust", &inputs.t_SII_90P_Thrust);
	CreateTextCtrl(panel, flexGridSizer, "S-II PU Valve", "sec", "Time from TB3 to S-II P.U. Valve Commanded to 5.5 Mixture Ratio", &inputs.t_SII_PU_Valve);
	CreateTextCtrl(panel, flexGridSizer, "S-II Skirt Jett", "sec", "Time from TB3 to S-IC/S-II Interstage Jettison", &inputs.t_SII_InterstageJettison);
	CreateTextCtrl(panel, flexGridSizer, "LET Jettison", "sec", "Time from Interstage Jettison to LET Jettison", &inputs.t_LET_Jettison);
	CreateTextCtrl(panel, flexGridSizer, "IGM", "sec", "Time from TB3 when Iterative Guidance Mode is started", &inputs.t_IGM);
	CreateTextCtrl(panel, flexGridSizer, "S-II CECO", "sec", "Time from TB3 to S-II Center Engine Cutoff", &inputs.t_SII_CECO);
	CreateTextCtrl(panel, flexGridSizer, "S-II MRS", "sec", "Time from IGM start when the S-II MRS is commanded", &inputs.t_SII_MRS);
	CreateTextCtrl(panel, flexGridSizer, "S-IVB Ullage", "sec", "Time from TB4 to S-IVB ullage ignition", &inputs.t_SIVB_Ullage);
	CreateTextCtrl(panel, flexGridSizer, "S-II Sep Cmd", "sec", "Time from TB4 to S-II/S-IVB Sep and Retrorockets Cmd", &inputs.t_SII_Sep_Cmd);
	CreateTextCtrl(panel, flexGridSizer, "S-II Sep Delay", "sec", "Time from S-II/S-IVB Separation Command to Separation", &inputs.t_SII_Sep_Delay);
	CreateTextCtrl(panel, flexGridSizer, "S-IVB Engine Start", "sec", "Time from TB4 to S-IVB engine ignition (Switch Selector channel 11)", &inputs.t_SIVB_Engine_Ignition);
	CreateTextCtrl(panel, flexGridSizer, "S-IVB 90% Thrust", "sec", "Time from S-IVB ignition to S-IVB engines at 90% thrust", &inputs.t_SIVB_90P_Thrust);
	column->Add(flexGridSizer);
	gridSizer->Add(column);

	column = new wxBoxSizer(wxVERTICAL);
	column->Add(new wxStaticText(panel, wxID_ANY, "TLI TIMES"), 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);
	
	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "Fuel Lead", "sec", "Time from TB6 to J-2 fuel lead", &inputs.t_TLI_Fuel_Lead);
	CreateTextCtrl(panel, flexGridSizer, "Ullage Off", "sec", "Time from TB6 when the APS ullage engines are turned off", &inputs.t_TLI_APS_Ullage_Off);
	CreateTextCtrl(panel, flexGridSizer, "Reignition", "sec", "Time from TB6 of reignition (Switch Selector channel 11)", &inputs.t_TLI_Reignition);
	CreateTextCtrl(panel, flexGridSizer, "IGM", "sec", "Time from TB6 when IGM is initiated", &inputs.t_TLI_IGM);
	CreateTextCtrl(panel, flexGridSizer, "MRS 1st Opp", "sec", "Time from IGM start when TLI MRS is initiated (1st opportunity)", &inputs.t_TLI_MRS_1st);
	CreateTextCtrl(panel, flexGridSizer, "MRS 2nd Opp", "sec", "Time from IGM start when TLI MRS is initiated (2nd opportunity)", &inputs.t_TLI_MRS_2nd);
	column->Add(flexGridSizer);

	column->Add(new wxStaticText(panel, wxID_ANY, "OPTIONS"), 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);
	
	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "S-IC Retro", "", "Number of S-IC retro rockets (up to 8)", &inputs.SI_RetroNum);
	CreateTextCtrl(panel, flexGridSizer, "S-II Ullage", "", "Number of S-II ullage rockets (up to 8)", &inputs.SII_UllageNum);
	CreateTextCtrl(panel, flexGridSizer, "S-II Retro", "", "Number of S-II retro rockets (up to 8)", &inputs.SII_RetroNum);
	column->Add(flexGridSizer);

	wxCheckBox* text_SII_CECO;
	column->Add(text_SII_CECO = new wxCheckBox(panel, ID_CHECKBOX, "S-II CECO", wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT,wxGenericValidator(&inputs.bDoSII_CECO)));
	text_SII_CECO->SetToolTip("S-II performs an early center engine cutoff or not");

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "S-IVB Ullage", "", "Number of S-IVB ullage rockets (up to 4)", &inputs.SIVB_UllageNum);
	column->Add(flexGridSizer);

	column->Add(new wxStaticText(panel, wxID_ANY, "PRINT OPTIONS"), 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 5);

	flexGridSizer = new wxFlexGridSizer(3, 5, 5);
	CreateTextCtrl(panel, flexGridSizer, "Mass Interval", "sec", "Nominal time interval between mass data points", &inputs.DT_MASS_PRINT);
	CreateTextCtrl(panel, flexGridSizer, "Nom. Prop Interval", "sec", "Nominal time interval between propulsion data points", &inputs.DT_PROP_PRINT_NOMINAL);
	CreateTextCtrl(panel, flexGridSizer, "Dense Prop Interval", "sec", "Dense time interval between propulsion data points used during thrust buildup and tailoff", &inputs.DT_PROP_PRINT_DENSE);
	column->Add(flexGridSizer);

	gridSizer->Add(column);

	gridSizer->Fit(this);
	gridSizer->SetSizeHints(this);

	TransferDataToWindow();
}

template <typename T>
void SaturnPropulsionSimFrame::CreateTextCtrl(wxPanel* panel, wxFlexGridSizer* flexGridSizer, const wxString& label, const wxString& unit, const wxString& tooltip, T val) const
{
	flexGridSizer->Add(new wxStaticText(panel, wxID_ANY, label), wxSizerFlags().CenterVertical().Border());

	wxTextCtrl* item1 = new wxTextCtrl(panel, ID_TEXTCTRL);
	item1->SetToolTip(tooltip);
	item1->SetValidator(wxGenericValidator(val));
	flexGridSizer->Add(item1, wxSizerFlags().CenterVertical().Border());

	flexGridSizer->Add(new wxStaticText(panel, wxID_ANY, unit), wxSizerFlags().CenterVertical().Border());
}

void SaturnPropulsionSimFrame::OnOpen(wxCommandEvent& event)
{
	wxFileName fn(wxGetCwd());
	wxString defaultDir = fn.GetPath();

	wxFileDialog dlg(this, "Open file", defaultDir, wxEmptyString, "Ini files (*.ini)|*.ini", wxFD_OPEN | wxFD_FILE_MUST_EXIST);

	if (dlg.ShowModal() == wxID_CANCEL)
	{
		return;
	}

	wxFileConfig config("Saturn V Propulsion", wxEmptyString, dlg.GetPath(), wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

	config.Read("/Masses/SI_EmptyMass", &inputs.SI_EmptyMass);
	config.Read("/Masses/SII_EmptyMass", &inputs.SII_EmptyMass);
	config.Read("/Masses/SIVB_EmptyMass", &inputs.SIVB_EmptyMass);
	config.Read("/Masses/SICMainPropellantMass", &inputs.SICMainPropellantMass);
	config.Read("/Masses/SICRetroRocketPropellantMass", &inputs.SICRetroRocketPropellantMass);
	config.Read("/Masses/SIIMainPropellantMass", &inputs.SIIMainPropellantMass);
	config.Read("/Masses/SIIInterstageMass", &inputs.SIIInterstageMass);
	config.Read("/Masses/SIIUllageRocketPropellantMass", &inputs.SIIUllageRocketPropellantMass);
	config.Read("/Masses/SIIRetroRocketPropellantMass", &inputs.SIIRetroRocketPropellantMass);
	config.Read("/Masses/SIVBMainPropellantMass", &inputs.SIVBMainPropellantMass);
	config.Read("/Masses/SIVBUllageRocketPropellantMass", &inputs.SIVBUllageRocketPropellantMass);
	config.Read("/Masses/SIVBUllageCaseEmptyMass", &inputs.SIVBUllageCaseEmptyMass);
	config.Read("/Masses/TotalJ2FuelLeadMass", &inputs.TotalJ2FuelLeadMass);
	config.Read("/Masses/LETMass", &inputs.LETMass);
	config.Read("/Masses/S4PL_Mass", &inputs.S4PL_Mass);

	config.Read("/Performance/THRUST_FIRST_VAC", &inputs.THRUST_FIRST_VAC);
	config.Read("/Performance/ISP_FIRST_VAC", &inputs.ISP_FIRST_VAC);
	config.Read("/Performance/THRUST_SECOND_VAC", &inputs.THRUST_SECOND_VAC);
	config.Read("/Performance/THRUST_THIRD_VAC", &inputs.THRUST_THIRD_VAC);

	config.Read("/Times/t_Liftoff", &inputs.t_Liftoff);
	config.Read("/Times/t_SIC_CECO", &inputs.t_SIC_CECO);
	config.Read("/Times/t_SII_Ullage", &inputs.t_SII_Ullage);
	config.Read("/Times/t_SIC_Sep_Cmd", &inputs.t_SIC_Sep_Cmd);
	config.Read("/Times/t_SIC_Sep_Delay", &inputs.t_SIC_Sep_Delay);
	config.Read("/Times/t_SII_Engine_Start", &inputs.t_SII_Engine_Start);
	config.Read("/Times/t_SII_90P_Thrust", &inputs.t_SII_90P_Thrust);
	config.Read("/Times/t_SII_PU_Valve", &inputs.t_SII_PU_Valve);
	config.Read("/Times/t_SII_InterstageJettison", &inputs.t_SII_InterstageJettison);
	config.Read("/Times/t_LET_Jettison", &inputs.t_LET_Jettison);
	config.Read("/Times/t_IGM", &inputs.t_IGM);
	config.Read("/Times/t_SII_CECO", &inputs.t_SII_CECO);
	config.Read("/Times/t_SII_MRS", &inputs.t_SII_MRS);
	config.Read("/Times/t_SIVB_Ullage", &inputs.t_SIVB_Ullage);
	config.Read("/Times/t_SII_Sep_Cmd", &inputs.t_SII_Sep_Cmd);
	config.Read("/Times/t_SII_Sep_Delay", &inputs.t_SII_Sep_Delay);
	config.Read("/Times/t_SIVB_Engine_Ignition", &inputs.t_SIVB_Engine_Ignition);
	config.Read("/Times/t_SIVB_90P_Thrust", &inputs.t_SIVB_90P_Thrust);
	config.Read("/Times/t_TLI_Fuel_Lead", &inputs.t_TLI_Fuel_Lead);
	config.Read("/Times/t_TLI_APS_Ullage_Off", &inputs.t_TLI_APS_Ullage_Off);
	config.Read("/Times/t_TLI_Reignition", &inputs.t_TLI_Reignition);
	config.Read("/Times/t_TLI_IGM", &inputs.t_TLI_IGM);
	config.Read("/Times/t_TLI_MRS_1st", &inputs.t_TLI_MRS_1st);
	config.Read("/Times/t_TLI_MRS_2nd", &inputs.t_TLI_MRS_2nd);

	config.Read("/Options/SI_RetroNum", &inputs.SI_RetroNum);
	config.Read("/Options/SII_UllageNum", &inputs.SII_UllageNum);
	config.Read("/Options/SII_RetroNum", &inputs.SII_RetroNum);
	config.Read("/Options/bDoSII_CECO", &inputs.bDoSII_CECO);
	config.Read("/Options/SIVB_UllageNum", &inputs.SIVB_UllageNum);

	// Include file name in title
	SetTitle(dlg.GetPath() + " - Saturn V Propulsion Simulation");

	if (!TransferDataToWindow())
	{
		SetStatusText("Invalid data!");
		return;
	}
	SetStatusText("File loaded.");
}

void SaturnPropulsionSimFrame::OnSave(wxCommandEvent& event)
{
	wxFileName fn(wxGetCwd());
	wxString defaultDir = fn.GetPath();

	wxFileDialog dlg(this, "Save file", defaultDir, wxEmptyString, "Ini files (*.ini)|*.ini", wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

	if (dlg.ShowModal() == wxID_CANCEL)
	{
		return;
	}

	if (!TransferDataFromWindow())
	{
		SetStatusText("Invalid data!");
		return;
	}

	wxFileConfig config("Saturn V Propulsion", wxEmptyString, dlg.GetPath(), wxEmptyString, wxCONFIG_USE_LOCAL_FILE | wxCONFIG_USE_RELATIVE_PATH);

	config.Write("/Masses/SI_EmptyMass", inputs.SI_EmptyMass);
	config.Write("/Masses/SII_EmptyMass", inputs.SII_EmptyMass);
	config.Write("/Masses/SIVB_EmptyMass", inputs.SIVB_EmptyMass);
	config.Write("/Masses/SICMainPropellantMass", inputs.SICMainPropellantMass);
	config.Write("/Masses/SICRetroRocketPropellantMass", inputs.SICRetroRocketPropellantMass);
	config.Write("/Masses/SIIMainPropellantMass", inputs.SIIMainPropellantMass);
	config.Write("/Masses/SIIInterstageMass", inputs.SIIInterstageMass);
	config.Write("/Masses/SIIUllageRocketPropellantMass", inputs.SIIUllageRocketPropellantMass);
	config.Write("/Masses/SIIRetroRocketPropellantMass", inputs.SIIRetroRocketPropellantMass);
	config.Write("/Masses/SIVBMainPropellantMass", inputs.SIVBMainPropellantMass);
	config.Write("/Masses/SIVBUllageRocketPropellantMass", inputs.SIVBUllageRocketPropellantMass);
	config.Write("/Masses/SIVBUllageCaseEmptyMass", inputs.SIVBUllageCaseEmptyMass);
	config.Write("/Masses/TotalJ2FuelLeadMass", inputs.TotalJ2FuelLeadMass);
	config.Write("/Masses/LETMass", inputs.LETMass);
	config.Write("/Masses/S4PL_Mass", inputs.S4PL_Mass);

	config.Write("/Performance/THRUST_FIRST_VAC", inputs.THRUST_FIRST_VAC);
	config.Write("/Performance/ISP_FIRST_VAC", inputs.ISP_FIRST_VAC);
	config.Write("/Performance/THRUST_SECOND_VAC", inputs.THRUST_SECOND_VAC);
	config.Write("/Performance/THRUST_THIRD_VAC", inputs.THRUST_THIRD_VAC);

	config.Write("/Times/t_Liftoff", inputs.t_Liftoff);
	config.Write("/Times/t_SIC_CECO", inputs.t_SIC_CECO);
	config.Write("/Times/t_SII_Ullage", inputs.t_SII_Ullage);
	config.Write("/Times/t_SIC_Sep_Cmd", inputs.t_SIC_Sep_Cmd);
	config.Write("/Times/t_SIC_Sep_Delay", inputs.t_SIC_Sep_Delay);
	config.Write("/Times/t_SII_Engine_Start", inputs.t_SII_Engine_Start);
	config.Write("/Times/t_SII_90P_Thrust", inputs.t_SII_90P_Thrust);
	config.Write("/Times/t_SII_PU_Valve", inputs.t_SII_PU_Valve);
	config.Write("/Times/t_SII_InterstageJettison", inputs.t_SII_InterstageJettison);
	config.Write("/Times/t_LET_Jettison", inputs.t_LET_Jettison);
	config.Write("/Times/t_IGM", inputs.t_IGM);
	config.Write("/Times/t_SII_CECO", inputs.t_SII_CECO);
	config.Write("/Times/t_SII_MRS", inputs.t_SII_MRS);
	config.Write("/Times/t_SIVB_Ullage", inputs.t_SIVB_Ullage);
	config.Write("/Times/t_SII_Sep_Cmd", inputs.t_SII_Sep_Cmd);
	config.Write("/Times/t_SII_Sep_Delay", inputs.t_SII_Sep_Delay);
	config.Write("/Times/t_SIVB_Engine_Ignition", inputs.t_SIVB_Engine_Ignition);
	config.Write("/Times/t_SIVB_90P_Thrust", inputs.t_SIVB_90P_Thrust);
	config.Write("/Times/t_TLI_Fuel_Lead", inputs.t_TLI_Fuel_Lead);
	config.Write("/Times/t_TLI_APS_Ullage_Off", inputs.t_TLI_APS_Ullage_Off);
	config.Write("/Times/t_TLI_Reignition", inputs.t_TLI_Reignition);
	config.Write("/Times/t_TLI_IGM", inputs.t_TLI_IGM);
	config.Write("/Times/t_TLI_MRS_1st", inputs.t_TLI_MRS_1st);
	config.Write("/Times/t_TLI_MRS_2nd", inputs.t_TLI_MRS_2nd);

	config.Write("/Options/SI_RetroNum", inputs.SI_RetroNum);
	config.Write("/Options/SII_UllageNum", inputs.SII_UllageNum);
	config.Write("/Options/SII_RetroNum", inputs.SII_RetroNum);
	config.Write("/Options/bDoSII_CECO", inputs.bDoSII_CECO);
	config.Write("/Options/SIVB_UllageNum", inputs.SIVB_UllageNum);

	SetStatusText("Save successful!");

	// Include file name in title
	SetTitle(dlg.GetPath() + " - Saturn V Propulsion Simulation");
}

void SaturnPropulsionSimFrame::OnButtonCalculate(wxCommandEvent& event)
{
	SaturnVPropulsionSimulation sim;

	if (!TransferDataFromWindow())
	{
		SetStatusText("Invalid data!");
		return;
	}

	SetStatusText("Simulating launch...");
	sim.Calc(inputs);
	SetStatusText("Propulsion simulation successful!");
}