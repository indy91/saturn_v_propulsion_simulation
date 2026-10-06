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

#include <wx/wx.h>
#include "saturn_v_propulsion_simulation.h"

// Declare the application class
class SaturnPropulsionSimApp : public wxApp
{
public:
	// Called on application startup
	bool OnInit() override;
};

// This defines the equivalent of main() for the current platform.
wxIMPLEMENT_APP(SaturnPropulsionSimApp);

// Declare our main frame class
class SaturnPropulsionSimFrame : public wxFrame
{
public:
	// Constructor
	SaturnPropulsionSimFrame(const wxString& title);

	// Event handlers
	void OnOpen(wxCommandEvent& event);
	void OnSave(wxCommandEvent& event);
	void OnButtonCalculate(wxCommandEvent& event);

protected:

	// Utilities
	template <typename T>
	void CreateTextCtrl(wxPanel* panel, wxFlexGridSizer* flexGridSizer, const wxString& label, const wxString& unit, const wxString& tooltip, T val) const;

	SaturnVPropulsionSimulationInputs inputs;

	// This class handles events
	DECLARE_EVENT_TABLE()
};