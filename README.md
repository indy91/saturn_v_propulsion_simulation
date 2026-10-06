# saturn_v_propulsion_simulation

This project generates tables of mass and thrust values for each stage of a Saturn V as well as a list of events in the format that is specified by the Launch Vehicle Simulation Program (LVSP), which was part of a suite of software that was used to target and generate presettings for the Translunar Injection maneuver (TLI). The real LVSP loaded this data as a propulsion tape and data cards for events and masses.

Notes on inputs:

The mass at TLI does not have to be very accurate. Mass changes in Earth parking orbit are simulated by the LVSP and the masses during the TLI burn are biased using the actual mass at the time when the table data is starting to be used.

## Propulsion Data Tape

The thrust of F-1 and J-2 engines, ullage engines, and retro engines is provided by the Propulsion Data Tape. The five files on the tape are composed of the following data:

File 1 - S-IC Stage Thrust

Each record contains six data words. Word 1 is the time (in seconds) from 90-percent thrust. The time during thrust buildup is therefore negative. Words 2-5 are the vacuum thrust in pounds of engines 1 - 4, respectively. Word 6 is the vacuum thrust (in pounds) of engine 5, ullage engines, and retro engines. Thrust decay is also included on the tape.

File 2 - S-II Stage Thrust

Each record contains six date words. The words are defined the same as those of File 1. Thrust buildup and decay are included.

File 3 - S-IVB First Burn Thrust

Each record contains two words. Word 1 is the same as Word 1 on the two previous files. Word 2 is the vacuum thrust (in pounds) of the engine. Thrust buildup is included. Thrust decay is not included. The file includes a time history that extends past the nominal first-guidance cutoff signal time (GCS(l)). When GCS(l) is reached, the program uses a special data table to compute thrust decay.

File 4 - S-IVB Second Burn Thrust (First Opportunity)

Each record contains two words. Word 1 is the same as Word 1 on the two previous files. Word 2 is the vacuum thrust (in pounds). The "lead thrust" that results from the J-2 chilldown process and the thrust buildup are included. Thrust decay is not included. The file includes a time history that extends past the nominal second guidance cutoff signal time (GCS(2)). When GCS(2) is reached, the program uses a special data table to compute thrust decay.

File 5 - S-IVB Second Burn Thrust (Second Opportunity)

The words are defined the same as those of File 4. The time of the mixture ratio shift (MRS) is usually different between the two TLI opportunities.

## Mass Data Format

The mass data format is one line in the text file per card. The first card of a data block is a Table ID Card:

Format: (5I5, 2A6, 2X, I1)

Columns 1-5 contain a right adjusted integer that is the MAJOR DIVISION NUMBER.
Columns 6-10 contain a right adjusted integer that is the SUBDIVISION NUMBER.
Columns 11-15 contain a right adjusted "2" for a single table and a right adjusted "3" for a double table.
Columns 16-20 contain a right adjusted Integer specifying the number of pairs of points of a single table or the number of values of the first independent variable for a double table.
Columns 21-25 contain a right adjusted integer specifying the number of values of the second independent variable for a double table. This field is blank for single tables.
Columns 26-37 may contain an alphanumeric table description or identification.

Single Table Data Cards

Format: (6F10.0)

Colums 1-60 contain numbers in fields of six columns.
Columns 1-10 contain the independent variable.
Columns 11-20 contain the corresponding value of the dependent variable.

The other four fields of a card similarly contain the independent variables followed by the corresponding values of dependent variables. The number of data cards required is dependent only upon the number of pairs of points in the table. If a table contains ten pairs of points, four Single Table Data Cards are required. The first three cards contain three pairs of points. The
fourth card contains one pair of points in columns 1-20.

| MAJOR DIVISION NUMBER | SUBDIVISION NUMBER | UNITS | DEPENDENT VARIABLE | INDEPENDENT VARIABLE |
| --- | --- | --- | --- | --- |
| 37 | 1 | lb | Instantaneous sea-level weight of the vehicle in S-IC flight stage | Time from first motion |
| 37 | 2 | lb | Instantaneous sea-level weight of the vehicle in S-II flight stage | Time from 90-percent thrust |
| 37 | 3 | lb | Instantaneous sea-level weight of the vehicle in S-IVB first burn | Time from 90-percent thrust |
| 37 | 4 | lb | Instantaneous sea-level weight of the vehicle in S-IVB second burn (first opportunity) | Time from lead thrust initation |
| 37 | 5 | lb | Instantaneous sea-level weight of the vehicle in S-IVB second burn (second opportunity) | Time from lead thrust initation |

## Event Times

In addition to the propulsion and mass data, an .ini file is generated that can be loaded into the LVSP as the initial settings that correspond to timing of the propulsion and mass data (e.g. engine cutoff). These variables are saved in the file:

| SYMBOL | UNITS | DEFINITION |
| --- | --- | --- |
| DELTSK(2) | sec | Time from S-IC/S-III separation to S-II interstage jettison |
| DTCOST(I) | sec | Time from physical stage separation of stage I to 90-percent thrust |
| DTLES | sec | Time from S-IC/S-II separation to LES jettison |
| EVAR2 | sec | Time from first motion to liftoff |
| EVAR6 | sec | Time from outboard engine cutoff to S-IC/S-II separation |
| EVAR7 | sec | Time from S-II ullage ignition to S-IC/S-II separation |
| EVAR9 | sec | Time from S-IC retroignition to S-IC/S-II separation |
| EVAR10 | sec | Time from S-IC/S-II separation to S-II ignition |
| EVAR11 | sec | Time from S-IC/S-II separation to S-II ullage cutoff |
| EVAR17 | sec | Time from S-II cutoff to S-II/S-IVB separation |
| EVAR18 | sec | Time from S-II ullage ignition to S-II/S-IVB separation |
| EVAR20 | sec | Time from S-II retroignition to S-II/S-IVB |
| EVAR22 | sec | Time from S-II/S-IVB separation to S-IVB ignition |
| EVAR23 | sec | Time from S-II/S-IVB separation to S-IVB ullage cutoff |
| EVAR31 | sec | Time from restart preparation to lead thrust initiation |
| EVAR33 | sec | Time from restart preparation to reignition |
| EVAR34 | sec | Time from restart preparation to ullage cutoff |
| EVAR35 | sec | Time from lead thrust initiation to S-IVB second 90-percent thrust |
| TBURN(1) | sec | Time from first motion to S-IC/S-II separation |
| TBURN(2) | sec | Time from first motion to S-II/S-IVB separation |
| VARIIN(4) | sec | Time from firstion motion to S-IC center engine cutoff |

## Dependencies

* wxWidgets