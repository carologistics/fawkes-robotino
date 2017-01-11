/***************************************************************************
 *  clips_smt_wrokingPiece.h - Smt feature for CLIPS
 *
 *  Created: Created on Mon Dec 19 14:09 2016 by Igor Nicolai Bongartz
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef _PLUGINS_CLIPS_SMT_WORKINGPIECE_H_
#define _PLUGINS_CLIPS_SMT_WORKINGPIECE_H_

#include <cassert>

/*
 * Base Colors:
 * Red 0
 * Black 1
 * Silver 2
 * Ring Colors:
 * Blue 3
 * Green 4
 * Yellow 5
 * Orange 6
 * Cap Colors:
 * Gray 7
 * Black 8
*/

enum WorkingPieceComponent
	{BASE_RED, BASE_SILVER, BASE_BLACK,
	 RING_BLUE, RING_GREEN, RING_YELLOW, RING_ORANGE,
	 CAP_BLACK, CAP_GRAY};


class WorkingPiece {

private:
  std::vector<WorkingPieceComponent> _workingPieceComponents;

public:
	WorkingPiece(){}

	WorkingPiece(std::string input)
	{
		_workingPieceComponents = convertToWorkPiece(input);
	}

	std::vector<WorkingPieceComponent> convertToWorkPiece(std::string input)
	{
		std::vector<WorkingPieceComponent> workingPieceComponents;
		std::vector<char> inputAsChars (input.begin(), input.end());
		for (unsigned int i = 0; i < inputAsChars.size(); ++i)
		{
			WorkingPieceComponent currentWorkingPieceComponent;
			assert((inputAsChars[i]>= 48) && (inputAsChars[i]<57));
			currentWorkingPieceComponent = static_cast<WorkingPieceComponent>((int)inputAsChars[i]-48);
			workingPieceComponents.push_back(currentWorkingPieceComponent);
		}

		//making shure all charackters were converted:
		assert (workingPieceComponents.size() == inputAsChars.size());
		return workingPieceComponents;
	}

  int getBaseComponent()
	{
		for(WorkingPieceComponent workingPieceComponent: _workingPieceComponents) {
			if(workingPieceComponent>=0 && workingPieceComponent<3) {
				return workingPieceComponent;
			}
		}

		return -1;
	}

	// std::vector<int> getRingComponent()

	int getCapComponent()
	{
		for(WorkingPieceComponent workingPieceComponent: _workingPieceComponents) {
			if(workingPieceComponent>=7 && workingPieceComponent<9) {
				return workingPieceComponent;
			}
		}

		return -1;
	}

	std::string toString()
	{
		std::string workingPieceDescription;
		workingPieceDescription += "[";
		for(WorkingPieceComponent workingPieceComponent: _workingPieceComponents) {
			switch(workingPieceComponent) {
				case 0: workingPieceDescription += " B_RED";
								break;
				case 1: workingPieceDescription += " B_SILVER";
							  break;
				case 2: workingPieceDescription += " B_BLACK";
								break;
				case 3: workingPieceDescription += " R_BLUE";
							  break;
				case 4: workingPieceDescription += " R_GREEN";
								break;
				case 5: workingPieceDescription += " R_YELLOW";
								break;
				case 6: workingPieceDescription += " R_ORANGE";
								break;
				case 7: workingPieceDescription += " C_BLACK";
				        break;
				case 8: workingPieceDescription += " C_GRAY";
								break;
				default: break;
			}
		}
		workingPieceDescription += " ]";
		return workingPieceDescription;
	}
};

#endif
