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
 * Red 		0
 * Black 	1
 * Silver 	2
 * Ring Colors:
 * Blue 	3
 * Green 	4
 * Yellow 	5
 * Orange 	6
 * Cap Colors:
 * Gray 	7
 * Black 	8
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

	WorkingPiece(int input)
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
			if ((inputAsChars[i]< 48) || (inputAsChars[i]>=57))
			{
				throw std::runtime_error("SMT_ERROR: Failed to create a WorkingPieceComponent with undefined Char.");
			}
			currentWorkingPieceComponent = static_cast<WorkingPieceComponent>((int)inputAsChars[i]-48);
			workingPieceComponents.push_back(currentWorkingPieceComponent);
		}

		//making shure all charackters were converted:
		if (workingPieceComponents.size() == inputAsChars.size())
			{
				throw std::runtime_error("SMT_ERROR: Not all input charackters could be converted.");
			}
		return workingPieceComponents;
	}


	std::vector<WorkingPieceComponent> convertToWorkPiece(int input)
	{
		if (input < 0)
			{
				throw std::runtime_error("SMT_ERROR: Cannot create WorkingPiece from Negative Int");
			}
		convertToWorkPiece(std::to_string(input));	
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

	bool isConsistent()
	{
		if (_workingPieceComponents.size() == 0)
		{
			return true;
		}

		for (std::vector<WorkingPieceComponent>::iterator it = _workingPieceComponents.begin() ; it != _workingPieceComponents.end(); ++it)
		{
			//Base only at first position
			if (it !=_workingPieceComponents.begin())
			{
				if(*it >= 0 && *it < 3)
				{
					return false;
				}
			}

			//Cap only at last position
			if (it != _workingPieceComponents.end()-1)
			{
				if(*it >= 7 && *it < 9)
				{
					return false;
				}				
			}
		}

		//No RingComponent/Cap as Base
		if (*_workingPieceComponents.begin()  >  3 && *_workingPieceComponents.begin() < 9)
		{
			return false;
		}
  
    	return true;
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

	int toInt()
	{
		std::string temporaryString;
		for(WorkingPieceComponent workingPieceComponent: _workingPieceComponents) {
			switch(workingPieceComponent) {
				case 0: temporaryString += "0";
								break;
				case 1: temporaryString += "1";
							  break;
				case 2: temporaryString += "2";
								break;
				case 3: temporaryString += "3";
							  break;
				case 4: temporaryString += "4";
								break;
				case 5: temporaryString += "5";
								break;
				case 6: temporaryString += "6";
								break;
				case 7: temporaryString += "7";
				        break;
				case 8: temporaryString += "8";
								break;
				default: break;
			}
		}

		if (_workingPieceComponents.size() != temporaryString.size())
		{
			throw std::runtime_error("SMT_ERROR: Unable to convert WorkingPiece to Int");
		}

		return std::stoi( temporaryString );
	}
};

#endif
