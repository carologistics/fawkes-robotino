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

/*
 * Base Colors:
 * Red
 * Black 
 * Silver
 * Ring Colors:
 * Blue
 * Green
 * Yellow
 * Orange
 * Cap Colors:
 * Gray
 * Black
*/

enum WorkingPieceComponent
	{BASE_RED, BASE_SILVER, BASE_BLACK, 
	 RING_BLUE, RING_GREEN, RING_YELLOW, RING_ORANGE,
	 CAP_BLACK, CAP_GRAY};


using namespace std;

class WorkingPiece {
private:
  std::vector<WorkingPieceComponent> workingPieceComponents;
public:

  //getEncodedWorkingPiece() TODO from Igor: What is the return value?
  //void setWorkingPiece() TODO from Igor: Which parameter to use?
};

#endif
