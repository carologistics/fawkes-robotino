/***************************************************************************
 *  data_table.h - This module stores all logical information
 *    about the llsf simulation (e.g. machine orientations,
 *    light signals, puck locations)
 *
 *  Created: Fri Aug 09 12:22:51 2013
 *  Copyright  2013  Frederik Zwilling
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

#ifndef DATATABLE_H__
#define DATATABLE_H__


#include "refbox_comm.h"
#include <string.h>
#include <llsf_msgs/PuckInfo.pb.h>


#define NUMBER_MACHINES 16
#define NUMBER_PUCKS 20

namespace gazebo
{

  class RefboxComm;
  
  typedef enum MachineName
  {
    M1,
    M2,
    M3,
    M4,
    M5,
    M6,
    M7,
    M8,
    M9,
    M10,
    D1,
    D2,
    D3,
    R1,
    R2,
    T,
    NONE
  } MachineName;

  typedef enum LightState
  {
    OFF,
    ON,
    BLINK
  } LightState;

  typedef struct Machine
  {
    MachineName name;
    std::string name_link;
    std::string name_string;
    double x;
    double y;
    double ori;
    LightState red;
    LightState yellow;
    LightState green;
   } Machine;

  typedef struct Puck
  {
    int number;
    std::string name_link;
    double x;
    double y;
    MachineName under_rfid;
    MachineName in_machine_area;
    llsf_msgs::PuckState state;
  } Puck;

  /**
   * This class stores all logical information
   *    about the llsf simulation (e.g. machine orientations,
   *    light signals, puck locations)
   */
  class LlsfDataTable
  {
  protected:
    //Constructor (Singleton!)
    LlsfDataTable(physics::WorldPtr world, transport::NodePtr gazebo_node);
  public:
    //Destructor
    ~LlsfDataTable();
  private:
    //reference to singleton
    static LlsfDataTable *table_;

  public:
    //get singleton instance (creates one if table_ is null)
    static LlsfDataTable* get_table();
    
    static void init(physics::WorldPtr world, transport::NodePtr gazebo_node);

    //clean everything up
    static void finalize();


    //Getter
    Machine get_machine(MachineName name);
    Machine get_machine(std::string name);
    Machine* get_machines();
    Puck get_puck(int number);

    // Setter
    void set_light_state(MachineName machine, LightState red,
			 LightState yellow, LightState green);
    void set_light_state(std::string machine, LightState red,
			 LightState yellow, LightState green);
    void set_puck_pos(int puck, double x, double y);
    void set_puck_under_rfid(int puck, MachineName machine);
    void remove_puck_under_rfid(int puck, MachineName machine);
    void set_puck_in_machine_area(int puck, MachineName machine);
    void set_puck_state(int puck, llsf_msgs::PuckState state);

  private:
    //Provides communication to the refbox
    RefboxComm *refbox_comm_;
    //llsf world to look up positions
    physics::WorldPtr world_;


    //data
    Machine machines_[17]; //[MX] gets machine X, except for [0]
    Puck pucks_[20];

    void init_table();
    void init_machine(MachineName number, std::string name_, std::string name_string);
    void init_puck(int number, std::string name);
  };
}
#endif
