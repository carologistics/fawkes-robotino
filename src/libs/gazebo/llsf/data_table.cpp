/***************************************************************************
 *  data_table.cpp - This module stores all logical information
 *    about the llsf simulation (e.g. machine orientations,
 *    light signals, puck locations)
 *
 *  Created: Fri Aug 09 12:33:22 2013
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

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <transport/transport.hh>
#include <stddef.h>
#include <sstream>

#include "data_table.h"
#include "refbox_comm.h"


using namespace gazebo;

LlsfDataTable::LlsfDataTable(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  world_ = world;

  //read out machine positions and orientation from world
  init_table();

  //initialize refbox communication
  refbox_comm_ = new RefboxComm(this, gazebo_node);
}

LlsfDataTable::~LlsfDataTable()
{
}

LlsfDataTable* LlsfDataTable::table_ = NULL;

void LlsfDataTable::init(physics::WorldPtr world, transport::NodePtr gazebo_node)
{
  if(!table_)
  {
    table_ = new LlsfDataTable(world, gazebo_node);
  }
}

LlsfDataTable* LlsfDataTable::get_table()
{
  return table_;
}

void LlsfDataTable::finalize()
{
  if(table_)
  {
    delete table_;
  }
}

Machine LlsfDataTable::get_machine(MachineName name)
{
  return machines_[name];
}

Machine LlsfDataTable::get_machine(std::string machine)
{
  for(int i = M1; i <= T; i++)
  {
    //is it the right machine?
    if(machines_[i].name_link.find(machine) != std::string::npos)
    {
      return machines_[i];
    }
  }
  //default  
  return machines_[0];
}

Machine* LlsfDataTable::get_machines()
{
  return machines_;
}

Puck LlsfDataTable::get_puck(int number)
{
  return pucks_[number]; //assuming the pucks numbers start at 0
}

void LlsfDataTable::set_light_state(MachineName machine, LightState red, 
				    LightState yellow, LightState green)
{
  machines_[machine].red = red;
  machines_[machine].yellow = yellow;
  machines_[machine].green = green;
}

void LlsfDataTable::set_light_state(std::string machine, LightState red,
		     LightState yellow, LightState green)
{
  for(int i = M1; i <= T; i++)
  {
    //is it the right machine?
    if(machines_[i].name_link.find(machine) != std::string::npos)
    {
      set_light_state(machines_[i].name, red, yellow, green);
      return;
    }
  }
}

void LlsfDataTable::set_puck_pos(int puck, double x, double y)
{
  pucks_[puck].x = x;
  pucks_[puck].y = y;
}

void LlsfDataTable::set_puck_under_rfid(int puck, MachineName machine)
{
  pucks_[puck].under_rfid = machine;
  //inform refbox
  refbox_comm_->send_puck_placed_under_rfid(puck, machines_[machine]);
}

void LlsfDataTable::remove_puck_under_rfid(int puck, MachineName machine)
{
  pucks_[puck].under_rfid = NONE;
  //inform refbox
  refbox_comm_->send_remove_puck_from_machine(puck, machines_[machine]);
}

void LlsfDataTable::set_puck_in_machine_area(int puck, MachineName machine)
{
  pucks_[puck].in_machine_area = machine;
  //TODO: inform refbox if a puck leaves a machine area
}

void LlsfDataTable::set_puck_state(int puck, llsf_msgs::PuckState state)
{
  pucks_[puck].state = state;
}

void LlsfDataTable::init_table()
{
  init_machine(M1, "llsf_field::M1::machine_link", "M1");
  init_machine(M2, "llsf_field::M2::machine_link", "M2");
  init_machine(M3, "llsf_field::M3::machine_link", "M3");
  init_machine(M4, "llsf_field::M4::machine_link", "M4");
  init_machine(M5, "llsf_field::M5::machine_link", "M5");
  init_machine(M6, "llsf_field::M6::machine_link", "M6");
  init_machine(M7, "llsf_field::M7::machine_link", "M7");
  init_machine(M8, "llsf_field::M8::machine_link", "M8");
  init_machine(M9, "llsf_field::M9::machine_link", "M9");
  init_machine(M10, "llsf_field::M10::machine_link", "M10");
  init_machine(D1, "llsf_field::D1::machine_link", "D1");
  init_machine(D2, "llsf_field::D2::machine_link", "D2");
  init_machine(D3, "llsf_field::D3::machine_link", "D3");
  init_machine(R1, "llsf_field::R1::machine_link", "R1");
  init_machine(R2, "llsf_field::R2::machine_link", "R2");
  init_machine(T, "llsf_field::TST::machine_link", "TST");

  for(int i = 0; i < NUMBER_PUCKS; i++)
  {
    std::ostringstream ss;
    ss << "Puck" << i << "::cylinder";
    init_puck(i, ss.str().c_str());
  }
}

void LlsfDataTable::init_machine(MachineName number, std::string name_link, std::string name_string)
{
  machines_[number].name = number;
  machines_[number].name_link = name_link;
  machines_[number].name_string = name_string;
  machines_[number].x = world_->GetEntity(name_link)->GetWorldPose().pos.x;
  machines_[number].y = world_->GetEntity(name_link)->GetWorldPose().pos.y;
  machines_[number].ori = world_->GetEntity(name_link)->GetWorldPose().rot.GetAsEuler().z;
  machines_[number].red = BLINK;//OFF;
  machines_[number].yellow = BLINK;//OFF;
  machines_[number].green = BLINK;//OFF;
}

void LlsfDataTable::init_puck(int number, std::string name)
{
  pucks_[number].number = number;
  pucks_[number].name_link = name;
  pucks_[number].under_rfid = NONE;
  pucks_[number].in_machine_area = NONE;
  pucks_[number].state = llsf_msgs::S0;
}
