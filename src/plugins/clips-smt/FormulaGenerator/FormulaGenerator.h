#ifndef FORMULAGENERATOR_H
#define FORMULAGENERATOR_H
#include <vector>
#include <iostream>
#include <z3++.h>


#include "Action.h"

/*
 * provides methods to generate a formula for the wished amount of steps, 
 * a step denotes the possible interactions of a robot with a station at a certain world state.
 * A variable assignment is a solution of the formula for k steps if the variables for a certain step 
 * encodes the world state after exactly one valid action of an robot based on the world state in the step before.
 * The 0th step represents the current world state and its variables are initially assigned with values from gameData.
 */

class FormulaGenerator {
public:

    FormulaGenerator(int amount, GameData& gameData);
    virtual ~FormulaGenerator();

    /*creates and returns the formula, the formula is saved in the formula attribute*/
    Formula createFormula();

    std::vector<Action> getActions(const z3::model& model);
    Action getAction(const z3::model& model, int step);
    
    std::string printActions(const z3::model& model);
    
    std::string printWorldStates(const z3::model& model);
    std::string printWorldState(const z3::model& model, int);

private:
    
    stepFormula_ptr getStep(int i);
    std::vector<stepFormula_ptr> getSteps();
    Formula getFormula();

    /* generates the needed amount of StepFormula objects 
     * based on the current world state encoded in gameData*/
    void generateSteps(int amount, GameData& gameData);

    void setGameData(const GameData &gameData);
    GameData getGameData();

    std::map<string, int> transferZ3ModelToMap(const z3::model& model);

    set<station_ptr> getStationOccAltered(const std::map<string, int>& model, int step);
    set<robot_ptr> getRobotMovAltered(const std::map<string, int>& model, int step);
    set<robot_ptr> getRobotWorkpieceAltered(const std::map<string, int>& model, int step);
    set<capStation_ptr> getCapStationWorkpieceAltered(const std::map<string, int>& model, int step);
    set<ringStation_ptr> getRingStationWorkpieceAltered(const std::map<string, int>& model, int step);
    set<capStation_ptr> getCapStationCapColorAltered(const std::map<string, int>& model, int step);
    set<ringStation_ptr> getRingStationRingColorAltered(const std::map<string, int>& model, int step);
    set<ringStation_ptr> getRingStationAddBasesReqAltered(const std::map<string, int>& model, int step);
    
    bool checkConsistency(const std::map<string, int>& model, int step);

    Action checkNoneAction(const std::map<string, int>&, int);
    Action checkCollectBaseAction(const station_ptr&, const robot_ptr&, const std::map<string, int>&, int);
    Action checkSetUpRingColorAction(set<ringStation_ptr>, const std::map<string, int>&, int);
    Action checkFeedBaseAction(const station_ptr&, const robot_ptr&, set<ringStation_ptr>, const std::map<string, int>&, int);
    Action checkMountRingAction(const station_ptr&, set<ringStation_ptr>, const robot_ptr&, const std::map<string, int>&, int);
    Action checkfeedCapAction(const station_ptr&, const robot_ptr&, const set<capStation_ptr>&, const std::map<string, int>&, int);
    Action checkMountCapAction(const station_ptr&, const robot_ptr&, const set<capStation_ptr>&, const std::map<string, int>&, int);
    Action checkPickUpWorkpiece(const station_ptr&, const robot_ptr&, const std::map<string, int>&, int);
    Action checkDeliverProductAction(const station_ptr&, const robot_ptr&, const std::map<string, int>&, int);

    std::string printWorldStateWorkpiece(std::map<string, int> model, int step, machine_ptr m);
    
    std::vector<stepFormula_ptr> steps;
    Formula formula;
    GameData gameData;
};




#endif /* FORMULAGENERATOR_H */

