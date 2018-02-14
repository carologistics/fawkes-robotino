#ifndef STEPFORMULA_H
#define STEPFORMULA_H

#include <memory>
#include <iostream>
#include <vector>
#include <map>

#include "carl/core/VariablePool.h"
#include "carl/formula/Formula.h"

#include "GameData.h"

//drop base from cap station or use it implementieren

using Rational = mpq_class;

using string = std::string;

typedef carl::MultivariatePolynomial<Rational> Pol;
typedef carl::Constraint<Pol> Constr;
typedef carl::Formula<Pol> Formula;
typedef carl::Variable Variable;

class StepFormula;
typedef std::shared_ptr<StepFormula> stepFormula_ptr;

/*
 * represents a Step, which contains the set of the possible actions based on a world state.
 * Constraints to check if the world state in the step before allows a certain action
 * and constraints to assign the values for the world state after a certain action will be created.
 */

class StepFormula {
public:
    StepFormula(stepFormula_ptr &previousStep, GameData& gameData);
    StepFormula(GameData& gameData);
    virtual ~StepFormula();

    int getStepNumber();
    string getStepName();
    stepFormula_ptr getPrevStep();
    GameData getGameData() const;
    Variable getVariable(string varName);

    void setStepNumber(int stepNumber);
    void setPreviousStep(stepFormula_ptr &previousStep);

    /*
     * creates the needed amount of StepFormula objects with the aid of the world state represented by gameData
     */
    static std::vector<stepFormula_ptr> generateSteps(uint amount, GameData& gameData);

    /*
     * creates a formula for the step this object represents
     */
    Formula create();

    /*
     * initial world state constraints creation, is used for the 0th step
     */
    Formula createInitialState();
    Formula createInitialStateOrders();
    Formula createInitialStateReward();
    Formula createInitialStateRobots();
    Formula createInitialStateStations();
    Formula createInitialStateMovingTimes(const Robot& r);
    
    /*
     * variable names creation
     * is used to translate a model back to a sequence of world states
     */
    static string getVarNameHoldsBase(const Machine & m, int step);
    static string getVarNameHoldsRing(const Machine &m, int i, int step);
    static string getVarNameHoldsCap(const Machine & m, int step);
    static string getVarNameMachineOccupied(const Station& m, int step);
    static string getVarNameMovingTime(const Robot &r, const Station & m, int step);
    static string getVarNameCapColor(const CapStation & cs, int step);
    static string getVarNameRingColor(const RingStation & rs, int step);
    static string getVarNameBaseReq(const RingStation & rs, int step);
    static string getVarNameReward(int step);
    static string getVarNameOrderDelivered(const Order& o, int step);
    
    static string getVarNameOrderMain(const Order& o, int step);
    static string getVarNameOrderRing(int position, int base, const Order& o, int step);
    static string getVarNameOrderCap(const Order& o, int step);

    /*
     * logic-variable creation
     */
    std::map<std::string, Variable> getVariables() const;

    Variable getVarHoldsBase(const Machine &m);
    Variable getVarHoldsRing(const Machine &m, int i);
    Variable getVarHoldsCap(const Machine &m);

    Variable getVarMachineOccupied(const Station& m); //only stations
    Variable getVarMovingTime(const Robot& r, const Station& m);

    Variable getVarCapColor(const CapStation &cs);

    Variable getVarRingColor(const RingStation &rs);
    Variable getVarBaseReq(const RingStation &rs);

    Variable getVarReward();
    Variable getVarOrderDelivered(const Order& o);
    
    Variable getVarOrderMain(const Order& o);
    Variable getVarOrderRing(int position, int base, const Order& o);
    Variable getVarOrderCap(const Order& o);

    /*
     * basic constraint creation
     */
    Formula equation(int value1, int value2);
    Formula equation(Variable const& var, int value);
    Formula equation(Variable const& var1, Variable const& var2);
    Formula equation(Variable const& var1, Pol const& pol);

    Formula lessEqual(Variable const& var1, Variable const& var2);
    Formula greater(Variable const& var1, Variable const& var2);
    Formula greater(Variable const& var1, Rational const r);

    /*
     * basic check-world-state formula creation 
     */
    Formula holdsBasePrev(const Machine& m, Workpiece::Color c);
    Formula holdsRingPrev(const Machine& m, Workpiece::Color c, int i);
    Formula holdsCapPrev(const Machine& m, Workpiece::Color c);
    Formula holdsWorkpiecePrev(const Machine& m, const Workpiece& workpiece);

    Formula ringColorSetupPrev(const RingStation& rs, Workpiece::Color c);
    Formula needsAddBasesPrev(const RingStation& rs);

    Formula loadCapPrev(const CapStation& cs, Workpiece::Color c);

    Formula orderNotDeliveredPrev(const Order& o);
    Formula orderDeliveredPrev(const Order& o);

    /*
     * basic changes-in-the-world-state formula creation 
     */
    Formula holdsBase(const Machine& m, Workpiece::Color c);
    Formula holdsRing(const Machine& m, Workpiece::Color c, int i);
    Formula holdsCap(const Machine& m, Workpiece::Color c);
    Formula holdsWorkpiece(const Machine& m, const Workpiece& workpiece);

    Formula swapBase(const Machine& m1, const Machine& m2);
    Formula swapRing(const Machine& m1, const Machine& m2, int i);
    Formula swapCap(const Machine& m1, const Machine& m2);
    Formula swapWorkpiece(const Machine &m1, const Machine &m2);

    Formula ringColorSetup(const RingStation& rs, Workpiece::Color c);
    Formula needsAddBases(const RingStation& rs, Workpiece::Color c);
    Formula feedAddBase(const RingStation& rs);

    Formula loadCap(const CapStation& cs, Workpiece::Color c);

    Formula updateReward(int value);

    Formula orderDelivered(const Order& o);
    
    Formula orderDeliveredRemains(const Order& o);

    /*
     * robot-action-formulas creation,
     * 
     */

    /*Formula to transfer the unchanging world state values to the variables of the next world state in each action formula*/
    Formula createRemainingStepConstraints(const Robot &robot, const Station &station);
    Formula createRemainingStepConstraints(const Station &station);
    Formula createRemainingStepConstraintsStations(const Station &station);
    Formula createRemainingStepConstraintsRobots(const Robot &robot);
    Formula createRemainingStepConstraintsRobots();
    Formula createRemainingStepConstraintsOrders();
    Formula createRemainingStepConstraintsOrders(const Order& without);
    
    Formula orderNeedsAction(const Order& o, Order::State state);
    Formula ordersNeedsAction(const std::vector<Order> orders, Order::State state);
    
    /*Formula to update the moving times of the robot and the the time a machine is occupied after a action*/
    Formula updateTimes(const Robot& r, const Station& station, int processingTimeStation, int processingTimeRobot);

    /* creates a Formula which encodes the actions of getting a base from a base station for all robots and all base stations, 
     * checks if this action is valid and needed to fulfill an order or if a ring station needs additional bases*/
    Formula collectBaseActions();
    Formula collectBaseAction(Robot &r, Workpiece::Color c, BaseStation& bs);
    Formula existOrderWithBaseColorReq(Robot &r, Workpiece::Color c);
    Formula collectBaseActionReward(Robot &r, Workpiece::Color c, BaseStation& bs);
    Formula existOrderWithAddBaseReqBaseStation();
    Formula needsBaseForRingStation();

    /* creates a Formula which encodes the actions of feeding a cap to a cap station for all robots and all cap stations, 
     * checks if this action is valid and needed to fulfill an order*/
    Formula feedCapActions();
    Formula feedCapAction(Robot &r, Workpiece::Color c, CapStation &cs);
    Formula existOrderWithCapColorReq(Workpiece::Color c);
    Formula feedCapActionReward(Robot &r, Workpiece::Color c, CapStation &cs);

    /* creates a Formula which encodes the actions of mounting a cap to a workpiece a robot holds for all robots and all cap stations, 
     * checks if this action is is valid and needed to fulfill an order*/
    Formula mountCapActions();
    Formula mountCapAction(Robot &r, CapStation &cs);
    Formula existOrderWithCapReq(Robot &r, CapStation &cs);
    Formula orderHasCapReq(Robot &r, CapStation &cs, Order &o);
    Formula mountCapActionReward(Robot &r, CapStation &cs);
    /* encodes the mounting of the cap in the workpiece encoding*/
    Formula mountCapWorkpiece(const CapStation& cs, const Robot& r);
    /* encodes the mounting of the cap in the component encoding*/
    Formula mountCapComponent(const CapStation& cs);

    /* creates a Formula which encodes the actions of setting up a ring color for all ring stations, 
     * checks if this action is is valid and needed to fulfill an order*/
    Formula setupRingColorActions();
    Formula timesDoNotChange();
    Formula setupRingColorAction(Workpiece::Color c, RingStation &rs);
    Formula existOrderWithRingColorReq(Workpiece::Color c);
    Formula setupRingColorActionReward(Workpiece::Color c, RingStation &rs);

    /* creates a Formula which encodes the actions of feeding a additional base to a ring station, 
     * checks if this action is valid and needed to fulfill an order*/
    Formula feedAdditionalBaseActions();
    Formula feedAdditionalBaseAction(Robot &r, RingStation &rs);
    Formula existOrderWithAddBaseReqRingStation();
    Formula feedAdditionalBaseActionReward(Robot &r, RingStation &rs);

    /* creates a Formula which encodes the actions of mounting a ring to a workpiece a robot holds 
     * checks if this action is valid and needed to fulfill an order*/
    Formula mountRingActions();
    Formula mountRingAction(Robot &r, RingStation &rs);
    Formula existOrderWithRingReq(Robot &r, RingStation &cs);
    Formula orderHasRingReq(Robot &r, RingStation &rs, Order &o, int position);
    Formula mountRingActionReward(Robot &r, RingStation &rs);
    /* encodes the mounting of the ring in the workpiece encoding*/
    Formula mountRingWorkpiece(const RingStation& rs, const Robot& r);
    /* encodes the mounting of the cap in the rings encoding*/
    Formula mountRingRings(const RingStation& rs, const Robot& r, int i);
    /* encodes the mounting of the cap in the ring encoding*/
    Formula mountRingComponent(const RingStation& rs, int i);

    Formula dropTransparentBaseActions();
    Formula dropTransparentBaseAction(Robot &r, CapStation & cs);
    

    Formula collectWorkpieceActions();
    Formula collectWorkpieceAction(Robot &r, CapStation & cs);
    Formula collectWorkpieceAction(Robot &r, RingStation & rs);
    
    Formula collectTransparentBaseOrder(Robot &r, CapStation & cs);
    
    Formula collectWorkpieceGeneralAction(Robot &r, Station &s);
    Formula collectWorkpieceActionReward(Robot &r, Station &s);


    Formula deliverWorkpieceActions();   
    Formula deliverWorkpieceAction(Robot &r, DeliveryStation &ds);
    Formula deliverWorkpieceActionReward(Robot &r, Station &d);
    Formula orderDelivered(Robot& r);
    Formula inheritRemainingOrders(Order& o);
    
       
    Formula allOrdersDelivered();

private:
    static int newId;
    int stepNumber;
    stepFormula_ptr previousStep = nullptr;

    GameData& gameData;
    std::map<std::string, Variable> variables = std::map<std::string, Variable>();
};

#endif /* STEPFORMULA_H */