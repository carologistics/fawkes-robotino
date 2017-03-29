#ifndef STEPFORMULA_H
#define STEPFORMULA_H

#include <string>
#include <memory>
#include <iostream>
#include <vector>
#include <map>

#include "carl/core/VariablePool.h"
#include "carl/formula/Formula.h"

#include "GameData.h"

using Rational = mpq_class;

typedef carl::MultivariatePolynomial<Rational> Pol;
typedef carl::Constraint<Pol> Constr;
typedef carl::Formula<Pol> Formula;
typedef carl::Variable Variable;


class StepFormula;
typedef std::shared_ptr<StepFormula> stepFormula_ptr;

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
     * creates the needed amount of StepFormula Objects with the aid of the world state represented by gameData
     */
    static std::vector<stepFormula_ptr> generateSteps(uint amount, GameData& gameData);

    /*
     * creates a formula for the step this object represent
     */
    Formula create();
    
    /*
     * initial world state constraints creation
     */
    Formula createInitialState();
    Formula createInitialStateRobots();
    Formula createInitialStateStations();
    Formula createInitialStateOrders();
    Formula createInitialStateMovingTimes(const Robot& r);
    
    /*
     * logic-variable creation
     */
    std::map<std::string, Variable> getVariables() const;

    Variable getVarHoldsBase(const Machine &m);
    Variable getVarHoldsRing(const Machine &m, int i);
    Variable getVarHoldsCap(const Machine &m);

    Variable getVarMachineOccupied(const Station& m);//only stations
    Variable getVarMovingTime(const Robot& r, const Station& m);

    Variable getVarCapColor(const CapStation &cs);

    Variable getVarRingColor(const RingStation &rs);
    Variable getVarBaseReq(const RingStation &rs);
    
    Variable getVarReward();
    Variable getVarOrderDelivered(const Order& o);
    
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

    /*
     * robot-action-formulas creation
     */
    Formula createRemainingStepConstraints(const Robot &robot, const Station &station);
    Formula createRemainingStepConstraints(const RingStation &rs);
    Formula stationIsNotBlockedFormula(const Robot& r, const Station& station, int processingTime);
    
    
    Formula collectBaseActions();
    Formula collectBaseAction(Robot &r, Workpiece::Color c, BaseStation& bs);
    Formula existOrderWithBaseColorReq(Workpiece::Color c);
    Formula collectBaseActionReward(Robot &r, Workpiece::Color c, BaseStation& bs);
    Formula needsBaseForRingStation();


    Formula feedCapActions();
    Formula feedCapAction(Robot &r, Workpiece::Color c, CapStation &cs);
    Formula existOrderWithCapColorReq(Workpiece::Color c);
    Formula feedCapActionReward(Robot &r, Workpiece::Color c, CapStation &cs);
   

    Formula mountCapActions();
    Formula mountCapAction(Robot &r, CapStation &cs);
    Formula existOrderWithCapReq(Robot &r, CapStation &cs);
    Formula orderHasCapReq(Robot &r, CapStation &cs, Order &o);
    Formula mountCapActionReward(Robot &r, CapStation &cs);
    
    Formula mountCapWorkpiece(const CapStation& cs, const Robot& r);
    Formula mountCapComponent(const CapStation& cs);
    
    
    Formula setupRingColorActions();
    Formula setupRingColorAction(Workpiece::Color c, RingStation &rs);
    Formula existOrderWithRingColorReq(Workpiece::Color c);
    Formula setupRingColorActionReward(Workpiece::Color c, RingStation &rs);
    
    
    Formula feedAdditionalBaseActions();
    Formula feedAdditionalBaseAction(Robot &r, RingStation &rs);
    Formula feedAdditionalBaseActionReward(Robot &r, RingStation &rs);
    
    
    Formula mountRingActions();
    Formula mountRingAction(Robot &r, RingStation &rs);
    Formula existOrderWithRingReq(Robot &r, RingStation &cs);
    Formula orderHasRingReq(Robot &r, RingStation &rs, Order &o, int position);
    Formula mountRingActionReward(Robot &r, RingStation &rs);
    
    Formula mountRingWorkpiece(const RingStation& rs, const Robot& r);
    Formula mountRingRings(const RingStation& rs, const Robot& r, int i);
    Formula mountRingComponent(const RingStation& rs, int i);
    
    
    Formula collectWorkpieceActions();
    Formula collectWorkpieceAction(Robot &r, Station &s);
    Formula collectWorkpieceActionReward(Robot &r, Station &s);
    
    
    Formula deliverWorkpieceActions();
    Formula deliverWorkpieceAction(Robot &r, DeliveryStation &ds);
    Formula deliverWorkpieceActionReward(Robot &r, Station &d);
    
private:
    static int newId;
    int stepNumber;
    stepFormula_ptr previousStep = nullptr;

    GameData& gameData;
    std::map<std::string, Variable> variables = std::map<std::string, Variable>();
};

#endif /* STEPFORMULA_H */