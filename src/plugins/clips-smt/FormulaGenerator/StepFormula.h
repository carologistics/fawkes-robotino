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

    static std::vector<stepFormula_ptr> generateSteps(uint number, GameData& gameData);

    int getStepNumber();
    string getStepName();
    stepFormula_ptr getPrevStep();
    GameData getGameData() const;
    Variable getVariable(string varName);

    void setStepNumber(int stepNumber);
    void setPreviousStep(stepFormula_ptr &previousStep);


    Formula holdsBaseFormula(Machine &m, Workpiece::Color color);
    Formula holdsRingFormula(Machine &m, Workpiece::Color number, Workpiece::Color color);
    Formula holdsCapFormula(Machine &m, Workpiece::Color color);

    Formula createInitialStateRobots();
    Formula createInitialStateStations();
    Formula createInitialStateOrders();
    
    Formula holdsWorkpiece(Machine &m, Workpiece workpiece);
    Formula createInitialStateMovingTimes(Robot& r);

    Formula createInitialState();
    Formula create();
    
    Formula stationIsNotBlockedFormula(Robot& r, Station& station, int processingTime);

    Variable getVarHoldsBase(Machine &m);
    Variable getVarHoldsRing(Machine &m, int i);
    Variable getVarHoldsCap(Machine &m);

    //we use a simplified version this vars are not used yet
    /*Variable getVarBaseProgress(Order &o);
    Variable getVarRingProgress(Order &o, int i);
    Variable getVarCapProgress(Order &o);*/

    Variable getVarMachineOccupied(Machine &m);//only stations
    Variable getVarMovingTime(Robot r, Station &m);

    Variable getVarCapColor(CapStation &cs);

    Variable getVarRingColor(RingStation &rs);
    Variable getVarBaseReq(RingStation &rs);
    
    Variable getVarReward();

    Formula equation(int value1, int value2);
    Formula equation(Variable var, int value);
    Formula equation(Variable var1, Variable var2);
    Formula equation(Variable var1, Pol pol);
    
    Formula lessEqual(Variable var1, Variable var2);
    Formula greater(Variable var1, Variable var2);

    Formula stationIsNotBlockedFormula(Robot& r, Station& s);
    
    Formula getCollectBaseFormula();
    Formula getCollectBaseFormula(Robot &r, Workpiece::Color c, BaseStation& bs);
    Formula getCollectBaseRewardFormula(Robot &r, Workpiece::Color c, BaseStation& bs);

    //Formula getCollectBaseStepNotFinishedFormula(Robot &r, Order &o);

    Formula getFeedCapFormula();
    Formula getFeedCapFormula(Robot &r, Workpiece::Color c, CapStation &cs);
    Formula getFeedCapRewardFormula(Robot &r, Workpiece::Color c, CapStation &cs);
    //Formula getFeedCapStepNotFinishedFormula(Robot &r, Order &o);
    
    Formula getMountCapFormula();
    Formula getMountCapFormula(Robot &r, CapStation &cs);
    Formula getMountCapRewardFormula(Robot &r, CapStation &cs);
    //Formula getMountCapStepNotFinishedFormula(Robot &r, Order &o, CapStation &cs);
    
    Formula getSetupRingColorFormula();
    Formula getSetupRingColorFormula(Robot &r, Workpiece::Color c, RingStation &rs);
    Formula getSetupRingColorRewardFormula(Robot &r, Workpiece::Color c, RingStation &rs);
    //Formula getSetupRingColorNotFinishedFormula(Robot &r, Workpiece::Color c, Order &o);
    
    Formula getFeedAdditionalBaseFormula();
    Formula getFeedAdditionalBaseFormula(Robot &r, RingStation &rs);
    Formula getFeedAdditionalBaseRewardFormula(Robot &r, RingStation &rs);
    
    Formula getMountRingFormula();
    Formula getMountRingFormula(Robot &r, RingStation &rs);
    Formula getMountRingRewardFormula(Robot &r, RingStation &rs);
    //Formula getMountRingNotFinishedFormula(Robot &r, RingStation &rs, Order &o);
    
    Formula getCollectWorkpieceFormula();
    Formula getCollectWorkpieceFormula(Robot &r, Station &s);
    Formula getCollectWorkpieceRewardFormula(Robot &r, Station &s);
    
    Formula getDeliverWorkpieceFormula();
    Formula getDeliverWorkpieceFormula(Robot &r, Station &d);
    Formula getDeliverWorkpieceRewardFormula(Robot &r, Station &d);
    //Formula getMarkOrderDeliveredFormula(Robot &r, Order &o);

private:
    static int newId;
    int stepNumber;
    stepFormula_ptr previousStep = nullptr;

    GameData& gameData;
    std::map<std::string, Variable> varibles = std::map<std::string, Variable>();
};

#endif /* STEPFORMULA_H */