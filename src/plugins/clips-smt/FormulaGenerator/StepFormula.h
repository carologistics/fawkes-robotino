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
    stepFormula_ptr getPreviousStep();
    GameData getGameData() const;
    Variable getVariable(string varName);
    
    void setStepNumber(int stepNumber);
    void setPreviousStep(stepFormula_ptr &previousStep);
    
    Variable getVarHoldsBase(Machine &m);
    Variable getVarHoldsRing(Machine &m, int i);
    Variable getVarHoldsCap(Machine &m);

    Variable getVarBaseProgress(Order &o);
    Variable getVarRingProgress(Order &o, int i);
    Variable getVarCapProgress(Order &o);

    Variable getVarMachineOccupied(Machine &m);
    Variable getVarMovingTime(Robot r, Station &m);

    Variable getVarCapColor(CapStation &cs);

    Variable getVarRingColor(RingStation &rs);
    Variable getBaseReq(RingStation &rs);
    Variable getVarBaseCount(RingStation &rs);
    
    
    Formula equation(int value1, int value2);
    Formula equation(Variable var, int value);
    Formula equation(Variable var1, Variable var2);
    Formula equation(Variable var1, Pol pol);
    
    
    Formula getCollectBaseFormula(Robot &r, Workpiece::Color c);
    Formula getCollectBaseStepNotFinishedFormula(Robot &r, Order &o);

    Formula getFeedCapFormula(Robot &r, Workpiece::Color c, CapStation &cs);
    Formula getFeedCapStepNotFinishedFormula(Robot &r, Order &o);

    Formula getMountCapFormula(Robot &r, CapStation &cs);
    Formula getMountCapStepNotFinishedFormula(Robot &r, Order &o, CapStation &cs);

    Formula getSetupRingColorFormula(Robot &r, Workpiece::Color c, RingStation &rs);
    Formula getSetupRingColorNotFinishedFormula(Robot &r, Workpiece::Color c, Order &o);

    Formula getFeedAdditionalBaseFormula(Robot &r, RingStation &rs);

    Formula getMountRingFormula(Robot &r, RingStation &rs);
    Formula getMountRingNotFinishedFormula(Robot &r, RingStation &rs, Order &o);

    Formula getCollectWorkpieceFormula(Robot &r, Station &s);

    Formula getDeliverWorkpieceFormula(Robot &r, Station &d);
    Formula getMarkOrderDeliveredFormula(Robot &r, Order &o);

private:
    static int newId;
    int stepNumber;
    stepFormula_ptr previousStep = nullptr;
    
    GameData& gameData;
    std::map<std::string, Variable> varibles = std::map<std::string, Variable>();
};

#endif /* STEPFORMULA_H */