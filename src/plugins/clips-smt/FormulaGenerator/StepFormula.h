/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   StepFormula.h
 * Author: leonard
 *
 * Created on January 31, 2017, 10:16 PM
 */

#ifndef STEPFORMULA_H
#define STEPFORMULA_H

#include "carl/core/VariablePool.h"
#include "carl/formula/Formula.h"
#include "carl/util/stringparser.h"

#include "tests/Common.h"

#include "Robot.h"
#include "Workpiece.h"
#include "Order.h"
#include "BaseStation.h"
#include "CapStation.h"
#include "Station.h"
#include "RingStation.h"
#include "DeliveryStation.h"

#include <tuple>
#include <map>

using Integer = mpz_class;

using namespace carl;

typedef MultivariatePolynomial<Rational> Pol;
typedef Constraint<Pol> Constr;
typedef Formula<Pol> FormulaT;

class StepFormula {
    
    //typedef boost::shared_ptr<StepFormula> stepFormula_ptr;
    typedef boost::shared_ptr<Robot> robot_ptr;
    typedef boost::shared_ptr<BaseStation> baseStation_ptr;
    typedef boost::shared_ptr<RingStation> ringStation_ptr;
    typedef boost::shared_ptr<CapStation> capStation_ptr;
    typedef boost::shared_ptr<DeliveryStation> deliveryStation_ptr;
    
public:
    StepFormula(int stepNumber);
    StepFormula(int stepNumber, StepFormula& pPrevStep);
    StepFormula(const StepFormula& orig);
    virtual ~StepFormula();
    
    int getStepNumber(){return stepNumber;}
    StepFormula* getPrevStep(){
        return prevStep;
    }
    FormulaT equation(int value1, int value2);
    FormulaT equation(Variable var, int value);
    FormulaT equation(Variable var1, Variable var2);
    FormulaT equation(Variable var1, Pol pol);
    
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
    
    
    FormulaT getCollectBaseFormula(Robot &r, Workpiece::Color c);
    FormulaT getCollectBaseStepNotFinishedFormula(Robot &r, Order &o);
    
    FormulaT getFeedCapFormula(Robot &r, Workpiece::Color c, CapStation &cs);
    FormulaT getFeedCapStepNotFinishedFormula(Robot &r, Order &o);
    
    FormulaT getMountCapFormula(Robot &r, CapStation &cs);
    FormulaT getMountCapStepNotFinishedFormula(Robot &r, Order &o, CapStation &cs);
    
    FormulaT getSetupRingColorFormula(Robot &r, Workpiece::Color c, RingStation &rs);
    FormulaT getSetupRingColorNotFinishedFormula(Robot &r, Workpiece::Color c, Order &o);
    
    FormulaT getFeedAdditionalBaseFormula(Robot &r, RingStation &rs);
    
    FormulaT getMountRingFormula(Robot &r, RingStation &rs);
    FormulaT getMountRingNotFinishedFormula(Robot &r, RingStation &rs, Order &o);
    
    FormulaT getCollectWorkpieceFormula(Robot &r, Station &s);
    
    FormulaT getDeliverWorkpieceFormula(Robot &r, Station &d);
    FormulaT getMarkOrderDeliveredFormula(Robot &r, Order &o);
    
    //int getMovingTimes(Machine &m1, Machine &m2){return movingTimes[(m1,m2)];}
    //void setMovingTimes(Machine &m1, Machine &m2, int time){return movingTimes[(m1,m2)];}
    
    vector<robot_ptr> getRobots();

private: 
    int stepNumber = -1;
    boost::shared_ptr<StepFormula> prevStep;
    vector<robot_ptr> robots;
    vector<baseStation_ptr> baseStations;
    vector<ringStation_ptr> ringStations;
    vector<capStation_ptr> capStations;
    vector<deliveryStation_ptr> deliveryStations;
    
    map <tuple<Machine,Machine>, int> movingTimes; 
};

#endif /* STEPFORMULA_H */

