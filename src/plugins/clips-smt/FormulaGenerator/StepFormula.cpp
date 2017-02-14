/*
 * Possible solving performance improvements:
 * - switch ORs to XORs to provide the information that two parts of the conjunctions can't be satisfied at the same time to the sat-solver 
 * (reduces the amount of possible SAT solutions)
 * - replace the Variables(Base, Ring1, Ring2, Ring3, Cap) which denotes a Workpice with a single Variable and give every possible composition an id, 
 * decreases the amount of needed Variables per step from 
 * (3 Robots + 4 Stations + up to 15 Orders) * 5 = 110
 * to 3 Robots + 4 Stations + up to 15 Orders = 22
 * but increases the amount of needed constraints x54 in the worst case
 * 
 */

/* 
 * File:   StepFormula.cpp
 * Author: leonard
 * 
 * Created on January 15, 2017, 10:16 PM
 */

#include "StepFormula.h"
#include <string>

StepFormula::StepFormula(int pStepNumber) {
    stepNumber = pStepNumber;
}

StepFormula::StepFormula(int pStepNumber, StepFormula& pPrevStep) {
    stepNumber = pStepNumber;
    prevStep = &pPrevStep;
}

StepFormula::StepFormula(const StepFormula& orig) {
}

StepFormula::~StepFormula() {
}

FormulaT StepFormula::equation(int value1, int value2) {
    return FormulaT(Pol(Rational(value1) - Rational(value2)), Relation::EQ);
}

FormulaT StepFormula::equation(Variable var, int value) {
    return FormulaT(Pol(var - Rational(value)), Relation::EQ);
}

FormulaT StepFormula::equation(Variable var1, Variable var2) {
    return FormulaT(Pol(var1)-Pol(var2), Relation::EQ);
}

FormulaT StepFormula::equation(Variable var1, Pol pol) {
    return FormulaT(Pol(var1)-pol, Relation::EQ);
}

Variable StepFormula::getVarHoldsBase(Machine &m) {
    string mid = boost::lexical_cast<string>(m.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable(m.getType() + "B" + mid + "." + snum);
}

Variable StepFormula::getVarHoldsRing(Machine &m, int i) {
    string mid = boost::lexical_cast<string>(m.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    string rnum = boost::lexical_cast<string>(i);
    return freshIntegerVariable(m.getType() + "R" + rnum + mid + "." + snum);
}

Variable StepFormula::getVarHoldsCap(Machine &m) {
    string mid = boost::lexical_cast<string>(m.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable(m.getType() + "C" + mid + "." + snum);
}

Variable StepFormula::getVarBaseProgress(Order &o) {
    string oid = boost::lexical_cast<string>(o.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("oBp" + oid + "." + snum);
};

Variable StepFormula::getVarRingProgress(Order &o, int i) {
    string oid = boost::lexical_cast<string>(o.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    string rnum = boost::lexical_cast<string>(i);
    return freshIntegerVariable("oR" + rnum + "p" + oid + "." + snum);
}

Variable StepFormula::getVarCapProgress(Order &o) {
    string oid = boost::lexical_cast<string>(o.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("oCp" + oid + "." + snum);
}

Variable StepFormula::getVarMachineOccupied(Machine &m) {
    string mid = boost::lexical_cast<string>(m.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("Occ" + mid + "." + snum);
}

Variable StepFormula::getVarMovingTime(Robot r, Station &m) {
    string rid = boost::lexical_cast<string>(r.getId());
    string mid = boost::lexical_cast<string>(m.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("MovR" + rid + "M" + mid + "." + snum);
}

Variable StepFormula::getVarCapColor(CapStation &cs) {
    string csid = boost::lexical_cast<string>(cs.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("cC" + csid + "." + snum);
}

Variable StepFormula::getVarRingColor(RingStation &rs) {
    string rsid = boost::lexical_cast<string>(rs.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("rC" + rsid + "." + snum);
}

Variable StepFormula::getBaseReq(RingStation &rs) {
    string rsid = boost::lexical_cast<string>(rs.getId());
    string snum = boost::lexical_cast<string>(getStepNumber());
    return freshIntegerVariable("AddB" + rsid + "." + snum);
}


FormulaT StepFormula::getCollectBaseFormula(Robot &r, Workpiece::Color c) {
    FormulaT baseNotMounted = equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE);
    FormulaT baseMounted = equation(getVarHoldsBase(r), c);
    return FormulaT(FormulaType::AND,{baseNotMounted, baseMounted});
    //@todo getCollectBaseStepNotFinishedFormula über alle Order o mit o.getBaseColorReq() == c
}

FormulaT StepFormula::getCollectBaseStepNotFinishedFormula(Robot &r, Order &o) {
    FormulaT workstepIsNotFinished = equation(getPrevStep()->getVarBaseProgress(o), Order::NOTFINISHED);
    FormulaT workstepIsFinished = equation(getVarBaseProgress(o), Order::FINISHED);
    return FormulaT(FormulaType::AND,{workstepIsNotFinished, workstepIsFinished});
}

FormulaT StepFormula::getFeedCapFormula(Robot &r, Workpiece::Color c, CapStation &cs) {
    FormulaT brHoldsNothing = equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE);
    FormulaT arHoldsNothing = equation(getVarHoldsBase(r), Workpiece::NONE);

    FormulaT capNotLoaded = equation(getPrevStep()->getVarCapColor(cs), Workpiece::NONE);
    FormulaT capLoaded = equation(getVarCapColor(cs), c);

    FormulaT outputNotLoaded = equation(getPrevStep()->getVarHoldsBase(cs), Workpiece::NONE);
    FormulaT outputLoaded = equation(getVarHoldsBase(cs), Workpiece::TRANSPARENT);

    return FormulaT(FormulaType::AND,
       {brHoldsNothing, 
        arHoldsNothing,
        capNotLoaded, 
        capLoaded,
        outputNotLoaded, 
        outputLoaded});
    //@todo getCollectBaseStepNotFinishedFormula über alle Order o mit o.getRingColorReq() == c
}

FormulaT StepFormula::getFeedCapStepNotFinishedFormula(Robot &r, Order &o) {
    FormulaT workstepIsNotFinished = equation(getPrevStep()->getVarCapProgress(o), Order::NOTFINISHED);
    FormulaT workstepIsSetUp = equation(getVarCapProgress(o), Order::SETUP);
    return FormulaT(FormulaType::AND, {workstepIsNotFinished, workstepIsSetUp});
}

FormulaT StepFormula::getMountCapFormula(Robot &r, CapStation &cs) {
    FormulaT holdsNoWorkpiece = FormulaT( FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    FormulaT isNotTransparent = FormulaT( FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::TRANSPARENT));

    FormulaT noCap = FormulaT(equation(getPrevStep()->getVarHoldsCap(r), Workpiece::NONE));
    
    FormulaT capLoaded = FormulaT( FormulaType::NOT, equation(getPrevStep()->getVarCapColor(cs), Workpiece::NONE));
    FormulaT capNotLoaded = equation(getVarCapColor(cs), Workpiece::NONE);
    
    FormulaT noWorkpiece = equation(getPrevStep()->getVarHoldsBase(cs), Workpiece::NONE);
    FormulaT WorkpieceInOutput = equation(getPrevStep()->getVarHoldsBase(cs), Workpiece::NONE);
    
    FormulaT rTocsB = equation(getPrevStep()->getVarHoldsBase(r), getVarHoldsBase(cs));
    FormulaT rTocsR1 = equation(getPrevStep()->getVarHoldsRing(r,0), getVarHoldsRing(cs,0));
    FormulaT rTocsR2 = equation(getPrevStep()->getVarHoldsRing(r,1), getVarHoldsRing(cs,1));
    FormulaT rTocsR3 = equation(getPrevStep()->getVarHoldsRing(r,2), getVarHoldsRing(cs,2));
    FormulaT capMounted = equation(getVarHoldsCap(cs), getPrevStep()->getVarCapColor(cs));
    
    FormulaT rTocs(FormulaType::AND, {rTocsB, rTocsR1, rTocsR2, rTocsR3, capMounted});
    
    FormulaT rNoBase = equation(getVarHoldsBase(r), Workpiece::NONE);
    FormulaT rNoR1 = equation(getVarHoldsRing(r,0), Workpiece::NONE);
    FormulaT rNoR2 = equation(getVarHoldsRing(r,1), Workpiece::NONE);
    FormulaT rNoR3 = equation(getVarHoldsRing(r,2), Workpiece::NONE);
    FormulaT rNoCap = equation(getVarHoldsCap(r), Workpiece::NONE);
    
    FormulaT rNoW(FormulaType::AND, {rNoBase, rNoR1, rNoR2, rNoR3, rNoCap});
    
    return FormulaT(FormulaType::AND, 
       {holdsNoWorkpiece, isNotTransparent,
        capLoaded, capNotLoaded,
        noWorkpiece, WorkpieceInOutput,
        noCap,
        rTocs, 
        rNoW});
    //@todo getMountCapStepNotFinishedFormula über alle Order o 
}


FormulaT StepFormula::getMountCapStepNotFinishedFormula(Robot &r, Order &o, CapStation &cs){
    FormulaT sameBase = equation(getPrevStep()->getVarHoldsBase(r), o.getBaseColorReq());
    FormulaT sameR0 = equation(getPrevStep()->getVarHoldsRing(r,0), o.getRingColorReq(0));
    FormulaT sameR1 = equation(getPrevStep()->getVarHoldsRing(r,1), o.getRingColorReq(1));
    FormulaT sameR2 = equation(getPrevStep()->getVarHoldsRing(r,2), o.getRingColorReq(2));
    FormulaT correctCapColor = equation(getPrevStep()->getVarCapColor(cs), o.getBaseColorReq());
    FormulaT notFinished = equation(getPrevStep()->getVarCapProgress(o), Order::NOTFINISHED);
    FormulaT finished = equation(getVarCapProgress(o), Order::FINISHED);
    
    return FormulaT(FormulaType::AND,
       {sameBase, 
        sameR0, 
        sameR1, 
        sameR2, 
        correctCapColor, 
        notFinished, 
        finished});
}

FormulaT StepFormula::getSetupRingColorFormula(Robot &r, Workpiece::Color c, RingStation &rs){
    FormulaT notSetUp = equation(getPrevStep()->getVarRingColor(rs), Workpiece::NONE);
    FormulaT setUp = equation(getVarRingColor(rs), c);
    
    FormulaT expectsAddBases = equation(getBaseReq(rs), rs.getReqBases(c));
    
    //getSetupRingColorFormula
    
}

FormulaT StepFormula::getSetupRingColorNotFinishedFormula(Robot &r, Workpiece::Color c, Order &o){
    
    FormulaT r0same = equation(getVarRingProgress(o, 0), getPrevStep()->getVarRingProgress(o, 0));
    FormulaT r1same = equation(getVarRingProgress(o, 1), getPrevStep()->getVarRingProgress(o, 1));
    FormulaT r2same = equation(getVarRingProgress(o, 2), getPrevStep()->getVarRingProgress(o, 2));
    
    FormulaT correctColorR0 = equation(o.getRingColorReq(0), c);
    FormulaT r0NotMounted = equation(getPrevStep()->getVarRingProgress(o, 0), Order::NOTFINISHED);
    FormulaT r0Setup = equation(getVarRingProgress(o, 0), Order::SETUP);
    
    FormulaT r0(FormulaType::AND, {correctColorR0, r0NotMounted, r0Setup, r1same, r2same});
        
    FormulaT correctColorR1 = equation(o.getRingColorReq(1), c);
    FormulaT r1NotMounted = equation(getPrevStep()->getVarRingProgress(o, 1), Order::NOTFINISHED);
    FormulaT r1Setup = equation(getVarRingProgress(o, 1), Order::SETUP);
    
    FormulaT r1(FormulaType::AND, {correctColorR1, r1NotMounted, r1Setup, r0same, r2same});
    
    FormulaT correctColorR2 = equation(o.getRingColorReq(2), c);
    FormulaT r2NotMounted = equation(getPrevStep()->getVarRingProgress(o, 2), Order::NOTFINISHED);
    FormulaT r2Setup = equation(getVarRingProgress(o, 2), Order::SETUP);
    
    FormulaT r2(FormulaType::AND, {correctColorR2, r2NotMounted, r2Setup, r0same, r1same});
    
    return FormulaT(FormulaType::OR, {r0, r1, r2});
}

FormulaT StepFormula::getFeedAdditionalBaseFormula(Robot &r, RingStation &rs){
    FormulaT holdsBase = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    FormulaT holdsNothing = equation(getVarHoldsBase(r), Workpiece::NONE);
    
    FormulaT needAddBases(Constr(getBaseReq(rs), Relation::GREATER, Rational(0)));
    FormulaT isFeedWithAddBase = equation(getBaseReq(rs), Pol(getPrevStep()->getBaseReq(rs) - Rational(1)));
    
    FormulaT outputEmpty = equation(getPrevStep()->getVarHoldsBase(rs), Workpiece::NONE);
    
    return FormulaT(FormulaType::AND, {holdsBase, holdsNothing, needAddBases, isFeedWithAddBase, outputEmpty});
}

FormulaT StepFormula::getMountRingFormula(Robot &r, RingStation &rs){
    FormulaT holdsBase = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    FormulaT notTransparent = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::TRANSPARENT));
    FormulaT noCap = equation(getPrevStep()->getVarHoldsCap(r), Workpiece::NONE);
    FormulaT not3Rings = equation(getPrevStep()->getVarHoldsRing(r,3), Workpiece::NONE);
    
    FormulaT outputEmpty = equation(getPrevStep()->getVarHoldsBase(rs), Workpiece::NONE);
    
    FormulaT setupForColor = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarRingColor(rs), Workpiece::NONE));
    FormulaT notSetupForColor = equation(getPrevStep()->getVarRingColor(rs), Workpiece::NONE);
    FormulaT addBasesReqFullfilled = equation(getPrevStep()->getBaseReq(rs), 0);
    
    FormulaT cond(FormulaType::AND, {holdsBase, notTransparent, noCap, not3Rings, outputEmpty, setupForColor, notSetupForColor, addBasesReqFullfilled});
         
    FormulaT holdsNothingB = equation(getVarHoldsBase(r), Workpiece::NONE);
    FormulaT holdsNothingR0 = equation(getVarHoldsRing(r, 0), Workpiece::NONE);
    FormulaT holdsNothingR1 = equation(getVarHoldsRing(r, 1), Workpiece::NONE);
    FormulaT holdsNothingR2 = equation(getVarHoldsRing(r, 2), Workpiece::NONE);
    FormulaT holdsNothingC = equation(getVarHoldsCap(r), Workpiece::NONE);
    FormulaT holdsNothing(FormulaType::AND, {holdsNothingB, holdsNothingR0, holdsNothingR1, holdsNothingR2, holdsNothingC});
    
    FormulaT rR0Same = equation(getPrevStep()->getVarHoldsRing(r,0), getVarHoldsRing(r,0));
    FormulaT rR1Same = equation(getPrevStep()->getVarHoldsRing(r,1), getVarHoldsRing(r,1));
    FormulaT rR2Same = equation(getPrevStep()->getVarHoldsRing(r,2), getVarHoldsRing(r,2));
    
    FormulaT rR0None = equation(getPrevStep()->getVarHoldsRing(r,0), Workpiece::NONE);
    FormulaT rsMountR0 = equation(getVarHoldsRing(rs,0), getVarRingColor(rs));
    FormulaT mountR0(FormulaType::AND, {rR0None, rR1Same, rR2Same, rsMountR0});
    
    FormulaT rR1None = equation(getPrevStep()->getVarHoldsRing(r,1), Workpiece::NONE);
    FormulaT rsMountR1 = equation(getVarHoldsRing(rs,1), getPrevStep()->getVarRingColor(rs));
    FormulaT mountR1(FormulaType::AND, {rR1None, rR0Same, rR2Same, rsMountR1});
    
    FormulaT rR2None = equation(getPrevStep()->getVarHoldsRing(r,2), Workpiece::NONE);
    FormulaT rsMountR2 = equation(getVarHoldsRing(rs,2), getPrevStep()->getVarRingColor(rs));
    FormulaT mountR2(FormulaType::AND, {rR2None, rR0Same, rR1Same, rsMountR2});
    
    FormulaT mount(FormulaType::OR, {mountR0, mountR1, mountR2});
    
    return FormulaT(FormulaType::AND, {cond, holdsNothing, mount});
}

FormulaT StepFormula::getMountRingNotFinishedFormula(Robot &r, RingStation &rs, Order &o){
    FormulaT orderB = equation(getPrevStep()->getVarHoldsBase(r), o.getBaseColorReq());

    FormulaT r0same = equation(getVarRingProgress(o,0), getPrevStep()->getVarRingProgress(o, 0));
    FormulaT r1same = equation(getVarRingProgress(o,1), getPrevStep()->getVarRingProgress(o, 1));
    FormulaT r2same = equation(getVarRingProgress(o,2), getPrevStep()->getVarRingProgress(o, 2));
    
    FormulaT rR0None = equation(getPrevStep()->getVarHoldsRing(r, 0), Workpiece::NONE);
    FormulaT orderR0 = equation(getPrevStep()->getVarRingColor(rs), o.getRingColorReq(0));
    FormulaT r0Setup = equation(getPrevStep()->getVarRingProgress(o, 0), Order::SETUP);
    FormulaT r0Finished = equation(getVarRingProgress(o,0), Order::FINISHED);
    
    FormulaT r0Work(FormulaType::AND, {rR0None, orderR0, r0Setup, r0Finished, r1same, r2same});
    
    FormulaT rR1None = equation(getPrevStep()->getVarHoldsRing(r,1), Workpiece::NONE);
    FormulaT orderR1 = equation(getPrevStep()->getVarRingColor(rs), o.getRingColorReq(0));
    FormulaT r1Setup = equation(getPrevStep()->getVarRingProgress(o,1), Order::SETUP);
    FormulaT r1Finished = equation(getVarRingProgress(o,1), Order::FINISHED);
    
    FormulaT r1Work(FormulaType::AND, {rR1None, orderR1, r1Setup, r1Finished, r0same, r2same});
    
    FormulaT rR2None = equation(getPrevStep()->getVarHoldsRing(r,2), Workpiece::NONE);
    FormulaT orderR2 = equation(getPrevStep()->getVarRingColor(rs), o.getRingColorReq(0));
    FormulaT r2Setup = equation(getPrevStep()->getVarRingProgress(o,2), Order::SETUP);
    FormulaT r2Finished = equation(getVarRingProgress(o,2), Order::FINISHED);
    
    FormulaT r2Work(FormulaType::AND, {rR2None, orderR2, r2Setup, r2Finished, r0same, r1same});
    
    return FormulaT(FormulaType::OR, {orderB, r0Work, r1Work, r2Work});
}

FormulaT StepFormula::getCollectWorkpieceFormula(Robot &r, Station &s){
    FormulaT holdsNothing = equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE);
    FormulaT outputNotEmpty = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(s), Workpiece::NONE));
    
    FormulaT rHoldsB = equation(getVarHoldsBase(r), getPrevStep()->getVarHoldsBase(s));
    FormulaT rHoldsR0 = equation(getVarHoldsRing(r,0), getPrevStep()->getVarHoldsRing(r,0));
    FormulaT rHoldsR1 = equation(getVarHoldsRing(r,1), getPrevStep()->getVarHoldsRing(r,1));
    FormulaT rHoldsR2 = equation(getVarHoldsRing(r,2), getPrevStep()->getVarHoldsRing(r,2));
    FormulaT rHoldsC = equation(getVarHoldsCap(r), getPrevStep()->getVarHoldsCap(s));
    
    FormulaT rHoldsW(FormulaType::AND, {rHoldsB, rHoldsR0, rHoldsR1, rHoldsR2, rHoldsC});
    
    FormulaT csBEmpty = equation(getVarHoldsBase(s), Workpiece::NONE);
    FormulaT csR0Empty = equation(getVarHoldsRing(s,0), Workpiece::NONE);
    FormulaT csR1Empty = equation(getVarHoldsRing(s,1), Workpiece::NONE);
    FormulaT cR2BEmpty = equation(getVarHoldsRing(s,2), Workpiece::NONE);
    FormulaT csCEmpty = equation(getVarHoldsCap(s), Workpiece::NONE);
    
    FormulaT csEmpty(FormulaType::AND, {csBEmpty, csR0Empty, csR1Empty, cR2BEmpty, csCEmpty});
    
    return FormulaT(FormulaType::AND, {holdsNothing, outputNotEmpty, rHoldsW, csEmpty});
}

FormulaT StepFormula::getDeliverWorkpieceFormula(Robot &r, Station &d){
    FormulaT hasBase = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    FormulaT hasCap = FormulaT(FormulaType::NOT, equation(getPrevStep()->getVarHoldsCap(r), Workpiece::NONE));
    
    FormulaT holdsNothingB = equation(getVarHoldsBase(r), Workpiece::NONE);
    FormulaT holdsNothingR0 = equation(getVarHoldsRing(r, 0), Workpiece::NONE);
    FormulaT holdsNothingR1 = equation(getVarHoldsRing(r, 1), Workpiece::NONE);
    FormulaT holdsNothingR2 = equation(getVarHoldsRing(r, 2), Workpiece::NONE);
    FormulaT holdsNothingC = equation(getVarHoldsCap(r), Workpiece::NONE);
    
    FormulaT holdsNothing(FormulaType::AND, {holdsNothingB, holdsNothingR0, holdsNothingR1, holdsNothingR2, holdsNothingC});
    
    return FormulaT(FormulaType::AND, {hasBase, hasCap, holdsNothing});
}

FormulaT StepFormula::getMarkOrderDeliveredFormula(Robot &r, Order &o){
    FormulaT correctB = equation(getVarHoldsBase(r), o.getBaseColorReq());
    FormulaT correctR0 = equation(getVarHoldsRing(r,0), o.getRingColorReq(0));
    FormulaT correctR1 = equation(getVarHoldsRing(r,1), o.getRingColorReq(1));
    FormulaT correctR2 = equation(getVarHoldsRing(r,2), o.getRingColorReq(2));
    FormulaT correctC = equation(getVarHoldsCap(r), o.getCapColorReq());
    
    FormulaT correctW(FormulaType::AND, {correctB, correctR0, correctR1, correctR2, correctC});
    
    FormulaT OrderFinishedB = equation(getVarBaseProgress(o), Order::FINISHED);
    FormulaT OrderFinishedR0 = equation(getVarRingProgress(o,0), Order::FINISHED);
    FormulaT OrderFinishedR1 = equation(getVarRingProgress(o,1), Order::FINISHED);
    FormulaT OrderFinishedR2 = equation(getVarRingProgress(o,2), Order::FINISHED);
    FormulaT OrderFinishedC = equation(getVarCapProgress(o), Order::FINISHED);
    
    FormulaT OrderFinished(FormulaType::AND, {OrderFinishedB, OrderFinishedR0, OrderFinishedR1, OrderFinishedR2, OrderFinishedC});
    
    FormulaT OrderDeliveredB = equation(getVarBaseProgress(o), Order::DELIVERED);
    FormulaT OrderDeliveredR0 = equation(getVarRingProgress(o,0), Order::DELIVERED);
    FormulaT OrderDeliveredR1 = equation(getVarRingProgress(o,1), Order::DELIVERED);
    FormulaT OrderDeliveredR2 = equation(getVarRingProgress(o,2), Order::DELIVERED);
    FormulaT OrderDeliveredC = equation(getVarCapProgress(o), Order::DELIVERED);
    
    FormulaT OrderDelivered(FormulaType::AND, {OrderDeliveredB, OrderDeliveredR0, OrderDeliveredR1, OrderDeliveredR2, OrderDeliveredC});
    
    return FormulaT(FormulaType::AND, {correctW, OrderFinished, OrderDelivered}); 
}