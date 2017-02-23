#include "StepFormula.h"

int StepFormula::newId = 0;

StepFormula::StepFormula(stepFormula_ptr &previousStep, GameData& gameData) : StepFormula(gameData) {
    setPreviousStep(previousStep);
}

StepFormula::StepFormula(GameData& gameData) : gameData(gameData) {
    setStepNumber(newId++);
}

StepFormula::~StepFormula() {
}
//@todo

std::vector<stepFormula_ptr> StepFormula::generateSteps(uint amount, GameData& gameData) {
    std::vector<stepFormula_ptr> steps;
    std::shared_ptr<StepFormula> prevStep = nullptr;

    for (int number = 0; number <= amount; number++) {

        std::shared_ptr<StepFormula> step = std::make_shared<StepFormula>(prevStep, gameData);
        steps.push_back(step);
        prevStep = step;
    }
    return steps;
}

//Getter

stepFormula_ptr StepFormula::getPrevStep() {
    return this->previousStep;
}

int StepFormula::getStepNumber() {
    return this->stepNumber;
}

string StepFormula::getStepName() {
    return std::to_string(getStepNumber());
}

GameData StepFormula::getGameData() const {
    return this->gameData;
}

Variable StepFormula::getVariable(string varName) {
    Variable var;
    if (varibles.find(varName) == varibles.end()) {
        var = carl::freshIntegerVariable(varName);
        varibles[varName] = var;
    } else {
        var = varibles.find(varName)->second;
    }
    return var;
}

//Setter

void StepFormula::setStepNumber(int stepNumber) {
    this->stepNumber = stepNumber;
}

void StepFormula::setPreviousStep(stepFormula_ptr &previousStep) {
    this->previousStep = previousStep;
}

Formula StepFormula::createInitialStateRobots() {
    GameData gD = getGameData();
    std::vector<Formula> formulas;
    for (auto const& r : gD.getRobots()) {
        formulas.push_back(holdsBaseFormula(*r, r->getWorkpiece().getBaseColor()));
    }
    return Formula();
}

Formula StepFormula::createInitialStateStations() {
    GameData gD = getGameData();
    std::vector<Formula> formulas;
    for (auto const& rs : gD.getRingStations()) {

    }
    for (auto const& cs : gD.getCapStations()) {

    }
    
    return Formula();
}

Formula StepFormula::createInitialStateOrders() {
    GameData gD = getGameData();
    for (auto const& r : gD.getOrders()) {

    }
    return Formula();
}

Formula StepFormula::holdsWorkpiece(Machine &m, Workpiece::Color color) {
    equation(getVarHoldsRing(m, number), color);
}


Formula StepFormula::createInitialState() {
    return Formula(carl::FormulaType::AND, {createInitialStateRobots(),
        createInitialStateStations(),
        createInitialStateOrders()});
}

Formula StepFormula::create() {
    if (getStepNumber() == 0)
        return createInitialState();

    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        formulas.push_back(equation(getPrevStep()->getVarHoldsBase(*r), Workpiece::NONE));
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Variable StepFormula::getVarHoldsBase(Machine &m) {
    return getVariable("B(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarHoldsRing(Machine &m, int i) {
    return getVariable("R" + std::to_string(i) + "(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarHoldsCap(Machine &m) {
    return getVariable("C(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarBaseProgress(Order &o) {
    return getVariable("BP(" + o.getVarIdentifier() + "," + getStepName() + ")");
};

Variable StepFormula::getVarRingProgress(Order &o, int i) {
    return getVariable("R" + std::to_string(i) + "P(" + o.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarCapProgress(Order &o) {
    return getVariable("CP(" + o.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarMachineOccupied(Machine &m) {
    return getVariable("OCC(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarMovingTime(Robot r, Station &m) {
    return getVariable("MOV(" + r.getVarIdentifier() + "," + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarCapColor(CapStation &cs) {
    return getVariable("CCOL(" + cs.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarRingColor(RingStation &rs) {
    return getVariable("RCOL(" + rs.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getBaseReq(RingStation &rs) {
    return getVariable("BREQ(" + rs.getVarIdentifier() + "," + getStepName() + ")");
}

Formula StepFormula::equation(int value1, int value2) {
    return Formula(Pol(Rational(value1) - Rational(value2)), carl::Relation::EQ);
}

Formula StepFormula::equation(Variable var, int value) {
    return Formula(Pol(var - Rational(value)), carl::Relation::EQ);
}

Formula StepFormula::equation(Variable var1, Variable var2) {
    return Formula(Pol(var1) - Pol(var2), carl::Relation::EQ);
}

Formula StepFormula::equation(Variable var1, Pol pol) {
    return Formula(Pol(var1) - pol, carl::Relation::EQ);
}

/*@todo generate constraints about a workpiece in separate functions to provide better readability 
 * and exchangeability of the workpiece encoding 
 * using StepFormula::holdsWorkpiece(Machine &m, Workpiece::Color color) 
 */

Formula StepFormula::getCollectBaseFormula(Robot &r, Workpiece::Color c) {
    Formula baseNotMounted = equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE);
    Formula baseMounted = equation(getVarHoldsBase(r), c);
    return Formula(carl::FormulaType::AND,{baseNotMounted, baseMounted});
    //@todo getCollectBaseStepNotFinishedFormula über alle Order o mit o.getBaseColorReq() == c
}

Formula StepFormula::getCollectBaseStepNotFinishedFormula(Robot &r, Order &o) {
    Formula workstepIsNotFinished = equation(getPrevStep()->getVarBaseProgress(o), Order::NOTFINISHED);
    Formula workstepIsFinished = equation(getVarBaseProgress(o), Order::FINISHED);
    return Formula(carl::FormulaType::AND,{workstepIsNotFinished, workstepIsFinished});
}

Formula StepFormula::getFeedCapFormula(Robot &r, Workpiece::Color c, CapStation &cs) {
    Formula brHoldsNothing = equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE);
    Formula arHoldsNothing = equation(getVarHoldsBase(r), Workpiece::NONE);

    Formula capNotLoaded = equation(getPrevStep()->getVarCapColor(cs), Workpiece::NONE);
    Formula capLoaded = equation(getVarCapColor(cs), c);

    Formula outputNotLoaded = equation(getPrevStep()->getVarHoldsBase(cs), Workpiece::NONE);
    Formula outputLoaded = equation(getVarHoldsBase(cs), Workpiece::TRANSPARENT);

    return Formula(carl::FormulaType::AND,
       {brHoldsNothing, 
        arHoldsNothing,
        capNotLoaded, 
        capLoaded,
        outputNotLoaded, 
        outputLoaded});
    //@todo getCollectBaseStepNotFinishedFormula über alle Order o mit o.getRingColorReq() == c
}

Formula StepFormula::getFeedCapStepNotFinishedFormula(Robot &r, Order &o) {
    Formula workstepIsNotFinished = equation(getPrevStep()->getVarCapProgress(o), Order::NOTFINISHED);
    Formula workstepIsSetUp = equation(getVarCapProgress(o), Order::SETUP);
    return Formula(carl::FormulaType::AND, {workstepIsNotFinished, workstepIsSetUp});
}

Formula StepFormula::getMountCapFormula(Robot &r, CapStation &cs) {
    Formula holdsNoWorkpiece = Formula( carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    Formula isNotTransparent = Formula( carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::TRANSPARENT));

    Formula noCap = Formula(equation(getPrevStep()->getVarHoldsCap(r), Workpiece::NONE));
    
    Formula capLoaded = Formula( carl::FormulaType::NOT, equation(getPrevStep()->getVarCapColor(cs), Workpiece::NONE));
    Formula capNotLoaded = equation(getVarCapColor(cs), Workpiece::NONE);
    
    Formula noWorkpiece = equation(getPrevStep()->getVarHoldsBase(cs), Workpiece::NONE);
    Formula WorkpieceInOutput = equation(getPrevStep()->getVarHoldsBase(cs), Workpiece::NONE);
    
    Formula rTocsB = equation(getPrevStep()->getVarHoldsBase(r), getVarHoldsBase(cs));
    Formula rTocsR1 = equation(getPrevStep()->getVarHoldsRing(r,0), getVarHoldsRing(cs,0));
    Formula rTocsR2 = equation(getPrevStep()->getVarHoldsRing(r,1), getVarHoldsRing(cs,1));
    Formula rTocsR3 = equation(getPrevStep()->getVarHoldsRing(r,2), getVarHoldsRing(cs,2));
    Formula capMounted = equation(getVarHoldsCap(cs), getPrevStep()->getVarCapColor(cs));
    
    Formula rTocs(carl::FormulaType::AND, {rTocsB, rTocsR1, rTocsR2, rTocsR3, capMounted});
    
    Formula rNoBase = equation(getVarHoldsBase(r), Workpiece::NONE);
    Formula rNoR1 = equation(getVarHoldsRing(r,0), Workpiece::NONE);
    Formula rNoR2 = equation(getVarHoldsRing(r,1), Workpiece::NONE);
    Formula rNoR3 = equation(getVarHoldsRing(r,2), Workpiece::NONE);
    Formula rNoCap = equation(getVarHoldsCap(r), Workpiece::NONE);
    
    Formula rNoW(carl::FormulaType::AND, {rNoBase, rNoR1, rNoR2, rNoR3, rNoCap});
    
    return Formula(carl::FormulaType::AND, 
       {holdsNoWorkpiece, isNotTransparent,
        capLoaded, capNotLoaded,
        noWorkpiece, WorkpieceInOutput,
        noCap,
        rTocs, 
        rNoW});
    //@todo getMountCapStepNotFinishedFormula über alle Order o 
}


Formula StepFormula::getMountCapStepNotFinishedFormula(Robot &r, Order &o, CapStation &cs){
    Formula sameBase = equation(getPrevStep()->getVarHoldsBase(r), o.getBaseColorReq());
    Formula sameR0 = equation(getPrevStep()->getVarHoldsRing(r,0), o.getRingColorReq(0));
    Formula sameR1 = equation(getPrevStep()->getVarHoldsRing(r,1), o.getRingColorReq(1));
    Formula sameR2 = equation(getPrevStep()->getVarHoldsRing(r,2), o.getRingColorReq(2));
    Formula correctCapColor = equation(getPrevStep()->getVarCapColor(cs), o.getBaseColorReq());
    Formula notFinished = equation(getPrevStep()->getVarCapProgress(o), Order::NOTFINISHED);
    Formula finished = equation(getVarCapProgress(o), Order::FINISHED);
    
    return Formula(carl::FormulaType::AND,
       {sameBase, 
        sameR0, 
        sameR1, 
        sameR2, 
        correctCapColor, 
        notFinished, 
        finished});
}

Formula StepFormula::getSetupRingColorFormula(Robot &r, Workpiece::Color c, RingStation &rs){
    Formula notSetUp = equation(getPrevStep()->getVarRingColor(rs), Workpiece::NONE);
    Formula setUp = equation(getVarRingColor(rs), c);
    
    Formula expectsAddBases = equation(getBaseReq(rs), rs.getNeededAdditinalBases(c));
    
    //getSetupRingColorFormula
    
}

Formula StepFormula::getSetupRingColorNotFinishedFormula(Robot &r, Workpiece::Color c, Order &o){
    
    Formula r0same = equation(getVarRingProgress(o, 0), getPrevStep()->getVarRingProgress(o, 0));
    Formula r1same = equation(getVarRingProgress(o, 1), getPrevStep()->getVarRingProgress(o, 1));
    Formula r2same = equation(getVarRingProgress(o, 2), getPrevStep()->getVarRingProgress(o, 2));
    
    Formula correctColorR0 = equation(o.getRingColorReq(0), c);
    Formula r0NotMounted = equation(getPrevStep()->getVarRingProgress(o, 0), Order::NOTFINISHED);
    Formula r0Setup = equation(getVarRingProgress(o, 0), Order::SETUP);
    
    Formula r0(carl::FormulaType::AND, {correctColorR0, r0NotMounted, r0Setup, r1same, r2same});
        
    Formula correctColorR1 = equation(o.getRingColorReq(1), c);
    Formula r1NotMounted = equation(getPrevStep()->getVarRingProgress(o, 1), Order::NOTFINISHED);
    Formula r1Setup = equation(getVarRingProgress(o, 1), Order::SETUP);
    
    Formula r1(carl::FormulaType::AND, {correctColorR1, r1NotMounted, r1Setup, r0same, r2same});
    
    Formula correctColorR2 = equation(o.getRingColorReq(2), c);
    Formula r2NotMounted = equation(getPrevStep()->getVarRingProgress(o, 2), Order::NOTFINISHED);
    Formula r2Setup = equation(getVarRingProgress(o, 2), Order::SETUP);
    
    Formula r2(carl::FormulaType::AND, {correctColorR2, r2NotMounted, r2Setup, r0same, r1same});
    
    return Formula(carl::FormulaType::OR, {r0, r1, r2});
}

Formula StepFormula::getFeedAdditionalBaseFormula(Robot &r, RingStation &rs){
    Formula holdsBase = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    Formula holdsNothing = equation(getVarHoldsBase(r), Workpiece::NONE);
    
    Formula needAddBases(Constr(getBaseReq(rs), carl::Relation::GREATER, Rational(0)));
    Formula isFeedWithAddBase = equation(getBaseReq(rs), Pol(getPrevStep()->getBaseReq(rs) - Rational(1)));
    
    Formula outputEmpty = equation(getPrevStep()->getVarHoldsBase(rs), Workpiece::NONE);
    
    return Formula(carl::FormulaType::AND, {holdsBase, holdsNothing, needAddBases, isFeedWithAddBase, outputEmpty});
}

Formula StepFormula::getMountRingFormula(Robot &r, RingStation &rs){
    Formula holdsBase = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    Formula notTransparent = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::TRANSPARENT));
    Formula noCap = equation(getPrevStep()->getVarHoldsCap(r), Workpiece::NONE);
    Formula not3Rings = equation(getPrevStep()->getVarHoldsRing(r,3), Workpiece::NONE);
    
    Formula outputEmpty = equation(getPrevStep()->getVarHoldsBase(rs), Workpiece::NONE);
    
    Formula setupForColor = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarRingColor(rs), Workpiece::NONE));
    Formula notSetupForColor = equation(getPrevStep()->getVarRingColor(rs), Workpiece::NONE);
    Formula addBasesReqFullfilled = equation(getPrevStep()->getBaseReq(rs), 0);
    
    Formula cond(carl::FormulaType::AND, {holdsBase, notTransparent, noCap, not3Rings, outputEmpty, setupForColor, notSetupForColor, addBasesReqFullfilled});
         
    Formula holdsNothingB = equation(getVarHoldsBase(r), Workpiece::NONE);
    Formula holdsNothingR0 = equation(getVarHoldsRing(r, 0), Workpiece::NONE);
    Formula holdsNothingR1 = equation(getVarHoldsRing(r, 1), Workpiece::NONE);
    Formula holdsNothingR2 = equation(getVarHoldsRing(r, 2), Workpiece::NONE);
    Formula holdsNothingC = equation(getVarHoldsCap(r), Workpiece::NONE);
    Formula holdsNothing(carl::FormulaType::AND, {holdsNothingB, holdsNothingR0, holdsNothingR1, holdsNothingR2, holdsNothingC});
    
    Formula rR0Same = equation(getPrevStep()->getVarHoldsRing(r,0), getVarHoldsRing(r,0));
    Formula rR1Same = equation(getPrevStep()->getVarHoldsRing(r,1), getVarHoldsRing(r,1));
    Formula rR2Same = equation(getPrevStep()->getVarHoldsRing(r,2), getVarHoldsRing(r,2));
    
    Formula rR0None = equation(getPrevStep()->getVarHoldsRing(r,0), Workpiece::NONE);
    Formula rsMountR0 = equation(getVarHoldsRing(rs,0), getVarRingColor(rs));
    Formula mountR0(carl::FormulaType::AND, {rR0None, rR1Same, rR2Same, rsMountR0});
    
    Formula rR1None = equation(getPrevStep()->getVarHoldsRing(r,1), Workpiece::NONE);
    Formula rsMountR1 = equation(getVarHoldsRing(rs,1), getPrevStep()->getVarRingColor(rs));
    Formula mountR1(carl::FormulaType::AND, {rR1None, rR0Same, rR2Same, rsMountR1});
    
    Formula rR2None = equation(getPrevStep()->getVarHoldsRing(r,2), Workpiece::NONE);
    Formula rsMountR2 = equation(getVarHoldsRing(rs,2), getPrevStep()->getVarRingColor(rs));
    Formula mountR2(carl::FormulaType::AND, {rR2None, rR0Same, rR1Same, rsMountR2});
    
    Formula mount(carl::FormulaType::OR, {mountR0, mountR1, mountR2});
    
    return Formula(carl::FormulaType::AND, {cond, holdsNothing, mount});
}

Formula StepFormula::getMountRingNotFinishedFormula(Robot &r, RingStation &rs, Order &o){
    Formula orderB = equation(getPrevStep()->getVarHoldsBase(r), o.getBaseColorReq());

    Formula r0same = equation(getVarRingProgress(o,0), getPrevStep()->getVarRingProgress(o, 0));
    Formula r1same = equation(getVarRingProgress(o,1), getPrevStep()->getVarRingProgress(o, 1));
    Formula r2same = equation(getVarRingProgress(o,2), getPrevStep()->getVarRingProgress(o, 2));
    
    Formula rR0None = equation(getPrevStep()->getVarHoldsRing(r, 0), Workpiece::NONE);
    Formula orderR0 = equation(getPrevStep()->getVarRingColor(rs), o.getRingColorReq(0));
    Formula r0Setup = equation(getPrevStep()->getVarRingProgress(o, 0), Order::SETUP);
    Formula r0Finished = equation(getVarRingProgress(o,0), Order::FINISHED);
    
    Formula r0Work(carl::FormulaType::AND, {rR0None, orderR0, r0Setup, r0Finished, r1same, r2same});
    
    Formula rR1None = equation(getPrevStep()->getVarHoldsRing(r,1), Workpiece::NONE);
    Formula orderR1 = equation(getPrevStep()->getVarRingColor(rs), o.getRingColorReq(0));
    Formula r1Setup = equation(getPrevStep()->getVarRingProgress(o,1), Order::SETUP);
    Formula r1Finished = equation(getVarRingProgress(o,1), Order::FINISHED);
    
    Formula r1Work(carl::FormulaType::AND, {rR1None, orderR1, r1Setup, r1Finished, r0same, r2same});
    
    Formula rR2None = equation(getPrevStep()->getVarHoldsRing(r,2), Workpiece::NONE);
    Formula orderR2 = equation(getPrevStep()->getVarRingColor(rs), o.getRingColorReq(0));
    Formula r2Setup = equation(getPrevStep()->getVarRingProgress(o,2), Order::SETUP);
    Formula r2Finished = equation(getVarRingProgress(o,2), Order::FINISHED);
    
    Formula r2Work(carl::FormulaType::AND, {rR2None, orderR2, r2Setup, r2Finished, r0same, r1same});
    
    return Formula(carl::FormulaType::OR, {orderB, r0Work, r1Work, r2Work});
}

Formula StepFormula::getCollectWorkpieceFormula(Robot &r, Station &s){
    Formula holdsNothing = equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE);
    Formula outputNotEmpty = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(s), Workpiece::NONE));
    
    Formula rHoldsB = equation(getVarHoldsBase(r), getPrevStep()->getVarHoldsBase(s));
    Formula rHoldsR0 = equation(getVarHoldsRing(r,0), getPrevStep()->getVarHoldsRing(r,0));
    Formula rHoldsR1 = equation(getVarHoldsRing(r,1), getPrevStep()->getVarHoldsRing(r,1));
    Formula rHoldsR2 = equation(getVarHoldsRing(r,2), getPrevStep()->getVarHoldsRing(r,2));
    Formula rHoldsC = equation(getVarHoldsCap(r), getPrevStep()->getVarHoldsCap(s));
    
    Formula rHoldsW(carl::FormulaType::AND, {rHoldsB, rHoldsR0, rHoldsR1, rHoldsR2, rHoldsC});
    
    Formula csBEmpty = equation(getVarHoldsBase(s), Workpiece::NONE);
    Formula csR0Empty = equation(getVarHoldsRing(s,0), Workpiece::NONE);
    Formula csR1Empty = equation(getVarHoldsRing(s,1), Workpiece::NONE);
    Formula cR2BEmpty = equation(getVarHoldsRing(s,2), Workpiece::NONE);
    Formula csCEmpty = equation(getVarHoldsCap(s), Workpiece::NONE);
    
    Formula csEmpty(carl::FormulaType::AND, {csBEmpty, csR0Empty, csR1Empty, cR2BEmpty, csCEmpty});
    
    return Formula(carl::FormulaType::AND, {holdsNothing, outputNotEmpty, rHoldsW, csEmpty});
}

Formula StepFormula::getDeliverWorkpieceFormula(Robot &r, Station &d){
    Formula hasBase = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsBase(r), Workpiece::NONE));
    Formula hasCap = Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarHoldsCap(r), Workpiece::NONE));
    
    Formula holdsNothingB = equation(getVarHoldsBase(r), Workpiece::NONE);
    Formula holdsNothingR0 = equation(getVarHoldsRing(r, 0), Workpiece::NONE);
    Formula holdsNothingR1 = equation(getVarHoldsRing(r, 1), Workpiece::NONE);
    Formula holdsNothingR2 = equation(getVarHoldsRing(r, 2), Workpiece::NONE);
    Formula holdsNothingC = equation(getVarHoldsCap(r), Workpiece::NONE);
    
    Formula holdsNothing(carl::FormulaType::AND, {holdsNothingB, holdsNothingR0, holdsNothingR1, holdsNothingR2, holdsNothingC});
    
    return Formula(carl::FormulaType::AND, {hasBase, hasCap, holdsNothing});
}

Formula StepFormula::getMarkOrderDeliveredFormula(Robot &r, Order &o){
    Formula correctB = equation(getVarHoldsBase(r), o.getBaseColorReq());
    Formula correctR0 = equation(getVarHoldsRing(r,0), o.getRingColorReq(0));
    Formula correctR1 = equation(getVarHoldsRing(r,1), o.getRingColorReq(1));
    Formula correctR2 = equation(getVarHoldsRing(r,2), o.getRingColorReq(2));
    Formula correctC = equation(getVarHoldsCap(r), o.getCapColorReq());
    
    Formula correctW(carl::FormulaType::AND, {correctB, correctR0, correctR1, correctR2, correctC});
    
    Formula OrderFinishedB = equation(getVarBaseProgress(o), Order::FINISHED);
    Formula OrderFinishedR0 = equation(getVarRingProgress(o,0), Order::FINISHED);
    Formula OrderFinishedR1 = equation(getVarRingProgress(o,1), Order::FINISHED);
    Formula OrderFinishedR2 = equation(getVarRingProgress(o,2), Order::FINISHED);
    Formula OrderFinishedC = equation(getVarCapProgress(o), Order::FINISHED);
    
    Formula OrderFinished(carl::FormulaType::AND, {OrderFinishedB, OrderFinishedR0, OrderFinishedR1, OrderFinishedR2, OrderFinishedC});
    
    Formula OrderDeliveredB = equation(getVarBaseProgress(o), Order::DELIVERED);
    Formula OrderDeliveredR0 = equation(getVarRingProgress(o,0), Order::DELIVERED);
    Formula OrderDeliveredR1 = equation(getVarRingProgress(o,1), Order::DELIVERED);
    Formula OrderDeliveredR2 = equation(getVarRingProgress(o,2), Order::DELIVERED);
    Formula OrderDeliveredC = equation(getVarCapProgress(o), Order::DELIVERED);
    
    Formula OrderDelivered(carl::FormulaType::AND, {OrderDeliveredB, OrderDeliveredR0, OrderDeliveredR1, OrderDeliveredR2, OrderDeliveredC});
    
    return Formula(carl::FormulaType::AND, {correctW, OrderFinished, OrderDelivered}); 
}