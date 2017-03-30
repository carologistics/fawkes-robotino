#include "StepFormula.h"
#include <iostream>
#include <time.h>

int StepFormula::newId = 0;

StepFormula::StepFormula(stepFormula_ptr &previousStep, GameData& gameData) : StepFormula(gameData) {
    setPreviousStep(previousStep);
}

std::map<std::string, Variable> StepFormula::getVariables() const {
    return variables;
}

StepFormula::StepFormula(GameData& gameData) : gameData(gameData) {
    setStepNumber(newId++);
}

StepFormula::~StepFormula() {
}

std::vector<stepFormula_ptr> StepFormula::generateSteps(uint amount, GameData& gameData) {
    std::vector<stepFormula_ptr> steps;
    std::shared_ptr<StepFormula> prevStep = nullptr;

    for (uint number = 0; number <= amount; number++) {

        std::shared_ptr<StepFormula> step = std::make_shared<StepFormula>(prevStep, gameData);
        steps.push_back(step);
        prevStep = step;
    }
    return steps;
}

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
    if (variables.find(varName) == variables.end()) {
        var = carl::freshIntegerVariable(varName);
        variables[varName] = var;
    } else {
        var = variables.find(varName)->second;
    }
    return var;
}

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
        formulas.push_back(holdsWorkpiece(*r, r->getWorkpiece()));
        formulas.push_back(createInitialStateMovingTimes(*r));
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createInitialStateMovingTimes(const Robot& r) {
    GameData gD = getGameData();
    std::vector<Formula> formulas;
    for (auto const& s : gD.getStations()) {
        formulas.push_back(equation(getVarMovingTime(r, *s), r.getMovingTime(*s)));
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createInitialStateStations() {
    GameData gD = getGameData();
    std::vector<Formula> formulas;
    for (auto const& s : getGameData().getStations()) {
        formulas.push_back(holdsWorkpiece(*s, s->getWorkpiece()));
        formulas.push_back(equation(getVarMachineOccupied(*s), s->getOccupiedUntil()));
    }

    for (auto const& cs : gD.getCapStations()) {
        formulas.push_back(equation(getVarCapColor(*cs), cs->getFedCapColor()));
    }

    for (auto const& rs : gD.getRingStations()) {
        formulas.push_back(equation(getVarRingColor(*rs), rs->getRingColorSetup()));
        formulas.push_back(equation(getVarBaseReq(*rs), rs->getReqBases()));
    }

    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createInitialState() {
    return Formula(carl::FormulaType::AND,{createInitialStateRobots(), createInitialStateStations()});
}

Formula StepFormula::create() {
    if (getStepNumber() == 0)
        return Formula(carl::FormulaType::TRUE); //createInitialState();

    std::vector<Formula> actions;
    actions.push_back(collectBaseActions());
    actions.push_back(feedCapActions());
    actions.push_back(mountCapActions());
    actions.push_back(setupRingColorActions());
    actions.push_back(feedAdditionalBaseActions());
    actions.push_back(mountRingActions());
    actions.push_back(collectWorkpieceActions());
    actions.push_back(deliverWorkpieceActions());

    return Formula(carl::FormulaType::OR, actions);
}

Formula StepFormula::swapWorkpiece(const Machine &m1, const Machine &m2) {
    Formula baseF = swapBase(m1, m2);

    std::vector<Formula> rings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++)
        rings.push_back(swapRing(m1, m2, i));

    Formula ringsF = Formula(carl::FormulaType::AND, rings);
    Formula capF = swapCap(m1, m2);

    return Formula(carl::FormulaType::AND,{baseF, ringsF, capF});
}

Formula StepFormula::holdsWorkpiece(const Machine& m, const Workpiece& workpiece) {
    Formula baseF = holdsBase(m, workpiece.getBaseColor());

    std::vector<Formula> rings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++)
        rings.push_back(holdsRing(m, workpiece.getRingColor(i), i));

    Formula ringsF = Formula(carl::FormulaType::AND, rings);
    Formula capF = holdsCap(m, workpiece.getCapColor());

    return Formula(carl::FormulaType::AND,{baseF, ringsF, capF});
}

Formula StepFormula::holdsWorkpiecePrev(const Machine& m, const Workpiece& workpiece) {
    Formula baseF = holdsBasePrev(m, workpiece.getBaseColor());

    std::vector<Formula> rings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++)
        rings.push_back(holdsRingPrev(m, workpiece.getRingColor(i), i));

    Formula ringsF = Formula(carl::FormulaType::AND, rings);
    Formula capF = holdsCapPrev(m, workpiece.getCapColor());

    return Formula(carl::FormulaType::AND,{baseF, ringsF, capF});
}

Formula StepFormula::holdsBasePrev(const Machine& m, Workpiece::Color c) {
    return equation(getPrevStep()->getVarHoldsBase(m), c);
}

Formula StepFormula::holdsRingPrev(const Machine& m, Workpiece::Color c, int i) {
    return equation(getPrevStep()->getVarHoldsRing(m, i), c);
}

Formula StepFormula::holdsCapPrev(const Machine& m, Workpiece::Color c) {
    return equation(getPrevStep()->getVarHoldsCap(m), c);
}

Formula StepFormula::orderNotDeliveredPrev(const Order& o){
    return equation(getPrevStep()->getVarOrderDelivered(o), Order::NOTDELIVERED);
}

Formula StepFormula::holdsBase(const Machine& m, Workpiece::Color c) {
    return equation(getVarHoldsBase(m), c);
}

Formula StepFormula::holdsRing(const Machine& m, Workpiece::Color c, int i) {
    return equation(getVarHoldsRing(m, i), c);
}

Formula StepFormula::ringColorSetupPrev(const RingStation& rs, Workpiece::Color c) {
    return equation(getPrevStep()->getVarRingColor(rs), c);
}

Formula StepFormula::ringColorSetup(const RingStation& rs, Workpiece::Color c) {
    return equation(getVarRingColor(rs), c);
}

Formula StepFormula::needsAddBases(const RingStation& rs, Workpiece::Color c) {
    return equation(getVarBaseReq(rs), rs.getNeededAdditinalBases(c));
}

Formula StepFormula::needsAddBasesPrev(const RingStation& rs) {
    return Formula(carl::FormulaType::NOT, equation(getPrevStep()->getVarBaseReq(rs), 0));
}

Formula StepFormula::feedAddBase(const RingStation& rs) {
    return equation(getVarBaseReq(rs), Pol(getPrevStep()->getVarBaseReq(rs) - Rational(1)));
}

Formula StepFormula::loadCapPrev(const CapStation& cs, Workpiece::Color c) {
    return equation(getPrevStep()->getVarCapColor(cs), c);
}

Formula StepFormula::loadCap(const CapStation& cs, Workpiece::Color c) {
    return equation(getVarCapColor(cs), c);
}

Formula StepFormula::orderDelivered(const Order& o){
    return equation(getVarOrderDelivered(o), Order::DELIVERED);
}

Formula StepFormula::mountRingComponent(const RingStation& rs, int i) {
    return equation(getVarHoldsRing(rs, i), getPrevStep()->getVarRingColor(rs));
}

Formula StepFormula::mountRingRings(const RingStation& rs, const Robot& r, int position) {
    std::vector<Formula> rings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++) {
        if (i < position) {
            rings.push_back(swapRing(rs, r, i));
        } else if (i == position) {
            rings.push_back(mountRingComponent(rs, i));
        } else {
            rings.push_back(holdsRing(rs, Workpiece::NONE, i));
        }
    }
    return Formula(carl::FormulaType::AND, rings);
}

Formula StepFormula::mountRingWorkpiece(const RingStation& rs, const Robot & r) {
    Formula baseF = swapBase(rs, r);
    Formula capF = holdsCap(rs, Workpiece::NONE);

    std::vector<Formula> formulas;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++) {
        Formula ringBeneathExists(carl::FormulaType::TRUE);
        if (i > 0) ringBeneathExists = Formula(carl::FormulaType::NOT, holdsRingPrev(r, Workpiece::NONE, i - 1));

        Formula freePrev = holdsRingPrev(r, Workpiece::NONE, i);
        Formula mountOnRingPosition = mountRingRings(rs, r, i);

        Formula mounted(carl::FormulaType::AND, ringBeneathExists, freePrev, mountOnRingPosition);

        formulas.push_back(mounted);
    }

    Formula mountedOnPosition(carl::FormulaType::OR, formulas);

    return Formula(carl::FormulaType::AND,{baseF, capF, mountedOnPosition});
}

Formula StepFormula::mountCapComponent(const CapStation & cs) {
    return equation(getVarHoldsCap(cs), getPrevStep()->getVarCapColor(cs));
}

Formula StepFormula::mountCapWorkpiece(const CapStation& cs, const Robot & r) {
    Formula baseF = swapBase(cs, r);

    std::vector<Formula> rings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++)
        rings.push_back(swapRing(cs, r, i));

    Formula ringsF = Formula(carl::FormulaType::AND, rings);
    Formula capF = mountCapComponent(cs);

    return Formula(carl::FormulaType::AND,{baseF, ringsF, capF});
}

Formula StepFormula::updateReward(int value) {
    return equation(getVarReward(), getPrevStep()->getVarReward() + Rational(value));
}

Formula StepFormula::holdsCap(const Machine& m, Workpiece::Color c) {
    return equation(getVarHoldsCap(m), c);
}

Formula StepFormula::swapBase(const Machine& m1, const Machine & m2) {
    return equation(getVarHoldsBase(m1), getPrevStep()->getVarHoldsBase(m2));
}

Formula StepFormula::swapRing(const Machine& m1, const Machine& m2, int i) {
    return equation(getVarHoldsRing(m1, i), getPrevStep()->getVarHoldsRing(m2, i));
}

Formula StepFormula::swapCap(const Machine& m1, const Machine & m2) {
    return equation(getVarHoldsCap(m1), getPrevStep()->getVarHoldsCap(m2));
}

Variable StepFormula::getVarHoldsBase(const Machine & m) {
    return getVariable("B(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarHoldsRing(const Machine &m, int i) {
    return getVariable("R" + std::to_string(i) + "(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarHoldsCap(const Machine & m) {
    return getVariable("C(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarMachineOccupied(const Station& m) {
    return getVariable("OCC(" + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarMovingTime(const Robot &r, const Station & m) {
    return getVariable("MOV(" + r.getVarIdentifier() + "," + m.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarCapColor(const CapStation & cs) {
    return getVariable("CCOL(" + cs.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarRingColor(const RingStation & rs) {
    return getVariable("RCOL(" + rs.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarBaseReq(const RingStation & rs) {
    return getVariable("BREQ(" + rs.getVarIdentifier() + "," + getStepName() + ")");
}

Variable StepFormula::getVarReward() {
    return getVariable("R(" + getStepName() + ")");
}

Variable StepFormula::getVarOrderDelivered(const Order& o) {
    return getVariable("OD(" + std::to_string(o.getId()) + "," + getStepName() + ")");
}

Formula StepFormula::equation(int value1, int value2) {
    return Formula(Pol(Rational(value1) - Rational(value2)), carl::Relation::EQ);
}

Formula StepFormula::equation(Variable const& var, int value) {
    return Formula(Pol(var - Rational(value)), carl::Relation::EQ);
}

Formula StepFormula::equation(Variable const& var1, Variable const& var2) {
    return Formula(Pol(var1) - Pol(var2), carl::Relation::EQ);
}

Formula StepFormula::equation(Variable const& var1, Pol const& pol) {
    return Formula(Pol(var1) - pol, carl::Relation::EQ);
}

Formula StepFormula::lessEqual(Variable const& var1, Variable const& var2) {
    return Formula(Pol(var1) - Pol(var2), carl::Relation::LEQ);
}

Formula StepFormula::greater(Variable const& var1, Variable const& var2) {
    return Formula(Pol(var1) - Pol(var2), carl::Relation::GREATER);
}

Formula StepFormula::greater(Variable const& var1, Rational const r) {
    return Formula(Constr(Pol(var1) - r, carl::Relation::GREATER));
}

Formula StepFormula::createRemainingStepConstraints(const Robot &robot, const Station &station) {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        if (robot != *r) {
            formulas.push_back(swapWorkpiece(*r, *r));
            for (auto const& s : getGameData().getStations()) {
                formulas.push_back(equation(getVarMovingTime(*r, *s), getPrevStep()->getVarMovingTime(*r, *s)));
            }
        }
    }

    for (auto const& s : getGameData().getStations()) {
        if (station != *s) {
            formulas.push_back(swapWorkpiece(*s, *s));
            formulas.push_back(equation(getVarMachineOccupied(*s), getPrevStep()->getVarMachineOccupied(*s)));
        }
    }

    for (auto const& cs : getGameData().getCapStations()) {
        if (station != *cs) {
            formulas.push_back(equation(getVarCapColor(*cs), getPrevStep()->getVarCapColor(*cs)));
        }
    }

    for (auto const& rs : getGameData().getRingStations()) {
        if (station != *rs) {
            formulas.push_back(equation(getVarRingColor(*rs), getPrevStep()->getVarRingColor(*rs)));
            formulas.push_back(equation(getVarBaseReq(*rs), getPrevStep()->getVarBaseReq(*rs)));
        }
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createRemainingStepConstraints(const RingStation &station) {
    std::vector<Formula> formulas;
    for (auto const& s : getGameData().getStations()) {
        if (station != *s) {
            formulas.push_back(swapWorkpiece(*s, *s));
            formulas.push_back(equation(getVarMachineOccupied(*s), getPrevStep()->getVarMachineOccupied(*s)));
        }
    }

    for (auto const& rs : getGameData().getRingStations()) {
        if (station != *rs) {
            formulas.push_back(equation(getVarRingColor(*rs), getPrevStep()->getVarRingColor(*rs)));
            formulas.push_back(equation(getVarBaseReq(*rs), getPrevStep()->getVarBaseReq(*rs)));
        }
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::stationIsNotBlockedFormula(const Robot& r, const Station& station, int processingTime) {

    Formula timeBlockedLEQprev = lessEqual(getPrevStep()->getVarMachineOccupied(station), getPrevStep()->getVarMovingTime(r, station));
    Formula timeBlockedLEQnext = equation(getVarMachineOccupied(station), getPrevStep()->getVarMovingTime(r, station) + Rational(processingTime));
    Formula timeBlockedLEQ(carl::FormulaType::AND,{timeBlockedLEQprev, timeBlockedLEQnext});

    Formula timeBlockedGprev = greater(getPrevStep()->getVarMachineOccupied(station), getPrevStep()->getVarMovingTime(r, station));
    Formula timeBlockedGnext = equation(getVarMachineOccupied(station), getPrevStep()->getVarMachineOccupied(station) + Rational(processingTime));
    Formula timeBlockedG(carl::FormulaType::AND,{timeBlockedGprev, timeBlockedGnext});

    std::vector<Formula> movingTimesLEQVector;
    std::vector<Formula> movingTimesGVector;

    for (auto const& s : getGameData().getStations()) {
        movingTimesLEQVector.push_back(equation(getVarMovingTime(r, *s), getPrevStep()->getVarMovingTime(r, station) + Rational(station.getMovingTime(*s))));
        movingTimesGVector.push_back(equation(getVarMovingTime(r, *s), getPrevStep()->getVarMachineOccupied(station) + Rational(station.getMovingTime(*s))));
    }

    Formula movingTimesLEQ(carl::FormulaType::AND, movingTimesLEQVector);
    Formula movingTimesG(carl::FormulaType::AND, movingTimesGVector);

    Formula leqTime(carl::FormulaType::AND,{timeBlockedLEQ, movingTimesLEQ});
    Formula greaterTime(carl::FormulaType::AND,{timeBlockedG, movingTimesG});

    return Formula(carl::FormulaType::OR,{leqTime, greaterTime});
}

Formula StepFormula::collectBaseActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& bs : getGameData().getBaseStations()) {
            for (auto const& c : bs->getPossibleBaseColors()) {
                if (getGameData().existsOrderWithBaseReq(c))
                    formulas.push_back(collectBaseAction(*r, c, *bs));
            }
            Formula forRingStation(carl::FormulaType::AND,{needsBaseForRingStation(),
                collectBaseAction(*r, bs->getColorForRingStation(), *bs)});
            formulas.push_back(forRingStation);
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::collectBaseAction(Robot &r, Workpiece::Color c, BaseStation & bs) {
    Formula holdsNothingPrev = holdsBasePrev(r, Workpiece::NONE);
    Formula holdsBase = holdsWorkpiece(r, Workpiece(c));
    Formula time = stationIsNotBlockedFormula(r, bs, bs.getDispenseBaseTime());

    Formula reward = collectBaseActionReward(r, c, bs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, bs);
    return Formula(carl::FormulaType::AND,{holdsNothingPrev, holdsBase, time, reward, inheritRemainingState});
}

Formula StepFormula::existOrderWithBaseColorReq(Workpiece::Color c){
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithBaseReq(c))
        formulas.push_back(orderNotDeliveredPrev(o));
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::collectBaseActionReward(Robot &r, Workpiece::Color c, BaseStation& bs) {
    return updateReward(0);
}

Formula StepFormula::needsBaseForRingStation() {
    std::vector<Formula> rsNeedsAddBases;
    for (auto const& rs : getGameData().getRingStations()) {
        rsNeedsAddBases.push_back(greater(getPrevStep()->getVarBaseReq(*rs), Rational(0)));
    }
    return Formula(carl::FormulaType::OR, rsNeedsAddBases);
}

Formula StepFormula::feedCapActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& cs : getGameData().getCapStations()) {
            for (auto const& c : cs->getPossibleCapColors()) {
                if (getGameData().existsOrderWithCapReq(c))
                    formulas.push_back(feedCapAction(*r, c, *cs));
            }
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::feedCapAction(Robot &r, Workpiece::Color c, CapStation & cs) {
    Formula rHoldsNothingPrev = holdsBasePrev(r, Workpiece::NONE);
    Formula rHoldsNothing = holdsWorkpiece(r, Workpiece());

    Formula capNotLoadedPrev = loadCapPrev(cs, Workpiece::NONE);
    Formula capLoaded = loadCap(cs, c);

    Formula outputNotLoadedPrev = holdsBasePrev(cs, Workpiece::NONE);
    Formula outputLoaded = holdsWorkpiece(cs, Workpiece(Workpiece::TRANSPARENT));

    Formula time = stationIsNotBlockedFormula(r, cs, cs.getFeedCapTime());
    Formula reward = feedCapActionReward(r, c, cs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, cs);

    return Formula(carl::FormulaType::AND,{
        rHoldsNothingPrev,
        rHoldsNothing,
        capNotLoadedPrev,
        capLoaded,
        outputNotLoadedPrev,
        outputLoaded,
        time,
        reward,
        inheritRemainingState
    });
}

Formula StepFormula::existOrderWithCapColorReq(Workpiece::Color c){
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithCapReq(c))
        formulas.push_back(orderNotDeliveredPrev(o));
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::feedCapActionReward(Robot &r, Workpiece::Color c, CapStation &cs) {
    return updateReward(0);
}

Formula StepFormula::mountCapActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& cs : getGameData().getCapStations()) {
            formulas.push_back(mountCapAction(*r, *cs));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::mountCapAction(Robot &r, CapStation & cs) {
    Formula existOrder = existOrderWithCapReq(r, cs);

    Formula rHoldsBasePrev(carl::FormulaType::NOT, holdsBasePrev(r, Workpiece::NONE));
    Formula rBaseNotTransparentPrev(carl::FormulaType::NOT, holdsBasePrev(r, Workpiece::TRANSPARENT));
    Formula rNoCapPrev = holdsCapPrev(r, Workpiece::NONE);

    Formula rHoldsNothing = holdsWorkpiece(r, Workpiece());

    Formula csNoWorkpiecePrev = holdsBasePrev(cs, Workpiece::NONE);
    Formula csCapLoadedPrev(carl::FormulaType::NOT, loadCapPrev(cs, Workpiece::NONE));

    Formula csCapNotLoaded = loadCap(cs, Workpiece::NONE);
    Formula csCapMounted = mountCapWorkpiece(cs, r);

    Formula time = stationIsNotBlockedFormula(r, cs, cs.getMountCapTime());
    Formula reward = mountCapActionReward(r, cs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, cs);

    return Formula(carl::FormulaType::AND,{
        existOrder,
        rHoldsBasePrev,
        rBaseNotTransparentPrev,
        rNoCapPrev,
        rHoldsNothing,
        csNoWorkpiecePrev,
        csCapLoadedPrev,
        csCapNotLoaded,
        csCapMounted,
        time,
        reward,
        inheritRemainingState
    });
}

Formula StepFormula::existOrderWithCapReq(Robot &r, CapStation & cs) {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrders()) {
        formulas.push_back(orderHasCapReq(r, cs, *o));
        formulas.push_back(orderNotDeliveredPrev(*o));
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::orderHasCapReq(Robot &r, CapStation &cs, Order & o) {
    Formula baseF = holdsBasePrev(r, o.getBaseColorReq());

    std::vector<Formula> rings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++)
        rings.push_back(holdsRing(r, o.getRingColorReq(i), i));

    Formula ringsF = Formula(carl::FormulaType::AND, rings);
    Formula capF = loadCapPrev(cs, o.getCapColorReq());

    return Formula(carl::FormulaType::AND,{baseF, ringsF, capF});
}

Formula StepFormula::mountCapActionReward(Robot &r, CapStation & cs) {
    return updateReward(getGameData().getReward().getMountCap());
}

Formula StepFormula::setupRingColorActions() {
    std::vector<Formula> formulas;
    for (auto const& rs : getGameData().getRingStations()) {
        for (auto const& c : rs->getPossibleRingColors()) {
            if (getGameData().existsOrderWithRingReq(c.first))
                formulas.push_back(setupRingColorAction(c.first, *rs));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::setupRingColorAction(Workpiece::Color c, RingStation& rs) {
    Formula emptyOutputPrev = holdsBasePrev(rs, Workpiece::NONE);
    Formula notSetUp = ringColorSetupPrev(rs, Workpiece::NONE);

    Formula emptyOutput = holdsWorkpiece(rs, Workpiece());
    Formula setUp = ringColorSetup(rs, c);

    Formula expectsAddBases = needsAddBases(rs, c);

    Formula reward = setupRingColorActionReward(c, rs);
    Formula inheritRemainingState = createRemainingStepConstraints(rs);

    return Formula(carl::FormulaType::OR,{
        emptyOutputPrev,
        notSetUp,
        emptyOutput,
        setUp,
        expectsAddBases,
        reward
    });
}

Formula StepFormula::existOrderWithRingColorReq(Workpiece::Color c){
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithRingReq(c))
        formulas.push_back(orderNotDeliveredPrev(o));

    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::setupRingColorActionReward(Workpiece::Color c, RingStation & rs) {
    return updateReward(0);
}

Formula StepFormula::feedAdditionalBaseActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& rs : getGameData().getRingStations()) {
            formulas.push_back(feedAdditionalBaseAction(*r, *rs));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::feedAdditionalBaseAction(Robot &r, RingStation & rs) {

    Formula rHoldsBasePrev(carl::FormulaType::NOT, holdsBasePrev(r, Workpiece::NONE));
    Formula rHoldsNoRingPrev = holdsRingPrev(r, Workpiece::NONE, 0);
    Formula rHoldsNoCapPrev = holdsCapPrev(r, Workpiece::NONE);
    Formula rHoldsNothing = holdsWorkpiece(r, Workpiece());

    Formula rsNeedsAddBasesPrev = needsAddBasesPrev(rs);
    Formula rsIsFedWithAddBase = feedAddBase(rs);

    Formula rsOutputEmptyPrev = holdsBasePrev(rs, Workpiece::NONE);

    Formula time = stationIsNotBlockedFormula(r, rs, rs.getFeedBaseTime());
    Formula reward = feedAdditionalBaseActionReward(r, rs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, rs);

    return Formula(carl::FormulaType::AND,{
        rHoldsBasePrev,
        rHoldsNoRingPrev,
        rHoldsNoCapPrev,
        rHoldsNothing,
        rsNeedsAddBasesPrev,
        rsIsFedWithAddBase,
        rsOutputEmptyPrev,
        time,
        reward,
        inheritRemainingState
    });
}

Formula StepFormula::feedAdditionalBaseActionReward(Robot &r, RingStation & rs) {
    return updateReward(getGameData().getReward().getAdditionalBase());
}

Formula StepFormula::mountRingActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& rs : getGameData().getRingStations()) {
            formulas.push_back(mountRingAction(*r, *rs));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::mountRingAction(Robot &r, RingStation & rs) {

    Formula existOrder = existOrderWithRingReq(r, rs);

    Formula holdsBase(carl::FormulaType::NOT, holdsBasePrev(r, Workpiece::NONE));
    Formula notTransparent(carl::FormulaType::NOT, holdsBasePrev(r, Workpiece::TRANSPARENT));
    Formula noCapR = holdsCapPrev(r, Workpiece::NONE);
    Formula noLastRing = holdsRingPrev(r, Workpiece::NONE, Workpiece::getMaxRingNumber());

    Formula outputEmpty = holdsBasePrev(rs, Workpiece::NONE);

    Formula setupForColor(carl::FormulaType::NOT, ringColorSetupPrev(rs, Workpiece::NONE));

    Formula addBasesReqFullfilled = Formula(carl::FormulaType::NOT, needsAddBasesPrev(rs));

    Formula cond(carl::FormulaType::AND,{holdsBase, notTransparent, noCapR, noLastRing,
        outputEmpty, setupForColor, addBasesReqFullfilled});

    Formula holdsNothing = holdsWorkpiece(r, Workpiece());
    Formula notSetupForColor = ringColorSetup(rs, Workpiece::NONE);

    Formula after(carl::FormulaType::AND,{holdsNothing, notSetupForColor});

    Formula mount = mountRingWorkpiece(rs, r);

    Formula time = stationIsNotBlockedFormula(r, rs, rs.getMountRingTime());
    Formula reward = mountRingActionReward(r, rs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, rs);

    return Formula(carl::FormulaType::AND,{
        existOrder,
        cond,
        after,
        mount,
        time,
        reward,
        inheritRemainingState
    });
}

Formula StepFormula::existOrderWithRingReq(Robot &r, RingStation & rs) {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrders()) {
        for (int i = 0; i < Workpiece::getMaxRingNumber(); i++)
            formulas.push_back(orderHasRingReq(r, rs, *o, i));
            formulas.push_back(orderNotDeliveredPrev(*o));
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::orderHasRingReq(Robot &r, RingStation &rs, Order &o, int position) {
    Formula baseF = holdsBasePrev(r, o.getBaseColorReq());

    std::vector<Formula> rings;
    for (int i = 0; i < position; i++)
        rings.push_back(holdsRingPrev(r, o.getRingColorReq(i), i));

    rings.push_back(ringColorSetupPrev(rs, o.getRingColorReq(position)));

    Formula ringsF = Formula(carl::FormulaType::AND, rings);

    return Formula(carl::FormulaType::AND,{baseF, ringsF});
}

Formula StepFormula::mountRingActionReward(Robot &r, RingStation & rs) {
    std::vector<Formula> colors;
    for (auto const& pc : rs.getPossibleRingColors()) {
        for (int i = 0; i < Workpiece::getMaxRingNumber(); i++) {
            Formula holdsNothingPrev = holdsRingPrev(r, Workpiece::NONE, i);
            Formula holdsColor = holdsRing(rs, pc.first, i);
            int preCap = getGameData().getReward().getFinishCXPreCap(i);
            int ccxStep = getGameData().getReward().getFinishCCXStep(pc.second);
            Formula reward = updateReward(preCap + ccxStep);
            colors.push_back(Formula(carl::FormulaType::AND,{holdsNothingPrev, holdsColor, reward}));
        }
    }
    return Formula(carl::FormulaType::OR, colors);
}

Formula StepFormula::collectWorkpieceActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& rs : getGameData().getRingStations()) {
            formulas.push_back(collectWorkpieceAction(*r, *rs));
        }
        for (auto const& cs : getGameData().getCapStations()) {
            formulas.push_back(collectWorkpieceAction(*r, *cs));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::collectWorkpieceAction(Robot &r, Station & s) {
    Formula holdsNothing = holdsBasePrev(r, Workpiece::NONE);
    Formula outputNotEmpty(carl::FormulaType::NOT, holdsBasePrev(s, Workpiece::NONE));

    Formula rHoldsW = swapWorkpiece(r, s);
    Formula csEmpty = holdsWorkpiece(s, Workpiece());

    Formula time = stationIsNotBlockedFormula(r, s, 0);
    Formula reward = collectWorkpieceActionReward(r, s);
    Formula inheritRemainingState = createRemainingStepConstraints(r, s);

    return Formula(carl::FormulaType::AND,{
        holdsNothing,
        outputNotEmpty,
        rHoldsW,
        csEmpty,
        time,
        reward,
        inheritRemainingState

    });
}

Formula StepFormula::collectWorkpieceActionReward(Robot &r, Station &s) {
    return updateReward(0);
}

Formula StepFormula::deliverWorkpieceActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& ds : getGameData().getDeliveryStations()) {
            formulas.push_back(deliverWorkpieceAction(*r, *ds));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::deliverWorkpieceAction(Robot &r, DeliveryStation & ds) {
    Formula hasBase = Formula(carl::FormulaType::NOT, holdsBasePrev(r, Workpiece::NONE));
    Formula hasCap = Formula(carl::FormulaType::NOT, holdsCapPrev(r, Workpiece::NONE));

    Formula holdsNothing = holdsWorkpiece(r, Workpiece());

    Formula time = stationIsNotBlockedFormula(r, ds, 0);
    Formula reward = deliverWorkpieceActionReward(r, ds);
    Formula inheritRemainingState = createRemainingStepConstraints(r, ds);

    return Formula(carl::FormulaType::AND,{
        hasBase,
        hasCap,
        holdsNothing,
        time,
        reward,
        inheritRemainingState
    });
}

Formula StepFormula::deliverWorkpieceActionReward(Robot &r, Station &d) {
    return updateReward(getGameData().getReward().getDelivery());
}
