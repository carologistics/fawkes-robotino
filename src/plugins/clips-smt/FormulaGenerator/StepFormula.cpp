#include "StepFormula.h"
#include "Action.h"
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

Formula StepFormula::createInitialStateOrders() {
    GameData gD = getGameData();
    std::vector<Formula> formulas;
    for (auto const& o : gD.getOrders()) {
        formulas.push_back(equation(getVarOrderMain(*o), Order::NONE));
        formulas.push_back(equation(getVarOrderCap(*o), Order::NONE));
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createInitialStateReward() {
    return Formula(carl::FormulaType::TRUE); //@todo
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

Formula StepFormula::createInitialStateMovingTimes(const Robot& r) {
    GameData gD = getGameData();
    std::vector<Formula> formulas;
    for (auto const& s : gD.getStations()) {
        formulas.push_back(equation(getVarMovingTime(r, *s), r.getMovingTime(*s)));
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createInitialState() {
    return Formula(carl::FormulaType::AND,{
        createInitialStateRobots(),
        createInitialStateStations(),
        createInitialStateOrders()
    });
}

Formula StepFormula::create() {
    if (getStepNumber() == 0)
        return createInitialState();

    std::vector<Formula> actions;
    //actions.push_back(allOrdersDelivered());
    actions.push_back(feedCapActions());
    actions.push_back(collectBaseActions());
    actions.push_back(mountCapActions());
    actions.push_back(dropTransparentBaseActions());
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

Formula StepFormula::orderNotDeliveredPrev(const Order& o) {
    return equation(getPrevStep()->getVarOrderDelivered(o), Order::NOTDELIVERED);
}

Formula StepFormula::orderDeliveredPrev(const Order& o) {
    return equation(getPrevStep()->getVarOrderDelivered(o), Order::DELIVERED);
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
    return equation(getVarBaseReq(rs), rs.getNeededAdditionalBases(c));
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

Formula StepFormula::orderDelivered(const Order& o) {
    return equation(getVarOrderDelivered(o), Order::DELIVERED);
}

Formula StepFormula::orderDeliveredRemains(const Order& o) {
    return equation(getVarOrderDelivered(o), getPrevStep()-> getVarOrderDelivered(o));
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

string StepFormula::getVarNameHoldsBase(const Machine & m, int step) {
    return "B_" + m.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameHoldsRing(const Machine &m, int i, int step) {
    return "R_" + std::to_string(i) + "_" + m.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameHoldsCap(const Machine & m, int step) {
    return "C_" + m.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameMachineOccupied(const Station& m, int step) {
    return "OCC_" + m.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameMovingTime(const Robot &r, const Station & m, int step) {
    return "MOV_" + r.getVarIdentifier() + "_" + m.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameCapColor(const CapStation & cs, int step) {
    return "CCOL_" + cs.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameRingColor(const RingStation & rs, int step) {
    return "RCOL_" + rs.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameBaseReq(const RingStation & rs, int step) {
    return "BREQ_" + rs.getVarIdentifier() + "_" + std::to_string(step);
}

string StepFormula::getVarNameReward(int step) {
    return "R_" + std::to_string(step);
}

string StepFormula::getVarNameOrderDelivered(const Order& o, int step) {
    return "OD_" + std::to_string(o.getId()) + "_" + std::to_string(step);
}

string StepFormula::getVarNameOrderMain(const Order& o, int step) {
    return "O_Main_" + std::to_string(o.getId()) + "_" + std::to_string(step);
}

string StepFormula::getVarNameOrderRing(int position, int base, const Order& o, int step) {
    return "O_Ring" + std::to_string(position) + "_" + std::to_string(base) + "_" + std::to_string(o.getId()) + "_" + std::to_string(step);
}

string StepFormula::getVarNameOrderCap(const Order& o, int step) {
    return "O_Cap_" + std::to_string(o.getId()) + "_" + std::to_string(step);
}

Variable StepFormula::getVarHoldsBase(const Machine &m) {
    return getVariable(StepFormula::getVarNameHoldsBase(m, getStepNumber()));
}

Variable StepFormula::getVarHoldsRing(const Machine &m, int i) {
    return getVariable(StepFormula::getVarNameHoldsRing(m, i, getStepNumber()));
}

Variable StepFormula::getVarHoldsCap(const Machine &m) {
    return getVariable(StepFormula::getVarNameHoldsCap(m, getStepNumber()));
}

Variable StepFormula::getVarMachineOccupied(const Station& m) {
    return getVariable(StepFormula::getVarNameMachineOccupied(m, getStepNumber()));
}

Variable StepFormula::getVarMovingTime(const Robot &r, const Station & m) {
    return getVariable(StepFormula::getVarNameMovingTime(r, m, getStepNumber()));
}

Variable StepFormula::getVarCapColor(const CapStation &cs) {
    return getVariable(StepFormula::getVarNameCapColor(cs, getStepNumber()));
}

Variable StepFormula::getVarRingColor(const RingStation &rs) {
    return getVariable(StepFormula::getVarNameRingColor(rs, getStepNumber()));
}

Variable StepFormula::getVarBaseReq(const RingStation &rs) {
    return getVariable(StepFormula::getVarNameBaseReq(rs, getStepNumber()));
}

Variable StepFormula::getVarReward() {
    return getVariable(StepFormula::getVarNameReward(getStepNumber()));
}

Variable StepFormula::getVarOrderDelivered(const Order& o) {
    return getVariable(StepFormula::getVarNameOrderDelivered(o, getStepNumber()));
}

Variable StepFormula::getVarOrderMain(const Order& o) {
    return getVariable(StepFormula::getVarNameOrderMain(o, getStepNumber()));
}

Variable StepFormula::getVarOrderRing(int position, int base, const Order& o) {
    return getVariable(StepFormula::getVarNameOrderRing(position, base, o, getStepNumber()));
}

Variable StepFormula::getVarOrderCap(const Order& o) {
    return getVariable(StepFormula::getVarNameOrderCap(o, getStepNumber()));
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

Formula StepFormula::createRemainingStepConstraintsRobots() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        formulas.push_back(swapWorkpiece(*r, *r));
        for (auto const& s : getGameData().getStations()) {
            formulas.push_back(equation(getVarMovingTime(*r, *s), getPrevStep()->getVarMovingTime(*r, *s)));
        }
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createRemainingStepConstraintsRobots(const Robot &robot) {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        if (robot != *r) {
            formulas.push_back(swapWorkpiece(*r, *r));
            for (auto const& s : getGameData().getStations()) {
                formulas.push_back(equation(getVarMovingTime(*r, *s), getPrevStep()->getVarMovingTime(*r, *s)));
            }
        }
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createRemainingStepConstraintsStations(const Station &station) {
    std::vector<Formula> formulas;
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

Formula StepFormula::createRemainingStepConstraints(const Station &station) {
    return Formula(carl::FormulaType::AND,{
        createRemainingStepConstraintsRobots(),
        createRemainingStepConstraintsStations(station),
    });
}

Formula StepFormula::createRemainingStepConstraints(const Robot &robot, const Station &station) {
    return Formula(carl::FormulaType::AND,{
        createRemainingStepConstraintsRobots(robot),
        createRemainingStepConstraintsStations(station),
    });
}

Formula StepFormula::createRemainingStepConstraintsOrders() {
    std::vector<Formula> formulas;
    for (auto o : getGameData().getOrders()) {
        formulas.push_back(equation(getVarOrderMain(*o), getPrevStep()->getVarOrderMain(*o)));
        formulas.push_back(equation(getVarOrderCap(*o), getPrevStep()->getVarOrderCap(*o)));
        std::vector<Workpiece::Color> rings = o->getRingColorReq();
        for (int position = 0; position < o->getRingCount(); position++) {
            for (int base = 0; base < getGameData().getNeededAdditionalBases(rings[position]); base++) {
                formulas.push_back(equation(getVarOrderRing(position, base, *o), getPrevStep()->getVarOrderRing(position, base, *o)));
            }
        }
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::createRemainingStepConstraintsOrders(const Order& without) {
    std::vector<Formula> formulas;
    for (auto o : getGameData().getOrders()) {
        if (*o != without) {
            formulas.push_back(equation(getVarOrderMain(*o), getPrevStep()->getVarOrderMain(*o)));
            formulas.push_back(equation(getVarOrderCap(*o), getPrevStep()->getVarOrderCap(*o)));
            std::vector<Workpiece::Color> rings = o->getRingColorReq();
            for (int position = 0; position < o->getRingCount(); position++) {
                for (int base = 0; base < getGameData().getNeededAdditionalBases(rings[position]); base++) {
                    formulas.push_back(equation(getVarOrderRing(position, base, *o), getPrevStep()->getVarOrderRing(position, base, *o)));
                }
            }
        }
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::orderNeedsAction(const Order& o, Order::State state) {
    Formula prevProdStep = Formula(carl::FormulaType::FALSE);
    Formula prodStep = Formula(carl::FormulaType::FALSE);
    Formula remCons = Formula(carl::FormulaType::FALSE);
    Formula remConsOrderMain = Formula(carl::FormulaType::FALSE);
    Formula remConsOrderRing = Formula(carl::FormulaType::FALSE);
    Formula remConsOrderCap = Formula(carl::FormulaType::FALSE);
    Formula fedCap = Formula(carl::FormulaType::TRUE);
    Formula addBase = Formula(carl::FormulaType::TRUE);

    remCons = createRemainingStepConstraintsOrders(o);

    remConsOrderMain = equation(getVarOrderMain(o), getPrevStep()->getVarOrderMain(o));
    std::vector<Workpiece::Color> rings = o.getRingColorReq();
    std::vector<Formula> remConsOrderRingList;
    for (int position = 0; position < o.getRingCount(); position++) {

        for (int base = 0; base < getGameData().getNeededAdditionalBases(rings[position]); base++) {
            remConsOrderRingList.push_back(equation(getVarOrderRing(position, base, o), getPrevStep()->getVarOrderRing(position, base, o)));
        }
    }
    remConsOrderRing = Formula(carl::FormulaType::AND, remConsOrderRingList);

    remConsOrderCap = equation(getVarOrderCap(o), getPrevStep()->getVarOrderCap(o));

    if (getGameData().isMainProd(o, state)) {
        prevProdStep = equation(getPrevStep()->getVarOrderMain(o), getGameData().getPrevProductionStepMain(o, state));
        prodStep = equation(getVarOrderMain(o), state);

        if (state == Order::CAP) {
            fedCap = equation(getPrevStep()->getVarOrderCap(o), Order::FEDCAP);
        }

        for (int position = 0; position < o.getRingCount(); position++) {
            if (state == o.getRingId(position)) {
                std::vector<Formula> addBaseList;
                for (int base = 0; base < getGameData().getNeededAdditionalBases(rings[position]); base++) {
                    addBaseList.push_back(equation(getPrevStep()->getVarOrderRing(position, base, o), o.getFedBaseId(position, base)));
                }
                addBase = Formula(carl::FormulaType::AND, addBaseList);
            }
        }

        remConsOrderMain = Formula(carl::FormulaType::TRUE);

    } else if (getGameData().isCapProd(state)) {
        prevProdStep = equation(getPrevStep()->getVarOrderCap(o), getGameData().getPrevProductionStepCap(o, state));
        prodStep = equation(getVarOrderCap(o), state);
        remConsOrderCap = Formula(carl::FormulaType::TRUE);

    } else if (getGameData().isRingProd(o, state)) {

        remConsOrderRingList.clear();
        for (int position = 0; position < o.getRingCount(); position++) {
            for (int base = 0; base < getGameData().getNeededAdditionalBases(rings[position]); base++) {
                if (state == o.getCollectBaseId(position, base)) {
                    prodStep = equation(getVarOrderRing(position, base, o), state);
                    prevProdStep = equation(getPrevStep()->getVarOrderRing(position, base, o), getGameData().getPrevProductionStepRing(o, state));

                } else if (state == o.getFedBaseId(position, base)) {
                    prodStep = equation(getVarOrderRing(position, base, o), state);
                    Formula prevProdStepRing = equation(getPrevStep()->getVarOrderRing(position, base, o), getGameData().getPrevProductionStepRing(o, state));
                    Formula prevProdStepMain = equation(getPrevStep()->getVarOrderMain(o), o.getSetupRingId(position));
                    prevProdStep = Formula(carl::FormulaType::AND,{prevProdStepRing, prevProdStepMain});
                } else {
                    remConsOrderRingList.push_back(equation(getVarOrderRing(position, base, o), getPrevStep()->getVarOrderRing(position, base, o)));
                }
            }
        }
        remConsOrderRing = Formula(carl::FormulaType::AND, remConsOrderRingList);
    }

    return Formula(carl::FormulaType::AND,{prevProdStep, prodStep, remConsOrderMain, remConsOrderRing, remConsOrderCap, fedCap, addBase, remCons});
}

Formula StepFormula::ordersNeedsAction(const std::vector<Order> orders, Order::State state) {
    std::vector<Formula> formulas;
    for (auto o : orders) {
        formulas.push_back(orderNeedsAction(o, state));
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::updateTimes(const Robot& r, const Station& station, int processingTimeStation, int processingTimeRobot) {
    Rational blocked = Rational(processingTimeStation + processingTimeRobot);
    Formula timeBlockedLEQprev = lessEqual(getPrevStep()->getVarMachineOccupied(station), getPrevStep()->getVarMovingTime(r, station));
    Formula timeBlockedLEQnext = equation(getVarMachineOccupied(station), getPrevStep()->getVarMovingTime(r, station) + blocked);
    Formula timeBlockedLEQ(carl::FormulaType::AND,{timeBlockedLEQprev, timeBlockedLEQnext});

    Formula timeBlockedGprev = greater(getPrevStep()->getVarMachineOccupied(station), getPrevStep()->getVarMovingTime(r, station));
    Formula timeBlockedGnext = equation(getVarMachineOccupied(station), getPrevStep()->getVarMachineOccupied(station) + blocked);
    Formula timeBlockedG(carl::FormulaType::AND,{timeBlockedGprev, timeBlockedGnext});

    std::vector<Formula> movingTimesLEQVector;
    std::vector<Formula> movingTimesGVector;

    for (auto const& s : getGameData().getStations()) {
        Rational mov = Rational(station.getMovingTime(*s) + processingTimeRobot);
        movingTimesLEQVector.push_back(equation(getVarMovingTime(r, *s), getPrevStep()->getVarMovingTime(r, station) + mov));
        movingTimesGVector.push_back(equation(getVarMovingTime(r, *s), getPrevStep()->getVarMachineOccupied(station) + mov));
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
                if (getGameData().existsOrderWithBaseReq(c)) {
                    Formula collectBase(carl::FormulaType::AND,{
                        collectBaseAction(*r, c, *bs),
                        existOrderWithBaseColorReq(*r, c)
                    });
                    formulas.push_back(collectBase);
                }
            }
            Formula forRingStation(carl::FormulaType::AND,{
                needsBaseForRingStation(),
                collectBaseAction(*r, bs->getColorForRingStation(), *bs),
                existOrderWithAddBaseReqBaseStation()
            });
            formulas.push_back(forRingStation);

        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::collectBaseAction(Robot &r, Workpiece::Color c, BaseStation & bs) {
    Formula holdsNothingPrev = holdsBasePrev(r, Workpiece::NONE);
    Formula holdsBase = holdsWorkpiece(r, Workpiece(c));
    Formula time = updateTimes(r, bs, bs.getDispenseBaseTime(), r.getTakeWorkpieceTime());

    Formula reward = collectBaseActionReward(r, c, bs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, bs);
    return Formula(carl::FormulaType::AND,{
        holdsNothingPrev,
        holdsBase,
        time,
        reward,
        inheritRemainingState
    });
}

Formula StepFormula::existOrderWithBaseColorReq(Robot &r, Workpiece::Color c) {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithBaseReq(c)) {
        formulas.push_back(orderNeedsAction(o, Order::State::BASE));
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::collectBaseActionReward(Robot &r, Workpiece::Color c, BaseStation & bs) {
    return updateReward(0);
}

Formula StepFormula::existOrderWithAddBaseReqBaseStation() {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithAddBaseReq()) {
        for (int position = 0; position < o.getRingCount(); position++) {
            for (int base = 0; base < getGameData().getNeededAdditionalBases(o.getRingColorReq(position)); base++) {
                formulas.push_back(orderNeedsAction(o, o.getCollectBaseId(position, base)));
            }
        }
    }

    if (getGameData().getOrdersWithAddBaseReq().size() == 0) {
        return Formula(carl::FormulaType::FALSE);
    }
    return Formula(carl::FormulaType::OR, formulas);
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
    Formula existOrder = existOrderWithCapColorReq(c);

    Formula rHoldsNothingPrev = holdsBasePrev(r, Workpiece::NONE);
    Formula rHoldsNothing = holdsWorkpiece(r, Workpiece());

    Formula capNotLoadedPrev = loadCapPrev(cs, Workpiece::NONE);
    Formula capLoaded = loadCap(cs, c);

    Formula outputNotLoadedPrev = holdsBasePrev(cs, Workpiece::NONE);
    Formula outputLoaded = holdsWorkpiece(cs, Workpiece(Workpiece::TRANSPARENT));

    Formula time = updateTimes(r, cs, cs.getFeedCapTime(), r.getTakeWorkpieceTime() + r.getFeedWorkpieceTime());
    Formula reward = feedCapActionReward(r, c, cs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, cs);

    return Formula(carl::FormulaType::AND,{
        existOrder,
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

Formula StepFormula::existOrderWithCapColorReq(Workpiece::Color c) {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithCapReq(c))
        formulas.push_back(orderNeedsAction(o, Order::State::FEDCAP));
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::feedCapActionReward(Robot &r, Workpiece::Color c, CapStation & cs) {
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

    Formula time = updateTimes(r, cs, cs.getMountCapTime(), r.getFeedWorkpieceTime());
    Formula reward = mountCapActionReward(r, cs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, cs);
    //Formula inheritOrders = createRemainingStepConstraintsOrders();

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
        inheritRemainingState,
        //inheritOrders
    });
}

Formula StepFormula::existOrderWithCapReq(Robot &r, CapStation & cs) {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrders()) {
        Formula orderCap = orderHasCapReq(r, cs, *o);
        formulas.push_back(orderCap);
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::orderHasCapReq(Robot &r, CapStation &cs, Order & o) {
    vector<Formula> formulas;
    Formula orderAction = orderNeedsAction(o, Order::CAP);
    Formula capF = loadCapPrev(cs, o.getCapColorReq());
    Formula baseF = holdsBasePrev(r, o.getBaseColorReq());

    formulas.push_back(orderAction);
    formulas.push_back(capF);
    formulas.push_back(baseF);

    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++) {
        formulas.push_back(holdsRingPrev(r, o.getRingColorReq(i), i));
    }

    return Formula(carl::FormulaType::AND, formulas);
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

Formula StepFormula::timesDoNotChange() {
    vector<Formula> formulas;
    for (auto r : getGameData().getRobots()) {
        for (auto s : getGameData().getStations()) {
            formulas.push_back(equation(getVarMovingTime(*r, *s), getPrevStep()->getVarMovingTime(*r, *s)));
        }
    }
    for (auto s : getGameData().getStations()) {
        formulas.push_back(equation(getVarMachineOccupied(*s), getPrevStep()->getVarMachineOccupied(*s)));
    }
    return Formula(carl::FormulaType::AND, formulas);
}

Formula StepFormula::setupRingColorAction(Workpiece::Color c, RingStation & rs) {

    Formula existsOrder = existOrderWithRingColorReq(c);

    Formula emptyOutputPrev = holdsBasePrev(rs, Workpiece::NONE);
    Formula notSetUp = ringColorSetupPrev(rs, Workpiece::NONE);

    Formula emptyOutput = holdsWorkpiece(rs, Workpiece());
    Formula setUp = ringColorSetup(rs, c);

    Formula expectsAddBases = needsAddBases(rs, c);

    Formula time = timesDoNotChange();

    Formula reward = setupRingColorActionReward(c, rs);
    Formula inheritRemainingState = createRemainingStepConstraints(rs);
    //Formula inheritOrders = createRemainingStepConstraintsOrders();

    return Formula(carl::FormulaType::AND,{
        existsOrder,
        emptyOutputPrev,
        notSetUp,
        emptyOutput,
        setUp,
        expectsAddBases,
        time,
        reward,
        inheritRemainingState,
        //inheritOrders
    });
}

Formula StepFormula::existOrderWithRingColorReq(Workpiece::Color c) {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithRingReq(c)) {
        for (int i = 0; i < o.getRingCount(); i++) {
            formulas.push_back(orderNeedsAction(o, o.getSetupRingId(i)));
        }
    }
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

    Formula rsNeedsAddBasesPrev = needsAddBasesPrev(rs);
    Formula rsOutputEmptyPrev = holdsBasePrev(rs, Workpiece::NONE);

    Formula rHoldsNothing = holdsWorkpiece(r, Workpiece());
    Formula rsHoldsNothing = holdsWorkpiece(rs, Workpiece());
    Formula rsIsFedWithAddBase = feedAddBase(rs);
    Formula setUp = equation(getVarRingColor(rs), getPrevStep()->getVarRingColor(rs));


    Formula time = updateTimes(r, rs, rs.getFeedBaseTime(), r.getFeedWorkpieceTime());
    Formula reward = feedAdditionalBaseActionReward(r, rs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, rs);
    Formula existOrder = existOrderWithAddBaseReqRingStation();

    return Formula(carl::FormulaType::AND,{
        rHoldsBasePrev,
        rHoldsNoRingPrev,
        rHoldsNoCapPrev,
        rsNeedsAddBasesPrev,
        rsOutputEmptyPrev,
        rHoldsNothing,
        rsHoldsNothing,
        rsIsFedWithAddBase,
        setUp,
        time,
        reward,
        inheritRemainingState,
        existOrder
    });
}

Formula StepFormula::existOrderWithAddBaseReqRingStation() {
    std::vector<Formula> formulas;
    for (auto const& o : getGameData().getOrdersWithAddBaseReq()) {
        for (int position = 0; position < o.getRingCount(); position++) {
            for (int base = 0; base < getGameData().getNeededAdditionalBases(o.getRingColorReq(position)); base++) {
                formulas.push_back(orderNeedsAction(o, o.getFedBaseId(position, base)));
            }
        }
    }

    if (getGameData().getOrdersWithAddBaseReq().size() == 0) {
        return Formula(carl::FormulaType::FALSE);
    }
    return Formula(carl::FormulaType::OR, formulas);
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
    Formula addBasesReqZero = needsAddBases(rs, Workpiece::NONE);

    Formula after(carl::FormulaType::AND,{holdsNothing, notSetupForColor, addBasesReqZero});

    Formula mount = mountRingWorkpiece(rs, r);

    Formula time = updateTimes(r, rs, rs.getMountRingTime(), r.getFeedWorkpieceTime());
    //Formula reward = mountRingActionReward(r, rs);
    Formula inheritRemainingState = createRemainingStepConstraints(r, rs);
    //Formula inheritOrders = createRemainingStepConstraintsOrders();

    return Formula(carl::FormulaType::AND,{
        existOrder,
        cond,
        after,
        mount,
        time,
        //reward,
        inheritRemainingState,
        //inheritOrders
    });
}

Formula StepFormula::existOrderWithRingReq(Robot &r, RingStation & rs) {
    vector<Formula> formulas;
    for (auto const& o : getGameData().getOrders()) {
        for (int i = 0; i < o->getRingCount(); i++) {

            formulas.push_back(orderHasRingReq(r, rs, *o, i));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::orderHasRingReq(Robot &r, RingStation &rs, Order &o, int position) {
    vector<Formula> formulas;

    Formula oNeedsAction = orderNeedsAction(o, o.getRingId(position));
    Formula reqRingMount = ringColorSetupPrev(rs, o.getRingColorReq(position));
    Formula reqRingMountFree = holdsRingPrev(r, Workpiece::NONE, position);

    Formula reqBaseExists = holdsBasePrev(r, o.getBaseColorReq());

    formulas.push_back(oNeedsAction);
    formulas.push_back(reqRingMount);
    formulas.push_back(reqRingMountFree);
    formulas.push_back(reqBaseExists);

    for (int i = 0; i < position; i++) {
        formulas.push_back(holdsRingPrev(r, o.getRingColorReq(i), i));
    }

    return Formula(carl::FormulaType::AND, formulas);
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

Formula StepFormula::dropTransparentBaseActions() {
    std::vector<Formula> formulas;
    for (auto const& r : getGameData().getRobots()) {
        for (auto const& cs : getGameData().getCapStations()) {

            formulas.push_back(dropTransparentBaseAction(*r, *cs));
        }
    }
    return Formula(carl::FormulaType::OR, formulas);
}

Formula StepFormula::dropTransparentBaseAction(Robot &r, CapStation & cs) {
    Formula outputTrans = holdsBasePrev(cs, Workpiece::TRANSPARENT);
    Formula holdsNothing = holdsBasePrev(r, Workpiece::NONE);

    Formula csEmpty = holdsWorkpiece(r, Workpiece());
    Formula rEmpty = holdsWorkpiece(cs, Workpiece());
    Formula capColor = equation(getVarCapColor(cs), getPrevStep()->getVarCapColor(cs));

    Formula time = updateTimes(r, cs, 0, r.getTakeWorkpieceTime());
    Formula inheritRemainingState = createRemainingStepConstraints(r, cs);
    Formula inheritOrders = createRemainingStepConstraintsOrders();

    return Formula(carl::FormulaType::AND,{
        holdsNothing,
        outputTrans,
        csEmpty,
        rEmpty,
        capColor,
        time,
        inheritRemainingState,
        inheritOrders
    });
}

Formula StepFormula::collectWorkpieceActions() {
    //@todo transparente nur aufheben wenn für rs notwendig
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

Formula StepFormula::collectWorkpieceAction(Robot &r, CapStation & cs) {

    Formula capColor = equation(getVarCapColor(cs), getPrevStep()->getVarCapColor(cs));
    Formula collectWorkpiece = collectWorkpieceGeneralAction(r, cs);

    Formula noTransparentBase = Formula(carl::FormulaType::NOT, holdsBasePrev(cs, Workpiece::TRANSPARENT));
    Formula inheritOrders = createRemainingStepConstraintsOrders();
    Formula noTransBaseInheritOrders = Formula(carl::FormulaType::AND,{noTransparentBase, inheritOrders});

    Formula transparentBaseOrder = collectTransparentBaseOrder(r, cs);

    Formula order = Formula(carl::FormulaType::OR,{transparentBaseOrder, noTransBaseInheritOrders});

    return Formula(carl::FormulaType::AND,{collectWorkpiece, capColor, order});
}

Formula StepFormula::collectTransparentBaseOrder(Robot &r, CapStation & cs) {
    Formula transparentBase = holdsBasePrev(cs, Workpiece::TRANSPARENT);

    std::vector<Formula> formulas;
    for (auto o : getGameData().getOrdersWithAddBaseReq()) {
        for (int i = 0; i < o.getRingCount(); i++) {
            formulas.push_back(orderNeedsAction(o, o.getCollectBaseId(i, 0)));
            formulas.push_back(orderNeedsAction(o, o.getCollectBaseId(i, 1)));
        }
    }
    Formula ordersNeedAction(carl::FormulaType::OR, formulas);

    return Formula(carl::FormulaType::AND,{transparentBase, ordersNeedAction});
}

Formula StepFormula::collectWorkpieceAction(Robot &r, RingStation & rs) {
    Formula ringColor = equation(getVarRingColor(rs), getPrevStep()->getVarRingColor(rs));
    Formula baseReq = equation(getVarBaseReq(rs), getPrevStep()->getVarBaseReq(rs));
    Formula ringColorBaseReq(carl::FormulaType::AND,{ringColor, baseReq});
    Formula collectWorkpiece = collectWorkpieceGeneralAction(r, rs);
    Formula inheritOrders = createRemainingStepConstraintsOrders();

    return Formula(carl::FormulaType::AND,{collectWorkpiece, ringColorBaseReq, inheritOrders});
}

Formula StepFormula::collectWorkpieceGeneralAction(Robot &r, Station & s) {
    Formula holdsNothing = holdsBasePrev(r, Workpiece::NONE);
    Formula outputNotEmpty(carl::FormulaType::NOT, holdsBasePrev(s, Workpiece::NONE));

    Formula rHoldsW = swapWorkpiece(r, s);
    Formula csEmpty = holdsWorkpiece(s, Workpiece());

    Formula time = updateTimes(r, s, 0, r.getTakeWorkpieceTime());

    Formula reward = collectWorkpieceActionReward(r, s);
    Formula inheritRemainingState = createRemainingStepConstraints(r, s);

    return Formula(carl::FormulaType::AND,{
        holdsNothing,
        outputNotEmpty,
        rHoldsW,
        csEmpty,
        time,
        reward,
        inheritRemainingState,
    });
}

Formula StepFormula::collectWorkpieceActionReward(Robot &r, Station & s) {

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

    Formula time = updateTimes(r, ds, ds.getDeliverProductTime(), r.getFeedWorkpieceTime());
    Formula reward = deliverWorkpieceActionReward(r, ds);
    Formula inheritRemainingState = createRemainingStepConstraints(r, ds);
    Formula oDelivered = orderDelivered(r);
    //@todo orderDelivered
    //@todo finde zu Product Order die diese Produkt bestellt und die kleinste Deadline hat

    return Formula(carl::FormulaType::AND,{
        hasBase,
        hasCap,
        holdsNothing,
        time,
        reward,
        oDelivered,
        inheritRemainingState
    });
}

Formula StepFormula::deliverWorkpieceActionReward(Robot &r, Station & d) {

    return updateReward(getGameData().getReward().getDelivery());
}

Formula StepFormula::orderDelivered(Robot & r) {
    std::vector<Formula> formulas;
    for (auto o : getGameData().getOrders()) {

        Formula isOrder(carl::FormulaType::AND,{
            holdsWorkpiecePrev(r, o->getProduct()),
            orderNeedsAction(*o, Order::DELIVERED),
        });
        formulas.push_back(isOrder);
    }
    return Formula(carl::FormulaType::OR, formulas);
}

//@todo aim to maximize the sum of order delivered vars besides maximizing the rewards only
//@todo diff in solving speed if the remainig worldstate is propagated further?

Formula StepFormula::allOrdersDelivered() {
    std::vector<Formula> formulas;
    for (auto o : getGameData().getOrders()) {
        formulas.push_back(orderDeliveredPrev(*o));
    }

    formulas.push_back(createRemainingStepConstraintsOrders());
    return Formula(carl::FormulaType::AND, formulas);
}