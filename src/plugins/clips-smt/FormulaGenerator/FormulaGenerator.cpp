#include <iostream>
#include "FormulaGenerator.h"

FormulaGenerator::FormulaGenerator(int amount, GameData& gameData) {
    generateSteps(amount, gameData);
    setGameData(gameData);
}

FormulaGenerator::~FormulaGenerator() {
}

stepFormula_ptr FormulaGenerator::getStep(int i) {
    return this->steps[i];
}

std::vector<stepFormula_ptr> FormulaGenerator::getSteps() {
    return this->steps;
}

void FormulaGenerator::generateSteps(int amount, GameData& gameData) {
    this->steps = StepFormula::generateSteps(amount, gameData);
}

Formula FormulaGenerator::createFormula() {

    std::vector<Formula> formulas;
    for (auto const& s : getSteps()) {
        formulas.push_back(s->create());
    }
    formula = Formula(carl::FormulaType::AND, formulas);
    return formula;
}

void FormulaGenerator::setGameData(const GameData &gameData) {
    this->gameData = gameData;
}

GameData FormulaGenerator::getGameData() {
    return this->gameData;
}

std::map<string, int> FormulaGenerator::transferZ3ModelToMap(const z3::model& model) {
    std::map<string, int> varnameToValue;
    for (uint i = 0; i < model.size(); i++) {
        z3::func_decl function = model[i];
        int value = model.get_const_interp(function).get_numeral_int();
        string varname = function.name().str();
        varnameToValue[varname] = value;
    }
    return varnameToValue;
}

std::string FormulaGenerator::printWorldStates(const z3::model& model) {
    std::string output;
    for (uint i = 0; i < getSteps().size(); i++) {
        output += printWorldState(model, i);
    }
    return output;
}

std::string FormulaGenerator::printWorldStateWorkpiece(std::map<string, int> model, int step, machine_ptr m) {
    string output;
    if (!m->isCapStation() && !m->isRingStation() && !m->isRobot()) {
        return output;
    }

    int intColorBase = model.at(StepFormula::getVarNameHoldsBase(*m, step));
    Workpiece::Color colorBase = Workpiece::intToColor(intColorBase);

    int intColorCap = model.at(StepFormula::getVarNameHoldsCap(*m, step));
    Workpiece::Color colorCap = Workpiece::intToColor(intColorCap);

    std::vector<int> intColorRings;
    std::vector<Workpiece::Color> colorRings;
    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++) {
        int intColorRing = model.at(StepFormula::getVarNameHoldsRing(*m, i, step));
        Workpiece::Color colorRing = Workpiece::intToColor(intColorRing);
        intColorRings.push_back(intColorRing);
        colorRings.push_back(colorRing);
    }

    Workpiece wp(Workpiece::intToColor(colorBase), colorRings, colorCap);
    output += wp.toString();

    if (colorBase == Workpiece::NOTDEFINED) {
        output += " Base: " + std::to_string(intColorBase);
    }

    for (int i = 0; i < Workpiece::getMaxRingNumber(); i++) {
        if (colorRings[i] == Workpiece::NOTDEFINED) {
            output += " Ring" + std::to_string(i) + ": " + std::to_string(intColorRings[i]);
        }
    }

    if (colorCap == Workpiece::NOTDEFINED) {
        output += " Cap: " + std::to_string(intColorCap);
    }
    return output;
}

std::string FormulaGenerator::printWorldState(const z3::model& z3model, int step) {
    string output = "Step " + std::to_string(step) + ":\n";
    std::map<string, int> model = transferZ3ModelToMap(z3model);

    for (auto r : getGameData().getRobots()) {
        output += r->getVarIdentifier() + ": ";
        output += printWorldStateWorkpiece(model, step, r);

        for (auto s : getGameData().getStations()) {
            output += "; (" + s->getVarIdentifier() + ", " + std::to_string(model.at(StepFormula::getVarNameMovingTime(*r, *s, step))) + ")";
        }
        output += "\n";
    }

    for (auto bs : getGameData().getBaseStations()) {
        output += bs->getVarIdentifier() + ": ";
        output += std::to_string(model.at(StepFormula::getVarNameMachineOccupied(*bs, step)));
        output += "\n";
    }

    for (auto rs : getGameData().getRingStations()) {
        output += rs->getVarIdentifier() + ": ";
        output += printWorldStateWorkpiece(model, step, rs);
        output += "; ";

        int intRingColor = model.at(StepFormula::getVarNameRingColor(*rs, step));
        Workpiece::Color ringColor = Workpiece::intToColor(intRingColor);

        output += Workpiece::toString(ringColor);
        if (ringColor == Workpiece::NOTDEFINED) {
            output += "; ND: " + std::to_string(intRingColor);
        }

        int intBaseReq = model.at(StepFormula::getVarNameBaseReq(*rs, step));
        output += "; " + std::to_string(intBaseReq);

        output += "; ";
        output += std::to_string(model.at(StepFormula::getVarNameMachineOccupied(*rs, step)));
        output += "\n";
    }

    for (auto cs : getGameData().getCapStations()) {
        output += cs->getVarIdentifier() + ": ";
        output += printWorldStateWorkpiece(model, step, cs);
        output += "; ";
        int intcapColor = model.at(StepFormula::getVarNameCapColor(*cs, step));
        Workpiece::Color capColor = Workpiece::intToColor(intcapColor);
        output += Workpiece::toString(capColor);
        if (capColor == Workpiece::NOTDEFINED) {
            output += "; ND: " + std::to_string(intcapColor);
        }
        output += "; ";
        output += std::to_string(model.at(StepFormula::getVarNameMachineOccupied(*cs, step)));
        output += "\n";
    }

    for (auto ds : getGameData().getDeliveryStations()) {
        output += ds->getVarIdentifier() + ": ";
        output += std::to_string(model.at(StepFormula::getVarNameMachineOccupied(*ds, step)));
        output += "\n";
    }

    for (auto o : getGameData().getOrders()) {
        output += "O" + std::to_string(o->getId()) + ": ";
        int intOrder = model.at(StepFormula::getVarNameOrderDelivered(*o, step));
        if (intOrder == 0) {
            output += "NOTDELIVERED";
        } else if (intOrder == 1) {
            output += "DELIVERED";
        } else {
            output += "NOTDEFINED";
        }
        output += "\n";
    }
    output += "\n";
    return output;
}

set<station_ptr> FormulaGenerator::getStationOccAltered(const std::map<string, int>& model, int step) {
    set<station_ptr> stations;
    int prev = 0;
    int now = 0;
    for (auto s : getGameData().getStations()) {
        try {
            prev = model.at(StepFormula::getVarNameMachineOccupied(*s, step - 1));
            now = model.at(StepFormula::getVarNameMachineOccupied(*s, step));
        } catch (std::out_of_range e) {
            std::cerr << "getStationOccAltered: "
                    << StepFormula::getVarNameMachineOccupied(*s, step - 1) << " or "
                    << StepFormula::getVarNameMachineOccupied(*s, step)
                    << " does not denote a Var Name" << std::endl;

        }
        if (prev != now)
            stations.insert(s);
    }
    return stations;
}

set<robot_ptr> FormulaGenerator::getRobotMovAltered(const std::map<string, int>& model, int step) {
    set<robot_ptr> robots;
    int prev = 0;
    int now = 0;
    for (auto r : getGameData().getRobots()) {
        for (auto s : getGameData().getStations()) {
            try {
                prev = model.at(StepFormula::getVarNameMovingTime(*r, *s, step - 1));
                now = model.at(StepFormula::getVarNameMovingTime(*r, *s, step));
            } catch (std::out_of_range e) {
                std::cerr << "getRobotMovAltered: "
                        << StepFormula::getVarNameMovingTime(*r, *s, step - 1) << " or "
                        << StepFormula::getVarNameMovingTime(*r, *s, step)
                        << " does not denote a Var Name" << std::endl;
            }
        }
        if (prev != now)
            robots.insert(r);
    }
    return robots;
}

set<robot_ptr> FormulaGenerator::getRobotWorkpieceAltered(const std::map<string, int>& model, int step) {
    set<robot_ptr> robots;
    int prev = 0;
    int now = 0;
    for (auto r : getGameData().getRobots()) {
        try {
            prev = model.at(StepFormula::getVarNameHoldsBase(*r, step - 1));
            now = model.at(StepFormula::getVarNameHoldsBase(*r, step));
        } catch (std::out_of_range e) {
            std::cerr << "getRobotWorkpieceAltered: "
                    << StepFormula::getVarNameHoldsBase(*r, step - 1) << " or "
                    << StepFormula::getVarNameHoldsBase(*r, step)
                    << " does not denote a Var Name" << std::endl;
        }
        if (prev != now)
            robots.insert(r);
    }
    return robots;
}

set<capStation_ptr> FormulaGenerator::getCapStationWorkpieceAltered(const std::map<string, int>& model, int step) {
    set<capStation_ptr>capStations;
    int prev = 0;
    int now = 0;
    for (auto cs : getGameData().getCapStations()) {
        try {
            prev = model.at(StepFormula::getVarNameHoldsBase(*cs, step - 1));
            now = model.at(StepFormula::getVarNameHoldsBase(*cs, step));
        } catch (std::out_of_range e) {
            std::cerr << "getCapStationWorkpieceAltered: "
                    << StepFormula::getVarNameHoldsBase(*cs, step - 1) << " or "
                    << StepFormula::getVarNameHoldsBase(*cs, step)
                    << " does not denote a Var Name" << std::endl;
        }
        if (prev != now)
            capStations.insert(cs);
    }
    return capStations;
}

set<ringStation_ptr> FormulaGenerator::getRingStationWorkpieceAltered(const std::map<string, int>& model, int step) {
    set<ringStation_ptr>ringStations;
    int prev = 0;
    int now = 0;
    for (auto rs : getGameData().getRingStations()) {
        try {
            prev = model.at(StepFormula::getVarNameHoldsBase(*rs, step - 1));
            now = model.at(StepFormula::getVarNameHoldsBase(*rs, step));
        } catch (std::out_of_range e) {
            std::cerr << "getCapStationWorkpieceAltered: "
                    << StepFormula::getVarNameHoldsBase(*rs, step - 1) << " or "
                    << StepFormula::getVarNameHoldsBase(*rs, step)
                    << " does not denote a Var Name" << std::endl;
        }
        if (prev != now)
            ringStations.insert(rs);
    }
    return ringStations;
}

set<capStation_ptr> FormulaGenerator::getCapStationCapColorAltered(const std::map<string, int>& model, int step) {
    set<capStation_ptr> capStations;
    int prev = 0;
    int now = 0;
    for (auto cs : getGameData().getCapStations()) {
        try {
            prev = model.at(StepFormula::getVarNameCapColor(*cs, step - 1));
            now = model.at(StepFormula::getVarNameCapColor(*cs, step));
        } catch (std::out_of_range e) {
            std::cerr << "getCapStationCapColorAltered: "
                    << StepFormula::getVarNameCapColor(*cs, step - 1) << " or "
                    << StepFormula::getVarNameCapColor(*cs, step)
                    << " does not denote a Var Name" << std::endl;
        }
        if (prev != now)
            capStations.insert(cs);
    }
    return capStations;
}

set<ringStation_ptr> FormulaGenerator::getRingStationRingColorAltered(const std::map<string, int>& model, int step) {
    set<ringStation_ptr> ringStations;
    int prev = 0;
    int now = 0;
    for (auto rs : getGameData().getRingStations()) {
        try {
            prev = model.at(StepFormula::getVarNameRingColor(*rs, step - 1));
            now = model.at(StepFormula::getVarNameRingColor(*rs, step));
        } catch (std::out_of_range e) {
            std::cerr << "getCapStationCapColorAltered: "
                    << StepFormula::getVarNameRingColor(*rs, step - 1) << " or "
                    << StepFormula::getVarNameRingColor(*rs, step)
                    << " does not denote a Var Name" << std::endl;
        }
        if (prev != now)
            ringStations.insert(rs);
    }
    return ringStations;
}

set<ringStation_ptr> FormulaGenerator::getRingStationAddBasesReqAltered(const std::map<string, int>& model, int step) {
    set<ringStation_ptr> ringStations;
    int prev = 0;
    int now = 0;
    for (auto rs : getGameData().getRingStations()) {
        try {
            prev = model.at(StepFormula::getVarNameBaseReq(*rs, step - 1));
            now = model.at(StepFormula::getVarNameBaseReq(*rs, step));
        } catch (std::out_of_range e) {
            std::cerr << "getCapStationCapColorAltered: "
                    << StepFormula::getVarNameBaseReq(*rs, step - 1) << " or "
                    << StepFormula::getVarNameBaseReq(*rs, step)
                    << " does not denote a Var Name" << std::endl;
        }
        if (prev != now)
            ringStations.insert(rs);
    }
    return ringStations;
}

bool FormulaGenerator::checkConsistency(const std::map<string, int>& model, int step) {
    bool result = true;
    set<station_ptr> stationOcc = getStationOccAltered(model, step);
    set<robot_ptr> robotMov = getRobotMovAltered(model, step);
    set<robot_ptr> robotWorkpiece = getRobotWorkpieceAltered(model, step);
    set<capStation_ptr> capStationWorkpiece = getCapStationWorkpieceAltered(model, step);
    set<capStation_ptr> capStationCapColor = getCapStationCapColorAltered(model, step);

    if (stationOcc.size() > 1) {
        result = false;
        std::cerr << "More than one StationOccupied altered in step " << std::to_string(step) << endl;
    }
    if (robotMov.size() > 1) {
        result = false;
        std::cerr << "More than one robot MovingTime altered in step " << std::to_string(step) << endl;
    }
    if (robotWorkpiece.size() > 1) {
        result = false;
        std::cerr << "More than one robot workpiece altered in step " << std::to_string(step) << endl;
    }
    if (capStationWorkpiece.size() > 1) {
        result = false;
        std::cerr << "More than one station workpiece altered in step " << std::to_string(step) << endl;
    }
    if (capStationCapColor.size() > 1) {
        result = false;
        std::cerr << "More than one capStation capColor altered in step " << std::to_string(step) << endl;
    }

    return result;
}

std::string FormulaGenerator::printActions(const z3::model& model) {
    std::string output;
    std::vector<Action> actions = getActions(model);
    for (auto a : actions) {
        output += a.toString() + "\n";
    }
    return output;
}

std::vector<Action> FormulaGenerator::getActions(const z3::model& model) {
    std::vector<Action> actions;
    for (uint i = 1; i < getSteps().size(); i++) {
        actions.push_back(getAction(model, i));
    }
    return actions;
}

Action FormulaGenerator::getAction(const z3::model& z3model, int step) {
    Action action(Action::ActionType::NOTDEFINED);
    std::map<string, int> model = transferZ3ModelToMap(z3model);

    //Check if all orders delivered and thus none action executed
    action = checkNoneAction(model, step);
    if (action.isNone()) {
        //after a step without action, the world state is no longe well defined
        //therefore we have to return here
        return action;
    }

    set<station_ptr> stationOcc = getStationOccAltered(model, step);
    set<robot_ptr> robotMov = getRobotMovAltered(model, step);
    set<capStation_ptr> capStationCapColor = getCapStationCapColorAltered(model, step);
    set<ringStation_ptr> ringStationRingColor = getRingStationRingColorAltered(model, step);
    set<ringStation_ptr> ringStationAddBasesReq = getRingStationAddBasesReqAltered(model, step);



    if (action.isNotDefined()) {
        action = checkSetUpRingColorAction(ringStationRingColor, model, step);
    }

    if (!robotMov.empty() && !stationOcc.empty()) {
        robot_ptr robot = *robotMov.begin();
        station_ptr station = *stationOcc.begin();

        if (action.isNotDefined()) {
            action = checkCollectBaseAction(station, robot, model, step);
        }
        if (action.isNotDefined()) {
            action = checkDeliverProductAction(station, robot, model, step);
        }
        if (action.isNotDefined()) {
            action = checkfeedCapAction(station, robot, capStationCapColor, model, step);
        }
        if (action.isNotDefined()) {
            action = checkMountCapAction(station, robot, capStationCapColor, model, step);
        }
        if (action.isNotDefined()) {
            action = checkFeedBaseAction(station, robot, ringStationAddBasesReq, model, step);
        }
        if (action.isNotDefined()) {
            action = checkMountRingAction(station, ringStationRingColor, robot, model, step);
        }
        if (action.isNotDefined()) {
            action = checkPickUpWorkpiece(station, robot, model, step);
        }
    }
    return action;
}

Action FormulaGenerator::checkNoneAction(const std::map<string, int>& model, int step) {

    for (auto o : getGameData().getOrders()) {
        string orderDelivered = StepFormula::getVarNameOrderDelivered(*o, step - 1);

        if (model.at(orderDelivered) == Order::NOTDELIVERED) {
            return Action(Action::ActionType::NOTDEFINED);
        }
    }
    return Action(Action::ActionType::NONE);
}

Action FormulaGenerator::checkCollectBaseAction(
        const station_ptr& station,
        const robot_ptr& robot,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    //If involved station is not a base station, the action can not be CollectBase
    if (!station->isBaseStation()) return action;

    string rHoldsBaseName = StepFormula::getVarNameHoldsBase(*robot, step);
    Workpiece::Color color;
    int intColor;
    try {
        intColor = model.at(rHoldsBaseName);
        color = Workpiece::intToColor(intColor);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << rHoldsBaseName + "does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + rHoldsBaseName + "]" + " : " + std::to_string(intColor)
                + " has value which does not represent a color" << endl;
        return action;
    }
    /*The action is CollecBase, return this information, the involved robot and station, 
    and the coloring of the collected base*/
    action = Action(Action::ActionType::COLLECTBASE, robot, station, color);
    return action;
}

Action FormulaGenerator::checkSetUpRingColorAction(
        set<ringStation_ptr> ringStationRingColor,
        const std::map<string, int>& model,
        int step) {
    Action action(Action::ActionType::NOTDEFINED);
    if (ringStationRingColor.empty()) return action;

    ringStation_ptr ringStation = *ringStationRingColor.begin();

    string ringColorNamePrev = StepFormula::getVarNameRingColor(*ringStation, step - 1);
    string ringColorName = StepFormula::getVarNameRingColor(*ringStation, step);

    int intRingColorPrev;
    int intRingColor;
    Workpiece::Color ringColorPrev;
    Workpiece::Color ringColor;
    try {
        intRingColorPrev = model.at(ringColorNamePrev);
        intRingColor = model.at(ringColorName);
        ringColorPrev = Workpiece::intToColor(intRingColorPrev);
        ringColor = Workpiece::intToColor(intRingColor);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << ringColorNamePrev + " or " + ringColorName + "does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + ringColorNamePrev + "]" + " : " + std::to_string(intRingColorPrev)
                + " or " + "[" + ringColorName + "]" + " : " + std::to_string(intRingColor) + " has a value which does not represent a color" << endl;
        return action;
    }

    if (ringColorPrev == Workpiece::NONE && ringColor != Workpiece::NONE) {
        action = Action(Action::ActionType::SETUPRINGCOLOR, ringStation, ringColor);
    }
    return action;
}

Action FormulaGenerator::checkFeedBaseAction(
        const station_ptr& station,
        const robot_ptr& robot,
        set<ringStation_ptr> ringStationAddBasesReq,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    if (ringStationAddBasesReq.empty()) return action;

    ringStation_ptr ringStation = *ringStationAddBasesReq.begin();

    string baseReqNamePrev = StepFormula::getVarNameBaseReq(*ringStation, step - 1);
    string baseReqName = StepFormula::getVarNameBaseReq(*ringStation, step);

    string baseColorNamePrev = StepFormula::getVarNameHoldsBase(*robot, step - 1);
    string baseColorName = StepFormula::getVarNameHoldsBase(*robot, step);

    int baseReqPrev;
    int baseReq;
    int intBaseColorPrev;
    int intBaseColor;
    Workpiece::Color baseColorPrev;
    Workpiece::Color baseColor;
    try {
        baseReqPrev = model.at(baseReqNamePrev);
        baseReq = model.at(baseReqName);

        intBaseColorPrev = model.at(baseColorNamePrev);
        intBaseColor = model.at(baseColorName);

        baseColorPrev = Workpiece::intToColor(intBaseColorPrev);
        baseColor = Workpiece::intToColor(intBaseColor);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << baseReqNamePrev + " or " + baseReqName + "does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + baseColorNamePrev + "]" + " : " + std::to_string(intBaseColorPrev)
                + " or " + "[" + baseColorName + "]" + " : " + std::to_string(intBaseColor) + " has a value which does not represent a color" << endl;
        return action;
    }

    if (baseReq == baseReqPrev - 1 && baseColorPrev != Workpiece::NONE && baseColor == Workpiece::NONE) {
        action = Action(Action::ActionType::FEEDBASE, ringStation, baseColorPrev);
    }
    return action;
}

Action FormulaGenerator::checkMountRingAction(
        const station_ptr& station,
        set<ringStation_ptr> ringStationRingColor,
        const robot_ptr& robot,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    if (ringStationRingColor.empty()) return action;

    ringStation_ptr ringStation = *ringStationRingColor.begin();

    string ringColorNamePrev = StepFormula::getVarNameRingColor(*ringStation, step - 1);
    string ringColorName = StepFormula::getVarNameRingColor(*ringStation, step);

    int intRingColorPrev;
    int intRingColor;
    Workpiece::Color ringColorPrev;
    Workpiece::Color ringColor;
    try {
        intRingColorPrev = model.at(ringColorNamePrev);
        intRingColor = model.at(ringColorName);
        ringColorPrev = Workpiece::intToColor(intRingColorPrev);
        ringColor = Workpiece::intToColor(intRingColor);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << ringColorNamePrev + " or " + ringColorName + "does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + ringColorNamePrev + "]" + " : " + std::to_string(intRingColorPrev)
                + " or " + "[" + ringColorName + "]" + " : " + std::to_string(intRingColor) + " has a value which does not represent a color" << endl;
        return action;
    }

    if (ringColorPrev != Workpiece::NONE && ringColor == Workpiece::NONE) {
        action = Action(Action::ActionType::MOUNTRING, ringStation, ringColorPrev);
    }
    return action;

}

Action FormulaGenerator::checkfeedCapAction(
        const station_ptr& station,
        const robot_ptr& robot,
        const set<capStation_ptr>& capStationCapColor,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    //If involved station is not a base station, the action can not be FeedCap
    if (!station->isCapStation()) return action;
    //If cap loaded in the capStation did not change, the action can not be feedCap
    if (capStationCapColor.size() == 0) return action;

    capStation_ptr cs = *capStationCapColor.begin();

    if (station != cs) {
        std::cerr << "checkfeedCapAction: station " << station->getVarIdentifier()
                << " and cs " << cs->getVarIdentifier() << " should be the same.\n";
    }

    string csCapColor = StepFormula::getVarNameCapColor(*cs, step);
    string csCapColorPrev = StepFormula::getVarNameCapColor(*cs, step - 1);

    Workpiece::Color capColor;
    int intCapColor;
    Workpiece::Color capColorPrev;
    int intCapColorPrev;
    try {
        intCapColor = model.at(csCapColor);
        capColor = Workpiece::intToColor(intCapColor);
        intCapColorPrev = model.at(csCapColorPrev);
        capColorPrev = Workpiece::intToColor(intCapColorPrev);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << csCapColor + " or " + csCapColorPrev + " does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + csCapColor + "]" + " : " + std::to_string(intCapColor)
                + " or " + csCapColorPrev + "]" + " : " + std::to_string(intCapColorPrev)
                + " has value which does not represent a color" << endl;
        return action;
    }

    //no cap fed previously, now a cap is fed
    if (capColorPrev == Workpiece::NONE && capColor != Workpiece::NONE) {
        action = Action(Action::ActionType::FEEDCAP, robot, station, capColor);
    }

    return action;
}

Action FormulaGenerator::checkMountCapAction(
        const station_ptr& station,
        const robot_ptr& robot,
        const set<capStation_ptr>& capStationCapColor,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    //If involved station is not a base station, the action can not be FeedCap
    if (!station->isCapStation()) return action;
    //If cap loaded in the capStation did not change, the action can not be feedCap
    if (capStationCapColor.size() == 0) return action;

    capStation_ptr cs = *capStationCapColor.begin();

    if (station != cs) {
        std::cerr << "checkfeedCapAction: station " << station->getVarIdentifier()
                << " and cs " << cs->getVarIdentifier() << " should be the same.\n";
    }

    string csCapColor = StepFormula::getVarNameCapColor(*cs, step);
    string csCapColorPrev = StepFormula::getVarNameCapColor(*cs, step - 1);

    Workpiece::Color capColor;
    int intCapColor;
    Workpiece::Color capColorPrev;
    int intCapColorPrev;
    try {
        intCapColor = model.at(csCapColor);
        capColor = Workpiece::intToColor(intCapColor);
        intCapColorPrev = model.at(csCapColorPrev);
        capColorPrev = Workpiece::intToColor(intCapColorPrev);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << csCapColor + " or " + csCapColorPrev + " does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + csCapColor + "]" + " : " + std::to_string(intCapColor)
                + " or " + csCapColorPrev + "]" + " : " + std::to_string(intCapColorPrev)
                + " has value which does not represent a color" << endl;
        return action;
    }

    //cap fed previously, now no cap is fed
    if (capColorPrev != Workpiece::NONE && capColor == Workpiece::NONE) {
        action = Action(Action::ActionType::MOUNTCAP, robot, station, capColorPrev);
    }

    return action;
}

Action FormulaGenerator::checkPickUpWorkpiece(
        const station_ptr& station,
        const robot_ptr& robot,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    if (!station->isCapStation() && !station->isRingStation()) return action;

    string csHoldsBase = StepFormula::getVarNameHoldsBase(*station, step);
    string csHoldsBasePrev = StepFormula::getVarNameHoldsBase(*station, step - 1);

    string rHoldsBase = StepFormula::getVarNameHoldsBase(*robot, step);
    string rHoldsBasePrev = StepFormula::getVarNameHoldsBase(*robot, step - 1);

    Workpiece::Color csBaseColor;
    int intCsBaseColor;
    Workpiece::Color csBaseColorPrev;
    int intCsBaseColorPrev;
    Workpiece::Color rBaseColor;
    int intRBaseColor;
    Workpiece::Color rBaseColorPrev;
    int intRBaseColorPrev;
    try {
        intCsBaseColor = model.at(csHoldsBase);
        csBaseColor = Workpiece::intToColor(intCsBaseColor);
        intCsBaseColorPrev = model.at(csHoldsBasePrev);
        csBaseColorPrev = Workpiece::intToColor(intCsBaseColorPrev);

        intRBaseColor = model.at(rHoldsBase);
        rBaseColor = Workpiece::intToColor(intRBaseColor);
        intRBaseColorPrev = model.at(rHoldsBasePrev);
        rBaseColorPrev = Workpiece::intToColor(intRBaseColorPrev);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << csHoldsBase + " or " + csHoldsBasePrev + " or " + rHoldsBase + " or " + rHoldsBasePrev + " does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + csHoldsBase + "]" + " : " + std::to_string(intCsBaseColor)
                + "or [" + csHoldsBasePrev + "]" + " : " + std::to_string(intCsBaseColorPrev)
                + "or [" + rHoldsBase + "]" + " : " + std::to_string(intRBaseColor)
                + "or [" + rHoldsBasePrev + "]" + " : " + std::to_string(intRBaseColorPrev)
                + " has value which does not represent a color" << endl;
        return action;
    }

    //Pick up Workpiece
    if (csBaseColorPrev != Workpiece::NONE && csBaseColor == Workpiece::NONE
            && rBaseColorPrev == Workpiece::NONE && rBaseColor != Workpiece::NONE) {
        action = Action(Action::ActionType::PICKWORKPIECE, robot, station, csBaseColorPrev);
    }

    //Pick up transparent base and drop it
    if (station->isCapStation() && csBaseColorPrev == Workpiece::TRANSPARENT && csBaseColor == Workpiece::NONE
            && rBaseColorPrev == Workpiece::NONE && rBaseColor == Workpiece::NONE) {
        action = Action(Action::ActionType::DROPTRANSPARENTBASE, robot, station, csBaseColorPrev);
    }

    return action;
}

Action FormulaGenerator::checkDeliverProductAction(
        const station_ptr& station,
        const robot_ptr& robot,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    if (!station->isDeliveryStation()) return action;

    string rHoldsBase = StepFormula::getVarNameHoldsBase(*robot, step);
    string rHoldsBasePrev = StepFormula::getVarNameHoldsBase(*robot, step - 1);

    Workpiece::Color baseColor;
    int intBaseColor;
    Workpiece::Color baseColorPrev;
    int intBaseColorPrev;
    try {
        intBaseColor = model.at(rHoldsBase);
        baseColor = Workpiece::intToColor(intBaseColor);
        intBaseColorPrev = model.at(rHoldsBasePrev);
        baseColorPrev = Workpiece::intToColor(intBaseColorPrev);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << rHoldsBase + " or " + rHoldsBasePrev + " does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + rHoldsBase + "]" + " : " + std::to_string(intBaseColor)
                + " or " + rHoldsBasePrev + "]" + " : " + std::to_string(intBaseColorPrev)
                + " has value which does not represent a color" << endl;
        return action;
    }

    if (baseColorPrev != Workpiece::NONE && baseColor == Workpiece::NONE) {
        action = Action(Action::ActionType::DELIVERPRODUCT, robot, station, baseColorPrev);
    }

    return action;
}