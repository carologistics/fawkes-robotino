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

std::string FormulaGenerator::printWorldState(const z3::model& z3model, int step) {
    string output = "Step " + std::to_string(step) + ":\n";
    std::map<string, int> model = transferZ3ModelToMap(z3model);

    for (auto r : getGameData().getRobots()) {
        int intColorBase = model.at(StepFormula::getVarNameHoldsBase(*r, step));
        int intColorCap = model.at(StepFormula::getVarNameHoldsCap(*r, step));

        Workpiece::Color colorBase = Workpiece::intToColor(intColorBase);
        Workpiece::Color colorCap = Workpiece::intToColor(intColorCap);


        output += r->getVarIdentifier() + ": ";
        Workpiece wp(Workpiece::intToColor(colorBase),{}, colorCap);
        output += wp.toString();

        if (colorBase == Workpiece::NOTDEFINED) {
            output += " Base: " + std::to_string(intColorBase);
        }

        if (colorCap == Workpiece::NOTDEFINED) {
            output += " Cap: " + std::to_string(intColorCap);
        }

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

    for (auto cs : getGameData().getCapStations()) {
        output += cs->getVarIdentifier() + ": ";
        Workpiece wp(Workpiece::intToColor(model.at(StepFormula::getVarNameHoldsBase(*cs, step))),{},
        Workpiece::intToColor(model.at(StepFormula::getVarNameHoldsCap(*cs, step))));
        output += wp.toString();
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

    if (stationOcc.size() == 0) {
        result = false;
        std::cerr << "At least one StationOccupied has to be altered in step " << std::to_string(step) << endl;
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


    if (!checkConsistency(model, step)) return action;

    set<station_ptr> stationOcc = getStationOccAltered(model, step);
    set<robot_ptr> robotMov = getRobotMovAltered(model, step);
    set<robot_ptr> robotWorkpiece = getRobotWorkpieceAltered(model, step);
    set<capStation_ptr> capStationWorkpiece = getCapStationWorkpieceAltered(model, step);
    set<capStation_ptr> capStationCapColor = getCapStationCapColorAltered(model, step);

    station_ptr station = *stationOcc.begin();


    if (!robotWorkpiece.empty()) {
        robot_ptr robot = *robotWorkpiece.begin();

        if (action.isNotDefined()) {
            action = checkCollectBaseAction(station, robot, model, step);
        }
        if (action.isNotDefined()) {
            action = checkPickWorkpieceAction(station, robot, capStationWorkpiece, model, step);
        }
        if (action.isNotDefined()) {
            action = checkDeliverProductAction(station, robot, model, step);
        }
    }

    if (!robotMov.empty()) {
        robot_ptr robot = *robotMov.begin();

        if (action.isNotDefined()) {
            action = checkfeedCapAction(station, robot, capStationCapColor, model, step);
        }
        if (action.isNotDefined()) {
            action = checkMountCapAction(station, robot, capStationCapColor, model, step);
        }
    }
    return action;
}

Action FormulaGenerator::checkNoneAction(const std::map<string, int>& model, int step) {

    for (auto o : getGameData().getOrders()) {
        string orderDelivered = StepFormula::getVarNameOrderDelivered(*o, step-1);

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

Action FormulaGenerator::checkPickWorkpieceAction(
        const station_ptr& station,
        const robot_ptr& robot,
        set<capStation_ptr> capStationWorkpiece,
        const std::map<string, int>& model,
        int step) {

    Action action(Action::ActionType::NOTDEFINED);
    if (!station->isCapStation()) return action;
    if (capStationWorkpiece.size() == 0) return action;

    capStation_ptr cs = *capStationWorkpiece.begin();

    if (station != cs) {
        std::cerr << "checkfeedCapAction: station " << station->getVarIdentifier()
                << " and cs " << cs->getVarIdentifier() << " should be the same.\n";
    }

    string csHoldsBase = StepFormula::getVarNameHoldsBase(*cs, step);
    string csHoldsBasePrev = StepFormula::getVarNameHoldsBase(*cs, step - 1);

    Workpiece::Color baseColor;
    int intBaseColor;
    Workpiece::Color baseColorPrev;
    int intBaseColorPrev;
    try {
        intBaseColor = model.at(csHoldsBase);
        baseColor = Workpiece::intToColor(intBaseColor);
        intBaseColorPrev = model.at(csHoldsBasePrev);
        baseColorPrev = Workpiece::intToColor(intBaseColorPrev);
        //Catch inconsistency in the Model and if inconsistent return a not-defined action 
    } catch (const std::out_of_range& e) {
        std::cerr << csHoldsBase + " or " + csHoldsBasePrev + " does not denote a variable \n";
        return action;
    } catch (std::invalid_argument e) {
        std::cerr << "[" + csHoldsBase + "]" + " : " + std::to_string(intBaseColor)
                + " or " + csHoldsBasePrev + "]" + " : " + std::to_string(intBaseColorPrev)
                + " has value which does not represent a color" << endl;
        return action;
    }

    //cap fed previously, now no cap is fed
    if (baseColorPrev != Workpiece::NONE && baseColor == Workpiece::NONE) {
        action = Action(Action::ActionType::PICKWORKPIECE, robot, station, baseColorPrev);
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