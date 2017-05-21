#include <iostream>
#include "FormulaGenerator.h"

FormulaGenerator::FormulaGenerator(int amount, GameData& gameData) {
    generateSteps(amount, gameData);
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

void FormulaGenerator::setGameData(GameData &gameData) {
    this->gameData = gameData;
}

GameData FormulaGenerator::getGameData() {
    return this->gameData;
}

GameData FormulaGenerator::translateZ3ModelToGameData(z3::model model, int step) {
    for (int i = 0; i < model.size(); i++) {
        z3::func_decl function = model[i];
        
        GameData gameData = getGameData();
        
        for (auto s : gameData.getStations()) {
            StepFormula::getVarNameHoldsBase(*s, step);
            for(int i = 0; i < Workpiece::getMaxRingNumber(); i++){
                StepFormula::getVarNameHoldsRing(*s, i, step);
            }
            StepFormula::getVarNameHoldsCap(*s, step);
            StepFormula::getVarNameMachineOccupied(*s, step);
        }

        for (auto rs : gameData.getRingStations()) {
            StepFormula::getVarNameRingColor(*rs, step);
            StepFormula::getVarNameBaseReq(*rs, step);
        }

        for (auto cs : gameData.getCapStations()) {
            StepFormula::getVarNameCapColor(*cs, step);
        }

        for (auto r : gameData.getRobots()) {
            StepFormula::getVarNameHoldsBase(*r, step);
            for(int i = 0; i < Workpiece::getMaxRingNumber(); i++){
                StepFormula::getVarNameHoldsRing(*r, i, step);
            }
            StepFormula::getVarNameHoldsCap(*r, step);
            for (auto s : gameData.getStations()) {
                StepFormula::getVarNameMovingTime(*r, *s, step);
            }
        }
        std::cout << "Model contains [" << function.name() << "] " << model.get_const_interp(function) << std::endl;
    }
}