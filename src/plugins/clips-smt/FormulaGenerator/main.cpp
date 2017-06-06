#include <cstdlib>
#include <iostream>
#include "FormulaGenerator.h"
#include "GameData.h"
#include "Workpiece.h"
#include "Order.h"

#include <z3++.h>
#include <carl/numbers/numbers.h>
// #include <carl/core/VariablePool.h>
#include <carl/formula/Formula.h>
#include <carl/io/SMTLIBStream.h>

using namespace std;

void testPrevStep();
GameData testGameData();
GameData testGameData1();
void testWorkpiece();
void testOrder();
void testMachine();
z3::model solve_formula_from_smt_file(std::string path);
void smt_test_formulaGenerator();

z3::context _z3_context;

int main(int argc, char** argv) {
    /*GameData gD = testGameData1();  
    FormulaGenerator fg = FormulaGenerator(1, gD);
    cout << fg.createFormula();*/
    smt_test_formulaGenerator();
    return 0;
    /*
                z3::model model = s.get_model();
                for(unsigned i=0; i<model.size(); ++i) {
                        z3::func_decl function = model[i];
                        std::cout << "Model contains [" << function.name() <<"] " << model.get_const_interp(function) << std::endl;
                }*/
}

void smt_test_formulaGenerator() {
    std::cout << "Test FormulaGenerator extern binary \n";

    GameData gD = testGameData1();
    FormulaGenerator fg = FormulaGenerator(30, gD);
    
    std::cout << gD.toString();

    std::cout << "Export FormulaGenerator formula to file fg_formula.smt\n";
    std::ofstream outputFile("/home/leonard/fg_formula.smt");
    outputFile << carl::outputSMTLIB(carl::Logic::QF_NIRA,{fg.createFormula()});
    outputFile.close();

    std::cout << "Import FormulaGenerator formula from file fg_formula.smt into z3 formula\n";

    z3::model model = solve_formula_from_smt_file("/home/leonard/fg_formula.smt");

    std::cout << fg.printWorldStates(model);
    std::cout << fg.printActions(model);
}

z3::model solve_formula_from_smt_file(std::string path) {
    Z3_ast a = Z3_parse_smtlib2_file(_z3_context, path.c_str(), 0, 0, 0, 0, 0, 0); // TODO (Igor) Exchange path with config value
    z3::expr e(_z3_context, a);

    z3::solver s(_z3_context);
    s.add(e);

    // Start measuring sovling time
    std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
    if (s.check() == z3::sat) {
        // Stop measuring sovling time
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

        // Compute time for solving
        double diff_ms = (double) std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count() / 1000;
        double diff_m = (double) std::chrono::duration_cast<std::chrono::seconds> (end - begin).count() / 60;

        std::cout << "Test of import .smt file into z3 constraint did work (SAT) [%f ms, %f m] \n";

        z3::model model = s.get_model();
        for (unsigned i = 0; i < model.size(); ++i) {
            z3::func_decl function = model[i];
            //std::cout << "Model contains [" << function.name() << "] " << model.get_const_interp(function) << std::endl;
        }
    } else std::cout << "Test of import .smt file into z3 constraint did NOT work (UNSAT)";
    return s.get_model();
}

GameData testGameData1() {
    GameData gD;

    auto r0 = make_shared<Robot>(0);
    auto r1 = make_shared<Robot>(1);
    auto r2 = make_shared<Robot>(2);
    gD.addMachine(r0);
    gD.addMachine(r1);
    gD.addMachine(r2);

    Workpiece wr0 = Workpiece();
    r0->setWorkpiece(wr0);
    r0->setFeedWorkpieceTime(1);
    r0->setTakeWorkpieceTime(1);
    r1->setFeedWorkpieceTime(1);
    r1->setTakeWorkpieceTime(1);
    r2->setFeedWorkpieceTime(1);
    r2->setTakeWorkpieceTime(1);

    auto bs0 = make_shared<BaseStation>(0);
    bs0->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER});
    bs0->setDispenseBaseTime(60);
    gD.addMachine(bs0);
    
    auto rs0 = make_shared<RingStation>(0);
    rs0->addPossibleRingColor(Workpiece::BLUE, 2);
    rs0->addPossibleRingColor(Workpiece::GREEN, 1);
    rs0->setFeedBaseTime(5000);
    rs0->setMountRingTime(40000);
    gD.addMachine(rs0);

    auto cs0 = make_shared<CapStation>(0);
    cs0->addPossibleCapColor(Workpiece::GREY);
    cs0->setFeedCapTime(50);
    cs0->setMountCapTime(150);
    gD.addMachine(cs0);

    auto ds0 = make_shared<DeliveryStation>(0);
    ds0->setDeliverProductTime(0);
    gD.addMachine(ds0);

    Machine::addMovingTime(*r0, *bs0, 5);
    Machine::addMovingTime(*r0, *cs0, 5);
    Machine::addMovingTime(*r0, *ds0, 5);
    Machine::addMovingTime(*r0, *rs0, 5);

    Machine::addMovingTime(*r1, *bs0, 6);
    Machine::addMovingTime(*r1, *cs0, 6);
    Machine::addMovingTime(*r1, *ds0, 6);
    Machine::addMovingTime(*r1, *rs0, 5);

    Machine::addMovingTime(*bs0, *cs0, 10);
    Machine::addMovingTime(*bs0, *ds0, 20);
    Machine::addMovingTime(*bs0, *rs0, 20);

    Machine::addMovingTime(*cs0, *ds0, 30);
    Machine::addMovingTime(*cs0, *rs0, 30);

    Machine::addMovingTime(*ds0, *rs0, 30);
    
    Workpiece p0(Workpiece::BLACK, {Workpiece::BLUE, Workpiece::GREEN, Workpiece::BLUE}, Workpiece::GREY);
    Workpiece p1(Workpiece::RED, {}, Workpiece::GREY);
    auto o0 = make_shared<Order>(6, p0, 1000);
    auto o1 = make_shared<Order>(7, p1, 11000);

    gD.addOrder(o0);
    gD.addOrder(o1);

    return gD;
}

GameData testGameData() {
    GameData gD;

    auto r0 = make_shared<Robot>(0);
    auto r1 = make_shared<Robot>(1);
    Workpiece pr0(Workpiece::BLACK,{}, Workpiece::NONE);
    r0->setWorkpiece(pr0);
    gD.addMachine(r0);
    //gD.addMachine(r1);

    auto bs0 = make_shared<BaseStation>(0);
    auto bs1 = make_shared<BaseStation>(1);
    bs0->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER});
    bs1->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER});
    bs0->setDispenseBaseTime(5000);
    bs1->setDispenseBaseTime(5000);
    gD.addMachine(bs0);
    //gD.addMachine(bs1);

    auto rs0 = make_shared<RingStation>(0);
    auto rs1 = make_shared<RingStation>(1);
    rs0->addPossibleRingColor(Workpiece::BLUE, 0);
    rs0->addPossibleRingColor(Workpiece::GREEN, 1);
    rs1->addPossibleRingColor(Workpiece::YELLOW, 0);
    rs1->addPossibleRingColor(Workpiece::ORANGE, 2);
    rs0->setFeedBaseTime(5000);
    rs1->setFeedBaseTime(5000);
    rs0->setMountRingTime(40000);
    rs1->setMountRingTime(60000);
    rs1->setAdditinalBasesFed(1);
    rs1->setRingColorSetup(Workpiece::ORANGE);
    gD.addMachine(rs0);
    gD.addMachine(rs1);

    auto cs0 = make_shared<CapStation>(0);
    auto cs1 = make_shared<CapStation>(1);
    cs0->addPossibleCapColor(Workpiece::GREY);
    cs1->addPossibleCapColor(Workpiece::BLACK);
    cs0->setFeedCapTime(5000);
    cs1->setFeedCapTime(5000);
    cs0->setMountCapTime(15000);
    cs1->setMountCapTime(25000);
    cs0->setFedCapColor(Workpiece::GREY);
    gD.addMachine(cs0);
    gD.addMachine(cs1);

    auto ds0 = make_shared<DeliveryStation>(0);
    gD.addMachine(ds0);

    Machine::addMovingTime(*r0, *bs0, 30000);
    Machine::addMovingTime(*r0, *bs1, 5000);
    Machine::addMovingTime(*r0, *rs0, 10000);
    Machine::addMovingTime(*r0, *rs1, 5000);
    Machine::addMovingTime(*r0, *cs0, 5000);
    Machine::addMovingTime(*r0, *cs1, 60000);
    Machine::addMovingTime(*r0, *ds0, 60000);

    Machine::addMovingTime(*r1, *bs0, 5000);
    Machine::addMovingTime(*r1, *bs1, 30000);
    Machine::addMovingTime(*r1, *rs0, 5000);
    Machine::addMovingTime(*r1, *rs1, 60000);
    Machine::addMovingTime(*r1, *cs0, 10000);
    Machine::addMovingTime(*r1, *cs1, 60000);
    Machine::addMovingTime(*r1, *ds0, 5000);

    Machine::addMovingTime(*bs0, *bs1, 10000);
    Machine::addMovingTime(*bs0, *rs0, 10000);
    Machine::addMovingTime(*bs0, *rs1, 30000);
    Machine::addMovingTime(*bs0, *cs0, 25000);
    Machine::addMovingTime(*bs0, *cs1, 25000);
    Machine::addMovingTime(*bs0, *ds0, 25000);

    Machine::addMovingTime(*bs1, *rs0, 5000);
    Machine::addMovingTime(*bs1, *rs1, 60000);
    Machine::addMovingTime(*bs1, *cs0, 5000);
    Machine::addMovingTime(*bs1, *cs1, 30000);
    Machine::addMovingTime(*bs1, *ds0, 5000);

    Machine::addMovingTime(*rs0, *rs1, 30000);
    Machine::addMovingTime(*rs0, *cs0, 25000);
    Machine::addMovingTime(*rs0, *cs1, 30000);
    Machine::addMovingTime(*rs0, *ds0, 30000);

    Machine::addMovingTime(*rs1, *cs0, 30000);
    Machine::addMovingTime(*rs1, *cs1, 25000);
    Machine::addMovingTime(*rs1, *ds0, 30000);

    Machine::addMovingTime(*cs0, *cs1, 30000);
    Machine::addMovingTime(*cs0, *ds0, 25000);

    Machine::addMovingTime(*cs1, *ds0, 25000);

    Workpiece p0(Workpiece::BLACK,{Workpiece::ORANGE}, Workpiece::GREY);
    auto o0 = make_shared<Order>(6, p0, 60000);
    Workpiece p1(Workpiece::BLACK,{Workpiece::BLUE}, Workpiece::BLACK);
    auto o1 = make_shared<Order>(7, p1, 300000);
    Workpiece p2(Workpiece::SILVER,{Workpiece::ORANGE}, Workpiece::GREY);
    auto o2 = make_shared<Order>(3, p2, 15000);

    gD.addOrder(o0);
    gD.addOrder(o1);
    gD.addOrder(o2);

    return gD;
}

//NONE, RED, BLACK, SILVER, TRANSPARENT, BLUE, GREEN, YELLOW, ORANGE, GREY

void testWorkpiece() {
    Workpiece wp;
    cout << wp.toString() << endl;
    wp.setBaseColor(Workpiece::RED);
    wp.setRingColor(Workpiece::YELLOW, 0);
    wp.setCapColor(Workpiece::BLUE);
    cout << wp.toString() << endl;

    Workpiece wp2(Workpiece::RED,{Workpiece::YELLOW}, Workpiece::BLUE);
    cout << wp2.toString() << endl;
}

void testOrder() {
    Workpiece product(Workpiece::RED,{Workpiece::YELLOW, Workpiece::BLACK}, Workpiece::BLUE);
    Order o(555, product, 15000);
    cout << o.toString();
}

/*void testPrevStep() {
    GameData gm = testGameData();
    FormulaGenerator formulaGenerator(3, gm);
    cout << "Step:" << formulaGenerator.getStep(0)->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(1)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(1)->getPrevStep()->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(2)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(2)->getPrevStep()->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(3)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(3)->getPrevStep()->getStepNumber() << endl;
}*/

void testMachine() {
    auto r99 = make_shared<Robot>(99);
    auto rs0 = make_shared<RingStation>(0);
    auto cs5 = make_shared<RingStation>(5);
    auto ds2 = make_shared<RingStation>(2);
    //r99->addMovingTime(*rs0, 15000);
    //r99->addMovingTime(*cs5, 90);
    //r99->addMovingTime(*ds2, 160);
    cout << r99->toString();
}
