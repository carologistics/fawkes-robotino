#include <cstdlib>
#include <iostream>
#include "FormulaGenerator.h"
#include "GameData.h"
#include "Workpiece.h"
#include "Order.h"

using namespace std;

void testPrevStep();
GameData testGameData();
void testWorkpiece();
void testOrder();
void testMachine();

int main(int argc, char** argv) {
    GameData gD = testGameData();  
    FormulaGenerator fg = FormulaGenerator(1, gD);
    cout << fg.createFormula();
    return 0;
}

GameData testGameData() {
    GameData gD;

    auto r0 = make_shared<Robot>(0);
    auto r1 = make_shared<Robot>(1);
    Workpiece pr0(Workpiece::BLACK,{}, Workpiece::NONE);
    r0->setWorkpiece(pr0);
    gD.addMachine(r0);
    gD.addMachine(r1);

    auto bs0 = make_shared<BaseStation>(0);
    auto bs1 = make_shared<BaseStation>(1);
    bs0->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER});
    bs1->setPossibleBaseColors({Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER});
    bs0->setDispenseBaseTime(5000);
    bs1->setDispenseBaseTime(5000);
    gD.addMachine(bs0);
    gD.addMachine(bs1);

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
    
    gD.fillStations();
    
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

void testPrevStep() {
    GameData gm = testGameData();
    FormulaGenerator formulaGenerator(3, gm);
    cout << "Step:" << formulaGenerator.getStep(0)->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(1)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(1)->getPrevStep()->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(2)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(2)->getPrevStep()->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(3)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(3)->getPrevStep()->getStepNumber() << endl;
}

void testMachine() {
    auto r99 = make_shared<Robot>(99);
    auto rs0 = make_shared<RingStation>(0);
    auto cs5 = make_shared<RingStation>(5);
    auto ds2 = make_shared<RingStation>(2);
    r99->addMovingTime(*rs0, 15000);
    r99->addMovingTime(*cs5, 90);
    r99->addMovingTime(*ds2, 160);
    cout << r99->toString();
}
