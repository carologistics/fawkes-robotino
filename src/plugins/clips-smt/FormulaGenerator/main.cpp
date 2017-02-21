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
    testPrevStep();
    return 0;
}

GameData testGameData() {
    GameData gM;

    auto r0 = make_shared<Robot>(0);
    auto r1 = make_shared<Robot>(1);
    auto r2 = make_shared<Robot>(2);
    gM.addMachine(r0);
    gM.addMachine(r1);
    gM.addMachine(r2);

    auto bs0 = make_shared<BaseStation>(0);
    auto bs1 = make_shared<BaseStation>(1);
    gM.addMachine(bs0);
    gM.addMachine(bs1);

    auto rs0 = make_shared<RingStation>(0);
    auto rs1 = make_shared<RingStation>(1);
    gM.addMachine(rs0);
    gM.addMachine(rs1);

    auto cs0 = make_shared<CapStation>(0);
    auto cs1 = make_shared<CapStation>(1);
    gM.addMachine(cs0);
    gM.addMachine(cs1);

    auto ds0 = make_shared<DeliveryStation>(0);
    gM.addMachine(ds0);

    //gM.addMovingTime(*rs0, *cs1, 150000);
    //gM.addMovingTime(*rs1, *rs0, 9986301);
    //gM.addMovingTime(*ds0, *cs0, 2866666);

    return gM;
}
//NONE, RED, BLACK, SILVER, TRANSPARENT, BLUE, GREEN, YELLOW, ORANGE, GREY
void testWorkpiece(){
    Workpiece wp;
    cout << wp.toString() << endl;
    wp.setBaseColor(Workpiece::RED);
    wp.setRingColor(Workpiece::YELLOW,0);
    wp.setCapColor(Workpiece::BLUE);
    cout << wp.toString() << endl;
    
    Workpiece wp2(Workpiece::RED, {Workpiece::YELLOW}, Workpiece::BLUE);
    cout << wp2.toString() << endl;
}


void testOrder(){
    Workpiece product(Workpiece::RED, {Workpiece::YELLOW, Workpiece::BLACK}, Workpiece::BLUE);
    Order o(555, product, 15000);
    cout << o.toString();
}

void testPrevStep() {
    GameData gm = testGameData();
    FormulaGenerator formulaGenerator(3, gm);
    cout << "Step:" << formulaGenerator.getStep(0)->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(1)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(1)->getPreviousStep()->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(2)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(2)->getPreviousStep()->getStepNumber() << endl;
    cout << "Step:" << formulaGenerator.getStep(3)->getStepNumber() << endl;
    cout << "PrevStep:" << formulaGenerator.getStep(3)->getPreviousStep()->getStepNumber() << endl;
}

void testMachine(){
    auto r99 = make_shared<Robot>(99);
    auto rs0 = make_shared<RingStation>(0);
    auto cs5 = make_shared<RingStation>(5);
    auto ds2 = make_shared<RingStation>(2);
    r99->setMovingTime(*rs0, 15000);
    r99->setMovingTime(*cs5, 90);
    r99->setMovingTime(*ds2, 160);
    cout << r99->toString();
}
