#include <cstdlib>

#include "StepFormula.h"
#include "Workpiece.h"


/*
 * 
 */
int main(int argc, char** argv) {
    
    StepFormula step0 = StepFormula(0);
    StepFormula step1 = StepFormula(1, step0);
    Workpiece w = Workpiece(Workpiece::RED, Workpiece::BLACK, Workpiece::SILVER, Workpiece::BLACK, Workpiece::BLACK);
    Robot r = Robot(2);
    Order o = Order(3, w);
    CapStation cs = CapStation(5);
    cout << step1.getMountCapStepNotFinishedFormula(r, o, cs) << endl;
    
    /*Variable x = freshRealVariable("x");
    Variable y = freshRealVariable("y");
    Variable i1 = freshIntegerVariable("i1");
    Variable i2 = freshIntegerVariable("i2");
    Variable i3 = freshIntegerVariable("i3");
//    Variable i = newArithmeticVariable( "i", VariableType::VT_INT );
    Variable b = freshBooleanVariable("b");
//    Sort sortS = newSort( "S" );
//    Sort sortT = newSort( "T" );
//    Variable u = VariableNamePool::getInstance().newUninterpretedVariable( "u" );
//    Variable v = VariableNamePool::getInstance().newUninterpretedVariable( "v" );
//    UVariable uu( u, sortS );
//    UVariable uv( v, sortT );

    // Next we see an example how to create polynomials, which form the left-hand sides of the constraints:
    Pol px( x );
    Pol py( y );
    Pol lhsA = px.pow(2) - py;
    Pol lhsB = Rational(4) * px + py - Rational(8) * py.pow(7);
    Pol lhsD = px*py;
    Pol pi1( i1 );
    Pol pi2( i2 );
    Pol pi3( i3 );
    Pol lhsC = Rational(2) * pi1 + Rational(2) * pi2 + Rational(2) * pi3 - Rational(5);

    // Constraints can then be constructed as follows:
    Constr constraintA = Constr( lhsD, Relation::EQ );
    EXPECT_EQ( Constr( px-Rational(1), Relation::LEQ ), Constr( x, Relation::LEQ, Rational(1) ) );
    EXPECT_EQ( Constr( px-Rational(1), Relation::LESS ), Constr( x, Relation::LESS, Rational(1) ) );
    EXPECT_EQ( Constr( px-Rational(1), Relation::GEQ ), Constr( x, Relation::GEQ, Rational(1) ) );
    EXPECT_EQ( Constr( px-Rational(1), Relation::GREATER ), Constr( x, Relation::GREATER, Rational(1) ) );
    EXPECT_EQ( Constr( px+Rational(1), Relation::LEQ ), Constr( x, Relation::LEQ, -Rational(1) ) );
    EXPECT_EQ( Constr( px+Rational(1), Relation::LESS ), Constr( x, Relation::LESS, -Rational(1) ) );
    EXPECT_EQ( Constr( px+Rational(1), Relation::GEQ ), Constr( x, Relation::GEQ, -Rational(1) ) );
    EXPECT_EQ( Constr( px+Rational(1), Relation::GREATER ), Constr( x, Relation::GREATER, -Rational(1) ) );

    // Uninterpreted functions are

    // Now, we can construct the atoms of the Boolean Ast
    FormulaT atomA( constraintA );
    cout << atomA <<endl;
    FormulaT atomB( lhsB, Relation::EQ );
    cout << atomB <<endl;
    FormulaT atomC( b );
    FormulaT inEq( lhsC, Relation::EQ );
    EXPECT_TRUE( inEq.getType() == FormulaType::FALSE );

    // and the Ast itself:
    Formulas<Pol> subAstsA;
    subAstsA.emplace_back( FormulaType::NOT, atomC );
    subAstsA.push_back( atomA );
    subAstsA.push_back( atomB );
    FormulaT phiA( FormulaType::AND, subAstsA );
    cout << phiA <<endl;
    FormulaT tsVarA = FormulaPool<Pol>::getInstance().createTseitinVar( phiA );
    FormulaT phiC( FormulaType::OR, {FormulaT( FormulaType::NOT, atomA ), atomC} );
    FormulaT tsVarC = FormulaPool<Pol>::getInstance().createTseitinVar( phiC );
    FormulaT phiE( FormulaType::IMPLIES, {phiA, phiC} );
    FormulaT tsVarE = FormulaPool<Pol>::getInstance().createTseitinVar( phiE );
    FormulaT( FormulaType::XOR, {tsVarA, tsVarC, tsVarE} );*/
    
    return 0;
}

