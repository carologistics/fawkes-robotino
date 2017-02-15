
/* 
 * File:   FormulaGenerator.h
 * Author: leonard
 *
 * Created on February 15, 2017, 11:28 AM
 */

#ifndef FORMULAGENERATOR_H
#define FORMULAGENERATOR_H

using namespace carl;

typedef MultivariatePolynomial<Rational> Pol;
typedef Constraint<Pol> Constr;
typedef Formula<Pol> FormulaT;

class FormulaGenerator {
public:
    FormulaGenerator();
    FormulaGenerator(const FormulaGenerator& orig);
    virtual ~FormulaGenerator();
    
    FormulaT create(int steps);
    
private:

};

#endif /* FORMULAGENERATOR_H */

