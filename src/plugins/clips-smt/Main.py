from z3 import *
import csv
import time

########################################################################################################

def loadDistances(navgraphFileName,inverted_indices,n_machines):
    distance_map = {}
    for i in range(0,n_machines+1):
       for j in range(0,n_machines+1):
           if(not(i==j) and not(j==0)):
               if not('{}_{}'.format(i,j) in distance_map) and not('{}_{}'.format(j,i) in distance_map):
                   distance_map['{}_{}'.format(i,j)] = 0
    dist = {}
    with open(navgraphFileName, 'r') as f:
        fileReader = csv.reader(f, delimiter=';')
        for row in fileReader:
            if ('C-' in row[0]) and (not ('M-' in row[1])):
                if(row[0] in inverted_indices and row[1] in inverted_indices):
                    if('{}_{}'.format(inverted_indices[row[0]],inverted_indices[row[1]]) in distance_map):
                        dist['dist_{}_{}'.format(inverted_indices[row[0]],inverted_indices[row[1]])] = round(float(row[2]), 1)
    return dist

########################################################################################################
## Declare problem variables#
#############################
def declareVariables(n_robots,n_machines):

    # Variables pos_i_j
    varPos = {}

    for i in range(1,n_robots+1):
        for j in range(-3,n_machines+1):
            varPos['pos_{}_{}'.format(i,j)] = Int('pos_{}_{}'.format(i,j))

    # Variables d_i_j
    varD = {}

    for i in range(1,n_robots+1):
        for j in range (-3,n_machines+1):
            if(not(j==-3) and (not(j==-2)) and (not(j==-1))):
                varD['d_{}_{}'.format(i,j)] = Real('d_{}_{}'.format(i,j))

    # Variables m_i to get the max d_i_M
    varM = {}
    for i in range(1,n_robots+1):
        varM['m_{}'.format(i)] = Bool('m_{}'.format(i))

    return varPos,varD,varM


########################################################################################################
## Adding constraints to the solver#
####################################
def addConstraints(o,varPos, varD, varM,distances,n_robots,n_machines):

    ##Assert d_i_0 must be 0 (no cost for the first action)
    for k,v in varD.items():
        if('_0' in k):
##            print(v == 0)
            o.add(v == 0)

    ## Assert d_i_j <= d_i_M
    for i in range(1,n_robots+1):
        for j in range(1,n_machines):
            o.add(varD['d_{}_{}'.format(i,j)] <= varD['d_{}_{}'.format(i,n_machines)])

    #Assert constraint for initial distance:  initial locations <= pos_i_j <= n_machines
    for k,v in varPos.items():
        if(not('_0'in k) and not('-' in k)):
            o.add(Or([v == k for k in range(-4,n_machines+1)]))



    ##Assert some constraints for initial positions
    for i in range(n_robots+1):
        for k,v in varPos.items():
            if('pos_{}_-'.format(i) in k):
                if(i == 1):
##                    print(v == -1)
                    o.add(v == -1)
                elif(i == 2):
                    if('-1' in k):
##                        print(v == -1)
                        o.add(v == -1)
                    elif('-2' in k):
##                        print(v == -2)
                        o.add(v == -2)
                    elif('-3' in k):
##                        print(v == -2)
                        o.add(v == -2)
                elif(i==3):
                    if('-1' in k):
##                        print(v == -1)
                        o.add(v == -1)
                    elif('-2' in k):
##                        print(v == -2)
                        o.add(v == -2)
                    elif('-3' in k):
##                        print(v == -3)
                        o.add(v == -3)
            if('pos_{}_0'.format(i) in k):
##                print(v == 0)
                o.add(v == 0)

    ###Constraint for the first step:  we move to the first location
    for i in range(1,n_robots+1):
        v1 = varPos['pos_{}_1'.format(i)]
        v2 = varD['d_{}_{}'.format(i,n_machines)]
        c2 = []
        for k in range(1,n_machines+1):
            v2 = varD['d_{}_1'.format(i)]
            v3 = distances['dist_0_{}'.format(k)]
            c2.append(And(v1 == k, v2 == v3))
        o.add(Or(c2))

    ##Constraint  for successive steps: if a robot moves, d is incremented, otherwise it is not
    c = []
    for i in range(1,n_robots+1):
        for j in range(2,n_machines+1):
            src = varPos['pos_{}_{}'.format(i,j-1)]
            dst = varPos['pos_{}_{}'.format(i,j)]

            dist_src = varD['d_{}_{}'.format(i,j-1)]
            dist_dst = varD['d_{}_{}'.format(i,j)]
            c.append(dst != 0)
            c.append(dst != -1)
            c.append(dst != -2)
            c.append(dst != -3)

            c.append(Or(Not(src == -4), And(dst == -4, dist_dst == dist_src)))
            c.append(Or(Not(dst == -4), dist_dst == dist_src))
            for k in range(1,n_machines+1):
                c.append(Or(Not(src == k), Not(dst == k)))
                for l in range(1,n_machines+1):
                    if k == l: continue
                    if('dist_{}_{}'.format(k,l) in [keys for keys in distances]):
                        v5 = distances['dist_{}_{}'.format(k,l)]
                        c.append(Or(Not(src == k), Not(dst == l), dist_dst == dist_src + v5))
                    elif('dist_{}_{}'.format(l,k) in [keys for keys in distances]):
                        v5 = distances['dist_{}_{}'.format(l,k)]
                        c.append(Or(Not(src == k), Not(dst == l), dist_dst == dist_src + v5))
    o.add(And(c))

#    ## A robot cannot visit the same machine twice
    c3 = []
    for i in range(1,n_machines+1):
        c4 = []
        for j in range(1,n_robots+1):
            for k in range(1,n_machines+1):
                v1 = varPos['pos_{}_{}'.format(j,k)]
                c1 = v1 == i
                c4.append(c1)
                c2 = []
                for u in range(1,n_robots+1):
                    for v in range(1,n_machines+1):
                        if u == j and v == k: continue
                        v2 = varPos['pos_{}_{}'.format(u,v)]
                        c2.append(Not(v2 == i))
                c3.append(Or(Not(c1), And(c2)))
        o.add(Or(c4))
    o.add(And(c3))

    for i in range(1,n_machines+1):
        c = []
        for j in range(1,n_robots+1):
            for k in range(1,n_machines+1):
                v = varPos['pos_{}_{}'.format(j,k)]
                c.append(v == i)
        o.add(Or(c))

    ## Constraints encoding max d_i_M:
    for i in range(1,n_robots+1):
        c1 = varM['m_{}'.format(i)] == False
        c2 = varM['m_{}'.format(i)] == True
        c3 = []
        for j in range(1,n_robots+1):
            if(not(j==i)):
                v1 = varD['d_{}_{}'.format(j,n_machines)]
                v2 = varD['d_{}_{}'.format(i,n_machines)]
                c3.append(v1 < v2)
##        print(Or(c1,And(c2,And(c3))))
        o.add(Or(c1,And(c2,And(c3))))

########################################################################################################
## Simple optimization adding only upper bounds#
################################################
##def myOptimize(o, varD, n_machines):
##    res = sat
##    v1 = varD['d_1_{}'.format(n_machines)]
##    v2 = varD['d_2_{}'.format(n_machines)]
##    v3 = varD['d_3_{}'.format(n_machines)]
##
##
##    o.add(v1+v2+v3 > 0)
##
##    while(res==sat):
##        res = o.check()
##        print(res)
##        if(res == unsat):
##            return 'Unsat: Can\'t go below {}'.format(totSum)
##        m = o.model()
##        totSum = 0
##        for k,v in varD.items():
##            if('_{}'.format(n_machines) in k):
##                totSum+=float(str(m[v]))
##
##        o.add(v1+v2+v3 < totSum)
##
##
########################################################################################################
## Optimization using binary search#
####################################
##
##def binarySearchOptimize(o,varD,n_machines):
##    epsilon = 1
##    res = o.check()
##    if(res == unsat):
##        return 'unsat'
##    elif(res == sat):
##        m = o.model()
##        low = 0
##        high=0
##        for k,v in varD.items():
##            if '_{}'.format(n_machines) in k:
##                high+=float(str(m[v]))
##        sol = high
##        print('initial bounds: {} {}'.format(low,high))
##        tmp = binaryOptimize(o,low,high,sol,varD, n_machines,epsilon)
##        return tmp, o.model()
##
##
##def binaryOptimize(o,low, high, sol,varD, n_machines,epsilon):
##    if((high - low) < epsilon):
##        res = o.check()
##        assert res == sat
##        return sol;
##    else:
####        mid = round((high + 2*low)/3, 2)
##        mid = round((2*high+low)/3,2)
##        o.push()
##
##        v1 = varD['d_1_{}'.format(n_machines)]
##        v2 = varD['d_2_{}'.format(n_machines)]
##        v3 = varD['d_3_{}'.format(n_machines)]
##
##
##        o.add(v1+v2+v3 >= low, v1+v2+v3 <= mid)
##        start = time.time()
##        print('Check sum >= {} & sum <= {}'.format(low,mid))
##        res = o.check()
##        elapsed = time.time() - start
##        print('Time taken to check: {}'.format(elapsed))
##        print('Result: {}'.format(res))
##        if(res == sat):
##            m = o.model()
##            sol = float(str(m[v1])) + float(str(m[v2])) + float(str(m[v3]))
##            high = sol
##            print('Move to lower half: {} {}'.format(low,high))
##            return binaryOptimize(o,low,high,sol,varD, n_machines,epsilon)
##        elif(res == unsat):
##            o.pop()
##            print('Move to upper half: {} {}'.format(mid,high))
##            return binaryOptimize(o,mid,high,sol,varD, n_machines,epsilon)
##
##
########################################################################################################
## Optimize using Z3 API#
#########################
def z3Optimize(o, varD, varM, n_machines):
    v1 = varD['d_1_{}'.format(n_machines)]
    v2 = varD['d_2_{}'.format(n_machines)]
    v3 = varD['d_3_{}'.format(n_machines)]

    m1 = varM['m_1']
    m2 = varM['m_2']
    m3 = varM['m_3']

    o.minimize(m1*v1 + m2*v2 + m3*v3)

    o.minimize(v1+v2+v3)

########################################################################################################
## Dump smt2 encoding to file#
##############################
def toSMT2Benchmark(o, status = "", name="visit cyan input", logic=""):
  v = (Ast * 0)()
  a = o.assertions()
  if len(a) == 0:
      o = BoolVal(True)
  else:
      o = And(*a)

  return Z3_benchmark_to_smtlib_string(o.ctx_ref(), name, logic, status, "", 0, v, o.as_ast())

########################################################################################################
## Main#
########

def main():

    ## Indices of Machines to be used in the encoding
##    indices = { '1' : 'C-BS-I' , '2' :'C-CS1-I', '3':'C-CS2-I', '4':'C-DS-I', '5':'C-RS1-I' , '6':'C-RS2-I',  '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}
##    indices = { '1' : 'C-BS-I' , '2' :'C-CS1-I', '3':'C-CS2-I', '4':'C-DS-I', '5':'C-RS1-I' , '6':'C-RS2-I', '7':'C-CS2-O','8':'C-DS-O', '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}
    indices = { '1' : 'C-BS-I' , '2' :'C-BS-O', '3':'C-CS1-I', '4':'C-CS1-O', '5':'C-CS2-I', '6':'C-CS2-O', '7':'C-DS-I', '8':'C-DS-O', '9':'C-RS1-I', '10':'C-RS1-O', '11': 'C-RS2-I', '12':'C-RS2-O', '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}
    inverted_indices = dict((v, k) for k, v in indices.items())

    ## Number of robots and machines in the arena
    n_robots = 3
    n_machines = len(indices) - 4


    ## Create solver instance
    o = Optimize()
    set_option(rational_to_decimal=True)


    ################################################
    ##TO be replaced with iterations over all files#
    ################################################
    navgraphFileName = './navgraph-costs-001.csv'

    varPos, varD, varM = declareVariables(n_robots,n_machines)

    distances = loadDistances(navgraphFileName,inverted_indices,n_machines)

    addConstraints(o,varPos, varD, varM ,distances,n_robots,n_machines)

    ####################################
    ## Using binary search optimization#
    ####################################

##    start = time.time()
##    totalCost, model = binarySearchOptimize(o,varD,n_machines)
##    #sol = 0
##    #binaryOptimize(o, 21.51, 24.3, sol, varD, n_machines, 1)
##    end = time.time()
##    print('Scheduling computed:')
##    print(model)
##    print('Total cost: {} \nElapsed Time: {}'.format(totalCost, end-start))

##  ###########################
##  ## Using optimization API##
##  ###########################
##    z3Optimize(o, varD, varM,n_machines)
##    start = time.time()
##    print(o.check())
##    end = time.time()
##    m = o.model()
##
##    totCost=0
##    for k,v in varD.items():
##        if '_{}'.format(n_machines) in k:
##            totCost+=float(str(m[v]))
##    print('Optimal cost: {}. Time: {}'.format(totCost,end-start))
##
##    for k,v in varPos.items():
##        print(k,m[v])
##
##
##    ##############################
##    ## Using simple optimization##
##    ##############################
##    start = time.time()
##    result = myOptimize(o, varD, n_machines)
##    end = time.time()
##    print('Optimal Cost: {}. Time: {}'.format(result, end-start))


##########################################################################################
##    ###############################
##    ## Write to smt2 format
##    ##############################
##
    fo = open('smt_benchmark.smt2', 'w')
    fo.write(toSMT2Benchmark(o))
    fo.close()

    #minimize total sum
    fo = open('smt_benchmark.smt2','a')
    con = '(minimize (+ '
    for i in range(1,n_robots+1):
        con+=' d_{}_{}'.format(i,n_machines)
    con+='))\n'
    fo.write(con)


    con='(get-value (  '
    for key in varPos:
        if not('-' in key):
            con+=' {}'.format(key)
    con+='))\n'
    fo.write(con)
    fo.close()

######################################################################################################


if __name__ == "__main__":
    main()
