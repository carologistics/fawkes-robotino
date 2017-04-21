from z3 import *
import csv
import time
import math



########################################################################################################
## Encode int for machine position in binary #
##############################################

def toBinary(k, n_bits):
    return format(k, '0{}b'.format(n_bits))
    

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
def declareVariables(n_robots,n_machines, n_bits):

    # Variables pos_i_j_k
    varPos = {}

    for i in range(1,n_robots+1):
        for j in range(-3,n_machines+1):
            for k in range(-4,n_machines+1):
                varPos['pos_{}_{}_{}'.format(i,j,k)] = Bool('pos_{}_{}_{}'.format(i,j,k))

    # Variables for boolean encoding of k, i.e., k = 1 -> 0001
    varBoolPos = {}

    for i in range(1,n_robots+1):
        for j in range(-3,n_machines+1):
            for k in range(0,n_bits):
                varBoolPos['p_{}_{}_{}'.format(i,j,k)] = Bool('p_{}_{}_{}'.format(i,j,k))


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

    return varPos,varBoolPos,varD,varM


########################################################################################################
## Adding constraints to the solver#
####################################
def addConstraints(o,varPos, varBoolPos, varD, varM,distances,n_robots,n_machines, n_bits):

    ##Assert d_i_0 must be 0 (no cost for the first action)
    for k,v in varD.items():
        if('_0' in k):
##            print(v == 0)
            o.add(v == 0)

    ## Assert d_i_j <= d_i_M
    for i in range(1,n_robots+1):
        for j in range(1,n_machines):
            o.add(varD['d_{}_{}'.format(i,j)] <= varD['d_{}_{}'.format(i,n_machines)])

    ## Assert d_i_j >= min(distances)
    for i in range(1,n_robots+1):
        for j in range(1,n_machines):
            o.add(varD['d_{}_{}'.format(i,j)] >= min(distances.values()) )

    ## Distance between two steps <= max distance
    for i in range(1,n_robots+1):
        for j in range(1,n_machines):
            o.add(varD['d_{}_{}'.format(i,j)] - varD['d_{}_{}'.format(i,j-1)] <= max(distances.values()))



    ##Assert some constraints for initial positions
    for k,v in varPos.items():
        if('pos_{}_{}_{}'.format(1,0,0) in k):  
            o.add(v)
        if('pos_{}_{}_{}'.format(2,0,0) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(3,0,0) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(1,-3,-1) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(1,-2,-1) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(1,-1,-1) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(2,-3,-2) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(2,-2,-2) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(2,-1,-1) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(3,-3,-3) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(3,-2,-2) in k):
            o.add(v)
        if('pos_{}_{}_{}'.format(3,-1,-1) in k):
            o.add(v)


    #set first step of robot 1 to the closest machine
    minVal, minKey = max(distances.values()), 0
    for k,v  in distances.items():
        if('dist_{}_'.format(0) in k):
            if v < minVal:
                minKey = k
                minVal = v

    s = minKey.split('_')
    
    o.add(And(varPos['pos_{}_{}_{}'.format(1,1,s[len(s)-1])], varD['d_{}_{}'.format(1,1)] == minVal ))
  


##  ##Constraint  for successive steps: if a robot moves, d is incremented, otherwise it is not
    for i in range(1,n_robots+1):
        for j in range(1,n_machines+1):
            c = []
            
            dist_src = varD['d_{}_{}'.format(i,j-1)]
            dist_dst = varD['d_{}_{}'.format(i,j)]
            dist_m = varD['d_{}_{}'.format(i,n_machines)]

            v1 = varPos['pos_{}_{}_{}'.format(i,j,-4)]
            
            for k in range(0,n_machines+1):
                src = varPos['pos_{}_{}_{}'.format(i,j-1,k)]
                for l in range(1,n_machines+1):
                    if k == l: continue
                    
                    dst = varPos['pos_{}_{}_{}'.format(i,j,l)]
                        
                    if('dist_{}_{}'.format(k,l) in [keys for keys in distances]):
                        dist = distances['dist_{}_{}'.format(k,l)]
                        c.append(And(src, dst, dist_dst == dist_src + dist))
                    elif('dist_{}_{}'.format(l,k) in [keys for keys in distances]):
                        dist = distances['dist_{}_{}'.format(l,k)]
                        c.append(And(src, dst, dist_dst == dist_src + dist))
            o.add(Or(And(v1,dist_m == dist_src),Or(c)))

                        

          


    ## A robot cannot visit the same machine twice
    for k in range(1,n_machines+1):
        c3 = []
        for i in range(1,n_robots+1):
            for j in range(1,n_machines+1):
                v1 = varPos['pos_{}_{}_{}'.format(i,j,k)]
                c2 = []
                for u in range(1,n_robots+1):
                    for v in range(1,n_machines+1):
                        if u == i and v == j: continue
                        v2 = varPos['pos_{}_{}_{}'.format(u,v,k)]
                        c2.append(Not(v2))
                c3.append(And(v1, And(c2)))
        o.add(Or(c3))
  


    ## Constraints encoding max d_i_M:
    c1 = []
    for i in range(1,n_robots+1):
        v3 = varM['m_{}'.format(i)] 
        c2 = []
        for j in range(1,n_robots+1):
            if(not(j==i)):
                v1 = varD['d_{}_{}'.format(j,n_machines)]
                v2 = varD['d_{}_{}'.format(i,n_machines)]
                c2.append(v1 <= v2)
        c1.append(And(v3,And(c2)))
    o.add(Or(c1))

    ## Constraints to encode the relation pos_i_j_k <-> (... binary encoding ...)
    for i in range(1,n_robots+1):
        for j in range(-3,n_machines+1):
            for k in range(-4,n_machines+1):
                v1 = varPos['pos_{}_{}_{}'.format(i,j,k)]

                if(k == -4):
                    s = toBinary(n_machines+4,n_bits)
                elif(k == -3):
                    s = toBinary(n_machines+3,n_bits)
                elif(k == -2):
                    s = toBinary(n_machines+2,n_bits)
                elif(k == -1):
                    s = toBinary(n_machines+1,n_bits)
                else:
                    s = toBinary(k,n_bits)
                    
                c = []
                ind = 0
                
                for l in range(len(s)-1, -1, -1):
                    v2 = varBoolPos['p_{}_{}_{}'.format(i,j,ind)]
                    if (int(s[l]) == 1):
                        c.append(v2)
                    if(int(s[l]) == 0):
                        c.append(Not(v2))
                    ind = ind+1

                o.add(And(Or(Not(v1), And(c)),Or(Not(And(c)),v1)))


   
    

########################################################################################################
## Simple optimization adding only upper bounds#
################################################
def simpleOptimize(o, varD, varM, n_machines, n_robots, distances):
    res = sat
    v1 = varD['d_1_{}'.format(n_machines)]
    v2 = varD['d_2_{}'.format(n_machines)]
    v3 = varD['d_3_{}'.format(n_machines)]

    m1 = varM['m_1']
    m2 = varM['m_2']
    m3 = varM['m_3']

    o.add(v1+v2+v3 >= min(distances.values())*n_machines)

    while(res==sat):
        start = time.time()
        res = o.check()
        end = time.time()
        print('Intermediate solving time: {}'.format(end-start))
        if(res == unsat):
            return 'Can\'t go below {}'.format(totSum)
        m = o.model()
        totSum = 0
        maxSum = 0

        for i in range(1,n_robots+1):
            tmp=float(str(m[varD['d_{}_{}'.format(i,n_machines)]]))
            if(m[varM['m_{}'.format(i)]]):
                maxSum+=tmp
            totSum+=tmp
        print('Total sum: {}'.format(totSum))
        o.add(m1*v1+m2*v2+m3*v3 < maxSum)
        o.add(v1+v2+v3 < totSum)

########################################################################################################
## Optimization adding upper bounds and ruling out paths#
#########################################################
def ruleOutOptimize(o, varPos, varD, n_machines, n_robots, dist):
    oldSum = sum(dist.values())
    totSum = oldSum
    res = sat
    v1 = varD['d_1_{}'.format(n_machines)]
    v2 = varD['d_2_{}'.format(n_machines)]
    v3 = varD['d_3_{}'.format(n_machines)]

    o.add(v1+v2+v3 < totSum)
    
    while(res == sat):
        
        start = time.time()
        res = o.check()
        end = time.time()
        print('Intermediate solving time: {}'.format(end-start))
        if(res == unsat):
            return ('Cannot solve below {}'.format(totSum))
        m = o.model()
        totSum = 0
        for k,v in varD.items():
            if('_{}'.format(n_machines) in k):
                totSum+=float(str(m[v]))
        if(totSum <= oldSum):
            D = computeSetD(m,varPos,n_robots,n_machines)
            c = []
            for el in D:
                key = el.split(' ')
                for i in range(1,n_robots+1):
                    for j in range(1,n_machines):
                        pos1 = varPos['pos_{}_{}'.format(i,j)]
                        pos2 = varPos['pos_{}_{}'.format(i,j+1)]
                        c.append(And(Not(pos1 == int(key[0])), Not(pos2 == int(key[1]))))
            o.add(Or(c))
            o.add(v1+v2+v3 < totSum)
            oldSum = totSum
    
    
def computeSetD(m,varPos,n_robots,n_machines):
    D = []

    for i in range(1,n_robots+1):
        for j in range(1,n_machines):
            k_1 = str(m[varPos['pos_{}_{}'.format(i,j)]])
            k_2 = str(m[varPos['pos_{}_{}'.format(i,j+1)]])
            D.append('{} {}'.format(k_1,k_2))

    return D


########################################################################################################
## Optimization using binary search#
####################################

def binarySearchOptimize(o,varD,n_machines):
    epsilon = 1
    res = o.check()
    if(res == unsat):
        return 'unsat'
    elif(res == sat):
        m = o.model()
        low = 0
        high=0
        for k,v in varD.items():
            if '_{}'.format(n_machines) in k:
                high+=float(str(m[v]))
        sol = high
        print('initial bounds: {} {}'.format(low,high))
        tmp = binaryOptimize(o,low,high,sol,varD, n_machines,epsilon)
        return tmp, o.model()


def binaryOptimize(o,low, high, sol,varD, n_machines,epsilon):
    if((high - low) < epsilon):
        res = o.check()
        assert res == sat
        return sol;
    else:
##        mid = round((high + 2*low)/3, 2)
        mid = round((2*high+low)/3,2)
        o.push()

        v1 = varD['d_1_{}'.format(n_machines)]
        v2 = varD['d_2_{}'.format(n_machines)]
        v3 = varD['d_3_{}'.format(n_machines)]


        o.add(v1+v2+v3 >= low, v1+v2+v3 <= mid)
        start = time.time()
        print('Check sum >= {} & sum <= {}'.format(low,mid))
        res = o.check()
        elapsed = time.time() - start
        print('Time taken to check: {}'.format(elapsed))
        print('Result: {}'.format(res))
        if(res == sat):
            m = o.model()
            sol = float(str(m[v1])) + float(str(m[v2])) + float(str(m[v3]))
            high = sol
            print('Move to lower half: {} {}'.format(low,high))
            return binaryOptimize(o,low,high,sol,varD, n_machines,epsilon)
        elif(res == unsat):
            o.pop()
            print('Move to upper half: {} {}'.format(mid,high))
            return binaryOptimize(o,mid,high,sol,varD, n_machines,epsilon)


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
def toSMT2Benchmark(o, status = "unknown", name="visit cyan input", logic=""):
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

    number = 6 
    ## Indices of Machines to be used in the encoding
    if number == 6:
        indices = { '1' : 'C-BS-I' , '2' :'C-CS1-I', '3':'C-CS2-I', '4':'C-DS-I', '5':'C-RS1-I' , '6':'C-RS2-I',  '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}
    elif number == 8:
        indices = { '1' : 'C-BS-I' , '2' :'C-CS1-I', '3':'C-CS2-I', '4':'C-DS-I', '5':'C-RS1-I' , '6':'C-RS2-I', '7':'C-CS2-O','8':'C-DS-O', '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}
    elif number == 10:
        indices = { '1' : 'C-BS-I' , '2' :'C-BS-O', '3':'C-CS1-I', '4':'C-CS1-O', '5':'C-CS2-I', '6':'C-CS2-O', '7':'C-DS-I', '8':'C-DS-O', '9':'C-RS1-I', '10':'C-RS1-O', '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}
    elif number == 12:
        indices = { '1' : 'C-BS-I' , '2' :'C-BS-O', '3':'C-CS1-I', '4':'C-CS1-O', '5':'C-CS2-I', '6':'C-CS2-O', '7':'C-DS-I', '8':'C-DS-O', '9':'C-RS1-I', '10':'C-RS1-O', '11': 'C-RS2-I', '12':'C-RS2-O', '0':'C-ins-in', '-1':'p1', '-2':'p2', '-3':'p3'}

    inverted_indices = dict((v, k) for k, v in indices.items())

    ## Number of robots and machines in the arena
    n_robots = 3
    n_machines = len(indices) - 4

    n_bits = int(math.ceil(math.log(len(indices)+1,2)))

    ## Create solver instance
    o = Optimize()
    set_option(rational_to_decimal=True)


    ################################################
    ##TO be replaced with iterations over all files#
    ################################################
    navgraphFileName = './navgraph-costs-050.csv'

    varPos, varBoolPos, varD, varM = declareVariables(n_robots,n_machines, n_bits)

    distances = loadDistances(navgraphFileName,inverted_indices,n_machines)

    addConstraints(o,varPos, varBoolPos, varD, varM ,distances,n_robots,n_machines, n_bits)

    ########################################################
    # Binary Search: 1, Z3 : 2, Simple Opt: 3, Rule Out: 4 #
    ########################################################
    opt = 2
    write = 0 #whether we want to dump the smtlib file or not

    ####################################
    ## Using binary search optimization#
    ####################################
    if(opt == 1):
        start = time.time()
        totalCost, model = binarySearchOptimize(o,varD,n_machines)
        #sol = 0
        #binaryOptimize(o, 21.51, 24.3, sol, varD, n_machines, 1)
        end = time.time()
        print('Scheduling computed:')
        print(model)
        print('Total cost: {} \nElapsed Time: {}'.format(totalCost, end-start))

    ###########################
    ## Using optimization API##
    ###########################
    if(opt == 2):
        z3Optimize(o, varD, varM, n_machines)
        print('Calling Z3 optimizer')
        start = time.time()
        res = o.check()
        end = time.time()
        if (res == sat):
            m = o.model()
            totCost=0
            for k,v in varD.items():
                if '_{}'.format(n_machines) in k:
                    totCost+=float(str(m[v]))
            print('Optimal cost: {}. Time: {}'.format(totCost,end-start))
            ##
            
##            for k,v in varPos.items():
##                if(m[v]):
##                    print(k,m[v])
##
##            for k,v in varD.items():
##                print(k,m[v])
        else:
            print('unsat')


    ##############################
    ## Using simple optimization##
    ##############################

    if(opt == 3):
        print('Calling Simple Optimizer')
        start = time.time()
        result = simpleOptimize(o, varD, varM, n_machines, n_robots, distances)
        end = time.time()
        print('Optimal Cost: {}. Time: {}'.format(result, end-start))

    ########################################
    ## Using optimization ruling out paths##
    ########################################
    if(opt == 4):                 
        print('Calling Rule-Out Optimizer')
        start = time.time()
        result = ruleOutOptimize(o, varPos, varD, n_machines, n_robots, distances)
        end = time.time()
        print('Optimal Cost: {}. Time: {}'.format(result, end-start))


##########################################################################################
##    ###############################
##    ## Write to smt2 format
##    ##############################
    if (write == 1):
        print('Start writing...')
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

        #minimize weighted sum
        fo = open('smt_benchmark.smt2','a')
        con = '(minimize (+ '
        for i in range(1,n_robots+1):
            con+=' ( * d_{}_{} m_{})'.format(i,n_machines,i)
        con+='))\n'
        fo.write(con)


        con='(get-value (  '
        for key in varPos:
            if(not('-' in key)):
                con+=' {}'.format(key)
        con+='))\n'
        fo.write(con)
        fo.close()

######################################################################################################


if __name__ == "__main__":
    main()
