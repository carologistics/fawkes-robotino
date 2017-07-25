#!/usr/bin/env python3

import re


class defglobal(object):
    def __init__(self, global_assignments, globalname=''):
        self.name = globalname
        self.variables = global_assignments
    def globals_print(self):
        print('Name: '+self.name)
        for key,value in self.variables.items():
            print('\tKey: '+key+', value: '+value)
class deffacts(object):
    def __init__(self,factsname,facts,comment=''):
        self.name = factsname
        self.facts = facts
        self.comment = comment
    def facts_print(self):
        print("Name: "+ self.name+' '+self.comment)
        print("Facts:")
        for f_ in self.facts:
            print('\t'+f_)

class slot(object):
    def __init__(self,slotname,slot_type,template_attribute=None):
        self.name = slotname
        self.slot_type = slot_type
        if template_attribute is None:
            self.attributes = []
        else:
            self.attributes = template_attribute
    def print_slot(self):
        print('')
        print('\tSlot name: '+self.name)
        print('\tSlot type: '+self.slot_type)
        print('\tAttributes:')
        for attr in self.attributes:
            print('\t\t'+attr)

class deftemplate(object):
    def __init__(self,templatename,slots,comment=''):
        self.name = templatename
        self.slots = slots
        self.comment = comment
    def template_print(self):
        print('Template name: '+self.name+' '+self.comment)
        for slot_ in self.slots:
            slot_.print_slot()


class defrule(object):
    def __init__(self,rulename,LHS,RHS,comment='',salience='0'):
        self.name = rulename
        self.LHS = LHS
        self.RHS = RHS
        self.comment = comment
        if salience != '':
            self.salience = salience
        else:
            self.salience = '0'
    def rule_print(self):
        print('Name: '+self.name+' '+self.comment+'\n')
        print('Salience: '+self.salience+'\n')
        print('LHS:')
        for match in self.LHS:
            print(match)
        print('\nRHS:')
        print('\n\tassert:')
        for a in self.RHS['asserts']:
            print('\t'+a)
        print('\n\tretract:')
        for a in self.RHS['retracts']:
            print('\t'+a)
        print('\n\tmodify:')
        for a in self.RHS['modifications']:
            print('\t'+a)
        print('\n\tswitch:')
        for a in self.RHS['switch']:
            print('\t'+a)
        print('\n\tif:')
        for a in self.RHS['if']:
            print('\t'+a)
        print('\n\tfunctions:')
        for a in self.RHS['functions']:
            print('\t'+a)

class deffunction(object):
    def __init__(self,functionname,parameters,actions,comment=''):
        self.name = functionname
        self.parameters = parameters
        self.actions = actions
        self.comment = comment

    def function_print(self):
        print('Name: '+self.name+' '+self.comment)
        print('Parameter(s): '+self.parameters)
        print('Actions:')
        for a_ in self.actions:
            print(a_)
def read_clp(path):
    text = ""
    with open(path,'r') as clp:
        for line in clp:
            text =  text + re.sub('(;.*\n)','',line)
    text = re.sub('(\t|\n)','',text)
    return text

test = read_clp('/home/basti/fawkes-robotino/src/agents/rcll2017/skills.clp')
test =  test + read_clp('/home/basti/fawkes-robotino/src/agents/rcll2017/facts.clp')
test =  test + read_clp('/home/basti/fawkes-robotino/src/agents/rcll2017/globals.clp')
test =  test + read_clp('/home/basti/fawkes-robotino/src/agents/rcll2017/utils.clp')

def search_for_units(string):
    units = []
    counter=0
    left, right= 0, 0
    for i, char in enumerate(string):
        if(char=='('):
            counter += 1
            if(counter==1):
                left=i
        if(char==')'):
            counter -= 1
            if(counter==0):
                right=i
                units.append(string[left+1:right])
    return units

def strip_deftype(string,deftype):
    return re.sub('(\s*'+deftype+'\s*)','',string)
def split_comment(string):
    comment = re.search('".*?"',string)
    if comment == None:
        comment=''
    else:
        comment = comment.group()
    string = re.sub('".*?"','',string)
    return comment, string

def extract_declare(units):
    index = ''
    declare = ''
    for i, u in enumerate(units):
        if 'declare' in u:
            index = i
    if index != '' :
        declare = units.pop(index)
    return declare, units


def extract_salience(declare_statement):
    if declare_statement != '':
        salience = re.search('salience\s*(.*)\)',declare_statement)
        if salience != None:
            salience=salience.group(1)
        else:
            salience=''
    else:
        salience = ''
    return salience

def extract_keyword(keyword,units):
    units_with_keyword = []
    for i, u in enumerate(units):
        if keyword in u:
            if re.search('(?<!.)('+keyword+')',u)!= None:
                 units_with_keyword.append(units[i])
    return units_with_keyword, [x for x in units if x not in units_with_keyword]
def extract_keyword_content(keyword, units):
    content = []
    for u in units:
        temp = re.search(keyword+'\s*(.*)\)',u)
        if temp != None:
            content.append(temp.group(1))
    return content

def strip_name(string):
    name = re.search(r"([^\s]*)",string).group()
    #string = re.sub(name,'',string)
    string = string.replace(name,'')
    string = string.lstrip()
    return name, string

def parse_deffacts(unit):
    unit = strip_deftype(unit,'deffacts')
    name, unit = strip_name(unit)
    comment, unit = split_comment(unit)
    facts = search_for_units(unit)
    return deffacts(name,facts,comment)

def parse_defrule(unit):
    unit = strip_deftype(unit,'defrule')
    name, unit = strip_name(unit)
    index = unit.index('=>')
    LHS,RHS = unit[:index] , unit[(index+2):]
    comment, LHS = split_comment(LHS)
    subunits = search_for_units(LHS)
    declare, subunits = extract_declare(subunits)
    salience = extract_salience(declare)
    if declare !='':
        LHS = LHS.replace('('+declare+')','')
    new_units = []
    for unit in subunits:
        temp = []
        index = LHS.index('('+unit)
        temp.append(LHS[:index].replace('<-','').replace(' ',''))
        temp.append(unit)
        new_units.append(temp)
        LHS = LHS[index:]
        LHS = LHS.replace('('+unit+')','')
    RHS = search_for_units(RHS)
    retract, RHS = extract_keyword('retract',RHS)
    rassert, RHS = extract_keyword('assert',RHS)
    modify, RHS = extract_keyword('modify',RHS)
    printout, RHS = extract_keyword('printout',RHS)
    switch, RHS = extract_keyword('switch',RHS)
    rif, RHS = extract_keyword('if',RHS)
    functions = RHS
    rassert = [a.replace('assert ','') for a in rassert]
    retract = [a.replace('retract ','') for a in retract]
    modify = [a.replace('modify ','') for a in modify]
    RHS = {'asserts': rassert, 'retracts': retract, 'modifications': modify, 'printouts': printout, 'switch': switch, 'if': rif, 'functions':functions}
    return defrule(name,new_units,RHS,comment=comment,salience=salience)

def parse_deftemplate(unit):
    unit = strip_deftype(unit,'deftemplate')
    name, unit = strip_name(unit)
    comment, unit = split_comment(unit)
    subunits = search_for_units(unit)
    template_slots = []
    for u in subunits:
        if 'multislot' in u:
            slot_ =u.replace('multislot ','')
            slottype='multislot'
        else:
            slot_ =u.replace('slot ','')
            slottype = 'slot'
        slotname, slot_ =strip_name(slot_)
        attributes = search_for_units(slot_)
        template_slots.append(slot(slotname,slottype,attributes))
    return deftemplate(name,template_slots,comment)

def parse_deffunction(unit):
    unit = strip_deftype(unit,'deffunction')
    name, unit = strip_name(unit)
    actions = search_for_units(unit)
    for t_ in actions:
        unit = unit.replace(t_,'')
    comment, unit = split_comment(unit)
    parameters = actions.pop(0) #First element are the parameters
    return deffunction(name,parameters,actions,comment)

def parse_defglobal(unit):
    unit = strip_deftype(unit,'defglobal')
    temp = re.findall(r'(\?\*[\-A-Z0-9_]*\*\s*=\s*[\.\-0-9\w\"\?]*)',unit)
    globals_dict = {}
    for t_ in temp:
        unit =unit.replace(t_,'')
        t_=t_.replace(' ','').split('=')
        globals_dict.update({t_[0]:t_[1]})
    name = unit.replace(' ','')
    return defglobal(globals_dict,name)

def determine_unit(unit):
    if('defrule' in unit):
        print('defrule:')
        new_rule = parse_defrule(unit)
        #new_rule.rule_print()
    elif('deftemplate' in unit):
        print('deftemplate:')
        new_template = parse_deftemplate(unit)
        #new_template.template_print()
    elif('deffacts' in unit):
        print('deffacts:')
        new_facts = parse_deffacts(unit)
        #new_facts.facts_print()
    elif('deffunction' in unit):
        print('deffunction:')
        new_function = parse_deffunction(unit)
        #new_function.function_print()
    elif('defglobal' in unit):
        print('defglobal:')
        new_globals = parse_defglobal(unit)
        #new_globals.globals_print()
units = search_for_units(test)

for u in units:
    determine_unit(u)
