"""
kltl.py
Description:
    A module for KLTL formulae.
"""

from typing import List, Tuple, Union

from kltl.types import AtomicProposition

from kltl.systems.pts.parametric_transition_system import ParametricTransitionSystem

# Define Operators
NextSymbol = 'X'
UntilSymbol = 'U'
AlwaysSymbol = 'G'
EventuallySymbol = 'F'
AndSymbol = 'A'
OrSymbol = 'O'
NotSymbol = 'N'
KnowsSymbol = 'K'

Symbols = [NextSymbol, UntilSymbol, AlwaysSymbol, EventuallySymbol, AndSymbol, OrSymbol, NotSymbol, KnowsSymbol] # Could make this/symbol declarations a dictionary

class KLTLFormula:
    def __init__(self, ap_or_operator: Union[AtomicProposition, 'KLTLFormula'], subformulae: List['KLTLFormula'] = []):
        self.ap_or_operator = ap_or_operator
        self.subformulae = subformulae
    def __str__(self):
        return f"KLTL Formula:\nAP/Operator: {self.ap_or_operator}\nSubformulae: {self.subformulae}"



""" Functions """
def Next(phi: Union[AtomicProposition, KLTLFormula]) -> KLTLFormula:
    return KLTLFormula(NextSymbol, [phi])

def Until(phi1: Union[AtomicProposition, KLTLFormula], phi2: Union[AtomicProposition, KLTLFormula]) -> KLTLFormula:
    return KLTLFormula(UntilSymbol, [phi1, phi2])

def Always(phi: Union[AtomicProposition, KLTLFormula]) -> KLTLFormula:
    return KLTLFormula(AlwaysSymbol, [phi])

def Eventually(phi: Union[AtomicProposition, KLTLFormula]) -> KLTLFormula:
    return KLTLFormula(EventuallySymbol, [phi])

def And(phi1: Union[AtomicProposition, KLTLFormula], phi2: Union[AtomicProposition, KLTLFormula], *args: Union[AtomicProposition, KLTLFormula]):
    phis = [phi1, phi2]
    phis.extend(args)
    return KLTLFormula(AndSymbol, phis)

def Or(phi1: Union[AtomicProposition, KLTLFormula], phi2: Union[AtomicProposition, KLTLFormula], *args: Union[AtomicProposition, KLTLFormula]):
    phis = [phi1, phi2]
    phis.extend(args)
    return KLTLFormula(OrSymbol, phis)

def Not(phi: Union[AtomicProposition, KLTLFormula]) -> KLTLFormula:
    return KLTLFormula(NotSymbol, [phi])

def Knows(phi: Union[AtomicProposition, KLTLFormula]) -> KLTLFormula:
    return KLTLFormula(KnowsSymbol, [phi])

def satisfies_next(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert len(trace_in) >= 2, '"Next" operator not applicable to length 1 trace'
    assert formula_in.ap_or_operator == NextSymbol, 'KLTL formula must begin with "Next" operator to check for satisfaction thereof'
    
    phis = formula_in.subformulae
            
    for phi in phis:
        if not eval(phi, trace_in[1:]): return False
    return True
        
def satisfies_until(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert len(trace_in) >= 2
    assert formula_in.ap_or_operator == UntilSymbol, 'KLTL formula must begin with "Until" operator to check for satisfaction thereof'
    
    ap1, ap2 = formula_in.subformulae

    for i in range(len(trace_in)):
        if eval(ap1, trace_in[i:]) and not eval(ap2, trace_in[i:]): continue
        elif i > 0 and eval(ap2, trace_in[i:]): return True
        else: return False
            
def satisfies_always(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert formula_in.ap_or_operator == AlwaysSymbol, 'KLTL formula must begin with "Always" operator to check for satisfaction thereof'
    
    phis = formula_in.subformulae
    
    for t in trace_in:
        for p in phis:
            if not eval(p, trace_in): return False
    return True
    
def satisfies_eventually(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert formula_in.ap_or_operator == EventuallySymbol, 'KLTL formula must begin with "Eventually" operator to check for satisfaction thereof'
    
    phis = formula_in.subformulae
    satisfied = {phi:False for phi in phis}
    
    for i in range(len(trace_in)):
        for phi in phis:
            if eval(phi, trace_in[i:]): satisfied[phi] = True
    return all(satisfied.values())

def satisfies_and(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert formula_in.ap_or_operator == AndSymbol, 'KLTL formula must begin with "And" operator to check for satisfaction thereof'
    
    phis = formula_in.subformulae
    
    for phi in phis:
        if not eval(phi, trace_in): return False
    return True

def satisfies_or(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert formula_in.ap_or_operator == OrSymbol, 'KLTL formula must begin with "Or" operator to check for satisfaction thereof'
    
    phis = formula_in.subformulae
    
    for phi in phis:
        if eval(phi, trace_in): return True
    return False

def satisfies_not(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert formula_in.ap_or_operator == NotSymbol, 'KLTL formula must begin with "Not" operator to check for satisfaction thereof'
    return not eval(formula_in, trace_in)

def satisfies_knows(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    assert formula_in.ap_or_operator == KnowsSymbol, 'KLTL formula must begin with "Knowledge" operator to check for satisfaction thereof'
    
    potential_traces = []
    output_map = {}
    
    # Create set of all potential traces
    for pair in system_in.output_map:
        y = pair[-1]
        s = pair[0]
        if y in output_map: output_map[y].append(s)
        else: output_map[y] = [s]
        
    for y in trace_in:
        temp = []
        for s in output_map[y]:
            for trace in potential_traces:
                temp.append(trace + [s])
        potential_traces = temp
        
    for trace in potential_traces:
        if not eval(formula_in.subformulae, trace): return False
    return True

function_map = {
    
    # NextSymbol:satisfies_next,
    UntilSymbol:satisfies_until,
    AlwaysSymbol:satisfies_always,
    EventuallySymbol:satisfies_eventually,
    AndSymbol:satisfies_and,
    OrSymbol:satisfies_or,
    NotSymbol:satisfies_not,
    KnowsSymbol:satisfies_knows
    
}

# The "eval" method is an internal method.
def eval(phi, trace_in, system_in:ParametricTransitionSystem):
        if type(phi) == str:
            return kltl_evaluate(KLTLFormula(phi, []), trace_in, system_in)
        if type(phi) != str:
            return kltl_evaluate(phi, trace_in, system_in)

def kltl_evaluate(formula_in:KLTLFormula, trace_in:List[List[str]], system_in:ParametricTransitionSystem):
    
    ap_or_op = formula_in.ap_or_operator
    sub = formula_in.subformulae
    
    if ap_or_op not in Symbols:
        assert sub == [], 'KLTL formula cannot contain more than one AP without an operator'
        
        if ap_or_op in trace_in[0]: return True
        return False
    
    elif ap_or_op == NextSymbol:    
        return satisfies_next(formula_in, trace_in, system_in)
    
    elif ap_or_op in Symbols and all(isinstance(s, str) for s in sub):
        return function_map[ap_or_op](formula_in, trace_in, system_in)
    elif ap_or_op in Symbols and sub != []:
        return function_map[ap_or_op](formula_in, trace_in, system_in)
