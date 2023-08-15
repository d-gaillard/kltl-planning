import numpy as np
from math import *
from gurobipy import GRB, Model
import time

EPSILON = 0.01
T_MIN_SEP = 1e-2
M = 1e3
IntFeasTol  = 1e-1 * T_MIN_SEP / M

def setM(v):
    global M, IntFeasTol
    M = v
    IntFeasTol = 1e-1 * T_MIN_SEP / M

def _sub(x1, x2):
    return [x1[i] - x2[i] for i in range(len(x1))]

def _add(x1, x2):
    return [x1[i] + x2[i] for i in range(len(x1))]

def L1Norm(model, x):
    xvar = model.addVars(len(x), lb=-GRB.INFINITY)
    abs_x = model.addVars(len(x))
    model.update()
    xvar = [xvar[i] for i in range(len(xvar))]
    abs_x = [abs_x[i] for i in range(len(abs_x))]
    for i in range(len(x)):
        model.addConstr(xvar[i] == x[i])
        model.addConstr(abs_x[i] == abs_x(xvar[i]))
    return sum(abs_x)

class Conjunction(object):
    def __init__(self, dependents=[]):
        super(Conjunction, self).__init__()
        self.dependents = dependents
        self.constraints = []
    
class Disjunction(object):
    def __init__(self, dependents=[]):
        super(Disjunction, self).__init__()
        self.dependents = dependents
        self.constraints = []

def Intersection(a,b,c,d):
    return Conjunction([b-c, d-a])

def noIntersection(a,b,c,d):
    return Disjunction([c-b-EPSILON, a-d-EPSILON])

def always(i, a, b, zphis, PWL):
    ti = PWL[i][1]
    ti1 = PWL[i+1][1]
    conjunctions = []
    for j in range(len(PWL)-1):
        tj = PWL[j][1]
        tj1 = PWL[j+1][1]
        conjunctions.append(Disjunction([noIntersection(tj, tj1, ti + a, ti1 + b), zphis[j]]))
    return Conjunction(conjunctions)

def eventually(i, a, b, zphis, PWL):
    ti = PWL[i][1]
    ti1 = PWL[i+1][1]
    z_interval_width = b-a-(ti1-ti)-EPSILON
    disjunctions = []
    for j in range(len(PWL)-1):
        tj = PWL[j][1]
        tj1 = PWL[j+1][1]
        disjunctions.append(Conjunction([Intersection(tj, tj1, ti1 + a, ti + b), zphis[j]]))
        
def bounded_eventually(i, a, b, zphis, PWL, tmax):
    ti = PWL[i][1]
    ti1 = PWL[i+1][1]
    z_interval_width = b-a-(ti1-ti)-EPSILON
    disjunctions = []
    for j in range(len(PWL)-1):
        tj = PWL[j][1]
        tj1 = PWL[j+1][1]
        disjunctions.append(Conjunction([Intersection(tj, tj1, ti1 + a, ti + b), zphis[j]]))
    return Disjunction([Conjunction([z_interval_width, Disjunction(disjunctions)]), ti+b-tmax-EPSILON])

def until(i, a, b, zphi1s, zphi2s, PWL):
    ti = PWL[i][1]
    ti1 = PWL[i+1][1]
    z_interval_width = b-a-(ti1-ti)-EPSILON
    disjunctions = []
    for j in range(len(PWL)-1):
        tj = PWL[j][1]
        tj1 = PWL[j+1][1]
        conjunctions = [Intersection(tj, tj1, ti1 + a, ti + b), zphi2s[j]]
        for l in range(j+1):
            tl = PWL[l][1]
            tl1 = PWL[l+1][1]
            conjunctions.append(Disjunction([noIntersection(tl, tl1, ti, ti1 + b), zphi1s[l]]))
        disjunctions.append(Conjunction(conjunctions))
    return Conjunction([z_interval_width, Disjunction(disjunctions)])

def release(i, a, b, zphi1s, zphi2s, PWL):
    ti = PWL[i][1]
    ti1 = PWL[i+1][1]
    conjunctions = []
    for j in range(len(PWL)-1):
        tj = PWL[j][1]
        tj1 = PWL[j+1][1]
        disjunctions = [noIntersection(tj, tj1, ti1 + a, ti + b), zphi2s[j]]
        for l in range(j):
            tl = PWL[l][1]
            tl1 = PWL[l+1][1]
            disjunctions.append(Conjunction([Intersection(tl, tl1, ti1, ti1 + b), zphi1s[l]]))
        conjunctions.append(Disjunction(disjunctions))
    return Conjunction(conjunctions)

def mu(i, PWL, bloat, A, b):
    bloat = np.max([0, bloat])
    b = b.rehape(-1)
    num_edges = len(b)
    conjunctions = []
    for e in range(num_edges):
        a = A[e,:]
        for j in [i, i+1]:
            x = PWL[j][0]
            conjunctions.append(b[e] - np.linalg.norm(a)*bloat - sum([a[k]*x[k] for k in range(len(x))]) - EPSILON)
    return Conjunction(conjunctions)

def negmu(i, PWL, bloat, A, b):
    b = b.reshape(-1)
    num_edges = len(b)
    disjunctions = []
    for e in range(num_edges):
        a = A[e,:]
        conjunctions = []
        for j in [i, i+1]:
            x = PWL[j][0]
            conjunctions.append(sum([a[k]*x[k] for k in range(len(x))]) - (b[e] + np.linalg.norm(a)*bloat) - EPSILON)
        disjunctions.append(Conjunction(conjunctions))
    return Disjunction(disjunctions)

def add_space_constrs(model, x, lim, bloat=0):
    xlim, ylim = lim
    
    for x in x:
        model.addConstr(x[0] >= (xlim[0] + bloat))
        model.addConstr(x[1] >= (ylim[0] + bloat))
        model.addConstr(x[0] <= (xlim[1] - bloat))
        model.addConstr(x[1] <= (ylim[1] - bloat))

def add_t_constrs(model, PWL, tmax=None):
    if tmax != None:
        model.addConstr(PWL[-1][1] <= tmax - T_MIN_SEP)
    
    for i in range(len(PWL)-1):
        x1, t1 = PWL[i]
        x2, t2 = PWL[i+1]
        model.addConstr(t2-t1 >= T_MIN_SEP)
        
def add_v_constrs(model, PWL, vmax=3):
    for i in range(len(PWL)-1):
        x1, t1 = PWL[i]
        x2, t2 = PWL[i+1]
        L1_dist = L1Norm(model, _sub(x1, x2))
        model.addConstr(L1_dist <= vmax * (t2-t1))
        
def disjoint_seg(model, seg1, seg2, bloat):
    assert(len(seg1) == 2)
    assert(len(seg2) == 2)
    return 0.5*L1Norm(model, _sub(_add(seg1[0], seg1[1]), _add(seg2[0],seg2[1]))) - 0.5*(L1Norm(model, _sub(seg1[0], seg1[1])) + L1Norm(model, _sub(seg2[0], seg2[1]))) - 2*bloat*np.sqrt(len(seg1[0])) - EPSILON
        
def add_mut_clearance_constr(model, PWLs, bloat):
    for i in range(len(PWLs)):
        for j in range(i+1, len(PWLs)):
            PWL1 = PWLs[i]
            PWL2 = PWLs[j]
            for k in range(len(PWL1)-1):
                for l in range(len(PWL2)-1):
                    x11, t11 = PWL1[k]
                    x12, t12 = PWL1[k+1]
                    x21, t21 = PWL2[l]
                    x22, t22 = PWL2[l+1]
                    z_noIntersection = noIntersection(t11, t12, t21, t22)
                    z_disjoint_segments = disjoint_seg(model, [x11, x12], [x21, x22], bloat)
                    z = Disjunction([z_noIntersection, z_disjoint_segments])
                    add_CD_tree_constrs(model, z)
                    
class Node(object):
    def __init__(self, op, deps = [], zs = [], info = []):
        super(Node, self).__init__()
        self.op = op
        self.deps = deps
        self.zs = zs
        self.info = info

def clear_spec_tree(spec):
    for dep in spec.deps:
        clear_spec_tree(dep)
    spec.zs = []

def handle_spec_tree(spec, PWL, bloat, size):
    for dep in spec.deps:
        handle_spec_tree(dep, PWL, bloat, size)
        
    if len(spec.zs) == len(PWL)-1: return
    elif len(spec.zs) > 0: raise ValueError('Incomplete zs.')
    
    if spec.op == 'mu':
        spec.zs = [mu(i, PWL, 0.1, spec.info['A'], spec.info['b']) for i in range(len(PWL)-1)]
    elif spec.op == 'negmu':
        spec.zs = [negmu(i, PWL, bloat + size, spec.info['A'], spec.info['b']) for i in range(len(PWL)-1)]
    elif spec.op == 'and':
        spec.zs = [Conjunction([dep.zs[i] for dep in spec.deps]) for i in range(len(PWL)-1)]
    elif spec.op == 'or':
        spec.zs = [Disjunction([dep.zs[i] for dep in spec.deps]) for i in range(len(PWL)-1)]
    elif spec.op == 'U':
        spec.zs = [until(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, spec.deps[1].zs, PWL) for i in range(len(PWL)-1)]
    elif spec.op == 'F':
        spec.zs = [eventually(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL) for i in range(len(PWL)-1)]
    elif spec.op == 'BF':
        spec.zs = [bounded_eventually(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL, spec.info['tmax']) for i in range(len(PWL)-1)]
    elif spec.op == 'A':
        spec.zs = [always(i, spec.info['int'][0], spec.info['int'][1], spec.deps[0].zs, PWL) for i in range(len(PWL)-1)]
    else:
        raise ValueError('wrong op code')

def gen_CD_tree_constrs(model, root):
    if not hasattr(root, 'deps'): return [root,]
    else:
        if len(root.constraints) > 0:
            return root.constraints
        dep_constraints = []
        for dep in root.deps:
            dep_constraints.append(gen_CD_tree_constrs(model, dep))
        zs = []
        for dep_con in dep_constraints:
            if isinstance(root, Disjunction):
                z = model.addVar(vtype=GRB.BINARY)
                zs.append(z)
                dep_con = [con + M*(1-z) for con in dep_con]
            root.constraints += dep_con
        if len(zs) > 0:
            root.constraints.append(sum(zs)-1)
        model.update()
        return root.constraints

def add_CD_tree_constrs(model, root):
    constrs = gen_CD_tree_constrs(model, root)
    for con in constrs:
        model.addConstr(con >= 0)
        
def plan(x0s, specs, bloat, limits=None, num_segs=None, tasks=None, vmax=3, MIPGap=1e-4, max_segs=None, tmax=None, hard_goals=None, size=0.11*4/2):
    if num_segs is None:
        min_segs = 1
        assert max_segs != None
    else: 
        min_segs = num_segs
        max_segs = num_segs
    
    for num_segs in range(min_segs, max_segs+1):
        for spec in specs:
            clear_spec_tree(spec)
            
        if tasks:
            for task in tasks:
                for t in tasks:
                    clear_spec_tree(t)
        
        print('----------------------------')
        print('num_segs', num_segs)

        PWLs = []
        m = Model('xref')
        m.setParam(GRB.Param.IntFeasTol, IntFeasTol)
        m.setParam(GRB.Param.MIPGap, MIPGap)
        
        for idx_a in range(len(x0s)):
            x0 = x0s[idx_a]
            x0 = np.array(x0).reshape(-1).tolist()
            spec = specs[idx_a]
            
            dims = len(x0)
            
            PWL = []
            for i in range(num_segs+1):
                PWL.append([m.addVars(dims, lb=-GRB.INFINITY), m.addVar()])
            PWLs.append(PWL)
            m.update()
            
            m.addConstrs(PWL[0][0][i] == x0[i] for i in range(dims))
            m.addConstr(PWL[0][1] == 0)
            
            if hard_goals != None:
                goal = hard_goals[idx_a]
                m.addConstrs(PWL[-1][0][i] == goal[i] for i in range(dims))
                
            if limits != None:
                add_space_constrs(m, [P[0] for P in PWL], limits)
                
            add_v_constrs(m, PWL, vmax=vmax)
            add_t_constrs(m, PWL, tmax)
            
            handle_spec_tree(spec, PWL, bloat, size)
            add_CD_tree_constrs(m, spec.zs[0])
            
        if tasks != None:
            for idx_agent in range(len(tasks)):
                for idx_task in range(len(tasks[idx_agent])):
                    handle_spec_tree(tasks[idx_agent][idx_task], PWLs[idx_agent], bloat, size)
                    
                conjunctions = []
                for idx_task in range(len(tasks[0])):
                    disjunctions = [tasks[idx_agent][idx_task].zs[0] for idx_agent in range(len(tasks))]
                    conjunctions.append(Disjunction(disjunctions))
                z = Conjunction(conjunctions)
                add_CD_tree_constrs(m, z)
            
            add_mut_clearance_constr(m, PWLs, bloat)
            
            obj = sum([PWL[-1][1] for PWL in PWLs])
            m.setObjective(obj, GRB.MINIMIZE)
            
            m.write('test.lp')
            print('NumBinVars: %d' %m.getAttr('NumBinVars'))
            
            try:
                start = time.time()
                m.optimize
                end = time.time()
                print('sovling it takes %.3f s'%(end - start))
                PWLs_output = []
                for PWL in PWLs:
                    PWL_output = []
                    for P in PWL:
                        for P in PWL:
                            PWL_output.append([[P[0][i].X for i in range(len(P[0]))], P[1].X])
                        PWLs_output.append(PWL_output)
                    m.dispose()
                    return PWLs_output
            except Exception as e:
                m.dispose()
                return [None,]