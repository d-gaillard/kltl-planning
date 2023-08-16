import sys
import numpy as np

from plan import plan, Node
from vis import vis

def test():
    x0 = [1, 1]

    wall_half_width = 0.1
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    _walls = []

    _walls.append(np.array([0, 0, 0, 8], dtype = np.float64))
    _walls.append(np.array([8, 8, 0, 8], dtype = np.float64))
    _walls.append(np.array([0, 8, 0, 0], dtype = np.float64))
    _walls.append(np.array([0, 8, 8, 8], dtype = np.float64))
    _walls.append(np.array([4, 8, 6, 6], dtype = np.float64))
    _walls.append(np.array([2, 2, 2, 8], dtype = np.float64))
    _walls.append(np.array([4, 4, 2, 6], dtype = np.float64))
    # _walls.append(np.array([2, 4, 4, 4], dtype = np.float64))
    # _walls.append(np.array([4, 4, 4, 6], dtype = np.float64))
    # _walls.append(np.array([6, 6, 0, 5], dtype = np.float64))

    walls = []
    for wall in _walls:
        if wall[0] == wall[1]:
            wall[0] -= wall_half_width
            wall[1] += wall_half_width
        elif wall[2] == wall[3]:
            wall[2] -= wall_half_width
            wall[3] += wall_half_width
        else:
            raise ValueError('wrong shape for axis-aligned wall')
        wall *= np.array([-1,1,-1,1])
        walls.append((A, wall))

    # _doors = []
    # _doors.append(np.array([0, 2, 4, 4], dtype = np.float64))
    # _doors.append(np.array([6, 6, 5, 6], dtype = np.float64))
    # _doors.append(np.array([6, 8, 2, 2], dtype = np.float64))
    # _doors.append(np.array([0, 2, 4, 4], dtype = np.float64))
    # _doors.append(np.array([7, 8, 6, 6], dtype = np.float64))

    # doors = []
    # for door in _doors:
    #     if door[0]==door[1]:
    #         door[0] -= wall_half_width
    #         door[1] += wall_half_width
    #     elif door[2]==door[3]:
    #         door[2] -= wall_half_width
    #         door[3] += wall_half_width
    #     else:
    #         raise ValueError('wrong shape for axis-aligned door')
    #     door *= np.array([-1,1,-1,1])
    #     doors.append((A, door))

    _disturbances = []
    _disturbances.append(np.array([1, 3], dtype = np.float64))
    _disturbances.append(np.array([3, 3], dtype = np.float64))
    # _disturbances.append(np.array([1, 1], dtype = np.float64))
    # _disturbances.append(np.array([7, 1], dtype = np.float64))
    # _disturbances.append(np.array([3, 5], dtype = np.float64))

    disturbances = []
    disturbance_half_width = 1
    for disturbance in _disturbances:
        disturbance = np.array([-(disturbance[0] - disturbance_half_width), (disturbance[0] + disturbance_half_width), -(disturbance[1] - disturbance_half_width), (disturbance[1] + disturbance_half_width)])
        disturbances.append((A, disturbance))
        
    _goals = []
    _goals.append(np.array([7,7], dtype = np.float64))
    _goals.append(np.array([1,7], dtype = np.float64))
    _goals.append(np.array([6,4], dtype = np.float64))
    # _keys.append(np.array([1,7], dtype = np.float64))

    goals = []
    goal_half_width = 1
    for goal in _goals:
        goal = np.array([-(goal[0] - goal_half_width), (goal[0] + goal_half_width), -(goal[1] - goal_half_width), (goal[1] + goal_half_width)])
        goals.append((A, goal))

    # b = np.array([-0.5, 1.5, -6.5, 7.5], dtype = np.float64)
    # goal = (A, b)

    tmax = 30.
    vmax = 3.

    avoid_walls = Node('and', deps=[Node('negmu', info={'A':A, 'b':b}) for A, b in walls])
    always_avoid_walls = Node('A', deps=[avoid_walls, ], info={'int':[0,tmax]})

    # # avoid_doors = [Node('negmu', info={'A':A, 'b':b}) for A, b in doors]
    # pick_keys = [Node('mu', info={'A':A, 'b':b}) for A, b in keys]
    # untils = [Node('U', deps=[pick_key], info={'int':[0,tmax]}) for pick_key in pick_keys]

    reach_goals = [Node('mu', info={'A':A, 'b':b}) for A, b in goals]
    # finally_reach_goal = Node('F', deps=[reach_goal,], info={'int':[0,tmax]})

    # spec = Node('and', deps = untils + [always_avoid_walls, finally_reach_goal])

    x0s = [x0,]
    # specs = [spec,]
    # PWL = plan(x0s, specs, bloat=0.18, MIPGap = 0.99, num_segs=26, tmax=tmax, vmax=vmax)

    plots = [[disturbances, 'r'], [goals, 'g'], [walls, 'k']] #[[goal,], 'b'], 
    return x0s, plots #, PWL

if __name__ == '__main__':
    results = vis(test)