"""
parametric_transition_system.py
Description:
    A parametric transition system for the robot motion planning example.
"""

from typing import List, Set, Tuple

from kltl_types import State, Action, AtomicProposition, Output
from .pts_types import Transition, Parameter

class ParametricTransitionSystem:
    """
    ParametricTransitionSystem
    Description:
        A class representing a parametric transition system.
    """
    def __init__(
            self,
            S: List[State],
            Act: List[Action],
            AP: List[AtomicProposition],
            I: List[State] = None,
            Y: List[Output] = None,
            Theta: List[Parameter] = None,
            transitions: List[Transition] = None,
            labels: List[Tuple[State, AtomicProposition]] = None,
            output_map: List[Tuple[State, Parameter, Output]] = None,
    ):
        # Input Processing
        assert len(S) > 0

        if I is None:
            I = []
        if Y is None:
            Y = []
        if Theta is None:
            Theta = []
        if transitions is None:
            transitions = []
        if labels is None:
            labels = []
        if output_map is None:
            output_map = []

        self.S = S
        self.Act = Act
        self.AP = AP
        self.I = I
        self.transitions = transitions
        self.labels = labels

        if Theta == []:
            Theta = ["theta1"]
        self.Theta = Theta

        if Y == []:  # If Y is undefined, then give it the value of the state set.
            Y = S
        self.Y, self.output_map = Y, output_map

    def add_transition(self, s1: State, a: Action, theta: Parameter, s2: State):
        assert s1 in self.S, f" State {s1} is not in state space!"
        assert s2 in self.S, f" State {s2} is not in state space!"
        assert theta in self.Theta, f"Parameter {theta} is not in parameter space!"
        assert a in self.Act

        self.transitions += [(self.S.index(s1), self.Act.index(a), self.Theta.index(theta), self.S.index(s2))]
        self.transitions = list(set(self.transitions))

    def add_label(self, s: State, ap: AtomicProposition):
        assert s in self.S, f" State {s} is not in state space!"
        assert ap in self.AP, f"Proposition {ap} is not in atomic proposition space!"

        self.labels += [(self.S.index(s), self.AP.index(ap))]
        self.labels = list(set(self.labels))

    def add_output(self, s: State, theta: Parameter, o: Output):
        """
        add_output
        Description:
            Adds output to the transition system (if it doesn't already exist).
        :param s:
        :param theta:
        :param o:
        :return:
        """
        assert s in self.S, f" State {s} is not in state space!"
        assert theta in self.Theta, f"Parameter {theta} is not in parameter space!"
        assert o in self.Y, f" Output {o} is not in the output space!"

        self.output_map += [(self.S.index(s), self.Theta.index(theta), self.Y.index(o))]
        self.output_map = list(set(self.output_map))

    def post(self, s: State, a: Action = None, theta: Parameter = None) -> List[State]:
        assert s in self.S, f"State {s} is not in state space!"
        assert (a in self.Act) or (a is None), f"Action {a} is not in action space!"

        if (a is None) and (theta is None):
            return [self.S[s2] for (s1, a1, theta1, s2) in self.transitions if s1 == self.S.index(s)]
        elif (a is None):
            return [self.S[s2] for (s1, a1, theta1, s2) in self.transitions if s1 == self.S.index(s) and theta1 == self.Theta.index(theta)]
        elif (theta is None):
            return [self.S[s2] for (s1, a1, theta1, s2) in self.transitions if s1 == self.S.index(s) and a1 == self.Act.index(a)]
        else:
            return [
                self.S[s2]
                for (s1, a1, theta1, s2) in self.transitions
                    if s1 == self.S.index(s) and a1 == self.Act.index(a) and theta1 == self.Theta.index(theta)
            ]

    def L(self, s: State) -> List[AtomicProposition]:
        """
        l = ts.L(s)
        Description:
            Defines te
        :param s:
        :return:
        """
        # Input Processing
        assert s in self.S, f" State {s} is not in state space!"

        # Return
        return [self.AP[ap1] for (s1, ap1) in self.labels if s1 == self.S.index(s)]

    def O(self, s: State, theta: Parameter = None) -> List[Output]:
        """
        O_s = ts.O(s)
        Description
            Defines the outputs associated with the state s.
        :param s: The state at which we evaluate the outputs.
        :return O_s: The set of outputs possible for state s.
        """

        # Input Processing
        assert s in self.S, f" State {s} is not in state space!"

        # Return
        O_s = []
        if theta is None:
            O_s = []
            for theta in self.Theta:
                O_s += self.O(s, theta)
        else:
            assert theta in self.Theta, f"Parameter {theta} is not in parameter space!"
            O_s = [
                self.Y[o_index]
                for (s_index, theta_index, o_index) in self.output_map
                if s_index == self.S.index(s) and theta_index == self.Theta.index(theta)
            ]

        # Get unique elements of O_s
        return list(set(O_s))
