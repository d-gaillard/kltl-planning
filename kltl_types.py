
from typing import Tuple

State = str
Action = str
AtomicProposition = str

StateIndex = int
ActionIndex = int
AtomicPropositionIndex = int
Transition = Tuple[StateIndex, ActionIndex, StateIndex]

Output = str