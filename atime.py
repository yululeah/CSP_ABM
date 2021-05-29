# -*- coding: utf-8 -*-
"""
Mesa Time Module
================
adjust time activiation in mesa
only when current time >=timestart, agent move
"""

from collections import OrderedDict

# mypy
from typing import Dict, Iterator, List, Optional, Union

from mesa import Model
from mesa import Agent


# BaseScheduler has a self.time of int, while
# StagedActivation has a self.time of float
TimeT = Union[float, int]


class SmartScheduler:
    def __init__(self, model: Model) -> None:
        self.model = model
        self.steps = 0
        self.time: TimeT = 0
        self._agents: Dict[int, Agent] = OrderedDict()
            
    def add(self, agent: Agent) -> None:
        if agent.unique_id in self._agents:
            raise Exception(
                "Agent with unique id {0} already added to scheduler".format(
                    repr(agent.unique_id)
                )
            )
        self._agents[agent.unique_id] = agent
            
    def remove(self, agent: Agent) -> None:
        """Remove all instances of a given agent from the schedule.

        Args:
            agent: An agent object.

        """
        del self._agents[agent.unique_id]
        
    
    def step(self) -> None:
        for agent in self.agent_buffer(shuffled=False):
            # only when current time >=timestart, agent move
            if self.steps >= agent.timestart:
                agent.step()
        
        self.steps += 1
        self.time  += 1

        
    def get_agent_count(self) -> int:
        """ Returns the current number of agents in the queue. """
        return len(self._agents.keys())


    @property
    def agents(self) -> List[Agent]:
        return list(self._agents.values())
    
    
    def agent_buffer(self, shuffled: bool = False) -> Iterator[Agent]:
        """Simple generator that yields the agents while letting the user
        remove and/or add agents during stepping.

        """
        agent_keys = list(self._agents.keys())
        if shuffled:
            self.model.random.shuffle(agent_keys)

        for key in agent_keys:
            if key in self._agents:
                yield self._agents[key]
