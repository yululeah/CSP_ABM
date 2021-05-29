# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 10:33:35 2021
Description: This is for simulation platform based on ABM method
@author: Lu Yu
"""

from __future__ import annotations
from mesa import Model
from mesa.space import NetworkGrid
from mesa.datacollection import DataCollector
import geopandas
from geopandas import GeoDataFrame, sjoin
from typing import Optional, Iterable
from atime import SmartScheduler
import agent
import atime


class EVModel(Model):
    """
    Seed optional 
    """
    def __init__(
            self,
            # hazard: GeoDataFrame,
            output_path: str,
            #domain: [Polygon] = None,
            EV_agent: [GeoDataFrame] = None,
            nodes_data: [GeoDataFrame] = None,
            stations: [GeoDataFrame] = None,
            iigraph :[] = None,
            network :[] = None,
            seed: Optional[int] = None):

        super().__init__() 
        self._seed = seed
        self.schedule = SmartScheduler(self)
        self.steptime = 60  #60 seconds refresh  once
        self.igraph = iigraph
        self.nodes = nodes_data
        self.network = network 
                
        assert len(stations) > 0, 'There are no stations' # check there is station
        assert len(EV_agent) > 0, 'There are no ev' #check there is ev
        
        self.station_nodes = stations.osmid

        self.grid = NetworkGrid(self.network)  #osmid = self.pos
        

        
    # data output : df
        self.data_collector = DataCollector(
            model_reporters={
            },
            agent_reporters={
                            'agent_type':'agent_type',
                            'lat':'lat',
                            'lon':'lon',
                            'SOC' : 'SOC',
                            'veh_speed':'veh_speed',
                            'remaining_range':'remaining_range',
                            'distance_travelled':'distance_travelled',
                            'status' : 'status',
                            'route_index':'route_index',
                            'charge_station':'charge_station'
                                         }
                             )

    

    #create EV agent
        for index, row in EV_agent.iterrows():
            t = agent.EV(self.next_id(),row['soc'],row['oosmid'],row['olat'],
                         row['olng'],row['trip_start_time'],row['range_fe'],
                         row['capacity'],row['charging_start_soc'],row['charging_end_soc'],
                         row['dosmid'],
                         self)
            self.schedule.add(t)
            self.grid.place_agent(t, row['oosmid'])    
            t.update_route()
            t.update_location()
    
    #create station agent
        for index, row in stations.iterrows():
            s = agent.station(self.next_id(), row['dcfc_count'], row['dcfc_var_charging_price_unit'],
                  row['dcfc_fixed_charging_price'], row['dcfc_var_charging_price'],
                  row['dcfc_fixed_parking_price'],  row['dcfc_var_parking_price'],row['osmid'], self)
            self.schedule.add(s)
            self.grid.place_agent(s, row['osmid'])    
            
    def step(self):
        """Advances the model by one step and then stores the current state in data_collector"""
        self.schedule.step()

        self.data_collector.collect(self)

    def run(self, steps: int):
        """Runs the model for the given number of steps`

        Args:
            steps: number of steps to run the model for
        Returns:
            DataFrame: the agent vars dataframe
        """
        self.data_collector.collect(self)
        for _ in range(steps):
            self.step()
            
        # self.data_collector.get_agent_vars_dataframe().to_csv(self.output_path + '.agent.csv')
        # self.data_collector.get_model_vars_dataframe().to_csv(self.output_path + '.model.csv')
        return self.data_collector.get_agent_vars_dataframe()
