# -*- coding: utf-8 -*-
"""
Created on Tue Apr 20 10:33:35 2021
Description: This is for simulation platform based on ABM method
@author: Lu Yu

igraph id to osmnx id
nodes.iloc[1].name 
osmnx id to igraph id
nodes.index.get_loc('47150803')  
"""

import model
from mesa import Agent
import osmnx
import numpy as np


class EV(Agent):
    """
    EV agent
    """
    
    global SOC_MAX, SOC_MIN, MIN_SOC_CHARGING, MAX_SOC_CHARGING, LOCK_SIZE, recondider_time
    SOC_MAX = 80  # SOC at which to stop charging
    SOC_MIN = 0  # SOC at which to stop driving if no charging is available
    MIN_SOC_CHARGING = 60  # Minimum SOC to consider charging - above this SOC the user must never charge
    MAX_SOC_CHARGING = 20  # maximum SOC to consider charging - below this SOC, the user must charge
    LOCK_SIZE = 4000.0  # Size of block in meters, where relocation preferred over waiting
    recondider_time = 600 #seconds
    
    
    def __init__(self, 
                 unique_id: int, 
                 SOC: int,  #soc when trip starts #default
                 pos: int,  #trip origin #default #osmid id 
                 lat: float,
                 lon: float,
                 timestart: int, #trip start time #default
                 range_fe: int, #EV's range when SOC = 100# #default
                 capacity: float, #EV's capacity  e.g. 30kW #default
                 charging_start_soc: int,
                 charging_end_soc: int,
                 destination: int, #trip destination #default #osmid id 
                 model: model.EVModel):
        super().__init__(unique_id, model)
        
        self.SOC =  SOC #soc when trip starts #default
        self.pos = pos
        self.lat = lat
        self.lon = lon
        self.timestart = timestart
        self.range_fe = range_fe *1000 #meter
        self.capacity = capacity
        self.charging_start_soc =  charging_start_soc
        self.charging_end_soc = charging_end_soc
        self.destination = destination
                 
                 
        self.agent_type = 'EV' #distinguish agent's type

        self.fuel_consumption = self.capacity/self.range_fe *100 #default
        self.remaining_range = self.range_fe * self.SOC / 100  # range*soc/100  /meter
        self.origin = self.pos #trip origin #default #osmid id 
        
        
        self.distance_travelled = 0 
        self.charging_start = None #a list to collect charging start time/ soc/ station
        self.prob_charging = 0  #probability of charging
        self.charge_decision_time = None #record time until 10 min
        self.find_next_charging_station = False # in case of always find the next station during wating
        
       
        self.destination_igraph_id = self.model.nodes.index.get_loc(self.destination)
        self.target =  self.destination_igraph_id  #current destination, may change to station: igraph ID
        self.trip_distance = self.model.igraph.shortest_paths_dijkstra(source = self.model.nodes.index.get_loc(self.origin),
                                                                  target = self.target ,
                                                                  weights = 'length')[0][0] #default
        self.route = [] #route
        self.route_index = 0 #Defined on which road 
        self.currentRoad = None #define current road index
        self.distance_along_edge = 0 #If it's halfway to the edge
        self.pos_igraph_id = None #Location on the path: igraph id
        self.station_nodes = self.model.station_nodes #inherit all station from model #osimid table
        self.stations = [self.model.nodes.index.get_loc(node) for node in self.station_nodes.values] #igraph id
        
        self.finised_time = None # finished time: arrive or stranded
        self.charge_station = None #igraph ID
        self.current_station = None #agent object
        
        self.status = 'driving' #driving, finished, stranded, to_charge, charge, wait
        
        #待更新
        self.veh_speed = 40 #km/h 
        #chargers_onpath = []  #Charging station in front of the road
        #compat_chargers = [] 
        #self.connector_code = 0 #1/3 chademo，2/3 combo；#default #need to update
        
    def update_location(self):
        '''update when driving 
        '''
        total_distance = self.distance_to_next_node() + self.distance_along_edge
        origin_node = self.model.nodes.loc[self.route[self.route_index]]
        if self.route_index == len(self.route)-1 or total_distance == 0:
            self.lat = origin_node.geometry.y
            self.lon = origin_node.geometry.x
        else:
            k = self.distance_along_edge / total_distance
            destination_node = self.model.nodes.loc[self.route[self.route_index + 1]]
            self.lat = k * destination_node.geometry.y + (1 - k) * origin_node.geometry.y
            self.lon = k * destination_node.geometry.x + (1 - k) * origin_node.geometry.x
         
        self.pos_igraph_id = self.model.nodes.index.get_loc(self.pos) 
        
    def update_route(self): 
        '''
        Find the shortest path on the graph between the EV and the target, 
		this should happen only once when the trip starts and then the vehicle will traverse on this path	
        happen when trip starts
        happen when charging station make sense
        happen when charging end 
        '''
        #PATH returns a lst of igraph node id
        path = self.model.igraph.get_shortest_paths(self.pos_igraph_id, self.target, weights='length')[0]
        #route returns a lst of multidigraph node id
        self.route = self.model.nodes.iloc[path].index
        self.route_index = 0 
        self.distance_along_edge = 0
        self.veh_speed = 40 
        
    def update_station(self):
        '''
        Find nearest station
        Unresolved problem with the site at the rear
        Now I use finding charging stations with the shortest distance. 
        The amount of calculation is large. 
        we should find charging stations that are 10km (or less) along the road
        
        distance = distance from current location to charging station + distance from charging station to destination
        '''
        #stations = [self.model.nodes.index.get_loc(node) for node in self.station_nodes.values]
        
        #find distances from current postion to all stations by dijkstra method
        pos_to_station_distances = self.model.igraph.shortest_paths_dijkstra(source=[self.pos_igraph_id],
                                                                             target=self.stations,
                                                                             weights='length')[0]
        
        sta_to_d_distance =  self.model.igraph.shortest_paths_dijkstra(source = self.stations,
                                                                       target = self.destination_igraph_id,
                                                                       weights = 'length')[0]
        
        # find the nearest staion igraph id
        station = self.stations[int(np.argmin(np.transpose(np.array(sta_to_d_distance))[0] + 
                                         np.array(pos_to_station_distances)))]

        
        self.charge_station = station #target is the igraph ID
   
    def find_next_station(self):
        '''
        when current station is full, find next station
        return distance between current pos to next station
        '''
        self.stations.remove(self.charge_station)
        
        #find distances from current postion to all stations by dijkstra method
        pos_to_station_distances = self.model.igraph.shortest_paths_dijkstra(source=[self.pos_igraph_id],
                                                                             target = self.stations,
                                                                             weights='length')[0]
        
        sta_to_d_distance =  self.model.igraph.shortest_paths_dijkstra(source = self.stations,
                                                                       target = self.destination_igraph_id,
                                                                       weights = 'length')[0]

        return min(np.transpose(np.array(sta_to_d_distance))[0] + np.array(pos_to_station_distances)) 
    
    def finish(self):
        self.route_index = len(self.route) - 1
        if self.status == 'driving':
            self.status ='to_be_finished'
        else:
            self.status ='to_be_finished_charge'
        self.model.grid.move_agent(self, self.route[len(self.route)-1]) #reach target
        
    def distance_to_next_node(self):
        """Finds the distance to the next node along the route"""
        if len(self.route) <=1 :
            return 0
        elif self.route_index == len(self.route) - 1:
            self.finish()
            return 100000000000 #make sure step_distance < distance_to_next_node
        else:
            edge = self.model.network.get_edge_data(self.route[self.route_index], self.route[self.route_index + 1])
            if 'osmid' in edge[0].keys():
                self.currentRoad = edge[0]['osmid']
            s = edge[0]['length'] - self.distance_along_edge
            if  s > 0 :
                return s
            else:
                self.finish()
                return 100000000000 #make sure step_distance < distance_to_next_node

        
    def update_status(self):
        '''
        as long as as the vehicle is approaching the target, the SOC should be updated
        '''
        if self.status == 'driving' or self.status == 'to_charge':
            self.move()
            self.update_location()
            #fuel_consumed = self.fuel_consumption * self.step_distance / 100 #Fuel consumed per mile = fuel_consumption / 100
            #self.SOC - fuel_consumed / self.capacity
            self.remaining_range = self.remaining_range - self.step_distance
            self.SOC = self.remaining_range / self.range_fe * 100
            self.distance_travelled = self.distance_travelled + self.step_distance
            
    def charge(self):
        '''charge status update SOC and station energe consumption
        '''
        self.SOC = self.SOC + self.current_station.max_power * self.model.steptime * 100.0 / self.capacity / 60.0 / 60.0
        self.remaining_range = self.range_fe * self.SOC /100 
        self.current_station.energy_consumed = self.current_station.energy_consumed +  self.current_station.max_power * self.model.steptime / 60.0 / 60.0

    def update_station_object_agent(self):  
        # turn igraph ID to osmid ID and find the station information
        station_temp = self.model.nodes.iloc[self.charge_station].name
        #station information extraction
        this_cell = self.model.grid.get_cell_list_contents([station_temp])
        self.current_station = [obj for obj in this_cell if isinstance(obj, station)][0]
            
    def charge_makes_sense(self):
        '''
         This is where we integrate Yan's CCDM 
         (GE Y. Discrete Choice Modeling of Plug-in Electric Vehicle Use and Charging Behavior Using Stated Preference Data[D].2019.)
         charge will make sense depending on following factors: 
         SOC, time_in_car, charging_cost,  charging_time, access_time, amenity_level (?), deviation(?)
         Considering SDCM4 cofficients
        '''
        intercept = 2.034
        c_soc = -4.584
        c_dev = 2.440
        c_time_in_car = -0.069
        c_charging_cost = -0.010
        c_charging_time = -0.242
        c_access_time = -0.025
        c_amenity_restroom = 0.049
        c_amenity_more = 0.213
        #soc = self.SOC / 100 # SOC in percent
        dev = 0 # still to find out how to calculate dev for a trip, but either 1 or 0
        #need to update
        time_in_car = (self.model.schedule.time - self.timestart) / 60 # time in hours since driving - need to check how this variable is affected after charging once  
        charging_price = 0.0
        parking_price = 0.0
        
        
        '''Still need to find values of following covariates '''
        #access_time
        #path path_to_cs
        amenity_restroom = 1 # al charging stations have restrooms, ** this assumption needs validation **
        restaurents = 1
        #amenity_more
        if restaurents > 0:
            amenity_more = 1
        else:
            amenity_more = 0
        
        if self.charge_station != None:  
            # turn igraph ID to osmid ID and find the station information
            station2 = self.model.nodes.iloc[self.charge_station].name
            #station information extraction
            this_cell = self.model.grid.get_cell_list_contents([station2])
            station3 = [obj for obj in this_cell if isinstance(obj, station)][0]
        
        # Talk to the nearest EVSE to find out VSE specific parameters
            charging_time = (max(80, self.charging_end_soc) - self.SOC) * self.capacity / 100 / station3.max_power # energy used / power = time /hours
            
            if station3.dcfc_var_parking_price_unit == "min":
                parking_price = station3.dcfc_fixed_parking_price + charging_time * 60 * station3.dcfc_var_parking_price
            
            if station3.dcfc_var_charging_price_unit == "min":
            # this is the charging cost in dollar - will be depedent on the price model of each EVSE
                charging_price = station3.dcfc_fixed_charging_price + charging_time * 60 * station3.dcfc_var_charging_price
            elif station3.dcfc_var_charging_price_unit == "kWh":
                charging_price = station3.dcfc_fixed_charging_price + ((max(80, self.charging_end_soc) - self.SOC) /100 * self.capacity * station3.dcfc_var_charging_price / station3.max_power)
            charging_cost_in_dollar = parking_price + charging_price
            
            access_time = self.model.igraph.shortest_paths_dijkstra(source = self.pos_igraph_id ,target = self.charge_station,weights='travel_time')[0][0] / 60# this converts the time to minutes

        #utility of charging
        u_charging = intercept + (c_soc * self.SOC / 100) + (c_dev * dev) + (c_time_in_car * time_in_car) + (c_charging_cost * charging_cost_in_dollar) + (c_charging_time * charging_time) + (c_access_time * access_time) + (c_amenity_restroom * amenity_restroom) + (c_amenity_more * amenity_more)
        odds_charging = np.exp(u_charging)
        prob_charging = odds_charging / (1 + odds_charging)

        # Make a random draw using the probability from a binomial distribution	
        return np.random.binomial(1, prob_charging)
    
    def distance_to_des(self):
        '''return distance between current position to the destination'''
        return self.model.igraph.shortest_paths_dijkstra(source = self.pos_igraph_id,
                                             target = self.destination_igraph_id,
                                             weights = 'length')[0][0] #default
        
    def move(self):
        #calculate travelling distance in a step
        self.step_distance = self.veh_speed * self.model.steptime / 3.6 # m/min
        
        step_dis = self.step_distance
        #if reach new node
        dis = self.distance_to_next_node()  
        global next_edg_dis
        if step_dis >= dis: 
            #find how many roads by one step
            global route_i
            if self.route_index + 1 < len(self.route)-2:
                for route_i in range(self.route_index + 1, len(self.route)-2):
                    #Distance to the next point + distance to the next edge
                    next_edg_dis = self.model.network.get_edge_data(self.route[route_i], self.route[route_i+1])[0]['length']
                    dis += next_edg_dis
                    if dis > step_dis:
                        break
                    #else:
                    #    dis -= next_edg_dis
            elif len(self.route) == 1:
                self.finish()
                self.veh_speed = 0
                step_dis = 0
            else:
                next_edg_dis = self.model.network.get_edge_data(self.route[-2], self.route[-1])[0]['length']
            
            step_dis = next_edg_dis - (dis - step_dis)
            
            self.route_index = route_i
            if self.route_index <= len(self.route) - 1:
                self.model.grid.move_agent(self, self.route[self.route_index])
                self.currentRoad = self.model.network.get_edge_data(self.route[route_i], self.route[route_i+1])[0]['osmid']
                #update speed
                self.veh_speed =   self.model.network.get_edge_data(self.route[route_i], self.route[route_i+1])[0]['speed_kph']
            else:
                self.finish()
                self.veh_speed = 0
                
            self.distance_along_edge = 0
            
        #If new node is not reached
        self.distance_along_edge += step_dis #add the unfinished section just now
    
        
    def status_switch(self):
        #fsm
        if self.status =='finished' or self.status == 'stranded' or self.status == 'error':
            self.model.schedule.remove(self)
            
        if self.status =='to_be_finished':
            self.status ='finished'
            self.finised_time = self.model.schedule.time
            
        if self.SOC < SOC_MIN:
            self.status ='stranded'
            self.finised_time = self.model.schedule.time
            return
        
        # reconsider charging time equals to 10 mins
        if self.charge_decision_time == None or (self.model.schedule.time - self.charge_decision_time) > recondider_time :
            if self.status == 'driving':
                #if distance to the destion is larger than remaing range 
                #or soc less than charging start soc
                #or soc less than 20% in case of the simulation has too may stranded EVs
                if self.SOC < min(self.charging_start_soc, MIN_SOC_CHARGING ) or self.distance_to_des() > self.remaining_range or self.SOC < MAX_SOC_CHARGING:
                    #update station nearby
                    self.update_station()
                    #if the nearest stion makes sense
                    if self.charge_makes_sense() > 0:
                        self.status = 'to_charge'
                        self.target = self.charge_station  #target is the igraph ID
                        #replan the route to the charger
                        self.update_route()
                        
                    else:
                        #remove the charger from the charger list
                        self.charge_decision_time = self.model.schedule.time
                        self.station_nodes = self.station_nodes[self.station_nodes.values != self.model.nodes.iloc[self.charge_station].name]
                
        if self.status =='to_be_finished_charge':
            if self.charge_station != None:
                self.update_station_object_agent()
                self.status = 'wait'
                self.current_station.waiting_evs_count += 1
                self.veh_speed = 0
            else:
                self.status ='error' #can't find charging station, delete the vehicle
        
        if self.status == 'wait': 
            # if target station still has available chargers:
            if  self.current_station.dcfc_count > self.current_station.plugs_in_use:
                self.status = 'charge'
                self.current_station.plugs_in_use += 1
                self.current_station.waiting_evs_count -= 1
                
        if self.status == 'wait' and self.find_next_charging_station == False: 
            origin_station = self.current_station
            
            # ensure there are still stations
            if len(self.stations) > 2:      
                # if EV can reach the next station
                if self.find_next_station() < self.remaining_range:
                    
                    self.status = 'to_charge'
                    self.station_nodes = self.station_nodes[self.station_nodes.values != self.model.nodes.iloc[self.charge_station].name]
                    #update station nearby
                    self.update_station()
                    #replan the route to the 
                    self.target = self.charge_station  #target is the igraph ID
                    self.update_route()
                    self.find_next_charging_station = False  #change to True for second charging
                else:
                    self.current_station = origin_station
                    self.find_next_charging_station = True
                        
                
        if self.status == 'charge':
            self.charge()
            #if charge to full
            if self.SOC > max(self.charging_end_soc, SOC_MAX):
                self.status = 'driving'
                self.current_station.plugs_in_use -= 1
                #Re-plan the route to the end
                self.target = self.destination_igraph_id 
                self.update_route()
                self.charge_decision_time = None
                
                
    def step(self):
        #refresh current location, status, and charges nearby
        self.update_status() 
        self.status_switch()
                
        
class station(Agent):
    """
    station agent
    """        
    def __init__(self, 
                 unique_id: int, 
                 dcfc_count: int, #num of plugs
                 dcfc_var_charging_price_unit: str,
                 dcfc_fixed_charging_price: int,
                 dcfc_var_charging_price: int,
                 dcfc_fixed_parking_price: int,
                 dcfc_var_parking_price: int,
                 osmid: int,
                 model: model.EVModel):
        super().__init__(unique_id, model)

        self.dcfc_count = dcfc_count
        self.dcfc_var_charging_price_unit = dcfc_var_charging_price_unit
        self.dcfc_fixed_charging_price = dcfc_fixed_charging_price
        self.dcfc_var_charging_price = dcfc_var_charging_price
        self.dcfc_fixed_parking_price = dcfc_fixed_parking_price
        self.dcfc_var_parking_price = dcfc_var_parking_price
        
        self.dcfc_var_parking_price_unit = "min"
        self.agent_type = 'station'
        self.plugs_in_use = 0
        self.waiting_evs_count = 0
        self.max_power = 50.0 #current
        self.restaurents = 1 
        self.current_power_draw = 0 
        #self.queued_evs = []
        self.igraph_index = None
        self.energy_consumed = 0
        
        ##just None for them in order to output
        self.timestart = 0 #always
        self.lat = None
        self.lon = None 
        self.status = 'station'
        self.veh_speed = None
        self.SOC = None
        self.distance_travelled  = None
        self.remaining_range = None
        self.route_index = None
        self.charge_station = None #igraph ID
        
    def step(self):    
        '''
        stay
        '''