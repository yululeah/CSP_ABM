# -*- coding: utf-8 -*-
"""
Created on Mon Apr 26 13:49:57 2021

@author: Lu Yu
"""

import warnings
warnings.filterwarnings("ignore")

from atime import *
from agent import *
from model import *


import pandas as pd
import numpy as np
#import geatpy as ea
import geopandas as gpd
#from sys import path as paths
#from os import path as path
import igraph as igp
import osmnx
import networkx as nx
#from scipy.spatial import cKDTree
#from mesa.space import NetworkGrid

'''load data'''
place = "Sioux Falls"
G =  osmnx.graph_from_place(place,network_type='drive',simplify=False,retain_all = True,
                                              truncate_by_edge = True,
                                              clean_periphery=False)
osmnx.plot_graph(G)
G = osmnx.add_edge_speeds(G)
G = osmnx.add_edge_travel_times(G)

 
#Pass in a few default speed values (km/hour) to fill in edges with 
#missing `maxspeed` from OSM
hwy_speeds = {"residential": 35, "secondary": 50, "tertiary": 60}
G  = osmnx.add_edge_speeds(G , hwy_speeds)
G  = osmnx.add_edge_travel_times(G )

nodes,edges = osmnx.graph_to_gdfs(G)

nx.write_gml(G, 'Sioux_Falls.gml')

'''read file'''
g = igp.read('Sioux_Falls.gml')
#G = nx.MultiDiGraph(nx.read_gml('Data\map.gml'))
#nodes,_ = osmnx.graph_to_gdfs(wa_G) 
#nodes_tree = cKDTree(np.transpose([nodes.geometry.x, nodes.geometry.y])) # 创建cKDTree减少计算量加速计算
#grid = NetworkGrid(G)

'''create stations geopandas file'''
stations = pd.read_csv("sixous_fall_alt_fuel_stations.csv")
point = gpd.GeoDataFrame(stations, geometry=gpd.points_from_xy(stations.Longitude, stations.Latitude)) #指定经纬度坐标所在列
point.crs = 'EPSG:4326' #Specify the coordinate system epsg4326 as WGS1984
#Find the osmid of the nearest node of stations and move the station to the road node
stations['osmid'] = stations['geometry'].apply(lambda x: osmnx.get_nearest_node(G,(x.y,x.x)))
station_noodes = [nodes.index.get_loc(node) for node in stations.osmid]  #igraph id

 
'''create ev geopandas file'''
EV_agent = pd.read_csv("Sioux_Falls_veh_data.csv")
d_point = gpd.GeoDataFrame(EV_agent, geometry = gpd.points_from_xy(EV_agent.dlng, EV_agent.dlat)) #指定经纬度坐标所在列
EV_agent['dosmid'] = d_point['geometry'].apply(lambda x: osmnx.get_nearest_node(G,(x.y,x.x)))
o_point = gpd.GeoDataFrame(EV_agent, geometry = gpd.points_from_xy(EV_agent.olng, EV_agent.olat)) #指定经纬度坐标所在列
EV_agent['oosmid'] = o_point['geometry'].apply(lambda x: osmnx.get_nearest_node(G,(x.y,x.x)))

'''ABM model'''
EVmodel = EVModel('trail1', EV_agent = EV_agent, nodes_data = nodes,
                    stations = stations, iigraph = g, network = G, seed = 1)

df = EVmodel.run(steps = 4000)
df_EV = df[df.agent_type == 'EV']
df_EV_stranded = df_EV[df_EV.status == 'stranded']
df_EV_finished = df_EV[df_EV.status == 'finished']
df_EV_error = df_EV[df_EV.status == 'error'] #these vehicles can't find stations
df_EV_finished.distance_travelled  = df_EV_finished.distance_travelled/1000
#check an agent
#check_single_ev = df_EV.xs(681, level='AgentID') #multi index