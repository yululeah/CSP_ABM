Based on the MESA package in Python, this project builds an electric vehicle driving and charging simulation platform based on the agent-based modeling (ABM).
CSP: charging station problem-- what I am working at based on the ABM platform.
The small case is Sioux Falls. If you want to run the road network of Shanghai or WA, in "run.py" replace place = "Sioux Falls" with place = "Shanghai, China" or "Washington State, USA" and so on. Automatically download the road network of the corresponding city from Openstreetmap. At the same time manually update "stations.csv" and "veh_data.csv" to change the data of the corresponding city.

please run the "run.py" file.

dependencies:
  - python==3.7
  - pip
  - osmnx==1.0.1
  - python-igraph==0.9.1
  - mesa==0.8.8.1


