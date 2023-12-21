""" AI air traffic control. CS 470 Semester Project """
import math
import random
from copy import deepcopy

import numpy as np
# Import the global bluesky objects. Uncomment the ones you need
from bluesky import core, stack, traf, navdb  # , settings, navdb, sim, scr, tools
from geographiclib.geodesic import Geodesic
from enum import Enum

STATUS = Enum('STATUS', ['REACHED', 'ADVANCED', 'TRAPPED'])
METERS_PER_MILE = 1609.344
FEET_PER_METER = 3.28084

# Safety Parameters
# Minimum distance between aircraft in miles
MIN_AIRCRAFT_DISTANCE = 1

# Amount of time to try to connect to the goal
# Also helps control amount of influx
K = 100
# Distance to extend the tree by in miles
e = 1

routes = dict()


# Utility Functions
def meters_to_miles(meters):
    return meters / METERS_PER_MILE


def miles_to_meters(miles):
    return miles * METERS_PER_MILE


def feet_to_meters(feet):
    return feet / FEET_PER_METER


def meters_to_feet(meters):
    return meters * FEET_PER_METER


def pythag(a, b):
    return math.sqrt(a ** 2 + b ** 2)


def enable_auto_nav(acid):
    stack.stack('LNAV %s ON' % acid)
    stack.stack('VNAV %s ON' % acid)


class Waypoint:
    def __init__(self, acid, lat, lon, alt, hdg):
        self.acid = acid
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.hdg = hdg

    def __eq__(self, other):
        return self.acid == other.acid \
            and self.lat == other.lat \
            and self.lon == other.lon \
            and self.alt == other.alt \
            and self.hdg == other.hdg

    def __str__(self):
        return 'Waypoint(%s, %f, %f, %f, %f)' % (self.acid, self.lat, self.lon, self.alt, self.hdg)


class Tree:
    def __init__(self, tree=None):
        self.swapped = False
        if tree is None:
            self.tree = []
        else:
            self.tree = tree

    def __str__(self):
        strTree = 'Tree:\nSwapped: ' + str(self.swapped) + '\n'
        for waypoint in self.tree:
            strTree += str(waypoint) + '\n'

        return strTree

    def add_waypoint_directives(self, acid):
        # TODO: Figure out how to calculate the speed of the aircraft
        spd = 150

        for waypoint in self.tree:
            stack.stack('ADDWPT %s %f %f %f %f' %
                        (waypoint.acid, waypoint.lat, waypoint.lon, meters_to_feet(waypoint.alt), spd))

        enable_auto_nav(acid)

    def add_vertex(self, node):
        self.tree.append(node)

    @staticmethod
    def combine(Ta, Tb):
        if Ta.swapped:
            Tree.swap(Ta, Tb)

        # Find the path from Ta to Tb
        combinedTree = Tree()
        combinedTree.tree = Ta.tree + Tb.tree[::-1]

        # print('Combined: %s' % combinedTree)

        return combinedTree

    @staticmethod
    def swap(Ta, Tb):
        Ta.swapped = not Ta.swapped
        Tb.swapped = not Tb.swapped
        oldTa = Ta.tree
        Ta.tree = Tb.tree
        Tb.tree = oldTa


# Approach End Runway 13 at KPVU
ULTIMATE_GOAL = Waypoint(None, 40.228764, -111.731163, feet_to_meters(1), 130.0)


def random_config(q_init, q_goal):
    # Generate a random configuration
    lat = random.uniform(min(q_init.lat, q_goal.lat), max(q_init.lat, q_goal.lat))
    lon = random.uniform(min(q_init.lon, q_goal.lon), max(q_init.lon, q_goal.lon))
    alt = random.uniform(min(q_init.alt, q_goal.alt), max(q_init.alt, q_goal.alt))
    hdg = random.uniform(0, 360)
    return Waypoint(q_init.acid, lat, lon, alt, hdg)


def nearest_neighbor(q, T):
    # Find the nearest neighbor to q in the tree T
    return min(T.tree,
               key=lambda w: pythag(Geodesic.WGS84.Inverse(q.lat, q.lon, w.lat, w.lon)['s12'], abs(q.alt - w.alt)))


def in_vicinity(q1, q2):
    # Check if q1 and q2 are within e miles of each other
    inverseResult = Geodesic.WGS84.Inverse(q1.lat, q1.lon, q2.lat, q2.lon)
    dist = meters_to_miles(inverseResult['s12'])
    altDiff = meters_to_miles(abs(q1.alt - q2.alt))

    return pythag(dist, altDiff) <= MIN_AIRCRAFT_DISTANCE


def new_config(q, q_near, q_new):
    # Check if a new configuration is possible
    # TODO: Improve collision detection
    for i in range(traf.ntraf):
        if traf.id[i] == q_new.acid:
            continue
        if traf.id[i] in routes:
            for waypoint in routes[traf.id[i]].tree:
                if in_vicinity(q_new, waypoint):
                    return False
        else:
            if in_vicinity(q_new, Waypoint(traf.id[i], traf.lat[i], traf.lon[i], traf.alt[i], traf.hdg[i])):
                return False

    return True


def new_q_point(q, q_near):
    # Calculate heading and distance to q from q_near
    inverseResult = Geodesic.WGS84.Inverse(q_near.lat, q_near.lon, q.lat, q.lon)
    hdg = inverseResult['azi1']
    dist = meters_to_miles(inverseResult['s12'])

    # If the goal is within e miles of q_near, return the goal
    if dist <= e:
        return q

    # Calculate new altitude, for now use heuristic algorithm
    # TODO: Use a better algorithm here to determine altitude
    distFromGoal = meters_to_miles(
        Geodesic.WGS84.Inverse(q_near.lat, q_near.lon, ULTIMATE_GOAL.lat, ULTIMATE_GOAL.lon)['s12'])
    newAlt = min(distFromGoal * 300, 1000)

    # Generate a new point e miles away from q_near in the hdg direction
    result = Geodesic.WGS84.Direct(q_near.lat, q_near.lon, hdg, miles_to_meters(e))

    return Waypoint(q_near.acid, result['lat2'], result['lon2'], feet_to_meters(newAlt), hdg)


def extend(T, q):
    # print('***')
    # print('Extend: %s' % q)
    # print(str(T))
    qNear = nearest_neighbor(q, T)
    # print('Nearest: %s' % qNear)
    qNew = new_q_point(q, qNear)
    # print('Extended: %s' % qNew)
    # print('***')
    if new_config(q, qNear, qNew):
        T.add_vertex(qNew)
        if qNew == q:
            return STATUS.REACHED, qNew
        else:
            return STATUS.ADVANCED, qNew
    return STATUS.TRAPPED, None


def connect(T, q):
    while True:
        status, _ = extend(T, q)
        if status != STATUS.ADVANCED:
            return status


def rrt_connect_planner(q_init, q_goal):
    # Initialize the tree
    Ta = Tree([q_init])
    Tb = Tree([q_goal])

    for k in range(K):
        q_rand = random_config(q_init, q_goal)
        result, q_new = extend(Ta, q_rand)
        if result != STATUS.TRAPPED:
            # print('---\nAdded: %s\n---' % q_new)
            if connect(Tb, q_new) == STATUS.REACHED:
                return Tree.combine(Ta, Tb)
        Tree.swap(Ta, Tb)

    return None


### Initialization function of plugin. Do not change the name of this
### function, as it is the way BlueSky recognises this file as a plugin.
def init_plugin():
    ''' Plugin initialisation function. '''
    # Instantiate our aiControl entity
    aiControl = AIControl()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name': 'AI-CONTROL',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type': 'sim',
    }

    # init_plugin() should always return a configuration dict.
    return config


class AIControl(core.Entity):
    ''' AI Air Traffic Control '''

    def __init__(self):
        super().__init__()
        self.onFinal = []
        self.test = True

    def init_traffic(self, numAircraft):
        ''' Initializes test simulation traffic. '''

        boundries = [(40.302122, -111.896141), (40.376965, -111.714866)]
        acid_base = 'HA00'
        type = 'B737'
        hdg = 180.0
        alt = 1500
        spd = 200

        for i in range(numAircraft):
            acid = acid_base + str(i + 1)
            lat = random.uniform(boundries[0][0], boundries[1][0])
            lon = random.uniform(boundries[0][1], boundries[1][1])
            stack.stack('CRE %s %s %f %f %f %f %f' % (acid, type, lat, lon, hdg, alt, spd))

        stack.stack('ASAS OFF')

    def run_algorithm(self):
        ''' Runs the RRT algorithm. '''
        for i in range(traf.ntraf):
            if i not in self.onFinal:
                q_init = Waypoint(traf.id[i], traf.lat[i], traf.lon[i], traf.alt[i], traf.hdg[i])
                # Approach End Runway 13 at KPVU
                q_goal = deepcopy(ULTIMATE_GOAL)
                q_goal.acid = traf.id[i]

                # print(q_init)
                # print(q_goal)

                route = rrt_connect_planner(q_init, q_goal)
                # print ('Route for Flight %d:\n%s' % (i, str(route)))
                if route is not None:
                    routes[traf.id[i]] = route
                    stack.stack('DELRTE %s' % traf.id[i])
                    route.add_waypoint_directives(traf.id[i])
                    stack.stack('%s ATALT %f DEL %s' % (traf.id[i], 2, traf.id[i]))
                    # TODO: Finish implementing final approach functionality
                    # stack.stack('%s AT %s%03d DO reached_final %s' %
                    #             (traf.id[i], traf.id[i], len(route.tree), traf.id[i]))

    @core.timed_function(name='example', dt=1)
    def update(self):
        ''' Periodic update for incoming traffic. '''
        if traf.ntraf > 0:
            self.run_algorithm()

    # TODO: Finish implementing final approach functionality
    # @stack.command
    # def reached_final(self, acidx: 'acid'):
    #     stack.stack('ECHO %s reached final' % acidx)
    #     self.onFinal.append(acidx)
    #     # Runway 13 at KPVU
    #     stack.stack('ADDWPT %s %f %f %f %f' % (traf.id[acidx], 40.230103, -111.732386, 10.0, 130))
    #     enable_auto_nav(traf.id[acidx])
    #     stack.stack('%s ATALT %f DEL %s' % (traf.id[acidx], 20.0, traf.id[acidx]))

    @stack.command
    def start_sim(self, count: int = 1):
        ''' Set the number of passengers on aircraft 'acid' to 'count'. '''
        stack.stack('ECHO Starting AI Control Simulation')
        self.init_traffic(count)
