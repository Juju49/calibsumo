from itertools import pairwise, chain
from collections.abc import Sequence
from collections import namedtuple, OrderedDict

import numpy as np

from shapely.geometry import LineString, Polygon, CAP_STYLE, JOIN_STYLE
from shapely.ops import unary_union

from . import TYP_PED, TYP_ES, TYP_BK
from .pysocialforceSUMO import Simulator, scene
TYP_UNK = 0


DesirePoint = namedtuple("DesirePoint", ["coord", "lane", "edge"])


def get_edge_from_lane(lane_id: str) -> str:
    """cuts the lane number from the name to return the edge ID"""
    return lane_id.rstrip("0123456789")[:-1]


class Lane:
    def __init__(self, lane_id: str, point_seq: Sequence[tuple[float, float]], allowed: set[str], width: float = 0.0, ):
        self.is_walking_area = width == 0

        self.lane_id = lane_id
        self.edge_id = get_edge_from_lane(lane_id)
        self.allowed = allowed

        if self.is_walking_area:
            self.line = None
            self.poly = Polygon(point_seq)
        else:
            self.line = LineString(point_seq)
            self.poly = self.line.buffer(width, cap_style=CAP_STYLE.flat, join_style=JOIN_STYLE.mitre)


class SFMsim:
    def __init__(self, traci_connection):
        self.traci = traci_connection

        # 1) generate allowed spaces per vehicle type from loaded network

        stl = self.traci.lane
        ped_lanes = {}
        bike_lanes = {}
        es_lanes = {}
        for lane in stl.getIDList():
            lane_shape = stl.getShape(lane)
            allowed = set(stl.getAllowed(lane))

            if ":" == lane[0] and "_w" in lane:  # this is a walking area
                geometry = Lane(lane, lane_shape, allowed)
            else:  # this is a regular or internal lane
                geometry = Lane(lane, lane_shape, allowed, stl.getWidth(lane))

            if not allowed:
                # set is empty hence everything is allowed, add to all filters
                ped_lanes[lane] = geometry
                bike_lanes[lane] = geometry
                es_lanes[lane] = geometry
            else:
                # add to filters
                if 'pedestrian' in allowed:
                    ped_lanes[lane] = geometry
                    es_lanes[lane] = geometry
                if 'bicycle' in allowed:
                    bike_lanes[lane] = geometry
                    es_lanes[lane] = geometry
                if 'car' in allowed:
                    # ES are not supposed to be there, but they still do. so here they are...
                    es_lanes[lane] = geometry

        self.lanes = {
            TYP_PED: ped_lanes,
            TYP_BK: bike_lanes,
            TYP_ES: es_lanes,
        }

        # merge all lanes to create a polygon playground for each vehicle type:
        zone = {v_type: unary_union(tuple(v_lanes.values())) for v_type, v_lanes in self.lanes.items()}

        # generating (start,end) tuples from exterior and interior geometry of the polygon
        # where the v_type is allowed to pass to the obstacle generator
        self.env_states = {v_type: scene.EnvState(
            chain(pairwise(poly.exterior.coords), pairwise([holes.coords for holes in chain(poly.interiors)]))) for
            v_type, poly in zone.items()}
        # register size of map for desire force computation later
        self.bounds = unary_union(tuple(zone.values())).bounds

        # 2) generate vehicles inside SFM sim from SUMO

        self.vehicles = OrderedDict()
        vehicle_id_list: list[str] = self.traci.vehicle.getIDList()
        self.state = np.zeros((len(vehicle_id_list), 9))
        state_id = 0
        for veh_id in vehicle_id_list:
            self.vehicles[veh_id] = SumoVehicle(self, veh_id, state_id)
            state_id += 1

        self.SFM_sim = Simulator()  # TODO: fill args for SFM

    def sim_step(self):
        pass  # TODO: do the step function


class SumoVehicle:
    def __init__(self, sfm_sim: SFMsim, veh_id: str, state_id: int):
        self.cur_lane: str = None
        self.cur_edge: str = None
        self.desire_queue: list[DesirePoint] = []

        self.sim = sfm_sim
        self.tc_veh = sfm_sim.traci.vehicle
        self.vehID = veh_id
        self.state_ref = self.sim.state[state_id] # [x, y, sx, sy, goalx, goaly, tau, typ, exists]

        v_type_id = self.tc_veh.getTypeID(self.vehID)
        self.state_ref[6] = self.tc_veh.getTau(v_type_id)

        shape = self.tc_veh.getShapeClass(v_type_id)
        if 'scooter' == shape:
            self.vType = TYP_ES
        elif 'bicycle' == shape:
            self.vType = TYP_BK
        elif 'pedestrian' == shape:
            self.vType = TYP_PED
        else:
            self.vType = TYP_UNK
        self.state_ref[7] = self.vType

        self.departed = False
        self.state_ref[8] = 0

        if self.vType:  # only do desire points if this is a vehicle we manage (not TYP_UNK)
            self.route = self.tc_veh.getRoute(self.vehID) or []
            self.generate_desire_queue()

    def step(self):
        if not self.departed and self.tc_veh.getRouteIndex(self.vehID) > -1:
            self.departed = True
            self.state_ref[8] = 1  # tel the simulator the vehicle is to be moved around
            self.state_ref[0:2] = np.array(self.sumo_xy)  # postion the vehicle
            self.state_ref[2:4] = (0, 0)  # TODO : set non null starting speed
            self.state_ref[4:6] = self.desire_queue[-1].coord

        self.tc_veh.moveToXY(
            self.vehID,
            self.desire_queue[-1].edge,
            self.desire_queue[-1].lane,
            self.state_ref[0], self.state_ref[1],
            angle=-1073741824.0,
            keepRoute=1,
            matchThreshold=100,
        )
        self.refresh_position()

    def next_goal(self):
        if self.desire_queue:  # not empty yet
            self.desire_queue.pop()
            if self.desire_queue:  # not empty yet
                self.state_ref[4:6] = self.desire_queue[-1].coord



    def refresh_position(self):
        self.cur_edge = self.tc_veh.getRoadID(self.vehID)
        self.cur_lane = self.tc_veh.getLaneID(self.vehID)

    def generate_desire_queue(self):
        # this dict wil be of use collect lanes while only going through all
        # lanes only once. this is important because there can be a lot there.
        available_lanes_on_edge = {edge: [] for edge in self.route}
        usable_lanes = self.sim.lanes[self.vType]

        for lane_id in usable_lanes.keys():
            for edge in self.route:
                if edge in lane_id:
                    # for each edge of the route we collecte lanes which can be used by our vehicle
                    available_lanes_on_edge[edge].append(lane_id)

        queue = []  # positions must be retrievable in order from start to finish
        # now sort lanes so the higher number lane is at the bottom of the list;
        # this allows to always select the most external lane.
        # then extract shape waypoints from the lane extend Point queue.
        for edge in self.route:
            available_lanes_on_edge[edge].sort()
            lane_to_use = available_lanes_on_edge[edge][-1]
            # here route are only comprised of lanes and no walking areas
            for point in usable_lanes[lane_to_use].line.coords:
                queue.append(DesirePoint(point, lane_to_use, edge))

        # reverse to be able to pop() postions
        queue.reverse()

        # save the result
        self.desire_queue = queue

    @property
    def sumo_xy(self) -> tuple[float, float]:
        return self.tc_veh.getPosition(self.vehID)

    # TODO: Method to call and retrieve positions from SFM sim
