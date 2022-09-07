# -*- coding: utf-8 -*-
"""
Created on Fri May  6 18:01:56 2022

@author: valero, merle-remond
"""

# builtins
import xml.etree.ElementTree as ET

# import optparse
import sys
import os

# import subprocess
from collections import defaultdict
import random as rnd

# major modules
import pandas as pd
import numpy as np

# from scipy import interpolate
# from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# SUMO
if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from sumolib import checkBinary
from sumolib.xml import parse_fast_nested
from sumolib.miscutils import uMin, uMax, parseTime


# data from https://sumo.dlr.de/docs/Vehicle_Type_Parameter_Defaults.html sauf vitesse
base_routes_xml = """<?xml version="1.0" encoding="UTF-8"?>

<!-- generated on 2022-06-22 16:14:50 by Eclipse SUMO netedit Version v1_13_0+0805-082359713bb
-->

<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
    <!-- VTypes -->
    <!-- <vType id="myESped" length="1.10" minGap="1.50" maxSpeed="12.50" speedFactor="normc(1.00,0.30,0.20,2.00)" vClass="pedestrian" emissionClass="Zero/default" guiShape="scooter" width="0.40" height="1.20" color="green" personCapacity="1" latAlignment="right" laneChangeModel="SL2015" jmDriveAfterYellowTime="3600.0" jmDriveAfterRedTime="200.0" jmIgnoreKeepClearTime="0.0" accel="2.0" decel="4.0" emergencyDecel="7.0" sd="0.9" tau="1.0"/> -->
</routes>
"""

BK_attrib = {"id": "0", "vClass": "bicycle", "color": "red", "maxSpeed": "5.56"}

ES_attrib = {
    "id": "0",
    "vClass": "bicycle",
    "color": "red",
    "maxSpeed": "8.9",
    "length": "1.10",
    "minGap": "1.50",
    "guiShape": "scooter",
    "width": "0.40",
    "height": "1.20",
    "color": "green",
    "accel": "2.0",
    "decel": "4.0",
    "emergencyDecel": "7.0",
}


def generate_rou_xml_from_xlsx(
    xlsx_path, rou_xml_path, lane_size, do_plot=False, bikeMode=0.5
):
    def correct_x_from_patras(x):
        return lane_size - float(x)

    root = ET.fromstring(base_routes_xml)

    data = pd.read_excel(xlsx_path)
    data = data[data["y_projected_smoothing"] < 40]
    data = data[data["y_projected_smoothing"] > 5]

    for_plot = np.array(data.groupby(["ID_Veh"]).count()["Frame"].sort_values().index)
    # for_plot_final=for_plot[-5:-1]
    for_plot_final = for_plot[:]
    objets_position = data.groupby(["ID_Veh"])

    # list for sorting vehicles and person per depart time
    vehicles_list = []

    # defaultdict for outputting clean data to compare with sumo
    xlsx_data = defaultdict(lambda: defaultdict(list))

    for name, group in objets_position:
        if name in for_plot_final:
            # find first useful data point:
            first_frame = group["speed"].first_valid_index()
            last_frame = group["y_projected_smoothing"].last_valid_index()

            # ===================build xlsx_data
            # disabled stuff to get interpolated data:

            # STEP = 30
            # if STEP * 3 < group.last_valid_index() - group.first_valid_index():
            # itere=0
            # modified=[]
            # for y,x in zip(group['y_projected_smoothing'],group['x_projected_smoothing']):
            #    if itere % STEP == 0:
            #        modified.append([x,y+float(np.random.normal(0, 0.1, 1)*.1)])
            # print(str(x),str(y))
            #    itere+=1
            # else: #append endpoint
            #    modified.append([group['x_projected_smoothing'].iloc[-1]+.02, group['y_projected_smoothing'].iloc[-1]])
            # arr=np.array(modified)
            # x, y = zip(*arr)

            # create spline function
            # f, u = interpolate.splprep((x,y,), s=0, per=False)

            # create interpolated lists of points
            # xint, yint = interpolate.splev(np.linspace(0, 1, 100), f)
            # w = savgol_filter(yint, 11, 2)

            # find first simulated point
            # start_y_pos = (w+0.15)[first_frame - group.first_valid_index()]

            if do_plot:
                # make the data for later plots

                try:
                    xlsx_data[name]["t"].extend(
                        map(float, group["Time"][range(first_frame, last_frame)])
                    )
                    xlsx_data[name]["s"].extend(
                        map(float, group["speed"][range(first_frame, last_frame)])
                    )
                    xlsx_data[name]["x"].extend(
                        map(
                            correct_x_from_patras,
                            group["x_projected_smoothing"][
                                range(first_frame, last_frame)
                            ],
                        )
                    )
                    xlsx_data[name]["y"].extend(
                        map(
                            float,
                            group["y_projected_smoothing"][
                                range(first_frame, last_frame)
                            ],
                        )
                    )
                    xlsx_data[name]["a"].extend(
                        map(float, group["accel"][range(first_frame, last_frame)])
                    )
                    if pd.isna(xlsx_data[name]["a"][0]):
                        xlsx_data[name]["a"][0] = xlsx_data[name]["a"][1]
                except KeyError as e:
                    print(e)
            # ===================build rou.xml
            # get start postions
            start_x_pos = correct_x_from_patras(
                group["x_projected_smoothing"][first_frame]
            )
            start_y_pos = group["y_projected_smoothing"][first_frame]
            # cap max speed to vType max speed to avoid crashes
            speed = group["speed"][first_frame]

            # print(f"veh {name} start y pos : {start_y_pos}")
            # print(persf"veh {name} :\n {pers.attrib}")

            if start_y_pos < 20:  # it is a bike or ES
                print(f"   veh {name} is Bike or ES")
                pers = ET.Element("trip")
                # if too fast for a bike, it can only be an ES
                maxSpeedReached = max(xlsx_data[name]["s"])
                maxAccelReached = max(max(xlsx_data[name]["a"]), 0.1)
                maxDecelReached = max(abs(min(xlsx_data[name]["a"])), 0.1)

                if rnd.triangular(0.0, 1.0, bikeMode) > 0.5:
                    vehtype = f"BK{name}"
                    myveh = ET.SubElement(root, "vType", attrib=BK_attrib)
                else:
                    vehtype = f"ES{name}"
                    myveh = ET.SubElement(root, "vType", attrib=ES_attrib)
                pers.set("type", vehtype)
                myveh.set("id", vehtype)
                myveh.set("maxSpeed", str(maxSpeedReached))
                myveh.set("accel", str(maxAccelReached))
                myveh.set("decel", str(maxDecelReached))

                # pers.set('fromXY',f'{start_x_pos},{start_y_pos}')
                pers.set("departPos", str(start_y_pos))
                pers.set("departSpeed", str(speed))
                pers.set("arrivalPos", "10")
                pers.set("from", "E0")
                pers.set("to", "E1")

            else:  # it is a pedestrian
                pers = ET.Element("person")
                mywalk = ET.SubElement(pers, "walk")
                mywalk.set("speed", str(group["speed"][first_frame]))

                if start_x_pos < 0:  # going left to right
                    mywalk.set(
                        "departPosLat", str(30 - start_y_pos)
                    )  # positive is right side in walking direction
                    mywalk.set("from", "E2")
                    mywalk.set("to", "E3")
                    pers.set("departPos", "0")
                    mywalk.set("arrivalPos", "10")

                else:  # going right to left
                    mywalk.set(
                        "departPosLat", str(start_y_pos - 30)
                    )  # positive is right side in walking direction
                    mywalk.set("from", "E3")
                    mywalk.set("to", "E2")
                    pers.set("departPos", "10")
                    mywalk.set("arrivalPos", "0")
                print(f"   veh {name} is pedestrian")

            # assign the same id from tracking phase
            pers.set("id", str(name))

            # get when the person is inserted into the flow
            pers.set("depart", str(group["Time"][first_frame]))

            # save for sort
            vehicles_list.append(pers)

    # insert ordered by depart
    vehicles_list.sort(key=lambda e: float(e.attrib["depart"]))
    root.append(ET.Comment(text="Vehicles, persons and containers (sorted by depart)"))
    root.extend(vehicles_list)

    # write rou.xml file to scenario
    tree = ET.ElementTree(element=root)
    ET.indent(tree)
    with open(rou_xml_path, mode="wb") as out_file:
        tree.write(out_file, encoding="utf-8")
    return xlsx_data


def loadFcd(fcd_file):
    element = "vehicle"
    location = "lane"

    routes = defaultdict(list)  # vehID -> recorded edges
    # vehID -> (times, speeds, distances, accelerations, angles, xPositions, yPositions, kilometrage)
    attrs = ["id", "x", "y", "angle", "speed", location]

    fcd_data = defaultdict(lambda: defaultdict(list))

    totalVehs = 0
    filteredVehs = 0
    for timestep, vehicle in parse_fast_nested(
        fcd_file, "timestep", ["time"], element, attrs
    ):
        totalVehs += 1
        vehID = int(vehicle.id)
        edge = vehicle.lane[0 : vehicle.lane.rfind("_")]
        if len(routes[vehID]) == 0 or routes[vehID][-1] != edge:
            routes[vehID].append(edge)
        time = parseTime(timestep.time)
        speed = float(vehicle.speed)
        prevTime = time
        prevSpeed = speed
        prevDist = 0
        if vehID in fcd_data:
            prevTime = fcd_data[vehID]["t"][-1]
            prevSpeed = fcd_data[vehID]["s"][-1]
            prevDist = fcd_data[vehID]["d"][-1]
        fcd_data[vehID]["t"].append(time)
        fcd_data[vehID]["s"].append(speed)
        # fcd_data[vehID]['i'].append(float(vehicle.angle))
        fcd_data[vehID]["x"].append(float(vehicle.x))
        fcd_data[vehID]["y"].append(float(vehicle.y))
        if prevTime == time:
            fcd_data[vehID]["a"].append(0)
        else:
            fcd_data[vehID]["a"].append((speed - prevSpeed) / (time - prevTime))

        avgSpeed = speed

        fcd_data[vehID]["d"].append(prevDist + (time - prevTime) * avgSpeed)

        filteredVehs += 1
    if totalVehs == 0 or filteredVehs == 0:
        print(f"Found {totalVehs} datapoints in {fcd_file} and kept {filteredVehs}")

    assert filteredVehs > 0, "no vehicles!!"
    return fcd_data


def plot_vehicle_trajectory(vehID, fcd_data, xlsx_data, directory, lane_size=1.5):
    fig = plt.figure(figsize=(14, 9), dpi=100)

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(f"trajectory of vehicle {vehID}")
    plt.xlim(0.0, lane_size)
    plt.ylim(0.0, 40.0)

    plt.plot(fcd_data[vehID]["x"], fcd_data[vehID]["y"], label=f"{vehID} SUMO")
    plt.plot(xlsx_data[vehID]["x"], xlsx_data[vehID]["y"], label=f"{vehID} real")

    plt.legend()

    dir_for_plots = os.path.join(directory, "trajectories")

    if not os.path.isdir(dir_for_plots):
        os.mkdir(dir_for_plots)

    plt.savefig(os.path.join(dir_for_plots, f"{vehID}_sumo_vs_real_trajectory.png"))
    plt.close()


def plot_vehicle_speed(vehID, fcd_data, xlsx_data, directory, t_step):
    fig = plt.figure(figsize=(14, 9), dpi=100)

    plt.xlabel("t (s)")
    plt.ylabel("s (m.s-1)")
    plt.title(f"speed of vehicle {vehID}")

    # visual correction SUMO having a per second resolution
    move_x = (
        np.ceil(xlsx_data[vehID]["t"][0] / t_step) * t_step - xlsx_data[vehID]["t"][0]
    )
    xlsx_data[vehID]["t"][:] = map(lambda e: e + move_x, xlsx_data[vehID]["t"])

    plt.plot(fcd_data[vehID]["t"], fcd_data[vehID]["s"], label=f"{vehID} SUMO")
    plt.plot(xlsx_data[vehID]["t"], xlsx_data[vehID]["s"], label=f"{vehID} real")

    # draw time consistently
    xlim_left, xlim_right = plt.xlim()
    xlim_left = np.floor(xlim_left)
    xlim_right = np.ceil(xlim_right)
    plt.xlim(xlim_left, xlim_right)
    plt.xticks(np.arange(xlim_left, xlim_right + 1))

    plt.legend()

    dir_for_plots = os.path.join(directory, "speeds")

    if not os.path.isdir(dir_for_plots):
        os.mkdir(dir_for_plots)

    plt.savefig(os.path.join(dir_for_plots, f"{vehID}_sumo_vs_real_speed.png"))
    plt.close()


# exec script
if __name__ == "__main__":
    SIM_STEP = 0.5  # min = 0.001, default = 1.0
    SOURCE_XLSX = (
        r"D:\Garyfallia\Cycle-Lane-Pedestrians-Crossing\DataBaseEXCEL\output.xlsx"
    )
    OUTPUT_ROU_XML = r"D:\Patras01\Patras01_real_start.rou.xml"
    LANE_SIZE = 1.5

    # extract starting positions and data for later plots
    print("-> PATRAS DATA EXTRACTION")
    xlsx_data = generate_rou_xml_from_xlsx(
        SOURCE_XLSX, OUTPUT_ROU_XML, LANE_SIZE, do_plot=True
    )

    # prepare SUMO exec
    sumoBinary = checkBinary("sumo")
    routes = OUTPUT_ROU_XML
    directory = os.path.dirname(routes)
    fcd_file = os.path.join(directory, "Patras01.fcd.xml")
    net = os.path.join(directory, "Patras01.net.xml")

    # start sumo simulation
    print("-> SIMULATION START")
    traci.start(
        [
            sumoBinary,
            "--lateral-resolution",
            "0.5",
            "--fcd-output",
            f"{fcd_file}",
            "--step-length",
            f"{SIM_STEP}",
            "-r",
            f"{routes}",
            "-n",
            f"{net}",
        ]
    )
    # step through the simulation
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # no traci stuff to do but kept for future use
    sys.stdout.flush()
    traci.close()

    # load data from latest simulation
    fcd_data = loadFcd(fcd_file)

    # plot data on a per vehicle basis to compare real and simulated outputs
    print("-> PLOT DATA")
    for vehID in fcd_data:
        print(f"   drawing veh {vehID}")
        plot_vehicle_speed(vehID, fcd_data, xlsx_data, directory, SIM_STEP)
        plot_vehicle_trajectory(vehID, fcd_data, xlsx_data, directory, LANE_SIZE)

    print("-> DONE")
    os.system("pause")
