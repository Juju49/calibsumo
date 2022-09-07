import os
import matplotlib.pyplot as plt


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
