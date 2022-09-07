import numpy as np
import pandas as pd

translateType = {0: 1,  # pedestrian
                 1: 3,  # bike
                 80: 2}  # ES


def chargement_donnees(exp, xlsx_path="output.xlsx"):
    """initialisation : chargement données et initialisation infos environnement"""

    def correct_x_from_patras(x):
        return exp.lane_size - float(x)

    data = pd.read_excel(xlsx_path)
    data = data[data["y_projected_smoothing"] < 40]
    data = data[data["y_projected_smoothing"] > 5]

    for_plot = np.array(data.groupby(["ID_Veh"]).count()["Frame"].sort_values().index)
    for_plot_final = for_plot[:]
    objets_position = data.groupby(["ID_Veh"])

    # defaultdict for outputting clean data to compare with sumo
    xlsx_data = exp.observationsTerrain

    for name, group in objets_position:
        if name in for_plot_final:
            # find first useful data point:
            first_frame = group["speed"].first_valid_index()
            last_frame = group["y_projected_smoothing"].last_valid_index()
            mobile = xlsx_data[name]

            # ===================build xlsx_data
            try:
                # time
                mobile["t"] = np.array(
                    tuple(map(float, group["Time"][range(first_frame, last_frame + 1)]))
                )

                # at some point: extract sx and sy too later but not needed

                # speed
                mobile["s"] = np.array(
                    tuple(map(float, group["speed"][range(first_frame, last_frame + 1)]))
                )

                # x position
                mobile["x"] = np.array(
                    tuple(map(
                        correct_x_from_patras,
                        group["x_projected_smoothing"][
                            range(first_frame, last_frame + 1)
                        ],
                    ))
                )
                exp.xmin = min(exp.xmin, mobile["x"].min())
                exp.xmax = max(exp.xmax, mobile["x"].max())

                # y position
                mobile["y"] = np.array(
                    tuple(map(
                        float,
                        group["y_projected_smoothing"][
                            range(first_frame, last_frame + 1)
                        ],
                    ))
                )
                exp.ymin = min(exp.ymin, *mobile["y"])
                exp.ymax = max(exp.ymax, *mobile["y"])

                # acceleration
                mobile["a"] = np.array(
                    tuple(map(float, group["accel"][range(first_frame, last_frame + 1)]))
                )
                if pd.isna(mobile["a"][0]):
                    mobile["a"][0] = mobile["a"][1]

                # ajout de l'objet trajectoire nommé
                exp.ajout_objet(first_frame, mobile)
                ob = exp.objets[-1]
                ob.id = int(name)
                ob.f_fin = last_frame
                ob.type = translateType[group["Type"][first_frame]]

                exp.f_max = max(exp.f_max, last_frame)
            except KeyError as e:
                print(e)

    # actualisation infos sur l'expérience
    exp.f_max = int(exp.f_max) + 1
    print("dimensions domaine détecté :", exp.xmin, exp.xmax, exp.ymin, exp.ymax)
