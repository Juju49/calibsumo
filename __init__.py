#!/python3

""" calibration and data extraction to test SUMO micro sim models for escooters and bikes

authors : J. MERLE-REMOND, Y. VALERO
date : 08/08/2022
"""
import os, sys
from time import strftime
import threading
from threading import Thread, Lock

import numpy as np

if __name__ == "__main__":
    np.seterr('raise')
    __path__ = [os.path.dirname(os.path.realpath(__file__))]

from . import simulation as sim
from . import build

""" Respecter l'ordre de définition du domaine :
             study_area[1] | study_area[2]
             --------------+--------------
             study_area[0] | study_area[3]"""
default_study_area = np.array(
    [
        (33.15, 6.0),  # P1
        (27.0, 15.03),  # P2
        (77.85, 45.0),  # ([177,55]) #P3
        (85.0, 38.61),  # ([180,-45]) #P4
    ]
)
MAX_STEP = 200

LOC_SUMO = 1
LOC_VTYP = 0

TYP_PED = 1
TYP_ES = 2
TYP_BK = 3

VAR_CONTINUOUS = True
VAR_DISCRETE = False

default_vars_list = [
    # SUMO
    sim.Variable(
        "--step-length",
        VAR_CONTINUOUS,
        (0.001, 10.0),
        location=LOC_SUMO,
        default=1.0,
    ),
    sim.Variable(
        "--lateral-resolution",
        VAR_CONTINUOUS,
        (0.1, 1.5),
        location=LOC_SUMO,
        default=0.5,
    ),
    # ES exclusive
    sim.Variable(
        "maxSpeed",
        VAR_CONTINUOUS,
        (1.0, 20.0),
        location=LOC_VTYP,
        default=8.9,
        subset={TYP_ES},
    ),
    sim.Variable(
        "vClass",
        VAR_DISCRETE,
        ("bicycle", "pedestrian"),
        location=LOC_VTYP,
        default="bicycle",
        subset={TYP_ES},
    ),
    sim.Variable(
        "latAlignment",
        VAR_DISCRETE,
        ("right",),
        location=LOC_VTYP,
        default="right",
        subset={TYP_ES},
    ),
    sim.Variable(
        "accel",
        VAR_CONTINUOUS,
        (0.1, 10.0),
        location=LOC_VTYP,
        default=2.0,
        subset={TYP_ES},
    ),
    sim.Variable(
        "decel",
        VAR_CONTINUOUS,
        (0.1, 10.0),
        location=LOC_VTYP,
        default=4.0,
        subset={TYP_ES},
    ),
    sim.Variable(
        "emergencyDecel",
        VAR_CONTINUOUS,
        (0.1, 15.0),
        location=LOC_VTYP,
        default=7.0,
        subset={TYP_ES},
    ),
    sim.Variable(
        "minGap",
        VAR_CONTINUOUS,
        (0.1, 10.0),
        location=LOC_VTYP,
        default=1.50,
        subset={TYP_ES},
    ),
    sim.Variable(
        "sd",
        VAR_CONTINUOUS,
        (0.0, 1.0),
        location=LOC_VTYP,
        default=0.9,
        subset={TYP_ES},
    ),
    sim.Variable(
        "tau",
        VAR_CONTINUOUS,
        (0.01, 5.0),
        location=LOC_VTYP,
        default=1.0,
        subset={TYP_ES},
    ),
    # Bike Exclusive
    sim.Variable(
        "maxSpeed",
        VAR_CONTINUOUS,
        (1.0, 20.0),
        location=LOC_VTYP,
        default=5.56,
        subset={TYP_BK},
    ),
    sim.Variable(
        "sd",
        VAR_CONTINUOUS,
        (0.0, 1.0),
        location=LOC_VTYP,
        default=0.9,
        subset={TYP_BK},
    ),
    sim.Variable(
        "tau",
        VAR_CONTINUOUS,
        (0.01, 5.0),
        location=LOC_VTYP,
        default=1.0,
        subset={TYP_BK},
    ),
    # Bike and ES
    sim.Constant(
        "carFollowModel",
        location=LOC_VTYP,
        default="IDM",
        subset={TYP_ES, TYP_BK},
    ),
]

default_src_folder = "Cycle-Lane-Pedestrians-Crossing"
default_start = "E:\\Garyfallia\\"
default_output_dir = r"E:\PatrasCalibsumo"

# display exceptions correctly:
__th_exc_lock = Lock()


def __threadingException(args):
    with __th_exc_lock:
        threading.__excepthook__(args)


threading.excepthook = __threadingException


def createDefaultExperience(folder, output_folder=default_output_dir, use_sql=False):
    default_xlsx = r"\DataBaseEXCEL\output.xlsx"
    default_sqlite = r"\DataFromTIAS\data.sqlite"

    # data to use
    data_path = default_start + folder
    if use_sql:
        data_path += default_sqlite
    else:
        data_path += default_xlsx

    # bike lane width and pedestrian crossing depending on scenario

    if folder in {"Cycle-Lane-Pedestrians-Crossing"}:
        bk_lane_size = 1.50
        crossing_dist = 30
    elif folder in {"Road-Pedestrians-Crossing-1"}:
        bk_lane_size = 3.0
        crossing_dist = 30
    else:
        raise ValueError("unknown folder")

    # create experiment instance
    exp = sim.Experience(
        os.path.join(output_folder, folder),  # path where the network file and iterations will be stored
        study_area=default_study_area,  # where to cut trajectories when using sql mostly
        vTypes_to_calibrate={TYP_ES, TYP_BK, TYP_PED},  # used to restrict to certain vehicle types to run the
        # calibration algorithm on
    )
    # load experiment data from file
    exp.chargement_donnees(data_path)

    build.createNet(  # creates + shaped networks with bike lane going upwards and an horizontal pedestrian path with
        # a crosswalk in the center
        os.path.join(exp.directory, f"network.net.xml"),  # place to put the network file
        bike_lane_width=bk_lane_size,
        crossing_location=crossing_dist,
    )

    if use_sql:
        exp.suppressions_objets_hors_cadre()
        exp.retraitement_donnees()
        exp.application_filtre()

    return exp


##########
# Programme optimisation
##########


def calibrate(experiences, variable_list, directory, traci_step_function=lambda conn: None):
    # create thread to kill the process when someone writes "stop" to the console
    stop_iterating = False

    # register experiments
    exp_set = experiences
    my_dir = os.path.join(directory, f"calibrate_{strftime('%Y-%m-%d_%H-%M-%S')}")

    # create parameters
    params = sim.Parameters(
        sim_step_traci_function=traci_step_function, variables=variable_list
    )

    # critère d'arrêt
    erreur_globale = 0
    tol = 0.01

    # compteur
    step = 0

    # nombre de (méta-)simulations à chaque itération
    N = 20

    # Ensemble de (méta-)simulations
    l_Meta_simulations = []

    # parametre algo CE
    rho = 0.75

    # pour suivi et tracé des variations des parametres
    l_memoire_parametres = [params]
    l_memoire_err = []

    erreur_globale_new = 1  # pour rentrer dans la boucle

    while (
            (step < MAX_STEP) or
            (abs(erreur_globale - erreur_globale_new) / max(erreur_globale, erreur_globale_new) > tol)
    ) and not stop_iterating:

        print("\n \n nouvelle itération : n°", step)
        # print('para input : ', params)
        erreur_globale = erreur_globale_new

        # génération des jeux de paramètre (on ajoute le param de base pour tester)
        jeux = params.newParametersSet(max(1, N - len(l_Meta_simulations) - 1))
        params.applyVars()
        jeux.append(params)
        # print('nouveau tirages :' ,jeux)

        # réalisation des Simulations
        folder = os.path.join(my_dir, f"step{step}")
        threadsToMonitor = []
        i = 0
        for jeu in jeux:
            metasim = sim.Meta_Simulation(exp_set, jeu, folder, i)
            l_Meta_simulations.append(metasim)
            t = Thread(
                target=lambda: metasim.seFaire(),
                name=f"tmeta{metasim.id}",
            )
            threadsToMonitor.append(t)
            t.start()
            i += 1
        try:
            print("-> Waiting for simulations, interruptions allowed for graceful early termination\n\n", end="")
            for t in threadsToMonitor:
                t.join()

        except KeyboardInterrupt:
            stop_iterating = True
            print("\n====!! STOP REQUEST REGISTERED !!====\n\n", end="")

        finally:
            for t in threadsToMonitor:
                t.join()
            print("\n  Simulations completed, interruptions not allowed for graceful termination\n", end="")

        # tri des simulations par err descroissantes
        l_Meta_simulations.sort(key=lambda x: x.err, reverse=False)

        # on jette les simulations les plus mauvaises
        l_Meta_simulations = l_Meta_simulations[: max(int(len(l_Meta_simulations) * rho), 1)]
        print("number of metasims kept:", len(l_Meta_simulations))
        # restimation des parametres #>> à faire

        params_new = sim.Parameters.parametersFromMetaSims(l_Meta_simulations)
        print(
            "\n: nouveau jeu de parametres: (en moyennes et écart-types) ",
            params_new,
        )

        # calcul erreur globale moyenne
        nn = 0
        erreur_globale_new = 0
        for meta_simul in l_Meta_simulations:
            erreur_globale_new += meta_simul.err
            nn += 1
        erreur_globale_new = erreur_globale_new / nn
        print(" \n erreur globale", erreur_globale_new)

        # actualisation compteur
        step += 1
        params = params_new

        # sauvegarde données intermédiaires
        l_memoire_err.append(erreur_globale_new)
        l_memoire_parametres.append(params_new)

    results = {
        "step": step,
        "memparams": l_memoire_parametres,
        "memerr": l_memoire_err,
        "metasim": l_Meta_simulations[0],
        "folder": my_dir,
    }
    saveData(results)
    return results


def saveData(calibrated):
    directory = calibrated["folder"]
    import matplotlib.pylab as plt
    if not os.path.exists(directory):
        os.makedirs(directory)

    plt.title("Evolution of RMSE during calibration")
    plt.xlabel("step")
    plt.ylabel("RMSE")
    plt.plot(np.arange(len(calibrated["memerr"])), calibrated["memerr"])
    plt.savefig(os.path.join(directory, "errorProgress.png"))
    plt.close()

    calibrated["memparams"][-1].dumpAsCSVfile(os.path.join(directory, "calibratedParams.csv"),
                                              calibrated["metasim"].parametres,
                                              nbSteps=calibrated["step"],
                                              RMSE=calibrated["memerr"][-1],
                                              )


if __name__ == "__main__":
    path = default_output_dir + r"\accelDecelEmgSpeedGap1"
    exp_set = set()

    exp_set.add(createDefaultExperience(default_src_folder, path))
    exp_set.add(createDefaultExperience("Road-Pedestrians-Crossing-1", path))

    myVars2 = [
        sim.Variable(
            "--step-length",
            VAR_CONTINUOUS,
            (0.01, 2.0),
            location=LOC_SUMO,
            default=1.0,
        ),
        sim.Variable(
            "--lateral-resolution",
            VAR_CONTINUOUS,
            (0.1, 1.5),
            location=LOC_SUMO,
            default=0.5,
        ),
    ]

    rivoliVars = [
        sim.Constant(
            "--step-length",
            location=LOC_SUMO,
            default=0.05,
        ),
        sim.Constant(
            "--lateral-resolution",
            location=LOC_SUMO,
            default=0.257,
        ),

        # ES exclusive
        sim.Variable(
            "maxSpeed",
            VAR_CONTINUOUS,
            (1.0, 20.0),
            location=LOC_VTYP,
            default=10.57,
            subset={TYP_ES},
        ),
        sim.Variable(
            "accel",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=1.73,
            subset={TYP_ES},
        ),
        sim.Variable(
            "decel",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=2.9,
            subset={TYP_ES},
        ),
        sim.Variable(
            "emergencyDecel",
            VAR_CONTINUOUS,
            (0.1, 15.0),
            location=LOC_VTYP,
            default=7.44,
            subset={TYP_ES},
        ),
        sim.Variable(
            "minGap",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=0.1,
            subset={TYP_ES},
        ),
        # Bike Exclusive
        sim.Variable(
            "maxSpeed",
            VAR_CONTINUOUS,
            (1.0, 20.0),
            location=LOC_VTYP,
            default=8.01,
            subset={TYP_BK},
        ),
        sim.Variable(
            "accel",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=1.50,
            subset={TYP_BK},
        ),
        sim.Variable(
            "decel",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=2.62,
            subset={TYP_BK},
        ),
        sim.Variable(
            "emergencyDecel",
            VAR_CONTINUOUS,
            (0.1, 15.0),
            location=LOC_VTYP,
            default=6.94,
            subset={TYP_BK},
        ),
        sim.Variable(
            "minGap",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=1.16,
            subset={TYP_BK},
        ),
        # Pedestrian Exclusive
        sim.Variable(
            "maxSpeed",
            VAR_CONTINUOUS,
            (1.0, 20.0),
            location=LOC_VTYP,
            default=5.06,
            subset={TYP_PED},
        ),
        sim.Variable(
            "accel",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=0.16,
            subset={TYP_PED},
        ),
        sim.Variable(
            "decel",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=7.60,
            subset={TYP_PED},
        ),
        sim.Variable(
            "emergencyDecel",
            VAR_CONTINUOUS,
            (0.1, 15.0),
            location=LOC_VTYP,
            default=4.57,
            subset={TYP_PED},
        ),
        sim.Variable(
            "minGap",
            VAR_CONTINUOUS,
            (0.1, 10.0),
            location=LOC_VTYP,
            default=0.18,
            subset={TYP_PED},
        ),
    ]

    # calibrated = calibrate(exp_set, default_vars_list, path)
    # calibrated = calibrate(exp_set, myVars2, path)
    calibrated = calibrate(exp_set, rivoliVars, path)

