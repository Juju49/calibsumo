import sqlite3 as lite
import numpy as np


def chargement_donnees(exp, nom_base_donnees="laurier.sqlite"):
    """initialisation : chargement données et initialisation infos environnement"""

    print("début chargement données")

    # chargement infos brutes
    conn = lite.connect(nom_base_donnees)
    cur = conn.cursor()

    # cur.execute('SELECT * FROM objects')
    # rows_objects = cur.fetchall()
    # print(rows_objects)

    cur.execute("SELECT * FROM objects_features")
    if NNN is None:
        rows_objects_features = np.array(cur.fetchall())
    else:
        rows_objects_features = np.array(cur.fetchmany(size=NNN))

    cur.execute("SELECT * FROM positions")
    if NNN is None:
        rows_positions = np.array(cur.fetchall())
    else:
        rows_positions = np.array(cur.fetchmany(size=NNN))

    cur.execute("SELECT * FROM velocities")
    if NNN is None:
        rows_velocities = np.array(cur.fetchall())
    else:
        rows_velocities = np.array(cur.fetchmany(size=NNN))

    conn.close()

    print("tailles données :", np.size(rows_positions), np.size(rows_velocities))

    # actualisation infos sur l'expérience
    exp.xmin = min(rows_positions[:, 2])
    exp.xmax = max(rows_positions[:, 2])
    exp.ymin = min(rows_positions[:, 3])
    exp.ymax = max(rows_positions[:, 3])
    exp.f_max = int(max(rows_positions[:, 1])) + 1
    print("dimensions domaine détecté :", exp.xmin, exp.xmax, exp.ymin, exp.ymax)

    # ajouts des trajectoires (plusieurs trajectoires par objet)
    traj_start_end_dict = {}
    id_trajectoire_courant = -1
    time = None
    pos = None
    spd = None
    f_start = 0
    f_num = 0
    j = 0  # indice de parcours de la liste des vitesses
    for i in range(np.shape(rows_positions)[0]):
        if rows_positions[i, 0] != id_trajectoire_courant:
            # finir la trajectoire precedente
            if id_trajectoire_courant != -1:
                # on enregistre le début et la fin pour apperer les objets
                traj_start_end_dict[id_trajectoire_courant] = (f_start, f_num)

                # creation de la trajectoire
                mobile = exp.observationsTerrain[str(id_trajectoire_courant)]
                f_num += 1  # pour que le slice marche bien
                mobile["t"] = time[
                    f_start:f_num
                ]  # [premier frame: dernier frame] de la trajectoire
                mobile["x"] = pos[:, 0][f_start:f_num]
                mobile["y"] = pos[:, 1][f_start:f_num]

                # on vérifie cohérence des vitesses
                if spd.sum() == 0:
                    P2 = pos[f_start + 1 : f_num + 1, :]
                    P1 = pos[f_start:f_num, :]
                    spd[f_start:f_num, :] = (1 / exp.f_duration) * (P2 - P1)
                mobile["sx"] = spd[:, 0][f_start:f_num]
                mobile["sy"] = spd[:, 1][f_start:f_num]

            f_start = int(rows_positions[i, 1])

            time = np.zeros((exp.f_max, 1))
            pos = np.zeros((exp.f_max, 2))
            spd = np.zeros((exp.f_max, 2))

            time[f_start, :] = f_start * exp.f_duration
            pos[f_start, :] = rows_positions[i, 2:]
            spd[f_start, :] = rows_velocities[j, 2:]

            id_trajectoire_courant = rows_positions[i, 0]
            # Rq : ici on a fait le choix d'approcher la vitesse à t par la vitesse à t+dt
        else:
            f_num = int(rows_positions[i, 1])
            time[f_num, :] = f_num * exp.f_duration
            pos[f_num, :] = rows_positions[i, 2:]
            spd[f_num, :] = rows_velocities[j, 2:]
            j += 1

    # regroupement des trajectoires par objets et création des objets

    for k in range(rows_objects_features[-1, 0]):
        # pour chaque objet, on choisit la trajectoire avec la plus grande persistence
        l_trajectoires = list(
            rows_objects_features[rows_objects_features[:, 0] == k, 1]
        )
        duree_max = 0
        id_traj_max = None
        for id_traj in l_trajectoires:
            if id_traj in traj_start_end_dict:
                duree = (
                    traj_start_end_dict[id_traj][1] - traj_start_end_dict[id_traj][0]
                )
                if duree > duree_max:
                    duree_max = duree
                    id_traj_max = id_traj
        if traj_max is not None:
            exp.ajout_objet(
                traj_start_end_dict[id_traj_max][0],
                exp.observationsTerrain[str(id_traj_max)],
            )
            self.objets[-1].id = id_traj_max
            self.objets[-1].f_fin = traj_start_end_dict[id_traj_max][1]
