"""Define classes and process to run the SUMO simulation
"""
import os, sys
from collections import defaultdict
from threading import Thread, BoundedSemaphore
from itertools import cycle

import numpy as np

import matplotlib.pylab as plt
from matplotlib.animation import FuncAnimation


# SUMO
if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from . import build as build
from sumolib import checkBinary
from sumolib.xml import parse_fast_nested
from sumolib.miscutils import uMin, uMax, parseTime

RNG = np.random.default_rng()
PORT = cycle(range(5350, 5390))
__PORT_SEM = BoundedSemaphore()
def getPort():
    with __PORT_SEM:
        p = next(PORT)
        #print(f"Port selected : {p}\n", end="")
    return p
    
VERBOSE_SUMO = False #print sumo log to console when there is no fatal errors ?
MAX_SIM_SEMAPHORE = BoundedSemaphore(value=5) #limit number of sumo instance because else TraCI freaks out

######################
# Créations classes d'objets
######################

# (sécurité) Limite MAX lecture données >> à faire sauté si exploitations bcp de données
NNN = 1000000  # mettre None si pas de limite voulue
GLUE_DIST = 1  # distance max acceptée pour recollage #>>>>A REGLER


class Experience:
    """Objet contenant toutes les données de l'experience"""

    def __init__(
        self,
        working_directory,
        study_area=np.array(((0, 0), (0, 0), (0, 0), (0, 0))),
        tolerated_distance_around_area=np.array((1, 1)),
        lane_size=1.5,
        fps=25,
        vTypes_to_calibrate={2, 3},
    ):
        self.id = RNG.integers(0, 2147483647)

        # liste des objets (piétons, trotti...)
        self.observationsTerrain = build.VehicleContainer()
        self.objets = []

        # infos temps expériences
        # self.temps_max=0
        # self.temps_min=0
        self.f_max = 0  # num dernière frame

        # images par seconde
        self.f_duration = 1 / fps

        # infos géométrie domaine
        self.xmin = study_area[:, 0].min()
        self.xmax = study_area[:, 0].max()
        self.ymin = study_area[:, 1].min()
        self.ymax = study_area[:, 1].max()
        self.domain = study_area
        self.glue_distance = tolerated_distance_around_area

        self.glue_area = study_area.copy() + np.array(
            (
                tolerated_distance_around_area,
                (tolerated_distance_around_area[0], -tolerated_distance_around_area[1]),
                -tolerated_distance_around_area,
                (
                    -tolerated_distance_around_area[0],
                    -tolerated_distance_around_area[1],
                ),
            )
        )

        self.lane_size = lane_size
        self.calibrate_vtypes = set(vTypes_to_calibrate)
        self.directory = working_directory
        
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        print(f"-> EXPERIMENT {self.id} CREATED")

    def chargement_donnees(self, chemin, determine_type_func=None):
        if "xls" in chemin:
            print("--> LOAD XLSX", chemin)
            from . import input_xlsx as iXlsx

            iXlsx.chargement_donnees(self, chemin)
        elif "sql" in chemin:
            print("-> LOAD SQL", chemin)
            from . import input_sql as iSql

            iSql.chargement_donnees(self, chemin)
        else:
            raise NotImplementedError(
                "no file handler found for this input file format."
            )
        print("   Determine vehicle types")
        # determination du type des objets
        for obj in self.objets:
            obj.determine_type(determine_type_func)

    # Ne servira plus jamais
    # def recalcul_vitesses(self):
    # """recalcul des vitesses car vitesses logiciel pas fiable"""
    # for objet in self.objets:
    # objet.recalcul_vitesses(self.f_duration)

    # Ne servira plus jamais
    # def ajout_trajectoire(self,f_start):
    # self.trajectoires.append(Objet(self,f_start))
    # self.trajectoires[-1].id=len(self.trajectoires)-1

    def ajout_objet(self, f_start, mobile):
        self.objets.append(Objet(self, f_start, mobile))
        self.objets[-1].id = len(self.objets) - 1

    def retraitement_donnees(self):
        """fait du recollage d'objets"""

        print(f"-> EXP {self.id} : data treatment")

        # recollage des objets
        l_objets_a_recoller_type1 = []
        for objet in self.objets:

            # detection des apparitions suspectes
            x_obj = objet.mobile["x"][0]
            y_obj = objet.mobile["y"][0]

            test1 = not aboveLine(x_obj, self.glue_area[0], self.glue_area[1])[
                0
            ]  # (x_obj>XLIM_MIN)
            test2 = aboveLine(x_obj, self.glue_area[2], self.glue_area[3])[
                0
            ]  # (x_obj<XLIM_MAX)
            if test1 and test2:
                l_objets_a_recoller_type1.append(objet)
                continue

            test3 = aboveLine(y_obj, self.glue_area[1], self.glue_area[2])[
                1
            ]  # (y_obj>YLIM_MIN)
            test4 = not aboveLine(y_obj, self.glue_area[3], self.glue_area[0])[
                1
            ]  # (y_obj<YLIM_MAX)
            if test3 and test4:
                l_objets_a_recoller_type1.append(objet)
                continue

        l_objets_a_recoller_type2 = []
        for objet in self.objets:

            # detection des disparitions suspectes
            x_obj = objet.mobile["x"][-1]
            y_obj = objet.mobile["y"][-1]

            test1 = not aboveLine(x_obj, self.glue_area[0], self.glue_area[1])[
                0
            ]  # (x_obj>XLIM_MIN)
            test2 = aboveLine(x_obj, self.glue_area[2], self.glue_area[3])[
                0
            ]  # (x_obj<XLIM_MAX)
            if test1 and test2:
                l_objets_a_recoller_type2.append(objet)
                continue

            test3 = aboveLine(y_obj, self.glue_area[1], self.glue_area[2])[
                1
            ]  # (y_obj>YLIM_MIN)
            test4 = not aboveLine(y_obj, self.glue_area[3], self.glue_area[0])[
                1
            ]  # (y_obj<YLIM_MAX)
            if test3 and test4:
                l_objets_a_recoller_type2.append(objet)
                continue

        # recollage : si disparition suspecte et apparition suspecte à côté ET à un pas de temps d'écart
        for objet_type1 in l_objets_a_recoller_type1:
            for objet_type2 in l_objets_a_recoller_type2:
                if abs(objet_type1.f_start - objet_type2.f_fin) < 100:
                    if objet_type1.calcul_distance(objet_type2) < GLUE_DIST:
                        objet_type1.recollage(objet_type2, self)
                        l_objets_a_recoller_type2.remove(objet_type2)

    def application_filtre(self):
        print(f"-> EXP {self.id} : filter trajectories and speed")
        for objet in self.objets:
            objet.lissage()

    def suppressions_objets_hors_cadre(self):
        """regarde si la trajectoire est dans le domaine d'étude :
        P2 | P3
        -------
        P1 | P4

        Nota : on ne teste que le début et la fin de la trajectoire"""

        print(f"-> EXP {self.id} : cut out of bounds data")

        # repérage des trajectoires à retirer > celles dont le début ET la fin sont en dehors
        objets_a_retirer = []
        for objet in self.objets:
            debut = objet.getPos(0)
            fin = objet.getPos(-1)

            test_fin_dehors = (
                (not aboveLine(self.domain[0], self.domain[1], fin)[0])
                or aboveLine(self.domain[1], self.domain[2], fin)[1]
                or aboveLine(self.domain[2], self.domain[3], fin)[0]
                or (not aboveLine(self.domain[3], self.domain[0], fin)[1])
            )

            if test_fin_dehors:  # alors on regarde si le début est aussi dehors
                if not aboveLine(self.domain[0], self.domain[1], debut)[0]:
                    objets_a_retirer.append(objet)
                    continue
                if aboveLine(self.domain[1], self.domain[2], debut)[1]:
                    objets_a_retirer.append(objet)
                    continue

                if aboveLine(self.domain[2], self.domain[3], debut)[0]:
                    objets_a_retirer.append(objet)
                    continue

                if not aboveLine(self.domain[3], self.domain[0], debut)[1]:
                    objets_a_retirer.append(objet)
                    continue

        # suppression des trajectoires
        for objet in objets_a_retirer:
            self.objets.remove(objet)
        print(len(objets_a_retirer), " objets hors cadre retirées")

    def representation_trajectoires_toutes(self):
        print("preparation sortie graphique")
        plt.title("Toutes les trajectoires")
        for ob_name, ob_mobile in self.observationsTerrain.items():
            plt.plot(
                ob_mobile["x"],
                ob_mobile["y"],
                c=RNG.rand(
                    3,
                ),
            )
        plt.show()

    def representation_trajectoires_objets(self):
        print("preparation sortie graphique")
        plt.title("Trajectoires conservées et post-traitées")
        plt.plot(
            [*self.domain[:, 0], self.domain[0][0]],
            [*self.domain[:, 1], self.domain[0][1]],
            c="k",
        )
        plt.plot(
            [*self.glue_area[:, 0], self.glue_area[0][0]],
            [*self.glue_area[:, 1], self.glue_area[0][1]],
            "--",
            c="k",
        )
        for objet in self.objets:
            color = RNG.rand(
                3,
            )
            plt.plot(objet.mobile["x"], objet.mobile["y"], c=color)
            plt.annotate(str(objet.id), objet.getPos(len(X) // 2, color=color, size=6))
        plt.show()

    def representation_animee_objets(self):
        """crée une animation représentant le mouvement des objets"""

        fig, ax = plt.subplots()
        xdata, ydata = [], []
        (ln,) = plt.plot([], [], ".", mew=0.1)

        def init():
            ax.set_xlim(self.xmin, self.xmax)
            ax.set_ylim(self.ymin, self.ymax)
            return (ln,)

        def update(frame):
            frame = int(frame)
            for objet in self.objets:
                if frame >= objet.f_start or objet.f_fin <= frame:
                    xdata.append(objet.mobile["x"][frame - objet.f_start])
                    ydata.append(objet.mobile["y"][frame - objet.f_start])
                else:
                    xdata.append(0.0)
                    ydata.append(0.0)
                ln.set_data(xdata, ydata)
            return (ln,)

        ani = FuncAnimation(
            fig,
            update,
            frames=np.linspace(0, self.f_max, self.f_max + 1),
            init_func=init,
            blit=True,
        )

        plt.show()

    def __str__(self):
        return (
            "Experience dont les premiers objets sont"
            + str(self.objets[0])
            + str(self.objets[1])
            + str(self.objets[2])
        )

    # plus d'utilité, noms affectés au chargement des données
    # def actualise_id(self):
    # """fonction technique """
    # iden=0
    # for objet in self.objets:
    # objet.id=iden
    # iden+=1


class Objet:
    """Piéton ou trottinette
    Rq : les trajectoires sont aussi traitées comme objet par simplicité
    """

    def __init__(
        self, experience, f_start, variables=defaultdict(lambda: np.array([]))
    ):
        self.id = -1
        self.type = 0  # 0=unknown; 1=piéton; 2=trottinette; 3=vélo
        self.f_start = f_start
        self.f_fin = f_start - 1
        self.mobile = variables

    def __str__(self):
        return (
            "objet de type"
            + str(self.type)
            + "\n ayant pour trajectoire :\n"
            + str(self.mobile)
            + str("\n\n dates apparition et disparition : ")
            + str(self.f_start)
            + " "
            + str(self.f_fin)
        )

    def getPos(self, f_local):
        return np.array((self.mobile["x"][f_local], self.mobile["y"][f_local]))

    def getPosAbsoluteFrame(self, f_global):
        f = f_global - self.f_start
        return np.array((self.mobile["x"][f], self.mobile["y"][f]))

    def determine_type(self, determine_type_func=None):
        """à partir de la vitesse max constatée, on détermine ici le type de l'objet : piéton ou trotti ou velo"""

        if self.type != 0 : # type already known
            return

        try:
            v_max = max(abs(self.mobile["s"]))
        except ValueError:
            self.mobile["s"] = np.tan(self.mobile["sy"], self.mobile["sx"])
            v_max = max(abs(self.mobile["s"]))
        
        if determine_type_func:
            self.type = determine_type_func(self)
        else:
            vitesse_pietons_max = 7 / 3.6
            if v_max > vitesse_pietons_max + 0.5:
                # TODO: find a way to differenciate bikes from e-scooters
                if RNG.triangular(0.0, 0.5, 1.0) > 0.5:
                    self.type = 2  #'trotti'
                else:
                    self.type = 3  #'velo'
            else:
                self.type = 1  #'pieton'

    def calcul_distance(self, other):
        """fonction outil : calcul distance de la tête de self à la queue de other
        ATTENTION A L ORDRE"""
        # pos_other=other.positions[other.f_fin,:]
        f_tail_other = other.f_start - min(self.f_fin, other.f_fin)
        diff = self.getPos(0) - other.getPos(f_tail_other)
        # print(pos_self,pos_other,diff)
        return np.sqrt(diff[0] ** 2 + diff[1] ** 2)

    def recollage(self, other, environnement):
        """fonction outil : ré-appareillage d'objets
        ATTENTION A L ORDRE : other se colle à l'avant de self"""

        # trou éventuel
        gap_size = max(other.f_fin + 1 - self.f_start, 0)
        gap_filler = np.zeros(gap_size)

        # on colle aux bons endroits toutes les vars
        for var in self.mobile:
            self.mobile[var] = np.concatenate(
                (other.mobile[var], gap_filler, self.mobile[var])
            )

            # bouchage du trou éventuel par interpolation
            for i in range(len(other.mobile), len(other.mobile) + gap_size):
                self.mobile[var][i] = np.interp(
                    i,
                    (len(other.mobile) - 1, len(other.mobile) + gap_size),
                    (other.mobile[var][-1], self.mobile[var][0]),
                )

        self.f_start = other.f_start
        # on retire des enregistrements de l'instance contenant
        environnement.objets.remove(other)
        print("recollage réalisé")

    def lissage(self):  # à coder (version provisoire)
        """filtre passe bas pour supprimer vibrations caméra"""
        from scipy.signal import savgol_filter

        self.mobile["x"] = savgol_filter(self.mobile["x"], 26, 5)
        self.mobile["y"] = savgol_filter(self.mobile["y"], 26, 5)
        if "s" in self.mobile:
            # pas besoin de le garder si on a les composants séparés
            del self.mobile["s"]
        self.mobile["sx"] = np.diff(self.mobile["x"])
        self.mobile["sy"] = np.diff(self.mobile["y"])
        if "a" in self.mobile:
            # ajouter que si déjà présent, prends de la place
            self.mobile["a"] = np.diff(np.tan(self.mobile["sy"], self.mobile["sx"]))


class Variable:
    def __init__(
        self,
        name,
        is_range,
        valid_values,
        location=0,
        subset=set(),
        default=None,
        mu=None,
        sigma=1.0,
        p=None,
    ):
        self.id = name
        if is_range:
            # it is a range, np.NINF or np.PINF can be input
            self.min, self.max = valid_values
            self.mu = mu
            self.sigma = sigma
            if not mu:
                self.mu, self.sigma = self.to_mu_sigma(default if default else 0.0, 1.0)
            self.values = None
        else:
            # enumerator of valid values
            self.values = tuple(valid_values)
            self.p = p if p else [1 / len(self.values) for i in range(len(self.values))]
        self.cur_val = default
        if not default:
            self.newCurVal()

        self.location = location  # vTypes=0, SUMOcfg=1
        self.type_set = set(subset)  # set of vTypes it should be used on

    def copy(self):
        is_range = self.values == None
        return Variable(
            self.id,
            is_range,
            (self.min, self.max) if is_range else self.values,
            self.location,
            self.type_set,
            self.cur_val,
            mu=self.mu if is_range else None,
            sigma=self.sigma if is_range else None,
            p=self.p if not is_range else None,
        )

    def newCurVal(self):
        if self.values:
            try:
                self.cur_val = RNG.choice(self.values, p=self.p)
            except TypeError as e:
                print(self.values, self.p)
                raise e
        else:
            self.cur_val = min(
                self.max, max(RNG.lognormal(self.mu, self.sigma), self.min)
            )

    def newVarWithNewVal(self):
        v = self.copy()
        v.newCurVal()
        return v

    @staticmethod
    def to_mu_sigma(moy, sd):
        """à partir des params voulus, donne les params à rentrer dans la fonction RNG.lognormal
        (= param de la loi normale associée"""

        mu = np.log(moy / np.sqrt(1 + (sd / moy) ** 2))
        sigma2 = np.log(1 + (sd / moy) ** 2)

        return (mu, np.sqrt(sigma2))

    @staticmethod
    def to_moy_sd(mu, sigma):
        """à partir des parametres mu et sigma de la loi lognormal , donne sa moyenne et son ecart-type"""

        moy = np.exp(mu + (sigma**2 / 2))
        ET = (np.exp(sigma**2) - 1) * np.exp(2 * mu + sigma**2)

        return (moy, np.sqrt(ET))

    def __str__(self):
        if self.values:
            return f"{{{self.id}={self.cur_val}; enum={self.values}; p={self.p}}}\n"
        else:
            return f"{{{self.id}={self.cur_val}; min={self.min}, max={self.max}; moy,sd={self.to_moy_sd(self.mu, self.sigma)}}}\n"


class Constant(Variable):
    def __init__(self, name, location=0, subset=set(), default=None):
        self.id = name
        self.values = (default,)
        self.p = (1,)
        self.cur_val = default
        self.location = location  # vTypes=0, SUMOcfg=1
        self.type_set = set(subset)  # set of vTypes it should be used on

    def copy(self):
        return Constant(self.id, self.location, self.type_set, self.cur_val)

    def newCurVal(self):
        pass

    def newVarWithNewVal(self):
        v = self.copy()
        return v

    def __str__(self):
        return f"{{{self.id}={self.cur_val}; CST}}\n"


class Parameters:
    def __init__(
        self,
        vTypes_vars=defaultdict(dict),
        sumocfg_vars=[],
        sim_step_traci_function=lambda conn: None,
        variables=[],
    ):
        self.vTypes = vTypes_vars
        self.sumocfg = sumocfg_vars
        self.simStep = sim_step_traci_function
        self.variables = variables

    def applyVars(self):
        self.sumocfg = [] #must be emptied else, risk of adding multiple of the same
        for v in self.variables:
            if v.location == 0:  # vTypes
                for vType in v.type_set:
                    self.vTypes[vType][str(v.id)] = str(v.cur_val)
            elif v.location == 1:  # SUMOcfg
                self.sumocfg.append(str(v.id))
                self.sumocfg.append(str(v.cur_val))
            else:
                raise RuntimeError(f"unkown variable location {v.location}")

    def searchAllVars(self, var_id):
        found = []
        for var in self.variables:
            if var.id == var_id:
                found.append(var)
        return found

    def searchVar(self, var_id):
        for var in self.variables:
            if var.id == var_id:
                return var
        return None
        
    def dumpAsCSVfile(self, filepath, best_params, **details):
        import csv
        typeEnum = {1:"Pedestrian", 2:"E-Scooter", 3:"Bike"}
        with open(filepath, 'w', newline='') as csvfile:
            wcsv = csv.writer(csvfile)
            wcsv.writerow(('var', 'type', 'best_value', 'average', 'sd', 'mu', 'sigma'))
            for var, best in zip(self.variables, best_params.variables):
                avg, sd = var.to_moy_sd(var.mu, var.sigma) if not var.values else ("", "")
                try:
                    types = var.type_set
                except AttributeError:
                    types = set()
                wcsv.writerow((
                    var.id, 
                    " ".join((typeEnum[typ] for typ in types)), 
                    best.cur_val, 
                    avg, 
                    sd,
                    var.mu if not var.values else "",
                    var.sigma if not var.values else "",
                ))
            if details:
                wcsv.writerow(("details :",))
                for key, value in details.items():
                    wcsv.writerow((key, value))

    def __str__(self):
        return "".join(map(str, self.variables)).join(["params: [[\n", "]]"])

    def newParametersSet(self, N):
        """génération de N jeux de données (caractéristiques uniformes
        pour l'ens des invidus : sd=0) selon les paramètres des variables du set original"""

        # realisation des tirages

        jeux_parametres = []
        for i in range(N):
            new_param = Parameters(
                sim_step_traci_function=self.simStep,
                variables=[v.newVarWithNewVal() for v in self.variables],
            )

            new_param.applyVars()

            jeux_parametres.append(new_param)

        return jeux_parametres

    @classmethod
    def parametersFromMetaSims(cls, meta_simulations : list):
        new_param = cls(
            sim_step_traci_function=meta_simulations[0].parametres.simStep,
            variables=[v.copy() for v in meta_simulations[0].parametres.variables],
        )

        var_list = []
        for i in range(len(new_param.variables)):
            var_list.append([])

        for meta_sim in meta_simulations:
            params = meta_sim.parametres
            for i in range(len(new_param.variables)):
                var_list[i].append(params.variables[i].cur_val)

        for var, vals in zip(new_param.variables, var_list):
            if var.values == None:  # continuous var
                #try:
                if len(vals) > 1:
                    var.mu, var.sigma = Variable.to_mu_sigma(
                        np.mean(vals), np.std(vals, ddof=1)
                    )
                else:
                    var.mu, var.sigma = Variable.to_mu_sigma(vals[0], 1.0)
                # except BaseException as e:
                    # print("vals:", vals, file=sys.stderr)
                    # raise e

            else:  # discrete var
                freqs = np.unique(vals, return_counts=True)
                var.values = freqs[0]
                var.p = freqs[1]
        return new_param


class Simulation:
    """objet stockant toutes les infos sur une simulation : input & output"""

    def __init__(self, experience, parametres, directory, network_file=None, route_file=None):
        """CI : numero de la feuille de conditions initiales (fichier excel)
        attention à l'ordre de spécification des parametres"""

        # identité
        self.id = RNG.integers(0, 2147483647)
        self.directory = os.path.join(directory, f"{experience.id}sim{self.id}")
        self.experience_associee = experience
        self.network_file = (
            network_file
            if network_file
            else (os.path.join(self.experience_associee.directory, f"network.net.xml"))
        )
        if route_file:
            self.route = route_file
        else:
            self.route = os.path.join(self.directory, f"routes.rou.xml")
            build.createRou(self.route, experience.objets, parametres.vTypes)
        
        # input
        self.params = parametres

        # output
        self.observations = build.VehicleContainer()
        self.err = -1

    def seFaire(self):
        net = str(self.network_file)
        rou = str(self.route)
        fcd = os.path.join(self.directory, "result.fcd.xml")
        log = os.path.join(self.directory, "logfile.txt")
        cfg = os.path.join(self.directory, "sim.sumocfg")

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        assert os.path.exists(net), "no network file"

        

        # prepare SUMO exec
        sumoBinary = checkBinary("sumo")
        sim_name = f"sim{self.id}"
        sumoCmd = [
                    sumoBinary,
                    '--log', 
                    log,
                    "--fcd-output",
                    fcd,
                    "-r",
                    rou,
                    "-n",
                    net,
                    #"--save-configuration", #sumo is not lauched afterwards!
                    #cfg,
                ] + self.params.sumocfg

        # start sumo simulation
        print(f"-> {self.id} SIMULATION START\n", end='')  
        try:
            with MAX_SIM_SEMAPHORE:
                traci.start(
                    sumoCmd,
                    port=getPort(),
                    label=sim_name,
                )
                conn = traci.getConnection(sim_name)
                # step through the simulation
                while conn.simulation.getMinExpectedNumber() > 0:
                    conn.simulationStep()
                    # call the SFM sim or something else
                    self.params.simStep(conn)
                conn.close()
            
        except traci.exceptions.FatalTraCIError as e:
            try:
                with open(log, mode='r') as f:
                    print(f"SUMO {self.id} LOG:", file=sys.stderr)
                    for line in f:
                        print(line, end='', file=sys.stderr)
                raise e
            except FileNotFoundError:
                raise ValueError("Invalid SUMO args")

        print(f"   {self.id} retrieve data\n", end='')
        # load data from latest simulation
        self.observations = build.loadFcd(fcd)
        
        return self.calcRMSE()
        

    def calcRMSE(self):
        """compare et calcule le RMSE"""
        print(f"   {self.id} calc RMSE\n", end="")
        calibrated_vtypes = self.experience_associee.calibrate_vtypes
        # initialisation
        err = 0
        N = 0  # nombre de sclaires comparés #pour normalisation

        # on repère quels sont les objets équivalents côté simul/exp
        f = np.vectorize(lambda x: x**2)
        for obj in self.experience_associee.objets:
            if obj.type in calibrated_vtypes:
                objet_exp = obj.mobile
                traj = self.observations[obj.id]
                if not traj:
                    print(f"      {obj.id} missing in sim! jump...\n", end="")
                    continue
        # for name, traj in self.observations.items():
            # objet_exp = self.experience_associee.observationsTerrain.get(name, None)
            # if objet_exp:
                max_len = min(len(traj["x"]), len(objet_exp["x"]))
                coords = np.column_stack(
                    (objet_exp["x"][:max_len] - traj["x"][:max_len], 
                    objet_exp["y"][:max_len] - traj["y"][:max_len])
                )
                
                err += np.sqrt(
                    np.sum(
                        f(coords),
                        #1,
                    )
                )
                N += max_len #len(coords)

        self.err = err / N
        print(f"   {self.id} done.\n", end='')
        return self.err

    def seRepresenter(self):
        """affiche en pointillé simulation et en trait plein l'experience"""

        exp = self.experience_associee
        print("preparation sortie graphique")

        from . import output_trace as oTrace

        try:
            sim_step = self.params.searchVar("--step-length").cur_val
        except:
            sim_step = 1.0

        print("-> PLOT DATA")
        for vehID in self.observations:
            print(f"   drawing veh {vehID}")
            oTrace.plot_vehicle_speed(
                vehID,
                self.observations,
                exp.observationsTerrain,
                os.path.join(self.directory, f"{self.id}speeds"),
                sim_step,
            )
            oTrace.plot_vehicle_trajectory(
                vehID,
                self.observations,
                exp.observationsTerrain,
                os.path.join(self.directory, f"{self.id}trajectory"),
                exp.lane_size,
            )

        print("-> DYN PLOT")

        plt.title("Comparaison simulation et expérience")
        plt.plot(
            exp.domain[:, 0],
            exp.domain[:, 1],
            c="k",
        )
        plt.plot(
            exp.glue_area[:, 0],
            exp.glue_area[:, 1],
            "--",
            c="k",
        )
        l_color = []
        for name, objet in self.observations.items():
            color = RNG.rand(
                3,
            )
            l_color.append(color)
            plt.plot(objet["x"], objet["y"], "--", c=color)
            plt.annotate(
                str(name),
                (objet["x"][len(objet["t"]) // 2], objet["y"][len(objet["t"]) // 2]),
                color=color,
                size=6,
            )
        l_obj_in = [obj.id for obj in self.objets]

        i = 0
        for objet in exp.objets:
            if objet.id in l_obj_in:
                col = l_color[i]
                i += 1
                plt.plot(objet.mobile["x"], objet.mobile["y"], c=col)
                plt.annotate(
                    str(objet.id),
                    objet.getPos(len(objet.mobile["t"]) // 2),
                    color=col,
                    size=6,
                )
        plt.show()


class Meta_Simulation:
    def __init__(self, experiences, parametres, directory, i=0, N_per_exp=1):
        """pour chaque fenêtre [temps_debut, temps_fin] : cet objet contiendra une simulation
        faite avec le même jeu de paramètres parametres (mais pas les mêmes CI)"""

        # identité
        self.id = i
        self.experiences = experiences
        self.parametres = parametres
        self.directory = directory
        self.err = -1
        self.N_per_exp = N_per_exp
        
    def seFaire(self):
        print(f"->  start méta simulation {self.id}\n", end="")
        folder = os.path.join(self.directory, f"{self.id}metasim")
        route = os.path.join(folder, f"routes.rou.xml")
        if not os.path.exists(folder):
            os.makedirs(folder)

        # simulations
        self.simulations = []
        threadsToMonitor = []
        for exp in self.experiences:
            #create route file
            route = os.path.join(folder, f"exp{exp.id}routes.rou.xml")
            build.createRou(route, exp.objets, self.parametres.vTypes)
            for i in range(self.N_per_exp):
                simul = Simulation(exp, self.parametres, folder, route_file=route)
                self.simulations.append(simul)
                
                t = Thread(
                    target=lambda:simul.seFaire(), 
                    name=f"tsim{simul.id}",
                )
                threadsToMonitor.append(t)
                t.start()
                
        for t in threadsToMonitor:
            t.join()
        
        print(f"    end méta simulation \n", end="")
        # evaluation
        return self.calcErr()

    def calcErr(self):
        """compare chaque simulation à la realité et calcule une erreure moyenne"""

        err = 0
        N = 0
        for simul in self.simulations:
            err += simul.err
            N += 1
        self.err = err / N
        print(f"->  end méta simulation {self.id}, err={self.err}", end="")
        return self.err


######################
# Outils
######################


def aboveLine(P1, P2, P):
    """test la position par rapport à la droite P1 - P2"""
    v_dir = P2 - P1
    v_dir = v_dir / np.linalg.norm(v_dir)
    projection = P1 + np.inner(P - P1, v_dir) * v_dir
    return (P[0] - projection[0] >= 0, P[1] - projection[1] >= 0)
