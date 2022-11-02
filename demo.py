from calibsumo import *

default_start = "C:\\Garyfallia\\" # Start path where folders containing data on vehicle traces are located
default_output_dir = r"C:\PatrasCalibsumo" # Folder where to store folders of results of the calibration process

path = default_output_dir+r"\accelDecelEmgSpeedGap1" # Folder containing results of this calibration process

exp_set = set() # this set() contains one or multiple experiences, i.e. network configurations with their associated routes
exp_set.add(createDefaultExperience("Cycle-Lane-Pedestrians-Crossing", path)) # createDefaultExperience() generates automatically what is needed for default folders listed above.
exp_set.add(createDefaultExperience("Road-Pedestrians-Crossing-1", path))

rivoliVars = [ # this list contains constants and variables  to set and ajust respectively. the order is even kept in result reports 
#--Constants--
    sim.Constant(
        "--step-length", # simulation step length
        location=LOC_SUMO, # data to be put in SUMO launch parameters (inline or sumocfg file)
        default=0.05, # value
    ),
    sim.Constant(
        "--lateral-resolution", # lateral resolution of the sublane lane change model
        location=LOC_SUMO, 
        default=0.257, # obtained by prior calibration on less variables
    ),
    
#--Variables--
    # E-Scooter exclusive variables
    sim.Variable(
        "maxSpeed",
        VAR_CONTINUOUS, # this variable is continuous in opposition to discrete variable
        (1.0, 20.0), # (min, max) values
        location=LOC_VTYP, # data to be put in .rou.xml file inside the vehicle type spec declared in the "subset" argument
        default=10.57, # default value to start calibration from
        subset={TYP_ES}, # one or more vehicle types using this variable. If multiple are present, the variable will be shared between vehicle types. If each vehicle type must have a different value, duplicate the sim.Variable entry in the variable list (as done in this file).
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
    
    # Bike exclusive variables
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
    
    # Pedestrian exclusive variables
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
calibrated = calibrate(exp_set, rivoliVars, path) # launches the calibration process. If launched in an IDE, "calibrated" collects data with more precise things than present in the final report. 
