"""Config"""
from pathlib import Path
from typing import Dict
import toml
import numpy as np

class Config:
    """Config loading and updating
    Attribute
    -------------
    config: dict

    Methods
    -------------
    from_dict: update from a dict
    load_config: update from file
    sub_config: return a sub dict wrapped in Config()
    """

    def __init__(self, config=None) -> None:
        self.config = {}
        if config:
            self.config = config

    def from_dict(self, config: Dict) -> None:
        """Update from dict"""
        self.config.update(config)

    def load_config(self, filename: str) -> None:
        """update from file"""
        user_config = toml.load(filename)
        self.from_dict(user_config)

    def sub_config(self, field_name: str) -> "Config":
        """return a sub dict wrapped in Config()"""
        sub_dict = self.config.get(field_name)
        if isinstance(sub_dict, dict):
            return Config(sub_dict)
        return Config()

    def __call__(self, entry: str, default=None) -> np.array or bool or int:
        ret = self.config.get(entry)
        if not ret:
            return np.array((0., default, default, default))
        elif len(ret) == 1:
            return ret[0]
        elif len(ret) < 4:
            return np.array((0.0, *ret))
        else:
            return np.array(ret)


class DefaultConfig(Config):
    """Default configs"""

    CONFIG = """
    title = "Social Force Default Config File"

    [scene]
    enable_group = [false]
    agent_radius = [3.5, 0.35, 1.5, 2.0]
    step_width = [1.0, 10.0, 10.0]
    max_speed_multiplier = [1.3, 1.5, 1.5]
    tau = [0.5, 0.5, 0.5]
    resolution = [10]

    [goal_attractive_force]
    factor = [1.0, 1.0, 1.0]

    [ped_repulsive_force]
    factor = [1.5, 1.5, 1.5]
    v0 = [2.1, 2.1, 2.1]
    sigma = [0.3, 0.3, 0.3]
    # fov params
    fov_phi = [100.0, 100.0, 100.0]
    fov_factor = [0.5, 0.5, 0.5] # out of view factor

    [space_repulsive_force]
    factor = [1.0, 1.0, 1.0]
    u0 = [10.0, 10.0, 10.0]
    r = [0.2, 0.2, 0.2]

    [group_coherence_force]
    factor = [3.0, 3.0, 3.0]

    [group_repulsive_force]
    factor = [1.0, 1.0, 1.0]
    threshold = [0.55, 1.0, 1.0]

    [group_gaze_force]
    factor = [4.0, 4.0, 4.0]
    # fov params
    fov_phi = [90, 80, 80]

    [desired_force]
    factor = [1.0, 1.0, 1.0]
    relaxation_time = [0.5, 0.5, 0.5]
    goal_threshold = [0.2, 1.0, 1.0]
    
    [social_force]
    factor = [5.1, 5.1, 5.1]
    lambda_importance = [2.0, 2.0, 2.0]
    gamma = [0.35, 0.35, 0.35]
    n = [2, 2, 2]
    n_prime = [3, 3, 3]

    [obstacle_force]
    factor = [10.0, 15.0, 15.0]
    sigma = [0.2, 0.2, 0.2]
    threshold = [3.0, 3.0, 3.0]

    [along_wall_force]
    """

    def __init__(self):
        # config_dir = Path(__file__).resolve().parent.parent.joinpath("/config")
        # super().__init__(toml.load(config_dir.joinpath(default_config)))
        super().__init__(toml.loads(self.CONFIG))
