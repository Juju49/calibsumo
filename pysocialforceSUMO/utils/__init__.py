"""Config module"""
from .config import Config, DefaultConfig
from .logging import logger, timeit
from .plot import *
from . import stateutils

TYP_UNK, TYP_PED, TYP_ES, TYP_BK = 0, 1, 2, 3
typ_set = {TYP_UNK, TYP_PED, TYP_ES, TYP_BK}