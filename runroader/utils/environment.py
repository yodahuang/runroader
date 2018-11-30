import pickle
from collections import namedtuple
from pathlib import Path

DEFAULT_FILE_NAME = 'random'
TEMP_DIRECTORY = Path('tmp_env')
PERMINANT_DIRECTORY = Path('saved_envs')

Rectangle = namedtuple('Rectangle', ['bottom_left_corner', 'width', 'height'])


def load_environment(name=None):
    if name is None:
        name = DEFAULT_FILE_NAME
        directory = TEMP_DIRECTORY
    else:
        directory = PERMINANT_DIRECTORY
    environment_path = directory / (name + '.pickle')
    with open(environment_path, 'rb') as f:
        return pickle.load(f)
