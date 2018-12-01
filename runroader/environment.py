import pickle
from collections import namedtuple
from pathlib import Path

import click
import matplotlib.patches as patches
import matplotlib.pyplot as plt

from .constants import S_0, S_F, T_MAX

DEFAULT_FILE_NAME = 'random'
TEMP_DIRECTORY = Path('tmp_env')
PERMINANT_DIRECTORY = Path('saved_envs')

Rectangle = namedtuple('Rectangle', ['bottom_left_corner', 'width', 'height'])


def get_pickle_path(name):
    if name is None:
        name = DEFAULT_FILE_NAME
        directory = TEMP_DIRECTORY
    else:
        directory = PERMINANT_DIRECTORY
    if not directory.exists():
        directory.mkdir()
    return directory / (name + '.pickle')


class Environment:
    @classmethod
    def from_pickle(cls, name=None):
        environment_path = get_pickle_path(name)
        with open(environment_path, 'rb') as f:
            return pickle.load(f)

    def __init__(self, rectangles):
        self.rectangles = rectangles

    def render(self, ax):
        ax.set_xlim([S_0, S_F])
        ax.set_ylim([0, T_MAX])
        for rec in self.rectangles:
            rectangle_patch = patches.Rectangle(rec.bottom_left_corner, rec.width, rec.height)
            ax.add_patch(rectangle_patch)

    def to_pickle(self, name=None):
        environment_path = get_pickle_path(name)
        with open(environment_path, 'wb') as f:
            pickle.dump(self, f)
        print('Saved the model to {}'.format(environment_path))


@click.command()
@click.option('--name', help='The name of the environment to visualize')
def visualize_pickle(name=None):
    environment = Environment.from_pickle(name)
    plt.figure()
    ax = plt.subplot(111)
    environment.render(ax)
    plt.show()
