import pickle

import click
import matplotlib.patches as patches
import matplotlib.pyplot as plt

from runroader.constants import S_0, S_F, T_MAX
from runroader.utils.environment import Rectangle, DEFAULT_FILE_NAME, TEMP_DIRECTORY, PERMINANT_DIRECTORY

class EnvironmentBuilder:
    def __init__(self, directory, name):
        self.directory = directory
        self.name = name if name is not None else DEFAULT_FILE_NAME
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title('drag to build rectangles')
        self.ax.set_xlim([S_0, S_F])
        self.ax.set_ylim([0, T_MAX])
        self.rectangles = []
        self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def on_press(self, event):
        self.starting_pos = (event.xdata, event.ydata)

    def on_release(self, event):
        self.ending_pos = (event.xdata, event.ydata)
        bottom_left_corner = (min(self.starting_pos[0], self.ending_pos[0]), min(self.starting_pos[1], self.ending_pos[1]))
        width = abs(self.starting_pos[0] - self.ending_pos[0])
        height = abs(self.starting_pos[1] - self.ending_pos[1])
        rectangle_patch = patches.Rectangle(bottom_left_corner, width, height)
        self.rectangles.append(Rectangle(bottom_left_corner, width, height))
        self.ax.add_patch(rectangle_patch)
        self.fig.canvas.draw()

    def on_key(self, event):
        if event.key == 'enter':
            save_path = self.directory / (self.name + '.pickle')
            with open(save_path, 'wb') as f:
                pickle.dump(self.rectangles, f)
            print('Saved the model to {}'.format(save_path))


@click.command()
@click.option('--name', help='The name of this newly-built environment. Providing this option means you want to save the model permanently')
def main(name):
    """Simple program for building the time-distance environment.

    Drag your mouse to build the environment.

    Press `enter` for the end of the process
    """
    if name is not None:
        directory = PERMINANT_DIRECTORY
    else:
        directory = TEMP_DIRECTORY
    if not directory.exists():
        directory.mkdir()

    builder = EnvironmentBuilder(directory, name)
    plt.show()


if __name__ == "__main__":
    main()
