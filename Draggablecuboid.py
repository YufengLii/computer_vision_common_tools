import matplotlib.pyplot as plt
import matplotlib.patches as patches
from DraggablePoint import DraggablePoint
from matplotlib.path import Path
import matplotlib.image as mpimg


class CubuoidMasker:
    def __init__(self,ax):

        v1 = (50., 150.)
        v2 = (150., 150.)
        v3 = (150., 50.)
        v4 = (50., 50.)
        v5 = (200, 100)
        v6 = (200, 200)
        v7 = (100, 200)
        v8 = (100, 100)


        cuboid_virtices = [patches.Circle(v1, 4, fc='r', alpha=0.5),
                        patches.Circle(v2, 4, fc='r', alpha=0.5),
                        patches.Circle(v3, 4, fc='r', alpha=0.5),
                        patches.Circle(v4, 4, fc='r', alpha=0.5),
                        patches.Circle(v5, 4, fc='r', alpha=0.5),
                        patches.Circle(v6, 4, fc='r', alpha=0.5),
                        patches.Circle(v7, 4, fc='r', alpha=0.5),
                        patches.Circle(v8, 4, fc='r', alpha=0.5)]

        drs = []

        for circ in cuboid_virtices:
            ax.add_patch(circ)
            dr = DraggablePoint(circ)
            dr.connect()
            drs.append(dr)

        self.DraggablePoint_list = drs
        self.cur_cuboid_virtices = []
        for point_i in self.DraggablePoint_list:
            self.cur_cuboid_virtices.append(point_i.get_xy())
        # print(self.cur_cuboid_virtices)
        self.verts = [
            self.cur_cuboid_virtices[3],
            self.cur_cuboid_virtices[2],
            self.cur_cuboid_virtices[1],
            self.cur_cuboid_virtices[0],
            self.cur_cuboid_virtices[3],
            self.cur_cuboid_virtices[0],
            self.cur_cuboid_virtices[6],
            self.cur_cuboid_virtices[5],
            self.cur_cuboid_virtices[4],
            self.cur_cuboid_virtices[2],
            self.cur_cuboid_virtices[3],
            self.cur_cuboid_virtices[7],
            self.cur_cuboid_virtices[6],
            self.cur_cuboid_virtices[1],
            self.cur_cuboid_virtices[5],
            self.cur_cuboid_virtices[7],
            self.cur_cuboid_virtices[4],
        ]

        self.codes = [
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
        ]

        self.path = Path(self.verts, self.codes)
        self.patch = patches.PathPatch(self.path , edgecolor= 'blue',facecolor='None', lw=2)
        self.ax = ax
        self.ax.add_patch(self.patch)
        self.cid = self.patch.figure.canvas.mpl_connect('button_release_event', self.button_release_callback)

    def button_release_callback(self, event):
        print('click', event)
        # if event.inaxes!=self.line.axes: return
        self.patch.remove()
        self.cur_cuboid_virtices = []
        for point_i in self.DraggablePoint_list:
            self.cur_cuboid_virtices.append(point_i.get_xy())
        # print(self.cur_cuboid_virtices)
        self.verts = [
            self.cur_cuboid_virtices[3],
            self.cur_cuboid_virtices[2],
            self.cur_cuboid_virtices[1],
            self.cur_cuboid_virtices[0],
            self.cur_cuboid_virtices[3],
            self.cur_cuboid_virtices[0],
            self.cur_cuboid_virtices[6],
            self.cur_cuboid_virtices[5],
            self.cur_cuboid_virtices[4],
            self.cur_cuboid_virtices[2],
            self.cur_cuboid_virtices[3],
            self.cur_cuboid_virtices[7],
            self.cur_cuboid_virtices[6],
            self.cur_cuboid_virtices[1],
            self.cur_cuboid_virtices[5],
            self.cur_cuboid_virtices[7],
            self.cur_cuboid_virtices[4],
        ]

        self.codes = [
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
            Path.MOVETO,
            Path.LINETO,
        ]
        self.path = Path(self.verts, self.codes)
        self.patch = patches.PathPatch(self.path, edgecolor= 'blue' ,facecolor='None',lw=2)
        self.ax.add_patch(self.patch)
        self.patch.figure.canvas.draw()        


def HandFitCuboidTest():

    # test_draggable_points()
    img = mpimg.imread('219.jpg')
    fig, ax = plt.subplots()
    ax.imshow(img)
    MyCubuoidMasker = CubuoidMasker(ax)
    plt.show()

if __name__ == "__main__":

    HandFitCuboidTest()