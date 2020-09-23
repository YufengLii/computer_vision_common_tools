import matplotlib.pyplot as plt
import matplotlib.patches as patches


class DraggablePoint:
    """
    This class is used to managed points which can be dragged around a grid for debugging purposes.
    This class is mot current used in typical analysis or visualized.
    """
    lock = None  # only one can be animated at a time

    def __init__(self, point, move_callback=None):
        """
        The DraggablePoint object creates a point which has a particular movement callback when its location change
        is released.
        :param point: the point location to be dragged
        :param move_callback: the callback to fire whenever a movement is completed (i.e. when on_release fires)
        """
        self.move_callback = move_callback
        self.point = point
        self.cidmotion = self.point.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.cidrelease = self.point.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.cidpress = self.point.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.press = None
        self.background = None

    def connect(self, move_callback=None):
        """
        This function is used to connect a movement callback function which can be used for redrawing based on updated
        draggable point locations.
        :param move_callback: the callback to fire whenever a movement is completed (i.e. when on_release fires)
        """
        self.move_callback = move_callback
        # connect to all the events we need
        pass

    def get_xy(self):
        # print(self.point.center)
        return self.point.center

    def on_press(self, event):
        """
        When the draggable object is pressed, this function fires.
        :param event: the event object associated with the object press
        :return: None
        """
        if event.inaxes != self.point.axes:
            return
        if DraggablePoint.lock is not None:
            return
        contains, attrd = self.point.contains(event)
        if not contains:
            return
        self.press = self.point.center, event.xdata, event.ydata
        DraggablePoint.lock = self

        # draw everything but the selected rectangle and store the pixel buffer
        canvas = self.point.figure.canvas
        axes = self.point.axes
        self.point.set_animated(True)
        canvas.draw()
        self.background = canvas.copy_from_bbox(self.point.axes.bbox)

        # now redraw just the rectangle
        axes.draw_artist(self.point)

        # and blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_motion(self, event):
        """
        When the draggable object is moved around, this function fires.
        :param event: the event object associated with the object movement
        :return: None
        """
        if DraggablePoint.lock is not self:
            return
        if event.inaxes != self.point.axes:
            return
        self.point.center, xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        self.point.center = (self.point.center[0]+dx, self.point.center[1]+dy)

        canvas = self.point.figure.canvas
        axes = self.point.axes

        # restore the background region
        canvas.restore_region(self.background)

        # redraw just the current rectangle
        axes.draw_artist(self.point)

        # blit just the redrawn area
        canvas.blit(axes.bbox)

    # noinspection PyUnusedLocal
    def on_release(self, e):
        """
        When the draggable point is release, this function fires.
        :param e: the event object associated with the object release
        :return: None
        """
        if self.move_callback:
            self.move_callback()

        # on release we reset the press data
        if DraggablePoint.lock is not self:
            return

        self.press = None
        DraggablePoint.lock = None

        # turn off the rect animation property and reset the background
        self.point.set_animated(False)
        self.background = None

        # redraw the full figure
        self.point.figure.canvas.draw()
        

    def disconnect(self):
        # disconnect all the stored connection ids
        """
        """
        self.point.figure.canvas.mpl_disconnect(self.cidpress)
        self.point.figure.canvas.mpl_disconnect(self.cidrelease)
        self.point.figure.canvas.mpl_disconnect(self.cidmotion)


def test_draggable_points():
    """
    This function allows quick, easy testing of the DraggablePoints class for the purpose of manually dragging
    points around for debugging purposes for the algorithms in this package.
    This module is mot current used in typical analysis or visualized.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111)
    drs = []
    circles = [patches.Circle((0.32, 0.3), 0.03, fc='r', alpha=0.5),
               patches.Circle((0.3, 0.3), 0.03, fc='g', alpha=0.5)]

    for circ in circles:
        ax.add_patch(circ)
        dr = DraggablePoint(circ)
        dr.connect()
        drs.append(dr)

    plt.show()
