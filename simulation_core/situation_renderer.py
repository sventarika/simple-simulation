from __future__ import annotations

import io
import numpy as np

from matplotlib.transforms import Bbox
from typing import TYPE_CHECKING

from commonroad.visualization.mp_renderer import MPRenderer

if TYPE_CHECKING:
    import matplotlib as mpl
    from commonroad.visualization.param_server import ParamServer
    from commonroad.scenario.obstacle import Obstacle


class SituationRenderer(MPRenderer):

    def __init__(self,
                 draw_params: ParamServer | dict | None = None,
                 plot_limits: list[int | float] | None = None,
                 ax: mpl.axes.Axes | None = None,
                 figsize: tuple[float, float] | None = None,
                 focus_obstacle: Obstacle| None = None
                 ) -> None:
        super().__init__(draw_params, plot_limits, ax, figsize, focus_obstacle)

    def render(self,
               show: bool = False,
               filename: str | None = None,
               keep_static_artists: bool = False,
               mode: str = "human",
               figsize: tuple[float, float] | None = None,
               plot_limits: list[float] | None = None
               ) -> np.ndarray | list[np.ndarray] | None:
        """
        Render all objects from buffer

        :param show: Show the resulting figure
        :param filename: If provided, saves the figure to the provided file
        :param plot_limits: [xmin, xmax, ymin, ymax]
        :return: List of drawn object's artists
        """

        render_frame = None

        self.ax.cla()
        artists_list = self.render_static()
        artists_list.extend(self.render_dynamic())

        if self.plot_limits is None:
            self.ax.autoscale(True)
        else:
            self.ax.set_xlim(self.plot_limits_focused[:2])
            self.ax.set_ylim(self.plot_limits_focused[2:])
        self.ax.set_aspect("equal")

        if mode == "rgb_array":

            if figsize is not None:
                self.f.set_size_inches(figsize)

            if plot_limits is not None:
                self.ax.set(xlim=(plot_limits[0], plot_limits[1]), ylim=(plot_limits[2], plot_limits[3]))

            self.ax.spines["top"].set_visible(False)
            self.ax.spines["right"].set_visible(False)
            self.ax.spines["bottom"].set_visible(False)
            self.ax.spines["left"].set_visible(False)
            self.ax.get_xaxis().set_ticks([])
            self.ax.get_yaxis().set_ticks([])

            dpi = 300
            self.f.set_dpi(dpi)

            f_size_inches = self.f.get_size_inches()
            fw = f_size_inches[0]
            fh = f_size_inches[1]
            rect = (0, 0, fw, fh)

            # Create rect out of tightbox (inches)
            tightbox = self.f.get_tightbbox()
            # rect : tuple (left, bottom, right, top), default: (0, 0, 1, 1)
            rect = np.array([
                (tightbox.x0 / fw) * fw,
                (tightbox.y0 / fh) * fh,
                (tightbox.x1 / fw) * fw,
                (tightbox.y1 / fh) * fh
            ])
            rect = np.round(rect, 1)
            rect = tuple(rect)

            with io.BytesIO() as io_buf:
                self.f.savefig(io_buf, format="raw", dpi=dpi, bbox_inches=Bbox.from_extents(*rect))
                io_buf.seek(0)
                img_data = np.frombuffer(io_buf.getvalue(), dtype=np.uint8)

            height_px = int((rect[3] - rect[1]) * dpi)
            width_px = int((rect[2] - rect[0]) * dpi)
            render_frame = np.reshape(img_data, newshape=(height_px, width_px, -1))

        if filename is not None:
            self.f.savefig(filename, bbox_inches="tight", format="png", dpi=300)
        if show:
            self.f.show()

        if self.draw_params.by_callstack(param_path="axis_visible", call_stack=()) is False:
            self.ax.axes.xaxis.set_visible(False)
            self.ax.axes.yaxis.set_visible(False)

        self.clear(keep_static_artists)
        return render_frame
