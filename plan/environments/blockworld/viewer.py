"""
This file is to create viewer for the
environment. The codes are similar to
that of in example of continous_tamp
within PDDLStream. The modification here
is we created top down view of the environment
so, there are few changes. Furthermore,
we also added obstacles in the environment.

PDDLStream Continous TAMP Viewer: https://github.com/caelan/pddlstream/blob/main/examples/continuous_tamp/viewer.py

"""

from __future__ import print_function

from numpy import fix

try:
    from Tkinter import Tk, Canvas, Toplevel
except ImportError:
    from tkinter import Tk, Canvas, Toplevel

from examples.continuous_tamp.primitives import forward_kin

SUCTION_WIDTH = 30
SUCTION_HEAD = 15.0
STEM_WIDTH = 1.0
STEM_HEIGHT = 2.5
BLOCK_WIDTH = 40
PIXEL_BUFFER = 10
ENV_HEIGHT = 1.0

BACKGROUND_COLOR = "light blue"


def get_width(interval):
    return interval[1] - interval[0]


##################################################

class ContinuousTMPViewer(object):
    def __init__(
        self,
        regions,
        obstacles,
        tl_x=0,
        tl_y=0,
        width=400,
        height=400,
        title="Grid",
        background=BACKGROUND_COLOR,
    ):
        self.tk = Tk()
        self.tk.withdraw()
        self.top = Toplevel(self.tk)
        self.top.wm_title(title)
        self.top.protocol("WM_DELETE_WINDOW", self.top.destroy)

        self.regions = regions
        self.obstacles = obstacles
        self.width = width
        self.height = height
        self.canvas = Canvas(
            self.top, width=self.width, height=self.height, background=background
        )
        self.canvas.pack()
        self.move_frame(tl_x, tl_y)

        max_width = 100
        self.dist_to_pixel = (self.width - 2 * PIXEL_BUFFER) / max_width
        self.dist_width = self.width / self.dist_to_pixel
        self.dist_height = self.height / self.dist_to_pixel
        self.ground_height = self.height - self.dist_to_pixel * ENV_HEIGHT

        self.dynamic = []
        self.static = []
        self.draw_environment()

    def center(self):
        self.top.update_idletasks()
        w = self.top.winfo_screenwidth()
        h = self.top.winfo_screenheight()
        size = tuple(int(_) for _ in self.top.geometry().split("+")[0].split("x"))
        x = w / 2 - size[0] / 2
        y = h / 2 - size[1] / 2
        self.top.geometry("%dx%d+%d+%d" % (size + (x, y)))

    def move_frame(self, x, y):
        self.top.update_idletasks()
        size = tuple(int(_) for _ in self.top.geometry().split("+")[0].split("x"))
        self.top.geometry("%dx%d+%d+%d" % (size + (x, y)))

    def scale_x(self, x):
        return self.width / 2.0 + self.dist_to_pixel * x

    def scale_y(self, y):
        return self.ground_height - self.dist_to_pixel * y

    def draw_roadmap(self, roadmap, color="black"):
        for line in roadmap:
            (point1, point2) = line
            x1, y1 = point1[0], point1[1]
            x2, y2 = point2[0], point2[1]
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=2)

    def draw_point(self, point, radius=2, color="white"):
        x, y = point[0], point[1]
        self.canvas.create_oval(
            x - radius, y - radius, x + radius, y + radius, fill=color
        )

    def draw_block(self, x, y, name="", color="blue"):
        if name == "ghost":
            self.dynamic.extend([])
        else:
            x1, x2, y1, y2 = (
                x + BLOCK_WIDTH / 2,
                x - BLOCK_WIDTH / 2,
                y + BLOCK_WIDTH / 2,
                y - BLOCK_WIDTH / 2,
            )
            self.dynamic.extend(
                [
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2, fill=color, outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, (y1 + y2)/2, font=("Purisa", 6), text=name),
                ]
            )

    def draw_region(self, region, color="azure3", name=""):
        x1, x2 = region[0][0], region[1][0]
        y1, y2 = region[0][1], region[1][1]
        rooms = ["bedroom", "kitchen", "livingroom"]
        boxes = ["box1", "box2", "box3", "box4"]
        if name in rooms:
            self.static.extend(
                [ self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill="antiquewhite3", outline="black", width=2
                ),]
            )
        elif name == "goal":
             [
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2+ 20, fill="cadetblue", outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name),
                ]

        elif name == "fakeenv1" or name == "fakeenv2":
            self.static.extend([])
            
        elif name in boxes:
            self.static.extend(
                [ self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill="azure1", outline="black", width=2
                ),]
            )
        elif name == "restroom":
            self.static.extend(
                [ self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill="azure3", outline="black", width=2
                ),]
            )

        elif name == "door":
            self.static.extend(
            [self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill="brown", outline="black", width=2
                ),
            self.canvas.create_oval(x1+20, y1+20, x1+30, y1+30, width=2, outline="black", fill="brown")
            ]
            
        )
        elif name == "bed":
            self.static.extend(
            [self.canvas.create_rectangle(
                        x1, y1, x2, y2, fill="cornsilk3", outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name),]
        )

        elif name == "bathtub" or name == "toilet" or name == "br-vanity":
            self.static.extend(
            [self.canvas.create_rectangle(
                        x1, y1, x2, y2, fill="white", outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name),]
        )


        elif name == "nightstand" or name == "dresser":
            self.static.extend(
            [self.canvas.create_rectangle(
                        x1, y1, x2, y2, fill="burlywood4", outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name),]
        ) 
        elif name == "roundtable":
            self.static.extend(
            [ self.canvas.create_oval(
                    x1-20, y1-20, x2+20, y2+20,
                    fill="burlywood4",
                    outline="black",
                    width=2,
                ),
                self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name)]
        ) 

        elif name == "armchair":
            self.static.extend(
            [self.canvas.create_rectangle(
                        x1, y1, x2, y2, fill="burlywood1", outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name),]
        ) 
        else:
            self.static.extend(
                [
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2, fill="cadetblue", outline="black", width=2
                    ),
                    self.canvas.create_text((x1 + x2) / 2, y2 - 10, text=name),
                ]
            )

    def draw_environment(self):
        # TODO: automatically draw in order
        self.static = []
        for name, region in self.regions.items():
            self.draw_region(region, name=name)

        if len(self.obstacles) != 0:
            for obstacle in self.obstacles:
                self.draw_obstacle(obstacle, name="")

    def draw_obstacle(self, obs, name=""):
        rooms = ['kitchen', 'rest-room', 'closet', 'office room']
        x1, x2 = obs[0][0], obs[1][0]
        y1, y2 = obs[0][1], obs[1][1]
        self.static.extend(
            [
                self.canvas.create_rectangle(
                    x1, y1, x2, y2, fill="azure1", outline="black", width=2
                ),
                self.canvas.create_text((x1 + x2) / 2, (y1 + y2)/2, text=""),
            ]
        )



    def draw_robot(self, x, y, name="", color="grey"):
        x1 = x + SUCTION_WIDTH
        x2 = x - SUCTION_WIDTH
        y1 = y + SUCTION_WIDTH
        y2 = y - SUCTION_WIDTH
        self.dynamic.extend(
            [
                self.canvas.create_oval(
                    x + SUCTION_HEAD,
                    y + SUCTION_HEAD,
                    x - SUCTION_HEAD,
                    y - SUCTION_HEAD,
                    fill=color,
                    outline="black",
                    width=2,
                ),   
                self.canvas.create_rectangle(
                    x - SUCTION_WIDTH + 10,
                    y + 15,
                    x + SUCTION_WIDTH - 10,
                    y + 1.5 * SUCTION_WIDTH,
                    fill=color,
                    outline="black",
                    width=2,
                ),
                self.canvas.create_text((x1 + x2) / 2, ((y + 15) + (y + 1.5 * SUCTION_WIDTH)) / 2, text="rob"),
                self.canvas.create_rectangle(
                    x - SUCTION_WIDTH + 5,
                    y + SUCTION_WIDTH - 5,
                    x - SUCTION_WIDTH,
                    y - SUCTION_WIDTH + 15,
                    fill=color,
                    outline="black",
                    width=2,
                ),
                self.canvas.create_rectangle(
                    x + SUCTION_WIDTH - 5,
                    y + SUCTION_WIDTH - 5,
                    x + SUCTION_WIDTH,
                    y - SUCTION_WIDTH + 15,
                    fill=color,
                    outline="black",
                    width=2,
                ),
            ]
        )

    def draw_state(self, state, colors):
        self.clear_state()
        for robot, conf in state.robot_confs.items():
            x, y = conf
            self.draw_robot(x, y)
        for block, pose in state.block_poses.items():
            a, b = pose
            self.draw_block(a, b, name=block, color=colors[block])
        for robot, holding in state.holding.items():
            block, grasp = holding
            pose = forward_kin(state.robot_confs[robot], grasp)
            c, d = pose
            self.draw_block(c, d, name=block, color=colors[block])
        self.tk.update()

    def clear_state(self):
        for part in self.dynamic:
            self.canvas.delete(part)
        self.dynamic = []

    def clear_all(self):
        self.canvas.delete("all")

    def return_image(self):
        """
        This function returns the image from PDDLStream viewer since it doesn't have it.
        """

        try:
            import pyscreenshot as ImageGrab
        except ImportError:
            print("Unable to load pyscreenshot")
            return None
        x, y = self.top.winfo_x(), 2 * self.top.winfo_y()
        width, height = self.top.winfo_width(), self.top.winfo_height()
        img = ImageGrab.grab((x, y, x + width, y + height))
        return img