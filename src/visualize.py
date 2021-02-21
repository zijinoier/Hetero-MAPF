#!/usr/bin/env python3
from matplotlib import animation
import matplotlib.animation as manimation
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import math
import os
import argparse
import numpy as np
import yaml
import matplotlib
matplotlib.use("Qt5Agg")

PLOTLINE = True  # True

carWidth = 2.0
LF = 2.0
LB = 1.0
obsRadius = 1
framesPerMove = 3
lf = []
lb = []
width = []

pause = False


def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)


class Animation:
    def onClick(self, event):
        global pause
        pause ^= True
        if (pause):
            self.anim.event_source.stop()
        else:
            self.anim.event_source.start()

    def __init__(self, map, schedule):
        self.map = map
        self.schedule = schedule

        aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

        self.fig = plt.figure(frameon=False, figsize=(8 * aspect, 8))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(
            left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        self.fig.canvas.mpl_connect('button_press_event', self.onClick)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.lines = []
        self.list_xdata = [[] for _ in range(0, len(map["agents"]))]
        self.list_ydata = [[] for _ in range(0, len(map["agents"]))]
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch
        xmin = -1
        ymin = -1
        xmax = map["map"]["dimensions"][0] + 0.5
        ymax = map["map"]["dimensions"][1] + 0.5

        # self.ax.relim()
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        # plt.xlim(10, 20)
        # plt.ylim(10, 20)
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        # plt.axis('off')
        # self.ax.axis('tight')
        self.ax.axis('off')

        self.patches.append(Rectangle(
            (xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='black', linewidth=1))
        if ("obs_radius" in map["map"]):
            obsRadius = map["map"]["obs_radius"]
        for o in map["map"]["obstacles"]:
            x, y = o[0], o[1]
            self.patches.append(
                # Circle((x, y), obsRadius, facecolor='grey', edgecolor='grey'))
                Rectangle((x - obsRadius/1.4, y - obsRadius/1.4), obsRadius*2/1.4, obsRadius*2/1.4, facecolor='grey', edgecolor='grey'))

        # create agents:
        self.T = 0
        cmap = get_cmap(len(map["agents"])+1)
        # draw goals first
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            if ("model" in d):
                if (d["model"]["type"] == "Ackermann"):
                    if ("width" in d["model"] and "width" in d["model"] and "width" in d["model"]):
                        lf.append(d["model"]["lf"])
                        lb.append(d["model"]["lb"])
                        width.append(d["model"]["width"])
                    else:
                        lf.append(LF)
                        lb.append(LB)
                        width.append(carWidth)
                elif(d["model"]["type"] == "Holonomic"):
                    if ("width" in d["model"]):
                        lf.append(d["model"]["width"]/2)
                        lb.append(d["model"]["width"]/2)
                        width.append(d["model"]["width"])
                    else:
                        lf.append(carWidth/2)
                        lb.append(carWidth/2)
                        width.append(carWidth)
                else:
                    print("Unrecognize model of agent", i)
                    lf.append(LF)
                    lb.append(LB)
                    width.append(carWidth)
            else:
                print("Undefined model of agent", i)
                lf.append(LF)
                lb.append(LB)
                width.append(carWidth)
            # cw = carWidth
            # lb = LB
            # lf = LF
            if ("goal" in d):
                self.patches.append(Rectangle(
                    (d["goal"][0] - math.sqrt(width[i]/2*width[i]/2+lb[i]*lb[i]) * math.sin(math.atan2(lb[i], width[i]/2)+d["goal"][2]),
                     d["goal"][1] - math.sqrt(width[i]/2*width[i]/2+lb[i]*lb[i]) * math.cos(math.atan2(lb[i], width[i]/2)+d["goal"][2])),
                    lb[i]+lf[i], width[i], -d["goal"][2] / math.pi * 180,
                    facecolor='none', edgecolor=cmap(i+1),  alpha=0.7, lw=1, linestyle=":"))
            if ("start" in d):
                self.list_xdata[i].append(d["start"][0])
                self.list_ydata[i].append(d["start"][1])
            line, = self.ax.plot(
                self.list_xdata[i], self.list_ydata[i], color=cmap(i+1),  alpha=0.5, lw=1, linestyle="-.")
            self.lines.append(line)

        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            name = d["name"]
            if ("start" in d):
                self.agents[name] = Rectangle(
                    (d["start"][0], d["start"][1]), 1, 2, 0.0, edgecolor='black', alpha=0.7)
            else:
                self.agents[name] = Rectangle(
                    (0, 0), 1, 2, 0.0, edgecolor='black', alpha=0.7)
            self.agents[name].original_face_color = cmap(i+1)
            self.patches.append(self.agents[name])
            self.T = max(self.T, schedule["schedule"][name][-1]["t"])
            if ("start" in d):
                self.agent_names[name] = self.ax.text(
                    d["start"][0], d["start"][1], name.replace('agent', ''), fontsize=6)
            else:
                self.agent_names[name] = self.ax.text(
                    0, 0, name.replace('agent', ''), fontsize=6)
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()
        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T+1) *
                                            framesPerMove,
                                            interval=100,
                                            repeat=False,
                                            blit=True)

    def save(self, file_name, speed):
        # self.anim.save(
        #     file_name,
        #     "ffmpeg",
        #     fps=10 * speed,
        #     dpi=300),
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})
        '''uncomment if want to save as gif'''
        self.anim.save(file_name.replace('.mp4', '') +
                       '.gif', writer='imagemagick', fps=15)

    def show(self):
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, i):
        for agent_name in self.schedule["schedule"]:
            agent = schedule["schedule"][agent_name]
            pos = self.getState(i / framesPerMove, agent)
            p = (pos[0], pos[1])
            # print(pos[0], pos[1], -pos[2] / 3.15159 * 180)'
            # cw = carWidth
            # lb = LB
            # lf = LF
            agentid = int(agent_name.replace("agent", ""))
            self.agents[agent_name]._x0 = pos[0] - math.sqrt(
                width[agentid]/2 * width[agentid]/2+lb[agentid]*lb[agentid]) * math.sin(math.atan2(lb[agentid], width[agentid]/2)+pos[2])
            self.agents[agent_name]._y0 = pos[1] - math.sqrt(
                width[agentid]/2 * width[agentid]/2+lb[agentid]*lb[agentid]) * math.cos(math.atan2(lb[agentid],  width[agentid]/2)+pos[2])
            self.agents[agent_name]._x1 = self.agents[agent_name]._x0 + \
                lb[agentid]+lf[agentid]
            self.agents[agent_name]._y1 = self.agents[agent_name]._y0 + \
                width[agentid]
            self.agents[agent_name].angle = -pos[2] / math.pi * 180
            self.agent_names[agent_name].set_position(p)
            if PLOTLINE:
                self.list_xdata[int(agent_name.replace(
                    "agent", ""))].append(pos[0])
                self.list_ydata[int(agent_name.replace(
                    "agent", ""))].append(pos[1])
                self.lines[int(agent_name.replace(
                    "agent", ""))].set_data(self.list_xdata[int(agent_name.replace(
                        "agent", ""))], self.list_ydata[int(agent_name.replace(
                            "agent", ""))])

        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        return self.patches + self.artists+self.lines

    def getState(self, t, d):
        idx = 0
        while idx < len(d) and d[idx]["t"] < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0]["x"]), float(d[0]["y"]), float(d[0]["yaw"])])
        elif idx < len(d):
            yawLast = float(d[idx-1]["yaw"])
            yawNext = float(d[idx]["yaw"])
            if (yawLast - yawNext) > math.pi:
                yawLast = yawLast - 2 * math.pi
            elif (yawNext - yawLast) > math.pi:
                yawLast = yawLast + 2 * math.pi
            posLast = np.array(
                [float(d[idx-1]["x"]), float(d[idx-1]["y"]), yawLast])
            posNext = np.array(
                [float(d[idx]["x"]), float(d[idx]["y"]), yawNext])
            dt = d[idx]["t"] - d[idx-1]["t"]
            t = (t - d[idx-1]["t"]) / dt
            pos = (posNext - posLast) * t + posLast
            return pos
        else:
            return np.array([float(d[-1]["x"]), float(d[-1]["y"]), float(d[-1]["yaw"])])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", default=None,
                        help="input file containing map")
    parser.add_argument("-s", "--schedule", default=None,
                        help="schedule for agents")
    parser.add_argument("-v", "--video", dest='video', default=None,
                        help="output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int,
                        default=1, help="speedup-factor")
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.schedule) as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)

    try:
        with open(os.path.abspath(os.path.join(
                os.getcwd(), ".."))+"/src/config.yaml") as config_file:
            carConfig = yaml.load(config_file, Loader=yaml.FullLoader)
            # global carWidth, LF, LB, obsRadius, framesPerMove
            carWidth = carConfig["ackerWidth"]
            LF = carConfig["ackerLF"]
            LB = carConfig["ackerLB"]
            obsRadius = carConfig["obsRadius"]-0.1
    except IOError:
        # do things with your exception
        print("ERROR loading config file", os.path.abspath(os.path.join(
            os.getcwd(), ".."))+"/src/config1.yaml", " using default param to plot")

    animation = Animation(map, schedule)

    if args.video:
        matplotlib.use("Agg")
        animation.save(args.video, args.speed)
    else:
        animation.show()
