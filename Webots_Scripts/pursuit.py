"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
from ccma import CCMA
import matplotlib.pyplot as plt

# Parameters
k = 0.7  # look forward gain
Lfc = 0.05  # [m] look-ahead distance
Kp = 0.43  # speed proportional gain
dt = 0.032  # [s] time tick
WB = 0.13  # [m] wheel base of vehicle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def plot_arrow(x, y, yaw, length=0.3, width=0.1, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    
    points = [[4.969661498981831, 0.47000000028253197], [5.000194878557161, 0.5652245387781784], 
            [4.971607956682302, 0.6610514029537358], [4.920712668403563, 0.747130840864447], 
            [4.889197898138004, 0.842035104494218], [4.846695630464466, 0.932553374708617], 
            [4.841616655583482, 1.0324243114928402], [4.777780797596064, 1.1093982241841553], 
            [4.8310273632925815, 1.1940433849526624], [4.7463995297507875, 1.247317485508396], 
            [4.670745000661553, 1.3127116154306309], [4.584751631376151, 1.3637521918276954], 
            [4.48631463849771, 1.3461408763444718], [4.3892407271078815, 1.3221272192410694], 
            [4.289392704375341, 1.3276383313250382], [4.199484383024113, 1.3714161082421503], 
            [4.102886566372709, 1.3455537452762132], [4.009315780405988, 1.3102761601325286], 
            [3.9101507186721642, 1.2973807915326427], [3.8221657754515164, 1.3449060455189117], 
            [3.724918489453801, 1.3216044282772133], [3.641951626075482, 1.377430045883858], 
            [3.594892604239847, 1.4656652342378997], [3.4994403495224256, 1.435851360464763], 
            [3.458171842888484, 1.5269387380766162], [3.3789697816491633, 1.5879881729261125],
            [3.322177465532087, 1.6702963306631076], [3.2817903634467753, 1.7617779243590561], 
            [3.2445743546689734, 1.8545947799977307], [3.2047808843642906, 1.9463361540942108],
            [3.1663590970373696, 2.038660399331336], [3.1415969753435355, 2.1355460914082354], 
            [3.0917747128463446, 2.222251005684538], [2.991859481161734, 2.2263676156260512], 
            [2.929490001563737, 2.3045344117397146], [2.9024522060421507, 2.4008098375415923], 
            [2.825842648172669, 2.465082503917296], [2.8115075698486924, 2.5640496981255056], 
            [2.711817299086269, 2.5719141698378762], [2.6773151667394095, 2.665773654513213], 
            [2.6499817078805856, 2.761965556598643], [2.5543427736678597, 2.791175045855628], 
            [2.5519768675308665, 2.891147054378769], [2.5286325165618617, 2.9883840909329336], 
            [2.5141694195004667, 3.087332657488456], [2.492067090951566, 3.184859510573729], 
            [2.5125556031709664, 3.2827381132764417], [2.4505509971796937, 3.3611946543413334],
            [2.4726166306431847, 3.4587298163079536], [2.4820848421181734, 3.5582805720633215],
            [2.470081402525539, 3.6575575454153832], [2.3719594791280247, 3.676847131952484], 
            [2.3583915584261974, 3.7759224140758723], [2.362712865718734, 3.875829001963176], 
            [2.4384025582866884, 3.9411824291033692], [2.416510525990543, 4.0387567029768885],
            [2.4335211016796854, 4.137299284203097], [2.3823113910960565, 4.22319204025354],
            [2.373569322107944, 4.322809188525347], [2.3966018519764627, 4.4201205576856655], 
            [2.3706356078157786, 4.5166905021], [2.3860832505364056, 4.615490149540545], 
            [2.3141350109706957, 4.6849412173753535], [2.2240251865223817, 4.7283027157107895], 
            [2.1750453579892834, 4.815486294421853], [2.1337292294519536, 4.9065520812063905], 
            [2.1461502945138617, 5.00577766834362], [2.0627671896400033, 5.060979641330055], 
            [1.9668824314140332, 5.032587509674484], [1.935995381275777, 5.127697919902581], 
            [1.8838241374150941, 5.213010060386976], [1.9339894823369146, 5.299516928142518], 
            [1.916086689440188, 5.397901327345323], [1.9290472633842157, 5.497057888010977], 
            [1.9135824288037546, 5.595854845916567], [1.971634933796717, 5.677279084715846], 
            [1.912150897161015, 5.757663469925428], [1.9403208751920378, 5.85361372963186], 
            [1.9991052767351951, 5.934511159351307], [2.0044146456881324, 6.03437011288739],
            [2.053179765741615, 6.121673967705888], [2.1433624473923, 6.16488373431358], 
            [2.20280856027557, 6.2452961690676005], [2.26225467315884, 6.325708603821621], 
            [2.3217007860421104, 6.406121038575641], [2.4045810789074173, 6.462075100577974], 
            [2.462995117553687, 6.543240364632446], [2.5209878589938004, 6.624707179595719], [2.5373800005768077, 6.723354519622314], [2.5774061141728803, 6.814994632184414], [2.519126198043979, 6.8962562552054205], [2.5855538140790806, 6.971004979795831], [2.6278296030949915, 7.061629246209867], [2.7113964754829554, 7.116552627745364], [2.790322029293129, 7.177959117270148], [2.8783208228309234, 7.225458720804205], [2.940915869433865, 7.3034450025515465], [3.0248496305163655, 7.357806052491238], [3.0784991580976775, 7.4421963848184784], [3.138786948669867, 7.521979730874682], [3.2249241554156933, 7.572777187641993], [3.2030866385298027, 7.670363676749135], [3.167885097550668, 7.7639630972211355], [3.2121743649551586, 7.853620560896914], [3.3013094468289075, 7.898951975824755], [3.3745842127033265, 7.967002021276974], [3.5, 8.0]]
    points =np.asarray(points)

# Create the CCMA-filter object
    w_ma = 4
    w_cc = 2
    ccma = CCMA(w_ma, w_cc, distrib="hanning")
    g_plan_smoothed = ccma.filter(points, cc_mode=False)

    cx = g_plan_smoothed[:,0]
    cy = g_plan_smoothed[:,1]
    target_speed = 0.2  # [m/s]
    T = 100.0  # max simulation time
    # initial state
    state = State(x=4.969661498981831, y=0.4700000002825319, yaw=0.0, v=0.0)
    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)
    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)
        state.update(ai, di)  # Control vehicle

        time += dt
        states.append(time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
