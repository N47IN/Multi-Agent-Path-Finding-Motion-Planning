import numpy as np
import cv2

def getDist(pos,obs):
    return np.linalg.norm(pos[0:2]-obs[0:2])

def create_obstacle(pos,state1,state2, sim_time, num_timesteps):
    """
    Creates obstacles for the simulation.
    """
    if getDist(state1,pos) < 1.6 :
        # Obstacle 1
        p0 = np.array([state1[0], state1[1]])
        #print(state1)
        obst = create_robot(p0, state1[2], state1[3], sim_time,
                            num_timesteps).reshape(4, num_timesteps, 1)
        obstacles = obst
    
    if getDist(state2,pos) < 1.6 :
        p0 = np.array([state2[0], state2[1]])
        obst = create_robot(p0, state2[2], state2[3], sim_time,
                            num_timesteps).reshape(4, num_timesteps, 1)
        if obstacles is None:
            obstacles = obst
        obstacles = np.dstack((obstacles, obst))
    return obstacles


def create_robot(p0, v, theta, sim_time, num_timesteps):
    # Creates obstacles starting at p0 and moving at v in theta direction
    t = np.linspace(0, sim_time, num_timesteps)
    theta = theta * np.ones(np.shape(t))
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    v = np.stack([vx, vy])
    p0 = p0.reshape((2, 1))
    p = p0 + np.cumsum(v, axis=1) * (sim_time / num_timesteps)
    p = np.concatenate((p, v))
    return p
