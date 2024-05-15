import numpy as np
import cv2

def getDist(pos,obs):
    return np.linalg.norm(pos[0:2]-obs[0:2])


def create_obstacles_from_img(image,sim_time, num_timesteps):
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 10
    params.maxThreshold = 150
    params.filterByArea = True
    params.minArea = 200
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)

    keypoints = detector.detect(image)
    blank = np.zeros((1, 1))
    obstacles = None
    for keyPoint in keypoints:
        x = int(keyPoint.pt[0])
        y = int(keyPoint.pt[1])
        s = int(keyPoint.size)
        pos = np.asarray([x,y])
        obst = create_robot(pos, 0, 0, sim_time,num_timesteps).reshape(4, num_timesteps, 1)
        if obstacles is None:
            obstacles = obst
        obstacles = np.dstack((obstacles, obst))
        cv2.circle(image, (x,y), s//2, (0,255,0), 2)
    cv2.imshow("Detected Circles", image)
    cv2.waitKey(500)
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

