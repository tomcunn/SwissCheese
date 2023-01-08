import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
import pygame
import time
from os import path


class SwissCheeseGymClass(gym.Env):
    metadata = {'render.modes': ['human']}
  
    def __init__(self):
        print('init basic')
        self.action_space = spaces.Discrete(4)
        height = 600
        width = 470
        movementparms = np.array([width,height])
        self.observation_space =spaces.Box(0, movementparms, dtype=np.float32)
        return

    def step(self, action):
        action = 1
        print('step')

    def reset(self, seed=None, options=None):
        print('reset')
    
    def render(self, mode='human'):
        print('render')
    
    def close(self):
        print('close')