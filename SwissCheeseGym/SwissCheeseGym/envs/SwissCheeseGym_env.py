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
     return

    def step(self, action):
        print('step)

    def reset(self):
        print('reset')
    
    def render(self, mode='human'):
        print('render')
    
    def close(self):
        print('close')