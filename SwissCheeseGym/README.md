# Installing the gym

pip install -e SwissCheeseGym

This needs to be performed at the top level of the directory.

Then to run:
>>> import gym
>>> custom_gym = gym.make('SwissCheeseGym:SwissCheeseGym-v0')
init basic
>>> custom_gym.step('some action')
step
>>> custom_gym.reset()
reset
>>> custom_gym.render()
render
>>> custom_gym.close()
close