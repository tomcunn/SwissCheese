from gym.envs.registration import register

register(
    id='SwissCheeseGym-v0',
    entry_point='SwissCheeseGym.envs:SwissCheeseGymClass',
)