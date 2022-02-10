from gym.envs.registration import register

register(
    id='la_robo_liga_arena-v0',
    entry_point='LRL_main_arena.envs:LaRoboLigaPs2Arena',
)