import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
map = world.get_map()

for idx, sp in enumerate(map.get_spawn_points()):
    world.debug.draw_string(
        sp.location, f'{idx}',
        draw_shadow=False,
        color=carla.Color(r=255, g=0, b=0),
        life_time=60.0,
        persistent_lines=False,
        size=2.0
    )
