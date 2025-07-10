import carla
from carla import TrafficManager

client = carla.Client("localhost", 12338)
client.set_timeout(10.0)
world = client.get_world()

tm = client.get_trafficmanager(12341)
tm.set_global_distance_to_leading_vehicle(1000.0)  # Keeps AI vehicles far
tm.global_percentage_speed_difference(100.0)       # AI stops

# Optional: destroy all NPC vehicles
for actor in world.get_actors().filter('vehicle.*'):
    if not actor.attributes.get('role_name') == 'hero':
        actor.destroy()
