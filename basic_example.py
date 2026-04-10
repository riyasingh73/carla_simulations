import carla
import os
import sys
import time
import random

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.basic_agent import BasicAgent

def main():
    client=carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    world=client.get_world()

    spawn_points=world.get_map().get_spawn_points()

    source_spawn_point=random.choice(spawn_points)
    blueprint=world.get_blueprint_library().filter('vehicle.*')[0]
    vehicle=world.try_spawn_actor(blueprint, source_spawn_point)

    if vehicle is None:
        print("Vehicle spawn failed")
        return
    
    agent=BasicAgent(vehicle)

    destination = random.choice(spawn_points).location

    settings=world.get_settings()
    original_settings=settings

    agent.set_destination(destination)

    try:
        settings.synchronous_mode=True
        settings.fixed_delta_seconds=0.05
        world.apply_settings(settings)


        while(True):
            world.tick()
            control=agent.run_step()
            vehicle.apply_control(control)

            spectator = world.get_spectator()
            transform = vehicle.get_transform()

            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(z=30),
                carla.Rotation(pitch=-90)
            ))
            current_location = vehicle.get_location()
            if(agent.done()):
                print("Reached destination at ", current_location.x, current_location.y, current_location.z)
                break

    finally:
        if world is not None:
            try:
                settings=world.get_settings()
                settings.synchronous_mode=False
                settings.fixed_delta_seconds=None
                world.apply_settings(settings)
            except Exception as e:
                print("failed to restore world settings: ", e)

            try:
                vehicle.destroy()
            except Exception as e:
                print("Failed to destroy world: ", e)


if __name__ == "__main__":
    main()
