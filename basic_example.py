import carla
import can
import cantools
import os
import sys
import time
import random
from pathlib import Path

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.basic_agent import BasicAgent

ATTACK_START=36
ATTACK_STOP=36.006
SIM_TIME=42

def vehicle_run(world, bus, msg_def, blueprint, source_spawn_point, destination, sync_dt, sim_time, spoof_mode, spoof_delay):

    vehicle=world.try_spawn_actor(blueprint, source_spawn_point)

    if vehicle is None:
        print("Vehicle spawn failed")
        return
    
    agent=BasicAgent(vehicle)


    # original_settings=settings

    agent.set_destination(destination)
    max_ticks = int(sim_time/sync_dt)

    try:
        settings=world.get_settings()
        settings.synchronous_mode=True
        settings.fixed_delta_seconds=0.008
        world.apply_settings(settings)


        for tick in range(0, max_ticks):
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

            if tick%20 ==0:
                print("tick:", tick, "location:", current_location.x, current_location.y)

            if(agent.done()):
                print("Reached destination at ", current_location.x, current_location.y, current_location.z)
                break
    finally:
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


def main():
    client=carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    world=client.load_world('Town05_Opt')

    script_dir=Path(__file__).resolve().parent
    dbc_file = script_dir / 'dbc_input.dbc'

    bus=can.Bus(channel='vcan0', interface='socketcan')
    dbc=cantools.database.load_file(dbc_file)

    msg_def = dbc.get_message_by_name('TheMessage')
    if not msg_def.signals:
        raise RuntimeError('TheMessage has no signals in the DBC.')
    signal_name = msg_def.signals[0].name

    blueprint = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]

    spawn_points=world.get_map().get_spawn_points()

    source_spawn_point=spawn_points[20]
    destination = spawn_points[11].location
    
    vehicle_run(bus=bus,world=world,msg_def=msg_def ,blueprint=blueprint, source_spawn_point=source_spawn_point, destination=destination, sync_dt=0.008, sim_time=42, spoof_mode=False, spoof_delay=0.0025)
    print("Benign run done.")



    vehicle_run(bus=bus,world=world,msg_def=msg_def , blueprint=blueprint, source_spawn_point=source_spawn_point, destination=destination, sync_dt=0.008, sim_time=42, spoof_mode=True, spoof_delay=0.0025)
    print("Attack done.")

if __name__ == "__main__":
    main()