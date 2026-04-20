
import csv
import math
import os
import random
import sys
import timeit
from pathlib import Path

import can
import cantools
import carla

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/carla")
except IndexError:
    pass

from agents.navigation.behavior_agent import BehaviorAgent

ATTACK_START = 36.0
ATTACK_STOP = 36.006
SIM_TIME = 42.0
SYNC_DT = 0.006
MAP_NAME = "Town05_Opt"
SOURCE_INDEX = 11
DEST_INDEX = 20
OUTPUT_DIR = "Logs_36.0_1"


def add_delay(delay):
    current_time = timeit.default_timer()
    end_time = current_time + delay
    while current_time < end_time:
        current_time = timeit.default_timer()


def generate_jitter_array(min_jitter, max_jitter, size):
    return [random.uniform(min_jitter, max_jitter) for _ in range(size)]


def euclid(a, b):
    return math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 +
        (a.z - b.z) ** 2
    )


class SimpleCANLogger:
    def __init__(self, dbc_path):
        self.dbc = cantools.database.load_file(dbc_path)
        self.bus = can.Bus(channel="vcan0", interface="socketcan")
        self.steer_message = self.dbc.get_message_by_name("TheMessage")

    def send_steer(self, steer_value):
        signal_name = self.steer_message.signals[0].name
        data = self.steer_message.encode({signal_name: float(steer_value)})
        msg = can.Message(
            arbitration_id=self.steer_message.frame_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)

    def send_dummy(self, arbitration_id):
        msg = can.Message(arbitration_id=arbitration_id, data=[0x00] * 8, is_extended_id=True)
        self.bus.send(msg)

    def log_tick_messages(self, tick, steer_value, jitter_array):
        message_specs = [
            ("dummy_1", lambda: self.send_dummy(0x00000170), 0.0),
            ("steer", lambda: self.send_steer(steer_value), 0.0),
            ("dummy_2", lambda: self.send_dummy(0x00000202), 0.25),
            ("dummy_3", lambda: self.send_dummy(0x0000018F), 0.5),
        ]

        jitter_chunk = jitter_array[tick * len(message_specs):(tick + 1) * len(message_specs)]
        actions = []

        for i, (_, fn, period) in enumerate(message_specs):
            if period == 0.0:
                eligible = True
            else:
                ticks_per_period = max(1, int(period / SYNC_DT))
                eligible = (tick % ticks_per_period == 0)
            if eligible:
                actions.append((jitter_chunk[i], fn))

        for _, fn in sorted(actions, key=lambda x: x[0]):
            fn()
            add_delay(0.0002)

    def shutdown(self):
        self.bus.shutdown()


def make_control_copy(control):
    return carla.VehicleControl(
        throttle=control.throttle,
        steer=control.steer,
        brake=control.brake,
        hand_brake=control.hand_brake,
        reverse=control.reverse,
        manual_gear_shift=control.manual_gear_shift,
        gear=control.gear,
    )


def run_thesis_like_experiment(world, can_logger, blueprint, source_spawn_point, destination):
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    vehicle_2 = world.try_spawn_actor(blueprint, source_spawn_point)
    if vehicle_2 is None:
        raise RuntimeError("Reference vehicle spawn failed")

    attack_spawn = carla.Transform(
        carla.Location(
            x=source_spawn_point.location.x,
            y=source_spawn_point.location.y - 25.0,
            z=source_spawn_point.location.z,
        ),
        source_spawn_point.rotation,
    )
    vehicle_1 = world.try_spawn_actor(blueprint, attack_spawn)
    if vehicle_1 is None:
        vehicle_2.destroy()
        raise RuntimeError("Attacked vehicle spawn failed")

    opt_dict = {
        "follow_speed_limits": False,
        "ignore_traffic_lights": True,
        "ignore_stop_signs": True,
        "ignore_vehicles": True,
    }
    agent_1 = BehaviorAgent(vehicle_1, behavior="normal", opt_dict=opt_dict)
    agent_2 = BehaviorAgent(vehicle_2, behavior="normal", opt_dict=opt_dict)
    agent_1.set_destination(destination)
    agent_2.set_destination(destination)

    total_iterations = int(SIM_TIME / SYNC_DT)
    jitter_array = generate_jitter_array(0, 100, (total_iterations + 1000) * 4)

    coord_rows = []
    control_rows = []
    spoof_rows = []
    vc_time_rows = []

    num_spoof_msgs = 50
    spoof_delay = 0.0025
    spoof_mode = False
    count_spoof = 0

    original_settings = world.get_settings()
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = SYNC_DT
        world.apply_settings(settings)

        start_time = timeit.default_timer()

        for i in range(total_iterations):
            current_time = timeit.default_timer() - start_time
            world.tick()

            control_1 = agent_1.run_step()
            control_2 = agent_2.run_step()

            current_location_1 = vehicle_1.get_location()
            current_location_2 = vehicle_2.get_location()

            vehicle_control_apply_time = timeit.default_timer() - start_time
            vc_time_rows.append({
                "tick": i,
                "time": round(current_time, 6),
                "apply_time": round(vehicle_control_apply_time, 6),
            })

            vehicle_1.apply_control(control_1)
            vehicle_2.apply_control(control_2)
            can_logger.log_tick_messages(i, control_1.steer, jitter_array)

            control_rows.append({
                "tick": i,
                "time": round(current_time, 6),
                "vehicle": "attacked",
                "agent_steer": control_1.steer,
                "applied_steer": control_1.steer,
                "attack_active": 0,
            })
            control_rows.append({
                "tick": i,
                "time": round(current_time, 6),
                "vehicle": "reference",
                "agent_steer": control_2.steer,
                "applied_steer": control_2.steer,
                "attack_active": 0,
            })

            if spoof_mode or (current_time >= ATTACK_START and current_time <= ATTACK_STOP):
                spoof_mode = True
                count_spoof += 1
                if count_spoof >= num_spoof_msgs:
                    spoof_mode = False

                delay_target = current_time + spoof_delay
                current_timestamp = timeit.default_timer() - start_time
                while current_timestamp < delay_target:
                    current_timestamp = timeit.default_timer() - start_time

                spoof_control = make_control_copy(control_1)
                spoof_control.steer = -1.0

                spoof_timestamp = timeit.default_timer() - start_time
                vehicle_1.apply_control(spoof_control)
                can_logger.log_tick_messages(i, spoof_control.steer, jitter_array)

                spoof_rows.append({
                    "tick": i,
                    "time": round(spoof_timestamp, 6),
                    "spoofed_steer": spoof_control.steer,
                })
                control_rows.append({
                    "tick": i,
                    "time": round(spoof_timestamp, 6),
                    "vehicle": "attacked",
                    "agent_steer": control_1.steer,
                    "applied_steer": spoof_control.steer,
                    "attack_active": 1,
                })

            current_location_1 = vehicle_1.get_location()
            current_location_2 = vehicle_2.get_location()
            coord_rows.append({
                "tick": i,
                "time": round(current_time, 6),
                "attacked_x": current_location_1.x,
                "attacked_y": current_location_1.y,
                "attacked_z": current_location_1.z,
                "reference_x": current_location_2.x,
                "reference_y": current_location_2.y,
                "reference_z": current_location_2.z,
                "euclid_distance": euclid(current_location_1, current_location_2),
            })

            delay_time = current_time + SYNC_DT
            now = timeit.default_timer() - start_time
            while now < delay_time:
                now = timeit.default_timer() - start_time

        with open(os.path.join(OUTPUT_DIR, "gen_coord_pair.csv"), "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "tick", "time",
                    "attacked_x", "attacked_y", "attacked_z",
                    "reference_x", "reference_y", "reference_z",
                    "euclid_distance",
                ],
            )
            writer.writeheader()
            writer.writerows(coord_rows)

        with open(os.path.join(OUTPUT_DIR, "control_trace.csv"), "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["tick", "time", "vehicle", "agent_steer", "applied_steer", "attack_active"],
            )
            writer.writeheader()
            writer.writerows(control_rows)

        with open(os.path.join(OUTPUT_DIR, "spoof_timestamp.csv"), "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["tick", "time", "spoofed_steer"])
            writer.writeheader()
            writer.writerows(spoof_rows)

        with open(os.path.join(OUTPUT_DIR, "vehicle_control_time.csv"), "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["tick", "time", "apply_time"])
            writer.writeheader()
            writer.writerows(vc_time_rows)

    finally:
        world.apply_settings(original_settings)
        vehicle_1.destroy()
        vehicle_2.destroy()


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)
    world = client.load_world(MAP_NAME)

    script_dir = Path(__file__).resolve().parent
    dbc_file = script_dir / "dbc_input.dbc"

    blueprint = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()

    can_logger = SimpleCANLogger(dbc_file)
    run_thesis_like_experiment(
        world=world,
        can_logger=can_logger,
        blueprint=blueprint,
        source_spawn_point=spawn_points[SOURCE_INDEX],
        destination=spawn_points[DEST_INDEX].location,
    )
    can_logger.shutdown()
    print("Thesis-like experiment run done.")


if __name__ == "__main__":
    main()
