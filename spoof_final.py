
import csv
import math
import os
import sys
from pathlib import Path

import carla
import can
import cantools
import matplotlib.pyplot as plt

try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

from agents.navigation.behavior_agent import BehaviorAgent

SIM_TIME = 42.0
SYNC_DT = 0.008
ATTACK_START = 36.0
ATTACK_STOP = 36.006
ATTACK_STEER = -1.0
MAP_NAME = "Town05_Opt"

OUT_DIR = Path("visual_run_outputs")
OUT_DIR.mkdir(exist_ok=True)

BENIGN_CSV = OUT_DIR / "visual_benign.csv"
ATTACK_CSV = OUT_DIR / "visual_attack.csv"
DIST_CSV = OUT_DIR / "visual_distance.csv"

PATH_PNG = OUT_DIR / "visual_paths.png"
STEER_PNG = OUT_DIR / "visual_steer.png"
DIST_PNG = OUT_DIR / "visual_distance.png"
DBC_FILE = Path(__file__).resolve().parent / "dbc_input.dbc"

def write_csv(path, rows, fieldnames):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

def clamp_percent(value):
    return max(0, min(100, int(round(value * 100))))


def send_can_control(bus, msg_def, steer, throttle, brake):
    payload = msg_def.encode({
        "Steer": int(round(steer * 127)),
        "Throttle": clamp_percent(throttle),
        "Brake": clamp_percent(brake),
    })
    msg = can.Message(
        arbitration_id=msg_def.frame_id,
        data=payload,
        is_extended_id=False,
    )
    bus.send(msg)

def euclid(a, b):
    return math.sqrt(
        (a["x"] - b["x"]) ** 2 +
        (a["y"] - b["y"]) ** 2 +
        (a["z"] - b["z"]) ** 2
    )


def update_spectator_topdown(world, vehicle):
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(z=40),
            carla.Rotation(pitch=-90)
        )
    )


def make_agent(vehicle):
    opt_dict = {
        "follow_speed_limits": False,
        "ignore_traffic_lights": True,
        "ignore_stop_signs": True,
        "ignore_vehicles": True,
    }
    return BehaviorAgent(vehicle, behavior="normal", opt_dict=opt_dict)


def plot_outputs(benign_rows, attack_rows, dist_rows):
    # paths
    plt.figure()
    plt.plot([r["x"] for r in benign_rows], [r["y"] for r in benign_rows], label="benign")
    plt.plot([r["x"] for r in attack_rows], [r["y"] for r in attack_rows], label="attack")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Benign vs attack path")
    plt.legend()
    plt.tight_layout()
    plt.savefig(PATH_PNG)
    plt.close()

    # steer timeline
    plt.figure()
    plt.plot([r["time"] for r in benign_rows], [r["applied_steer"] for r in benign_rows], label="benign applied steer")
    plt.plot([r["time"] for r in attack_rows], [r["applied_steer"] for r in attack_rows], label="attack applied steer")
    plt.xlabel("time (s)")
    plt.ylabel("steer")
    plt.title("Applied steer over time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(STEER_PNG)
    plt.close()

    # distance timeline
    plt.figure()
    plt.plot([r["time"] for r in dist_rows], [r["distance"] for r in dist_rows])
    plt.xlabel("time (s)")
    plt.ylabel("euclidean distance")
    plt.title("Vehicle separation over time")
    plt.tight_layout()
    plt.savefig(DIST_PNG)
    plt.close()


def run_visual_experiment():
    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)
    world = client.load_world(MAP_NAME)

    dbc = cantools.database.load_file(DBC_FILE)
    msg_def = dbc.get_message_by_name("TheMessage")
    bus = can.Bus(channel="vcan0", interface="socketcan")
 

    blueprint = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()

    benign_spawn = spawn_points[11]
    attack_spawn = carla.Transform(
        carla.Location(
            x=benign_spawn.location.x,
            y=benign_spawn.location.y - 25.0,
            z=benign_spawn.location.z,
        ),
        benign_spawn.rotation,
    )
    destination = spawn_points[20].location

    benign_vehicle = None
    attack_vehicle = None

    benign_rows = []
    attack_rows = []
    dist_rows = []

    try:
        benign_vehicle = world.try_spawn_actor(blueprint, benign_spawn)
        attack_vehicle = world.try_spawn_actor(blueprint, attack_spawn)

        if benign_vehicle is None or attack_vehicle is None:
            raise RuntimeError("Vehicle spawn failed")

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = SYNC_DT
        world.apply_settings(settings)

        benign_agent = make_agent(benign_vehicle)
        attack_agent = make_agent(attack_vehicle)
        benign_agent.set_destination(destination)
        attack_agent.set_destination(destination)

        max_ticks = int(SIM_TIME / SYNC_DT)

        for tick in range(max_ticks):
            world.tick()
            sim_time_now = tick * SYNC_DT

            benign_control = benign_agent.run_step()
            attack_control = attack_agent.run_step()

            applied_attack = carla.VehicleControl(
                throttle=attack_control.throttle,
                steer=attack_control.steer,
                brake=attack_control.brake,
                hand_brake=attack_control.hand_brake,
                reverse=attack_control.reverse,
                manual_gear_shift=attack_control.manual_gear_shift,
                gear=attack_control.gear,
            )

            attack_active = ATTACK_START <= sim_time_now <= ATTACK_STOP
            if attack_active:
                applied_attack.steer = ATTACK_STEER

            benign_vehicle.apply_control(benign_control)
            attack_vehicle.apply_control(applied_attack)

            send_can_control(
                            bus,
                            msg_def,
                            benign_control.steer,
                            benign_control.throttle,
                            benign_control.brake,
                        )
            send_can_control(
                            bus,
                            msg_def,
                            applied_attack.steer,
                            applied_attack.throttle,
                            applied_attack.brake,
                        )
 

            update_spectator_topdown(world, attack_vehicle)

            benign_loc = benign_vehicle.get_location()
            attack_loc = attack_vehicle.get_location()

            benign_rows.append({
                "tick": tick,
                "time": round(sim_time_now, 6),
                "applied_throttle": benign_control.throttle,
                "applied_steer": benign_control.steer,
                "applied_brake": benign_control.brake,
                "x": benign_loc.x,
                "y": benign_loc.y,
                "z": benign_loc.z,
            })
            attack_rows.append({
                "tick": tick,
                "time": round(sim_time_now, 6),
                "agent_steer": attack_control.steer,
                "agent_throttle": attack_control.throttle,
                "agent_brake": attack_control.brake,
                "applied_throttle": applied_attack.throttle,
                "applied_steer": applied_attack.steer,
                "applied_brake": applied_attack.brake,
                "attack_active": int(attack_active),
                "x": attack_loc.x,
                "y": attack_loc.y,
                "z": attack_loc.z,
            })

            dist_rows.append({
                "tick": tick,
                "time": round(sim_time_now, 6),
                "distance": euclid(benign_rows[-1], attack_rows[-1]),
            })

            if tick % 20 == 0:
                print(
                    f"tick={tick} t={sim_time_now:.3f} "
                    f"benign=({benign_loc.x:.2f},{benign_loc.y:.2f}) "
                    f"attack=({attack_loc.x:.2f},{attack_loc.y:.2f}) "
                    f"dist={dist_rows[-1]['distance']:.3f} "
                    f"attack_active={int(attack_active)}"
                )

            if benign_agent.done() and attack_agent.done():
                print("Both vehicles reached destination.")
                break

        write_csv(
            BENIGN_CSV,
            benign_rows,
            ["tick", "time", "applied_throttle", "applied_steer", "applied_brake", "x", "y", "z"],
        )
        write_csv(
            ATTACK_CSV,
            attack_rows,
            [
                "tick", "time",
                "agent_throttle", "agent_steer", "agent_brake",
                "applied_throttle", "applied_steer", "applied_brake",
                "attack_active", "x", "y", "z"
            ])
        write_csv(
            DIST_CSV,
            dist_rows,
            ["tick", "time", "distance"],
        )
        plot_outputs(benign_rows, attack_rows, dist_rows)
        print(f"Saved outputs in: {OUT_DIR.resolve()}")

    finally:
        try:
            bus.shutdown()
        except Exception:
            pass

        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        except Exception as e:
            print("failed to restore world settings:", e)

        if benign_vehicle is not None:
            benign_vehicle.destroy()
        if attack_vehicle is not None:
            attack_vehicle.destroy()


if __name__ == "__main__":
    run_visual_experiment()
