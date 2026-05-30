import argparse
import os
import signal
import subprocess
import time

from stable_baselines3 import SAC # pyright: ignore[reportMissingImports]
from stable_baselines3.common.vec_env import DummyVecEnv  # pyright: ignore[reportMissingImports]

from turtle_sac_env import TurtleSacEnv


def launch_turtlesim(namespace: str, quiet: bool = True):
    cmd = [
        "ros2",
        "run",
        "turtlesim",
        "turtlesim_node",
        "--ros-args",
        "-r",
        f"__ns:=/{namespace}",
        "-r",
        f"__node:=turtlesim_{namespace}",
    ]
    stdout = subprocess.DEVNULL if quiet else None
    stderr = subprocess.DEVNULL if quiet else None
    return subprocess.Popen(cmd, stdout=stdout, stderr=stderr, preexec_fn=os.setsid)


def stop_processes(processes):
    for p in processes:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                p.terminate()


def make_env(rank: int, show_goal_marker: bool):
    def _init():
        return TurtleSacEnv(
            namespace=f"env{rank}",
            node_name=f"turtle_sac_test_env_{rank}",
            show_goal_marker=show_goal_marker,
        )

    return _init


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sac_turtlesim_multi", help="Đường dẫn model .zip")
    parser.add_argument("--num-envs", type=int, default=4)
    parser.add_argument("--no-launch", action="store_true")
    parser.add_argument("--show-all-goals", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    processes = []
    if not args.no_launch:
        for i in range(args.num_envs):
            ns = f"env{i}"
            print(f"[LAUNCH] turtlesim namespace /{ns}")
            processes.append(launch_turtlesim(ns, quiet=args.quiet))
            time.sleep(0.5)
        time.sleep(2.0)

    env = None
    try:
        env = DummyVecEnv([
            make_env(i, show_goal_marker=(args.show_all_goals or i == 0))
            for i in range(args.num_envs)
        ])
        model = SAC.load(args.model)

        obs = env.reset()
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, rewards, dones, infos = env.step(action)
            for i, done in enumerate(dones):
                if done:
                    print(f"[env{i}] done, info={infos[i]}")
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nDừng test")

    finally:
        if env is not None:
            env.close()
        if processes:
            stop_processes(processes)


if __name__ == "__main__":
    main()
