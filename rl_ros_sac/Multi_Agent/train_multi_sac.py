import argparse
import os
import signal
import subprocess
import time

from stable_baselines3 import SAC # pyright: ignore[reportMissingImports]
from stable_baselines3.common.monitor import Monitor # pyright: ignore[reportMissingImports]
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor # pyright: ignore[reportMissingImports]

from turtle_sac_env import TurtleSacEnv


def launch_turtlesim(namespace: str, quiet: bool = True):
    """Launch một turtlesim_node trong namespace riêng."""
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

    return subprocess.Popen(
        cmd,
        stdout=stdout,
        stderr=stderr,
        preexec_fn=os.setsid,
    )


def stop_processes(processes):
    for p in processes:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                p.terminate()

    time.sleep(0.5)

    for p in processes:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                p.kill()


def make_env(rank: int, show_goal_marker: bool):
    """Hàm tạo env cho SubprocVecEnv."""

    def _init():
        namespace = f"env{rank}"
        env = TurtleSacEnv(
            namespace=namespace,
            node_name=f"turtle_sac_env_{rank}",
            show_goal_marker=show_goal_marker,
        )
        env = Monitor(env, filename=f"./logs/monitor_env{rank}.csv")
        env.reset(seed=1000 + rank)
        return env

    return _init


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-envs", type=int, default=4, help="Số turtlesim/env chạy song song")
    parser.add_argument("--total-timesteps", type=int, default=100_000, help="Tổng số timestep train")
    parser.add_argument("--model-out", default="sac_turtlesim_multi", help="Tên model output")
    parser.add_argument("--no-launch", action="store_true", help="Không tự launch turtlesim, dùng nếu bạn đã tự mở env0..envN")
    parser.add_argument("--show-all-goals", action="store_true", help="Hiện marker goal ở tất cả env; mặc định chỉ env0 để nhẹ hơn")
    parser.add_argument("--quiet", action="store_true", help="Ẩn log turtlesim_node")
    args = parser.parse_args()

    os.makedirs("./logs", exist_ok=True)
    os.makedirs("./runs", exist_ok=True)

    processes = []
    if not args.no_launch:
        for i in range(args.num_envs):
            ns = f"env{i}"
            print(f"[LAUNCH] turtlesim namespace /{ns}")
            processes.append(launch_turtlesim(ns, quiet=args.quiet))
            time.sleep(0.5)

        # Cho turtlesim đủ thời gian tạo topic/service.
        time.sleep(2.0)

    env = None
    try:
        env_fns = []
        for i in range(args.num_envs):
            show_marker = args.show_all_goals or i == 0
            env_fns.append(make_env(i, show_goal_marker=show_marker))

        env = SubprocVecEnv(env_fns, start_method="spawn")
        env = VecMonitor(env)

        model = SAC(
            policy="MlpPolicy",
            env=env,
            verbose=1,
            learning_rate=3e-4,
            buffer_size=200_000,
            learning_starts=max(1_000, 500 * args.num_envs),
            batch_size=256,
            tau=0.005,
            gamma=0.98,
            train_freq=1,
            gradient_steps=1,
            ent_coef="auto",
            tensorboard_log="./runs/sac_turtlesim_multi",
        )

        model.learn(total_timesteps=args.total_timesteps)
        model.save(args.model_out)
        print(f"Đã lưu model: {args.model_out}.zip")

    except KeyboardInterrupt:
        print("\nDừng train bằng Ctrl+C")
        if 'model' in locals():
            model.save(args.model_out + "_interrupt")
            print(f"Đã lưu model tạm: {args.model_out}_interrupt.zip")

    finally:
        if env is not None:
            env.close()
        if processes:
            stop_processes(processes)


if __name__ == "__main__":
    main()
