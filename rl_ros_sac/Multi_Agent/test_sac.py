import argparse
import time

from stable_baselines3 import SAC  # pyright: ignore[reportMissingImports]
from turtle_sac_env import TurtleSacEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="sac_turtlesim_reach", help="Đường dẫn model .zip")
    parser.add_argument("--namespace", default="", help="Namespace turtlesim, ví dụ env0")
    args = parser.parse_args()

    env = TurtleSacEnv(namespace=args.namespace)
    model = SAC.load(args.model)

    obs, info = env.reset()
    print("Goal:", info["goal"])

    try:
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)

            if terminated:
                print("Reached goal:", info["goal"])
                time.sleep(1.0)
                obs, info = env.reset()
                print("New goal:", info["goal"])

            if truncated:
                print("Timeout, reset.")
                time.sleep(1.0)
                obs, info = env.reset()
                print("New goal:", info["goal"])

    finally:
        env.close()


if __name__ == "__main__":
    main()
