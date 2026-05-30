import time

from stable_baselines3 import SAC # pyright: ignore[reportMissingImports]
from turtle_sac_env import TurtleSacEnv


def main():
    env = TurtleSacEnv()
    model = SAC.load("sac_turtlesim_reach_interrupt") #sac_turtlesim_reach

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
