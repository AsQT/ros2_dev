from stable_baselines3 import SAC  # pyright: ignore[reportMissingImports]
from stable_baselines3.common.monitor import Monitor  # pyright: ignore[reportMissingImports]

from turtle_sac_env import TurtleSacEnv


def main():
    env = TurtleSacEnv()
    env = Monitor(env)

    model = SAC(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        learning_rate=3e-4,
        buffer_size=100_000,
        learning_starts=1_000,
        batch_size=256,
        tau=0.005,
        gamma=0.98,
        train_freq=1,
        gradient_steps=1,
        ent_coef="auto",
        tensorboard_log="./runs/sac_turtlesim",
    )

    try:
        model.learn(total_timesteps=10_000)
        model.save("sac_turtlesim_reach")
        print("Đã lưu model: sac_turtlesim_reach.zip")

    except KeyboardInterrupt:
        print("\nDừng train bằng Ctrl+C")
        model.save("sac_turtlesim_reach_interrupt")
        print("Đã lưu model tạm: sac_turtlesim_reach_interrupt.zip")

    finally:
        env.close()


if __name__ == "__main__":
    main()
