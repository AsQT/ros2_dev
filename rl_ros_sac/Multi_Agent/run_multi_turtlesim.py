import argparse
import os
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-envs", type=int, default=4)
    args = parser.parse_args()

    processes = []
    try:
        for i in range(args.num_envs):
            ns = f"env{i}"
            cmd = [
                "ros2",
                "run",
                "turtlesim",
                "turtlesim_node",
                "--ros-args",
                "-r",
                f"__ns:=/{ns}",
                "-r",
                f"__node:=turtlesim_{ns}",
            ]
            print("Launching:", " ".join(cmd))
            processes.append(subprocess.Popen(cmd, preexec_fn=os.setsid))
            time.sleep(0.5)

        print("\nĐã mở các turtlesim namespace /env0, /env1, ...")
        print("Nhấn Ctrl+C để đóng tất cả.")
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nClosing turtlesim nodes...")

    finally:
        for p in processes:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    p.terminate()


if __name__ == "__main__":
    main()
