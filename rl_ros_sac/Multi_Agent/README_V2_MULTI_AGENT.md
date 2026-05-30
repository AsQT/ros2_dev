# rl_ros_sac v2 - train nhiều turtlesim cùng lúc

Bản v2 này dùng nhiều `turtlesim_node` độc lập theo namespace:

- `/env0/turtle1/...`
- `/env1/turtle1/...`
- `/env2/turtle1/...`
- `/env3/turtle1/...`

Một policy SAC duy nhất học từ nhiều môi trường song song. Đây là kiểu **parallel environment / vectorized training**, không phải multi-agent game đối kháng. Với bài học SAC ban đầu, cách này phù hợp nhất vì sample nhanh hơn và dữ liệu đa dạng hơn.

## Train 4 env cùng lúc

```bash

source ~/venvs/rl_ros/bin/activate
python3 train_multi_sac.py --num-envs 4 --total-timesteps 100000 --quiet
```

Mặc định script tự mở 4 cửa sổ turtlesim. Marker `goal` chỉ hiện ở `/env0` để nhẹ hơn. Nếu muốn hiện goal ở tất cả env:

```bash
python3 train_multi_sac.py --num-envs 4 --total-timesteps 10000 --show-all-goals --quiet
```

Model sau train:

```text
sac_turtlesim_multi.zip
```

## Test model trên nhiều env

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/rl_ros/bin/activate

cd ~/rl_ros_sac
python3 test_multi_sac.py --model sac_turtlesim_multi --num-envs 4 --quiet
```

## Chỉ mở nhiều turtlesim để kiểm tra namespace

```bash
python3 run_multi_turtlesim.py --num-envs 4
```
