## Terminal_1 run turtlesim_node
```bash
ros2 run turtlesim turtlesim_node

```
## Terminal_2 Train 
```bash
source ~/venvs/rl_ros/bin/activate

python3 train_sac.py
```
/______________________________________________/
## Test modle 
model = SAC.load("sac_turtlesim_reach_interrupt") #sac_turtlesim_reach
```bash
python3 test_sac.py

```modle