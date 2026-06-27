mình bổ sung thểm những vấn đề liên quan đển model rl đã được train , huật toán hiện tại là SAC

workspace:
  x_min: 0.25
  x_max: 0.5
  y_min: -0.15
  y_max: 0.15
  z_min: 0.02
  z_max: 0.3


# Start position 

  mode: random  
  fixed_position: [0.30, 0.00, 0.30]
  random_bounds:
    min: [0.25, -0.15, 0.1]
    max: [0.5, 0.15, 0.30]

# Target region (tọa độ pick)

target:
  mode: random          
  fixed_position: [0.30, 0.00, 0.10]
  random_bounds:
    min: [0.25, -0.15, 0.02]
    max: [0.5, 0.15, 0.15]
# Obstacle / Box —

obstacle:
  mode: random       
  type: box
  safety_margin: 0.03 #` minimum distance to maintain from obstacle for success

  random_bounds:
    min: [0.28, -0.10, 0.080]  
    max: [0.42, 0.10, 0.080]  

  size_random:  

    length_min: 0.05  
    length_max: 0.1  
    width_min: 0.05
    width_max: 0.1
    height_min: 0.05
    height_max: 0.15

tất cả là đơn vị met

- Observation là Box(-inf, inf, shape=(15,), dtype=float32).
- Thứ tự ghép:
  observation = [
      current_pos,
      target_pos,
      err = target_pos - current_pos,
      rel_obs = (obstacle_center - current_pos) / workspace_range,
      obs_size = obstacle_half_extent / workspace_range
  ]

Không gian hành động:
- Action là Box(-1.0, 1.0, shape=(3,), dtype=float32).
- Ý nghĩa: delta Cartesian chuẩn hóa theo các trục x, y, z.
- Môi trường scale vật lý bằng:
  delta = action * environment.action_step
  next_pos = current_pos + delta
- Với config hiện tại: action_step = 0.01 m.