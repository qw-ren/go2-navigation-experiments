import numpy as np
from PIL import Image
import yaml
import os
from pathlib import Path

# --- 配置参数 (同步自 env/sim_env.py 中的 obstacle-dense 配置) ---
TERRAIN_CFG = {
    "terrain_type": "generator",
    "terrain_generator": {
        "name": "HfUniformDiscreteObstaclesTerrain",
        "size": (50.0, 50.0), # ==== 修了这里！地形总尺寸变为 50x50 ====
        "platform_width": 1.0,
        "obstacle_width_range": (0.5, 1.0), # 障碍物宽度范围
        "obstacle_height_range": (1.0, 2.0),
        "num_obstacles": 400, # ==== 修了这里！障碍物数量增加到 400 ====
        "obstacles_distance": 2.0, # 障碍物之间的最小距离
        "seed": 0, # 随机种子
    }
}

# --- 地图生成参数 ---
MAP_RESOLUTION = 0.1  # 米/像素，需与 env/terrain.py 里的 horizontal_scale 保持一致
MAP_OUTPUT_DIR = str(Path(__file__).resolve().parents[1] / "maps" / "simulation")
MAP_NAME = "sim_map"

def generate_map_from_config():
    """根据仿真地形配置生成2D栅格地图."""
    gen_cfg = TERRAIN_CFG["terrain_generator"]
    size_x, size_y = gen_cfg["size"]
    num_obstacles = gen_cfg["num_obstacles"]
    min_dist = gen_cfg["obstacles_distance"]
    border_width = 5.0
    platform_width = gen_cfg["platform_width"]
    min_width, max_width = gen_cfg["obstacle_width_range"]
    seed = gen_cfg["seed"]

    np.random.seed(seed)

    map_width_px = int(size_x / MAP_RESOLUTION)
    map_height_px = int(size_y / MAP_RESOLUTION)
    border_px = int(border_width / MAP_RESOLUTION)
    sub_width_px = map_width_px - 2 * border_px
    sub_height_px = map_height_px - 2 * border_px
    platform_width_px = int(platform_width / MAP_RESOLUTION)

    grid_map = np.full((map_height_px, map_width_px), 254, dtype=np.uint8)
    sub_grid = np.full((sub_height_px, sub_width_px), 254, dtype=np.uint8)

    obstacle_positions = list(gen_cfg.get("avoid_positions", [[0, 0]]))
    obs_x_range = np.arange(0, sub_width_px, 4)
    obs_y_range = np.arange(0, sub_height_px, 4)

    for _ in range(num_obstacles):
        # 与 env/terrain.py 保持一致：先采样高度，再采样宽/长，再采样位置。
        np.random.uniform(gen_cfg["obstacle_height_range"][0], gen_cfg["obstacle_height_range"][1])
        width = int(np.random.uniform(min_width, max_width) / MAP_RESOLUTION)
        length = int(np.random.uniform(min_width, max_width) / MAP_RESOLUTION)

        good_position = False
        sample_start = None
        for _ in range(100):
            x_start = int(np.random.choice(obs_x_range))
            y_start = int(np.random.choice(obs_y_range))
            x_scale = x_start * MAP_RESOLUTION
            y_scale = y_start * MAP_RESOLUTION

            good_position = True
            if min_dist > 0:
                for other_pos in obstacle_positions:
                    dist = np.sqrt((x_scale - other_pos[0])**2 + (y_scale - other_pos[1])**2)
                    if dist < min_dist:
                        good_position = False
                        break

            if good_position:
                sample_start = (x_start, y_start)
                obstacle_positions.append([x_scale, y_scale])
                break

        if not good_position or sample_start is None:
            break

        x_start, y_start = sample_start
        if x_start + width > sub_width_px:
            x_start = sub_width_px - width
        if y_start + length > sub_height_px:
            y_start = sub_height_px - length

        sub_grid[sub_height_px - (y_start + length):sub_height_px - y_start, x_start:x_start + width] = 0

    # 与 terrain.py 里的中心平台清空逻辑对齐
    x1 = (sub_width_px - platform_width_px) // 2
    x2 = (sub_width_px + platform_width_px) // 2
    y1 = (sub_height_px - platform_width_px) // 2
    y2 = (sub_height_px + platform_width_px) // 2
    sub_grid[sub_height_px - y2:sub_height_px - y1, x1:x2] = 254

    # 把带边框的子地形嵌回完整地图，和 IsaacLab 的 border_width 语义保持一致
    grid_map[border_px:border_px + sub_height_px, border_px:border_px + sub_width_px] = sub_grid

    os.makedirs(MAP_OUTPUT_DIR, exist_ok=True)

    pgm_path = os.path.join(MAP_OUTPUT_DIR, f"{MAP_NAME}.pgm")
    img = Image.fromarray(grid_map)
    img.save(pgm_path)

    yaml_data = {
        "image": f"{MAP_NAME}.pgm",
        "resolution": MAP_RESOLUTION,
        "origin": [-size_x / 2, -size_y / 2, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path = os.path.join(MAP_OUTPUT_DIR, f"{MAP_NAME}.yaml")
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, sort_keys=False)
    print(f"地图已经更新: 尺寸 {size_x}x{size_y}, 分辨率 {MAP_RESOLUTION}")

if __name__ == "__main__":
    generate_map_from_config()
