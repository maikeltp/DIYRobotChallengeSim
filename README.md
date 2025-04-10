# DIY Robot Challenge Simulator
This repository contains the code for a simulator that can be used on the DIY robot challenge.

## Environment Setup
See [docker/README.md](docker/README.md) for instructions on setting up the docker development environment.

### Pre-commit
To keep the code formatting consistent, it is recommended to install pre-commit.

1. For C++:
   - Install clang-format: `sudo apt-get install clang-format` (Ubuntu) or `brew install clang-format` (macOS)

2. For Python:
   - Install the required tools:
     ```bash
     pip install black isort pylint pycodestyle
     ```

3. Git pre-commit hooks:
   ```bash
   pip install pre-commit
   ```

4. Initialize pre-commit:
   ```bash
   pre-commit install
   ```

This setup will:
- Format C++ code according to Google style with some customizations
- Format Python code using Black with consistent line lengths
- Sort imports using isort
- Run additional checks with pylint
- Enforce these rules automatically on commit

## Running the Simulation
See [src/sim/README.md](src/sim/README.md) for instructions on running the simulation.


## Quick start
Start docker container
```bash
cd <repo-root>/docker
./start.sh
```
Next steps are inside the docker container

Build project
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install.bash
```

Launch simulator
```bash
ros2 launch sim load_sim.launch.py
```

Launch app nodes, for example:
```bash
ros2 launch example example.launch.py
```

![start_signal-ezgif com-optimize](https://github.com/user-attachments/assets/8332d560-ec9e-4ddb-97f8-0c8ba03b8dc3)

![dynamic_obstacle-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/e1f520d6-ebd3-4276-a2c2-996ce3666fa8)

![car_moving-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/d19f96a1-2d77-4192-bdec-13031874625d)
