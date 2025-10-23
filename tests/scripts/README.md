# Robot Kinematics Helper Script

Python script that uses the Pinocchio library to verify world transforms and Jacobian matrices for a given
URDF file, end effector name and optionally a list of joint positions.

## Setup (run once):

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

## Running the script:

```bash
source venv/bin/activate  # On Windows: venv\Scripts\activate
python robot_kinematics.py ../../data/scara_robot.urdf end_effector
```

Optionally, you can pass joint positions:

```bash
python robot_kinematics.py ../../data/scara_robot.urdf end_effector --joint-positions 0.1 0.2 0.3
```
