import pinocchio
import numpy as np
import argparse
import sys
import time

###############################################################################
### Robot class
###############################################################################
class Robot:
    def __init__(self, urdf_path, end_effector_name):
        self.model = pinocchio.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        if not self.model.existFrame(end_effector_name):
            raise ValueError(f"Frame '{end_effector_name}' not found in the model")
        self.end_effector_id = self.model.getFrameId(end_effector_name)
        self.joint_positions = pinocchio.neutral(self.model)
        self.limits = self.get_joint_limits()

    def __str__(self):
        return f"Robot with {self.model.nq} degrees of freedom. Available joints: {[self.model.names[i] for i in range(1, self.model.njoints)]}"

    def forward_kinematics(self, joint_positions=None):
        start_time = time.perf_counter()
        if joint_positions is None:
            #print(f"Forward kinematics: Using neutral joint configuration")
            self.joint_positions = pinocchio.neutral(self.model)
        else:
            #print(f"Forward kinematics: Using joint positions: {joint_positions}")
            if len(joint_positions) != self.model.nq:
                raise ValueError(f"Joint positions must have {self.model.nq} elements")
            self.joint_positions = np.array(joint_positions)
        pinocchio.forwardKinematics(self.model, self.data, self.joint_positions)
        pinocchio.updateFramePlacements(self.model, self.data)
        elapsed_time = time.perf_counter() - start_time
        print(f"Forward kinematics: elapsed time = {elapsed_time*1000:.4f} ms")

    def get_end_effector_transform(self):
        return self.data.oMf[self.end_effector_id]

    def get_end_effector_jacobian(self, frame_type, joint_positions=None):
        if joint_positions is None:
            joint_positions = self.joint_positions
        J = pinocchio.computeFrameJacobian(self.model, self.data, joint_positions, self.end_effector_id, frame_type)
        # Ensure Jacobian is 2D for proper indexing
        # Reshape to (6, nq) when nq, the number of joints, is available
        return J.reshape(6, self.model.nq)

    def inverse_kinematics(self, target_pose, max_iter=500, tol=1e-4, dt=1.0, damp=0.01):
        q = self.joint_positions.copy()
        for iter in range(max_iter):
            # Forward kinematics
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.updateFramePlacements(self.model, self.data)

            # Current end-effector pose
            current_pose = self.get_end_effector_transform()

            # Error (difference between target and current pose)
            error_se3 = current_pose.actInv(target_pose)
            error_vec = pinocchio.log(error_se3).vector
            error_norm = np.linalg.norm(error_vec)

            # Convergence check
            if error_norm < tol:
                return q, True, error_norm, iter

            # Damped least squares
            J = self.get_end_effector_jacobian(pinocchio.ReferenceFrame.LOCAL, q)
            j_pseudo_inverse = J.T @ np.linalg.inv(J @ J.T + damp * np.eye(6))

            # Update configuration
            dq = j_pseudo_inverse @ error_vec
            q = pinocchio.integrate(self.model, q, dq * dt)

        # Not converged
        return q, False, error_norm, iter

    def get_joint_limits(self):
        print("\nJoint limits:")
        joint_limits = []
        q_idx = 0  # Index for configuration variables
        for i in range(1, self.model.njoints):  # Skip universe joint (index 0)
            joint = self.model.joints[i]
            if joint.nq > 0:
                lower = self.model.lowerPositionLimit[q_idx]
                upper = self.model.upperPositionLimit[q_idx]
                print(f"  Joint '{self.model.names[i]}' limits: {lower}, {upper}")
                joint_limits.append((lower, upper))
                q_idx += joint.nq  # Move to next configuration variable
        return joint_limits

    def compute_three_poses(self):
        joint_configurations = []
        end_effector_poses=[]

        # Configuration 0: center (neutral)
        q = []
        for min_val, max_val in self.limits:
            q.append((max_val + min_val) / 2.0)
        self.forward_kinematics(q)
        end_effector_poses.append(self.get_end_effector_transform().copy())
        joint_configurations.append(q)

        # Configuration 1: 1/3 from min
        q = []
        for min_val, max_val in self.limits:
            q.append(min_val + (max_val - min_val) * 1.0 / 3.0)
        self.forward_kinematics(q)
        end_effector_poses.append(self.get_end_effector_transform().copy())
        joint_configurations.append(q)

        # Configuration 2: 2/3 from min
        q = []
        for min_val, max_val in self.limits:
            q.append(min_val + (max_val - min_val) * 2.0 / 3.0)
        self.forward_kinematics(q)
        end_effector_poses.append(self.get_end_effector_transform().copy())
        joint_configurations.append(q)

        return joint_configurations, end_effector_poses

def test_inverse_kinematics(robot, desired_end_effector_pose, expected_joint_configuration):
    print(f"\nTesting inverse kinematics with desired end-effector pose:\n{desired_end_effector_pose}")
    q, converged, error, iterations = robot.inverse_kinematics(desired_end_effector_pose)
    if converged:
        robot.forward_kinematics(q)
        print(f"Inverse kinematics: converged in {iterations} iterations")
        print(f"Inverse kinematics: error: {error}")
        print(f"New joint positions: {q}")
        print(f"Expected joint positions: {expected_joint_configuration}")
        print(f"Current end-effector pose:\n{robot.get_end_effector_transform()}")
    else:
        print(f"Inverse kinematics: not converged in {iterations} iterations")
        print(f"Inverse kinematics: error: {error}")

###############################################################################
### Main function
###############################################################################
def main(urdf_file, end_effector_name, joint_positions=None):
    print(f"\nLoading URDF model: {urdf_file} with desired end-effector: '{end_effector_name}'")

    robot = Robot(urdf_file, end_effector_name)
    print(f"\nRobot loaded with success: {robot}")

    print(f"\nSet forward kinematics with joint positions: {joint_positions}")
    robot.forward_kinematics(joint_positions)
    world_transform = robot.get_end_effector_transform()
    print(f"World transform (4x4 matrix):")
    print(world_transform.homogeneous)

    J_local = robot.get_end_effector_jacobian(pinocchio.ReferenceFrame.LOCAL)
    print(f"\nJacobian in local frame, shape: {J_local.shape} of the end-effector:")
    print(f"Linear part:\n{J_local[:3, :]}")
    print(f"Angular part:\n{J_local[3:, :]}")

    print("\nComputing 3 poses to reach with IK:")
    joint_configurations, end_effector_poses = robot.compute_three_poses()
    test_inverse_kinematics(robot, end_effector_poses[1], joint_configurations[1])
    test_inverse_kinematics(robot, end_effector_poses[2], joint_configurations[2])
    test_inverse_kinematics(robot, end_effector_poses[0], joint_configurations[0])

    print(joint_configurations[1])
    print(joint_configurations[2])
    print(joint_configurations[0])

###############################################################################
### CLI
###############################################################################
def parse_arguments():
    """Parse command line arguments for the robot CLI."""
    parser = argparse.ArgumentParser(
        description="Robot URDF analysis tool with forward kinematics and jacobian computation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python test.py robot.urdf end_effector
  python test.py robot.urdf end_effector --joint-positions 0.1 0.2 0.3
  python test.py robot.urdf end_effector -j 0 0.5 -0.3
        """
    )

    parser.add_argument(
        'urdf_file',
        help='Path to the URDF file'
    )

    parser.add_argument(
        'end_effector_name',
        help='Name of the end effector frame'
    )

    parser.add_argument(
        '-j', '--joint-positions',
        nargs='*',
        type=float,
        help='Joint positions (optional). If not provided, neutral configuration is used.'
    )

    return parser.parse_args()

###############################################################################
### Entry point
###############################################################################
if __name__ == "__main__":
    try:
        args = parse_arguments()

        # Convert joint positions to numpy array if provided
        joint_positions = None
        if args.joint_positions:
            joint_positions = np.array(args.joint_positions)
            print(f"CLI: Using joint positions: {joint_positions}")
        else:
            print("CLI: Using neutral joint configuration")

        main(args.urdf_file, args.end_effector_name, joint_positions)

    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
