import pinocchio
import numpy as np
import argparse
import sys

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

    def __str__(self):
        return f"Robot with {self.model.nq} degrees of freedom. Available joints: {[self.model.names[i] for i in range(1, self.model.njoints)]}"

    def forward_kinematics(self, joint_positions=None):
        if joint_positions is None:
            self.joint_positions = pinocchio.neutral(self.model)
        else:
            if len(joint_positions) != self.model.nq:
                raise ValueError(f"Joint positions must have {self.model.nq} elements")
            self.joint_positions = joint_positions
        pinocchio.forwardKinematics(self.model, self.data, self.joint_positions)
        pinocchio.updateFramePlacements(self.model, self.data)

    def get_end_effector_transform(self):
        return self.data.oMf[self.end_effector_id]

    def get_end_effector_jacobian(self, frame_type):
        J = pinocchio.computeFrameJacobian(self.model, self.data, self.joint_positions, self.end_effector_id, frame_type)
        # Ensure Jacobian is 2D for proper indexing
        # Reshape to (6, nq) when nq, the number of joints, is available
        return J.reshape(6, self.model.nq)

###############################################################################
### Main function
###############################################################################
def main(urdf_file, end_effector_name, joint_positions=None):
    print(f"Loading URDF model: {urdf_file} and end-effector: {end_effector_name}")

    robot = Robot(urdf_file, end_effector_name)
    print(f"Robot: {robot}")

    robot.forward_kinematics(joint_positions)
    world_transform = robot.get_end_effector_transform()
    print(f"World transform (4x4 matrix):")
    print(world_transform.homogeneous)

    J_local = robot.get_end_effector_jacobian(pinocchio.ReferenceFrame.LOCAL)
    print(f"\nJacobian in local frame, shape: {J_local.shape}")
    print(f"Linear part:\n{J_local[:3, :]}")
    print(f"Angular part:\n{J_local[3:, :]}")


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
            print(f"Using joint positions: {joint_positions}")
        else:
            print("Using neutral joint configuration")

        main(args.urdf_file, args.end_effector_name, joint_positions)

    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
