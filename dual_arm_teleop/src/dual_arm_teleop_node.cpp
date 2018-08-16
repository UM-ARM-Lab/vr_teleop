#include <ros/ros.h>

#include <queue>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

// Vive
#include <vive_msgs/ViveSystem.h>

// Victor
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>

// TF
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

struct SeedDistanceFunctor
{
  using Solution = std::vector<double>;
  const Solution seed;
  SeedDistanceFunctor(Solution _seed) : seed(std::move(_seed)) {}
  static double distance(const Solution& a, const Solution& b)
  {
    assert(a.size() == b.size());
    double d = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
    {
      //d += fabs(a[i]-b[i]);
      d += pow(a[i]-b[i], 2);
    }
    return d;
  }

  // NB: priority_queue is a max-heap structure, so less() should actually return >
  // "highest priority"
  bool operator()(const Solution& a, const Solution& b) const
  {
    return distance(seed, a) > distance(seed, b);
  }
};

struct victor_arm
{
  victor_arm() : activated(false), trackpad_pressed(false), enabled(false), initialized(false) {}

  bool activated; // true if arm is currently being teleoperated
  bool trackpad_pressed;
  bool enabled; // true if arm has been paired with a controller
  bool initialized; // true if *_start_[translation/rotation] has been initialized

  int assigned_controller_index;
  int assigned_controller_id;

  // Kinematics
  std::string joint_model_group_name;
  robot_state::JointModelGroup* joint_model_group;

  Eigen::Affine3d ee_last_valid_pose;
  Eigen::Affine3d controller_reset_pose;
  Eigen::Affine3d ee_reset_pose;

  std::vector<double> joint_position_measured;

  // Topic publishers/subscribers
  ros::Publisher pub_arm;
  ros::Publisher pub_gripper;
  ros::Subscriber sub_arm_status;

  visualization_msgs::Marker err_msg;
};

class DualArmTeleop
{
public:
  DualArmTeleop()
  {
    sub = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);

    // Initialize victor kinematic model
    robot_model_loader::RobotModelLoader robot_model_load("robot_description");

    kinematic_model = robot_model_load.getModel();
    kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

    // Enforce joint limits
    kinematic_state->setToDefaultValues();
    kinematic_state->enforceBounds();

    // Initialize victor arms
    victor_arms[0].joint_model_group_name = "left_arm";
    victor_arms[1].joint_model_group_name = "right_arm";

    pub_rviz = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

    for (int arm = 0; arm < 2; ++arm) {
      victor_arms[arm].joint_model_group = kinematic_model->getJointModelGroup(victor_arms[arm].joint_model_group_name);

      victor_arms[arm].ee_last_valid_pose = kinematic_state->getGlobalLinkTransform("victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7");
      victor_arms[arm].ee_reset_pose = kinematic_state->getGlobalLinkTransform("victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7");

      victor_arms[arm].joint_position_measured.resize(7);

      victor_arms[arm].pub_arm = n.advertise<victor_hardware_interface::MotionCommand>(victor_arms[arm].joint_model_group->getName() + "/motion_command", 10);
      victor_arms[arm].pub_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>(victor_arms[arm].joint_model_group->getName() + "/gripper_command", 10);

      if (arm == 0)
      {
        victor_arms[arm].sub_arm_status = n.subscribe(victor_arms[arm].joint_model_group->getName() + "/motion_status", 10, &DualArmTeleop::updateArmStatusLeft, this);
      }
      else
      {
        victor_arms[arm].sub_arm_status = n.subscribe(victor_arms[arm].joint_model_group->getName() + "/motion_status", 10, &DualArmTeleop::updateArmStatusRight, this);
      }
    }
  }

  void callback(vive_msgs::ViveSystem msg)
  {
    for (int arm = 0; arm < 2; ++arm)
    {
      victor_arms[arm].enabled = false;
    }

    // Gather information about controllers
    std::vector<int> unassigned_controller_indices;
    for (int controller = 0; controller < msg.controllers.size(); ++controller)
    {
      int controller_id = msg.controllers[controller].id;

      bool assigned = false;
      for (int arm = 0; arm < 2; ++arm)
      {
        if (victor_arms[arm].assigned_controller_id == controller_id)
        {
          victor_arms[arm].enabled = true;
          assigned = true;
          victor_arms[arm].assigned_controller_index = controller;
        }
      }
      if (!assigned) unassigned_controller_indices.push_back(controller);
    }

    for (int arm = 0; arm < 2; ++arm)
    {
      if (!victor_arms[arm].enabled)
      {
        victor_arms[arm].assigned_controller_index = unassigned_controller_indices.back();
        victor_arms[arm].assigned_controller_id = msg.controllers[victor_arms[arm].assigned_controller_index].id;
        unassigned_controller_indices.pop_back();
        victor_arms[arm].enabled = true;
      }
    }

    for (int arm = 0; arm < 2; ++arm)
    {
      Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();

      // Skip this arm if no controller is assigned to it
      if (!victor_arms[arm].enabled)
      {
        victor_arms[arm].initialized = false;
        continue;
      }

      vive_msgs::Controller msg_controller = msg.controllers[victor_arms[arm].assigned_controller_index];

      // Update this arm's activation status
      if (msg_controller.joystick.buttons[2] == 2) {
        if (!victor_arms[arm].trackpad_pressed) {
          victor_arms[arm].trackpad_pressed = true;
          victor_arms[arm].activated = !victor_arms[arm].activated;
        }
      } else {
        victor_arms[arm].trackpad_pressed = false;
      }
      if (!victor_arms[arm].activated) {
        continue;
      }



      Eigen::Affine3d controller_pose = poseMsgToEigen(msg_controller.posestamped.pose);

      // Store reset pose
      if (msg_controller.joystick.buttons[1] == 2 || !victor_arms[arm].initialized)
      {
        victor_arms[arm].controller_reset_pose = controller_pose;
        victor_arms[arm].ee_reset_pose = kinematic_state->getGlobalLinkTransform("victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7");

        victor_arms[arm].initialized = true;
      }



      // Pose representing controller delta between last and current
      Eigen::Affine3d controller_delta_pose = controller_pose.inverse() * victor_arms[arm].controller_reset_pose;

      // Pose representing desired end effector pose
      Eigen::Affine3d ee_target_pose = victor_arms[arm].ee_reset_pose * controller_delta_pose;



      // Generate IK solutions
      const kinematics::KinematicsBaseConstPtr& solver = victor_arms[arm].joint_model_group->getSolverInstance();
      assert(solver.get());

      Eigen::Affine3d solverTrobot = Eigen::Affine3d::Identity();
      kinematic_state->setToIKSolverFrame(solverTrobot, solver);

      // Convert to solver frame
      Eigen::Affine3d pt_solver = solverTrobot * ee_target_pose;

      std::vector<geometry_msgs::Pose> target_poses;
      geometry_msgs::Pose pose;
      Eigen::Quaterniond q(pt_solver.linear());
      pose.position.x = pt_solver.translation().x();
      pose.position.y = pt_solver.translation().y();
      pose.position.z = pt_solver.translation().z();
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      target_poses.push_back(pose);

      std::vector<double> seed = victor_arms[arm].joint_position_measured;
      std::vector<std::vector<double>> solutions;
      kinematics::KinematicsResult result;
      kinematics::KinematicsQueryOptions options;
      options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;

      solver->getPositionIK(target_poses, seed, solutions, result, options);

      // Pick the solution that matches closest to the measured joint state
      if (!solutions.empty()) {
        SeedDistanceFunctor functor(seed);
        std::priority_queue<std::vector<double>, std::vector<std::vector<double>>, SeedDistanceFunctor> slnQueue(solutions.begin(), solutions.end(), functor);
        kinematic_state->setJointGroupPositions(victor_arms[arm].joint_model_group, slnQueue.top());

        victor_arms[arm].ee_last_valid_pose = ee_target_pose;
      }

      std::cerr << "Got " << solutions.size() << " solutions for " << victor_arms[arm].joint_model_group->getName() << std::endl;

      // Arm control
      victor_hardware_interface::MotionCommand msg_out_arm;
      msg_out_arm.control_mode.mode = 2;

      std::vector<double> joint_values;
      kinematic_state->copyJointGroupPositions(victor_arms[arm].joint_model_group, joint_values);

      msg_out_arm.joint_position.joint_1 = joint_values[0];
      msg_out_arm.joint_position.joint_2 = joint_values[1];
      msg_out_arm.joint_position.joint_3 = joint_values[2];
      msg_out_arm.joint_position.joint_4 = joint_values[3];
      msg_out_arm.joint_position.joint_5 = joint_values[4];
      msg_out_arm.joint_position.joint_6 = joint_values[5];
      msg_out_arm.joint_position.joint_7 = joint_values[6];

      // Gripper control
      victor_hardware_interface::Robotiq3FingerCommand msg_out_gripper;

      victor_hardware_interface::Robotiq3FingerActuatorCommand scissor;
      scissor.speed = 1.0;
      scissor.force = 1.0;
      scissor.position = 1;

      victor_hardware_interface::Robotiq3FingerActuatorCommand finger_a;
      finger_a.speed = 1.0;
      finger_a.force = 1.0;
      finger_a.position = msg_controller.joystick.axes[2];

      victor_hardware_interface::Robotiq3FingerActuatorCommand finger_b;
      finger_b.speed = 1.0;
      finger_b.force = 1.0;
      finger_b.position = msg_controller.joystick.axes[2];

      victor_hardware_interface::Robotiq3FingerActuatorCommand finger_c;
      finger_c.speed = 1.0;
      finger_c.force = 1.0;
      finger_c.position = msg_controller.joystick.axes[2];

      msg_out_gripper.scissor_command = scissor;
      msg_out_gripper.finger_a_command = finger_a;
      msg_out_gripper.finger_b_command = finger_b;
      msg_out_gripper.finger_c_command = finger_c;

      // Publish state messages
      if (armWithinDelta(jvqToVector(msg_out_arm.joint_position), arm)) {
        victor_arms[arm].pub_arm.publish(msg_out_arm);
      }

      victor_arms[arm].pub_gripper.publish(msg_out_gripper);

      // Display rviz poses
      tf::Transform tf_controller_global;
      tf::poseEigenToTF(controller_pose, tf_controller_global);
      tf_broadcaster.sendTransform(tf::StampedTransform(tf_controller_global, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/controller_global"));

      tf::Transform tf_controller_reset;
      tf::poseEigenToTF(victor_arms[arm].controller_reset_pose, tf_controller_reset);
      tf_broadcaster.sendTransform(tf::StampedTransform(tf_controller_reset, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/controller_reset"));

      tf::Transform tf_ee_last_valid;
      tf::poseEigenToTF(victor_arms[arm].ee_last_valid_pose, tf_ee_last_valid);
      tf_broadcaster.sendTransform(tf::StampedTransform(tf_ee_last_valid, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/ee_last_valid"));

      tf::Transform tf_ee_target;
      tf::poseEigenToTF(ee_target_pose, tf_ee_target);
      tf_broadcaster.sendTransform(tf::StampedTransform(tf_ee_target, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/ee_target_pose"));
    }

    moveit_msgs::DisplayRobotState display_robot_state;
    robot_state::robotStateToRobotStateMsg(*kinematic_state, display_robot_state.state);
    pub_rviz.publish(display_robot_state);
  }

  void updateArmStatus(victor_hardware_interface::MotionStatus msg, int arm) {
    victor_arms[arm].joint_position_measured = jvqToVector(msg.measured_joint_position);
  }

  void updateArmStatusLeft(victor_hardware_interface::MotionStatus msg)
  {
    updateArmStatus(msg, 0);
  }

  void updateArmStatusRight(victor_hardware_interface::MotionStatus msg)
  {
    updateArmStatus(msg, 1);
  }

  bool armWithinDelta(std::vector<double> joint_position_commanded, int arm)
  {
    assert(joint_position_commanded.size() == victor_arms[arm].joint_position_measured.size());

    double distance = 0;

    for (int i = 0; i < joint_position_commanded.size(); ++i) {
      distance += pow(joint_position_commanded[i] - victor_arms[arm].joint_position_measured[i], 2);
    }

    distance = sqrt(distance);

    std::cout << "Joint space error for " << victor_arms[arm].joint_model_group->getName() << ": " << distance << std::endl;

    return distance < .7;
  }

  Eigen::Affine3d translationAndRotationToAffine(Eigen::Vector3d translation, Eigen::Quaterniond rotation)
  {
    Eigen::Affine3d out = Eigen::Affine3d::Identity();
    out.translate(translation);
    out.rotate(rotation);
    return out;
  }

  Eigen::Affine3d poseViveToVictor(Eigen::Affine3d pose) {
    Eigen::Matrix3d mat = pose.linear();
    Eigen::Quaterniond q(mat);
    return translationAndRotationToAffine(viveToVictorTranslation(pose.translation()), viveToVictorRotation(q));
  }

  Eigen::Vector3d viveToVictorTranslation(Eigen::Vector3d vive)
  {
    return Eigen::Vector3d(
      -vive[2],
      -vive[0],
      vive[1]
    );
  }

  Eigen::Quaterniond viveToVictorRotation(Eigen::Quaterniond vive)
  {
    return Eigen::Quaterniond(
      vive.x(),
      vive.w(),
      -vive.y(),
      -vive.z()
    );
  }


  Eigen::Vector3d pointMsgToEigen(geometry_msgs::Point point)
  {
    return Eigen::Vector3d(
      point.x,
      point.y,
      point.z
    );
  }

  Eigen::Quaterniond quatMsgToEigen(geometry_msgs::Quaternion quaternion)
  {
    return Eigen::Quaterniond(
      quaternion.x,
      quaternion.y,
      quaternion.z,
      quaternion.w
    );
  }

  Eigen::Affine3d poseMsgToEigen(geometry_msgs::Pose pose)
  {
    return translationAndRotationToAffine(pointMsgToEigen(pose.position), quatMsgToEigen(pose.orientation));
  }

  std::vector<double> jvqToVector(victor_hardware_interface::JointValueQuantity jvq)
  {
    std::vector<double> v{jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4,
                          jvq.joint_5, jvq.joint_6, jvq.joint_7};
    return v;
  };

private:
  ros::NodeHandle n;
  ros::Subscriber sub;
  tf::TransformBroadcaster tf_broadcaster;
  ros::Publisher pub_rviz;
  //ros::Publisher pub_err_msg;
  //rviz_visual_tools::RvizVisualTools rvt;

  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;

  victor_arm victor_arms[2];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");

  DualArmTeleop dual_arm_teleop;

  ros::spin();

  ros::shutdown();
  return 0;
}