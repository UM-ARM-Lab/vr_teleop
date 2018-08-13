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
  victor_arm() : activated(false), trackpad_pressed(false), enabled(false), initialized(false), ee_start_translation(Eigen::Vector3d::Identity()) {}

  bool activated; // true if arm is currently being teleoperated
  bool trackpad_pressed;
  bool enabled; // true if arm has been paired with a controller
  bool initialized; // true if *_start_[translation/rotation] has been initialized

  int assigned_controller_index;
  int assigned_controller_id;

  // Kinematics
  std::string joint_model_group_name;
  robot_state::JointModelGroup* joint_model_group;

  Eigen::Vector3d ee_start_translation;
  Eigen::Vector3d controller_start_translation;
  Eigen::Quaterniond controller_start_rotation;
  Eigen::Affine3d last_valid_pose;
  Eigen::Affine3d last_valid_synced_pose;

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

    //rvt = rviz_visual_tools::RvizVisualTools("victor_root", "err_msg", n);
    //pub_err_msg = n.advertise<visualization_msgs::Marker>("err_msg", 10);

    pub_rviz = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

    for (int arm = 0; arm < 2; ++arm) {
      victor_arms[arm].joint_model_group = kinematic_model->getJointModelGroup(victor_arms[arm].joint_model_group_name);

      victor_arms[arm].last_valid_pose = kinematic_state->getGlobalLinkTransform("victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7");
      victor_arms[arm].last_valid_synced_pose = kinematic_state->getGlobalLinkTransform("victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7");
      victor_arms[arm].ee_start_translation = victor_arms[arm].last_valid_pose.translation();

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

      // Visualization messages
      victor_arms[arm].err_msg = visualization_msgs::Marker();
      victor_arms[arm].err_msg.header.frame_id = "victor_root";
      victor_arms[arm].err_msg.ns = "my_namespace";
      victor_arms[arm].err_msg.id = arm;
      victor_arms[arm].err_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      victor_arms[arm].err_msg.action = visualization_msgs::Marker::ADD;
      victor_arms[arm].err_msg.lifetime = ros::Duration();
      victor_arms[arm].err_msg.pose.position.x = victor_arms[arm].ee_start_translation[0];
      victor_arms[arm].err_msg.pose.position.y = victor_arms[arm].ee_start_translation[1];
      victor_arms[arm].err_msg.pose.position.z = victor_arms[arm].ee_start_translation[2];
      victor_arms[arm].err_msg.pose.orientation.x = 0.0;
      victor_arms[arm].err_msg.pose.orientation.y = 0.0;
      victor_arms[arm].err_msg.pose.orientation.z = 0.0;
      victor_arms[arm].err_msg.pose.orientation.w = 1.0;
      victor_arms[arm].err_msg.scale.x = 1;
      victor_arms[arm].err_msg.scale.y = 0.1;
      victor_arms[arm].err_msg.scale.z = 0.1;
      victor_arms[arm].err_msg.color.a = 1.0;
      victor_arms[arm].err_msg.color.r = 0.0;
      victor_arms[arm].err_msg.color.g = 1.0;
      victor_arms[arm].err_msg.color.b = 0.0;
      victor_arms[arm].err_msg.text = "This msg is being published for arm " + std::to_string(victor_arms[arm].err_msg.id);
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


      // Reset frame when button is pressed
      if (msg_controller.joystick.buttons[0] == 2 || !victor_arms[arm].initialized)
      {
        //victor_arms[arm].ee_start_pose = victor_arms[arm].last_valid_pose;
        /*Eigen::Transform<double, 3, 2, 0>::LinearMatrixType rot = kinematic_state->getGlobalLinkTransform(
          "victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7").linear();
        victor_arms[arm].ee_start_rotation = viveToVictorRotation(Eigen::Quaterniond(rot[0], rot[1], rot[2], rot[3]));*/
        /*victor_arms[arm].ee_start_pose = kinematic_state->getGlobalLinkTransform(
          "victor_" + victor_arms[arm].joint_model_group->getName() + "_link_7");*/

        // Controller frame
        victor_arms[arm].controller_start_translation = pointMsgToEigen(msg_controller.posestamped.pose.position);
        //victor_arms[arm].controller_start_rotation = quatMsgToEigen(msg_controller.posestamped.pose.orientation);
        //victor_arms[arm].controller_start_rotation = Eigen::Quaterniond::Identity();

        victor_arms[arm].initialized = true;
      }

      // Reset orientation
      if (msg_controller.joystick.buttons[1] == 2) {
        victor_arms[arm].controller_start_rotation = quatMsgToEigen(msg_controller.posestamped.pose.orientation);
      }

      // Calculate the relative pose
      Eigen::Affine3d relative_pose = Eigen::Affine3d::Identity();

      // Base
      relative_pose.translate(victor_arms[arm].ee_start_translation);

      // Translation
      Eigen::Vector3d translation(0, 0, 0);
      translation += viveToVictorTranslation(pointMsgToEigen(msg_controller.posestamped.pose.position));
      translation -= viveToVictorTranslation(victor_arms[arm].controller_start_translation);
      translation *= (msg_controller.joystick.axes[1] + 1.5) * .5; // motion scaling

      relative_pose.translate(translation);

      // Rotation
      relative_pose.rotate(viveToVictorRotation(quatMsgToEigen(msg_controller.posestamped.pose.orientation)));
      relative_pose.rotate(viveToVictorRotation(victor_arms[arm].controller_start_rotation).inverse());

      // Generate IK solutions
      const kinematics::KinematicsBaseConstPtr& solver = victor_arms[arm].joint_model_group->getSolverInstance();
      assert(solver.get());

      Eigen::Affine3d solverTrobot = Eigen::Affine3d::Identity();
      kinematic_state->setToIKSolverFrame(solverTrobot, solver);

      // Convert to solver frame
      Eigen::Affine3d pt_solver = solverTrobot * relative_pose;

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

        victor_arms[arm].last_valid_pose = relative_pose;
        victor_arms[arm].ee_start_translation = relative_pose.translation();

        victor_arms[arm].err_msg.text = "";
      }
      else
      {
        victor_arms[arm].err_msg.text = "Did not find IK solution";
      }

      std::cerr << "Got " << solutions.size() << " solutions for " << victor_arms[arm].joint_model_group->getName() << std::endl;

      // Arm control
      victor_hardware_interface::MotionCommand msg_out_arm;

      std::vector<double> joint_values;
      kinematic_state->copyJointGroupPositions(victor_arms[arm].joint_model_group, joint_values);

      msg_out_arm.joint_position.joint_1 = joint_values[0];
      msg_out_arm.joint_position.joint_2 = joint_values[1];
      msg_out_arm.joint_position.joint_3 = joint_values[2];
      msg_out_arm.joint_position.joint_4 = joint_values[3];
      msg_out_arm.joint_position.joint_5 = joint_values[4];
      msg_out_arm.joint_position.joint_6 = joint_values[5];
      msg_out_arm.joint_position.joint_7 = joint_values[6];

      victor_arms[arm].controller_start_translation = pointMsgToEigen(msg_controller.posestamped.pose.position);

      victor_arms[arm].err_msg.id = arm;
      victor_arms[arm].err_msg.pose.position.x = relative_pose.translation()[0];
      victor_arms[arm].err_msg.pose.position.y = relative_pose.translation()[1];
      victor_arms[arm].err_msg.pose.position.z = relative_pose.translation()[2];
      victor_arms[arm].err_msg.header.stamp = ros::Time();
      //pub_err_msg.publish(victor_arms[arm].err_msg);

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

        victor_arms[arm].last_valid_synced_pose = relative_pose;
      }

      victor_arms[arm].pub_gripper.publish(msg_out_gripper);

      // Display rviz poses
      tf::Transform transform1;
      tf::poseEigenToTF(relative_pose, transform1);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/relative_pose"));

      tf::Transform transform2;
      tf::poseEigenToTF(translationAndRotationToAffine(victor_arms[arm].controller_start_translation, victor_arms[arm].controller_start_rotation), transform2);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/reset_pose"));

      tf::Transform transform3;
      tf::poseEigenToTF(poseMsgToEigen(msg_controller.posestamped.pose), transform3);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/global_pose"));

      tf::Transform transform4;
      tf::poseEigenToTF(victor_arms[arm].last_valid_pose, transform4);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/last_valid_pose"));

      tf::Transform transform5;
      tf::poseEigenToTF(victor_arms[arm].last_valid_synced_pose, transform5);
      tf_broadcaster.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group->getName() + "/last_valid_synced_pose"));
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

    return distance < 1.5;
  }

  Eigen::Affine3d translationAndRotationToAffine(Eigen::Vector3d translation, Eigen::Quaterniond rotation)
  {
    Eigen::Affine3d out = Eigen::Affine3d::Identity();
    out.translate(translation);
    out.rotate(rotation);
    return out;
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