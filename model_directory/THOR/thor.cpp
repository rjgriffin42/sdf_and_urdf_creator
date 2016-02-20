#include "Mechanics.h"
#include "factory/RegisterPlugin.h"

#include <stdio.h>

// hands: undefined (use ends of wrists)
// chest: avg of shoulders
// torso: avg of hips
// thigh (e.g.): transform from hip to knee
// foot: center bottom of plate

using namespace KDL;

GazeboThorMechanics::GazeboThorMechanics()
{
  // coordinate transforms
  /////////////////////////////////////////////////////////////////////////

  // head
  clavicle_transform = Frame(Vector(0, 0, 0.064));
  neck_transform = Frame(Vector(0, 0, 0.027));
  head_transform = Frame(Vector(0, 0, 0));

  // l_arm
  l_shoulder_transform = Frame(Vector(0, 0.2375, 0));
  l_arm_transform = Frame(Vector(0.03, 0, -0.27));
  l_forearm_transform = Frame(Vector(-0.03, 0, -0.224));
  l_foreforearm_transform = Frame(Vector(0, 0, -0.135));
  l_hand_transform = Frame(Vector(0, 0.00686, -0.0649));

  // r_arm
  r_shoulder_transform = Frame(Vector(0, -0.2375, 0));
  r_arm_transform = Frame(Vector(0.03, 0, -0.27));
  r_forearm_transform = Frame(Vector(-0.03, 0, -0.224));
  r_foreforearm_transform = Frame(Vector(0, 0, -0.135));
  r_hand_transform = Frame(Vector(0, -0.00686, -0.0649));

  // waist
  torso_waist_transform = Frame(Vector(0, 0, 0.365));
  waist_chest_transform = Frame(Vector(0, 0, 0.214));

  // l_leg
  l_hip_transform = Frame(Vector(0, 0.1, 0));
  l_thigh_transform = Frame(Vector(0, 0, -0.41));
  l_shin_transform = Frame(Vector(0, 0, -0.41));
  l_foot_transform = Frame(Vector(0.025, 0, -0.0740875));

  // r_leg
  r_hip_transform = Frame(Vector(0, -0.1, 0));
  r_thigh_transform = Frame(Vector(0, 0, -0.41));
  r_shin_transform = Frame(Vector(0, 0, -0.41));
  r_foot_transform = Frame(Vector(0.025, 0, -0.0740875));

  // link inertias
  /////////////////////////////////////////////////////////////////////////

  // head
  neck_inertia = RigidBodyInertia(
                   0.4321475707231,
                   Vector(1.2e-08, 0.000124683, 0.024528056),
                   RotationalInertia(0.000336505, 0.000143407, 0.000323735,
                                     -0, 0, -8.463e-06)
                 );
  head_inertia = RigidBodyInertia(
                   2.7485744797002,
                   Vector(-0.011854433, 3.4004e-05, 0.134411858),
                   RotationalInertia(0.015624603, 0.014437678, 0.00866519,
                                     7.4e-07, 0.001265119, -8.757e-06)
                 );

  // l_arm
  l_arm_inertia = RigidBodyInertia(
                    3.0525004835626,
                    Vector(-0.022785913, -0.001779321, 0.157131069),
                    RotationalInertia(0.038478468, 0.039609455, 0.004412843,
                                      4.9829e-05, -0.003774572, -0.000681544)
                  );
  l_forearm_inertia = RigidBodyInertia(
                        1.5602876810053,
                        Vector(-0.000544339, 0.00037668700000001, 0.088847148),
                        RotationalInertia(0.006819683, 0.006750973, 0.001205454,
                                          5.676e-06, 0.000101312, 4.3534e-05)
                      );
  l_foreforearm_inertia = RigidBodyInertia(
                        0.48270235234568,
                        Vector(0.000452337, 0, 0.055526871),
                        RotationalInertia(0.000584304, 0.000662668, 0.000222334,
                                          -0, 1.3796e-05, -0)
                      );
  l_hand_inertia = RigidBodyInertia(
                     0.91893928798941,
                     Vector(-0.004426292, -0.018563864, 0.00036891500000003),
                     RotationalInertia(0.002110654, 0.001815588, 0.00141256,
                                       7.1762e-05, 0.000134995, 0.000302996)
                   );

  // r_arm
  r_arm_inertia = RigidBodyInertia(
                    3.0525004835626,
                    Vector(-0.022786665, 0.001779321, 0.157131069),
                    RotationalInertia(0.038478468, 0.039609764, 0.004413151,
                                      -4.9825e-05, -0.003774703, 0.000681544)
                  );
  r_forearm_inertia = RigidBodyInertia(
                        1.5602876810053,
                        Vector(-0.000544339, 8.3232999999988e-05, 0.088847148),
                        RotationalInertia(0.006819886, 0.006750973, 0.001205657,
                                          4.675e-06, 0.000101316, 3.9504e-05)
                      );
  r_foreforearm_inertia = RigidBodyInertia(
                        0.48270235234568,
                        Vector(0.000452337, 0, 0.055526871),
                        RotationalInertia(0.000584304, 0.000662668, 0.000222334,
                                          -0, 1.3796e-05, -0)
                      );
  r_hand_inertia = RigidBodyInertia(
                     0.93828493014109,
                     Vector(0.002188328, 0.01931272, 0.00019496600000002),
                     RotationalInertia(0.002124517, 0.0018306, 0.001442877,
                                       7.5076e-05, -0.000143739, -0.000313261)
                   );

  // waist
  torso_inertia = RigidBodyInertia(
                    12.334925665414,
                    Vector(-0.02884476, -0.000339931, 0.139567304),
                    RotationalInertia(0.168786051, 0.151804339, 0.183945912,
                                      0.000230307, 0.000191055, 0.000299081)
                  );
  chest_inertia = RigidBodyInertia(
                    8.9898046997178,
                    Vector(-0.009475235,
                           0.003477582,
                           -0.104207367),
                    RotationalInertia(0.212139518, 0.177608078, 0.082558505,
                                      0.000361146, -0.001780773, 0.000482353)
                  );

  // l_leg
  l_thigh_inertia = RigidBodyInertia(
                      5.1833879853968,
                      Vector(-0.02004484, 0.027446943, 0.190293591),
                      RotationalInertia(0.050251535, 0.04823009, 0.01825638,
                                        -0.000369726, -0.00485734, 0.000428731)
                    );
  l_shin_inertia = RigidBodyInertia(
                     4.2674961878836,
                     Vector(0.035524575, 0.00050081099999999, 0.250155231),
                     RotationalInertia(0.042672286, 0.043770804, 0.012910027,
                                       -4.6942e-05, 0.005863435, -0.000372327)
                   );
  l_foot_inertia = RigidBodyInertia(
                     1.7321033562081,
                     Vector(-0.028042516, 0.00096434599999999, 0.033604241),
                     RotationalInertia(0.00336244, 0.006955959, 0.007417438,
                                       5.764e-06, -0.001236319, 3.0838e-05)
                   );

  // r_leg
  r_thigh_inertia = RigidBodyInertia(
                      5.1856016175308,
                      Vector(-0.020031387, -0.027475579, 0.190254768),
                      RotationalInertia(0.050266937, 0.048269843, 0.018232029,
                                        0.000400241, -0.004868597, -0.000479151)
                    );
  r_shin_inertia = RigidBodyInertia(
                     4.2674961878836,
                     Vector(0.035524569, -0.000282961, 0.250155231),
                     RotationalInertia(0.042672987, 0.043770806, 0.012910729,
                                       6.7751e-05, 0.005863442, 0.000406027)
                   );
  r_foot_inertia = RigidBodyInertia(
                     1.7353424510053,
                     Vector(-0.027942557, -0.00094983899999999, 0.033597042),
                     RotationalInertia(0.003363358, 0.006961913, 0.007424205,
                                       -3.631e-06, -0.001236643, -3.0884e-05)
                   );

  // kinematic chains
  /////////////////////////////////////////////////////////////////////////

  // head
  head = Chain();
  head.addSegment(Segment("clavicle", Joint(Joint::None),
                          clavicle_transform));
  head.addSegment(Segment("neck", Joint(Joint::RotZ),
                          neck_transform, neck_inertia));
  head.addSegment(Segment("head", Joint(Joint::RotY),
                          head_transform, head_inertia));

  // l_arm
  l_arm = Chain();
  l_arm.addSegment(Segment("l_shoulder", Joint(Joint::None),
                           l_shoulder_transform));
  l_arm.addSegment(Segment("l_shoulder_pitch", Joint(Joint::RotY)));
  l_arm.addSegment(Segment("l_shoulder_roll", Joint(Joint::RotX)));
  l_arm.addSegment(Segment("l_arm", Joint(Joint::RotZ),
                           l_arm_transform, l_arm_inertia));
  l_arm.addSegment(Segment("l_forearm", Joint(Joint::RotY),
                           l_forearm_transform, l_forearm_inertia));
  l_arm.addSegment(Segment("l_wrist_yaw", Joint(Joint::RotZ)));
  l_arm.addSegment(Segment("l_wrist_roll", Joint(Joint::RotX),
                           l_foreforearm_transform, l_foreforearm_inertia));
  l_arm.addSegment(Segment("l_hand", Joint(Joint::RotZ),
                           l_hand_transform, l_hand_inertia));

  // r_arm
  r_arm = Chain();
  r_arm.addSegment(Segment("r_shoulder", Joint(Joint::None),
                           r_shoulder_transform));
  r_arm.addSegment(Segment("r_shoulder_pitch", Joint(Joint::RotY)));
  r_arm.addSegment(Segment("r_shoulder_roll", Joint(Joint::RotX)));
  r_arm.addSegment(Segment("r_arm", Joint(Joint::RotZ),
                           r_arm_transform, r_arm_inertia));
  r_arm.addSegment(Segment("r_forearm", Joint(Joint::RotY),
                           r_forearm_transform, r_forearm_inertia));
  r_arm.addSegment(Segment("r_wrist_yaw", Joint(Joint::RotZ)));
  r_arm.addSegment(Segment("r_wrist_roll", Joint(Joint::RotX),
                           r_foreforearm_transform, r_foreforearm_inertia));
  r_arm.addSegment(Segment("r_hand", Joint(Joint::RotZ),
                           r_hand_transform, r_hand_inertia));

  // waist
  waist = Chain();
  waist.addSegment(Segment("waist", Joint(Joint::RotZ),
                           torso_waist_transform));
  waist.addSegment(Segment("chest", Joint(Joint::RotY),
                           waist_chest_transform, chest_inertia));

  // l_leg
  l_leg = Chain();
  l_leg.addSegment(Segment("l_hip", Joint(Joint::None),
                           l_hip_transform));
  l_leg.addSegment(Segment("l_hip_yaw", Joint(Joint::RotZ)));
  l_leg.addSegment(Segment("l_hip_roll", Joint(Joint::RotX)));
  l_leg.addSegment(Segment("l_thigh", Joint(Joint::RotY),
                           l_thigh_transform, l_thigh_inertia));
  l_leg.addSegment(Segment("l_shin", Joint(Joint::RotY),
                           l_shin_transform, l_shin_inertia));
  l_leg.addSegment(Segment("l_ankle_pitch", Joint(Joint::RotY)));
  l_leg.addSegment(Segment("l_foot", Joint(Joint::RotX),
                           l_foot_transform, l_foot_inertia));

  // r_leg
  r_leg = Chain();
  r_leg.addSegment(Segment("r_hip", Joint(Joint::None),
                           r_hip_transform));
  r_leg.addSegment(Segment("r_hip_yaw", Joint(Joint::RotZ)));
  r_leg.addSegment(Segment("r_hip_roll", Joint(Joint::RotX)));
  r_leg.addSegment(Segment("r_thigh", Joint(Joint::RotY),
                           r_thigh_transform, r_thigh_inertia));
  r_leg.addSegment(Segment("r_shin", Joint(Joint::RotY),
                           r_shin_transform, r_shin_inertia));
  r_leg.addSegment(Segment("r_ankle_pitch", Joint(Joint::RotY)));
  r_leg.addSegment(Segment("r_foot", Joint(Joint::RotX),
                           r_foot_transform, r_foot_inertia));

  // kinematic trees
  /////////////////////////////////////////////////////////////////////////
  body = Tree();
  body.addSegment(Segment("torso", Joint(Joint::None),
                          Frame::Identity(), torso_inertia), "root");
  body.addChain(l_leg, "torso");
  body.addChain(r_leg, "torso");
  body.addChain(waist, "torso");
  body.addChain(l_arm, "chest");
  body.addChain(r_arm, "chest");
  body.addChain(head, "chest");

  // tree segment indices
  /////////////////////////////////////////////////////////////////////////
  body_torso_index = 0;
  body_l_foot_index = body_torso_index + l_leg.getNrOfSegments();
  body_r_foot_index = body_l_foot_index + r_leg.getNrOfSegments();
  body_chest_index = body_r_foot_index + waist.getNrOfSegments();
  body_l_hand_index = body_chest_index + l_arm.getNrOfSegments();
  body_r_hand_index = body_l_hand_index + r_arm.getNrOfSegments();
  body_head_index = body_r_hand_index + head.getNrOfSegments();

  // kinematic and dynamic solvers
  /////////////////////////////////////////////////////////////////////////

  // head
  head_fk_pos_solver = new ChainFkSolverPos_recursive(head);
  head_fk_vel_solver = new ChainFkSolverVel_recursive(head);
  head_jnt_to_jac_solver = new ChainJntToJacSolver(head);
  head_id_solver = new ChainIdSolver_RNE(head, Mechanics::Gravity);

  // l_arm
  l_arm_fk_pos_solver = new ChainFkSolverPos_recursive(l_arm);
  l_arm_fk_vel_solver = new ChainFkSolverVel_recursive(l_arm);
  l_arm_ik_pos_solver = new ChainIkSolverPos_6dof_arm(l_arm);
  l_arm_ik_vel_solver = new ChainIkSolverVel_pinv(l_arm);
  l_arm_jnt_to_jac_solver = new ChainJntToJacSolver(l_arm);
  l_arm_id_solver = new ChainIdSolver_RNE(l_arm, Mechanics::Gravity);

  // r_arm
  r_arm_fk_pos_solver = new ChainFkSolverPos_recursive(r_arm);
  r_arm_fk_vel_solver = new ChainFkSolverVel_recursive(r_arm);
  r_arm_ik_pos_solver = new ChainIkSolverPos_6dof_arm(r_arm);
  r_arm_ik_vel_solver = new ChainIkSolverVel_pinv(r_arm);
  r_arm_jnt_to_jac_solver = new ChainJntToJacSolver(r_arm);
  r_arm_id_solver = new ChainIdSolver_RNE(r_arm, Mechanics::Gravity);

  // waist
  waist_fk_pos_solver = new ChainFkSolverPos_recursive(waist);
  waist_fk_vel_solver = new ChainFkSolverVel_recursive(waist);
  waist_jnt_to_jac_solver = new ChainJntToJacSolver(waist);
  waist_id_solver = new ChainIdSolver_RNE(waist, Mechanics::Gravity);

  // l_leg
  l_leg_fk_pos_solver = new ChainFkSolverPos_recursive(l_leg);
  l_leg_fk_vel_solver = new ChainFkSolverVel_recursive(l_leg);
  l_leg_ik_pos_solver = new ChainIkSolverPos_6dof_leg(l_leg);
  l_leg_ik_vel_solver = new ChainIkSolverVel_pinv(l_leg);
  l_leg_jnt_to_jac_solver = new ChainJntToJacSolver(l_leg);
  l_leg_id_solver = new ChainIdSolver_RNE(l_leg, Mechanics::Gravity);

  // r_leg
  r_leg_fk_pos_solver = new ChainFkSolverPos_recursive(r_leg);
  r_leg_fk_vel_solver = new ChainFkSolverVel_recursive(r_leg);
  r_leg_ik_pos_solver = new ChainIkSolverPos_6dof_leg(r_leg);
  r_leg_ik_vel_solver = new ChainIkSolverVel_pinv(r_leg);
  r_leg_jnt_to_jac_solver = new ChainJntToJacSolver(r_leg);
  r_leg_id_solver = new ChainIdSolver_RNE(r_leg, Mechanics::Gravity);

  // body
  body_fk_solver = new TreeFkFbSolverPos_recursive(body);
  body_id_solver = new TreeIdFbSolver_RNE(body, Mechanics::Gravity);
  body_fixed_base_id_solver = new TreeIdSolver_RNE(body, Mechanics::Gravity);
  body_jnt_to_cmm_solver = new TreeJntToCMMFbSolver_recursive(body);
  body_jnt_to_jac_solver = new TreeJntToJacFbSolver(body);
  body_dyn_param = new TreeDynParamFb(body, Mechanics::Gravity);
}

REGISTER_PLUGIN(gazebo_thor, Mechanics, GazeboThorMechanics);

