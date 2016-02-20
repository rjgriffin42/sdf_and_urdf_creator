#include "Mechanics.h"
#include "factory/RegisterPlugin.h"

#include <stdio.h>

// hands: undefined (use ends of wrists)
// chest: avg of shoulders
// torso: avg of hips
// thigh (e.g.): transform from hip to knee
// foot: center bottom of plate

using namespace KDL;

GazeboEscherMechanics::GazeboEscherMechanics()
{
  // coordinate transforms
  /////////////////////////////////////////////////////////////////////////

  // head
  clavicle_transform = Frame(Vector(0, 0, 0.0846));
  neck_transform = Frame(Vector(0, 0, 0.0286));
  head_transform = Frame(Vector(0, 0, 0));

  // l_arm
  l_shoulder_transform = Frame(Vector(0, 0.1275, 0));
  l_shoulder_roll_transform = Frame(Vector(0, 0.163, 0));
  l_arm_transform = Frame(Vector(0, 0, -0.3186));
  l_forearm_transform = Frame(Vector(0.2071, 0, 0));
  l_foreforearm_transform = Frame(Vector(0.0741, 0, 0));
  l_hand_transform = Frame(Vector(0.13, 0, 0));

  // r_arm
  r_shoulder_transform = Frame(Vector(0, -0.1275, 0));
  r_shoulder_roll_transform = Frame(Vector(0, -0.163, 0));
  r_arm_transform = Frame(Vector(0, 0, -0.3186));
  r_forearm_transform = Frame(Vector(0.2071, 0, 0));
  r_foreforearm_transform = Frame(Vector(0.0741, 0, 0));
  r_hand_transform = Frame(Vector(0.13, 0, 0));

  // waist
  torso_waist_transform = Frame(Vector(0, 0, 0.273));
  waist_chest_transform = Frame(Vector(0, 0, 0.2804));

  // l_leg
  l_hip_transform = Frame(Vector(0, 0.1, 0));
  l_thigh_transform = Frame(Vector(0, 0, -0.41));
  l_shin_transform = Frame(Vector(0, 0, -0.41));
  l_foot_transform = Frame(Vector(0.025, 0, -0.08));

  // r_leg
  r_hip_transform = Frame(Vector(0, -0.1, 0));
  r_thigh_transform = Frame(Vector(0, 0, -0.41));
  r_shin_transform = Frame(Vector(0, 0, -0.41));
  r_foot_transform = Frame(Vector(0.025, 0, -0.08));

  // link inertias
  /////////////////////////////////////////////////////////////////////////

  // head
  neck_inertia = RigidBodyInertia(
                   0.44686844714975,
                   Vector(-1e-08, 0.000288781, 0.025711325),
                   RotationalInertia(0.000347468, 0.000151286, 0.000333572,
                                     0, -0, -8.157e-06)
                 );
  head_inertia = RigidBodyInertia(
                   2.2541382751238,
                   Vector(-0.010119341, 8.991e-06, 0.1270393),
                   RotationalInertia(0.011491873, 0.012981774, 0.005290433,
                                     3.329e-06, 0.0013042, -8.284e-06)
                 );

  // l_arm
  l_shoulder_inertia = RigidBodyInertia(
                         1.4029669162343,
                         Vector(0.000688524, -0.080502545, -0.00011075299999996),
                         RotationalInertia(0.002733115, 0.001266362, 0.002795471,
                                           7.3154e-05, 1.54e-07, -1.541e-06)
                       );
  l_shoulder_roll_link_inertia = RigidBodyInertia(
                         0.7400862795871,
                         Vector(-0.000157497, -6.628799999997e-05, -0.015018902),
                         RotationalInertia(0.000940092, 0.000866608, 0.000365724,
                                           -7e-09, -1.865e-06, 2.817e-06)
                       );
  l_arm_inertia = RigidBodyInertia(
                    1.3481077477662,
                    Vector(4e-09, 0.00094170500000001, 0.109484526),
                    RotationalInertia(0.007190942, 0.007116565, 0.000965322,
                                      -1e-09, -0, -0.000130703)
                  );
  l_elbow_link_inertia = RigidBodyInertia(
                    0.96411904608338,
                    Vector(0.014760471, 0.00015822900000001, -3.8344000000023e-05),
                    RotationalInertia(0.000477001, 0.001214959, 0.001118907,
                                      -2.405e-06, -4.205e-06, 1.1e-08)
                  );
  l_forearm_inertia = RigidBodyInertia(
                        1.0316073057275,
                        Vector(-0.061792793, -0.00075732699999997, 5.1295999999867e-05),
                        RotationalInertia(0.000942831, 0.001534454, 0.001587995,
                                          -3.4005e-05, 2.35e-07, 4.3e-07)
                      );
  l_foreforearm_inertia = RigidBodyInertia(
                        1.1704258386716,
                        Vector(-0.037050444, -6.1067999999997e-05, 9.8300999999967e-05),
                        RotationalInertia(0.000543674, 0.002086265, 0.002086264,
                                          4.334e-06, 4.323e-06, -3.6e-08)
                      );
  l_hand_inertia = RigidBodyInertia(
                     0.28728661642083,
                     Vector(-0.10573191, -0.00039961099999997, 0.002145442),
                     RotationalInertia(0.000286377, 0.000387459, 0.000332968,
                                       -3.162e-06, -1.1537e-05, 2.06e-07)
                   );

  // r_arm
  r_shoulder_inertia = RigidBodyInertia(
                         1.4029669162343,
                         Vector(0.000693283, 0.080502545, -0.00011074799999999),
                         RotationalInertia(0.002733115, 0.001266353, 0.002795463,
                                           -7.2667e-05, 1.21e-07, 1.541e-06)
                       );
  r_shoulder_roll_link_inertia = RigidBodyInertia(
                         0.7400862795871,
                         Vector(0.000157497, 6.628799999997e-05, -0.015018902),
                         RotationalInertia(0.000940092, 0.000866608, 0.000365724,
                                           -7e-09, 1.865e-06, -2.817e-06)
                       );
  r_arm_inertia = RigidBodyInertia(
                    1.3481077477662,
                    Vector(-4e-09, -0.00094170500000001, 0.109484526),
                    RotationalInertia(0.007190942, 0.007116565, 0.000965322,
                                      -1e-09, 0, 0.000130703)
                  );
  r_elbow_link_inertia = RigidBodyInertia(
                    0.96411904608338,
                    Vector(0.014760471, 0.00015822899999995, -3.8344000000023e-05),
                    RotationalInertia(0.000477001, 0.001214959, 0.001118907,
                                      -2.405e-06, -4.205e-06, 1.1e-08)
                  );
  r_forearm_inertia = RigidBodyInertia(
                        1.0316073057275,
                        Vector(-0.061792793, 0.00072781699999996, -5.9229999999966e-05),
                        RotationalInertia(0.000942865, 0.001532968, 0.001589516,
                                          3.3796e-05, -5.48e-07, 2.168e-06)
                      );
  r_foreforearm_inertia = RigidBodyInertia(
                        1.1704258386716,
                        Vector(-0.037050444, 6.1067999999997e-05, -2.330099999992e-05),
                        RotationalInertia(0.000543674, 0.002086265, 0.002086264,
                                          -4.334e-06, -4.323e-06, -3.6e-08)
                      );
  r_hand_inertia = RigidBodyInertia(
                     0.28728661642083,
                     Vector(-0.10573191, 0.00039961099999997, -0.002070442),
                     RotationalInertia(0.000286377, 0.000387459, 0.000332968,
                                       3.162e-06, 1.1537e-05, 2.06e-07)
                   );

  // waist
  torso_inertia = RigidBodyInertia(
                    12.772560578258,
                    Vector(-0.027546522, -8.1528e-05, 0.133351888),
                    RotationalInertia(0.189594996, 0.165226373, 0.191129536,
                                      0.000243698, -0.00084363, 2.491e-05)
                  );
  chest_inertia = RigidBodyInertia(
                    16.106280313052,
                    Vector(-0.024651386,-0.000572102, -0.10682948),
                    RotationalInertia(0.320061097, 0.258881187, 0.19263458,
                                      -0.001679142, -0.006857708, 0.005595512)
                  );

  // l_leg
  l_thigh_inertia = RigidBodyInertia(
                      7.1681601066269,
                      Vector(-0.012893801, 0.021195329, 0.202905819),
                      RotationalInertia(0.079832533, 0.090497961, 0.04397275,
                                        0.005608169, -0.014464827, -0.003214845)
                    );
  l_shin_inertia = RigidBodyInertia(
                     4.9340514217509,
                     Vector(0.039203201, 0.000387055, 0.255835621),
                     RotationalInertia(0.049926224, 0.051902624, 0.017996958,
                                       -6.1882e-05, 0.008459786, -0.000412746)
                   );
  l_foot_inertia = RigidBodyInertia(
                     1.8136595759808,
                     Vector(-0.028540497, 0.00037102999999999, 0.036306518),
                     RotationalInertia(0.003629189, 0.007181627, 0.007515445,
                                       1.3079e-05, -0.001366256, 2.3132e-05)
                   );

  // r_leg
  r_thigh_inertia = RigidBodyInertia(
                      7.1608352785347,
                      Vector(-0.013034539, -0.020824424, 0.202998937),
                      RotationalInertia(0.079523937, 0.090222577, 0.044391657,
                                        -0.005865006, -0.014386485, 0.003378144)
                    );
  r_shin_inertia = RigidBodyInertia(
                     4.9340513403528,
                     Vector(0.039203093, -0.000281832, 0.25583528),
                     RotationalInertia(0.049926754, 0.051902784, 0.01799724,
                                       8.0717e-05, 0.008459765, 0.000377725)
                   );
  r_foot_inertia = RigidBodyInertia(
                     1.8175685938097,
                     Vector(-0.028252817, -0.00035825099999999, 0.036307553),
                     RotationalInertia(0.003638673, 0.007191304, 0.007526564,
                                       8.912e-06, -0.001360529, -2.3098e-05)
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
  l_arm.addSegment(Segment("l_shoulder_pitch", Joint(Joint::RotY),
                           l_shoulder_roll_transform, l_shoulder_inertia));
  l_arm.addSegment(Segment("l_shoulder_roll", Joint(Joint::RotX),
                           Frame::Identity(), l_shoulder_roll_link_inertia));
  l_arm.addSegment(Segment("l_arm", Joint(Joint::RotZ),
                           l_arm_transform, l_arm_inertia));
  l_arm.addSegment(Segment("l_elbow", Joint(Joint::RotY),
                           Frame::Identity(), l_elbow_link_inertia));
  l_arm.addSegment(Segment("l_forearm", Joint(Joint::RotX),
                           l_forearm_transform, l_forearm_inertia));
  l_arm.addSegment(Segment("l_foreforearm", Joint(Joint::RotY),
                           l_foreforearm_transform, l_foreforearm_inertia));
  l_arm.addSegment(Segment("l_hand", Joint(Joint::RotZ),
                           l_hand_transform, l_hand_inertia));

  // r_arm
  r_arm = Chain();
  r_arm.addSegment(Segment("r_shoulder", Joint(Joint::None),
                           r_shoulder_transform));
  r_arm.addSegment(Segment("r_shoulder_pitch", Joint(Joint::RotY),
                           r_shoulder_roll_transform, r_shoulder_inertia));
  r_arm.addSegment(Segment("r_shoulder_roll", Joint(Joint::RotX),
                           Frame::Identity(), r_shoulder_roll_link_inertia));
  r_arm.addSegment(Segment("r_arm", Joint(Joint::RotZ),
                           r_arm_transform, r_arm_inertia));
  r_arm.addSegment(Segment("r_elbow", Joint(Joint::RotY),
                           Frame::Identity(), r_elbow_link_inertia));
  r_arm.addSegment(Segment("r_forearm", Joint(Joint::RotX),
                           r_forearm_transform, r_forearm_inertia));
  r_arm.addSegment(Segment("r_foreforearm", Joint(Joint::RotY),
                           r_foreforearm_transform, r_foreforearm_inertia));
  r_arm.addSegment(Segment("r_hand", Joint(Joint::RotZ),
                           r_hand_transform, r_hand_inertia));

  // waist
  waist = Chain();
  waist.addSegment(Segment("waist", Joint(Joint::RotZ),
                           torso_waist_transform));
  waist.addSegment(Segment("chest", Joint(Joint::None),
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

REGISTER_PLUGIN(gazebo_escher, Mechanics, GazeboEscherMechanics);

