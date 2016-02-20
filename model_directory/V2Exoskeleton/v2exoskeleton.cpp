#include <stdio.h>

using namespace KDL;

V2ExoskeletonMechanics::V2ExoskeletonMechanics()
{
  // coordinate transforms
  /////////////////////////////////////////////////////////////////////////

  // l_leg
  l_hip_transform = Frame(Vector(0, 0.1, 0));
  l_thigh_transform = Frame(Vector(0, 0, -0.41));
  l_shin_transform = Frame(Vector(0, 0, -0.41));
  l_foot_transform = Frame(Vector(0.025, 0, -0.0815875));

  // r_leg
  r_hip_transform = Frame(Vector(0, -0.1, 0));
  r_thigh_transform = Frame(Vector(0, 0, -0.41));
  r_shin_transform = Frame(Vector(0, 0, -0.41));
  r_foot_transform = Frame(Vector(0.025, 0, -0.0815875));

  // link inertias
  /////////////////////////////////////////////////////////////////////////

  // waist
  torso_inertia = RigidBodyInertia(
                    12.432352263,
                    Vector(-0.011453866, -2.8926e-05, 0.132504541),
                    RotationalInertia(0.192434317, 0.138750791, 0.152566986,
                                      0.000154048, 0.005354688, 6.1397e-05)
                  );

  // l_leg
  l_thigh_inertia = RigidBodyInertia(
                      7.19530739,
                      Vector(-0.009556145, 0.018857411, 0.198004556),
                      RotationalInertia(0.084250058, 0.096381201, 0.045852856,
                                        -0.004359838, -0.016761334, -0.001798911)
                    );
  l_shin_inertia = RigidBodyInertia(
                     4.554559181,
                     Vector(0.039081423, 0.00039793299999999, 0.254864228),
                     RotationalInertia(0.046092742, 0.047576074, 0.017089465,
                                       -5.375e-05, 0.007328075, -0.000398792)
                   );
  l_foot_inertia = RigidBodyInertia(
                     1.766778963,
                     Vector(-0.028605608, 0.000378564, 0.038443551),
                     RotationalInertia(0.003706824, 0.007284474, 0.007592302,
                                       1.3207e-05, -0.001373646, 2.331e-05)
                   );

  // r_leg
  r_thigh_inertia = RigidBodyInertia(
                      6.78272923,
                      Vector(-0.014241341, -0.020689569, 0.20301533),
                      RotationalInertia(0.079240154, 0.089456087, 0.043072055,
                                        -0.005800059, -0.014283399, 0.003245855)
                    );
  r_shin_inertia = RigidBodyInertia(
                     4.554559104,
                     Vector(0.039081302, -0.000317208, 0.254863833),
                     RotationalInertia(0.046093172, 0.047576244, 0.017089664,
                                       7.7546e-05, 0.007328046, 0.000388378)
                   );
  r_foot_inertia = RigidBodyInertia(
                     1.770237351,
                     Vector(-0.028322076, -0.000365984, 0.038447661),
                     RotationalInertia(0.003363358, 0.006961913, 0.007424205,
                                       -8.734e-06, -0.001368115, -2.3263e-05)
                   );

  // kinematic chains
  /////////////////////////////////////////////////////////////////////////

  // l_leg
  l_leg = Chain();
  l_leg.addSegment(Segment("l_hip", Joint(Joint::None),
                           l_hip_transform));
  l_leg.addSegment(Segment("l_thigh", Joint(Joint::RotY),
                           l_thigh_transform, l_thigh_inertia));
  l_leg.addSegment(Segment("l_shin", Joint(Joint::RotY),
                           l_shin_transform, l_shin_inertia));
  l_leg.addSegment(Segment("l_foot", Joint(Joint::RotY),
                           l_foot_transform, l_foot_inertia));

  // r_leg
  r_leg = Chain();
  r_leg.addSegment(Segment("r_hip", Joint(Joint::None),
                           r_hip_transform));
  r_leg.addSegment(Segment("r_thigh", Joint(Joint::RotY),
                           r_thigh_transform, r_thigh_inertia));
  r_leg.addSegment(Segment("r_shin", Joint(Joint::RotY),
                           r_shin_transform, r_shin_inertia));
  r_leg.addSegment(Segment("r_foot", Joint(Joint::RotY),
                           r_foot_transform, r_foot_inertia));

  // kinematic trees
  /////////////////////////////////////////////////////////////////////////
  body = Tree();
  body.addSegment(Segment("torso", Joint(Joint::None),
                          Frame::Identity(), torso_inertia), "root");
  body.addChain(l_leg, "torso");
  body.addChain(r_leg, "torso");

  // tree segment indices
  /////////////////////////////////////////////////////////////////////////
  body_torso_index = 0;
  body_l_foot_index = body_torso_index + l_leg.getNrOfSegments();
  body_r_foot_index = body_l_foot_index + r_leg.getNrOfSegments();

  // kinematic and dynamic solvers
  /////////////////////////////////////////////////////////////////////////

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

REGISTER_PLUGIN(v2exoskeleton, Mechanics, V2ExoskeletonMechanics);

