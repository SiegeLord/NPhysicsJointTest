#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_variables)]

use na::{Isometry3, Matrix4, Point3, Quaternion, RealField, Rotation3, Unit, Vector3};
use nalgebra as na;

use ncollide3d::pipeline::object::CollisionGroups;
use ncollide3d::shape::{Ball, ConvexHull, Plane, ShapeHandle};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::joint::{BallJoint, FixedJoint, FreeJoint, RevoluteJoint};
use nphysics3d::material::{BasicMaterial, MaterialHandle};
use nphysics3d::object::{
	Body, BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderSet,
	Ground, MultibodyDesc, RigidBody, RigidBodyDesc,
};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use gnuplot::*;

fn main()
{
	let v_high = 1.;
	let v_low = -1.;

	let mut mechanical_world = DefaultMechanicalWorld::new(Vector3::new(0., 0., 0.));
	let mut geometrical_world = DefaultGeometricalWorld::new();
	let mut bodies = DefaultBodySet::new();
	let mut colliders = DefaultColliderSet::new();
	let mut joint_constraints = DefaultJointConstraintSet::new();
	let mut force_generators = DefaultForceGeneratorSet::new();

	let mut body_desc = MultibodyDesc::new(FixedJoint::new(Isometry3::identity()));

	// Arm High
	let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::z()), 0.);
	joint.enable_angular_motor();
	joint.set_desired_angular_motor_velocity(v_high);
	joint.enable_max_angle(0.5);

	let arm_high_desc = body_desc
		.add_child(joint)
		.set_parent_shift(Vector3::new(0., 0., 0.));
	let arm_high_idx = 1;

	// Arm Low
	let mut joint = RevoluteJoint::new(Unit::new_normalize(Vector3::z()), 0.);
	joint.enable_angular_motor();
	joint.set_desired_angular_motor_velocity(v_low);

	arm_high_desc
		.add_child(joint)
		.set_parent_shift(Vector3::z() * 3.);
	let arm_low_idx = 2;

	let multibody_handle = bodies.insert(body_desc.build());

	// Add some mass.
	let mut coll_handles = vec![];
	for i in 0..3
	{
		let part_handle = BodyPartHandle(multibody_handle, i);

		let shape = ShapeHandle::new(Ball::new(0.5));
		let collider = ColliderDesc::new(shape)
			.density(1.)
			.collision_groups(
				CollisionGroups::new()
					.with_membership(&[1])
					.with_blacklist(&[1]),
			)
			.build(part_handle);

		let coll_handle = colliders.insert(collider);
		coll_handles.push(coll_handle);
	}

	let mut angles_vs_time: Vec<Vec<f32>> = vec![vec![], vec![], vec![]];

	for i in 0..100
	{
		mechanical_world.step(
			&mut geometrical_world,
			&mut bodies,
			&mut colliders,
			&mut joint_constraints,
			&mut force_generators,
		);

		for (j, &coll_handle) in coll_handles.iter().enumerate()
		{
			let rot = colliders.get(coll_handle).unwrap().position().rotation;
			let euler = rot.euler_angles();
			println!("{}", euler.2);
			angles_vs_time[j].push(euler.2);
		}
		println!("");
	}

	let mut fg = Figure::new();
	fg.axes2d()
		.set_title(&format!("v_{{high}} = {}, v_{{low}} = {}", v_high, v_low), &[])
		.lines(0..100, &angles_vs_time[1], &[Caption("angle high")])
		.lines(0..100, &angles_vs_time[2], &[Caption("angle low")]);
	fg.show().unwrap();
	fg.save_to_png(&format!("v_high_{}_v_low_{}.png", v_high, v_low), 800, 600).unwrap();
}
