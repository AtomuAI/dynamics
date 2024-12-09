// Copyright 2024 Bewusstsein Labs

use std::fmt::Debug;
use std::ops::{ Deref, DerefMut, Mul, Div, AddAssign };

use kinematics::{
    Assert, IsTrue,
    constraint::Constraint,
    joint::Joint,
    linkage::{
        Linkage,
        Linkage1D,
        Linkage2D,
        Linkage3D,
        Linkage4D,
    }
};

use kinetics::force::Force;

#[derive( Default, Debug )]
pub struct DynLinkage<I, T, const DIM: usize, const ORD: usize>( Linkage<I, T, DIM, ORD> )
where
    I: 'static + Default + Debug,
    T: 'static + Default + Copy + Debug + PartialEq,
    [Constraint<T, DIM>; (ORD + 1) * 2]: Default,
    [(); ORD + 1]:,
    [(); (ORD + 1) * 2]:;

impl <I, T, const DIM: usize, const ORD: usize> DynLinkage<I, T, DIM, ORD>
where
    I: 'static + Default + Copy + Debug + Ord,
    T: 'static + Default + Copy + Debug + PartialEq,
    [Constraint<T, DIM>; (ORD + 1) * 2]: Default,
    [(); ORD + 1]:,
    [(); (ORD + 1) * 2]:
{
    pub fn new() -> Self {
        Self( Linkage::<I, T, DIM, ORD>::new() )
    }

    pub fn apply_force( &mut self, id: I, force: Force<T, DIM>, time_step: T )
    where
        Assert<{ ORD >= 1 }>: IsTrue,
        T: 'static + Default + Copy + Debug + PartialEq + PartialOrd + Mul<Output = T> + Div<Output = T> + AddAssign
    {
        if let Some( joint ) = self.0.get_joint_mut( id ) {
            force.apply( joint, time_step );
        }
    }
}

impl <I, T, const DIM: usize, const ORD: usize> Deref for DynLinkage<I, T, DIM, ORD>
where
    I: 'static + Default + Copy + Debug + Ord,
    T: 'static + Default + Copy + Debug + PartialEq + PartialOrd + AddAssign + Mul<Output = T>,
    [Constraint<T, DIM>; (ORD + 1) * 2]: Default,
    [(); ORD + 1]:,
    [(); (ORD + 1) * 2]:
{
    type Target = Linkage<I, T, DIM, ORD>;

    fn deref( &self ) -> &Self::Target {
        &self.0
    }
}

impl <I, T, const DIM: usize, const ORD: usize> DerefMut for DynLinkage<I, T, DIM, ORD>
where
    I: 'static + Default + Copy + Debug + Ord,
    T: 'static + Default + Copy + Debug + PartialEq + PartialOrd + AddAssign + Mul<Output = T>,
    [Constraint<T, DIM>; (ORD + 1) * 2]: Default,
    [(); ORD + 1]:,
    [(); (ORD + 1) * 2]:
{
    fn deref_mut( &mut self ) -> &mut Self::Target {
        &mut self.0
    }
}

pub type DynLinkage1D<I, T, const ORD: usize> = DynLinkage<I, T, 1, ORD>;
pub type DynLinkage2D<I, T, const ORD: usize> = DynLinkage<I, T, 2, ORD>;
pub type DynLinkage3D<I, T, const ORD: usize> = DynLinkage<I, T, 3, ORD>;
pub type DynLinkage4D<I, T, const ORD: usize> = DynLinkage<I, T, 4, ORD>;

#[cfg(test)]
mod tests {
    use linear_algebra::vector::Vector3;
    use super::*;
    use kinematics::{
        link::Link,
        body::Body3D,
        joint::Joint3D,
        constraint::{ Range, Constraint3D }
    };

    #[test]
    fn new_test() {
        let mut linkage = DynLinkage3D::<usize, f32, 1>::new();
        linkage.add_joint( 0,
            Joint3D::new(
                Body3D::new( 1.0, [ Vector3::default(); 2 ], [ Vector3::default(); 2 ] ),
                [ Constraint3D::default(); 4 ]
            )
        ).unwrap();
        linkage.add_joint( 1,
            Joint3D::new(
                Body3D::new( 1.0, [ Vector3::default(); 2 ], [ Vector3::default(); 2 ] ),
                [ Constraint3D::default(); 4 ]
            )
        ).unwrap();
        linkage.add_joint( 2,
            Joint3D::new(
                Body3D::new( 1.0, [ Vector3::default(); 2 ], [ Vector3::default(); 2 ] ),
                [ Constraint3D::default(); 4 ]
            )
        ).unwrap();
        linkage.add_link( 0, 1, Link::default() ).unwrap();
        linkage.add_link( 1, 2, Link::default() ).unwrap();

        loop {
            if let Some( node ) = linkage.get_joint( 0 ) {
                println!( "Node 0:" );
                println!( "Velocity {:?}", node.spatial_velocity() );
                println!( "Position {:?}", node.position() );
            }
            linkage.apply_force( 0, Force::<f32, 3>::new( Vector3::<f32>::take([ 1.0, 2.0, 3.0 ]) ), 1.0 );
            linkage.update( 1.0 );
            linkage.constrain_joints();
            std::thread::sleep( std::time::Duration::from_secs_f32( 1.0 ) );
        }
    }

    /*
    #[test]
    fn display_test() {
        use kiss3d::nalgebra::{NaPoint3, NaTranslation3, NaUnitQuaternion, NaVector3};
        use kiss3d::scene::SceneNode;
        use kiss3d::window::Window;
        use kiss3d::light::Light;
        use std::f32::consts::PI;

        let mut window = Window::new("Kiss3d: cube");

        let mut linkage = DynLinkage3D::<usize, f32>::new();
        linkage.add_joint( 0,
            Joint3D::<f32>::new(
                Body3D::<f32>::new( 1.0, Vector3::<f32>::new(), Vector3::<f32>::new(), Vector3::<f32>::new(), Vector3::<f32>::new() ),
                Constraint3D::default(), Constraint3D::default(), Constraint3D::default(), Constraint3D::default()
            )
        ).unwrap();
        linkage.add_joint( 1,
            Joint3D::<f32>::new(
                Body3D::<f32>::new( 1.0, Vector3::<f32>::new(), Vector3::<f32>::new(), Vector3::<f32>::new(), Vector3::<f32>::new() ),
                Constraint3D::default(), Constraint3D::default(), Constraint3D::default(), Constraint3D::default()
            )
        ).unwrap();
        linkage.add_joint( 2,
            Joint3D::<f32>::new(
                Body3D::<f32>::new( 1.0, Vector3::<f32>::new(), Vector3::<f32>::new(), Vector3::<f32>::new(), Vector3::<f32>::new() ),
                Constraint3D::default(), Constraint3D::default(), Constraint3D::default(), Constraint3D::default()
            )
        ).unwrap();
        linkage.add_link( 0, 1, Link::<f32>::new() ).unwrap();
        linkage.add_link( 1, 2, Link::<f32>::new() ).unwrap();

        // Create visual nodes for joints
        let mut joint_nodes: Vec<SceneNode> = Vec::new();
        for _ in 0..3 {
            let mut joint_node = window.add_sphere(0.1);
            joint_node.set_color(1.0, 0.0, 0.0);
            joint_nodes.push(joint_node);
        }

        // Create visual nodes for links
        let mut link_nodes: Vec<SceneNode> = Vec::new();
        for _ in 0..2 {
            let mut link_node = window.add_cylinder(0.05, 1.0);
            link_node.set_color(0.0, 1.0, 0.0);
            link_nodes.push(link_node);
        }

        while window.render() {
            // Update joint positions
            for (i, joint_node) in joint_nodes.iter_mut().enumerate() {
                if let Some(joint) = linkage.get_joint(i) {
                    let position = joint.position();
                    joint_node.set_local_translation(Translation3::new(position.x, position.y, position.z));
                }
            }

            // Update link positions and orientations
            for (i, link_node) in link_nodes.iter_mut().enumerate() {
                if let (Some(joint_a), Some(joint_b)) = (linkage.get_joint(i), linkage.get_joint(i + 1)) {
                    let pos_a = joint_a.position();
                    let pos_b = joint_b.position();
                    let mid_point = (*pos_a + *pos_b) / 2.0;
                    let direction = *pos_b - *pos_a;
                    let length = direction.norm();
                    let rotation = UnitQuaternion::rotation_between(&Vector3::z_axis(), &direction.normalize()).unwrap_or(UnitQuaternion::identity());

                    link_node.set_local_translation(Translation3::new(mid_point.x, mid_point.y, mid_point.z));
                    link_node.set_local_rotation(rotation);
                    link_node.set_local_scale(1.0, 1.0, length);
                }
            }

            linkage.apply_force(0, Force::<f32, 3>::new(Vector3::<f32>::new(1.0, 2.0, 3.0)), 0.1);
            linkage.update(0.1);
            linkage.constrain_joints();
        }
    }
    */
}
