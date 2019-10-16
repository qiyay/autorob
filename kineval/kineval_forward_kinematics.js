
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms();

}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
kineval.buildFKTransforms = function buildFKTransforms () {
    traverseFKBase();
    for (var i = 0; i < robot.links[robot.base].children.length; i++)
        traverseFKJoint(robot.links[robot.base].children[i]);
}

function traverseFKBase(){
    var rx = generate_rotation_matrix_X(robot.origin.rpy[0]);
    var ry = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    var rz = generate_rotation_matrix_Z(robot.origin.rpy[2]);
    var rotate = matrix_multiply(matrix_multiply(rz, ry), rx);
    var trans = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
    robot.links[robot.base].xform = matrix_multiply(trans, rotate);

    if (robot.links_geom_imported) {
      var rosx = generate_rotation_matrix_X(-Math.PI/2);
      var rosy = generate_rotation_matrix_Y(-Math.PI/2);
      var ros_tran = matrix_multiply(rosy, rosx);
      robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, ros_tran);
    }

    var l_heading = [[0], [0], [1], [1]];
    var l_lateral = [[1], [0], [0], [1]];
    robot_heading = matrix_multiply(robot.links[robot.base].xform,l_heading);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform,l_lateral);
}

function traverseFKLink(link) {
    robot.links[link].xform = robot.joints[robot.links[link].parent].xform;
    if (typeof robot.links[link].children === 'undefined'){
        return;
    }
    for (var i = 0; i < robot.links[link].children.length; i++){
        traverseFKJoint(robot.links[link].children[i]);
    }
}

function traverseFKJoint(joint) {
    var rx = generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0]);
    var ry = generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1]);
    var rz = generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]);
    var rotate = matrix_multiply(matrix_multiply(rz, ry), rx);
    var trans = generate_translation_matrix(robot.joints[joint].origin.xyz[0], robot.joints[joint].origin.xyz[1], robot.joints[joint].origin.xyz[2]);
    var xform = matrix_multiply(trans, rotate);
    var parent_xform = robot.links[robot.joints[joint].parent].xform;
    var angle = robot.joints[joint].angle;

    var Rq = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(angle, robot.joints[joint].axis)));
    // console.log(Rq)
    if (robot.links_geom_imported) {
        if (robot.joints[joint].type == "fixed") Rq = generate_identity(4);
        else if (robot.joints[joint].type == "prismatic"){
            Rq = generate_translation_matrix(robot.joints[joint].axis[0]*angle,robot.joints[joint].axis[1]*angle,robot.joints[joint].axis[2]*angle);
        }
    }


    robot.joints[joint].xform = matrix_multiply(matrix_multiply(parent_xform, xform), Rq);
    traverseFKLink(robot.joints[joint].child);
}