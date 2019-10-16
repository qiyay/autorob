//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
links_geom_imported = false;
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "myrobot";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,1,0], rpy:[0,0,0]};  // held a bit over the ground plane

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

robot.links = {
    "base": {},  
    "head":{},
    "neck":{},
    "clavicle_right": {}, 
    "shoulder_right": {}, 
    "upperarm_right": {}, 
    "forearm_right": {},
    "clavicle_left": {} , 
    "shoulder_left": {}, 
    "upperarm_left": {}, 
    "forearm_left": {},
    "forward_right_leg":{},
    "forward_left_leg":{},
    "back_right_leg":{},
    "back_left_leg":{}
};

robot.joints = {};

robot.joints.clavicle_right_yaw = {parent:"base", child:"clavicle_right"};
robot.joints.clavicle_right_yaw.origin = {xyz: [0.5,0.4,0.5], rpy:[-Math.PI/2,0,0]};
robot.joints.clavicle_right_yaw.axis = [0.0,0.0,-1.0]; 

robot.joints.shoulder_right_yaw = {parent:"clavicle_right", child:"shoulder_right"};
robot.joints.shoulder_right_yaw.origin = {xyz: [0.0,-0.15,0.85], rpy:[Math.PI/2,0,0]};
robot.joints.shoulder_right_yaw.axis = [0.0,0.707,0.707]; 

robot.joints.upperarm_right_pitch = {parent:"shoulder_right", child:"upperarm_right"};
robot.joints.upperarm_right_pitch.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.upperarm_right_pitch.axis = [0.0,1.0,0.0]; 

robot.joints.forearm_right_yaw = {parent:"upperarm_right", child:"forearm_right"};
robot.joints.forearm_right_yaw.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.forearm_right_yaw.axis = [1.0,0.0,0.0]; 

robot.joints.clavicle_left_roll = {parent:"base", child:"clavicle_left"};
robot.joints.clavicle_left_roll.origin = {xyz: [-0.5,0.4,0.5], rpy:[-Math.PI/2,0,0]};
robot.joints.clavicle_left_roll.axis = [0.0,0.0,1.0]; 

robot.joints.shoulder_left_yaw = {parent:"clavicle_left", child:"shoulder_left"};
robot.joints.shoulder_left_yaw.origin = {xyz: [0.0,-0.15,0.85], rpy:[Math.PI/2,0,0]};
robot.joints.shoulder_left_yaw.axis = [0.0,0.707,0.707]; 

robot.joints.upperarm_left_pitch = {parent:"shoulder_left", child:"upperarm_left"};
robot.joints.upperarm_left_pitch.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.upperarm_left_pitch.axis = [0.0,1.0,0.0]; 

robot.joints.forearm_left_yaw = {parent:"upperarm_left", child:"forearm_left"};
robot.joints.forearm_left_yaw.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.forearm_left_yaw.axis = [1.0,0.0,0.0]; 



robot.joints.neck_yaw = {parent:"base", child:"neck"};
robot.joints.neck_yaw.origin = {xyz: [0,0.4,-0.5], rpy:[-Math.PI/2,0,0]};
robot.joints.neck_yaw.axis = [0.0,0.0,1.0]; 

robot.joints.head_roll = {parent:"neck", child:"head"};
robot.joints.head_roll.origin = {xyz: [0,0,1], rpy:[0,0,0]};
robot.joints.head_roll.axis = [0.0,0.0,1.0]; 

robot.joints.forward_right = {parent:"base", child:"forward_right_leg"};
robot.joints.forward_right.origin = {xyz:[-0.6, -0.1, 0.6], rpy:[0,0,Math.PI/2]}
robot.joints.forward_right.axis = [0.0, 1.0, 0.0]

robot.joints.forward_left = {parent:"base", child:"forward_left_leg"};
robot.joints.forward_left.origin = {xyz:[ 0.6, -0.1, 0.6], rpy:[0,0,Math.PI/2]}
robot.joints.forward_left.axis = [0.0, 1.0, 0.0]

robot.joints.back_right = {parent:"base", child:"back_right_leg"};
robot.joints.back_right.origin = {xyz:[-0.6, -0.1, -0.6], rpy:[0,0,Math.PI/2]}
robot.joints.back_right.axis = [0.0, 1.0, 0.0]

robot.joints.back_left = {parent:"base", child:"back_left_leg"};
robot.joints.back_left.origin = {xyz:[ 0.6, -0.1, -0.6], rpy:[0,0,Math.PI/2]}
robot.joints.back_left.axis = [0.0, 1.0, 0.0]

robot.endeffector = {};
robot.endeffector.frame = "forearm_right_yaw";
robot.endeffector.position = [[0],[0],[0.5],[1]]

links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 2, 0.4, 2);
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

links_geom["neck"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["neck"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["head"] = new THREE.CubeGeometry( 0.6, 0.6, 0.6 );
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["clavicle_right"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["clavicle_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["clavicle_left"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["clavicle_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["shoulder_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["shoulder_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["upperarm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["upperarm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["forearm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.5 );
links_geom["forearm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25) );

links_geom["shoulder_left"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["shoulder_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["upperarm_left"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["upperarm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["forearm_left"] = new THREE.CubeGeometry( 0.3, 0.3, 0.5 );
links_geom["forearm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25) );

links_geom["forward_right_leg"] = new THREE.CubeGeometry(1, 0.3, 0.3);
links_geom["forward_right_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.25, 0, 0.0) );

links_geom["forward_left_leg"] = new THREE.CubeGeometry(1, 0.3, 0.3);
links_geom["forward_left_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.25, 0, 0.0) );

links_geom["back_right_leg"] = new THREE.CubeGeometry(1, 0.3, 0.3);
links_geom["back_right_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.25, 0, 0.0) );

links_geom["back_left_leg"] = new THREE.CubeGeometry(1, 0.3, 0.3);
links_geom["back_left_leg"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.25, 0, 0.0) );