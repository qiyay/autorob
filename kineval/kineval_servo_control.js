
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
    // update time, update running sum
    update_time();
    kineval.params.sumTime += kineval.dt;
    // console.log(kineval.params.sumTime)
    // if 2 seconds have passed, reset timer
    if (kineval.params.sumTime>=0.5){
        // console.log("t")
        kineval.params.sumTime=0.0;
        kineval.params.dance_pose_index = kineval.params.dance_sequence_index.shift();
        kineval.params.dance_sequence_index.push(kineval.params.dance_pose_index);
        for (x in robot.joints) {
                kineval.params.setpoint_target[x] = kineval.setpoints[kineval.params.dance_pose_index][x];
        }
    }
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (x in robot.joints) {
        robot.joints[x].control = robot.joints[x].servo.p_gain*(kineval.params.setpoint_target[x] - robot.joints[x].angle);
    }
}

function update_time(){
    var t = Date.now();
    kineval.dt = (t - kineval.t_)/1000;
    kineval.t_ = t;
}
