
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    var world_endeffector_trans = robot.joints[endeffector_joint].xform;
    var endeffector_position_world = matrix_multiply(world_endeffector_trans, endeffector_position_local);
    var world_r = Math.atan2(world_endeffector_trans[2][1], world_endeffector_trans[2][2]);
    var world_p = Math.atan2(-world_endeffector_trans[2][0], Math.sqrt(world_endeffector_trans[2][1]**2 + world_endeffector_trans[2][2]**2));
    var world_y = Math.atan2(world_endeffector_trans[1][0], world_endeffector_trans[0][0]);
    var endeffector_orientation_world = [world_r, world_p, world_y];
    // console.log(world_r);
    var delta_x = [];
    for(var i = 0; i < 6; ++i){
        delta_x[i] = [];
        if(i < 3) delta_x[i][0] = endeffector_target_world.position[i] - endeffector_position_world[i][0];
        else{
            if(kineval.params.ik_orientation_included){
                delta_x[i][0] = endeffector_target_world.orientation[i-3] - endeffector_orientation_world[i-3];
            }
            else delta_x[i][0] = 0;
        }
    }
    // console.log("jacob");
    var jacobain = []
    var curr_joint = endeffector_joint;
    var jnum = 0;

    while(1){

        var axis = [[robot.joints[curr_joint].axis[0]],
                    [robot.joints[curr_joint].axis[1]],
                    [robot.joints[curr_joint].axis[2]]];

        var axis_trans = [];
        for(let i = 0; i < 3; ++i){
            axis_trans[i] = [];
            for(let j = 0; j < 3; ++j){
                axis_trans[i][j] = robot.joints[curr_joint].xform[i][j];
            }
        }


        var world_joint_axis = []; 
        world_joint_axis = matrix_multiply(axis_trans, axis);
        world_joint_axis = [world_joint_axis[0][0], world_joint_axis[1][0], world_joint_axis[2][0]];
        world_joint_axis = vector_normalize(world_joint_axis);

        if(robot.joints[curr_joint].type == "prismatic") jacobain[jnum] = [world_joint_axis[0], world_joint_axis[1], world_joint_axis[2], 0, 0, 0];
        else{
    
            var world_joint_origin = matrix_multiply(robot.joints[curr_joint].xform, [[0], [0], [0], [1]]); 
            var origin_dis = [endeffector_position_world[0][0] - world_joint_origin[0][0], 
                              endeffector_position_world[1][0] - world_joint_origin[1][0],
                              endeffector_position_world[2][0] - world_joint_origin[2][0]];
            var origin_cross = vector_cross(world_joint_axis, origin_dis);
            jacobain[jnum] = [origin_cross[0], origin_cross[1], origin_cross[2],
                              world_joint_axis[0], world_joint_axis[1], world_joint_axis[2]];
        }



        if(robot.joints[curr_joint].parent == robot.base){
            // console.log("break");
            break;
        }

        jnum += 1;
        curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
    }

    // console.log("pinv");
    if(kineval.params.ik_pseudoinverse){
        var jacobain_t = matrix_transpose(jacobain);
        var jacobian_pinv = matrix_pseudoinverse(jacobain_t);
        var state_control = matrix_multiply(jacobian_pinv, delta_x);
    }

    else{
        var state_control = matrix_multiply(jacobain, delta_x);
    }
    // console.log("control");
    curr_joint = endeffector_joint;
    jnum = 0;
    while(1){
        robot.joints[curr_joint].control += kineval.params.ik_steplength * state_control[jnum][0];
        if(robot.joints[curr_joint].parent == robot.base) break;
        jnum += 1;
        curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
  }




}



