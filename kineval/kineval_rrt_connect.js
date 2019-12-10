
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;
    if(robot.origin.xyz[1]>0.5) q_goal_config[1] = robot.origin.xyz[1];
    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    eps = 1.0
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
        var q_rand = random_config();
        
        if(rrt_alg == 0){
            T_a = rrt_extend(T_a, q_rand);
            if(cdist(T_a.vertices[T_a.newest].vertex, q_goal_config) < 0.7 * eps){
                kineval.motion_plan = path_dfs(T_a);
                kineval.motion_plan_traversal_index = 0;
                return "reached";
            }
            else return "not reach";
        }

        else if(rrt_alg == 1){
            T_a = rrt_extend(T_a, q_rand);
            if(rrt_connect(T_a.vertices[T_a.newest].vertex)){
                
                var path_Ta = [], path_Tb = [];
                var path = [path_Ta, path_Tb];
                var Tree = [T_a, T_b];
                var start = 0;
                
                for(var i = 0; i < 2; ++i){
                    var path_idx = Tree[i].newest;
                    while(path_idx != null){
                        path[i].push(Tree[i].vertices[path_idx]);
                        Tree[i].vertices[path_idx].geom.material.color = {r:1,g:0,b:0};
                        path_idx = Tree[i].vertices[path_idx].parent;
                  }
                }                
                kineval.motion_plan = (path[1].reverse()).concat(path[0]);
                kineval.motion_plan_traversal_index = 0;
                return "reached";
            }
            var tmp = T_a;
            T_a = T_b;
            T_b = tmp;

            return "not reach";
        }

        else{
            var nearest = nearest_neighbor(T_a, q_rand);
            var q_new = new_config(q_rand, T_a.vertices[nearest].vertex);
            // console.log((T_a.vertices).length,1)
            if(!kineval.poseIsCollision(q_new)){
                var near = Near(q_new, 1.2*eps);
                var q_min_idx = choose_parent(near, nearest, q_new);
                rewire(near, q_min_idx);
            }

            if(cdist(T_a.vertices[T_a.newest].vertex, q_goal_config) < eps*0.7){
                kineval.motion_plan = path_dfs(T_a);
                kineval.motion_plan_traversal_index = 0;
                return "reached";
            }
            else return "not reach";

        }


    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].cost = 0;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function rrt_extend(tree, q_rand){
    var q_nearest = nearest_neighbor(tree, q_rand);
    var q_new = new_config(q_rand, tree.vertices[q_nearest].vertex);
    if(!kineval.poseIsCollision(q_new)){
        tree_add_vertex(tree, q_new);
        tree.vertices[tree.newest].parent = q_nearest;
    }
    return tree;
}

function rrt_connect(q_new){
    while(!(cdist(T_b.vertices[T_b.newest].vertex, q_new) < 0.5 * eps)){
        var last = T_b.newest;
        T_b = rrt_extend(T_b, q_new);
        if(last == T_b.newest){
            return false;
        }
    }
    return true;
}

function cdist(q1, q2){
    var dist = (q1[0]-q2[0])**2+(q1[2]-q2[2])**2+0.02*((q1[4]-q2[4])**2);
    // for(var i = 0; i <= q1.length - 1; ++i){
    //     if(i > 5) dist += 0.001 * (q1[i] - q2[i])**2;
    //     else dist += (q1[i] - q2[i])**2;
    // }
    // for(var i = 0; i <= q1.length - 1; ++i){
    //     if(i > 5) dist += 0.005 * (q1[i] - q2[i])**2;
    // }
    return Math.sqrt(dist);
}


function random_config(){
    var x_min = robot_boundary[0][0];
    var x_max = robot_boundary[1][0];
    var z_min = robot_boundary[0][2];
    var z_max = robot_boundary[1][2];
    var angle_min = -Math.PI;
    var angle_max = Math.PI;
  
    var ang_eps = (angle_max - angle_min)/eps;
    var x_rand = x_min + Math.floor((x_max - x_min)/eps * Math.random()) * eps;
    var z_rand = z_min + Math.floor((z_max - z_min)/eps * Math.random()) * eps;
    var angle_rand = angle_min + Math.floor(ang_eps * Math.random()) * eps;
    var q_rand = [x_rand,robot.origin.xyz[1],z_rand,0,angle_rand,0];

    for (joint in robot.joints){
        angle_rand = angle_min + Math.floor(ang_eps * Math.random()) * eps;
        q_rand.push(angle_rand);
    }

    if (rrt_alg != 1){ 
        var num_rand = Math.random();
        if (num_rand > 0.7){
            return q_rand;
        }
        else{
            return q_goal_config;
        }
    }

    return q_rand;
}

function new_config(q_rand,q_nearest){
    var dist = cdist(q_rand, q_nearest);
    var prop = eps / dist;
    var q_new = [];
    for(var i = 0; i < q_rand.length; ++i){
        q_new.push(q_nearest[i] + (q_rand[i] - q_nearest[i])*prop);
    }
    for(x in robot.joint){
        var type = robot.joints[x].type;
        if(type == "revolute" || type == "prismatic"){
            if(q_new[q_names[x]] > robot.joints[x].limit.upper){
                q_new[q_names[x]] = robot.joints[x].limit.upper;
            }
            if(q_new[q_names[x]] < robot.joints[x].limit.lower){
                q_new[q_names[x]] = robot.joints[x].limit.lower;
            }
        }
        else if(type == fixed) q_new[q_names[x]] = 0;
    }
    q_new[1] = robot.origin.xyz[1];;
    q_new[3] = 0;
    q_new[5] = 0;
    return q_new;
}

function nearest_neighbor(tree, q_rand){
    var idx = 0;
    var mindist = cdist(q_rand, tree.vertices[0].vertex);
    for(var i = 1; i<= tree.newest; ++i){
        var new_dist = cdist(q_rand, tree.vertices[i].vertex);
        if(mindist > new_dist){
            mindist = new_dist;
            idx = i;
        }
    }
    return idx;
}

function path_dfs(tree){
    // rrt_iterate = false;
    var path_idx = tree.newest;
    var path = [];
    while(path_idx != null){
        path.push(tree.vertices[path_idx]);
        tree.vertices[path_idx].geom.material.color = {r:1, g:0, b:0};
        path_idx = tree.vertices[path_idx].parent;
    }
    return path.reverse();
}

function Near(q_new, rad){
    var near = [];
    for(var i = 0; i <= T_a.newest; ++i){
        var dist = cdist(T_a.vertices[i].vertex, q_new);
        if(dist < rad){
            near.push(i);
        }
    }
    return near;
}

function choose_parent(near, nearest, q_new){
    var q_min_idx = nearest;
    var min_cost = T_a.vertices[nearest].cost + eps;
    for(var i = 0; i < near.length; ++i){
        if(near[i] != nearest){
            var node = T_a.vertices[near[i]];
            var cost = node.cost + cdist(node.vertex, q_new);
            // console.log(near.length, cost)
            if(cost < min_cost){
                q_min_idx = near[i];
                min_cost = cost;
            }
        }
    }
    tree_add_vertex(T_a, q_new);
    T_a.vertices[T_a.newest].cost = min_cost;
    T_a.vertices[T_a.newest].parent = q_min_idx;
    return q_min_idx;
}

function rewire(near, q_min_idx){
    for(var i = 0; i < near.length; ++i){
        if(near[i] != q_min_idx){

            var node = T_a.vertices[near[i]];
            var cost = T_a.vertices[T_a.newest].cost + cdist(node.vertex, T_a.vertices[T_a.newest].vertex);
            if(cost < node.cost){
                T_a.vertices[near[i]].parent = T_a.newest;
                T_a.vertices[near[i]].cost = cost;
            }
        }
    }
}