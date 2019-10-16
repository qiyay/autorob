//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

function quaternion_from_axisangle(t, a) {
	return [Math.cos(t/2), a[0]*Math.sin(t/2), a[1]*Math.sin(t/2), a[2]*Math.sin(t/2)];
}

function quaternion_normalize(q) {
	return vector_normalize(q);



}

function quaternion_to_rotation_matrix(q) {
	return [[1-2*(q[2]**2 + q[3]**2), 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[0]*q[2]+q[1]*q[3]),0],
            [2*(q[1]*q[2]+q[0]*q[3]), 1-2*(q[1]**2 + q[3]**2), 2*(q[2]*q[3]-q[0]*q[1]),0],
            [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2 + q[2]**2),0],
            [                      0,                       0,                       0,1]];
}

function quaternion_multiply(q1,q2) {
  return [q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3],
  		  q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2],
  		  q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1],
  		  q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]];
}