//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z
function matrix_multiply(m1, m2){
    var mat = [];
    var i,j,k;
    for (i=0;i<m1.length;i++) {
        mat[i] = [];
        for (j = 0; j < m2[0].length; j++){
            mat[i][j] = 0;
            for (k = 0; k < m2.length; k++){
                mat[i][j] += m1[i][k]*m2[k][j];
            }
        }
    }
    return mat;
}

function matrix_transpose(m1) {
    var mat = [];
    var i,j;
    for (i = 0; i < m1[0].length; i++){
        mat[i] = [];
        for (j = 0; j < m1.length; j++)
           mat[i][j] = m1[j][i];
    }
    return mat;
}

function matrix_pseudoinverse(m1) {
    var m1_t = matrix_transpose(m1);
    if (m1.length > m1[0].length){
        return matrix_multiply(numeric.inv(matrix_multiply(m1_t,m1)), m1_t);
    }
    else if (m1.length < m1[0].length){
        return matrix_multiply(m1_t, numeric.inv(matrix_multiply(m1,m1_t)));
    }
     
}

function matrix_invert(mat){
    var det = mat[0][0]*(mat[1][1]*mat[2][2]-mat[1][2]*mat[2][1])-mat[1][0]*(mat[0][1]*mat[2][2]-mat[0][2]*mat[2][1])+mat[2][0]*(mat[0][1]*mat[1][2]-mat[0][2]*mat[1][1]);
    var inv = [
            [(mat[1][1]*mat[2][2]-mat[1][2]*mat[2][1])/det, (mat[0][2]*mat[2][1]-mat[0][1]*mat[2][2])/det, (mat[0][1]*mat[1][2]-mat[0][2]*mat[1][1])/det],
            [(mat[1][2]*mat[2][0]-mat[1][0]*mat[2][2])/det, (mat[0][0]*mat[2][2]-mat[0][2]*mat[2][0])/det, (mat[0][2]*mat[1][0]-mat[0][0]*mat[1][2])/det],
            [(mat[1][0]*mat[2][1]-mat[1][1]*mat[2][0])/det, (mat[0][1]*mat[2][0]-mat[0][0]*mat[2][1])/det, (mat[0][0]*mat[1][1]-mat[1][0]*mat[0][1])/det]
            ];
    return inv;
}

function matrix_invert_affine(m1) {

    m1_r_inv = matrix_invert(m1);
    m1_t_inv = matrix_multiply(m1_r_inv, [[m1[0][3]],[m1[1][3]],[m1[2][3]]])

    mat = [
            [m1_r_inv[0][0], m1_r_inv[0][1], m1_r_inv[0][2], m1_t_inv[0][0]],
            [m1_r_inv[1][0], m1_r_inv[1][1], m1_r_inv[1][2], m1_t_inv[1][0]],
            [m1_r_inv[2][0], m1_r_inv[2][1], m1_r_inv[2][2], m1_t_inv[2][0]],
            [             0,              0,              0,              1]
            ]
    return mat;

}

function vector_normalize(v1) {
    var sum = 0;
    var norm = [];
    for (let i = 0; i < v1.length; i++)
        sum += v1[i] ** 2;
    sum = Math.sqrt(sum);
    for (let i = 0; i < v1.length; i++)
        norm[i] = v1[i]/sum;
    return norm;
}

function vector_cross(v1, v2) {
    var v_cross = [];
    v_cross[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v_cross[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v_cross[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return v_cross;
}

function generate_identity(n) {
    var mat = [];
    for (let i = 0; i < n; i++){
        mat[i] = [];
        for (let j = 0; j < n; j++){
            if (i == j) mat[i][j] = 1;
            else mat[i][j] = 0; 
        }

    }
    return mat;
}

function generate_translation_matrix(tx, ty, tz) {
    var mat = generate_identity(4);
    mat[0][3] = tx;
    mat[1][3] = ty;
    mat[2][3] = tz;
    return mat;
}


function generate_rotation_matrix_X(theta) {
    var mat = generate_identity(4);
    mat[1][1] = Math.cos(theta);
    mat[1][2] = -Math.sin(theta);
    mat[2][1] = Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}


function generate_rotation_matrix_Y(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][2] = Math.sin(theta);
    mat[2][0] = -Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

function generate_rotation_matrix_Z(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][1] = -Math.sin(theta);
    mat[1][0] = Math.sin(theta);
    mat[1][1] = Math.cos(theta);
    return mat;
}

