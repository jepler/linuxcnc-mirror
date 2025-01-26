/********************************************************************
* Description: genserfuncs.c
*   (originally part of genserkins.c)
*   Kinematics for a generalised serial kinematics machine
*
*   Derived from a work by Fred Proctor,
*   changed to work with emc2 and HAL
*
* Adapting Author: Alex Joni
* License: GPL Version 2
* System: Linux
*
* Users:
*    1) genserkins.c  -- kinematics modules
*    2) ugenserkins.c -- usermode test program (needs work)
*******************************************************************

  These are the forward and inverse kinematic functions for a general
  serial-link manipulator. Thanks to Herman Bruyninckx and John
  Hallam at http://www.roble.info/ for this.

  The functions are general enough to be configured for any serial
  configuration.
  The kinematics use Denavit-Hartenberg definition for the joint and
  links. The DH definitions are the ones used by John J Craig in
  "Introduction to Robotics: Mechanics and Control"
  The parameters for the manipulator are defined by hal pins.
  Currently the type of the joints is hardcoded to ANGULAR, although
  the kins support both ANGULAR and LINEAR axes.

  TODO:
    * make number of joints a loadtime parameter
    * add HAL pins for all settable parameters, including joint type: ANGULAR / LINEAR
*/

#include "rtapi_math.h"
#include "gotypes.h"    /* go_result, go_integer */
#include "gomath.h"     /* go_pose */
#include "genserkins.h" /* these decls */
#include "kinematics.h"
#include "hal.h"

#ifdef RTAPI
#include "rtapi.h"
#endif

// Only gcc/g++ supports the #pragma
#if __GNUC__ && !defined(__clang__)
// The matrix and vector storage is just big.
// genser_kin_jac_inv() is 2112
// genserKinematicsInverse() is 2576
  #pragma GCC diagnostic warning "-Wframe-larger-than=2600"
#endif

static struct haldata {
    hal_u32_t     *max_iterations;
    hal_u32_t     *last_iterations;
    hal_float_t   *a[GENSER_MAX_JOINTS];
    hal_float_t   *alpha[GENSER_MAX_JOINTS];
    hal_float_t   *d[GENSER_MAX_JOINTS];
    hal_s32_t     *unrotate[GENSER_MAX_JOINTS];
    genser_struct *kins;
    go_pose *pos; // used in various functions, we malloc it
                  // only once in genserKinematicsSetup()
} *haldata = 0;

static int total_joints;
double j[GENSER_MAX_JOINTS];

#define A(i) (*(haldata->a[i]))
#define ALPHA(i) (*(haldata->alpha[i]))
#define D(i) (*(haldata->d[i]))

#define KINS_PTR (haldata->kins)

#if GENSER_MAX_JOINTS < 6
#error GENSER_MAX_JOINTS must be at least 6; fix genserkins.h
#endif

static int genser_hal_inited = 0;

int genser_kin_init(void) {
    genser_struct *genser = KINS_PTR;
    int t;

    static volatile double tst=0;tst=sqrt(tst); // ensure -lm used
    /* init them all and make them revolute joints */
    /* FIXME: should allow LINEAR joints based on HAL param too */
    for (t = 0; t < GENSER_MAX_JOINTS; t++) {
        genser->links[t].u.dh.a = A(t);
        genser->links[t].u.dh.alpha = ALPHA(t);
        genser->links[t].u.dh.d = D(t);
        genser->links[t].u.dh.theta = 0;
        genser->links[t].type = GO_LINK_DH;
        genser->links[t].quantity = GO_QUANTITY_ANGLE;
    }

    /* set a select few to make it PUMA-like */
    // FIXME-AJ: make a hal pin, also set number of joints based on it
    genser->link_num = 6;

    return GO_RESULT_OK;
}

/* compute the forward jacobian function:
   the jacobian is a linear approximation of the kinematics function.
   It is calculated using derivation of the position transformation matrix,
   and usually used for feeding velocities through it.
   It is analytically possible to calculate the inverse of the jacobian
   (sometimes only the pseudoinverse) and to use that for the inverse kinematics.
*/
int compute_jfwd(go_link * link_params,
                 int link_number,
                 go_matrix * Jfwd,
                 go_pose * T_L_0)
{
    GO_MATRIX_DECLARE(Jv, Jvstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jw, Jwstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_i_ip1, R_i_ip1stg, 3, 3);
    GO_MATRIX_DECLARE(scratch, scratchstg, 3, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(R_inv, R_invstg, 3, 3);
    go_pose pose;
    go_quat quat;
    go_vector P_ip1_i[3];
    int row, col;

    /* init matrices to possibly smaller size */
    go_matrix_init(Jv, Jvstg, 3, link_number);
    go_matrix_init(Jw, Jwstg, 3, link_number);
    go_matrix_init(R_i_ip1, R_i_ip1stg, 3, 3);
    go_matrix_init(scratch, scratchstg, 3, link_number);
    go_matrix_init(R_inv, R_invstg, 3, 3);

    Jv.el[0][0] = 0, Jv.el[1][0] = 0, Jv.el[2][0] = (GO_QUANTITY_LENGTH == link_params[0].quantity ? 1 : 0);
    Jw.el[0][0] = 0, Jw.el[1][0] = 0, Jw.el[2][0] = (GO_QUANTITY_ANGLE == link_params[0].quantity ? 1 : 0);

    /* initialize inverse rotational transform */
    if (GO_LINK_DH == link_params[0].type) {
        go_dh_pose_convert(&link_params[0].u.dh, &pose);
    } else if (GO_LINK_PP == link_params[0].type) {
        pose = link_params[0].u.pp.pose;
    } else {
        return GO_RESULT_IMPL_ERROR;
    }

    *T_L_0 = pose;

    for (col = 1; col < link_number; col++) {
        /* T_ip1_i */
        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }

        go_cart_vector_convert(&pose.tran, P_ip1_i);
        go_quat_inv(&pose.rot, &quat);
        go_quat_matrix_convert(&quat, &R_i_ip1);

        /* Jv */
        go_matrix_vector_cross(&Jw, P_ip1_i, &scratch);
        go_matrix_matrix_add(&Jv, &scratch, &scratch);
        go_matrix_matrix_mult(&R_i_ip1, &scratch, &Jv);
        Jv.el[0][col] = 0, Jv.el[1][col] = 0, Jv.el[2][col] = (GO_QUANTITY_LENGTH == link_params[col].quantity ? 1 : 0);
        /* Jw */
        go_matrix_matrix_mult(&R_i_ip1, &Jw, &Jw);
        Jw.el[0][col] = 0, Jw.el[1][col] = 0, Jw.el[2][col] = (GO_QUANTITY_ANGLE == link_params[col].quantity ? 1 : 0);
        if (GO_LINK_DH == link_params[col].type) {
            go_dh_pose_convert(&link_params[col].u.dh, &pose);
        } else if (GO_LINK_PP == link_params[col].type) {
            pose = link_params[col].u.pp.pose;
        } else {
            return GO_RESULT_IMPL_ERROR;
        }
        go_pose_pose_mult(T_L_0, &pose, T_L_0);
    }

    /* rotate back into {0} frame */
    go_quat_matrix_convert(&T_L_0->rot, &R_inv);
    go_matrix_matrix_mult(&R_inv, &Jv, &Jv);
    go_matrix_matrix_mult(&R_inv, &Jw, &Jw);

    /* put Jv atop Jw in J */
    for (row = 0; row < 6; row++) {
        for (col = 0; col < link_number; col++) {
            if (row < 3) {
                Jfwd->el[row][col] = Jv.el[row][col];
            } else {
                Jfwd->el[row][col] = Jw.el[row - 3][col];
            }
        }
    }

    return GO_RESULT_OK;
}

/* compute the inverse of the jacobian matrix */
int compute_jinv(go_matrix * Jfwd, go_matrix * Jinv)
{
    int retval;
    GO_MATRIX_DECLARE(JT, JTstg, GENSER_MAX_JOINTS, 6);

    /* compute inverse, or pseudo-inverse */
    if (Jfwd->rows == Jfwd->cols) {
        retval = go_matrix_inv(Jfwd, Jinv);
        if (GO_RESULT_OK != retval)
            return retval;
    } else if (Jfwd->rows < Jfwd->cols) {
        /* underdetermined, optimize on smallest sum of square of speeds */
        /* JT(JJT)inv */
        GO_MATRIX_DECLARE(JJT, JJTstg, 6, 6);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JJT, JJTstg, Jfwd->rows, Jfwd->rows);
        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(Jfwd, &JT, &JJT);
        retval = go_matrix_inv(&JJT, &JJT);
        if (GO_RESULT_OK != retval)
            return retval;
        go_matrix_matrix_mult(&JT, &JJT, Jinv);
    } else {
        /* overdetermined, do least-squares best fit */
        /* (JTJ)invJT */
        GO_MATRIX_DECLARE(JTJ, JTJstg, GENSER_MAX_JOINTS, GENSER_MAX_JOINTS);

        go_matrix_init(JT, JTstg, Jfwd->cols, Jfwd->rows);
        go_matrix_init(JTJ, JTJstg, Jfwd->cols, Jfwd->cols);
        go_matrix_transpose(Jfwd, &JT);
        go_matrix_matrix_mult(&JT, Jfwd, &JTJ);
        retval = go_matrix_inv(&JTJ, &JTJ);
        if (GO_RESULT_OK != retval)
            return retval;
        go_matrix_matrix_mult(&JTJ, &JT, Jinv);
    }

    return GO_RESULT_OK;
}

int genser_kin_jac_inv(void *kins,
    const go_pose * pos,
    const go_screw * vel, const go_real * joints, go_real * jointvels)
{
    (void)pos;
    genser_struct *genser = (genser_struct *) kins;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);
    go_pose T_L_0;
    go_link linkout[GENSER_MAX_JOINTS];
    go_real vw[6];
    int link;
    int retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);
    go_matrix_init(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);

    for (link = 0; link < genser->link_num; link++) {
        retval =
            go_link_joint_set(&genser->links[link], joints[link],
            &linkout[link]);
        if (GO_RESULT_OK != retval)
            return retval;
    }
    retval = compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
    if (GO_RESULT_OK != retval)
        return retval;
    retval = compute_jinv(&Jfwd, &Jinv);
    if (GO_RESULT_OK != retval)
        return retval;

    vw[0] = vel->v.x;
    vw[1] = vel->v.y;
    vw[2] = vel->v.z;
    vw[3] = vel->w.x;
    vw[4] = vel->w.y;
    vw[5] = vel->w.z;

    return go_matrix_vector_mult(&Jinv, vw, jointvels);
}

int genser_kin_jac_fwd(void *kins,
    const go_real * joints,
    const go_real * jointvels, const go_pose * pos, go_screw * vel)
{
    (void)pos;
    genser_struct *genser = (genser_struct *) kins;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    go_pose T_L_0;
    go_link linkout[GENSER_MAX_JOINTS];
    go_real vw[6];
    int link;
    int retval;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);

    for (link = 0; link < genser->link_num; link++) {
        retval =
            go_link_joint_set(&genser->links[link], joints[link],
            &linkout[link]);
        if (GO_RESULT_OK != retval)
            return retval;
    }

    retval = compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
    if (GO_RESULT_OK != retval)
        return retval;

    go_matrix_vector_mult(&Jfwd, jointvels, vw);
    vel->v.x = vw[0];
    vel->v.y = vw[1];
    vel->v.z = vw[2];
    vel->w.x = vw[3];
    vel->w.y = vw[4];
    vel->w.z = vw[5];

    return GO_RESULT_OK;
}

/* main function called by emc2 for forward Kins */
int genserKinematicsForward(const double *joint,
                            EmcPose * world,
                            const KINEMATICS_FORWARD_FLAGS * fflags,
                            KINEMATICS_INVERSE_FLAGS * iflags) {
    (void)fflags;
    (void)iflags;

    go_pose *pos;
    go_rpy rpy;
    go_real jcopy[GENSER_MAX_JOINTS]; // will hold the radian conversion of joints
    int ret = 0;
    int i, changed=0;
    if (!genser_hal_inited) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "genserKinematicsForward: not initialized\n");
        return -1;
    }

    for (i=0; i< 6; i++)  {
        // FIXME - debug hack
        if (!GO_ROT_CLOSE(j[i],joint[i])) changed = 1;
        // convert to radians to pass to genser_kin_fwd
        jcopy[i] = joint[i] * PM_PI / 180;
        if ((i) && *(haldata->unrotate[i]))
            jcopy[i] -= *(haldata->unrotate[i])*jcopy[i-1];
    }

    if (changed) {
        for (i=0; i< 6; i++)
            j[i] = joint[i];
            // rtapi_print("genserKinematicsForward(joints: %f %f %f %f %f %f)\n",
            //joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
    }
    // AJ: convert from emc2 coords (XYZABC - which are actually rpy euler
    // angles)
    // to go angles (quaternions)
    pos = haldata->pos;
    rpy.y = world->c * PM_PI / 180;
    rpy.p = world->b * PM_PI / 180;
    rpy.r = world->a * PM_PI / 180;

    go_rpy_quat_convert(&rpy, &pos->rot);
    pos->tran.x = world->tran.x;
    pos->tran.y = world->tran.y;
    pos->tran.z = world->tran.z;

    //pass through unused 678 as uvw
    if (total_joints > 6) world->u = joint[6];
    if (total_joints > 7) world->v = joint[7];
    if (total_joints > 8) world->w = joint[8];

    // pos will be the world location
    // jcopy: joitn position in radians
    ret = genser_kin_fwd(KINS_PTR, jcopy, pos);
    if (ret < 0)
        return ret;

    // AJ: convert back to emc2 coords
    ret = go_quat_rpy_convert(&pos->rot, &rpy);
    if (ret < 0)
        return ret;
    world->tran.x = pos->tran.x;
    world->tran.y = pos->tran.y;
    world->tran.z = pos->tran.z;
    world->a = rpy.r * 180 / PM_PI;
    world->b = rpy.p * 180 / PM_PI;
    world->c = rpy.y * 180 / PM_PI;

    if (changed) {
// rtapi_print("genserKinematicsForward(world: %f %f %f %f %f %f)\n", world->tran.x, world->tran.y, world->tran.z, world->a, world->b, world->c);
    }
    return 0;
}

int genser_kin_fwd(void *kins, const go_real * joints, go_pose * pos)
{
    genser_struct *genser = kins;
    go_link linkout[GENSER_MAX_JOINTS];

    int link;
    int retval;

    genser_kin_init();

    for (link = 0; link < genser->link_num; link++) {
        retval = go_link_joint_set(&genser->links[link], joints[link], &linkout[link]);
        if (GO_RESULT_OK != retval)
            return retval;
    }

    retval = go_link_pose_build(linkout, genser->link_num, pos);
    if (GO_RESULT_OK != retval)
        return retval;

    return GO_RESULT_OK;
}

int genserKinematicsInverse(const EmcPose * world,
                            double *joints,
                            const KINEMATICS_INVERSE_FLAGS * iflags,
                            KINEMATICS_FORWARD_FLAGS * fflags)
{
    (void)iflags;
    (void)fflags;

    genser_struct *genser = KINS_PTR;
    GO_MATRIX_DECLARE(Jfwd, Jfwd_stg, 6, GENSER_MAX_JOINTS);
    GO_MATRIX_DECLARE(Jinv, Jinv_stg, GENSER_MAX_JOINTS, 6);
    go_pose T_L_0;
    go_real dvw[6];
    go_real jest[GENSER_MAX_JOINTS];
    go_real dj[GENSER_MAX_JOINTS];
    go_pose pest, pestinv, Tdelta; // pos = converted pose from EmcPose
    go_rpy rpy;
    go_rvec rvec;
    go_cart cart;
    go_link linkout[GENSER_MAX_JOINTS];
    int link;
    int smalls;
    int retval;

    // rtapi_print("kineInverse(joints: %f %f %f %f %f %f)\n",
    //      joints[0],joints[1],joints[2],joints[3],joints[4],joints[5]);
    // rtapi_print("kineInverse(world: %f %f %f %f %f %f)\n",
    //      world->tran.x, world->tran.y, world->tran.z, world->a, world->b, world->c);

#ifndef ULAPI
    genser_kin_init();
    if (!genser_hal_inited) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "genserKinematicsInverse: not initialized\n");
        return -1;
    }
#endif

    // FIXME-AJ: rpy or zyx ?
    rpy.y = world->c * PM_PI / 180;
    rpy.p = world->b * PM_PI / 180;
    rpy.r = world->a * PM_PI / 180;

    go_rpy_quat_convert(&rpy, &haldata->pos->rot);
    haldata->pos->tran.x = world->tran.x;
    haldata->pos->tran.y = world->tran.y;
    haldata->pos->tran.z = world->tran.z;

    go_matrix_init(Jfwd, Jfwd_stg, 6, genser->link_num);
    go_matrix_init(Jinv, Jinv_stg, genser->link_num, 6);

    /* jest[] is a copy of joints[], which is the joint estimate */
    for (link = 0; link < genser->link_num; link++) {
        // jest, and the rest of joint related calcs are in radians
        jest[link] = joints[link] * (PM_PI / 180);
    }

    for (genser->iterations = 0;
         genser->iterations < *haldata->max_iterations;
         genser->iterations++) {
         *(haldata->last_iterations) = genser->iterations;
        /* update the Jacobians */
        for (link = 0; link < genser->link_num; link++) {
            go_link_joint_set(&genser->links[link], jest[link], &linkout[link]);
        }
        retval = compute_jfwd(linkout, genser->link_num, &Jfwd, &T_L_0);
        if (GO_RESULT_OK != retval) {
            rtapi_print("ERR kI - compute_jfwd (joints: %f %f %f %f %f %f), (iterations=%d)\n",
                 joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
            return retval;
        }
        retval = compute_jinv(&Jfwd, &Jinv);
        if (GO_RESULT_OK != retval) {
            rtapi_print("ERR kI - compute_jinv (joints: %f %f %f %f %f %f), (iterations=%d)\n",
                 joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
            return retval;
        }

        /* pest is the resulting pose estimate given joint estimate */
        genser_kin_fwd(KINS_PTR, jest, &pest);
        //printf("jest: %f %f %f %f %f %f\n",jest[0],jest[1],jest[2],jest[3],jest[4],jest[5]);
        /* pestinv is its inverse */
        go_pose_inv(&pest, &pestinv);
        /*
            Tdelta is the incremental pose from pest to pos, such that

            0        L         0
            . pest *  Tdelta =  pos, or
            L        L         L

            L         L          0
            .Tdelta =  pestinv *  pos
            L         0          L
        */
        go_pose_pose_mult(&pestinv, haldata->pos, &Tdelta);

        /*
            We need Tdelta in 0 frame, not pest frame, so rotate it
            back. Since it's effectively a velocity, we just rotate it, and
            don't translate it.
        */

        /* first rotate the translation differential */
        go_quat_cart_mult(&pest.rot, &Tdelta.tran, &cart);
        dvw[0] = cart.x;
        dvw[1] = cart.y;
        dvw[2] = cart.z;

        /* to rotate the rotation differential, convert it to a
           velocity screw and rotate that */
        go_quat_rvec_convert(&Tdelta.rot, &rvec);
        cart.x = rvec.x;
        cart.y = rvec.y;
        cart.z = rvec.z;
        go_quat_cart_mult(&pest.rot, &cart, &cart);
        dvw[3] = cart.x;
        dvw[4] = cart.y;
        dvw[5] = cart.z;

        /* push the Cartesian velocity vector through the inverse Jacobian */
        go_matrix_vector_mult(&Jinv, dvw, dj);

        //pass through 678 as uvw
        if (total_joints > 6) joints[6] = world->u;
        if (total_joints > 7) joints[7] = world->v;
        if (total_joints > 8) joints[8] = world->w;

        /* check for small joint increments, if so we're done */
        for (link = 0, smalls = 0; link < genser->link_num; link++) {
            if (GO_QUANTITY_LENGTH == linkout[link].quantity) {
            if (GO_TRAN_SMALL(dj[link]))
                smalls++;
            } else {
                if (GO_ROT_SMALL(dj[link]))
                    smalls++;
            }
        }
        if (smalls == genser->link_num) {
            /* converged, copy jest[] out */
            for (link = 0; link < genser->link_num; link++) {
                // convert from radians back to angles
                joints[link] = jest[link] * 180 / PM_PI;
                if ((link) && *(haldata->unrotate[link]))
                    joints[link] += *(haldata->unrotate[link]) * joints[link-1];
            }
            //rtapi_print("DONEkineInverse(joints: %f %f %f %f %f %f), (iterations=%d)\n",
            //     joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
            //rtapi_print("OKkineInverse: %.2f %.2f %.2f %.2f %.2f %.2f)\n",
            //     world->tran.x, world->tran.y, world->tran.z, world->a, world->b, world->c);
            return GO_RESULT_OK;
        }
        /* else keep iterating */
        for (link = 0; link < genser->link_num; link++) {
            jest[link] += dj[link]; //still in radians
        }
    } /* for (iterations) */

    rtapi_print("ERRkineInverse(joints: %f %f %f %f %f %f), (iterations=%d)\n",
         joints[0],joints[1],joints[2],joints[3],joints[4],joints[5], genser->iterations);
    return GO_RESULT_ERROR;
}

/*
  Extras, not callable using go_kin_ wrapper but if you know you have
  linked in these kinematics, go ahead and call these for your ad hoc
  purposes.
*/

int genser_kin_inv_iterations(genser_struct * genser)
{
    return genser->iterations;
}

int genser_kin_inv_set_max_iterations(int i)
{
    if (i <= 0) return GO_RESULT_ERROR;
    *haldata->max_iterations = i;
    return GO_RESULT_OK;
}

int genser_kin_inv_get_max_iterations()
{
    return *haldata->max_iterations;
}

int genserKinematicsSetup(const int comp_id,
                    const char* coordinates,
                    kparms*     kp)
{
    (void)coordinates;
    int i,res=0;
    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata) {goto error;}

    // allow for pass through joints 6,7,8 u,v,w
    total_joints = kp->max_joints;

    // only the first 6 joints have A,ALPHA,D,unrotate pins
    for (i = 0; i < 6; i++) {
        res += hal_pin_float_newf(HAL_IN, &(haldata->a[i]), comp_id,
               "%s.A-%d", kp->halprefix, i);
        *(haldata->a[i])=0;
        res += hal_pin_float_newf(HAL_IN, &(haldata->alpha[i]), comp_id,
               "%s.ALPHA-%d", kp->halprefix, i);
        *(haldata->alpha[i])=0;
        res += hal_pin_float_newf(HAL_IN, &(haldata->d[i]), comp_id,
               "%s.D-%d", kp->halprefix, i);
        *(haldata->d[i])=0;
        res += hal_pin_s32_newf(HAL_IN, &(haldata->unrotate[i]), comp_id,
              "%s.unrotate-%d", kp->halprefix, i);
        *haldata->unrotate[i]=0;
    }
    res += hal_pin_u32_newf(HAL_OUT, &(haldata->last_iterations), comp_id,
          "%s.last-iterations",kp->halprefix);

    KINS_PTR = hal_malloc(sizeof(genser_struct));
    haldata->pos = (go_pose *) hal_malloc(sizeof(go_pose));
    if (KINS_PTR     == NULL) {goto error;}
    if (haldata->pos == NULL) {goto error;}
    res += hal_pin_u32_newf(HAL_IN, &haldata->max_iterations, comp_id,
          "%s.max-iterations",kp->halprefix);

    if (res) {goto error;}

    *haldata->max_iterations = GENSER_DEFAULT_MAX_ITERATIONS;

    A(0) = DEFAULT_A1;
    A(1) = DEFAULT_A2;
    A(2) = DEFAULT_A3;
    A(3) = DEFAULT_A4;
    A(4) = DEFAULT_A5;
    A(5) = DEFAULT_A6;
    ALPHA(0) = DEFAULT_ALPHA1;
    ALPHA(1) = DEFAULT_ALPHA2;
    ALPHA(2) = DEFAULT_ALPHA3;
    ALPHA(3) = DEFAULT_ALPHA4;
    ALPHA(4) = DEFAULT_ALPHA5;
    ALPHA(5) = DEFAULT_ALPHA6;
    D(0) = DEFAULT_D1;
    D(1) = DEFAULT_D2;
    D(2) = DEFAULT_D3;
    D(3) = DEFAULT_D4;
    D(4) = DEFAULT_D5;
    D(5) = DEFAULT_D6;

    genser_hal_inited = 1;
    return 0;

error:
    return -1;
} // genserKinematicsSetup()
