/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */
// standard
#include <stdio.h>
#include <stdlib.h>

// acados
#include "acados_c/external_function_interface.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#include "acados/sim/sim_common.h"
#include "acados/utils/external_function_generic.h"
#include "acados/utils/print.h"


// example specific
#include "model_simplified_model/model_simplified_model.h"
#include "acados_sim_solver_model_simplified.h"


// ** solver data **

model_simplified_sim_solver_capsule * model_simplified_acados_sim_solver_create_capsule()
{
    void* capsule_mem = malloc(sizeof(model_simplified_sim_solver_capsule));
    model_simplified_sim_solver_capsule *capsule = (model_simplified_sim_solver_capsule *) capsule_mem;

    return capsule;
}


int model_simplified_acados_sim_solver_free_capsule(model_simplified_sim_solver_capsule * capsule)
{
    free(capsule);
    return 0;
}


int model_simplified_acados_sim_create(model_simplified_sim_solver_capsule * capsule)
{
    // initialize
    const int nx = MODEL_SIMPLIFIED_NX;
    const int nu = MODEL_SIMPLIFIED_NU;
    const int nz = MODEL_SIMPLIFIED_NZ;
    const int np = MODEL_SIMPLIFIED_NP;
    bool tmp_bool;

    double Tsim = 0.025;

    capsule->acados_sim_mem = NULL;

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);
    ext_fun_opts.external_workspace = false;

    
    capsule->sim_gnsf_phi_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_gnsf_phi_fun_jac_y = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_gnsf_phi_jac_y_uhat = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
  
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));
    capsule->sim_gnsf_get_matrices_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi));

  
    capsule->sim_gnsf_phi_fun->casadi_fun = &model_simplified_gnsf_phi_fun;
    capsule->sim_gnsf_phi_fun->casadi_n_in = &model_simplified_gnsf_phi_fun_n_in;
    capsule->sim_gnsf_phi_fun->casadi_n_out = &model_simplified_gnsf_phi_fun_n_out;
    capsule->sim_gnsf_phi_fun->casadi_sparsity_in = &model_simplified_gnsf_phi_fun_sparsity_in;
    capsule->sim_gnsf_phi_fun->casadi_sparsity_out = &model_simplified_gnsf_phi_fun_sparsity_out;
    capsule->sim_gnsf_phi_fun->casadi_work = &model_simplified_gnsf_phi_fun_work;
    external_function_param_casadi_create(capsule->sim_gnsf_phi_fun, np, &ext_fun_opts);

    capsule->sim_gnsf_phi_fun_jac_y->casadi_fun = &model_simplified_gnsf_phi_fun_jac_y;
    capsule->sim_gnsf_phi_fun_jac_y->casadi_n_in = &model_simplified_gnsf_phi_fun_jac_y_n_in;
    capsule->sim_gnsf_phi_fun_jac_y->casadi_n_out = &model_simplified_gnsf_phi_fun_jac_y_n_out;
    capsule->sim_gnsf_phi_fun_jac_y->casadi_sparsity_in = &model_simplified_gnsf_phi_fun_jac_y_sparsity_in;
    capsule->sim_gnsf_phi_fun_jac_y->casadi_sparsity_out = &model_simplified_gnsf_phi_fun_jac_y_sparsity_out;
    capsule->sim_gnsf_phi_fun_jac_y->casadi_work = &model_simplified_gnsf_phi_fun_jac_y_work;
    external_function_param_casadi_create(capsule->sim_gnsf_phi_fun_jac_y, np, &ext_fun_opts);

    capsule->sim_gnsf_phi_jac_y_uhat->casadi_fun = &model_simplified_gnsf_phi_jac_y_uhat;
    capsule->sim_gnsf_phi_jac_y_uhat->casadi_n_in = &model_simplified_gnsf_phi_jac_y_uhat_n_in;
    capsule->sim_gnsf_phi_jac_y_uhat->casadi_n_out = &model_simplified_gnsf_phi_jac_y_uhat_n_out;
    capsule->sim_gnsf_phi_jac_y_uhat->casadi_sparsity_in = &model_simplified_gnsf_phi_jac_y_uhat_sparsity_in;
    capsule->sim_gnsf_phi_jac_y_uhat->casadi_sparsity_out = &model_simplified_gnsf_phi_jac_y_uhat_sparsity_out;
    capsule->sim_gnsf_phi_jac_y_uhat->casadi_work = &model_simplified_gnsf_phi_jac_y_uhat_work;
    external_function_param_casadi_create(capsule->sim_gnsf_phi_jac_y_uhat, np, &ext_fun_opts);

  
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z->casadi_fun = &model_simplified_gnsf_f_lo_fun_jac_x1k1uz;
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z->casadi_n_in = &model_simplified_gnsf_f_lo_fun_jac_x1k1uz_n_in;
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z->casadi_n_out = &model_simplified_gnsf_f_lo_fun_jac_x1k1uz_n_out;
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z->casadi_sparsity_in = &model_simplified_gnsf_f_lo_fun_jac_x1k1uz_sparsity_in;
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z->casadi_sparsity_out = &model_simplified_gnsf_f_lo_fun_jac_x1k1uz_sparsity_out;
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z->casadi_work = &model_simplified_gnsf_f_lo_fun_jac_x1k1uz_work;
    external_function_param_casadi_create(capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z, np, &ext_fun_opts);

    capsule->sim_gnsf_get_matrices_fun->casadi_fun = &model_simplified_gnsf_get_matrices_fun;
    capsule->sim_gnsf_get_matrices_fun->casadi_n_in = &model_simplified_gnsf_get_matrices_fun_n_in;
    capsule->sim_gnsf_get_matrices_fun->casadi_n_out = &model_simplified_gnsf_get_matrices_fun_n_out;
    capsule->sim_gnsf_get_matrices_fun->casadi_sparsity_in = &model_simplified_gnsf_get_matrices_fun_sparsity_in;
    capsule->sim_gnsf_get_matrices_fun->casadi_sparsity_out = &model_simplified_gnsf_get_matrices_fun_sparsity_out;
    capsule->sim_gnsf_get_matrices_fun->casadi_work = &model_simplified_gnsf_get_matrices_fun_work;
    external_function_param_casadi_create(capsule->sim_gnsf_get_matrices_fun, np, &ext_fun_opts);
    

    // sim plan & config
    sim_solver_plan_t plan;
    plan.sim_solver = GNSF;

    // create correct config based on plan
    sim_config * model_simplified_sim_config = sim_config_create(plan);
    capsule->acados_sim_config = model_simplified_sim_config;

    // sim dims
    void *model_simplified_sim_dims = sim_dims_create(model_simplified_sim_config);
    capsule->acados_sim_dims = model_simplified_sim_dims;
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nx", &nx);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nu", &nu);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nz", &nz);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "np", &np);

    int gnsf_nx1 = 3;
    int gnsf_nz1 = 0;
    int gnsf_nout = 2;
    int gnsf_ny = 3;
    int gnsf_nuhat = 2;

    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nx1", &gnsf_nx1);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nz1", &gnsf_nz1);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nout", &gnsf_nout);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "ny", &gnsf_ny);
    sim_dims_set(model_simplified_sim_config, model_simplified_sim_dims, "nuhat", &gnsf_nuhat);


    // sim opts
    sim_opts *model_simplified_sim_opts = sim_opts_create(model_simplified_sim_config, model_simplified_sim_dims);
    capsule->acados_sim_opts = model_simplified_sim_opts;
    int tmp_int = 3;
    sim_opts_set(model_simplified_sim_config, model_simplified_sim_opts, "newton_iter", &tmp_int);
    double tmp_double = 0;
    sim_opts_set(model_simplified_sim_config, model_simplified_sim_opts, "newton_tol", &tmp_double);
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    sim_opts_set(model_simplified_sim_config, model_simplified_sim_opts, "collocation_type", &collocation_type);

 
    tmp_int = 4;
    sim_opts_set(model_simplified_sim_config, model_simplified_sim_opts, "num_stages", &tmp_int);
    tmp_int = 1;
    sim_opts_set(model_simplified_sim_config, model_simplified_sim_opts, "num_steps", &tmp_int);
    tmp_bool = 0;
    sim_opts_set(model_simplified_sim_config, model_simplified_sim_opts, "jac_reuse", &tmp_bool);


    // sim in / out
    sim_in *model_simplified_sim_in = sim_in_create(model_simplified_sim_config, model_simplified_sim_dims);
    capsule->acados_sim_in = model_simplified_sim_in;
    sim_out *model_simplified_sim_out = sim_out_create(model_simplified_sim_config, model_simplified_sim_dims);
    capsule->acados_sim_out = model_simplified_sim_out;

    sim_in_set(model_simplified_sim_config, model_simplified_sim_dims,
               model_simplified_sim_in, "T", &Tsim);

    // model functions
  
    model_simplified_sim_config->model_set(model_simplified_sim_in->model,
                 "phi_fun", capsule->sim_gnsf_phi_fun);
    model_simplified_sim_config->model_set(model_simplified_sim_in->model,
                 "phi_fun_jac_y", capsule->sim_gnsf_phi_fun_jac_y);
    model_simplified_sim_config->model_set(model_simplified_sim_in->model,
                 "phi_jac_y_uhat", capsule->sim_gnsf_phi_jac_y_uhat);
  
    model_simplified_sim_config->model_set(model_simplified_sim_in->model,
                 "f_lo_jac_x1_x1dot_u_z", capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z);
    model_simplified_sim_config->model_set(model_simplified_sim_in->model,
                 "gnsf_get_matrices_fun", capsule->sim_gnsf_get_matrices_fun);

    // sim solver
    sim_solver *model_simplified_sim_solver = sim_solver_create(model_simplified_sim_config,
                                               model_simplified_sim_dims, model_simplified_sim_opts, model_simplified_sim_in);
    capsule->acados_sim_solver = model_simplified_sim_solver;

    capsule->acados_sim_mem = model_simplified_sim_solver->mem;


    /* initialize parameter values */
    double* p = calloc(np, sizeof(double));
    
    p[0] = 5;
    p[1] = 1.25;
    p[2] = -0.0232;
    p[3] = 0.875;

    model_simplified_acados_sim_update_params(capsule, p, np);
    free(p);


    /* initialize input */
    // x
    double x0[4];
    for (int ii = 0; ii < 4; ii++)
        x0[ii] = 0.0;

    sim_in_set(model_simplified_sim_config, model_simplified_sim_dims,
               model_simplified_sim_in, "x", x0);


    // u
    double u0[2];
    for (int ii = 0; ii < 2; ii++)
        u0[ii] = 0.0;

    sim_in_set(model_simplified_sim_config, model_simplified_sim_dims,
               model_simplified_sim_in, "u", u0);

    // S_forw
    double S_forw[24];
    for (int ii = 0; ii < 24; ii++)
        S_forw[ii] = 0.0;
    for (int ii = 0; ii < 4; ii++)
        S_forw[ii + ii * 4 ] = 1.0;


    sim_in_set(model_simplified_sim_config, model_simplified_sim_dims,
               model_simplified_sim_in, "S_forw", S_forw);

    int status = sim_precompute(model_simplified_sim_solver, model_simplified_sim_in, model_simplified_sim_out);

    return status;
}


int model_simplified_acados_sim_solve(model_simplified_sim_solver_capsule *capsule)
{
    // integrate dynamics using acados sim_solver
    int status = sim_solve(capsule->acados_sim_solver,
                           capsule->acados_sim_in, capsule->acados_sim_out);
    if (status != 0)
        printf("error in model_simplified_acados_sim_solve()! Exiting.\n");

    return status;
}




int model_simplified_acados_sim_free(model_simplified_sim_solver_capsule *capsule)
{
    // free memory
    sim_solver_destroy(capsule->acados_sim_solver);
    sim_in_destroy(capsule->acados_sim_in);
    sim_out_destroy(capsule->acados_sim_out);
    sim_opts_destroy(capsule->acados_sim_opts);
    sim_dims_destroy(capsule->acados_sim_dims);
    sim_config_destroy(capsule->acados_sim_config);

    // free external function
  
    external_function_param_casadi_free(capsule->sim_gnsf_phi_fun);
    external_function_param_casadi_free(capsule->sim_gnsf_phi_fun_jac_y);
    external_function_param_casadi_free(capsule->sim_gnsf_phi_jac_y_uhat);
    free(capsule->sim_gnsf_phi_fun);
    free(capsule->sim_gnsf_phi_fun_jac_y);
    free(capsule->sim_gnsf_phi_jac_y_uhat);
  
    external_function_param_casadi_free(capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z);
    free(capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z);
    external_function_param_casadi_free(capsule->sim_gnsf_get_matrices_fun);
    free(capsule->sim_gnsf_get_matrices_fun);


    return 0;
}


int model_simplified_acados_sim_update_params(model_simplified_sim_solver_capsule *capsule, double *p, int np)
{
    int status = 0;
    int casadi_np = MODEL_SIMPLIFIED_NP;

    if (casadi_np != np) {
        printf("model_simplified_acados_sim_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
  
    capsule->sim_gnsf_phi_fun[0].set_param(capsule->sim_gnsf_phi_fun, p);
    capsule->sim_gnsf_phi_fun_jac_y[0].set_param(capsule->sim_gnsf_phi_fun_jac_y, p);
    capsule->sim_gnsf_phi_jac_y_uhat[0].set_param(capsule->sim_gnsf_phi_jac_y_uhat, p);
  
    capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z[0].set_param(capsule->sim_gnsf_f_lo_jac_x1_x1dot_u_z, p);
    capsule->sim_gnsf_get_matrices_fun[0].set_param(capsule->sim_gnsf_get_matrices_fun, p);


    return status;
}

/* getters pointers to C objects*/
sim_config * model_simplified_acados_get_sim_config(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_config;
};

sim_in * model_simplified_acados_get_sim_in(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_in;
};

sim_out * model_simplified_acados_get_sim_out(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_out;
};

void * model_simplified_acados_get_sim_dims(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_dims;
};

sim_opts * model_simplified_acados_get_sim_opts(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_opts;
};

sim_solver  * model_simplified_acados_get_sim_solver(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_solver;
};

void * model_simplified_acados_get_sim_mem(model_simplified_sim_solver_capsule *capsule)
{
    return capsule->acados_sim_mem;
};

