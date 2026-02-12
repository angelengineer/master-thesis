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

#ifndef cart_pole_MODEL
#define cart_pole_MODEL

#ifdef __cplusplus
extern "C" {
#endif


/* GNSF Functions */
    
// phi_fun
int cart_pole_gnsf_phi_fun(const double** arg, double** res, int* iw, double* w, void *mem);
int cart_pole_gnsf_phi_fun_work(int *, int *, int *, int *);
const int *cart_pole_gnsf_phi_fun_sparsity_in(int);
const int *cart_pole_gnsf_phi_fun_sparsity_out(int);
int cart_pole_gnsf_phi_fun_n_in(void);
int cart_pole_gnsf_phi_fun_n_out(void);

// phi_fun_jac_y
int cart_pole_gnsf_phi_fun_jac_y(const double** arg, double** res, int* iw, double* w, void *mem);
int cart_pole_gnsf_phi_fun_jac_y_work(int *, int *, int *, int *);
const int *cart_pole_gnsf_phi_fun_jac_y_sparsity_in(int);
const int *cart_pole_gnsf_phi_fun_jac_y_sparsity_out(int);
int cart_pole_gnsf_phi_fun_jac_y_n_in(void);
int cart_pole_gnsf_phi_fun_jac_y_n_out(void);

// phi_jac_y_uhat
int cart_pole_gnsf_phi_jac_y_uhat(const double** arg, double** res, int* iw, double* w, void *mem);
int cart_pole_gnsf_phi_jac_y_uhat_work(int *, int *, int *, int *);
const int *cart_pole_gnsf_phi_jac_y_uhat_sparsity_in(int);
const int *cart_pole_gnsf_phi_jac_y_uhat_sparsity_out(int);
int cart_pole_gnsf_phi_jac_y_uhat_n_in(void);
int cart_pole_gnsf_phi_jac_y_uhat_n_out(void);
    
// f_lo_fun_jac_x1k1uz
int cart_pole_gnsf_f_lo_fun_jac_x1k1uz(const double** arg, double** res, int* iw, double* w, void *mem);
int cart_pole_gnsf_f_lo_fun_jac_x1k1uz_work(int *, int *, int *, int *);
const int *cart_pole_gnsf_f_lo_fun_jac_x1k1uz_sparsity_in(int);
const int *cart_pole_gnsf_f_lo_fun_jac_x1k1uz_sparsity_out(int);
int cart_pole_gnsf_f_lo_fun_jac_x1k1uz_n_in(void);
int cart_pole_gnsf_f_lo_fun_jac_x1k1uz_n_out(void);
// used to import model matrices
int cart_pole_gnsf_get_matrices_fun(const double** arg, double** res, int* iw, double* w, void *mem);
int cart_pole_gnsf_get_matrices_fun_work(int *, int *, int *, int *);
const int *cart_pole_gnsf_get_matrices_fun_sparsity_in(int);
const int *cart_pole_gnsf_get_matrices_fun_sparsity_out(int);
int cart_pole_gnsf_get_matrices_fun_n_in(void);
int cart_pole_gnsf_get_matrices_fun_n_out(void);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // cart_pole_MODEL
