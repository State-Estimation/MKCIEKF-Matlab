/* Include files */

#include "RIEKF_test_sfun.h"
#include "c2_RIEKF_test.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "RIEKF_test_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[7] = { "encoder_noise_std", "nargin",
  "nargout", "encoder", "contact", "encoder_n", "encoder_noise" };

/* Function Declarations */
static void initialize_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance);
static void initialize_params_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance);
static void enable_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance);
static void disable_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *
  chartInstance);
static void set_sim_state_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void finalize_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance);
static void sf_gateway_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance);
static void mdl_start_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance);
static void initSimStructsc2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_b_encoder_noise, const char_T *c2_identifier, real_T c2_y[14]);
static void c2_b_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[14]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_c_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_error(SFc2_RIEKF_testInstanceStruct *chartInstance);
static real_T c2_randn(SFc2_RIEKF_testInstanceStruct *chartInstance);
static void c2_genrandu(SFc2_RIEKF_testInstanceStruct *chartInstance, uint32_T
  c2_s, uint32_T *c2_e_state, real_T *c2_r);
static void c2_eml_rand_shr3cong(SFc2_RIEKF_testInstanceStruct *chartInstance,
  uint32_T c2_e_state[2], uint32_T c2_f_state[2], real_T *c2_r);
static void c2_eml_rand_mt19937ar(SFc2_RIEKF_testInstanceStruct *chartInstance,
  uint32_T c2_e_state[625], uint32_T c2_f_state[625], real_T *c2_r);
static void c2_genrand_uint32_vector(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_mt[625], uint32_T c2_b_mt[625], uint32_T c2_u[2]);
static void c2_b_genrandu(SFc2_RIEKF_testInstanceStruct *chartInstance, uint32_T
  c2_mt[625], uint32_T c2_b_mt[625], real_T *c2_r);
static void c2_b_error(SFc2_RIEKF_testInstanceStruct *chartInstance);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_d_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint32_T c2_e_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_c_method, const char_T *c2_identifier,
  boolean_T *c2_svPtr);
static uint32_T c2_f_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T *c2_svPtr);
static void c2_g_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_e_state, const char_T *c2_identifier, boolean_T *c2_svPtr,
  uint32_T c2_y[625]);
static void c2_h_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, boolean_T
  *c2_svPtr, uint32_T c2_y[625]);
static void c2_i_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_e_state, const char_T *c2_identifier, boolean_T *c2_svPtr,
  uint32_T c2_y[2]);
static void c2_j_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, boolean_T
  *c2_svPtr, uint32_T c2_y[2]);
static uint8_T c2_k_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_RIEKF_test, const char_T
  *c2_identifier);
static uint8_T c2_l_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static real_T c2_b_eml_rand_shr3cong(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_e_state[2]);
static real_T c2_b_eml_rand_mt19937ar(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_e_state[625]);
static void c2_b_genrand_uint32_vector(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_mt[625], uint32_T c2_u[2]);
static real_T c2_c_genrandu(SFc2_RIEKF_testInstanceStruct *chartInstance,
  uint32_T c2_mt[625]);
static void init_dsm_address_info(SFc2_RIEKF_testInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc2_RIEKF_testInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_RIEKF_test(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_method_not_empty = false;
  chartInstance->c2_state_not_empty = false;
  chartInstance->c2_b_method_not_empty = false;
  chartInstance->c2_b_state_not_empty = false;
  chartInstance->c2_c_state_not_empty = false;
  chartInstance->c2_d_state_not_empty = false;
  chartInstance->c2_is_active_c2_RIEKF_test = 0U;
}

static void initialize_params_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *
  chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  uint32_T c2_hoistedGlobal;
  const mxArray *c2_d_y = NULL;
  uint32_T c2_b_hoistedGlobal;
  const mxArray *c2_e_y = NULL;
  uint32_T c2_c_hoistedGlobal;
  const mxArray *c2_f_y = NULL;
  const mxArray *c2_g_y = NULL;
  const mxArray *c2_h_y = NULL;
  const mxArray *c2_i_y = NULL;
  uint8_T c2_d_hoistedGlobal;
  const mxArray *c2_j_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(9, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", *chartInstance->c2_encoder_n, 0, 0U,
    1U, 0U, 1, 14), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", *chartInstance->c2_encoder_noise, 0,
    0U, 1U, 0U, 1, 14), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_hoistedGlobal = chartInstance->c2_b_method;
  c2_d_y = NULL;
  if (!chartInstance->c2_b_method_not_empty) {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_hoistedGlobal, 7, 0U, 0U, 0U,
      0), false);
  }

  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_b_hoistedGlobal = chartInstance->c2_method;
  c2_e_y = NULL;
  if (!chartInstance->c2_b_method_not_empty) {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_b_hoistedGlobal, 7, 0U, 0U, 0U,
      0), false);
  }

  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_c_hoistedGlobal = chartInstance->c2_b_state;
  c2_f_y = NULL;
  if (!chartInstance->c2_b_method_not_empty) {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_c_hoistedGlobal, 7, 0U, 0U, 0U,
      0), false);
  }

  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_g_y = NULL;
  if (!chartInstance->c2_d_state_not_empty) {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_g_y, sf_mex_create("y", chartInstance->c2_d_state, 7, 0U,
      1U, 0U, 1, 625), false);
  }

  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_h_y = NULL;
  if (!chartInstance->c2_c_state_not_empty) {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_h_y, sf_mex_create("y", chartInstance->c2_c_state, 7, 0U,
      1U, 0U, 1, 2), false);
  }

  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_i_y = NULL;
  if (!chartInstance->c2_c_state_not_empty) {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c2_i_y, sf_mex_create("y", chartInstance->c2_state, 7, 0U, 1U,
      0U, 1, 2), false);
  }

  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_d_hoistedGlobal = chartInstance->c2_is_active_c2_RIEKF_test;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_d_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c2_y, 8, c2_j_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[14];
  int32_T c2_i0;
  real_T c2_dv1[14];
  int32_T c2_i1;
  uint32_T c2_uv0[625];
  int32_T c2_i2;
  uint32_T c2_uv1[2];
  int32_T c2_i3;
  uint32_T c2_uv2[2];
  int32_T c2_i4;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                      "encoder_n", c2_dv0);
  for (c2_i0 = 0; c2_i0 < 14; c2_i0++) {
    (*chartInstance->c2_encoder_n)[c2_i0] = c2_dv0[c2_i0];
  }

  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                      "encoder_noise", c2_dv1);
  for (c2_i1 = 0; c2_i1 < 14; c2_i1++) {
    (*chartInstance->c2_encoder_noise)[c2_i1] = c2_dv1[c2_i1];
  }

  chartInstance->c2_b_method = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 2)), "method", &chartInstance->c2_b_method_not_empty);
  chartInstance->c2_method = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 3)), "method", &chartInstance->c2_method_not_empty);
  chartInstance->c2_b_state = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 4)), "state", &chartInstance->c2_b_state_not_empty);
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 5)),
                        "state", &chartInstance->c2_d_state_not_empty, c2_uv0);
  for (c2_i2 = 0; c2_i2 < 625; c2_i2++) {
    chartInstance->c2_d_state[c2_i2] = c2_uv0[c2_i2];
  }

  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 6)),
                        "state", &chartInstance->c2_c_state_not_empty, c2_uv1);
  for (c2_i3 = 0; c2_i3 < 2; c2_i3++) {
    chartInstance->c2_c_state[c2_i3] = c2_uv1[c2_i3];
  }

  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 7)),
                        "state", &chartInstance->c2_state_not_empty, c2_uv2);
  for (c2_i4 = 0; c2_i4 < 2; c2_i4++) {
    chartInstance->c2_state[c2_i4] = c2_uv2[c2_i4];
  }

  chartInstance->c2_is_active_c2_RIEKF_test = c2_k_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 8)),
     "is_active_c2_RIEKF_test");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_RIEKF_test(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance)
{
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  real_T c2_b_encoder[14];
  uint32_T c2_debug_family_var_map[7];
  real_T c2_b_contact[2];
  real_T c2_encoder_noise_std[14];
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 2.0;
  real_T c2_b_encoder_n[14];
  real_T c2_b_encoder_noise[14];
  int32_T c2_i9;
  real_T c2_x;
  boolean_T c2_b0;
  int32_T c2_i10;
  real_T c2_d0;
  int32_T c2_i11;
  int32_T c2_i12;
  int32_T c2_i13;
  int32_T c2_i14;
  int32_T c2_i15;
  int32_T c2_i16;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i5 = 0; c2_i5 < 2; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_contact)[c2_i5], 1U);
  }

  for (c2_i6 = 0; c2_i6 < 14; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_encoder)[c2_i6], 0U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i7 = 0; c2_i7 < 14; c2_i7++) {
    c2_b_encoder[c2_i7] = (*chartInstance->c2_encoder)[c2_i7];
  }

  for (c2_i8 = 0; c2_i8 < 2; c2_i8++) {
    c2_b_contact[c2_i8] = (*chartInstance->c2_contact)[c2_i8];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_encoder_noise_std, 0U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_c_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_encoder, 3U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_b_contact, 4U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_encoder_n, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_encoder_noise, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  for (c2_i9 = 0; c2_i9 < 14; c2_i9++) {
    c2_encoder_noise_std[c2_i9] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  if (c2_b_contact[1] > 0.1) {
    c2_x = c2_b_contact[1];
    if (muDoubleScalarIsNaN(c2_x)) {
      c2_error(chartInstance);
    }

    if (c2_b_contact[1] != 0.0) {
      c2_b0 = true;
    } else {
      c2_b0 = false;
    }
  } else {
    c2_b0 = false;
  }

  if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, (real_T)c2_b0, 0.9, -1,
        2U, (real_T)c2_b0 < 0.9)) || (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U,
         0U, 1, c2_b_contact[0], 0.1, -1, 4U, c2_b_contact[0] > 0.1)) &&
       CV_EML_COND(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c2_b_contact[0], 0.9,
         -1, 2U, c2_b_contact[0] < 0.9)))) {
    CV_EML_MCDC(0, 1, 0, true);
    CV_EML_IF(0, 1, 0, true);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
    for (c2_i10 = 0; c2_i10 < 14; c2_i10++) {
      c2_encoder_noise_std[c2_i10] = 0.087266462599716474;
    }
  } else {
    CV_EML_MCDC(0, 1, 0, false);
    CV_EML_IF(0, 1, 0, false);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_d0 = c2_randn(chartInstance);
  for (c2_i11 = 0; c2_i11 < 14; c2_i11++) {
    c2_b_encoder_noise[c2_i11] = c2_encoder_noise_std[c2_i11] * c2_d0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  for (c2_i12 = 0; c2_i12 < 14; c2_i12++) {
    c2_b_encoder_n[c2_i12] = c2_b_encoder[c2_i12] + c2_b_encoder_noise[c2_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -8);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i13 = 0; c2_i13 < 14; c2_i13++) {
    (*chartInstance->c2_encoder_n)[c2_i13] = c2_b_encoder_n[c2_i13];
  }

  for (c2_i14 = 0; c2_i14 < 14; c2_i14++) {
    (*chartInstance->c2_encoder_noise)[c2_i14] = c2_b_encoder_noise[c2_i14];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_RIEKF_testMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c2_i15 = 0; c2_i15 < 14; c2_i15++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_encoder_n)[c2_i15], 2U);
  }

  for (c2_i16 = 0; c2_i16 < 14; c2_i16++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c2_encoder_noise)[c2_i16], 3U);
  }
}

static void mdl_start_c2_RIEKF_test(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  sim_mode_is_external(chartInstance->S);
}

static void initSimStructsc2_RIEKF_test(SFc2_RIEKF_testInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)(c2_machineNumber);
  (void)(c2_chartNumber);
  (void)(c2_instanceNumber);
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i17;
  const mxArray *c2_y = NULL;
  real_T c2_u[14];
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i17 = 0; c2_i17 < 14; c2_i17++) {
    c2_u[c2_i17] = (*(real_T (*)[14])c2_inData)[c2_i17];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 14), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_b_encoder_noise, const char_T *c2_identifier, real_T c2_y[14])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_encoder_noise),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_encoder_noise);
}

static void c2_b_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[14])
{
  real_T c2_dv2[14];
  int32_T c2_i18;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv2, 1, 0, 0U, 1, 0U, 1, 14);
  for (c2_i18 = 0; c2_i18 < 14; c2_i18++) {
    c2_y[c2_i18] = c2_dv2[c2_i18];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_encoder_noise;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[14];
  int32_T c2_i19;
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_b_encoder_noise = sf_mex_dup(c2_mxArrayInData);
  c2_thisId.fIdentifier = (const char *)c2_varName;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_encoder_noise),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_encoder_noise);
  for (c2_i19 = 0; c2_i19 < 14; c2_i19++) {
    (*(real_T (*)[14])c2_outData)[c2_i19] = c2_y[c2_i19];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i20;
  const mxArray *c2_y = NULL;
  real_T c2_u[2];
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i20 = 0; c2_i20 < 2; c2_i20++) {
    c2_u[c2_i20] = (*(real_T (*)[2])c2_inData)[c2_i20];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_c_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d1;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d1, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_thisId.fIdentifier = (const char *)c2_varName;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_RIEKF_test_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c2_nameCaptureInfo;
}

static void c2_error(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_cv0[19] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'n', 'o', 'l',
    'o', 'g', 'i', 'c', 'a', 'l', 'n', 'a', 'n' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv0, 10, 0U, 1U, 0U, 2, 1, 19),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static real_T c2_randn(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  real_T c2_r;
  int32_T c2_i21;
  real_T c2_d2;
  uint32_T c2_e_state;
  uint32_T c2_f_state;
  real_T c2_b_r;
  uint32_T c2_g_state;
  real_T c2_t;
  int32_T c2_i22;
  int32_T c2_i23;
  uint32_T c2_h_state;
  real_T c2_c_r;
  real_T c2_d3;
  real_T c2_d4;
  real_T c2_b_t;
  uint32_T c2_i_state;
  real_T c2_d_r;
  uint32_T c2_e_r;
  uint32_T c2_j_state;
  real_T c2_c_t;
  int32_T c2_mti;
  real_T c2_f_r;
  real_T c2_d_t;
  real_T c2_d5;
  uint32_T c2_u0;
  if (!chartInstance->c2_method_not_empty) {
    chartInstance->c2_method = 0U;
    chartInstance->c2_method_not_empty = true;
    for (c2_i21 = 0; c2_i21 < 2; c2_i21++) {
      chartInstance->c2_state[c2_i21] = 362436069U + (uint32_T)(-362436069 *
        c2_i21);
    }

    if ((real_T)chartInstance->c2_state[1] == 0.0) {
      chartInstance->c2_state[1] = 521288629U;
    }

    chartInstance->c2_state_not_empty = true;
  }

  if (chartInstance->c2_method == 0U) {
    if (!chartInstance->c2_b_method_not_empty) {
      chartInstance->c2_b_method = 7U;
      chartInstance->c2_b_method_not_empty = true;
    }

    if (chartInstance->c2_b_method == 4U) {
      if (!chartInstance->c2_b_state_not_empty) {
        chartInstance->c2_b_state = 1144108930U;
        chartInstance->c2_b_state_not_empty = true;
      }

      c2_h_state = chartInstance->c2_b_state;
      do {
        c2_genrandu(chartInstance, c2_h_state, &c2_i_state, &c2_d_r);
        c2_genrandu(chartInstance, c2_i_state, &c2_j_state, &c2_c_t);
        c2_h_state = c2_j_state;
        c2_f_r = 2.0 * c2_d_r - 1.0;
        c2_d_t = 2.0 * c2_c_t - 1.0;
        c2_d_t = c2_d_t * c2_d_t + c2_f_r * c2_f_r;
      } while (!(c2_d_t <= 1.0));

      c2_f_r *= muDoubleScalarSqrt(-2.0 * muDoubleScalarLog(c2_d_t) / c2_d_t);
      chartInstance->c2_b_state = c2_j_state;
      c2_r = c2_f_r;
    } else if (chartInstance->c2_b_method == 5U) {
      if (!chartInstance->c2_c_state_not_empty) {
        for (c2_i23 = 0; c2_i23 < 2; c2_i23++) {
          chartInstance->c2_c_state[c2_i23] = 362436069U + 158852560U *
            (uint32_T)c2_i23;
        }

        chartInstance->c2_c_state_not_empty = true;
      }

      c2_d4 = c2_b_eml_rand_shr3cong(chartInstance, chartInstance->c2_c_state);
      c2_r = c2_d4;
    } else {
      if (!chartInstance->c2_d_state_not_empty) {
        for (c2_i22 = 0; c2_i22 < 625; c2_i22++) {
          chartInstance->c2_d_state[c2_i22] = 0U;
        }

        c2_e_r = 5489U;
        chartInstance->c2_d_state[0] = 5489U;
        for (c2_mti = 0; c2_mti < 623; c2_mti++) {
          c2_d5 = muDoubleScalarRound((2.0 + (real_T)c2_mti) - 1.0);
          if (c2_d5 < 4.294967296E+9) {
            if (c2_d5 >= 0.0) {
              c2_u0 = (uint32_T)c2_d5;
            } else {
              c2_u0 = 0U;
              _SFD_OVERFLOW_DETECTION(SFDB_SATURATE, 1U, 0U, 0U);
            }
          } else if (c2_d5 >= 4.294967296E+9) {
            c2_u0 = MAX_uint32_T;
            _SFD_OVERFLOW_DETECTION(SFDB_SATURATE, 1U, 0U, 0U);
          } else {
            c2_u0 = 0U;
          }

          c2_e_r = (c2_e_r ^ c2_e_r >> 30U) * 1812433253U + c2_u0;
          chartInstance->c2_d_state[c2_mti + 1] = c2_e_r;
        }

        chartInstance->c2_d_state[624] = 624U;
        chartInstance->c2_d_state_not_empty = true;
      }

      c2_d3 = c2_b_eml_rand_mt19937ar(chartInstance, chartInstance->c2_d_state);
      c2_r = c2_d3;
    }
  } else if (chartInstance->c2_method == 4U) {
    c2_e_state = chartInstance->c2_state[0];
    do {
      c2_genrandu(chartInstance, c2_e_state, &c2_f_state, &c2_b_r);
      c2_genrandu(chartInstance, c2_f_state, &c2_g_state, &c2_t);
      c2_e_state = c2_g_state;
      c2_c_r = 2.0 * c2_b_r - 1.0;
      c2_b_t = 2.0 * c2_t - 1.0;
      c2_b_t = c2_b_t * c2_b_t + c2_c_r * c2_c_r;
    } while (!(c2_b_t <= 1.0));

    c2_c_r *= muDoubleScalarSqrt(-2.0 * muDoubleScalarLog(c2_b_t) / c2_b_t);
    chartInstance->c2_state[0] = c2_g_state;
    c2_r = c2_c_r;
  } else {
    c2_d2 = c2_b_eml_rand_shr3cong(chartInstance, chartInstance->c2_state);
    c2_r = c2_d2;
  }

  return c2_r;
}

static void c2_genrandu(SFc2_RIEKF_testInstanceStruct *chartInstance, uint32_T
  c2_s, uint32_T *c2_e_state, real_T *c2_r)
{
  uint32_T c2_hi;
  uint32_T c2_test1;
  uint32_T c2_test2;
  (void)chartInstance;
  c2_hi = c2_s / 127773U;
  c2_test1 = 16807U * (c2_s - c2_hi * 127773U);
  c2_test2 = 2836U * c2_hi;
  if (c2_test1 < c2_test2) {
    *c2_e_state = (c2_test1 - c2_test2) + 2147483647U;
  } else {
    *c2_e_state = c2_test1 - c2_test2;
  }

  *c2_r = (real_T)*c2_e_state * 4.6566128752457969E-10;
}

static void c2_eml_rand_shr3cong(SFc2_RIEKF_testInstanceStruct *chartInstance,
  uint32_T c2_e_state[2], uint32_T c2_f_state[2], real_T *c2_r)
{
  int32_T c2_i24;
  for (c2_i24 = 0; c2_i24 < 2; c2_i24++) {
    c2_f_state[c2_i24] = c2_e_state[c2_i24];
  }

  *c2_r = c2_b_eml_rand_shr3cong(chartInstance, c2_f_state);
}

static void c2_eml_rand_mt19937ar(SFc2_RIEKF_testInstanceStruct *chartInstance,
  uint32_T c2_e_state[625], uint32_T c2_f_state[625], real_T *c2_r)
{
  int32_T c2_i25;
  for (c2_i25 = 0; c2_i25 < 625; c2_i25++) {
    c2_f_state[c2_i25] = c2_e_state[c2_i25];
  }

  *c2_r = c2_b_eml_rand_mt19937ar(chartInstance, c2_f_state);
}

static void c2_genrand_uint32_vector(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_mt[625], uint32_T c2_b_mt[625], uint32_T c2_u[2])
{
  int32_T c2_i26;
  for (c2_i26 = 0; c2_i26 < 625; c2_i26++) {
    c2_b_mt[c2_i26] = c2_mt[c2_i26];
  }

  c2_b_genrand_uint32_vector(chartInstance, c2_b_mt, c2_u);
}

static void c2_b_genrandu(SFc2_RIEKF_testInstanceStruct *chartInstance, uint32_T
  c2_mt[625], uint32_T c2_b_mt[625], real_T *c2_r)
{
  int32_T c2_i27;
  for (c2_i27 = 0; c2_i27 < 625; c2_i27++) {
    c2_b_mt[c2_i27] = c2_mt[c2_i27];
  }

  *c2_r = c2_c_genrandu(chartInstance, c2_b_mt);
}

static void c2_b_error(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  const mxArray *c2_y = NULL;
  static char_T c2_cv1[37] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'r', 'a', 'n', 'd', '_', 'i', 'n', 'v', 'a', 'l', 'i', 'd',
    'T', 'w', 'i', 's', 't', 'e', 'r', 'S', 't', 'a', 't', 'e' };

  (void)chartInstance;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_cv1, 10, 0U, 1U, 0U, 2, 1, 37),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c2_y));
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_d_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i28;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i28, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i28;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_thisId.fIdentifier = (const char *)c2_varName;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint32_T c2_e_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_c_method, const char_T *c2_identifier,
  boolean_T *c2_svPtr)
{
  uint32_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_c_method),
    &c2_thisId, c2_svPtr);
  sf_mex_destroy(&c2_c_method);
  return c2_y;
}

static uint32_T c2_f_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T *c2_svPtr)
{
  uint32_T c2_y;
  uint32_T c2_u1;
  (void)chartInstance;
  if (mxIsEmpty(c2_u)) {
    *c2_svPtr = false;
  } else {
    *c2_svPtr = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u1, 1, 7, 0U, 0, 0U, 0);
    c2_y = c2_u1;
  }

  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_g_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_e_state, const char_T *c2_identifier, boolean_T *c2_svPtr,
  uint32_T c2_y[625])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_e_state), &c2_thisId,
                        c2_svPtr, c2_y);
  sf_mex_destroy(&c2_e_state);
}

static void c2_h_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, boolean_T
  *c2_svPtr, uint32_T c2_y[625])
{
  uint32_T c2_uv3[625];
  int32_T c2_i29;
  (void)chartInstance;
  if (mxIsEmpty(c2_u)) {
    *c2_svPtr = false;
  } else {
    *c2_svPtr = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_uv3, 1, 7, 0U, 1, 0U, 1, 625);
    for (c2_i29 = 0; c2_i29 < 625; c2_i29++) {
      c2_y[c2_i29] = c2_uv3[c2_i29];
    }
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_e_state, const char_T *c2_identifier, boolean_T *c2_svPtr,
  uint32_T c2_y[2])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_e_state), &c2_thisId,
                        c2_svPtr, c2_y);
  sf_mex_destroy(&c2_e_state);
}

static void c2_j_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct *chartInstance,
  const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, boolean_T
  *c2_svPtr, uint32_T c2_y[2])
{
  uint32_T c2_uv4[2];
  int32_T c2_i30;
  (void)chartInstance;
  if (mxIsEmpty(c2_u)) {
    *c2_svPtr = false;
  } else {
    *c2_svPtr = true;
    sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_uv4, 1, 7, 0U, 1, 0U, 1, 2);
    for (c2_i30 = 0; c2_i30 < 2; c2_i30++) {
      c2_y[c2_i30] = c2_uv4[c2_i30];
    }
  }

  sf_mex_destroy(&c2_u);
}

static uint8_T c2_k_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_RIEKF_test, const char_T
  *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = (const char *)c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_RIEKF_test), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_RIEKF_test);
  return c2_y;
}

static uint8_T c2_l_emlrt_marshallIn(SFc2_RIEKF_testInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u2;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u2, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u2;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static real_T c2_b_eml_rand_shr3cong(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_e_state[2])
{
  uint32_T c2_icng;
  uint32_T c2_jsr;
  uint32_T c2_ui;
  uint32_T c2_b_icng;
  uint32_T c2_b_jsr;
  uint32_T c2_j;
  real_T c2_b_r;
  static real_T c2_dv3[65] = { 0.340945, 0.4573146, 0.5397793, 0.6062427,
    0.6631691, 0.7136975, 0.7596125, 0.8020356, 0.8417227, 0.8792102, 0.9148948,
    0.9490791, 0.9820005, 1.0138492, 1.044781, 1.0749254, 1.1043917, 1.1332738,
    1.161653, 1.189601, 1.2171815, 1.2444516, 1.2714635, 1.298265, 1.3249008,
    1.3514125, 1.3778399, 1.4042211, 1.4305929, 1.4569915, 1.4834527, 1.5100122,
    1.5367061, 1.5635712, 1.5906454, 1.617968, 1.6455802, 1.6735255, 1.7018503,
    1.7306045, 1.7598422, 1.7896223, 1.8200099, 1.851077, 1.8829044, 1.9155831,
    1.9492166, 1.9839239, 2.0198431, 2.0571356, 2.095993, 2.136645, 2.1793713,
    2.2245175, 2.2725186, 2.3239338, 2.3795008, 2.4402218, 2.5075117, 2.5834658,
    2.6713916, 2.7769942, 2.7769942, 2.7769942, 2.7769942 };

  real_T c2_x;
  real_T c2_c_r;
  uint32_T c2_c_icng;
  uint32_T c2_c_jsr;
  real_T c2_y;
  real_T c2_s;
  uint32_T c2_d_icng;
  uint32_T c2_d_jsr;
  uint32_T c2_e_icng;
  uint32_T c2_e_jsr;
  (void)chartInstance;
  c2_icng = 69069U * c2_e_state[0] + 1234567U;
  c2_jsr = c2_e_state[1] ^ c2_e_state[1] << 13;
  c2_jsr ^= c2_jsr >> 17;
  c2_jsr ^= c2_jsr << 5;
  c2_ui = c2_icng + c2_jsr;
  c2_b_icng = c2_icng;
  c2_b_jsr = c2_jsr;
  c2_j = (c2_ui & 63U) + 1U;
  c2_b_r = (real_T)(int32_T)c2_ui * 4.6566128730773926E-10 * c2_dv3[(int32_T)
    c2_j];
  if (muDoubleScalarAbs(c2_b_r) <= c2_dv3[(int32_T)c2_j - 1]) {
    c2_c_r = c2_b_r;
  } else {
    c2_x = (muDoubleScalarAbs(c2_b_r) - c2_dv3[(int32_T)c2_j - 1]) / (c2_dv3
      [(int32_T)c2_j] - c2_dv3[(int32_T)c2_j - 1]);
    c2_c_icng = 69069U * c2_icng + 1234567U;
    c2_c_jsr = c2_jsr ^ c2_jsr << 13;
    c2_c_jsr ^= c2_c_jsr >> 17;
    c2_c_jsr ^= c2_c_jsr << 5;
    c2_b_icng = c2_c_icng;
    c2_b_jsr = c2_c_jsr;
    c2_y = 0.5 + (real_T)(int32_T)(c2_c_icng + c2_c_jsr) * 2.328306436538696E-10;
    c2_s = c2_x + c2_y;
    if (c2_s > 1.301198) {
      if (c2_b_r < 0.0) {
        c2_c_r = 0.4878992 * c2_x - 0.4878992;
      } else {
        c2_c_r = 0.4878992 - 0.4878992 * c2_x;
      }
    } else if (c2_s <= 0.9689279) {
      c2_c_r = c2_b_r;
    } else {
      c2_x = 0.4878992 - 0.4878992 * c2_x;
      if (c2_y > 12.67706 - 12.37586 * muDoubleScalarExp(-0.5 * c2_x * c2_x)) {
        if (c2_b_r < 0.0) {
          c2_c_r = -c2_x;
        } else {
          c2_c_r = c2_x;
        }
      } else if (muDoubleScalarExp(-0.5 * c2_dv3[(int32_T)c2_j] * c2_dv3
                  [(int32_T)c2_j]) + c2_y * 0.01958303 / c2_dv3[(int32_T)c2_j] <=
                 muDoubleScalarExp(-0.5 * c2_b_r * c2_b_r)) {
        c2_c_r = c2_b_r;
      } else {
        do {
          c2_d_icng = 69069U * c2_b_icng + 1234567U;
          c2_d_jsr = c2_b_jsr ^ c2_b_jsr << 13;
          c2_d_jsr ^= c2_d_jsr >> 17;
          c2_d_jsr ^= c2_d_jsr << 5;
          c2_x = muDoubleScalarLog(0.5 + (real_T)(int32_T)(c2_d_icng + c2_d_jsr)
            * 2.328306436538696E-10) / 2.776994;
          c2_e_icng = 69069U * c2_d_icng + 1234567U;
          c2_e_jsr = c2_d_jsr ^ c2_d_jsr << 13;
          c2_e_jsr ^= c2_e_jsr >> 17;
          c2_e_jsr ^= c2_e_jsr << 5;
          c2_b_icng = c2_e_icng;
          c2_b_jsr = c2_e_jsr;
        } while (!(-2.0 * muDoubleScalarLog(0.5 + (real_T)(int32_T)(c2_e_icng +
                    c2_e_jsr) * 2.328306436538696E-10) > c2_x * c2_x));

        if (c2_b_r < 0.0) {
          c2_c_r = c2_x - 2.776994;
        } else {
          c2_c_r = 2.776994 - c2_x;
        }
      }
    }
  }

  c2_e_state[0] = c2_b_icng;
  c2_e_state[1] = c2_b_jsr;
  return c2_c_r;
}

static real_T c2_b_eml_rand_mt19937ar(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_e_state[625])
{
  uint32_T c2_u32[2];
  uint32_T c2_i;
  real_T c2_b_r;
  static real_T c2_dv4[257] = { 0.0, 0.215241895984875, 0.286174591792068,
    0.335737519214422, 0.375121332878378, 0.408389134611989, 0.43751840220787,
    0.46363433679088, 0.487443966139235, 0.50942332960209, 0.529909720661557,
    0.549151702327164, 0.567338257053817, 0.584616766106378, 0.601104617755991,
    0.61689699000775, 0.63207223638606, 0.646695714894993, 0.660822574244419,
    0.674499822837293, 0.687767892795788, 0.700661841106814, 0.713212285190975,
    0.725446140909999, 0.737387211434295, 0.749056662017815, 0.760473406430107,
    0.771654424224568, 0.782615023307232, 0.793369058840623, 0.80392911698997,
    0.814306670135215, 0.824512208752291, 0.834555354086381, 0.844444954909153,
    0.854189171008163, 0.863795545553308, 0.87327106808886, 0.882622229585165,
    0.891855070732941, 0.900975224461221, 0.909987953496718, 0.91889818364959,
    0.927710533401999, 0.936429340286575, 0.945058684468165, 0.953602409881086,
    0.96206414322304, 0.970447311064224, 0.978755155294224, 0.986990747099062,
    0.99515699963509, 1.00325667954467, 1.01129241744, 1.01926671746548,
    1.02718196603564, 1.03504043983344, 1.04284431314415, 1.05059566459093,
    1.05829648333067, 1.06594867476212, 1.07355406579244, 1.0811144097034,
    1.08863139065398, 1.09610662785202, 1.10354167942464, 1.11093804601357,
    1.11829717411934, 1.12562045921553, 1.13290924865253, 1.14016484436815,
    1.14738850542085, 1.15458145035993, 1.16174485944561, 1.16887987673083,
    1.17598761201545, 1.18306914268269, 1.19012551542669, 1.19715774787944,
    1.20416683014438, 1.2111537262437, 1.21811937548548, 1.22506469375653,
    1.23199057474614, 1.23889789110569, 1.24578749554863, 1.2526602218949,
    1.25951688606371, 1.26635828701823, 1.27318520766536, 1.27999841571382,
    1.28679866449324, 1.29358669373695, 1.30036323033084, 1.30712898903073,
    1.31388467315022, 1.32063097522106, 1.32736857762793, 1.33409815321936,
    1.3408203658964, 1.34753587118059, 1.35424531676263, 1.36094934303328,
    1.36764858359748, 1.37434366577317, 1.38103521107586, 1.38772383568998,
    1.39441015092814, 1.40109476367925, 1.4077782768464, 1.41446128977547,
    1.42114439867531, 1.42782819703026, 1.43451327600589, 1.44120022484872,
    1.44788963128058, 1.45458208188841, 1.46127816251028, 1.46797845861808,
    1.47468355569786, 1.48139403962819, 1.48811049705745, 1.49483351578049,
    1.50156368511546, 1.50830159628131, 1.51504784277671, 1.521803020761,
    1.52856772943771, 1.53534257144151, 1.542128153229, 1.54892508547417,
    1.55573398346918, 1.56255546753104, 1.56939016341512, 1.57623870273591,
    1.58310172339603, 1.58997987002419, 1.59687379442279, 1.60378415602609,
    1.61071162236983, 1.61765686957301, 1.62462058283303, 1.63160345693487,
    1.63860619677555, 1.64562951790478, 1.65267414708306, 1.65974082285818,
    1.66683029616166, 1.67394333092612, 1.68108070472517, 1.68824320943719,
    1.69543165193456, 1.70264685479992, 1.7098896570713, 1.71716091501782,
    1.72446150294804, 1.73179231405296, 1.73915426128591, 1.74654827828172,
    1.75397532031767, 1.76143636531891, 1.76893241491127, 1.77646449552452,
    1.78403365954944, 1.79164098655216, 1.79928758454972, 1.80697459135082,
    1.81470317596628, 1.82247454009388, 1.83028991968276, 1.83815058658281,
    1.84605785028518, 1.8540130597602, 1.86201760539967, 1.87007292107127,
    1.878180486293, 1.88634182853678, 1.8945585256707, 1.90283220855043,
    1.91116456377125, 1.91955733659319, 1.92801233405266, 1.93653142827569,
    1.94511656000868, 1.95376974238465, 1.96249306494436, 1.97128869793366,
    1.98015889690048, 1.98910600761744, 1.99813247135842, 2.00724083056053,
    2.0164337349062, 2.02571394786385, 2.03508435372962, 2.04454796521753,
    2.05410793165065, 2.06376754781173, 2.07353026351874, 2.0833996939983,
    2.09337963113879, 2.10347405571488, 2.11368715068665, 2.12402331568952,
    2.13448718284602, 2.14508363404789, 2.15581781987674, 2.16669518035431,
    2.17772146774029, 2.18890277162636, 2.20024554661128, 2.21175664288416,
    2.22344334009251, 2.23531338492992, 2.24737503294739, 2.25963709517379,
    2.27210899022838, 2.28480080272449, 2.29772334890286, 2.31088825060137,
    2.32430801887113, 2.33799614879653, 2.35196722737914, 2.36623705671729,
    2.38082279517208, 2.39574311978193, 2.41101841390112, 2.42667098493715,
    2.44272531820036, 2.4592083743347, 2.47614993967052, 2.49358304127105,
    2.51154444162669, 2.53007523215985, 2.54922155032478, 2.56903545268184,
    2.58957598670829, 2.61091051848882, 2.63311639363158, 2.65628303757674,
    2.68051464328574, 2.70593365612306, 2.73268535904401, 2.76094400527999,
    2.79092117400193, 2.82287739682644, 2.85713873087322, 2.89412105361341,
    2.93436686720889, 2.97860327988184, 3.02783779176959, 3.08352613200214,
    3.147889289518, 3.2245750520478, 3.32024473383983, 3.44927829856143,
    3.65415288536101, 3.91075795952492 };

  real_T c2_u;
  real_T c2_b_u;
  static real_T c2_dv5[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  real_T c2_x;
  real_T c2_c_u;
  int32_T exitg1;
  do {
    exitg1 = 0;
    c2_b_genrand_uint32_vector(chartInstance, c2_e_state, c2_u32);
    c2_i = (c2_u32[1] >> 24U) + 1U;
    c2_b_r = (((real_T)(c2_u32[0] >> 3U) * 1.6777216E+7 + (real_T)(c2_u32[1] &
                16777215U)) * 2.2204460492503131E-16 - 1.0) * c2_dv4[(int32_T)
      c2_i];
    if (muDoubleScalarAbs(c2_b_r) <= c2_dv4[(int32_T)c2_i - 1]) {
      exitg1 = 1;
    } else if ((real_T)c2_i < 256.0) {
      c2_u = c2_c_genrandu(chartInstance, c2_e_state);
      if (c2_dv5[(int32_T)c2_i] + c2_u * (c2_dv5[(int32_T)c2_i - 1] - c2_dv5
           [(int32_T)c2_i]) < muDoubleScalarExp(-0.5 * c2_b_r * c2_b_r)) {
        exitg1 = 1;
      }
    } else {
      do {
        c2_b_u = c2_c_genrandu(chartInstance, c2_e_state);
        c2_x = muDoubleScalarLog(c2_b_u) * 0.273661237329758;
        c2_c_u = c2_c_genrandu(chartInstance, c2_e_state);
      } while (!(-2.0 * muDoubleScalarLog(c2_c_u) > c2_x * c2_x));

      if (c2_b_r < 0.0) {
        c2_b_r = c2_x - 3.65415288536101;
      } else {
        c2_b_r = 3.65415288536101 - c2_x;
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_b_r;
}

static void c2_b_genrand_uint32_vector(SFc2_RIEKF_testInstanceStruct
  *chartInstance, uint32_T c2_mt[625], uint32_T c2_u[2])
{
  int32_T c2_j;
  uint32_T c2_mti;
  int32_T c2_kk;
  uint32_T c2_y;
  int32_T c2_b_kk;
  uint32_T c2_b_y;
  uint32_T c2_c_y;
  uint32_T c2_d_y;
  (void)chartInstance;
  for (c2_j = 0; c2_j < 2; c2_j++) {
    c2_mti = c2_mt[624] + 1U;
    if ((real_T)c2_mti >= 625.0) {
      for (c2_kk = 0; c2_kk < 227; c2_kk++) {
        c2_y = (c2_mt[c2_kk] & 2147483648U) | (c2_mt[(int32_T)((1.0 + (real_T)
          c2_kk) + 1.0) - 1] & 2147483647U);
        if ((real_T)(c2_y & 1U) == 0.0) {
          c2_b_y = c2_y >> 1U;
        } else {
          c2_b_y = c2_y >> 1U ^ 2567483615U;
        }

        c2_mt[c2_kk] = c2_mt[(int32_T)((1.0 + (real_T)c2_kk) + 397.0) - 1] ^
          c2_b_y;
      }

      for (c2_b_kk = 0; c2_b_kk < 396; c2_b_kk++) {
        c2_y = (c2_mt[c2_b_kk + 227] & 2147483648U) | (c2_mt[(int32_T)((228.0 +
          (real_T)c2_b_kk) + 1.0) - 1] & 2147483647U);
        if ((real_T)(c2_y & 1U) == 0.0) {
          c2_d_y = c2_y >> 1U;
        } else {
          c2_d_y = c2_y >> 1U ^ 2567483615U;
        }

        c2_mt[c2_b_kk + 227] = c2_mt[(int32_T)(((228.0 + (real_T)c2_b_kk) + 1.0)
          - 228.0) - 1] ^ c2_d_y;
      }

      c2_y = (c2_mt[623] & 2147483648U) | (c2_mt[0] & 2147483647U);
      if ((real_T)(c2_y & 1U) == 0.0) {
        c2_c_y = c2_y >> 1U;
      } else {
        c2_c_y = c2_y >> 1U ^ 2567483615U;
      }

      c2_mt[623] = c2_mt[396] ^ c2_c_y;
      c2_mti = 1U;
    }

    c2_y = c2_mt[(int32_T)c2_mti - 1];
    c2_mt[624] = c2_mti;
    c2_y ^= c2_y >> 11U;
    c2_y ^= c2_y << 7U & 2636928640U;
    c2_y ^= c2_y << 15U & 4022730752U;
    c2_y ^= c2_y >> 18U;
    c2_u[c2_j] = c2_y;
  }
}

static real_T c2_c_genrandu(SFc2_RIEKF_testInstanceStruct *chartInstance,
  uint32_T c2_mt[625])
{
  real_T c2_r;
  uint32_T c2_u[2];
  boolean_T c2_isvalid;
  boolean_T c2_b_isvalid;
  int32_T c2_k;
  int32_T exitg1;
  boolean_T exitg2;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on:        */
  /*                                                                         */
  /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
  /*                                                                         */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
  /*  All rights reserved.                                                   */
  /*                                                                         */
  /*  Redistribution and use in source and binary forms, with or without     */
  /*  modification, are permitted provided that the following conditions     */
  /*  are met:                                                               */
  /*                                                                         */
  /*    1. Redistributions of source code must retain the above copyright    */
  /*       notice, this list of conditions and the following disclaimer.     */
  /*                                                                         */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer      */
  /*       in the documentation and/or other materials provided with the     */
  /*       distribution.                                                     */
  /*                                                                         */
  /*    3. The names of its contributors may not be used to endorse or       */
  /*       promote products derived from this software without specific      */
  /*       prior written permission.                                         */
  /*                                                                         */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
  /*                                                                         */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    c2_b_genrand_uint32_vector(chartInstance, c2_mt, c2_u);
    c2_r = 1.1102230246251565E-16 * ((real_T)(c2_u[0] >> 5U) * 6.7108864E+7 +
      (real_T)(c2_u[1] >> 6U));
    if (c2_r == 0.0) {
      if (((real_T)c2_mt[624] >= 1.0) && ((real_T)c2_mt[624] < 625.0)) {
        c2_isvalid = true;
      } else {
        c2_isvalid = false;
      }

      c2_b_isvalid = c2_isvalid;
      if (c2_isvalid) {
        c2_b_isvalid = false;
        c2_k = 0;
        exitg2 = false;
        while ((!exitg2) && (c2_k + 1 < 625)) {
          if ((real_T)c2_mt[c2_k] == 0.0) {
            c2_k++;
          } else {
            c2_b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!c2_b_isvalid) {
        c2_b_error(chartInstance);
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return c2_r;
}

static void init_dsm_address_info(SFc2_RIEKF_testInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc2_RIEKF_testInstanceStruct
  *chartInstance)
{
  chartInstance->c2_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c2_encoder = (real_T (*)[14])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_encoder_n = (real_T (*)[14])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_contact = (real_T (*)[2])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_encoder_noise = (real_T (*)[14])
    ssGetOutputPortSignal_wrapper(chartInstance->S, 2);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_RIEKF_test_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4016986952U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(662808743U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3998398739U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3264109674U);
}

mxArray* sf_c2_RIEKF_test_get_post_codegen_info(void);
mxArray *sf_c2_RIEKF_test_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("zEMSb1E1uvWmSUxhqQ6hlB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(14);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(14);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(14);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c2_RIEKF_test_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_RIEKF_test_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_RIEKF_test_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_RIEKF_test_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_RIEKF_test_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c2_RIEKF_test(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x9'type','srcId','name','auxInfo'{{M[1],M[5],T\"encoder_n\",},{M[1],M[7],T\"encoder_noise\",},{M[4],M[0],T\"method\",S'l','i','p'{{M1x2[523 529],M[1],T\"D:\\Program Files\\MATLAB\\R2017b\\toolbox\\eml\\lib\\matlab\\randfun\\private\\eml_rand.m\"}}},{M[4],M[0],T\"method\",S'l','i','p'{{M1x2[649 655],M[1],T\"D:\\Program Files\\MATLAB\\R2017b\\toolbox\\eml\\lib\\matlab\\randfun\\private\\eml_randn.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[165 170],M[1],T\"D:\\Program Files\\MATLAB\\R2017b\\toolbox\\eml\\lib\\matlab\\randfun\\private\\eml_rand_mcg16807_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[166 171],M[1],T\"D:\\Program Files\\MATLAB\\R2017b\\toolbox\\eml\\lib\\matlab\\randfun\\private\\eml_rand_mt19937ar_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[165 170],M[1],T\"D:\\Program Files\\MATLAB\\R2017b\\toolbox\\eml\\lib\\matlab\\randfun\\private\\eml_rand_shr3cong_stateful.m\"}}},{M[4],M[0],T\"state\",S'l','i','p'{{M1x2[656 661],M[1],T\"D:\\Program Files\\MATLAB\\R2017b\\toolbox\\eml\\lib\\matlab\\randfun\\private\\eml_randn.m\"}}},{M[8],M[0],T\"is_active_c2_RIEKF_test\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 9, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_RIEKF_test_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_RIEKF_testInstanceStruct *chartInstance =
      (SFc2_RIEKF_testInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _RIEKF_testMachineNumber_,
           2,
           1,
           1,
           0,
           4,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_RIEKF_testMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_RIEKF_testMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _RIEKF_testMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"encoder");
          _SFD_SET_DATA_PROPS(1,1,1,0,"contact");
          _SFD_SET_DATA_PROPS(2,2,0,1,"encoder_n");
          _SFD_SET_DATA_PROPS(3,2,0,1,"encoder_noise");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,1,0,0,0,0,0,3,1);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,344);
        _SFD_CV_INIT_EML_IF(0,1,0,112,182,-1,244);

        {
          static int condStart[] = { 115, 150, 166 };

          static int condEnd[] = { 147, 164, 180 };

          static int pfixExpr[] = { 0, 1, 2, -3, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,115,181,3,0,&(condStart[0]),&(condEnd[0]),
                                5,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,115,147,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,150,164,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,166,180,-1,2);

        {
          unsigned int dimVector[1];
          dimVector[0]= 14U;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2U;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 14U;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 14U;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _RIEKF_testMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_RIEKF_testInstanceStruct *chartInstance =
      (SFc2_RIEKF_testInstanceStruct *)sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, (void *)chartInstance->c2_encoder);
        _SFD_SET_DATA_VALUE_PTR(2U, (void *)chartInstance->c2_encoder_n);
        _SFD_SET_DATA_VALUE_PTR(1U, (void *)chartInstance->c2_contact);
        _SFD_SET_DATA_VALUE_PTR(3U, (void *)chartInstance->c2_encoder_noise);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sQNmBslFt1ICiHHlylFUyXD";
}

static void sf_opaque_initialize_c2_RIEKF_test(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*)
    chartInstanceVar);
  initialize_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_RIEKF_test(void *chartInstanceVar)
{
  enable_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_RIEKF_test(void *chartInstanceVar)
{
  disable_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_RIEKF_test(void *chartInstanceVar)
{
  sf_gateway_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_RIEKF_test(SimStruct* S)
{
  return get_sim_state_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_RIEKF_test(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c2_RIEKF_test(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_RIEKF_test_optimization_info();
    }

    finalize_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_RIEKF_test(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_RIEKF_test((SFc2_RIEKF_testInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c2_RIEKF_test(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetStatesModifiedOnlyInUpdate(S, 0);
  ssSetBlockIsPurelyCombinatorial_wrapper(S, 0);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_RIEKF_test_optimization_info(sim_mode_is_rtw_gen
      (S), sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 2);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 2; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(223226506U));
  ssSetChecksum1(S,(1117932563U));
  ssSetChecksum2(S,(624092014U));
  ssSetChecksum3(S,(2882314164U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_RIEKF_test(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_RIEKF_test(SimStruct *S)
{
  SFc2_RIEKF_testInstanceStruct *chartInstance;
  chartInstance = (SFc2_RIEKF_testInstanceStruct *)utMalloc(sizeof
    (SFc2_RIEKF_testInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc2_RIEKF_testInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_RIEKF_test;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c2_RIEKF_test;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c2_RIEKF_test;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_RIEKF_test;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_RIEKF_test;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c2_RIEKF_test;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c2_RIEKF_test;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c2_RIEKF_test;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_RIEKF_test;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_RIEKF_test;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_RIEKF_test;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c2_RIEKF_test(chartInstance);
}

void c2_RIEKF_test_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_RIEKF_test(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_RIEKF_test(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_RIEKF_test(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_RIEKF_test_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
