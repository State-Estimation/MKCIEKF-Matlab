#ifndef __c2_RIEKF_test_h__
#define __c2_RIEKF_test_h__

/* Type Definitions */
#ifndef typedef_SFc2_RIEKF_testInstanceStruct
#define typedef_SFc2_RIEKF_testInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_RIEKF_test;
  uint32_T c2_method;
  boolean_T c2_method_not_empty;
  uint32_T c2_state[2];
  boolean_T c2_state_not_empty;
  uint32_T c2_b_method;
  boolean_T c2_b_method_not_empty;
  uint32_T c2_b_state;
  boolean_T c2_b_state_not_empty;
  uint32_T c2_c_state[2];
  boolean_T c2_c_state_not_empty;
  uint32_T c2_d_state[625];
  boolean_T c2_d_state_not_empty;
  void *c2_fEmlrtCtx;
  real_T (*c2_encoder)[14];
  real_T (*c2_encoder_n)[14];
  real_T (*c2_contact)[2];
  real_T (*c2_encoder_noise)[14];
} SFc2_RIEKF_testInstanceStruct;

#endif                                 /*typedef_SFc2_RIEKF_testInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_RIEKF_test_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_RIEKF_test_get_check_sum(mxArray *plhs[]);
extern void c2_RIEKF_test_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
