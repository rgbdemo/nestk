#ifndef TESTS_TEST_COMMON_H
#define TESTS_TEST_COMMON_H

#include <ntk/ntk.h>

#define NTK_TEST_FLOAT_EQ(Value, Ref) test_float_eq(#Value, Value, Ref)

namespace ntk
{

inline bool test_float_eq(const char* value_name, float value, float ref)
{
  ntk_dbg(1) << "[TEST " << value_name << "] " << value << " == " << ref;
  bool ok = (flt_eq(value, ref, 1e-3f));
  ntk_ensure(ok, "Test failed");
  return ok;
}

}

#endif // TESTS_TEST_COMMON_H
