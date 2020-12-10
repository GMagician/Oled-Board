#pragma once

#define FORCE_INLINE      __attribute__((always_inline)) inline

#define TPASTE2(a,b)            a##b

#define MREPEAT(count,macro)    TPASTE2(MREPEAT, count)(macro)

#define MREPEAT0(macro)
#define MREPEAT1(macro)         MREPEAT0(macro) macro(0)
#define MREPEAT2(macro)         MREPEAT1(macro) macro(1)
#define MREPEAT3(macro)         MREPEAT2(macro) macro(2)
#define MREPEAT4(macro)         MREPEAT3(macro) macro(3)
#define MREPEAT5(macro)         MREPEAT4(macro) macro(4)
#define MREPEAT6(macro)         MREPEAT5(macro) macro(5)
#define MREPEAT7(macro)         MREPEAT6(macro) macro(6)
#define MREPEAT8(macro)         MREPEAT7(macro) macro(7)
#define MREPEAT9(macro)         MREPEAT8(macro) macro(8)
