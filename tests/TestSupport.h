#pragma once

// doctest 2.4.11 cannot stringify scoped enums with recent MSVC (C2110 inside
// stringifyBinaryExpr), so enum comparisons in the tests are written as
// CHECK((a == b)), which evaluates the expression without decomposing it.

#include <doctest/doctest.h>
