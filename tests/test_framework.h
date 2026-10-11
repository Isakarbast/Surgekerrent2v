/* Minimal dependency-free test harness. */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <math.h>
#include <stdio.h>

static int g_tests_run, g_tests_failed, g_checks_failed;
static const char *g_current_test;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        g_checks_failed++; \
    } } while (0)

#define CHECK_NEAR(a, b, tol) do { \
    double _a = (a), _b = (b); \
    if (fabs(_a - _b) > (tol)) { \
        printf("  FAIL %s:%d: %s = %g, expected %g +- %g\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); \
        g_checks_failed++; \
    } } while (0)

#define RUN_TEST(fn) do { \
    int _before = g_checks_failed; \
    g_current_test = #fn; \
    g_tests_run++; \
    fn(); \
    if (g_checks_failed != _before) { g_tests_failed++; printf("[FAIL] %s\n", #fn); } \
    else printf("[ OK ] %s\n", #fn); \
    } while (0)

#define TEST_SUMMARY() ( \
    printf("\n%d tests, %d failed\n", g_tests_run, g_tests_failed), \
    g_tests_failed ? 1 : 0)

#endif
