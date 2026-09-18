/**
 * @file    unity_min.c
 * @brief   Test framework implementation - see unity_min.h.
 */

#include "unity_min.h"

unsigned int UnityTestsRun          = 0U;
unsigned int UnityTestsFailed       = 0U;
bool         UnityCurrentTestFailed = false;
const char  *UnityCurrentTestName   = "";

/* Weak definitions so a suite that needs no fixtures can omit them.
 * A suite that does need them simply defines its own, which overrides these. */
__attribute__((weak)) void setUp(void)    { }
__attribute__((weak)) void tearDown(void) { }

void UnityBegin(const char *suite_name)
{
    UnityTestsRun    = 0U;
    UnityTestsFailed = 0U;

    printf("\n");
    printf("========================================================\n");
    printf("  %s\n", suite_name);
    printf("========================================================\n");
}

void UnityFail(const char *file, int line, const char *message)
{
    /* Only the first failure in a test is reported. The assertions return
     * immediately on failure, so any later ones would be consequences of the
     * first rather than independent findings. */
    if (!UnityCurrentTestFailed)
    {
        UnityCurrentTestFailed = true;
        UnityTestsFailed++;

        printf("  [FAIL] %s\n", UnityCurrentTestName);
        printf("         %s:%d\n", file, line);
        printf("         %s\n", message);
    }
}

void UnityRunTest(void (*test_function)(void), const char *name)
{
    UnityCurrentTestName   = name;
    UnityCurrentTestFailed = false;
    UnityTestsRun++;

    /* setUp runs before every test so each one starts from an identical,
     * known state. Tests that depend on the order they run in are worse than
     * no tests: they pass or fail for reasons unrelated to the code. */
    setUp();
    test_function();
    tearDown();

    if (!UnityCurrentTestFailed)
    {
        printf("  [PASS] %s\n", name);
    }
}

int UnityEnd(void)
{
    const unsigned int passed = UnityTestsRun - UnityTestsFailed;

    printf("--------------------------------------------------------\n");
    printf("  %u tests, %u passed, %u failed\n",
           UnityTestsRun, passed, UnityTestsFailed);
    printf("  RESULT: %s\n", (UnityTestsFailed == 0U) ? "OK" : "FAILED");
    printf("========================================================\n\n");

    /* Non-zero exit on failure is what makes `make test` and the CI job fail.
     * Without it the pipeline would go green while the tests were failing. */
    return (UnityTestsFailed == 0U) ? 0 : 1;
}
