/**
 * @file    unity_min.h
 * @brief   Minimal xUnit-style test framework with a Unity-compatible API.
 *
 * Why a small framework instead of vendoring Unity
 * ------------------------------------------------
 * ThrowTheSwitch's Unity is the de facto standard for unit testing embedded
 * C, and it is what a production project would use. It is also around 2000
 * lines of third-party source, and vendoring it into a teaching repository
 * buries the interesting part - the tests themselves - under a dependency
 * nobody reads.
 *
 * This file implements the subset of Unity's assertion API that the tests
 * actually use, with identical macro names and semantics. Migrating to real
 * Unity later is a matter of deleting this file and adding
 *
 *     git submodule add https://github.com/ThrowTheSwitch/Unity
 *
 * then pointing the Makefile at it. Not a single test needs to change.
 *
 * The framework has no dependency on the STM32 HAL, on hardware, or on
 * anything but the C standard library, so it builds and runs on any PC.
 */

#ifndef TESTS_UNITY_MIN_H
#define TESTS_UNITY_MIN_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================= */
/* Framework state                                                           */
/* ========================================================================= */

extern unsigned int UnityTestsRun;
extern unsigned int UnityTestsFailed;
extern bool         UnityCurrentTestFailed;
extern const char  *UnityCurrentTestName;

/**
 * @brief Start a test session and print the header.
 */
void UnityBegin(const char *suite_name);

/**
 * @brief Finish the session, print the summary, and return the exit code.
 *
 * @return 0 if every test passed, 1 otherwise. Returning a non-zero exit
 *         code on failure is what lets `make test` and the CI pipeline fail
 *         the build automatically - a test suite nobody is forced to look at
 *         provides no protection at all.
 */
int UnityEnd(void);

/**
 * @brief Report one failed assertion.
 */
void UnityFail(const char *file, int line, const char *message);

/**
 * @brief Run one test function, with its setUp and tearDown.
 */
void UnityRunTest(void (*test_function)(void), const char *name);

/* Optional per-test fixtures. Define them in the test file; empty defaults
 * are provided by the framework if a suite does not need them. */
void setUp(void);
void tearDown(void);

/* ========================================================================= */
/* Test runner macro                                                         */
/* ========================================================================= */

#define RUN_TEST(fn)   UnityRunTest((fn), #fn)

/* ========================================================================= */
/* Assertions                                                                */
/* ========================================================================= */

/* Every assertion returns early on failure. Continuing after a failed
 * assertion would report a cascade of consequential errors and bury the one
 * that actually matters. */

#define TEST_FAIL_MESSAGE(msg)                                                \
    do { UnityFail(__FILE__, __LINE__, (msg)); return; } while (0)

#define TEST_ASSERT_TRUE(condition)                                           \
    do {                                                                      \
        if (!(condition)) {                                                   \
            UnityFail(__FILE__, __LINE__, "expected TRUE: " #condition);      \
            return;                                                           \
        }                                                                     \
    } while (0)

#define TEST_ASSERT_FALSE(condition)                                          \
    do {                                                                      \
        if (condition) {                                                      \
            UnityFail(__FILE__, __LINE__, "expected FALSE: " #condition);     \
            return;                                                           \
        }                                                                     \
    } while (0)

#define TEST_ASSERT_NULL(pointer)                                             \
    TEST_ASSERT_TRUE((pointer) == NULL)

#define TEST_ASSERT_NOT_NULL(pointer)                                         \
    TEST_ASSERT_TRUE((pointer) != NULL)

/**
 * @brief Compare two integers, printing both values on failure.
 *
 * Printing expected *and* actual is the difference between a report that
 * tells you what broke and one that only tells you that something did.
 */
#define TEST_ASSERT_EQUAL_INT(expected, actual)                               \
    do {                                                                      \
        const long long _e = (long long)(expected);                           \
        const long long _a = (long long)(actual);                             \
        if (_e != _a) {                                                       \
            char _m[192];                                                     \
            snprintf(_m, sizeof(_m),                                          \
                     "%s: expected %lld but got %lld", #actual, _e, _a);      \
            UnityFail(__FILE__, __LINE__, _m);                                \
            return;                                                           \
        }                                                                     \
    } while (0)

#define TEST_ASSERT_EQUAL(expected, actual)        TEST_ASSERT_EQUAL_INT((expected), (actual))
#define TEST_ASSERT_EQUAL_UINT(expected, actual)   TEST_ASSERT_EQUAL_INT((expected), (actual))
#define TEST_ASSERT_EQUAL_UINT8(expected, actual)  TEST_ASSERT_EQUAL_INT((expected), (actual))
#define TEST_ASSERT_EQUAL_UINT16(expected, actual) TEST_ASSERT_EQUAL_INT((expected), (actual))
#define TEST_ASSERT_EQUAL_UINT32(expected, actual) TEST_ASSERT_EQUAL_INT((expected), (actual))
#define TEST_ASSERT_EQUAL_INT8(expected, actual)   TEST_ASSERT_EQUAL_INT((expected), (actual))

/** Same comparison as EQUAL_INT, but reports in hexadecimal - far easier to
 *  read when the value is a bitmask or a raw protocol byte. */
#define TEST_ASSERT_EQUAL_HEX8(expected, actual)                              \
    do {                                                                      \
        const unsigned _e = (unsigned)(expected) & 0xFFu;                     \
        const unsigned _a = (unsigned)(actual)   & 0xFFu;                     \
        if (_e != _a) {                                                       \
            char _m[192];                                                     \
            snprintf(_m, sizeof(_m),                                          \
                     "%s: expected 0x%02X but got 0x%02X", #actual, _e, _a);  \
            UnityFail(__FILE__, __LINE__, _m);                                \
            return;                                                           \
        }                                                                     \
    } while (0)

#define TEST_ASSERT_EQUAL_HEX16(expected, actual)                             \
    do {                                                                      \
        const unsigned _e = (unsigned)(expected) & 0xFFFFu;                   \
        const unsigned _a = (unsigned)(actual)   & 0xFFFFu;                   \
        if (_e != _a) {                                                       \
            char _m[192];                                                     \
            snprintf(_m, sizeof(_m),                                          \
                     "%s: expected 0x%04X but got 0x%04X", #actual, _e, _a);  \
            UnityFail(__FILE__, __LINE__, _m);                                \
            return;                                                           \
        }                                                                     \
    } while (0)

#define TEST_ASSERT_EQUAL_STRING(expected, actual)                            \
    do {                                                                      \
        const char *_e = (expected);                                          \
        const char *_a = (actual);                                            \
        if ((_e == NULL) || (_a == NULL) || (strcmp(_e, _a) != 0)) {          \
            char _m[192];                                                     \
            snprintf(_m, sizeof(_m), "%s: expected \"%s\" but got \"%s\"",    \
                     #actual, _e ? _e : "(null)", _a ? _a : "(null)");        \
            UnityFail(__FILE__, __LINE__, _m);                                \
            return;                                                           \
        }                                                                     \
    } while (0)

/**
 * @brief Compare two byte arrays, reporting the first differing index.
 *
 * Used heavily by the CAN tests, where the exact byte layout *is* the
 * specification and the index of a mismatch immediately identifies which
 * signal was packed wrongly.
 */
#define TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, length)               \
    do {                                                                      \
        const uint8_t *_e = (const uint8_t *)(expected);                      \
        const uint8_t *_a = (const uint8_t *)(actual);                        \
        for (size_t _i = 0; _i < (size_t)(length); _i++) {                    \
            if (_e[_i] != _a[_i]) {                                           \
                char _m[192];                                                 \
                snprintf(_m, sizeof(_m),                                      \
                         "%s: byte[%zu] expected 0x%02X but got 0x%02X",      \
                         #actual, _i, _e[_i], _a[_i]);                        \
                UnityFail(__FILE__, __LINE__, _m);                            \
                return;                                                       \
            }                                                                 \
        }                                                                     \
    } while (0)

#endif /* TESTS_UNITY_MIN_H */
