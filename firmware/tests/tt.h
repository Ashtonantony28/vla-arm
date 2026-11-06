/* tt.h - tiny single-header test framework for the armfw host tests (GCC/Clang, host only).
 *
 *   TT_TEST(name) { TT_ASSERT(x); TT_EQ(a, b); TT_NEAR(a, b, tol); }
 *   TT_MAIN()
 *
 * The test binary accepts: (no args) run all | --list | <test name>. CMake registers every
 * TT_TEST(...) as its own ctest case by scanning the source file. */
#ifndef TT_H
#define TT_H

#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void (*tt_fn)(void);
typedef struct {
    const char *name;
    tt_fn fn;
} tt_case_t;

#define TT_MAX_CASES 256
static tt_case_t tt_cases[TT_MAX_CASES];
static int tt_ncases;
static jmp_buf tt_jmp;
static int tt_checks;

static inline void tt_register(const char *name, tt_fn fn) {
    if (tt_ncases < TT_MAX_CASES) {
        tt_cases[tt_ncases].name = name;
        tt_cases[tt_ncases].fn = fn;
        tt_ncases++;
    }
}

#define TT_TEST(name)                                                                  \
    static void name(void);                                                            \
    __attribute__((constructor)) static void tt_reg_##name(void) { tt_register(#name, name); } \
    static void name(void)

#define TT_FAIL(...)                                                       \
    do {                                                                   \
        fprintf(stderr, "  FAIL %s:%d: ", __FILE__, __LINE__);             \
        fprintf(stderr, __VA_ARGS__);                                      \
        fprintf(stderr, "\n");                                             \
        longjmp(tt_jmp, 1);                                                \
    } while (0)

#define TT_ASSERT(cond)                                     \
    do {                                                    \
        tt_checks++;                                        \
        if (!(cond)) TT_FAIL("assertion failed: %s", #cond); \
    } while (0)

#define TT_ASSERT_MSG(cond, ...)               \
    do {                                       \
        tt_checks++;                           \
        if (!(cond)) TT_FAIL(__VA_ARGS__);     \
    } while (0)

#define TT_EQ(a, b)                                                                         \
    do {                                                                                    \
        long long tt_a_ = (long long)(a), tt_b_ = (long long)(b);                           \
        tt_checks++;                                                                        \
        if (tt_a_ != tt_b_) TT_FAIL("%s == %s: %lld != %lld (0x%llx != 0x%llx)", #a, #b, tt_a_, tt_b_, \
                                    (unsigned long long)tt_a_, (unsigned long long)tt_b_);  \
    } while (0)

#define TT_NEAR(a, b, tol)                                                                    \
    do {                                                                                      \
        double tt_a_ = (double)(a), tt_b_ = (double)(b), tt_t_ = (double)(tol);               \
        tt_checks++;                                                                          \
        if (!(tt_a_ - tt_b_ <= tt_t_ && tt_b_ - tt_a_ <= tt_t_))                              \
            TT_FAIL("%s ~= %s: %.9g vs %.9g (tol %.3g)", #a, #b, tt_a_, tt_b_, tt_t_);       \
    } while (0)

#define TT_MEM_EQ(a, b, n)                                                  \
    do {                                                                    \
        tt_checks++;                                                        \
        if (memcmp((a), (b), (n)) != 0) {                                   \
            fprintf(stderr, "  got: ");                                     \
            for (size_t i_ = 0; i_ < (size_t)(n); i_++)                     \
                fprintf(stderr, "%02X ", ((const uint8_t *)(a))[i_]);       \
            fprintf(stderr, "\n  exp: ");                                   \
            for (size_t i_ = 0; i_ < (size_t)(n); i_++)                     \
                fprintf(stderr, "%02X ", ((const uint8_t *)(b))[i_]);       \
            fprintf(stderr, "\n");                                          \
            TT_FAIL("memory mismatch %s vs %s", #a, #b);                    \
        }                                                                   \
    } while (0)

static inline int tt_run_one(const tt_case_t *c) {
    tt_checks = 0;
    if (setjmp(tt_jmp) == 0) {
        c->fn();
        printf("ok   %s (%d checks)\n", c->name, tt_checks);
        return 0;
    }
    printf("FAIL %s\n", c->name);
    return 1;
}

static inline int tt_main(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "--list") == 0) {
        for (int i = 0; i < tt_ncases; i++) printf("%s\n", tt_cases[i].name);
        return 0;
    }
    int failed = 0, ran = 0;
    for (int i = 0; i < tt_ncases; i++) {
        if (argc > 1 && strcmp(argv[1], tt_cases[i].name) != 0) continue;
        ran++;
        failed += tt_run_one(&tt_cases[i]);
    }
    if (ran == 0) {
        fprintf(stderr, "no test matched\n");
        return 2;
    }
    printf("%d/%d passed\n", ran - failed, ran);
    return failed ? 1 : 0;
}

#define TT_MAIN() \
    int main(int argc, char **argv) { return tt_main(argc, argv); }

/* Deterministic PRNG for property tests. */
typedef struct {
    uint64_t s;
} tt_rng_t;
static inline uint32_t tt_rand(tt_rng_t *r) {
    r->s ^= r->s << 13;
    r->s ^= r->s >> 7;
    r->s ^= r->s << 17;
    return (uint32_t)(r->s >> 16);
}
static inline double tt_randf(tt_rng_t *r, double lo, double hi) {
    return lo + (hi - lo) * ((double)tt_rand(r) / 4294967295.0);
}
static inline int tt_randi(tt_rng_t *r, int lo, int hi) { /* inclusive */
    return lo + (int)(tt_rand(r) % (uint32_t)(hi - lo + 1));
}

#endif
