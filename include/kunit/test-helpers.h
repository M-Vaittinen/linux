// SPDX-License-Identifier: GPL-2.0

#ifndef __KUNIT_TEST_HELPERS__
#define __KUNIT_TEST_HELPERS__

#include <kunit/test.h>

struct device;

struct device *test_kunit_helper_alloc_device(struct kunit *test);
void test_kunit_helper_free_device(struct kunit *test, struct device *dev);

#endif
