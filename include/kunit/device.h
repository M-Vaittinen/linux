/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __KUNIT_DEVICE_H__
#define __KUNIT_DEVICE_H__

#include <kunit/test.h>

struct device;

/* Register a new device against a KUnit test. */
struct device *kunit_device_register(struct kunit *test, const char *name);
/*
 * Unregister a device created by kunit_device_register() early (i.e.,
 * before test cleanup).
 */
void kunit_device_unregister(struct kunit *test, struct device *dev);

#endif
