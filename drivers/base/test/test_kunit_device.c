// SPDX-License-Identifier: GPL-2.0
/*
 * These helpers have been extracted from drm test code at
 * drm_kunit_helpers.c which was authored by
 * Maxime Ripard <maxime@cerno.tech>
 */

#include <linux/device.h>
#include <linux/platform_device.h>

#include <kunit/test-helpers.h>

#define KUNIT_DEVICE_NAME	"test-kunit-mock-device"

static int fake_probe(struct platform_device *pdev)
{
	return 0;
}

static int fake_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver fake_platform_driver = {
	.probe	= fake_probe,
	.remove	= fake_remove,
	.driver = {
		.name	= KUNIT_DEVICE_NAME,
	},
};

/**
 * test_kunit_helper_alloc_device - Allocate a mock device for a KUnit test
 * @test: The test context object
 *
 * This allocates a fake struct &device to create a mock for a KUnit
 * test. The device will also be bound to a fake driver. It will thus be
 * able to leverage the usual infrastructure and most notably the
 * device-managed resources just like a "real" device.
 *
 * Callers need to make sure test_kunit_helper_free_device() on the
 * device when done.
 *
 * Returns:
 * A pointer to the new device, or an ERR_PTR() otherwise.
 */
struct device *test_kunit_helper_alloc_device(struct kunit *test)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&fake_platform_driver);
	KUNIT_ASSERT_EQ(test, ret, 0);

	pdev = platform_device_alloc(KUNIT_DEVICE_NAME, PLATFORM_DEVID_NONE);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, pdev);

	ret = platform_device_add(pdev);
	KUNIT_ASSERT_EQ(test, ret, 0);

	return &pdev->dev;
}
EXPORT_SYMBOL_GPL(test_kunit_helper_alloc_device);

/**
 * test_kunit_helper_free_device - Frees a mock device
 * @test: The test context object
 * @dev: The device to free
 *
 * Frees a device allocated with test_kunit_helper_alloc_device().
 */
void test_kunit_helper_free_device(struct kunit *test, struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);
	platform_driver_unregister(&fake_platform_driver);
}
EXPORT_SYMBOL_GPL(test_kunit_helper_free_device);

MODULE_AUTHOR("Matti Vaittinen <mazziesaccount@gmail.com>");
MODULE_LICENSE("GPL");
