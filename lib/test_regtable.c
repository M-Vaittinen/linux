// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit test for the linear_ranges helper.
 *
 * Copyright (C) 2020, ROHM Semiconductors.
 * Author: Matti Vaittinen <matti.vaittien@fi.rohmeurope.com>
 */
#include <kunit/test.h>

#include <linux/regtable.h>

#define REGTABLE_TEST_CMP_IDX 3
/*
 * Please, keep arrays sorted.
 * The test assumes value[i - 1] < value[i] < value[i + 1]
 */
static int g_reg_vals[] = { 5, 10, 15, 20, 25, 30 };
static int g_vals[] = { 50, 100, 150, 200, 250, 300 };
static u64 g_vals64[] = { 50, 100, 150, 200, 250, 300 };
static const struct reg_val_table g_test_table = REGVAL_TABLE(g_reg_vals, g_vals);
static const struct reg_val_table_64 g_test_table64 = REGVAL_TABLE64(g_reg_vals, g_vals64);

#define REGTABLE_TEST_VAL_NOT_FOUND	12
#define REGTABLE_TEST_REG_NOT_FOUND	12

static void regtable_test_cmp_find_invalid(struct kunit *test)
{
	int ret, invalid, dummy, *int_null = NULL;
	struct reg_val_table *null_tbl = NULL;

	invalid = g_vals[ARRAY_SIZE(g_vals) - 1];
	ret = regtable_find_greater_than_val(&g_test_table, invalid, &dummy,
					     NULL);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	invalid = g_vals[0];
	ret = regtable_find_smaller_than_val(&g_test_table, invalid, &dummy,
					     NULL);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	ret = regtable_find_smaller_than_val(NULL, 0, NULL, NULL);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	ret = regtable_find_greater_than_val(NULL, 0, NULL, NULL);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	ret = regtable_find_val(null_tbl, 0, &dummy);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	ret = regtable_find_val(&g_test_table, 0, int_null);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	ret = regtable_find_reg(null_tbl, 0, &dummy);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);

	ret = regtable_find_reg(&g_test_table, 0, int_null);
	KUNIT_EXPECT_EQ(test, -EINVAL, ret);
}

static void regtable_test_cmp_find_bigger(struct kunit *test)
{
	int ret, found_val, found_reg, expect_idx;

	ret = regtable_find_greater_than_val(&g_test_table,
					     g_vals[REGTABLE_TEST_CMP_IDX] - 1,
					     &found_reg, &found_val);
	KUNIT_EXPECT_EQ(test, 0, ret);
	KUNIT_EXPECT_EQ(test, found_reg, g_reg_vals[REGTABLE_TEST_CMP_IDX]);
	KUNIT_EXPECT_EQ(test, found_val, g_vals[REGTABLE_TEST_CMP_IDX]);

	ret = regtable_find_greater_than_val(&g_test_table,
					     g_vals[REGTABLE_TEST_CMP_IDX],
					     &found_reg, &found_val);
	KUNIT_EXPECT_EQ(test, 0, ret);
	/* arrays are sorted so greater than val[i] is val[i + 1] */
	expect_idx = REGTABLE_TEST_CMP_IDX + 1;
	KUNIT_EXPECT_EQ(test, found_reg, g_reg_vals[expect_idx]);
	KUNIT_EXPECT_EQ(test, found_val, g_vals[expect_idx]);
}

static void regtable_test_cmp_find_smaller(struct kunit *test)
{
	int ret, found_val, found_reg, expect_idx;

	ret = regtable_find_smaller_than_val(&g_test_table,
					     g_vals[REGTABLE_TEST_CMP_IDX] + 1,
					     &found_reg, &found_val);
	KUNIT_EXPECT_EQ(test, 0, ret);
	KUNIT_EXPECT_EQ(test, found_reg, g_reg_vals[REGTABLE_TEST_CMP_IDX]);
	KUNIT_EXPECT_EQ(test, found_val, g_vals[REGTABLE_TEST_CMP_IDX]);

	ret = regtable_find_smaller_than_val(&g_test_table,
					     g_vals[REGTABLE_TEST_CMP_IDX],
					     &found_reg, &found_val);
	KUNIT_EXPECT_EQ(test, 0, ret);
	/* arrays are sorted so smaller than val[i] is val[i - 1] */
	expect_idx = REGTABLE_TEST_CMP_IDX - 1;
	KUNIT_EXPECT_EQ(test, found_reg, g_reg_vals[expect_idx]);
	KUNIT_EXPECT_EQ(test, found_val, g_vals[expect_idx]);
}

static void regtable_test_find(struct kunit *test)
{
	int *testreg, *testval;
	u64 *testval64, found64;
	int ret, i, found;

	for (i = 0; i < g_test_table.num_vals; i++) {
		testreg = &g_reg_vals[i];
		testval = &g_vals[i];
		testval64 = &g_vals64[i];
		ret = regtable_find_val(&g_test_table, *testreg, &found);
		KUNIT_EXPECT_EQ(test, 0, ret);
		KUNIT_EXPECT_EQ(test, found, *testval);

		ret = regtable_find_reg(&g_test_table,  *testval, &found);
		KUNIT_EXPECT_EQ(test, 0, ret);
		KUNIT_EXPECT_EQ(test, found, *testreg);

		ret = regtable_find_val(&g_test_table64, *testreg, &found64);
		KUNIT_EXPECT_EQ(test, 0, ret);
		KUNIT_EXPECT_EQ(test, found64, *testval64);

		ret = regtable_find_reg(&g_test_table64,  *testval64, &found);
		KUNIT_EXPECT_EQ(test, 0, ret);
		KUNIT_EXPECT_EQ(test, found, *testreg);
	}

	ret = regtable_find_val(&g_test_table, REGTABLE_TEST_REG_NOT_FOUND,
				&found);
	KUNIT_EXPECT_NE(test, 0, ret);

	ret = regtable_find_reg(&g_test_table, REGTABLE_TEST_VAL_NOT_FOUND,
				&found);
	KUNIT_EXPECT_NE(test, 0, ret);
}

static struct kunit_case regval_test_cases[] = {
	KUNIT_CASE(regtable_test_find),
	KUNIT_CASE(regtable_test_cmp_find_smaller),
	KUNIT_CASE(regtable_test_cmp_find_bigger),
	KUNIT_CASE(regtable_test_cmp_find_invalid),
	{},
};

static struct kunit_suite range_test_module = {
	.name = "register-val-table-test",
	.test_cases = regval_test_cases,
};

kunit_test_suites(&range_test_module);

MODULE_LICENSE("GPL");
