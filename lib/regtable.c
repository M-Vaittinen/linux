// SPDX-License-Identifier: GPL-2.0
/*
 * Simple helpers to do key <=> value look-up from tables.
 *
 * Copyright 2020 ROHM Semiconductors
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/regtable.h>

/**
 * regtable_find_val - find a value matching register setting
 *
 * Search given table for value mathcing a register setting.
 *
 * @table:	Table from which the register setting - value pairs are
 * 		searched.
 * @reg:	Register value for which the matching physical value is
 * 		searched.
 * @val:	Pointer to location where the found value will be stored.
 *
 * returns:	0 on success, negative errno if table is invalid or match is
 * 		not found.
int regtable_find_val(const struct reg_val_table *table, int reg, int *val)
{
	int i;

	if (!table || !val)
		return -EINVAL;

	for (i = 0; i < table->num_vals; i++) {
		if (table->reg_vals[i] == reg) {
			*val = table->vals[i];

			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(regtable_find_val);
 */

/**
 * regtable_find_reg - find a register setting matching given value.
 *
 * Search given table for a register setting matching a value.
 *
 * @table:	Table from which the register setting - value pairs are
 * 		searched.
 * @val:	Value for which the matching register setting is searched.
 * @reg:	Pointer to location where the found register value will be
 * 		stored.
 *
 * returns:	0 on success, negative errno if table is invalid or match is
 * 		not found.
int regtable_find_reg(const struct reg_val_table *table, int val, int *reg)
{
	int i;

	if (!table || !reg)
		return -EINVAL;

	for (i = 0; i < table->num_vals; i++) {
		if (table->vals[i] == val) {
			*reg = table->reg_vals[i];

			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(regtable_find_reg);
 */

/**
 * regtable_find_greater_than_val - find the closest greater val and reg
 *
 * Search given table for the smallest value which is still greater than
 * the given value. Both the found value and corresponding register
 * setting are returned unless given pointers are NULL.
 *
 * @table:	Table from which the register setting - value pairs are
 *		searched.
 * @val_cmp:	Value to which the values stored in table are compared to.
 * @reg:	NULL or pointer to location where the matching register
 *		setting value will be stored.
 * @val:	NULL or pointer to location where the found value will be
 *		stored.
 *
 * returns:	0 on success, negative errno if table is invalid or match is
 * 		not found.
 */
int regtable_find_greater_than_val(const struct reg_val_table *table, int val_cmp,
				   int *reg, int *val)
{
	int i, candidate_val, candidate_reg;
	bool found = false;

	if (!table || (!reg && !val))
		return -EINVAL;

	for (i = 0; i < table->num_vals; i++) {
		if (table->vals[i] > val_cmp) {
			/* Take smallest candidate */
			if (!found || candidate_val > table->vals[i]) {
				candidate_val = table->vals[i];
				candidate_reg = table->reg_vals[i];
				found = true;
			}
		}
	}
	if (!found)
		return -EINVAL;

	if (reg)
		*reg = candidate_reg;
	if (val)
		*val = candidate_val;

	return 0;
}
EXPORT_SYMBOL_GPL(regtable_find_greater_than_val);

/**
 * regtable_find_smaller_than_val - find the closest smaller val and reg
 *
 * Search given table for the greatest value which is still smaller than
 * the given value. Both the found value and corresponding register
 * setting are returned unless given pointers are NULL.
 *
 * @table:	Table from which the register setting - value pairs are
 *		searched.
 * @val_cmp:	Value to which the values stored in table are compared to.
 * @reg:	NULL or pointer to location where the matching register
 *		setting value will be stored.
 * @val:	NULL or pointer to location where the found value will be
 *		stored.
 *
 * returns:	0 on success, negative errno if table is invalid or match is
 * 		not found.
 */
int regtable_find_smaller_than_val(const struct reg_val_table *table,
				   int val_cmp, int *reg, int *val)
{
	int i, candidate_val, candidate_reg;
	bool found = false;

	if (!table || (!reg && !val))
		return -EINVAL;

	for (i = 0; i < table->num_vals; i++) {
		/* Take biggest candidate */
		if (table->vals[i] < val_cmp) {
			if (!found || candidate_val < table->vals[i]) {
				candidate_val = table->vals[i];
				candidate_reg = table->reg_vals[i];
				found = true;
			}
		}
	}
	if (!found)
		return -EINVAL;

	if (reg)
		*reg = candidate_reg;
	if (val)
		*val = candidate_val;

	return 0;
}
EXPORT_SYMBOL_GPL(regtable_find_smaller_than_val);

MODULE_DESCRIPTION("register-value table helpers");
MODULE_LICENSE("GPL");
