/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023 ROHM Semiconductors */

#ifndef REG_VAL_TABLES_H
#define REG_VAL_TABLES_H

struct reg_val_table {
	int *reg_vals;
	int *vals;
	int num_vals;
};

#define REGVAL_TABLE(_regvals, _vals) {						\
	.reg_vals = (_regvals),							\
	.vals = (_vals),							\
	.num_vals = ARRAY_SIZE(_vals) +						\
		    BUILD_BUG_ON_ZERO(ARRAY_SIZE(_regvals) != ARRAY_SIZE(_vals)), \
};

int regtable_find_val(const struct reg_val_table *table, int reg, int *val);
int regtable_find_reg(const struct reg_val_table *table, int val, int *reg);
int regtable_find_greater_than_val(const struct reg_val_table *table, int val_cmp,
				   int *reg, int *val);
int regtable_find_smaller_than_val(const struct reg_val_table *table, int val_cmp,
				   int *reg, int *val);

#endif
