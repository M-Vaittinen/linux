/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2023 ROHM Semiconductors */

#ifndef REG_VAL_TABLES_H
#define REG_VAL_TABLES_H

struct reg_val_table {
	int type;
	int *reg_vals;
	int *vals;
	int num_vals;
};

struct reg_val_table_64 {
	int type;
	int *reg_vals;
	u64 *vals;
	int num_vals;
};

enum {
	REGVAL_TABLE_TYPE_INVALID,
	REGVAL_TABLE_TYPE_INT,
	REGVAL_TABLE_TYPE64,
	#define REGVAL_TABLE_NUM_VALID_TYPES REGVAL_TABLE_TYPE64
};

#define __REGVAL_TABLE(_regvals, _vals, _type) {						\
	.type = (_type),							\
	.reg_vals = (_regvals),							\
	.vals = (_vals),							\
	.num_vals = ARRAY_SIZE(_vals) +						\
		    BUILD_BUG_ON_ZERO(ARRAY_SIZE(_regvals) != ARRAY_SIZE(_vals)), \
};

#define REGVAL_TABLE(_regvals, _vals) __REGVAL_TABLE(_regvals, _vals, \
						     REGVAL_TABLE_TYPE_INT)
#define REGVAL_TABLE64(_regvals, _vals) __REGVAL_TABLE(_regvals, _vals, \
						       REGVAL_TABLE_TYPE64)

/**
 * regtable_find_val - find a value matching register setting
 *
 * Search given table for value mathcing a register setting.
 *
 * @table:	Pointer to a table from which the register setting - value
 * 		pairs arei searched.
 * @reg:	Register value for which the matching physical value is
 * 		searched.
 * @val:	Pointer to location where the found value will be stored.
 *
 * returns:	0 on success, negative errno if table is invalid or match is
 * 		not found.
 */
#define regtable_find_val(_table, _reg, _val)				\
({									\
	int _ret = -EINVAL, _i;						\
									\
	if ((void *)(_table) != (void *)NULL && (void *)(_val) != (void *)NULL) {	\
		for (_i = 0; _i < (_table)->num_vals; _i++) {		\
			if ((_table)->reg_vals[_i] == (_reg)) {		\
				*(_val) = (_table)->vals[_i];		\
				_ret = 0;				\
			}						\
		}							\
	}								\
	_ret;								\
})

/**
 * regtable_find_reg - find a register setting matching given value.
 *
 * Search given table for a register setting matching a value.
 *
 * @table:	Pointer to a table from which the register setting - value
 * 		pairs are searched.
 * @val:	Value for which the matching register setting is searched.
 * @reg:	Pointer to location where the found register value will be
 * 		stored.
 *
 * returns:	0 on success, negative errno if table is invalid or match is
 * 		not found.
 */
#define regtable_find_reg(_table, _val, _reg)				\
({									\
	int _i, _ret = -EINVAL;						\
									\
	if ((void *)(_table) != (void *)NULL && (void *)(_reg) != (void *)NULL) {			\
		for (_i = 0; _i < (_table)->num_vals; _i++) {		\
			if ((_table)->vals[_i] == (_val)) {		\
				*(_reg) = (_table)->reg_vals[_i];	\
									\
				_ret = 0;				\
			}						\
		}							\
	}								\
									\
	_ret;								\
})

/*
 * int regtable_find_val(const struct reg_val_table *table, int reg, int *val);
int regtable_find_reg(const struct reg_val_table *table, int val, int *reg);
*/
int regtable_find_greater_than_val(const struct reg_val_table *table, int val_cmp,
				   int *reg, int *val);
int regtable_find_smaller_than_val(const struct reg_val_table *table, int val_cmp,
				   int *reg, int *val);

#endif
