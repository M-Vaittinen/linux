// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2025 Matti Vaittinen

#define VERSION "1"

#include <ctype.h>
#include <getopt.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <mysql/mysql.h>
#include <ncurses/ncurses.h>
#include <ncurses/form.h>
#include <ncurses/menu.h>
#include <assert.h>
#include <signal.h>

#define OPTSTRING "E:vh?"
#define KEY_ESC 27
/* ENDKEY proceeds to write card */
#define ENDKEY KEY_F(1)
/* QUITKEY quits program */
#define QUITKEY KEY_F(2)

#define ARRAY_SIZE(foo) ( sizeof(foo) / sizeof(foo[0]) )

enum {
	EATTACK,
	EDEFEND,
	EDEBT,
	EGATHER,
	EENDURE,
	EDESTROY,
	ECURSE,
	ENONE
};

#define MASK_ATTACK 1
#define MASK_DEFENCE (1 << 1)
#define MASK_DEBT (1 << 2)
#define MASK_GATHERS (1 << 3)
#define MASK_ENDURES (1 << 4)
#define MASK_DESTROYS (1 << 5)
#define MASK_CURSES (1 << 6)

static char *bool_menu_opts[] =
{
	[EATTACK] = "hyokkaa",
	[EDEFEND] = "puolustaa",
	[EDEBT] = "on velka",
	[EGATHER] = "keraa",
	[EENDURE] = "kestaa",
	[EDESTROY] = "tuhoaa",
	[ECURSE] = "kiroaa",
	[ENONE] = " ",
};

/* These MUST match the type IDs in the database */
enum {
	PRIZETYPE_NORMAL = 1,
       	PRIZETYPE_DEBT = 2,
};

static MYSQL *g_conn;

static struct option long_options[] =
{
	{"exclude-chapter", required_argument, 0, 'E'},
	{"version",  no_argument, 0, 'v'},
	{"help",  no_argument, 0, 'h'},
	{0,0,0,0}
};

struct card {
	struct card *next;
	unsigned int prize;
	char sequel[255];
	char name[255];
};

static struct card g_cards[1000];
static int g_num_cards = 0;

struct exlist {
	struct exlist *next;
	char *name;
};

static struct exlist g_exhead;

static WINDOW *win_body, *win_menu_multi, *win_menu_type, *win_sequel, *win_form;
/*
|-------------------------------|
|card name, guide		|
|-------------------------------|
|		    |		|
|	multi	    |	types	|
|		    |		|
|		    |		|
|-------------------|-----------|
| prize   |			|
| tuhina  |	sequels		|
|-------------------------------|
*/
#define WIN_BASE_HEIGHT		LINES
#define WIN_BASE_WIDTH		COLS

#define FREE_INSIDE_BASE_Y	(LINES - 3) /* available when base borders and card name are occupied */
#define FREE_INSIDE_BASE_X	(COLS - 2) /* available when base borders are occupied */

#define WIN_MENU_MULTI_START_Y	2 /* basewin border + card name/guide line */
#define WIN_MENU_MULTI_START_X	1 /* basewin border */
#define WIN_MENU_MULTI_HEIGHT	(FREE_INSIDE_BASE_Y - (FREE_INSIDE_BASE_Y / 3)) /* 1/3  of available */
#define WIN_MENU_MULTI_WIDTH	(FREE_INSIDE_BASE_X - FREE_INSIDE_BASE_X / 3)

#define WIN_MENU_TYPES_START_Y	(WIN_MENU_MULTI_START_Y)
#define WIN_MENU_TYPES_START_X	(WIN_MENU_MULTI_START_X + WIN_MENU_MULTI_WIDTH + 1)
#define WIN_MENU_TYPES_HEIGHT	WIN_MENU_MULTI_HEIGHT
#define WIN_MENU_TYPES_WIDTH	(FREE_INSIDE_BASE_X - WIN_MENU_MULTI_WIDTH)

/*
 * The longest form will be:
 * " Tuhinakerroin: XX "
 * which requires 19 characters. Let's add few more in case we want borders or some such
 * TODO: Should we have values below labels?
 */
#define WIN_FORMS_WIDTH		(25)
#define WIN_FORMS_HEIGHT	(FREE_INSIDE_BASE_Y - WIN_MENU_MULTI_HEIGHT)
#define WIN_FORMS_START_Y	(WIN_MENU_MULTI_START_Y + WIN_MENU_MULTI_HEIGHT + 1)
#define WIN_FORMS_START_X	WIN_MENU_MULTI_START_X

#define WIN_SEQUEL_WIDTH	(FREE_INSIDE_BASE_X - WIN_FORMS_WIDTH)
#define WIN_SEQUEL_HEIGHT	WIN_FORMS_HEIGHT
#define WIN_SEQUEL_START_Y	WIN_FORMS_START_Y
#define WIN_SEQUEL_START_X	(WIN_FORMS_START_X + WIN_FORMS_WIDTH + 1)

static void err_out(const char *reason, int err)
{
	endwin();
	printf("%s %d\n", reason, err);

	exit(err);
}

static void add_menu(MENU **menu, char *options[], int num_options)
{
	ITEM **items;
	int i;

	items = calloc(num_options + 1, sizeof(ITEM *));

	for (i = 0; i < num_options; i++) {
		items[i] = new_item(options[i], options[i]);
		if (!items[i]) {
			endwin();
			printf("%s %s\n", options[i], options[i]);
			exit(-1);
			err_out("item alloc failed\n", ENOMEM);
		}
		item_opts_on(items[i], O_SELECTABLE);
	}
	items[i] = NULL;
	*menu = new_menu(items);
}


static void init()
{
	/* Ncurses inits */
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);

	win_body = newwin(0, 0, 0, 0);
	if (!win_body)
		err_out("newwin() FAIL\n", errno);

	/* Menu window will be 1/3 of the base window, minus 2 rows for the card name and instructions */
	win_menu_multi = derwin(win_body, WIN_MENU_MULTI_HEIGHT, WIN_MENU_MULTI_WIDTH, WIN_MENU_MULTI_START_Y, WIN_MENU_MULTI_START_X);
	assert(win_menu_multi != NULL);
	box(win_menu_multi, 0, 0);

	win_menu_type = derwin(win_body, WIN_MENU_TYPES_HEIGHT, WIN_MENU_TYPES_WIDTH, WIN_MENU_TYPES_START_Y, WIN_MENU_TYPES_START_X);
	assert(win_menu_multi != NULL);
	box(win_menu_type, 0, 0);

	win_form = derwin(win_body, WIN_FORMS_HEIGHT, WIN_FORMS_WIDTH, WIN_FORMS_START_Y, WIN_FORMS_START_X);
	assert(win_form != NULL);
	box(win_form, 0, 0);

	win_sequel = derwin(win_body, WIN_SEQUEL_HEIGHT, WIN_SEQUEL_WIDTH, WIN_SEQUEL_START_Y, WIN_SEQUEL_START_X);
	assert(win_sequel != NULL);
	box(win_sequel, 0, 0);

	refresh();
	wrefresh(win_body);

	/* Init lists */
	g_exhead.next = NULL;
}

static void trim(char *buf)
{
	int len;

	len = strlen(buf);
	while (len--)
		if (isspace(buf[len]))
			buf[len] = 0;
		else
			break;
}

static int exlist_add(char *chapter)
{
	struct exlist *e;

	if (!chapter) {
		printf("-E with no chapter?\n");
		return EINVAL;
	}

	e = malloc(sizeof(*e));
	if (!e)
		return ENOMEM;

	e->name = strdup(chapter);
	trim(e->name);
	e->next = g_exhead.next;
	g_exhead.next = e;

	return 0;
}

static void add_card(struct card *c)
{
	struct card *new = &g_cards[g_num_cards];
	struct exlist *ex;

	for (ex = g_exhead.next; ex; ex = ex->next)
		if (!strcmp(ex->name, c->sequel))
			return;

	memcpy(new, c, sizeof(*c));
	g_num_cards++;
}

struct seq_list {
	struct seq_list *next;
	char *name;
	int num;
};

static int read_cards()
{
	FILE *cf;
	int ret;
 
	cf = fopen("cards-new.txt","r");
	if (!cf) {
		perror("cards-new.txt");
		return EINVAL;
	}

	while(1) {
		struct card c;

		ret = fscanf(cf,"%[^#\n]## %u ## %[^\n]", &c.name[0], &c.prize, &c.sequel[0]);
		if (ret == EOF)
			return 0;

		/*
		 * if the line could not be parsed the \n is left to the file.
		 * We need to read it to avoid looping endlessly
		 * */
		if (ret == 0)
			fscanf(cf, "\n");

		if (ret == 3) {
			trim(c.name);
			trim(c.sequel);
			add_card(&c);
		}
	}

	return -1;
}

static void print_usage()
{
	printf("\t-E --exclude-chapter <CHAPTER>\n");
	printf("\t-v --version\n");
	printf("\t-h --help\n");
}

enum {
	NUM_CHEAP,
	NUM_MID,
	NUM_EXP
};

static int already_added(struct card *c)
{
	char query[255];
	MYSQL_RES *res;
	int ret;

	ret = snprintf(query, 254, "SELECT * FROM cards WHERE name = '%s' LIMIT 1", c->name);
	if (ret >= 254) {
		err_out("already_added(): Query too long\n", ERANGE);
	}

	/* send SQL query */
	if (mysql_query(g_conn, query)) {
		fprintf(stderr, "%s: %s\n", query, mysql_error(g_conn));
		return -1;
	}

	res = mysql_store_result(g_conn);
	if (!res) {
		fprintf(stderr, "mysql_store_result(): %s\n", mysql_error(g_conn));
		return -1;
	}
	ret = mysql_num_rows(res);

	return ret;
}
struct expansion {
	int id;
	const char *name;
};

char *strtolow(char *str)
{
	char *p = strdup(str);
	int i;

	if (!p)
		err_out("strdup failed\n", ENOMEM);

	for (i = 0; str[i]; i++)
		p[i] = tolower(str[i]);

	return p;
}

static int get_expansion(unsigned int *id, struct card *c)
{
	const char *query = "SELECT * FROM expansion";
	MYSQL_RES *res;
	MYSQL_ROW row;
	int num_exp, i;
	bool found;

	if (mysql_query(g_conn, query)) {
		fprintf(stderr, "%s: %s\n", query, mysql_error(g_conn));
		return -1;
	}

	res = mysql_store_result(g_conn);
	if (!res) {
		fprintf(stderr, "mysql_store_result(): %s\n", mysql_error(g_conn));
		return -1;
	}
	num_exp = mysql_num_rows(res);
	if (num_exp < 0 || num_exp > 50) {
		err_out("Unexpected num of expansions\n", num_exp);
		return -1;
	}

	for (i = 0; i < num_exp; i++) {
		char *chkptr;
		char *cmp1, *cmp2;

	       	row = mysql_fetch_row(res);
		cmp1 = strtolow(c->sequel);
		cmp2 = strtolow(row[1]);
		if (!strcmp(cmp1, cmp2)) {
			*id = strtoul(row[0], &chkptr, 0);
			found = true;
			free(cmp1);
			free(cmp2);
			break;
		}
		free(cmp1);
		free(cmp2);
	}
	if (!found)
		err_out("Unknown expansion\n", EINVAL);
	mysql_free_result(res);

	return !found;
}

#define LARGE_QUERY_SIZE 2048
static char g_large_query[LARGE_QUERY_SIZE + 1];

static void add_card_to_db(const char *name, int exp, int type, int prizetype,
			   unsigned int prize, unsigned int multimask, unsigned int tuhisee)
{
	const char *query = "INSERT INTO cards SET name='%s', expansion_id='%u', type_id='%u', prizetype_id='%u', prize='%u', attack='%u', defence='%u', endure='%u', gather='%u', destroy='%u', tuhinakerroin='%u', curse='%u'";

	snprintf(&g_large_query[0], LARGE_QUERY_SIZE, query, name, exp, type, prizetype, prize, !!(multimask & MASK_ATTACK), !!(multimask & MASK_DEFENCE), !!(multimask & MASK_ENDURES), !!(multimask & MASK_GATHERS), !!(multimask & MASK_DESTROYS), tuhisee, !!(multimask & MASK_CURSES));
	g_large_query[LARGE_QUERY_SIZE] = '\0';

	if (mysql_query(g_conn, &g_large_query[0]))
		err_out(mysql_error(g_conn), EINVAL);
}

enum {
	DUAL_CARD_TOP = 1,
	DUAL_CARD_BOTTOM,
};

void reference_top_bot(char *top, char *bot)
{
	const char *query = "SELECT id FROM cards WHERE name='%s' LIMIT 1";
	const char *id_update_query = "UPDATE cards SET %s=%s WHERE id=%s LIMIT 1";
	int i;
	char *names[] = { top, bot };
	char *ids[2];
	MYSQL_RES *res[2];
	MYSQL_ROW row;

	for (i = 0; i < 2; i++) {
		snprintf(&g_large_query[0], LARGE_QUERY_SIZE, query, names[i]);
		g_large_query[LARGE_QUERY_SIZE] = '\0';

		if (mysql_query(g_conn, g_large_query)) {
			endwin();
			printf("%s: %s\n", g_large_query, mysql_error(g_conn));
			printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
			exit(1);
		}

		res[i] = mysql_store_result(g_conn);
		if (!res[i]) {
			endwin();
			printf("mysql_store_result(): %s\n", mysql_error(g_conn));
			printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
			exit(1);
		}
		if (1 != mysql_num_rows(res[i])) {
			endwin();
			printf("Unexpected num of IDs returned. Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
			exit(1);
		}
		row = mysql_fetch_row(res[i]);
		ids[i] = row[0];
	}
	snprintf(&g_large_query[0], LARGE_QUERY_SIZE, id_update_query, "dual_top_of_id", ids[1], ids[0]);
	if (mysql_query(g_conn, g_large_query)) {
		endwin();
		printf("%s: %s\n", query, mysql_error(g_conn));
		printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
		exit(1);
	}
	snprintf(&g_large_query[0], LARGE_QUERY_SIZE, id_update_query, "dual_below_id", ids[0], ids[1]);
	if (mysql_query(g_conn, g_large_query)) {
		endwin();
		printf("%s: %s\n", query, mysql_error(g_conn));
		printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
		exit(1);
	}
	for (i = 0; i < 2; i++)
		mysql_free_result(res[i]);
};

enum {
	FIELD_TUHINA_LABEL,
	FIELD_TUHINA_DATA,
	FIELD_PRIZE_LABEL,
	FIELD_PRIZE_DATA,
	NUM_FIELDS
};

struct card_window {
	FORM *c_tuh_prize_form;
	FIELD *field[NUM_FIELDS + 1];
	MENU *c_booleans;
	MENU *c_types;
};

static struct card_window g_card_window;


void init_boolean_menu()
{
	int rows, cols;

	add_menu(&g_card_window.c_booleans, bool_menu_opts, ARRAY_SIZE(bool_menu_opts));
	menu_opts_off(g_card_window.c_booleans, O_ONEVALUE);
	scale_menu(g_card_window.c_booleans, &rows, &cols);
	set_menu_win(g_card_window.c_booleans, win_menu_multi);
	set_menu_sub(g_card_window.c_booleans, derwin(win_menu_multi, rows + 2, cols + 2, 2, 2));
	set_menu_mark(g_card_window.c_booleans, "*");
	menu_opts_off(g_card_window.c_booleans, O_SHOWDESC);
	post_menu(g_card_window.c_booleans);
	menu_driver(g_card_window.c_booleans, REQ_LAST_ITEM);
}

struct card_type {
	char *name;
	char *id;
};

int get_card_types(struct card_type **ctypes, int *num_types)
{
	const char *query = "SELECT * FROM cardtype";
	MYSQL_RES *res;
	MYSQL_ROW row;
	int i;

	if (mysql_query(g_conn, query)) {
		fprintf(stderr, "%s: %s\n", query, mysql_error(g_conn));
		return -1;
	}

	res = mysql_store_result(g_conn);
	if (!res) {
		fprintf(stderr, "mysql_store_result(): %s\n", mysql_error(g_conn));
		return -1;
	}
	*num_types = mysql_num_rows(res);
	if (*num_types < 0 || *num_types > 50) {
		fprintf(stderr, "Unexpected num of types (%d)\n", *num_types);
		return -EINVAL;
	}
	*ctypes = calloc(*num_types, sizeof(struct card_type));
	if (!*ctypes)
		return -ENOMEM;

	for (i = 0; i < *num_types; i++) {
		row = mysql_fetch_row(res);
		(*ctypes)[i].id = strdup(row[0]);
		(*ctypes)[i].name = strdup(row[1]);
	}
	mysql_free_result(res);

	return 0;
}

int g_default_type_index = 0;
int *g_type_ids;
int g_num_types;

int init_type_menu()
{
	struct card_type *ctypes;
	int ret, num_types;
	ITEM **items;
	int rows, cols;
	int i;

	ret = get_card_types(&ctypes, &num_types);
	if (ret)
		return ret;

	items = calloc(num_types + 1, sizeof(ITEM *));
	if (!items)
		return -ENOMEM;

	g_type_ids = calloc(num_types, sizeof(*g_type_ids));
	if (!g_type_ids)
		return -ENOMEM;

	g_num_types = num_types;

	for (i = 0; i < num_types; i++) {
		if (!strcmp(ctypes[i].name, "toiminto"))
			g_default_type_index = i;
		items[i] = new_item(ctypes[i].name, ctypes[i].id);
		if (!items[i]) {
			endwin();
			printf("%s %s\n", ctypes[i].name, ctypes[i].id);
			exit(-1);
			err_out("item alloc failed\n", ENOMEM);
		}
		item_opts_on(items[i], O_SELECTABLE);
		g_type_ids[i] = atoi(ctypes[i].id);
	}
	items[i] = NULL;

	g_card_window.c_types = new_menu(items);

	scale_menu(g_card_window.c_types, &rows, &cols);
	set_menu_win(g_card_window.c_types, win_menu_type);
	set_menu_sub(g_card_window.c_types, derwin(win_menu_type, rows + 2, cols + 2, 2, 2));
	set_menu_mark(g_card_window.c_types, "*");
	menu_opts_off(g_card_window.c_types, O_SHOWDESC);
	post_menu(g_card_window.c_types);

	for (i = 0; i < g_default_type_index; i++)
		menu_driver(g_card_window.c_types, REQ_NEXT_ITEM);

	refresh();
	wrefresh(win_menu_type);

	return 0;
}

const char *tuhinalebel = "Tuhinakerroin:";
const char *prizelebel = "Card prize:";

int init_forms()
{
	int field_len, field_height = 1;
	int rows, cols;
	FORM *f;

	field_len = strlen(tuhinalebel) + 1;
	g_card_window.field[FIELD_TUHINA_LABEL] = new_field(field_height, field_len, 2, 2, 0, 0 );
	g_card_window.field[FIELD_TUHINA_DATA] = new_field(field_height, 3/* len */, 2 /* start Y */, field_len + 2 + 2 /* start X */, 0, 0);
	field_len = strlen(prizelebel) + 1;
	g_card_window.field[FIELD_PRIZE_LABEL] = new_field(field_height, field_len, 1, 2, 0 ,0);
	g_card_window.field[FIELD_PRIZE_DATA] = new_field(field_height, 3, 1, field_len + 2 + 2, 0, 0);
	g_card_window.field[NUM_FIELDS] = NULL;

	f = new_form(g_card_window.field);
	assert(f != NULL);

	scale_form(f, &rows, &cols);

	set_field_buffer(g_card_window.field[FIELD_TUHINA_LABEL], 0, tuhinalebel);
	set_field_buffer(g_card_window.field[FIELD_PRIZE_LABEL], 0, prizelebel);

	set_field_opts(g_card_window.field[FIELD_TUHINA_LABEL], O_VISIBLE | O_PUBLIC | O_AUTOSKIP);
	set_field_opts(g_card_window.field[FIELD_PRIZE_LABEL], O_VISIBLE | O_PUBLIC | O_AUTOSKIP);
	set_field_opts(g_card_window.field[FIELD_TUHINA_DATA], O_VISIBLE | O_PUBLIC | O_EDIT | O_ACTIVE);
	set_field_opts(g_card_window.field[FIELD_PRIZE_DATA], O_VISIBLE | O_PUBLIC | O_EDIT | O_ACTIVE);
	set_field_back(g_card_window.field[FIELD_PRIZE_DATA], A_UNDERLINE);
	set_field_back(g_card_window.field[FIELD_TUHINA_DATA], A_UNDERLINE);

	set_form_win(f, win_form);
	set_form_sub(f, derwin(win_form, rows+1, cols+1, 1, 1));
	g_card_window.c_tuh_prize_form = f;

	set_field_type(g_card_window.field[FIELD_TUHINA_DATA], TYPE_INTEGER, 2, 0, 10);
	set_field_type(g_card_window.field[FIELD_PRIZE_DATA], TYPE_INTEGER, 2, 0, 20);

	post_form(g_card_window.c_tuh_prize_form);

	refresh();
	wrefresh(win_body);
	wrefresh(win_form);

	return 0;
}

int init_card_menus()
{
	int ret;

	init_boolean_menu();
	ret = init_type_menu();
	if (ret)
		return ret;

	ret = init_forms();
	if (ret)
		return ret;

	return 0;
}

static void update_screen()
{
	wnoutrefresh(win_body);
	wnoutrefresh(win_form);
	wnoutrefresh(win_menu_type);
	wnoutrefresh(win_menu_multi);
	wnoutrefresh(win_sequel);
	doupdate();
}
const char *g_help_text = "arrows move, enter (de)select, ESC back, F1 write card, F2 exit";

static char *itoa(int i)
{
	int len, ret, tmp = i;
	char *arr;

	if (i < 0)
		len = 1;
	else
		len = 0;

	for (len++; tmp / 10; len++)
		tmp /= 10;

	arr = malloc(len + 1);
	if (!arr)
		err_out("malloc\n", ENOMEM);

	if (1 != (ret = sprintf(arr, "%d", i)))
		err_out("not integer\n", ret);

	return arr;
}

static void update_card_menusforms(struct card *c)
{
	char *clearline;
	int ret;

	clearline = malloc(WIN_BASE_WIDTH);
	memset(clearline, ' ', WIN_BASE_WIDTH);
	clearline[WIN_BASE_WIDTH - 1] = '\0';
	mvwprintw(win_body, 0, 0, "%s:", clearline);

	mvwprintw(win_body, 0, 0, "Card: %s:", c->name);
	mvwprintw(win_body, 0, WIN_BASE_WIDTH - strlen(g_help_text) - 1, "%s", g_help_text);
	mvwprintw(win_sequel, 1, 1, "Expansion: %s\n", c->sequel);

	/* leak mem */
	ret = set_field_buffer(g_card_window.field[FIELD_PRIZE_DATA], 0, itoa(c->prize));
	if (ret != E_OK)
		err_out("Set field buffer failed %d\n", ret);
}

static unsigned int currently_selected;

static void __menu_drv(WINDOW *w, MENU *m, bool toggle)
{
	int ch;

	box(w, ACS_DIAMOND, ACS_DIAMOND);
	wrefresh(w);
	while (KEY_ESC != (ch = getch())) {
		switch (ch) {
		case KEY_UP:
			menu_driver(m, REQ_UP_ITEM);
			break;
		case KEY_DOWN:
			menu_driver(m, REQ_DOWN_ITEM);
			break;
		case '\n':
		case KEY_ENTER:
			if (toggle)
				menu_driver(m, REQ_TOGGLE_ITEM);
			break;
		default:
		}
	wrefresh(w);
	}
	box(w, 0, 0);
	wrefresh(w);
}

static void multi_menu_drv()
{
	WINDOW *w = win_menu_multi;
	MENU *m = g_card_window.c_booleans;

	__menu_drv(w, m, true);
}

static void type_menu_drv()
{
	WINDOW *w = win_menu_type;
	MENU *m = g_card_window.c_types;

	__menu_drv(w, m, false);
}

static void form_drv()
{
	int ch;
	WINDOW *w = win_form;
	FORM *f = g_card_window.c_tuh_prize_form;

	wrefresh(w);
	while (KEY_ESC != (ch = getch())) {
		switch (ch) {
		case KEY_UP:
			form_driver(f, REQ_PREV_FIELD);
			form_driver(f, REQ_END_LINE);
			break;
		case KEY_DOWN:
			form_driver(f, REQ_NEXT_FIELD);
			form_driver(f, REQ_END_LINE);
			break;
		case KEY_BACKSPACE:
		case 127:
			form_driver(f, REQ_DEL_PREV);
			break;
		case KEY_DC:
			form_driver(f, REQ_DEL_CHAR);
			break;
		case '\n':
		case KEY_ENTER:
			form_driver(f, REQ_VALIDATION);
			break;
		default:
			if (E_INVALID_FIELD == form_driver(f, ch))
				form_driver(f, REQ_CLR_FIELD);
			break;
		}
	wrefresh(w);
	}
	box(w, 0, 0);
	wrefresh(w);
}

static int expansion_drv()
{
	int ch;
	WINDOW *w = win_sequel;
	wrefresh(w);

	while (KEY_ESC != (ch = getch()) && ch != ENDKEY)
		if (ch == QUITKEY)
			err_out("See You!\n", 0);

	box(w, 0, 0);
	wrefresh(w);

	return ch;
}

static int sub_driver(int drv)
{
	int ret = 0;

	switch(drv)
	{
		case 0:
			multi_menu_drv();
			break;
		case 1:
			type_menu_drv();
			break;
		case 2:
			form_drv();
			break;
		case 3:
			ret = expansion_drv();
			break;
		default:
			err_out("coder did not know what he did.\n", EINVAL);
	}

	return ret;
}

static int main_driver(int ctrl)
{
	WINDOW *w[] = { win_menu_multi, win_menu_type, win_form, win_sequel };
	int i, ret = 0;

	switch(ctrl)
	{
		case KEY_UP:
		case KEY_DOWN:
			currently_selected += 2;
			break;
		case KEY_LEFT:
			currently_selected--;
			break;
		case KEY_RIGHT:
			currently_selected++;
			break;
		case '\n':
		case KEY_ENTER:
			ret = sub_driver(currently_selected);
			break;
		case QUITKEY:
			err_out("All good\n", 0);
			break;
		default:
			break;
	}
	currently_selected = (currently_selected % 4);
	if (currently_selected < 0 || currently_selected > 3) {
		printf("WTF? cs = %d\n", currently_selected);
	}
	for (i = 0; i < 4; i++) {
		if (currently_selected != i)
			box(w[i], 0, 0);
		else
			box(w[i], ACS_DIAMOND, ACS_DIAMOND);
	}
	update_screen();

	return ret;
}

static void get_multi_bitmask(unsigned int *mask)
{
	ITEM **items;
	ITEM *ci;
	int i, numi;

	*mask = 0;

	ci = current_item(g_card_window.c_booleans);

	items = menu_items(g_card_window.c_booleans);
	numi = item_count(g_card_window.c_booleans);
	for (i = 0; i < numi; ++i) {
		if (ci == items[i] || item_value(items[i]))
			*mask |= (1 << i);
	}
}

static int get_type_id(unsigned int *id)
{
	char *chkptr;
	const char *idesc;
	ITEM *it;

	it = current_item(g_card_window.c_types);
	if (!it)
		err_out("No type selected\n", EINVAL);

	idesc = item_description(it);
	if (!idesc)
		err_out("bad type ID\n", EINVAL);

	*id = strtoul(idesc, &chkptr, 0);
	if (chkptr != idesc && *chkptr == '\0')
		return 0;

	err_out("bad type ID\n", EINVAL);

	return EINVAL;
}

static void get_int_from_formfield(FIELD *f, unsigned int *i)
{
	char *fb;

	fb = field_buffer(f, 0);
	if (!fb)
		err_out("NULL field_buffer, err %d\n", errno);

	*i = (unsigned int)atoi(fb);
}
static void get_prize_from_form(struct card *c)
{
	get_int_from_formfield(g_card_window.field[FIELD_PRIZE_DATA], &c->prize);
}

static void get_tuhina_from_form(unsigned int *tuhina)
{
	get_int_from_formfield(g_card_window.field[FIELD_TUHINA_DATA], tuhina);
}

static void my_add_cards()
{
	int i, ret, ctrl;

	ret = init_card_menus();
	if (ret)
		err_out("Menu inits failed\n", ret);

	for (i = 0; i < g_num_cards; i++) {
		struct card *c;
		int ret;
		char *ch;
		int dual_card = 0;
		unsigned int multimask;
		unsigned int expansion_id, type_id, prizetype_id, tuhisee;
		char *top_name = NULL;

		c = &g_cards[i];
	/*
	 * Some off the card stacks are dual. Eg, when they are played the stack
	 * has two different cards. Like the 'Leiri' and the 'Ryostosaalis',
	 * where the stack consists 5 of each. The original text file listing
	 * the cards were treating this as a single card, named "top card /
	 * bottom card". For example, "Laina / Ryöstösaalis". Hence we expect
	 * the card names in file to contain the '/' only when there is such
	 * dual stack.
	 *
	 * When such a dual stack is detected, the 'prize' in file is the prize
	 * of the 'top' card. The name of the bottom card follows the '/' - and
	 * may have leading whitespace. Hence we trim the whitespace. We will
	 * also need to ask the prize for the bottom card.
	 */
		for (ch = &c->name[0]; *ch; ch++)
			if (*ch == '/')
				break;

		if (*ch == '/') {
			dual_card = DUAL_CARD_TOP;
			*ch = '\0';
			trim(c->name);
		}

		if(0) {
		/*
		 * Enter this section by goto only, and only when the 'top' card
		 * from a dual card stack was inserted.
		 */
add_dual:
			top_name = strdup(c->name);
			dual_card = DUAL_CARD_BOTTOM;
			ch++;
			/* Trim whitespace from bottom card name */
			while (*ch == ' ')
				ch++;
			if (*ch == '\0') {
				err_out("Bad name for 'bottom' card\n", EINVAL);
				return;
			}
			strcpy(c->name, ch);

			c->prize = 0;

		}
		if (already_added(c))
			continue;

		update_card_menusforms(c);
		update_screen();

		while (ENDKEY != (ctrl = getch()))
			if (ENDKEY == main_driver(ctrl))
				break;

		get_multi_bitmask(&multimask);
		get_type_id(&type_id);

		get_prize_from_form(c);
		get_tuhina_from_form(&tuhisee);

		if (multimask & MASK_DEBT)
			prizetype_id = PRIZETYPE_DEBT;
		else
			prizetype_id = PRIZETYPE_NORMAL;

		ret = get_expansion(&expansion_id, c);
		if (ret)
			return;

		add_card_to_db(c->name, expansion_id, type_id, prizetype_id, c->prize, multimask, tuhisee);

		if (dual_card == DUAL_CARD_TOP)
			goto add_dual;
		if (dual_card == DUAL_CARD_BOTTOM) {
			reference_top_bot(top_name, c->name);
			free(top_name);
			top_name = NULL;
		}
	}
}

static int myslize()
{
	g_conn = mysql_init(NULL);
	if (!g_conn) {
		endwin();
		printf("MySQL Connection failed %d\n", errno);

		return -1;
	}

	if (!mysql_real_connect(g_conn, "host", "user", "pass", "database", 0, NULL, 0)) {
		endwin();
		fprintf(stderr, "real_conn: %s\n", mysql_error(g_conn));
		return -1;
	}
	my_add_cards(g_conn);

	/* close connection */
	mysql_close(g_conn);
	endwin();

	return 0;
}

static void out(int sig)
{
	endwin();
	printf("Got sig %d\n",sig);

	printf("Thanks for using diceadmin :)\n");
	fflush(stdout);
	if(SIGINT==sig || SIGTERM == sig) {
		signal(sig,SIG_DFL);
		kill(getpid(),sig);
	} else {
		exit(sig);
	}
}

int main(int argc, char *argv[])
{
	int ret, /*i,*/ c;
	int index;

	signal(SIGSTOP, SIG_IGN);
	signal(SIGTERM, &out);
	signal(SIGINT, &out);

	init();

	while (-1 != (c = getopt_long(argc, argv, OPTSTRING, long_options, &index)))
	{
		switch(c) {
		case 'E': 
		{
			int ret;

			ret = exlist_add(optarg);
			if (ret)
				return ret;
			break;
		}
		case 'v':
			printf("%s version: %s\n",argv[0],VERSION);
			return 0;
			break;
		case '?':
		case 'h':
			printf("%s version: %s\n",argv[0],VERSION);
			print_usage();
			return 0;
			break;
		default:
			break;
		}
	}

	ret = read_cards();
	if (ret)
		out(ret);

	if (!g_num_cards)
		err_out("No cards?\n", EINVAL);

	return myslize();
}
