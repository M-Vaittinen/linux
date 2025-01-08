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
#include <assert.h>

#define OPTSTRING "CE:svh?"

/* These MUST match the type IDs in the database */
enum {
	PRIZETYPE_NORMAL = 1,
       	PRIZETYPE_DEBT = 2,
};

static MYSQL *g_conn;

static struct option long_options[] =
{
	{"exclude-chapter", required_argument, 0, 'E'},
	{"stats", no_argument,  0, 's'},
	{"chapters", no_argument,  0, 'C'},
/*	{"num-cheap" , required_argument, 0, 'c'},
	{"num-four" , required_argument, 0, 'f'},
	{"num-expensive" , required_argument, 0, 'e'}, */
	{"version",  no_argument, 0, 'v'},
	{"help",  no_argument, 0, 'h'},
	{0,0,0,0}
};

struct card {
	struct card *next;
	unsigned prize;
	char sequel[255];
	char name[255];
};

static struct card g_cards[1000];
static int g_num_cards = 0;

static struct card tmpcheap;
int g_numcheap = 0;
static struct card tmpmiddle;
int g_numtasan = 0;
static struct card tmpexpensive;
int g_numexpensive = 0;
static struct card cheap;
static struct card middle;
static struct card expensive;

struct exlist {
	struct exlist *next;
	char *name;
};

struct exlist g_exhead;
/*
static WINDOW *win_body, *win_form;
static FORM *form;

void err_out(const char *reason, int err)
{
	endwin();
	printf("%s %d\n", reason, err);

	exit(err);
}

int add_menu(MENU *menu, char *options[], int num_options)
{
	ITEM **items;

	items = calloc(num_options + 1, sizeof(ITEM *));

	for (i = 0; i < num_options; i++) {
		items[i] = new_item(options[i], options[i]);
		if (!items[i])
			err_out("item alloc failed\n", ENOMEM);
	}
}
*/
void init()
{
	/* Ncurses inits */
	/*
	initscr();
	cbreak();
	noecho();
	keypad(stdscr, TRUE);

	win_body = newwin(24, 80, 0, 0);
	if (!win_body)
		exit(1);

	win_form = derwin(win_body, 20, 78, 3, 1);
	assert(win_form != NULL);
	box(win_form, 0, 0);
	*/

	/* Init lists */
	g_exhead.next = NULL;
	tmpcheap.next = NULL;
	tmpmiddle.next = NULL;
	tmpexpensive.next = NULL;
	cheap.next = NULL;
	middle.next = NULL;
	expensive.next = NULL;
}

void add(struct card *head, struct card *new)
{
	new->next = head->next;
	head->next = new;
}

void del(struct card *head, struct card *foo)
{
	struct card *tmp = head;

	while (tmp->next != foo) {
		if (!tmp->next) {
			printf("Not found\n");
			return;
		}
		tmp = tmp->next;
	}

	tmp->next = foo->next;
}

void suffle()
{
	int i, j, seed = (int)time(NULL);
	int num, g_num[] = {g_numcheap, g_numtasan, g_numexpensive };
	struct card *tmphead[] = { &tmpcheap, &tmpmiddle, &tmpexpensive };
	struct card *head[] = { &cheap, &middle, &expensive };

	srand(seed);

	for (j = 0; j < 3; j++)
		for (i = g_num[j]; i > 0; i--) {
			struct card *c = tmphead[j]->next;

			num = rand() % i;
			while (num > 0) {
				c = c->next;
				num--;
			}
			del(tmphead[j], c);
			add(head[j], c);
		}
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

void add_card(struct card *c)
{
	struct card *new = &g_cards[g_num_cards];
	struct exlist *ex;

	for (ex = g_exhead.next; ex; ex = ex->next)
		if (!strcmp(ex->name, c->sequel))
			return;

	memcpy(new, c, sizeof(*c));
	g_num_cards++;

	if (new->prize < 4) {
		add(&tmpcheap, new);
		g_numcheap++;
	}
	if (new->prize == 4) {
		add(&tmpmiddle, new);
		g_numtasan++;
	}
	if (new->prize > 4) {
		add(&tmpexpensive, new);
		g_numexpensive++;
	}
}

struct seq_list {
	struct seq_list *next;
	char *name;
	int num;
};

static int print_chapters()
{
	struct seq_list list;
	struct seq_list *head = &list;
	int i;

	head->next = NULL;

	for (i = 0; i < g_num_cards; i++) {
		struct card *c = &g_cards[i];
		struct seq_list *iter = head->next;
		int found = 0;

		while (iter) {
			if (!strcmp(iter->name, c->sequel)) {
				found = 1;
				iter->num++;
				break;
			}
			iter = iter->next;
		}
		if (!found) {
			iter = malloc(sizeof(*iter));
			if (!iter)
				return ENOMEM;

			iter->name = c->sequel;
			iter->num = 1;
			iter->next = head->next;
			head->next = iter;
		}
	}
	printf("Chapters:\n");
	for (head = head->next; head; head=head->next)
		printf("\t %s: (%d cards)\n", head->name, head->num);

	return 0;
}

int print_stats()
{
	printf("Number of cards: %d\n", g_num_cards); 
	printf("Cheap (<4) : %d\n", g_numcheap); 
	printf("Mid-range (4) : %d\n", g_numtasan); 
	printf("Expensive (>4) : %d\n", g_numexpensive); 

	return 0;
}

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
	printf("Usage: ./dice [-s -e<exp> -f<mid> -c<cheap> -v -h]\n");
	printf("\t-s --stats\n");
	printf("\t-C --chapters\n");
	printf("\t-E --exclude-chapter <CHAPTER>\n");
	printf("\t-c --num-cheap <NUM>\n");
	printf("\t-f --num-four <NUM>\n");
	printf("\t-e --num-expensive <NUM>\n");
	printf("\t-v --version\n");
	printf("\t-h --help\n");
	printf("-s: summary of known cards\n");
	printf("-C: print known chapters\n");
	printf("-c -f -e: override default number of cards (3,3,4)\n");
	printf("-E: Don't include cards from given chapter.\n");
}

int err_info(int err)
{
	print_usage();

	return err;
}

int display_cards(int num, int foo)
{
	int i;
	struct card *head[] = { &cheap, &middle, &expensive };
	struct card *c;

	c = head[foo]->next;
	for (i = 0; i < num && c; i++) {
		printf("%s\t (%u) \t %s\n", c->name, c->prize, c->sequel);
		c = c->next;
	}

	return 0;
}

enum {
	NUM_CHEAP,
	NUM_MID,
	NUM_EXP
};

int already_added(struct card *c)
{
	char query[255];
	MYSQL_RES *res;
	int ret;

	ret = snprintf(query, 254, "SELECT * FROM cards WHERE name = '%s' LIMIT 1", c->name);
	if (ret >= 254) {
		printf("%s: Query too long\n", __func__);
		return -1;
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

	printf("Card '%s': %s\n", c->name, (ret)?"FOUND!" : "Not found");

	return ret;
}

int ask_user(const char *q, bool *yn)
{
	char yns[3];
	char *ret;

retry:
	printf("%s\n", q);
	printf("y/n\n");
	ret = fgets(&yns[0], 3, stdin);
	if (!ret)
		return EINVAL;

	if (yns[0] == 'y')
		*yn = true;
	else if (yns[0] == 'n')
		*yn = false;
	else
		goto retry;

	return 0;
}

struct expansion {
	int id;
	const char *name;
};

static int mva_get_knum(unsigned int *num)
{
	char sid[5];
	char *chkptr, *ret;

	ret = fgets(&sid[0], 4, stdin);
	if (!ret)
		return EINVAL;

	*num = strtoul(&sid[0], &chkptr, 0);
	if (chkptr == &sid[0] || (*chkptr && *chkptr != '\n'))
		return EINVAL;

	return 0;
}

static bool ask_id(struct expansion *exp, int num_exp, unsigned int *id)
{
	int ret, i;
	//unsigned int id;

	printf("No martching expansion found. Please select ID or type 'N' to cancel\n");

	for (i = 0; i < num_exp; i++)
		printf("ID %u - expansion '%s'\n", exp[i].id, exp[i].name);

	ret = mva_get_knum(id);
	if (ret)
		return false;

	for (i = 0; i < num_exp; i++)
		if (exp[i].id == *id) {
			printf("Selected ID %u (expansion '%s')\n", *id, exp[i].name);
			return true;
		}

	return false;
}

int get_cardtype(unsigned int *id, struct card *c)
{
	const char *query = "SELECT * FROM cardtype";
	MYSQL_RES *res;
	MYSQL_ROW row;
	int num_types, i, ret;

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
	num_types = mysql_num_rows(res);
	if (num_types < 0 || num_types > 50) {
		printf("Unexpected num of types (%d)\n", num_types);
		return -1;
	}

	printf("\n");
	printf("Give type ID for card '%s'\n", c->name);
	for (i = 0; i < num_types; i++) {
	       	row = mysql_fetch_row(res);
		printf("ID: %s: Type '%s'\n", row[0], row[1]);
	}

	ret = mva_get_knum(id);
	if (ret)
		return false;

	mysql_data_seek(res, 0);

	/* Initialize to 'ENOENT' and change to '0' if value passes the check below */
	ret = ENOENT;
	if (*id > 9 || *id < 0) {
		printf("type ID has grown to two digits. Fix the 'Ok'-check! %s:%d\n",
		       __FILE__, __LINE__);
		return EINVAL;
	}

	for (i = 0; i < num_types; i++) {
		row = mysql_fetch_row(res);
		if ((*id) + '0' == *row[0]) {
			printf("Selected type '%s' for card '%s'\n", row[1], c->name);
			ret = 0;
			break;
		}
	}

	mysql_free_result(res);

	return ret;
}

int get_expansion(unsigned int *id, struct card *c)
{
	const char *query = "SELECT * FROM expansion";
	MYSQL_RES *res;
	MYSQL_ROW row;
	struct expansion *exp;
	int num_exp, i;
	bool found;

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
	num_exp = mysql_num_rows(res);
	if (num_exp < 0 || num_exp > 50) {
		printf("Unexpected num of expansions (%d)\n", num_exp);
		return -1;
	}

	exp = calloc(num_exp, sizeof(*exp));
	if (!exp) {
		printf("OOM\n");
		return ENOMEM;
	}

	printf("\n");
	printf("Which expansion the card belongs to?\n");
	for (i = 0; i < num_exp; i++) {
		char *chkptr;

	       	row = mysql_fetch_row(res);
		exp[i].id = strtoul(row[0], &chkptr, 0);
		if (chkptr == row[0] || *chkptr) {
			printf("Bad ID\n");
			return EINVAL;
		}
		if (!strcmp(row[1], c->sequel)) {
			printf("%s \n", row[0]);
			printf("Found expansion %s\n", c->sequel);
			*id = exp[i].id;
			found = true;
		}
	}
	if (!found)
		found = ask_id(exp, num_exp, id);
	mysql_free_result(res);

	free(exp);

	if (found)
		printf("Card '%s': Using equel ID %d\n", c->name, *id);
	else
		printf("Card '%s': No Sequel ID!\n", c->name);

	return !found;
}

#define LARGE_QUERY_SIZE 2048
static char g_large_query[LARGE_QUERY_SIZE + 1];

void add_card_to_db(const char *name, int exp, int type, int prizetype,
		    int prize, bool attack, bool defence, bool endures,
		    bool gathers, bool destroys, unsigned int tuhisee, bool curses)
{
	const char *query = "INSERT INTO cards SET name='%s', expansion_id='%u', type_id='%u', prizetype_id='%u', prize='%u', attack='%u', defence='%u', endure='%u', gather='%u', destroy='%u', tuhinakerroin='%u', curse='%u'";
	int ret;
	bool ok;

	snprintf(&g_large_query[0], LARGE_QUERY_SIZE, query, name, exp, type, prizetype, prize, attack, defence, endures, gathers, destroys, tuhisee, curses);
	g_large_query[LARGE_QUERY_SIZE] = '\0';

	printf("Going to execute SQL:\n");
	printf("%s", &g_large_query[0]);
	ret = ask_user("Ok?", &ok);
	if (ret || !ok) {
		printf("Skipped query\n");
		return;
	}

	if (mysql_query(g_conn, &g_large_query[0])) {
		fprintf(stderr, "Adding card failed: %s\n", mysql_error(g_conn));
		return;
	}

	printf("Card Added\n");

	return;
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

	printf("Referencing top %s and bot %s\n", top, bot);

	for (i = 0; i < 2; i++) {
		snprintf(&g_large_query[0], LARGE_QUERY_SIZE, query, names[i]);
		g_large_query[LARGE_QUERY_SIZE] = '\0';

		/* send SQL query */
		if (mysql_query(g_conn, g_large_query)) {
			printf("%s: %s\n", g_large_query, mysql_error(g_conn));
			printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
			return;
		}

		res[i] = mysql_store_result(g_conn);
		if (!res[i]) {
			printf("mysql_store_result(): %s\n", mysql_error(g_conn));
			printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
			return;
		}
		if (1 != mysql_num_rows(res[i])) {
			printf("Unexpected num of IDs returned. Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
			return;
		}
		row = mysql_fetch_row(res[i]);
		ids[i] = row[0];
	}
	snprintf(&g_large_query[0], LARGE_QUERY_SIZE, id_update_query, "dual_top_of_id", ids[1], ids[0]);
	if (mysql_query(g_conn, g_large_query)) {
		printf("%s: %s\n", query, mysql_error(g_conn));
		printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
		return;
	}
	snprintf(&g_large_query[0], LARGE_QUERY_SIZE, id_update_query, "dual_below_id", ids[0], ids[1]);
	if (mysql_query(g_conn, g_large_query)) {
		printf("%s: %s\n", query, mysql_error(g_conn));
		printf("Fix top/bottom references for cards '%s' and '%s' manually!\n", top, bot);
		return;
	}
	for (i = 0; i < 2; i++)
		mysql_free_result(res[i]);
};

void my_add_cards()
{
	int i;

	for (i = 0; i < g_num_cards; i++) {
		struct card *c;
		int ret;
		char *ch;
		int dual_card = 0;
		bool attack, defence, debt, yn, gathers, endures, destroys, curses;
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
				printf("Bad name for 'bottom' card\n");
				return;
			}
			strcpy(c->name, ch);
//			c->name = ch + 1;

			printf("Give prize for '%s'\n", c->name);
			ret = mva_get_knum(&c->prize);
			if (ret) {
				printf("Ouch! Adding bottom card '%s* failed. Add manually and fix bottom reference for card '%s'\n", c->name, top_name);
				return;
			}

		}
		if (already_added(c))
			continue;

		printf("Add %s, prize %u:, expansion: %s\n", c->name, c->prize, c->sequel);
		if (dual_card != DUAL_CARD_BOTTOM) {
			ret = ask_user("Add?", &yn);
			if (ret)
				return;

			if (!yn)
				continue;
		}

		ret = ask_user("kiroaa?", &curses);
		if (ret)
			return;

		ret = ask_user("keräävä?", &gathers);
		if (ret)
			return;

		ret = ask_user("jatkuva?", &endures);
		if (ret)
			return;

		ret = ask_user("tuhoava?", &destroys);
		if (ret)
			return;

		ret = ask_user("Hyökkää?", &attack);
		if (ret)
			return;

		ret = ask_user("Puolustaa?", &defence);
		if (ret)
			return;

		printf("Tuhinakerroin 1 - 10\n");
		ret = mva_get_knum(&tuhisee);
		if (ret)
			return;

		ret = ask_user("Prize is debt?", &debt);
		if (ret)
			return;

		if (debt)
			prizetype_id = PRIZETYPE_DEBT;
		else
			prizetype_id = PRIZETYPE_NORMAL;

		ret = get_expansion(&expansion_id, c);
		if (ret)
			return;

		ret = get_cardtype(&type_id, c);
		if (ret)
			return;

		add_card_to_db(c->name, expansion_id, type_id, prizetype_id, c->prize, attack, defence, endures, gathers, destroys, tuhisee, curses);

		if (dual_card == DUAL_CARD_TOP)
			goto add_dual;
		if (dual_card == DUAL_CARD_BOTTOM) {
			reference_top_bot(top_name, c->name);
			free(top_name);
			top_name = NULL;
		}
	}
}

int myslize()
{
	MYSQL_RES *res;
	MYSQL_ROW row;

	g_conn = mysql_init(NULL);
	if (!g_conn) {
		printf("MySQL Connection failed %d\n", errno);
		return -1;
	}

	if (!mysql_real_connect(g_conn, "hostname", "username", "password", "database", 0, NULL, 0)) {
		fprintf(stderr, "real_conn: %s\n", mysql_error(g_conn));
		return -1;
	}
	/* send SQL query */
	if (mysql_query(g_conn, "show tables")) {
		fprintf(stderr, "show tables: %s\n", mysql_error(g_conn));
		return -1;
      }

	res = mysql_use_result(g_conn);

	/* output table name */
	printf("MySQL Tables in mysql database:\n");
	while ((row = mysql_fetch_row(res)) != NULL)
		printf("%s \n", row[0]);
	mysql_free_result(res);

	my_add_cards(g_conn);

	/* close connection */
	mysql_close(g_conn);

	return 0;
}

int main(int argc, char *argv[])
{
	int ret, /*i,*/ c;
	//int num_cards[] = {3, 3, 4};
	int pr_stats = 0, pr_chaps = 0;
	int index;

	init();

	while (-1 != (c = getopt_long(argc, argv, OPTSTRING, long_options, &index)))
	{
		//char *chkptr;

		switch(c) {
		case 'E': 
		{
			int ret;

			ret = exlist_add(optarg);
			if (ret)
				return ret;
			break;
		}
		case 'C':
			pr_chaps = 1;
			break;
/*		case 'c':
	 		num_cards[NUM_CHEAP] = strtoul(optarg, &chkptr, 0);
			if (chkptr == optarg || (*chkptr && *chkptr != '\n'))
				return err_info(EINVAL);
			break;
		case 'f':
 			num_cards[NUM_MID] = strtoul(optarg, &chkptr, 0);
			if (chkptr == optarg || (*chkptr && *chkptr != '\n'))
				return err_info(EINVAL);
		break;
		case 'e':
 			num_cards[NUM_EXP] = strtoul(optarg, &chkptr, 0);
			if (chkptr == optarg || (*chkptr && *chkptr != '\n'))
				return err_info(EINVAL);
		break;
		*/
		case 's':
			pr_stats = 1;
			break;
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
		return ret;

	if (pr_stats)
		return print_stats();

	if (pr_chaps)
		return print_chapters();

	if (!g_num_cards)
		return -EINVAL;

	return myslize();

	/*


	suffle();

	for (i = 0; i < 3; i++) {
		ret = display_cards(num_cards[i], i);
		if (ret)
			return ret;
	}
	*/
//	return 0;
}
