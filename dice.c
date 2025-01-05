// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2025 Matti Vaittinen

#define VERSION "0.1"

#include <getopt.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#define OPTSTRING "c:e:f:svh?"

static struct option long_options[] =
{
	{"stats", no_argument,  0, 's'},
	{"num-cheap" , required_argument, 0, 'c'},
	{"num-four" , required_argument, 0, 'f'},
	{"num-expensive" , required_argument, 0, 'e'},
	{"version",  no_argument, 0, 'v'},
	{"help",  no_argument, 0, 'h'},
	{0,0,0,0}
};

struct card {
	struct card *next;
	struct card *prev;
	unsigned prize;
	char sequel[255];
	char name[255];
};

static struct card g_cards[1000];
static int g_num_cards = 0;

static struct card tmpalle4;
int g_numalle = 0;
static struct card tmpnelja;
int g_numtasan = 0;
static struct card tmpyli4;
int g_numyli = 0;
static struct card alle4;
static struct card nelja;
static struct card yli4;

void init()
{
	tmpalle4.next = NULL;
	tmpnelja.next = NULL;
	tmpyli4.next = NULL;
	alle4.next = NULL;
	nelja.next = NULL;
	yli4.next = NULL;
	tmpalle4.prev = NULL;
	tmpnelja.prev = NULL;
	tmpyli4.prev = NULL;
	alle4.prev = NULL;
	nelja.prev = NULL;
	yli4.prev = NULL;
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
	int num, g_num[] = {g_numalle, g_numtasan, g_numyli };
	struct card *tmphead[] = { &tmpalle4, &tmpnelja, &tmpyli4 };
	struct card *head[] = { &alle4, &nelja, &yli4 };

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

void add_card(struct card *c)
{
	struct card *new = &g_cards[g_num_cards];

	memcpy(new, c, sizeof(*c));
	g_num_cards++;

	if (new->prize < 4) {
		add(&tmpalle4, new);
		g_numalle++;
	}
	if (new->prize == 4) {
		add(&tmpnelja, new);
		g_numtasan++;
	}
	if (new->prize > 4) {
		add(&tmpyli4, new);
		g_numyli++;
	}
}

int print_stats()
{
	struct card *c;

	printf("Number of cards: %d\n", g_num_cards); 
	printf("Cheap (<4) : %d\n", g_numalle); 
	printf("Mid-range (4) : %d\n", g_numtasan); 
	printf("Expensive (>4) : %d\n", g_numyli); 

	return 0;
}

int read_cards()
{
	FILE *cf;
	int ret;
 
	init();

	cf = fopen("cards-new.txt","r");
	if (!cf) {
		perror("cards-new.txt");
		return EINVAL;
	}

	while(1) {
		struct card c;

		ret = fscanf(cf,"%s ## %u ## %s\n", &c.name[0], &c.prize, &c.sequel[0]);
		if (ret == EOF)
			return 0;

		if (ret == 3)
			add_card(&c);
	}

	return -1;
}

static void print_usage()
{
	printf("Usage: ./dice [-s -e<exp> -f<mid> -c<cheap> -v -h]\n");
	printf("\t-s --stats\n");
	printf("\t-c --num-cheap <NUM>\n");
	printf("\t-f --num-four <NUM>\n");
	printf("\t-e --num-expensive <NUM>\n");
	printf("\t-v --version\n");
	printf("\t-h --help\n");
	printf("-s: summary of known cards\n");
	printf("-c -f -e: override default number of cards (3,3,4)\n");
}

int err_info(int err)
{
	print_usage();

	return err;
}

int arvo(int num, int foo)
{
	int i;
	struct card *head[] = { &alle4, &nelja, &yli4 };
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

int main(int argc, char *argv[])
{
	int ret, i, c;
	int num_cards[] = {3, 3, 4};
	int pr_stats = 0;
	int index;

	while (-1 != (c = getopt_long(argc, argv, OPTSTRING, long_options, &index)))
	{
		char *chkptr;

		switch(c) {
		case 'c':
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

	init();
	ret = read_cards();
	if (ret)
		return ret;

	if (pr_stats)
		return print_stats();

	if (!g_num_cards)
		return -EINVAL;

	suffle();

	for (i = 0; i < 3; i++) {
		ret = arvo(num_cards[i], i);
		if (ret)
			return ret;
	}

	return 0;
}
