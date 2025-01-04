// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2025 Matti Vaittinen

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

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

int arvo(char *num, int foo)
{
	int i;
	unsigned long ret;
	char *chkptr;
	struct card *head[] = { &alle4, &nelja, &yli4 };
	struct card *c;

 	ret = strtoul(num, &chkptr, 0);
	if (*chkptr && *chkptr != '\n' && chkptr != num)
		return ret;

	c = head[foo]->next;
	for (i = 0; i < ret && c; i++) {
		printf("%s\t (%u) \t %s\n", c->name, c->prize, c->sequel);
		c = c->next;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	int ret, i;

	if (argc != 4) {
		printf("Parametrit puuttuu, num < 4, num 4, num > 4\n");
		return EINVAL;
	}
	init();
	ret = read_cards();
	if (ret)
		return ret;

	if (!g_num_cards)
		return -EINVAL;

	suffle();

	for (i = 0; i < 3; i++) {
		ret = arvo(argv[i + 1], i);
		if (ret)
			return ret;
	}

	return 0;
}
