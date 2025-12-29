<?php
/*
 * Dominon card randomizer.
 * Uses MySQL database.
 *
 * AUTHOR: Matti Vaittinen <mazziesaccount@gmail.com>
 *
 * Written just for fun. No Warranty. Use at your own risk!
 *
 * TODO: This class should be changed to suit also event / landmark cards.
 * - This way the event / landmark card sets could also be handled by the dom_card_set.php.
 *   and support for those should be added in this. We might want to have the card.php as an
 *   common 'base card class', which gets inherited by 'kingdom_cards' and 'event_cards' /
 *   'landmark_cards'.
 * - We should add show() functions to these card classes, to generate the HTML table-cells.
 *
 * Copyright 2025, Matti Vaittinen mazziesaccount@gmail.com>
 */

$CARD_DEFAULT_WEIGH = 1;

/*
 * cards table:
 * id 	int unsigned 	NO 	PRI 	NULL 	auto_increment
 * dual_top_of_id 	int unsigned 	YES 		0
 * dual_below_id 	int unsigned 	YES 		0
 * name 	varchar(255) 	NO 		NULL
 * en_name 	varchar(255) 	YES 		NULL
 * expansion_id 	int unsigned 	YES 		NULL
 * type_id 	int unsigned 	NO 		NULL
 * prizetype_id 	int unsigned 	NO 		NULL
 * prize 	int unsigned 	NO 		NULL
 * drawcards 	int 	YES 		0
 * buys 	int 	YES 		NULL
 * attack 	tinyint(1) 	YES 		NULL
 * defence 	tinyint(1) 	YES 		NULL
 * endure 	tinyint(1) 	YES 		0
 * gather 	tinyint(1) 	YES 		0
 * destroy 	tinyint(1) 	YES 		0
 * curse 	tinyint(1) 	YES 		0
 * tuhinakerroin 	int unsigned 	YES 		0
 * dropcards 	tinyint(1) 	YES 		0
 * actionmoney 	int unsigned 	YES 		0
 */

class dom_card
{
	public $id		= null;
	public $dual_top_of_id	= null;
	public $dual_below_id	= null;
	public $name		= null;
	public $en_name		= null;
	public $prize		= null;
	public $type_id		= null;
	public $prizetype_id	= null;
	public $expansion_id	= null;
	public $drawcards	= null;
	public $buys		= null;
	public $attack		= null;
	public $defence		= null;
	public $endure		= null;
	public $gather		= null;
	public $destroy		= null;
	public $curse		= null;
	public $tuhinakerroin	= null;
	public $dropcards	= null;
	public $actionmoney	= null;

	public $type_name	= null;
	public $prizetype_name	= null;
	public $expansion_name	= null;
	public $below_name	= null;
	public $above_name	= null;
	public $setup_text	= null;

	public $weight		= 0;  // For weighted random selection

	private function __populate($row, $tableprefix)
	{
		$this->setup_text = isset($row[$tableprefix.'setup_text']) ? $row[$tableprefix.'setup_text'] : null;
		$this->id = isset($row[$tableprefix.'id']) ? $row[$tableprefix.'id'] : null;
		$this->dual_top_of_id = isset($row[$tableprefix.'dual_top_of_id']) ? $row[$tableprefix.'dual_top_of_id'] : null;
		$this->dual_below_id = isset($row[$tableprefix.'dual_below_id']) ? $row[$tableprefix.'dual_below_id'] : null;
		$this->name = isset($row[$tableprefix.'name']) ? $row[$tableprefix.'name'] : null;
		$this->en_name = isset($row[$tableprefix.'en_name']) ? $row[$tableprefix.'en_name'] : null;
		$this->prize = isset($row[$tableprefix.'prize']) ? $row[$tableprefix.'prize'] : null;
		$this->type_id = isset($row[$tableprefix.'type_id']) ? $row[$tableprefix.'type_id'] : null;
		$this->prizetype_id = isset($row[$tableprefix.'prizetype_id']) ? $row[$tableprefix.'prizetype_id'] : null;
		$this->expansion_id = isset($row[$tableprefix.'expansion_id']) ? $row[$tableprefix.'expansion_id'] : null;
		$this->drawcards = isset($row[$tableprefix.'drawcards']) ? $row[$tableprefix.'drawcards'] : null;
		$this->buys = isset($row[$tableprefix.'buys']) ? $row[$tableprefix.'buys'] : 0;
		$this->attack = isset($row[$tableprefix.'attack']) ? $row[$tableprefix.'attack'] : 0;
		$this->defence = isset($row[$tableprefix.'defence']) ? $row[$tableprefix.'defence'] : 0;
		$this->endure = isset($row[$tableprefix.'endure']) ? $row[$tableprefix.'endure'] : 0;
		$this->gather = isset($row[$tableprefix.'gather']) ? $row[$tableprefix.'gather'] : 0;
		$this->destroy = isset($row[$tableprefix.'destroy']) ? $row[$tableprefix.'destroy'] : 0;
		$this->curse = isset($row[$tableprefix.'curse']) ? $row[$tableprefix.'curse'] : 0;
		$this->tuhinakerroin = isset($row[$tableprefix.'tuhinakerroin']) ? $row[$tableprefix.'tuhinakerroin'] : null;
		$this->dropcards = isset($row[$tableprefix.'dropcards']) ? $row[$tableprefix.'dropcards'] : null;
		$this->actionmoney = isset($row[$tableprefix.'actionmoney']) ? $row[$tableprefix.'actionmoney'] : null;
		$this->type_name = isset($row[$tableprefix.'type_name']) ? $row[$tableprefix.'type_name'] : null;
		$this->prizetype_name = isset($row[$tableprefix.'prizetype_name']) ? $row[$tableprefix.'prizetype_name'] : null;
		$this->expansion_name = isset($row[$tableprefix.'expansion_name']) ? $row[$tableprefix.'expansion_name'] : null;
		$this->below_name = isset($row[$tableprefix.'below_name']) ? $row[$tableprefix.'below_name'] : null;
		$this->above_name = isset($row[$tableprefix.'above_name']) ? $row[$tableprefix.'above_name'] : null;
	}

	public function get_id()
	{
		return $this->id;
	}

	private function check_row_has(array $row, array $keys, $tableprefix)
	{
		foreach($keys as $key)
			if (!isset($row[$tableprefix.$key]))
				die($tableprefix.$key.' Missing');
	}

	public static function from_full_row(array $row, $tableprefix = '')
	{
		$card = new self();

		/*
		 * Some cards don't have names in both languages.
		 * If this is the case, just use the name we have.
		 */
		if (isset($row[$tableprefix.'name']) && !isset($row[$tableprefix.'en_name']))
			$row[$tableprefix.'en_name'] = $row[$tableprefix.'name'];

		if (!isset($row[$tableprefix.'name']) && isset($row[$tableprefix.'en_name']))
			$row[$tableprefix.'name'] = $row[$tableprefix.'en_name'];

		/* Check all required data is given */
		$required_keys = array('id', 'name', 'en_name', 'dual_top_of_id', 'prize', 'type_id', 'prizetype_id', 'expansion_id', 'drawcards', 'tuhinakerroin', 'dropcards', 'actionmoney', 'type_name', 'prizetype_name', 'expansion_name');
		$card->check_row_has($row, $required_keys, $tableprefix);

		$card->__populate($row, $tableprefix);

		return $card;
	}

	public static function from_partial_row(array $row, $tableprefix='')
	{
		global $CARD_DEFAULT_WEIGH;

		$card = new self();

		debug_print("from_partial_row: tableprefix: $tableprefix, checks like $tableprefix'id'");

		if (!isset($row[$tableprefix.'id']) ||
		    !isset($row[$tableprefix.'tuhinakerroin']) ||
		    !isset($row[$tableprefix.'actionmoney']))
			die('Info necessary for randomizing is missing');

		$card->__populate($row, $tableprefix);
		$card->weight = $CARD_DEFAULT_WEIGH;

		return $card;
	}
}
