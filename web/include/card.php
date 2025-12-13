<?php

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

	public $weight		= 0;  // For weighted random selection

	private function __populate($row)
	{
		$this->id = isset($row['id']) ? $row['id'] : null;
		$this->dual_top_of_id = isset($row['dual_top_of_id']) ? $row['dual_top_of_id'] : null;
		$this->dual_below_id = isset($row['dual_below_id']) ? $row['dual_below_id'] : null;
		$this->name = isset($row['name']) ? $row['name'] : null;
		$this->en_name = isset($row['en_name']) ? $row['en_name'] : null;
		$this->prize = isset($row['prize']) ? $row['prize'] : null;
		$this->type_id = isset($row['type_id']) ? $row['type_id'] : null;
		$this->prizetype_id = isset($row['prizetype_id']) ? $row['prizetype_id'] : null;
		$this->expansion_id = isset($row['expansion_id']) ? $row['expansion_id'] : null;
		$this->drawcards = isset($row['drawcards']) ? $row['drawcards'] : null;
		$this->buys = isset($row['buys']) ? $row['buys'] : null;
		$this->attack = isset($row['attack']) ? $row['attack'] : null;
		$this->defence = isset($row['defence']) ? $row['defence'] : null;
		$this->endure = isset($row['endure']) ? $row['endure'] : null;
		$this->gather = isset($row['gather']) ? $row['gather'] : null;
		$this->destroy = isset($row['destroy']) ? $row['destroy'] : null;
		$this->curse = isset($row['curse']) ? $row['curse'] : null;
		$this->tuhinakerroin = isset($row['tuhinakerroin']) ? $row['tuhinakerroin'] : null;
		$this->dropcards = isset($row['dropcards']) ? $row['dropcards'] : null;
		$this->actionmoney = isset($row['actionmoney']) ? $row['actionmoney'] : null;
		$this->type_name = isset($row['type_name']) ? $row['type_name'] : null;
		$this->prizetype_name = isset($row['prizetype_name']) ? $row['prizetype_name'] : null;
		$this->expansion_name = isset($row['expansion_name']) ? $row['expansion_name'] : null;
		$this->below_name = isset($row['below_name']) ? $row['below_name'] : null;
		$this->above_name = isset($row['above_name']) ? $row['above_name'] : null;
	}

	public function get_id()
	{
		return $this->id;
	}

	private function check_row_has(array $row, array $keys)
	{
		foreach($keys as $key)
			if (!isset($row[$key]))
				die($key.' Missing');
	}

	public static function from_full_row(array $row)
	{
		$card = new self();

		/*
		 * Some cards don't have names in both languages.
		 * If this is the case, just use the name we have.
		 */
		if (isset($row['name']) && !isset($row['en_name']))
			$row['en_name'] = $row['name'];

		if (!isset($row['name']) && isset($row['en_name']))
			$row['name'] = $row['en_name'];

		/* Check all required data is given */
		$required_keys = array('id', 'name', 'en_name', 'dual_top_of_id', 'prize', 'type_id', 'prizetype_id', 'expansion_id', 'drawcards', 'attack', 'defence', 'endure', 'gather', 'destroy', 'curse', 'tuhinakerroin', 'dropcards', 'actionmoney', 'type_name', 'prizetype_name', 'expansion_name');
		$card->check_row_has($row, $required_keys);

		$card->__populate($row);

		return $card;
	}

	public static function from_partial_row(array $row)
	{
		global $CARD_DEFAULT_WEIGH;

		$card = new self();

		if (!isset($row['id']) ||
		    !isset($row['tuhinakerroin']) ||
		    !isset($row['actionmoney']) ||
		    !isset($row['curse']) ||
		    !isset($row['attack']) ||
		    !isset($row['defence']))
	    	    die('Info missing');

		$card->__populate($row);
		$card->weight = $CARD_DEFAULT_WEIGH;

		return $card;
	}
}
