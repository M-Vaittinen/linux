<?php
/*
 * Dominon card randomizer.
 * Uses MySQL database.
 *
 * AUTHOR: Matti Vaittinen <mazziesaccount@gmail.com>
 *
 * Written just for fun. No Warranty. Use at your own risk!
 *
 * Copyright 2025, Matti Vaittinen mazziesaccount@gmail.com>
 */

class dom_card_set {
	public $set_name	= null;
	public $ids		= null;
	public $all_ids		= null;
	public $cards		= null;
	private $conn		= null;

	public function add_set($skeleton_cards, $set_name) {
		$this->set_name[] = $set_name;

		$ids = array_map(fn($c) => $c->get_id(), $skeleton_cards);
		$tmp = implode(',', $ids);
		$this->ids[] = $tmp;
		if (count($this->ids) > 1)
			$this->all_ids .= ', '.$tmp;
		else
			$this->all_ids .= $tmp;
	}
	public function get_cards()
	{
		debug_print("$this->all_ids");

		$query = "SELECT c.*, e.name AS expansion_name, pt.name AS prizetype_name, ct.name AS type_name FROM cards AS c ";
		$query .= "JOIN expansion AS e ";
		$query .= "JOIN cardtype AS ct ";
		$query .= "JOIN prizetype AS pt ";
		$query .= "WHERE e.id = c.expansion_id AND ";
		$query .= "ct.id = c.type_id AND ";
		$query .= "pt.id = c.prizetype_id AND ";
	        $query .= "c.id IN ($this->all_ids)";
		//$query .= "JOIN expansion AS e ";
		$query .= "";
		$query .= "";
		$res = query_cards($this->conn, $query);
		while ($row = mysqli_fetch_assoc($res))
			$this->cards[] = dom_card::from_full_row($row);
	}
	public function show_sets($mobile = 0) {

		$vals_on_sets = array(3,3,4);

		$out = "";
		for ($i = 0; $i < 3; $i++) {
			$title = $this->set_name[$i];
	
			if (!$mobile) {
				$out .= '<h3>' . $title . '</h3>'."\n";
				$out .= '<table class="cardlist"><tr>'."\n";
				$out .= '<th>Kortti</th><th>Korttityyppi</th><th>Hinta</th><th>Peliosa</th></tr>'."\n";
			} else {
				$out .= "<h3> . $title . </h3>\n";
				$out .= '<table class="cardlist"><tr>'."\n";
				$out .= '<th>Kortti</th> <th>Peliosa</th></tr>'."\n";
			}
			$tuhinasum = 0;
			/* This is a horrible hack, trusting sets have 3, 3, 4 cards */
			for ($j = 0; $j < $vals_on_sets[$i]; $j++) {
				$c = $this->cards[$j + 3 * $i];
				$tuhinasum += $c->tuhinakerroin;

				$name = htmlspecialchars($c->name);
				$en_name = htmlspecialchars($c->en_name);
				$prize = htmlspecialchars($c->prize);
				$prizetype = htmlspecialchars($c->prizetype_name);
				$expansion = htmlspecialchars($c->expansion_name);
				$cardtype = htmlspecialchars($c->type_name);

				if ($name != "" && $en_name != "" && $name != $en_name)
					$name = $name . " (" . $en_name . ")";

				if (!$mobile)
					$out .= '<tr><td>' . $name . '</td><td>' . $cardtype . '</td><td>' . $prize . ' (' . $prizetype . ')</td><td>' . $expansion . '</td></tr>'."\n";
				else
					$out .= '<tr><td>' . $name . '</td><td>' . $expansion . '</td></tr>'."\n";
			}
			$out .= "</table>"."\n";
			$out .= "Tuhina " . $tuhinasum."\n";
		}
		echo $out;
	}
	public static function prepare_set($conn) {
		$card_set =  new self();

		$card_set->conn = $conn;
		return $card_set;
	}
}

?>
