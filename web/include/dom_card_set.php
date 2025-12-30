<?php
/*
 * Dominon card randomizer.
 * Uses MySQL database.
 *
 * AUTHOR: Matti Vaittinen <mazziesaccount@gmail.com>
 *
 * Written just for fun. No Warranty. Use at your own risk!
 *
 * TODO: This class should be rewritten so that:
 * - A single object instantiated from this class, would represent a single set of cards.
 * - Eg, own object for cards cheaper than 4. Own object for cards costing 4,
 *   and own object for cards costing more than 4.
 * - This would simplify the logic and get rid of the 'set' arrays.
 * - If we still want to do only one query to the database for retrieving the selected
 *   cards, then we need to pull the query logic out of the class, and add a function which
 *   gets the query results as an argument, matches the IDs from results to IDs in the set,
 *   and updates the card information based on the result data. Other option would be to do
 *   a query / set, and keep the SQL in the class. It's sure cleaner but maybe less efficient.
 *   Hence I might prefer having the SQL outside this class.
 * - Furthermore, we should do the TODO: in cards.php -file. That should help quite a bit
 *   to further simplify this class. I think the cards should be having their own 'show()'
 *   function which outputs the table cell with card data. This class should just call the
 *   card's show().
 *
 * Copyright 2025, Matti Vaittinen mazziesaccount@gmail.com>
 */

class dom_card_set {
	public $set_name	= null;
	public $ids		= null;
	public $all_ids		= null;
	public $cards		= null;
	private $conn		= null;

	private function add_change_input($id, $set_num, $prize, $checked)
	{
		$out = '<input form="theform" type="checkbox" name="keepid[]" value="'.$id.'"'.$checked.'>'."\n";
		$out .= '<input form="theform" type="hidden" name="keepprize'.$id.'" value="'.$prize.'"'.$checked.'>'."\n";
		return $out;
	}
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

		$query = "SELECT c.*, e.name AS expansion_name, pt.name AS prizetype_name, ct.name AS type_name, setup.text AS setup_text FROM cards AS c ";
		$query .= "LEFT JOIN expansion AS e ON c.expansion_id = e.id ";
		$query .= "LEFT JOIN cardtype AS ct ON c.type_id = ct.id ";
		$query .= "LEFT JOIN prizetype AS pt ON c.prizetype_id = pt.id ";
		$query .= "LEFT JOIN setup_extras AS setup ON c.setup_extras_id = setup.id ";
		$query .= "WHERE c.id IN ($this->all_ids) ";
		$query .= "ORDER BY c.prize";
		$res = query_cards($this->conn, $query);
		while ($row = mysqli_fetch_assoc($res))
			$this->cards[] = dom_card::from_full_row($row);
	}
	public function show_sets($keepids, $mobile = 0) {

		$vals_on_sets = array(3,3,4);
		$omena = false;

		$out = "";
		for ($i = 0; $i < 3; $i++) {
			$title = $this->set_name[$i];

			if (!$mobile) {
				$out .= '<h3>' . $title . '</h3>'."\n";
				$out .= '<table class="cardlist"><tr>'."\n";
				$out .= '<th class="checkbox">[pid&auml;]</th><th>Kortti</th><th class="squeeze">Specials</th><th>Korttityyppi</th><th>Hinta</th><th>Peliosa</th></tr>'."\n";
			} else {
				$out .= "<h3> $title </h3>\n";
				$out .= '<table class="cardlist"><tr>'."\n";
				$out .= '<th class="checkbox">[pid&auml;]</th><th>Kortti</th> <th>Peliosa</th></tr>'."\n";
			}
			$tuhinasum = 0;
			/* This is a horrible hack, trusting sets have 3, 3, 4 cards */
			for ($j = 0; $j < $vals_on_sets[$i]; $j++) {
				$c = $this->cards[$j + 3 * $i];
				$tuhinasum += $c->tuhinakerroin;

				$checked = "";
				if (isset($keepids[$i])) {
					foreach($keepids[$i] AS $keep) {
						if ($c->id == $keep)
							$checked = " checked";
					}
				}

				$name = htmlspecialchars($c->name);
				$en_name = htmlspecialchars($c->en_name);
				$prize = htmlspecialchars($c->prize);
//				$prizetype = htmlspecialchars($c->prizetype_name);
				$expansion = htmlspecialchars($c->expansion_name);
				$cardtype = htmlspecialchars($c->type_name);

				$setup_tip = '';

				if ($c->omen) {
					$setup_tip .= '<div class="image-container">'."\n";
					$setup_tip .= '<img src="img/omena.png" alt="Omen" tabindex="0">'."\n";
					$setup_tip .= '<div class="hover-text">Olen Omena</div>'."\n";
					$setup_tip .= '</div>'."\n";
					/* We return the information that an omen was included so we can later add the prophecies */
					$omena = true;
				}
				if ($c->prizetype_id == PRIZETYPE_ID_DEBT) {
					$setup_tip .= '<div class="image-container">'."\n";
					$setup_tip .= '<img src="img/debt.png" alt="Myyd&auml;&auml;n Rahoituksella" tabindex="0">'."\n";
					$setup_tip .= '<div class="hover-text">Myyd&auml;&auml;n Rahoituksella</div>'."\n";
					$setup_tip .= '</div>'."\n";
				}
				if ($c->curse) {
					$setup_tip .= '<div class="image-container">'."\n";
					$setup_tip .= '<img src="img/curse.png" alt="Kiroukset" tabindex="0">'."\n";
					$setup_tip .= '<div class="hover-text">Kirous</div>'."\n";
					$setup_tip .= '</div>'."\n";
				}
				if ($c->attack) {
					$setup_tip .= '<div class="image-container">'."\n";
					$setup_tip .= '<img src="img/speargoblin.png" alt="Valmistelut" tabindex="0">'."\n";
					$setup_tip .= '<div class="hover-text">Hy&ouml;kk&auml;ys</div>'."\n";
					$setup_tip .= '</div>'."\n";
				}
				if ($c->setup_text) {
					$setup_tip .= '<div class="image-container">'."\n";
					$setup_tip .= '<img src="img/peasant.png" alt="Valmistelut" tabindex="0">'."\n";
					$setup_tip .= '<div class="hover-text">Extra valmisteluja: '.$c->setup_text.'</div>'."\n";
					$setup_tip .= '</div>'."\n";
				}

				if ($name != "" && $en_name != "" && $name != $en_name)
					$name = $name . " (" . $en_name . ")";

				if (!$mobile)
					$out .= '<tr><td class="checkbox">' . $this->add_change_input($c->id, $i, $c->prize, $checked).'</td><td>' . $name . '</td><td>'. (($setup_tip != '') ? $setup_tip : '--') . '</td><td>' . $cardtype . '</td><td>' . $prize . '</td><td>' . $expansion . '</td></tr>'."\n";
					//$out .= '<tr><td class="checkbox">' . $this->add_change_input($c->id, $i, $c->prize, $checked).'</td><td>' . $name . '</td><td>'. (($setup_tip != '') ? $setup_tip : '--') . '</td><td>' . $cardtype . '</td><td>' . $prize . ' (' . $prizetype . ')</td><td>' . $expansion . '</td></tr>'."\n";
				else
					$out .= '<tr><td>' . $this->add_change_input($c->id, $i, $c->prize, $checked) . '</td><td>'. $name . $setup_tip . '</td><td>' . $expansion . '</td></tr>'."\n";
			}
			$out .= "</table>"."\n";
			$out .= "Tuhina " . $tuhinasum."\n";
		}
		echo $out;

		return $omena;
	}

	public static function prepare_set($conn) {
		$card_set =  new self();

		$card_set->conn = $conn;
		return $card_set;
	}
}

?>
