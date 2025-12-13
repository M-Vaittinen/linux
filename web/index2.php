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
 *
 * Tuhina'o-meter => paljon toimintoja (tuhinaa)
 * Tupina'o-meter => Nihilistipeli (paljon tupinaa lisäpiinalla)
 * Kapita'o-meter => Rahaa!
 *
 */


//$DBG=true;
$DBG=false;
$TESTING=false;

if (isset($_POST['expansion']))
	$exp = $_POST['expansion'];
else
	if ($TESTING)
		$exp = array(3,4,5,6, 15);
	else
		$exp = 0;

$tuh_inafactor = 0;
if (isset($_POST['tuhinarange']) && is_numeric($_POST['tuhinarange'])) {
	if ($_POST['tuhinarange'] <= 10 && $_POST['tuhinarange'] >= -10)
		$tuh_inafactor = $_POST['tuhinarange'];
}

$tup_inafactor = 0;
if (isset($_POST['tupinarange']) && is_numeric($_POST['tupinarange'])) {
	if ($_POST['tupinarange'] <= 10 && $_POST['tupinarange'] >= -10)
		$tup_inafactor = $_POST['tupinarange'];
}

if (isset($_POST['add_nihilism'])) {
	$nihilism = true;
} else {
	$nihilism = false;
}

$kap_itafactor = 0;
if (isset($_POST['kapitarange']) && is_numeric($_POST['kapitarange'])) {
	if ($_POST['kapitarange'] <= 10 && $_POST['kapitarange'] >= -10)
		$kap_itafactor = $_POST['kapitarange'];
}

require 'include/db.php';
require 'include/header.php';
require 'include/dominion_common.php';
require 'include/card.php';
require 'include/dom_card_set.php';

/* On a mobile device we try to fit the tables on a screen */
$mobile = isMobileDevice();
do_head("Dominion - korttiarvonta v2");
echo '<h1>Dominion - Arvo kortit v2 (<a href="index2.php">Yhyy, Wanha oli parempi</a>)</h1>';

echo output_input_form($conn, $mobile, $exp);

function random_card_from_array(&$card, $min_weight)
{
	$idx = 0;
	$sum_weigh = 0;

	debug_print(count($card)." cards");
	foreach($card as $c)
		$sum_weigh += $c->weight - $min_weight + 1;

	$random = mt_rand(1, $sum_weigh);

	$weigh_accum = 0;
	foreach($card as $c) {
		$weigh_accum += $c->weight - $min_weight + 1;

		if ($weigh_accum >= $random) {
			$picked = $c;
			break;
		}
		$idx++;
	}
	if (!isset($picked))
		die("<br />Weighing failed!");
	array_splice($card, $idx, 1);

	return $picked;
}

function randomize_cards($card, $tuh_inafactor, $tup_inafactor, $nihilism, $kap_itafactor, $num_cards)
{
	global $CARDTYPE_MONEY;
	global $CARD_DEFAULT_WEIGH;

	debug_print("weighing ... $num_cards");

	$min_weight = $CARD_DEFAULT_WEIGH;

	if (isset($tuh_inafactor)) {
		/*
		 * Each card has "tuhinakerroin" from 0 => 10, describing how much "hassle" the card causes.
		 * The "$tuh_inafactor" is user's preference (selected by slider), ranging from -10 to +10.
		 * If user enables the slider, we will either increase or decrease the card's chance of being
		 * selected by adding a factor to it's weigh. The addition is user's selection * card's
		 * "tuhinakerroin", so maximum 10 * 10 (100), minimum -10 * 10 (-100).
		 * Original unaltered weight for every card is 100, so this can either double or zero specific
		 * card's chances to be selected. TODO: Do we really want to zero card's chance? If so, we can
		 * simply omit all cards with weight 0 or less when randomizing.
		 */
		foreach($card as $c) {
			$c->weight += $c->tuhinakerroin * $tuh_inafactor - 10;
			if ($c->weight < $min_weight)
				$min_weight = $c->weight;
		}
	}
	if (isset($tup_inafactor)) {
		foreach($card as $c) {
			/* Let's increase/decrease the chances of cards which are attacks or curses */
			/* Attack or curse cards get -100 ... 100 added to weigh */
			if ($c->attack || $c->curse) {
				$c->weight += $tup_inafactor * 10;
			}
			if ($nihilism) {
				/*
				 * Oh, you really want to suffer ?
				 * We decrease chances of defensive cards and cards with 'actionmoney'
				 * The largest 'actionmoney' ATM is the Death Cart, 5. This means the
				 * weigh can change -50..0
				 */
				$c->weight -= $c->actionmoney * 10;
				/* And, for defence cards, another -50 */
				if ($c->defence)
					$c->weight -= 50;

			}
			if ($c->weight < $min_weight)
				$min_weight = $c->weight;
		}
	}
	if (isset($kap_itafactor)) {
		foreach($card as $c) {
			/*
			 * And here the biggest change is Death Cart's actionmoney 5 * min/max capitafactor * 2
			 * => 5 * (+/-)10 * 2 => -100 ... +100
			 */
			$c->weight += $c->actionmoney * $kap_itafactor * 2;
			if ($c->type_id == $CARDTYPE_MONEY)
				$c->weight += $kap_itafactor * 10;
			if ($c->weight < $min_weight)
				$min_weight = $c->weight;
		}
	}

	for ($i = 0; $i < $num_cards; $i++) {
		debug_print("weighing loop, $i of ".($num_cards - 1)." ...");
		$selected[] = random_card_from_array($card, $min_weight);
	}

	return $selected;
}

function get_prize_buckets($conn, $exp)
{
	return array(3, 4);
	/*
	 * Idea of this code was to split the cards to 3 buckets, cheap, mid and expensive (as previous version did).
	 * Original versions used prizes prize < 4, prize == 4 and prize > 4 for categories.
	 *
	 * While testing things with 'Nousukausi' expansion alone, I noticed that it only had 3 cards that costed less than 3.
	 * So, with the original division, this resulted same 3 cards to always be selected for 'cheap' category.
	 *
	 * So, I developed the code below, which queried the prizes of the cards in selected expansions, and tried to
	 * compute them into 3 roughly same-sized buckets. Fine ?
	 *
	 * No. There are severe problems with this approach.
	 * 
	 * 1. Several expansions have majority of cards costing around 3 and 4 coins. With the code below, for example the
	 * 'Guilds' expansion will have no cards in the 'expensive' set - and things just break apart.
	 *
	 * 2. When the game begins, there is 10 cards in hand, 7 of which are copper, 3 are victory cards. It means that the
	 * typical hand(s) during first rounds will be 3 and 4 coins - which means that if the 'cheap' bucket ends up having
	 * only cards costing 4, it gets unlikely players have an option to buy other but silver. This sounds boring. Even more
	 * boring it gets if low limit could climb to 5 or more.
	 *
	 * So, let's just keep the old ranges: ([<4], [4], [>4]) for now.
	 *
	 * TODO: Find a way to make amount of cards in each bucket more flexible. This helps if some expansions don't have
	 * enough cards in a specific bucket.
	 *
	 *

	if ($exp)
		$QUERY = "SELECT prize FROM cards WHERE" . SQL_add_expansion_where( 'expansion_id', $exp);
	else
		$QUERY = "SELECT prize FROM cards";

	$res = query_cards($conn, $QUERY);
	$num_cards = mysqli_num_rows($res);

	$bucket_optimal = $num_cards / 3;

	$prizes = array();
	while ($row = mysqli_fetch_assoc($res)) {
		$prize = $row['prize'];
		if (!isset($prizes[$prize]))
			$prizes[$prize] = 1;
		else
			$prizes[$prize] ++;
	}
	ksort($prizes);

	$sumnum = 0;
	$prevprize = 0;

	$dbg = $num_cards;

	foreach($prizes as $prize => $num) {
		debug_print("prize $prize, cards $num, bucket-size ". ($sumnum + $num) .", optimal $bucket_optimal ");
		if ($sumnum + $num > $bucket_optimal) {
			if ($sumnum > 3) {
				$boundary[] = $prevprize;
				debug_print("BOUNDARY: $prevprize, ($sumnum cards)");
				$dbg -= $sumnum;
				$sumnum = $num;
			} else {
				$boundary[] = $prize;
				debug_print("BOUNDARY: $prize, (".($sumnum + $num)." cards)");
				$dbg -= ($sumnum + $num);
				$sumnum = 0;
			}
			if (count($boundary) == 2) {
				debug_print("last bucket has $dbg cards");
				return $boundary;
			}
		} else {
			$sumnum += $num;
		}
		$prevprize = $prize;
	}

	die("Can't create prize buckets. Too few cards in selected expansions?<br />");
	*/
}

function do_prize_bucket_where($boundaries, $prize_column)
{
	$where[] = "$prize_column <= $boundaries[0]";
	$where[] = "($prize_column > $boundaries[0] AND $prize_column <= $boundaries[1])";
	$where[] = "$prize_column > $boundaries[1]";

	return $where;
}

$boundary_prizes = get_prize_buckets($conn, $exp);

$QUERY_BASE = 'SELECT id, tuhinakerroin, actionmoney, curse, attack, defence, type_id FROM cards WHERE ';  


$PRIZEBUCKETS = do_prize_bucket_where($boundary_prizes, 'prize');

$num_cards = array(3, 3, 4);
$i = 0;

$card_group_names = array('Halpaa ku saippua', 'Keskiluokan keskiostos', 'N&auml;&auml; M&auml;&auml; Tahdon!');
$card_set = dom_card_set::prepare_set($conn);

foreach($PRIZEBUCKETS as $PRIZE_LIMIT) {
	$exp_where = SQL_add_expansion_where('expansion_id', $exp);

	$query = $QUERY_BASE.$PRIZE_LIMIT;
	if ($exp_where)
		$query .= " AND ".$exp_where;

	$result = query_cards($conn, $query);
	$foo = 0;
	while ($row = mysqli_fetch_assoc($result)) {
		$foo++;
		$card[] = dom_card::from_partial_row($row);
	}
	debug_print("$foo cards fetched for $card_group_names[$i] - selecting from those:");

	$selected = randomize_cards($card, $tuh_inafactor, $tup_inafactor, $nihilism, $kap_itafactor, $num_cards[$i]);
	$card_set->add_set($selected, $card_group_names[$i]);
	$card = array();

	$i++;

	debug_print("bucket $i: $PRIZE_LIMIT");
}

$card_set->get_cards();
$card_set->show_sets($mobile);

/* Close connection, print (c) and send </body> </html> */
echo '<p><h1><a href="aloittaja.php" target="_blank">Arvo my&ouml;s aloittaja?</a></h1>';
require 'include/footer.php';

?>
