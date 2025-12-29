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
define("LAND_ID_OFFSET", 1000000);
define("EVENT_ID_OFFSET", 2000000);
define("PRIZETYPE_ID_DEBT", 2);

//$DBG=true;
$DBG=false;
$TESTING=false;

$preselected = null;
$keep_land_ids = null;
$keep_event_ids = null;
if (isset($_POST['keepid'])) {
	foreach ($_POST['keepid'] AS $keepid) {
		if (!is_numeric($keepid))
			die('Non numeric KID');

		if ($keepid < LAND_ID_OFFSET) { /* Regular Kingdom card */
			if (!isset($_POST['keepprize'.$keepid]))
				die('Prizeless KID');
			$kidprize = $_POST['keepprize'.$keepid];
			if (!is_numeric($kidprize))
				die('KID is not a number');
			if ($kidprize < 4) {
				$preselected[0][] = $keepid;
				$preselected_prize[0][] = $kidprize;
			} else if ($kidprize == 4) {
				$preselected[1][] = $keepid;
				$preselected_prize[1][] = $kidprize;
			} else {
				$preselected[2][] = $keepid;
				$preselected_prize[2][] = $kidprize;
			}
		} else if ($keepid < EVENT_ID_OFFSET) { /* Landmark card */
			$keep_land_ids[] = $keepid - LAND_ID_OFFSET;
		} else { /* Event card */
			$keep_event_ids[] = $keepid - EVENT_ID_OFFSET;
		}
	}
}

if (isset($_POST['event_expansions']))
	$event_exp = $_POST['event_expansions'];
else
	$event_exp = 0;

if (isset($_POST['landmark_expansions']))
	$land_exp = $_POST['landmark_expansions'];
else
	$land_exp = 0;

if (isset($_POST['expansion'])) {
	$exp = $_POST['expansion'];
} else {
	if ($TESTING)
		$exp = array(3,4,5,6, 15);
	else
		$exp = 0;
}

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

debug_print("Recv'd tuhina: $tuh_inafactor, Tupina: $tup_inafactor (nihilism $nihilism), Kapita: $kap_itafactor");
/* On a mobile device we try to fit the tables on a screen */
$mobile = isMobileDevice();
do_head("Dominion - korttiarvonta v2");
echo '<h1>Dominion - Arvo kortit v2 (<a href="index2.php">Yhyy, Wanha oli parempi</a>)</h1>';

echo output_input_form($conn, $mobile, $exp, $land_exp, $event_exp, $tuh_inafactor, $tup_inafactor, $kap_itafactor);

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
$PRIZEBUCKETS = do_prize_bucket_where($boundary_prizes, 'c.prize');

$QUERY_BASE = 'SELECT c.id AS id, c.tuhinakerroin AS tuhinakerroin, c.actionmoney AS actionmoney, c.curse AS curse, c.attack AS attack, c.defence AS defence, c.type_id AS type_id FROM cards AS c LEFT JOIN expansion as e ON c.expansion_id = e.id WHERE e.disabled != 1 AND ';

$num_cards = array(3, 3, 4);
$card_group_names = array('Halpaa ku saippua', 'Keskiluokan keskiostos', 'N&auml;&auml; M&auml;&auml; Tahdon!');

$card_set = dom_card_set::prepare_set($conn);

function add_existing(&$selected, $preselected)
{
	foreach($preselected AS $new_id) {
		$new_card = new dom_card();

		$new_card->id = $new_id;
		$selected[] = $new_card;	
	}
}

function exclude_presel_id($presel)
{
	$where = '';
	foreach($presel AS $exclude_id)
		$where .= " AND c.id != $exclude_id";

	return $where;
}

function add_landmark_kinput($land_id, $offset, $checked) {
	$land_id += $offset;
	$out = '<input form="theform" type="checkbox" name="keepid[]" value="'.$land_id.'"'.$checked.'>'."\n";
	return $out;
}

function show_eventland($conn, $event_exp_ids, $land_exp_ids, $keep_land_ids, $keep_event_ids, $mobile = true)
{
	$out = "";

	if (!($event_exp_ids || $land_exp_ids || $keep_land_ids || $keep_event_ids))
		return;

	if ($event_exp_ids || $keep_event_ids) {
		$query_base = "SELECT events.id, events.name, events.prize, events.debt, events.curses, setup.text, setup.id AS stupid, expansion.name AS exp_name ";
		$query_base .= "FROM events AS events ";
		$query_base .= "LEFT JOIN setup_extras AS setup ON events.setup_id = setup.id ";
		$query_base .= "LEFT JOIN expansion AS expansion ON events.expansion_id = expansion.id ";
		$query_base .= "WHERE ";

		$query_keep = "";

		$num_keep_evs = 0;
		if ($keep_event_ids) {
			$num_keep_evs = count($keep_event_ids);
			$query_keep = $query_base; 
			$query_keep .= "events.id = $keep_event_ids[0]";
		}
		/*
		 * There is a special case where user has selected to keep an event,
		 * but disabled the 'events' from expansion for the new query. In this
		 * case the RightThingToDo(tm) is to keep the selected event but to not
		 * randomize new card. So, let's forget all the expansion_id stuff when
		 * !$event_exp_ids.
		 */

		if ($num_keep_evs == 2 || !$event_exp_ids) {
			if ($num_keep_evs == 2)
				$query = $query_keep . " OR events.id = $keep_event_ids[1]";
			else
				$query = $query_keep;
			$query .= " LIMIT 2";
		} else {
			$query = $query_base;

			$where = "";
			foreach($event_exp_ids as $expid) {
				if ($where == "")
					$where .= "(expansion.id = $expid";
				else
					$where .= " OR expansion.id = $expid";
			}
			$where .= ')';
			$query .= $where;
			$query .= " ORDER BY RAND() LIMIT ".(2 - $num_keep_evs);

			if ($num_keep_evs) {
				$query = "(($query) UNION " . $query_keep . ") LIMIT 2";
			}
		}

		$result = mysqli_query($conn, $query);
		if (!$result)
			die("no expansions".mysql_error($conn));

		if (!$mobile) {
			$out .= '<h3>Tapahtumat</h3>'."\n";
			$out .= '<table class="cardlist"><tr>'."\n";
			$out .= '<th class="checkbox">[pid&auml;]</th><th>Kortti</th><th class="squeeze">Specials</th><th>Hinta</th><th>Peliosa</th></tr>'."\n";
		} else {
			$out .= "<h3> $title </h3>\n";
			$out .= '<table class="cardlist"><tr>'."\n";
			$out .= '<th class="checkbox">[pid&auml;]</th><th>Kortti</th><th>Specials</th> <th>Peliosa</th></tr>'."\n";
		}

		while ($row = mysqli_fetch_assoc($result)) {
			$checked = "";
			$setup_tip = "";
			if ($keep_event_ids) {
				foreach($keep_event_ids AS $kid) {
					if ($kid == $row['id']) {
						$checked = "checked";
						break;
					}
				}
			}

			$cardname = htmlspecialchars($row['name']);
			$expansionname = htmlspecialchars($row['exp_name']);

			if ($row['text']) {
				$setup_tip_text = htmlspecialchars($row['text']);
				$setup_tip .= '<div class="image-container">'."\n";
				$setup_tip .= '<img src="img/peasant.png" alt="Valmistelut" tabindex="0">'."\n";
				$setup_tip .= '<div class="hover-text">Extra valmisteluja: '.$setup_tip_text.'</div>'."\n";
				$setup_tip .= '</div>'."\n";
			}
			if ($row['debt']) {
				$setup_tip .= '<div class="image-container">'."\n";
				$setup_tip .= '<img src="img/debt.png" alt="Myyd&auml;&auml;n Rahoituksella" tabindex="0">'."\n";
				$setup_tip .= '<div class="hover-text">Myyd&auml;&auml;n Rahoituksella</div>'."\n";
				$setup_tip .= '</div>'."\n";
			}
			if (!$mobile) {
				//$prizetype = ($row['debt']) ? '(Velka)' : '(Raha)';
				$out .= '<tr><td class="checkbox">'.add_landmark_kinput($row['id'], EVENT_ID_OFFSET, $checked).'</td><td>'.$cardname.'</td><td>'. (($setup_tip != '') ? $setup_tip : '--') .'</td><td>'.$row['prize'].'</td><td>'.$expansionname.'</td></tr>'."\n";
				//$out .= '<tr><td class="checkbox">'.add_landmark_kinput($row['id'], EVENT_ID_OFFSET, $checked).'</td><td>'.$cardname.'</td><td>'. (($setup_tip != '') ? $setup_tip : '--') .'</td><td>'.$row['prize'].' '.$prizetype. '</td><td>'.$expansionname.'</td></tr>'."\n";
			} else {
				$out .= '<tr><td class="checkbox">'.add_landmark_kinput($row['id'], EVENT_ID_OFFSET, $checked).'</td><td>'.$cardname.'</td><td>'. (($setup_tip != '') ? $setup_tip : '--') .'</td><td>'.$expansionname.'</td></tr>'."\n";
			}
		} // while () MySQL results ends
		$out .= '</table>';
	} // if $event_exp_ids ends

	if ($land_exp_ids || $keep_land_ids) {
		$query = 'SELECT landmark.id, landmark.name, landmark.description, setup.text, expansion.name AS exp_name FROM landmarks AS landmark ';
		$query .= 'LEFT JOIN setup_extras AS setup ON landmark.setup_id = setup.id ';
		$query .= 'LEFT JOIN expansion AS expansion ON landmark.expansion_id = expansion.id ';
		$query .= 'WHERE ';

		if (!$keep_land_ids) {
			$where = "";
			foreach($land_exp_ids as $expid) {
				if ($where == "")
					$query .= "expansion.id = $expid";
				else
					$query .= " OR expansion.id = $expid";
			}
			$query .= ' ORDER BY RAND()';
		} else {
			$query .= 'landmark.id = '.$keep_land_ids[0];
		}
		$query .= ' LIMIT 1';

		$result = mysqli_query($conn, $query);
		if (!$result)
			die("no expansions".mysql_error($conn));

		if (!$mobile) {
			$out .= '<h3>Maamerkit</h3>'."\n";
			$out .= '<table class="cardlist"><tr>'."\n";
			$out .= '<th class="checkbox">[pid&auml;]</th><th>Kortti</th><th>Selitys</th><th class="squeeze">Specials</th><th>Peliosa</th></tr>'."\n";
		} else {
			$out .= "<h3> $title </h3>\n";
			$out .= '<table class="cardlist"><tr>'."\n";
			$out .= '<th class="checkbox">[pid&auml;]</th><th>Kortti</th><th>Specials</th><th>Peliosa</th></tr>'."\n";
		}

		while ($row = mysqli_fetch_assoc($result)) {
			$setup_tip = "";
			$checked = "";

			$cardname = htmlspecialchars($row['name']);
			$expansionname = htmlspecialchars($row['exp_name']);
			$description = htmlspecialchars($row['description']);

			if ($keep_land_ids)
				if ($keep_land_ids[0] == $row['id'])
					$checked = "checked";
			if ($row['text']) {
				$setup_tip_text = htmlspecialchars($row['text']);
				$setup_tip .= '<div class="image-container">'."\n";
				$setup_tip .= '<img src="img/peasant.png" alt="Valmistelut" tabindex="0">'."\n";
				$setup_tip .= '<div class="hover-text">Extra valmisteluja: '.$setup_tip_text.'</div>'."\n";
				$setup_tip .= '</div>'."\n";
			}
			if (!$mobile) {
				$out .= '<tr>'."\n";
				$out .= '<td class="checkbox">'.add_landmark_kinput($row['id'], LAND_ID_OFFSET, $checked).'</td>'."\n";
				$out .= '<td>'.$cardname.'</td>'."\n";
				$out .= '<td>'.$description.'</td>'."\n";
				$out .= '<td>'. (($setup_tip != '') ? $setup_tip : '--') .'</td>'."\n";
				$out .= '<td>'.$expansionname.'</td>'."\n";
				$out .= '</tr>'."\n";
			} else {
				$out .= '<tr>'."\n";
				$out .= '<td class="checkbox">'.add_landmark_kinput($row['id'], LAND_ID_OFFSET, $checked).'</td>'."\n";
				$out .= '<td>'.$cardname.'</td>'."\n";
				$out .= '<td>'. (($setup_tip != '') ? $setup_tip : '--') .'</td>'."\n";
				$out .= '<td>'.$expansionname.'</td>'."\n";
				$out .= '</tr>'."\n";
			}
		} // while MySQL results ends
		$out .= '</table>';
	} // if ($land_exp_ids) ends

	echo $out;
}

//if ($exp) {
	$i = 0;
	foreach($PRIZEBUCKETS as $PRIZE_LIMIT) {
		$exp_where = SQL_add_expansion_where('c.expansion_id', $exp);

		$query = $QUERY_BASE.$PRIZE_LIMIT;
		if ($exp_where)
			$query .= " AND ".$exp_where;

		if (isset($preselected[$i]))
			$num_presel=count($preselected[$i]);
		else
			$num_presel = 0;
		if ($num_presel) {
			$query .= exclude_presel_id($preselected[$i]);
		}

		$result = query_cards($conn, $query);
		$foo = 0;
		while ($row = mysqli_fetch_assoc($result)) {
			$foo++;
			$card[] = dom_card::from_partial_row($row);
		}
		debug_print("$foo cards fetched for $card_group_names[$i] - selecting from those:");

		$selected = randomize_cards($card, $tuh_inafactor, $tup_inafactor, $nihilism, $kap_itafactor, $num_cards[$i] - $num_presel);
		if ($num_presel)
			add_existing($selected, $preselected[$i]);

		$card_set->add_set($selected, $card_group_names[$i]);
		$card = array();

		$i++;

		debug_print("bucket $i: $PRIZE_LIMIT");
	}

	$card_set->get_cards();
	$card_set->show_sets($preselected, $mobile);

	if ($land_exp || $event_exp || $keep_land_ids || $keep_event_ids) {
		show_eventland($conn, $event_exp, $land_exp, $keep_land_ids, $keep_event_ids, $mobile);
	}
//}

//echo output_input_form($conn, $mobile, $exp);

/* Close connection, print (c) and send </body> </html> */
echo '<p><h1><a href="aloittaja.php" target="_blank">Arvo my&ouml;s aloittaja?</a></h1>';
require 'include/footer.php';

?>
