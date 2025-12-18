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
 * Tupina'o-meter => Nihilistipeli (paljon tupinaa)
 *
 * TODO:
 * Öky'o-meter => Priorisoi isoja rahoja
 */

$DBG=false;
//$DBG=true;

if (isset($_POST['expansion']))
	$exp = $_POST['expansion'];
else
	$exp = 0;

$tuh_inafactor = null;
if (isset($_POST['tuhinaenable'])) {
	if (is_numeric($_POST['tuhinarange'])) {
		if ($_POST['tuhinarange'] <= 20 && $_POST['tuhinarange'] >= 0)
			$tuh_inafactor = $_POST['tuhinarange'];
	}
}

$tup_inafactor = null;
if (isset($_POST['tupinaenable'])) {
	if (is_numeric($_POST['tupinarange'])) {
		if ($_POST['tupinarange'] <= 20 && $_POST['tupinarange'] >= 0)
			$tup_inafactor = $_POST['tupinarange'];
	}
}

if (isset($_POST['add_nihilism'])) {
	$nihilism = true;
} else {
	$nihilism = false;
}

$kap_itafactor = null;
if (isset($_POST['kapitaenable'])) {
	if (is_numeric($_POST['kapitarange'])) {
		if ($_POST['kapitarange'] <= 20 && $_POST['kapitarange'] >= 0)
			$kap_itafactor = $_POST['kapitarange'];
	}
}

require 'include/db.php';
require 'include/header.php';
require 'include/dominion_common.php';

function num_cards_in_res($result)
{
	$retprize[0] = 0;
	$retprize[1] = 0;
	$retprize[2] = 0;
	
	while ($row = mysqli_fetch_assoc($result)) {
		if (!is_numeric($row['prize']))
			die('bad data in database: prize');
		if ($row['prize'] < 4)
			$retprize[0]++;
		if ($row['prize'] == 4)
			$retprize[1]++;
		if ($row['prize'] > 4)
			$retprize[2]++;
	}
	mysqli_data_seek($result, 0);

	return $retprize;
}

function card_ids_in_res($result) {
	$ids = array();

	while ($row = mysqli_fetch_assoc($result)) {
		if (!is_numeric($row['id']))
			die('bad data in database: id');
		$ids[] = $row['id'];
	}

	mysqli_data_seek($result, 0);

	return $ids;
}

function __print_cards($result, $title, $mobile)
{
	$tuhinasum = 0;
	while ($row = mysqli_fetch_assoc($result)) {
		if (!is_numeric($row['tuhinakerroin']))
			die('invalid data in database - tuhinakerroin');
		$tuhinasum += $row['tuhinakerroin'];
	}
	mysqli_data_seek($result, 0);

	if (!$mobile) {
		$out = '<h3>' . $title . '</h3>
		<table class="cardlist"><tr>
			<th>Kortti</th><th>Korttityyppi</th><th>Hinta</th><th>Peliosa</th></tr>';
	} else {
		$out = '<h3>' . $title . '</h3>
		<table class="cardlist"><tr>
			<th>Kortti</th> <th>Peliosa</th></tr>';
	}

	while ($row = mysqli_fetch_assoc($result)) {
		$name = htmlspecialchars($row['c_name']);
		$en_name = htmlspecialchars($row['en_name']);
		$prize = htmlspecialchars($row['prize']);
		$prizetype = htmlspecialchars($row['p_name']);
		$expansion = htmlspecialchars($row['e_name']);
		$cardtype = htmlspecialchars($row['ct_name']);

		if ($name != "" && $en_name != "" && $name != $en_name)
			$name = $name . " (" . $en_name . ")";

		if (!$mobile)
			$out .= '<tr><td>' . $name . '</td><td>' . $cardtype . '</td><td>' . $prize . ' (' . $prizetype . ')</td><td>' . $expansion . '</td></tr>';
		else
			$out .= '<tr><td>' . $name . '</td><td>' . $expansion . '</td></tr>';
	}
	$out .= "</table>";
	echo $out;
	echo "Tuhina " . $tuhinasum;
}

function print_cards($conn, $query, $title, $mobile)
{
	$result = query_cards($conn, $query);
	__print_cards($result, $title, $mobile);
}

function card_query_start()
{
	return 'SELECT c.id, c.tuhinakerroin, c.prize, c.en_name, c.name AS c_name, p.name AS p_name, e.name AS e_name, ct.name AS ct_name FROM cards AS c JOIN prizetype as p JOIN expansion AS e JOIN cardtype AS ct';
}

function card_query_where($tuh_inafactor, $exp, $exclude_ids = array())
{
	$base_where = "WHERE p.id = c.prizetype_id AND c.expansion_id = e.id AND c.type_id = ct.id AND e.disabled != 1";
	/*
	 * TODO: Find a good way to prevent the cards which belong to same storage deck from being suffled in.
	 * It'd be easy to just exclude all cards which belong to the "bottom deck" (see query below), but that would
	 * exclude a few of the interesting options from 'tupina' and 'tuhina' weighing.
	 *
	 *	$base_where = "WHERE p.id = c.prizetype_id AND c.expansion_id = e.id AND c.type_id = ct.id AND c.dual_below_id = 0";
	 */
	$where = $base_where;

	foreach($exclude_ids as $id)
		$where .= " AND c.id !='" . $id . "'";

	if (isset($tuh_inafactor)) {
		if ($tuh_inafactor == 0)
			$where .= ' AND c.tuhinakerroin = 0';
	}
	$expansion_where = SQL_add_expansion_where('c.expansion_id', $exp);
	if ($expansion_where)
		$where .= " AND ".$expansion_where;

	$where_arr[] = $where . ' AND c.prize < 4';
	$where_arr[] = $where . ' AND c.prize = 4';
	$where_arr[] = $where . ' AND c.prize > 4';
	$where_arr[] = $where;

	return $where_arr;
}

function card_query_order($tuh_inafactor, $tup_inafactor, $nihilism, $kap_itafactor)
{
	$base_order = 'ORDER BY (RAND()';
	$order = $base_order;

	/*
	 * Weighs to be added to the RAND for biasing the card selection based on
	 * tuhinakerron and actionmoney.
	 */
	$tuhinaweigh = '(c.tuhinakerroin / 40 * '.$tuh_inafactor.' + 1 )';
	$actionweigh = '( c.actionmoney / 12 * '. $tup_inafactor . ' + 1)';
	$kapitaweigh = '(c.actionmoney / 40 * ' . $kap_itafactor .' + 1 )';

	/* echo "tuhweigh $tmp"; */

	if (isset($tuh_inafactor) && $tuh_inafactor != 0) {
		$order .= ' * ' . $tuhinaweigh;
	}
	/*
	 * If nihilism is checked, then we decrease the weigh for action cards with money.
	 *
	 * TODO: We should add information about the actions which allow picking more cards to hand. The
	 * nihilism should probably also decrease probability of such cards to maximize the agony.
	 */
	if (isset($tup_inafactor) && $tup_inafactor > 0 && $nihilism) {
		$order .= ' / ' . $actionweigh;
	}

	if (isset($kap_itafactor) && $kap_itafactor > 0) {
		$order .= ' * ' . $kapitaweigh;
	}

	$order .= ') DESC';

	return $order;
}

function card_query_limit($num_ch, $num_md, $num_ex)
{
	$limit[] = 'LIMIT ' . $num_ch;
	$limit[] = 'LIMIT ' . $num_md;
	$limit[] = 'LIMIT ' . $num_ex;

	return $limit;
}

/* On a mobile device we try to fit the tables on a screen */
$mobile = isMobileDevice();

do_head("Dominion - korttiarvonta");
echo "<h1>Dominion - Arvo kortit</h1>";

echo '<p>Heps Kukkuu. Olet vanhalla korttiarvontasivulla. Uusi on <a href="index.php">t&auml;&auml;ll&auml;</a><br />'."\n";


//die("WTF2");
$result = get_expansions($conn, false);

/* Output the form table */
if (!$mobile) {
	$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Tuhina\'o-meter</th> <th>Tupina\'o-meter</th><th>Kapita\'o-meter</th></tr><tr><td>';
	} else {
	$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Painotukset</th></tr><tr><td>';
	}
$output .= '<form action="" method="post">';

/* Print expansion checkboxes */
$tmp_exp_idx = 0;
while ($row = mysqli_fetch_assoc($result)) {
	if ($exp && $row['id'] == $exp[$tmp_exp_idx]) {
		$checked = " checked";
		$tmp_exp_idx ++;
	} else {
		$checked = "";
	}

	if (!is_numeric($row['id']))
		die("bad data in database - expansion id");

	$output .= '<input type="checkbox" id="' . htmlspecialchars($row['name']) . '" name="expansion[]" value="' . $row['id'] . '"'."$checked>";
	$output .= '<label for="' . htmlspecialchars($row['name']) . '">' . htmlspecialchars($row['name']) . '</label><br>';
}
$output .= '</td>';



/* ...Tuhina cell: */
$output .= '<td>';
if ($mobile)
	$output .= '<b>Tuhina\'o-meter</b> <br />';
$output .= '<div class="slidecontainer">
  <input type="range" min="0" max="20" value="0" class="slider" name="tuhinarange" id="tuhinarange">
</div>';
$output .= '<input type="checkbox" id="tuhinaenable" name="tuhinaenable" value="1">';
$output .= '<label for="tuhinaenable">Ota Tuhina\'o-meter k&auml;ytt&ouml;&ouml;n</label><br>';
$output .= '<div class="help-tip">
    <p>Tuhina\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa priorisoimaan toimintoketjuja lis&auml;&auml;vi&auml; kortteja. Asettamalla arvon 0:aan voit my&ouml;s est&auml;&auml; isomman Tuhinaindex&copy; :n korttien valinnan kokonaan.</p>
</div>';
$output .= '</td>';

if ($mobile) {
	/* On a mobile we end the row here */
	$output .= '</tr><tr><td></td>';
}

/* Tupina cell: */
$output .= '<td>';
if ($mobile)
	$output .= '<b>Tupina\'o-meter</b><br />';
$output .= '<div class="slidecontainer">
  <input type="range" min="0" max="20" value="0" class="slider" name="tupinarange" id="tupinarange">
</div>';
$output .= '<input type="checkbox" id="tupinaenable" name="tupinaenable" value="1">';
$output .= '<label for="tupinaenable">Ota Tupina\'o-meter k&auml;ytt&ouml;&ouml;n...</label><br>';
$output .= '<input type="checkbox" id="add_nihilism" name="add_nihilism" value="1">';
$output .= '<label for="add_nihilism">...ripauksella nihilismi&auml;</label><br>';
$output .= '<div class="help-tip">
    <p>Tupina\'o-meter&copy; :ll&auml; lis&auml;&auml;t peliin tupinaa ja jupinaa aiheuttavia elementtej&auml;. Tupina\'o-meter&copy; eroaa Tuhina\'o-meter&copy; :st&auml; siin&auml;, ett&auml; arvon asettaminen 0:ksi ei kuitenkaan kokonaan poista tupinaa aiheuttavien korttien mahdollisuutta pelist&auml;. <br /><br />Ja jos todella haluat koetella k&auml;rsiv&auml;llisyytesi rajoja niin voit h&ouml;yst&auml;&auml; peli&auml; ripauksella nihilismi&auml; ja pienent&auml;&auml; rahaa tuovien toimintojen mahdollisuutta.</p>
</div>';
$output .= '</td>';

if ($mobile) {
	/* On a mobile we end the row here */
	$output .= '</tr><tr><td></td>';
}

/* ...Kapita cell: */
$output .= '<td>';
if ($mobile)
	$output .= '<b>Kapita\'o-meter</b><br />';
$output .= '<div class="slidecontainer">
  <input type="range" min="0" max="20" value="0" class="slider" name="kapitarange" id="kapitarange">
</div>';
$output .= '<input type="checkbox" id="kapitaenable" name="kapitaenable" value="1">';
$output .= '<label for="kapitaenable">Ota Kapita\'o-meter k&auml;ytt&ouml;&ouml;n</label><br>';
$output .= '<div class="help-tip">
    <p>Kapita\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa priorisoimaan raha- ja rahaa lis&auml;&auml;vi&auml; toimintakortteja.</p>
</div>';
$output .= '</td>';


/* End of the form table and form */
$output .= '</tr></table>';
$output .= '<input type="submit" value="Submit">';
$output .= '</form>';

/* Output the input form */
echo $output;

$printed_prizes[0] = 0;
$printed_prizes[1] = 0;
$printed_prizes[2] = 0;

$exclude_ids = array();
$exclude_ids_kapita = array();

$printed_prizes_kapita[0] = 0;
$printed_prizes_kapita[1] = 0;
$printed_prizes_kapita[2] = 0;

if (isset($kap_itafactor) && $kap_itafactor > 0) {
	if ($kap_itafactor < 7) {
		$num_money = 2;
	} else if ($kap_itafactor < 14) {
		$num_money = 3;
	} else {
		$num_money = 4;
	}
	$special_query_base .= card_query_start();
	$special_where = card_query_where(0, $exp);
	$special_where_type .= " AND c.type_id = 1";

	$special_query = $special_query_base . ' ' . $special_where[3] . $special_where_type . ' ORDER BY RAND() LIMIT ' . $num_money;

	$spec_res = query_cards($conn, $special_query);
	$printed_prizes_kapita = num_cards_in_res($spec_res);

	$exclude_ids_kapita = card_ids_in_res($spec_res);

	__print_cards($spec_res, "Kauppakillan Erikoiset", $mobile);
}

if (isset($tup_inafactor) && $tup_inafactor > 0) {
	$num_curses = 0;
	$num_drops = 0;
	if ($tup_inafactor < 5) {
		$num_attack = 2;
	} else if ($tup_inafactor < 10) {
		$num_attack = 2;
		$num_drops = 1;
	} else if ($tup_inafactor < 15) {
		$num_attack = 1;
		$num_curses = 1;
		$num_drops = 1;
	} else {
		$num_attack = 2;
		$num_curses = 1;
		$num_drops = 1;
	}

	$special_query = "";

	$special_query_base = card_query_start();
	$special_where = card_query_where(0, $exp);
	if ($num_drops > 0)
		$special_query .= '(' . $special_query_base. ' ' . $special_where[3] . ' AND c.dropcards = 1 ORDER BY RAND() LIMIT ' . $num_drops . ')';
	if ($num_curses > 0) {
		if ($num_drops > 0)
			$special_query .= ' UNION ' . '('. $special_query_base. ' ' . $special_where[3] . ' AND c.curse = 1 ORDER BY RAND() LIMIT ' . $num_curses . ')';
		else
			$special_query = '('. $special_query_base. ' ' . $special_where[3] . ' AND c.curse = 1 ORDER BY RAND() LIMIT ' . $num_curses . ')';
	}
	if ($num_drops > 0 || $num_curses > 0)
		$special_query .= ' UNION (' . $special_query_base . ' ' . $special_where[3] . ' AND c.attack = 1 ORDER BY RAND() )';
	else
		$special_query = $special_query_base . ' ' . $special_where[3] . ' AND c.attack = 1 ORDER BY RAND()'; 

	$special_query .= ' LIMIT ' . $num_attack + $num_drops + $num_curses;

	$spec_res = query_cards($conn, $special_query);
	$printed_prizes = num_cards_in_res($spec_res);

	$exclude_ids = card_ids_in_res($spec_res);

	__print_cards($spec_res, "Tupina Specials", $mobile);
}

$exclude_ids = array_merge($exclude_ids, $exclude_ids_kapita);

$query_start = card_query_start();
$query_where = card_query_where($tuh_inafactor, $exp, $exclude_ids);

$query_order = card_query_order($tuh_inafactor, $tup_inafactor, $nihilism, $kap_itafactor);

$printed_cheap = $printed_prizes_kapita[0] + $printed_prizes[0];
$printed_mid = $printed_prizes_kapita[1] + $printed_prizes[1];
$printed_exp = $printed_prizes_kapita[2] + $printed_prizes[2];

while ($printed_cheap > 3) {
	$printed_mid++;
	$printed_cheap--;
}

while ($printed_mid > 3) {
	$printed_exp++;
	$printed_mid--;
}

while ($printed_exp > 4) {
	$printed_exp--;
	$printed_mid++;
}

while ($printed_mid > 3) {
	$printed_cheap++;
	$printed_mid--;
}

if ($printed_cheap > 3)
	die('Sorry, cheap > 3');

if ($printed_mid > 3)
	die('Sorry, cheap > 3');

if ($printed_exp > 4)
	die('Sorry, cheap > 3');

$query_limit = card_query_limit(3 - $printed_cheap, 3 - $printed_mid, 4 - $printed_exp);

$query_cheap = $query_start . ' ' . $query_where[0] . ' ' . $query_order . ' ' .$query_limit[0];
$query_mid = $query_start . ' ' . $query_where[1] . ' ' . $query_order . ' ' .$query_limit[1];
$query_exp = $query_start . ' ' . $query_where[2] . ' ' . $query_order . ' ' .$query_limit[2];


echo '<hr style="height:10px;border-width:0;color:#d2691e;background-color:#d2691e">';

/* Output cards */
/* if (!$mobile)
	echo '<table class="structure"><tr><td>';
 */
if ($printed_cheap != 3)
	print_cards($conn, $query_cheap, "Hinta &lt; 4", $mobile);

/* if (!$mobile)
	echo '</td><td>';
 */
if ($printed_mid != 3)
	print_cards($conn, $query_mid, "Hinta 4", $mobile);

/* if (!$mobile)
	echo '</td><td>';
 */
if ($printed_exp != 4)
	print_cards($conn, $query_exp, "Hinta &gt; 4", $mobile);

/* if (!$mobile)
	echo '</td></tr></table>';
 */
echo '<p><h1><a href="aloittaja.php" target="_blank">Arvo my&ouml;s aloittaja?</a></h1>';

/* Close connection, print (c) and send </body> </html> */
require 'include/footer.php';

?>
