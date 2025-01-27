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

if (isset($_POST['expansion']))
	$exp = $_POST['expansion'];

if (isset($_POST['tuhinaenable'])) {
	if (is_numeric($_POST['tuhinarange'])) {
		if ($_POST['tuhinarange'] <= 20 && $_POST['tuhinarange'] >= 0)
			$tuh_inafactor = $_POST['tuhinarange'];
	}
}

if (isset($_POST['tupinaenable'])) {
	if (is_numeric($_POST['tupinarange'])) {
		if ($_POST['tupinarange'] <= 20 && $_POST['tupinarange'] >= 0)
			$tup_inafactor = $_POST['tupinarange'];
	}
}

require 'include/db.php';
require 'include/header.php';

function query_cards($conn, $query)
{
	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no cards");

	if (mysqli_num_rows($result) <= 0)
		die("still no cards");

	return $result;
}

function num_cards_in_res($result)
{
	$retprize[0] = 0;
	$retprize[1] = 0;
	$retprize[2] = 0;
	
	while ($row = mysqli_fetch_assoc($result)) {
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

	while ($row = mysqli_fetch_assoc($result))
		$ids[] = $row['id'];

	mysqli_data_seek($result, 0);

	return $ids;

}

function __print_cards($result, $title, $mobile)
{
	$tuhinasum = 0;
	while ($row = mysqli_fetch_assoc($result)) {
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
		$name = $row['c_name'];
		$prize = $row['prize'];
	       	$prizetype = $row['p_name'];
		$expansion = $row['e_name'];
		$cardtype = $row['ct_name'];
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
	return 'SELECT c.id, c.tuhinakerroin, c.prize, c.name AS c_name, p.name AS p_name, e.name AS e_name, ct.name AS ct_name FROM cards AS c JOIN prizetype as p JOIN expansion AS e JOIN cardtype AS ct';
}

function card_query_where($tuh_inafactor, $exp, $exclude_ids = array())
{
	$base_where = "WHERE p.id = c.prizetype_id AND c.expansion_id = e.id AND c.type_id = ct.id";
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
	if (isset($exp)) {
		$i = 0;
		$where .= ' AND (';
		foreach( $exp as $e) {
			if (!is_numeric($e))
				die('Bad expansion ID');
			if ($i == 0)
				$where .= "c.expansion_id = '" . $e . "'";
			else
				$where .= " OR c.expansion_id = '" . $e . "'";
			$i++;
		}
		$where .= ')';
	}
	
	$where_arr[] = $where . ' AND c.prize < 4';
	$where_arr[] = $where . ' AND c.prize = 4';
	$where_arr[] = $where . ' AND c.prize > 4';
	$where_arr[] = $where;

	return $where_arr;
}

function card_query_order($tuh_inafactor)
{
	$base_order = 'ORDER BY RAND()';

	$order = $base_order;

	if (isset($tuh_inafactor) && $tuh_inafactor != 0) {
		$order .= ' / ( c.tuhinakerroin * '. $tuh_inafactor . ')';
	}
	if (isset($tup_inafactor) && $tup_inafactor != 0) {
		if ($tup_inafactor > 0)
			$order .= ' * ( c.actionmoney * '. $tup_inafactor . ')';
	}

	$order .= ' DESC';

	return $order;
}

function card_query_limit($num_ch, $num_md, $num_ex)
{
	$limit[] = 'LIMIT ' . $num_ch;
	$limit[] = 'LIMIT ' . $num_md;
	$limit[] = 'LIMIT ' . $num_ex;

	return $limit;
}


do_head("Dominion - korttiarvonta");
echo "<h1>Dominion - Arvo kortit</h1>";


$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN cards as c WHERE e.id = c.expansion_id";
$result = mysqli_query($conn, $query);
if (!$result)
	die("no expansions");

if (mysqli_num_rows($result) <= 0)
	die("still no expansions");


/* Output the form table */
$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Tuhina\'o-meter</th> <th>Tupina\'o-meter</th></tr><tr><td>';
$output .= '<form action="" method="post">';

/* Print expansion checkboxes */
while ($row = mysqli_fetch_assoc($result)) {
	$output .= '<input type="checkbox" id="' . $row['name'] . '" name="expansion[]" value="' . $row['id'] . '" checked>';
	$output .= '<label for="' . $row['name'] . '">' . $row['name'] . '</label><br>';
}
$output .= '</td>';



/* ...Tuhina cell: */
$output .= '<td>';
$output .= '<div class="slidecontainer">
  <input type="range" min="0" max="20" value="10" class="slider" name="tuhinarange" id="tuhinarange">
</div>';
$output .= '<input type="checkbox" id="tuhinaenable" name="tuhinaenable" value="1">';
$output .= '<label for="tuhinaenable">Enable Tuhina\'o-meter</label><br>';
$output .= '<div class="help-tip">
    <p>Tuhina\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa priorisoimaan toimintoketjuja lis&auml;&auml;vi&auml; kortteja. Asettamalla arvon 0:aan voit my&ouml;s est&auml;&auml; isomman Tuhinaindex&copy; :n korttien valinnan kokonaan.</p>
</div>';
$output .= '</td>';

/* Tupina cell: */
$output .= '<td>';
$output .= '<div class="slidecontainer">
  <input type="range" min="0" max="20" value="10" class="slider" name="tupinarange" id="tupinarange">
</div>';
$output .= '<input type="checkbox" id="tupinaenable" name="tupinaenable" value="1">';
$output .= '<label for="tupinaenable">Enable Tupina\'o-meter</label><br>';
$output .= '<div class="help-tip">
    <p>Tupina\'o-meter&copy; :ll&auml; lis&auml;&auml;t peliin tupinaa ja jupinaa aiheuttavia elementtej&auml;. Tupina\'o-meter&copy; eroaa Tuhina\'o-meter&copy; :st&auml; siin&auml;, ett&auml; arvon asettaminen 0:ksi ei kuitenkaan poista tupinaa aiheuttavien korttien mahdollisuutta pelist&auml;.</p>
</div>';

/* End of the form table and form */
$output .= '</td></tr></table>';
$output .= '<input type="submit" value="Submit">';
$output .= '</form>';

/* Output the input form */
echo $output;

$printed_prizes[0] = 0;
$printed_prizes[1] = 0;
$printed_prizes[2] = 0;

$exclude_ids = array();

if ($tup_inafactor > 0) {
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

	$special_query_base .= card_query_start();
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

$query_start = card_query_start();
$query_where = card_query_where($tuh_inafactor, $exp, $exclude_ids);

$query_order = card_query_order($tuh_inafactor, $tup_inafactor);

$query_limit = card_query_limit(3 - $printed_prizes[0], 3 - $printed_prizes[1], 4 - $printed_prizes[2]);

$query_cheap = $query_start . ' ' . $query_where[0] . ' ' . $query_order . ' ' .$query_limit[0];
$query_mid = $query_start . ' ' . $query_where[1] . ' ' . $query_order . ' ' .$query_limit[1];
$query_exp = $query_start . ' ' . $query_where[2] . ' ' . $query_order . ' ' .$query_limit[2];


echo '<hr style="height:10px;border-width:0;color:#d2691e;background-color:#d2691e">';

/* On a mobile device we try to fit the tables on a screen */

$mobile = isMobileDevice();

/* Output cards */
if (!$mobile)
	echo '<table class="structure"><tr><td>';

print_cards($conn, $query_cheap, "Cards &lt; 4", $mobile);

if (!$mobile)
	echo '</td><td>';

print_cards($conn, $query_mid, "Cards 4", $mobile);

if (!$mobile)
	echo '</td><td>';

print_cards($conn, $query_exp, "Cards &gt; 4", $mobile);

if (!$mobile)
	echo '</td></tr></table>';

echo '<p><a href="aloittaja.php">Arvo aloittaja?</a>';

/* Close connection, print (c) and send </body> </html> */
require 'include/footer.php';

?>
