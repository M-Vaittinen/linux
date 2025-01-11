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

if (isset($_POST['expansion']))
	$exp = $_POST['expansion'];

if (isset($_POST['tuhinaenable'])) {
	if (is_numeric($_POST['tuhinarange'])) {
		if ($_POST['tuhinarange'] <= 20 && $_POST['tuhinarange'] >= 0)
			$tuhinafactor = $_POST['tuhinarange'];
	}
}

$limit_cheap = 3;
$limit_mid = 3;
$limit_exp = 4;

require 'include/db.php';

echo "<h1>Dominion - Arvo kortit</h1>";

function print_cards($conn, $query, $title, $tuhinafactor)
{
	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no cards");

	if (mysqli_num_rows($result) <= 0)
		die("still no cards");

	$tuhinasum = 0;
	while ($row = mysqli_fetch_assoc($result)) {
		$tuhinasum += $row['tuhinakerroin'];
	}
	mysqli_data_seek($result, 0);

	$out = '<h3>' . $title . '</h3>
	<table class="cardlist"><tr>
		<th>Kortti</th> <th>Hinta</th> <th>Hintatyyppi</th> <th>Peliosa</th> <th>Korttityyppi</th></tr>';

	while ($row = mysqli_fetch_assoc($result)) {
		$name = $row['c_name'];
		$prize = $row['prize'];
	       	$prizetype = $row['p_name'];
		$expansion = $row['e_name'];
		$cardtype = $row['ct_name'];
		$out .= '<tr><td>' . $name . '</td><td>' . $prize . '</td><td>' . $prizetype . '</td><td>' . $expansion . '</td><td>' . $cardtype . '</td></tr>';
	}
	$out .= "</table>";
	echo $out;
	echo "Tuhina " . $tuhinasum;
}
$all = false;

if (!isset($exp))
	$all = true;
else
	foreach( $exp as $e)
		if (!is_numeric($e))
			die('Something went wrong');
// Create connection
$conn = mysqli_connect($servername, $username, $password, $dbname);

// Check connection
if (!$conn) {
  die("Connection failed: " . mysqli_connect_error());
}

$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN cards as c WHERE e.id = c.expansion_id";

$result = mysqli_query($conn, $query);
if (!$result)
	die("no expansions");

if (mysqli_num_rows($result) <= 0)
	die("still no expansions");

$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Tuhina\'o-meter (säädä tuhinamäärää)</th></tr><tr><td>';
$output .= '<form action=""' . ' method="post">';

/* Print expansion checkboxes */

while ($row = mysqli_fetch_assoc($result)) {
	$output .= '<input type="checkbox" id="' . $row['name'] . '" name="expansion[]" value="' . $row['id'] . '" checked>';
	$output .= '<label for="' . $row['name'] . '">' . $row['name'] . '</label><br>';
}

/* End of checkbox cell and ... */
$output .= '</td><td>';

/* ...start of slider cell and slider */

$output .= '<div class="slidecontainer">
  <input type="range" min="0" max="20" value="10" class="slider" name="tuhinarange" id="tuhinarange">
</div>';
$output .= '<input type="checkbox" id="tuhinaenable" name="tuhinaenable" value="1">';
$output .= '<label for="tuhinaenable">Enable Tuhina\'o-meter</label><br>';

/* End of the form table and form */
$output .= '</td></tr></table>';
$output .= '<input type="submit" value="Submit">';
$output .= '</form>';

/* Output the input form */
echo $output;

$tuhinalimit = '';
$tuhinaorder = '';

if (isset($tuhinafactor)) {
	if ($tuhinafactor == 0)
		$tuhinalimit = 'c.tuhinakerroin = 0 AND ';
	else
		$tuhinaorder = '/ ( c.tuhinakerroin * '. $tuhinafactor . ') DESC ';
}

$i = 0;
if ($all) {
	$query_cheap = "SELECT c.tuhinakerroin, c.prize, c.name AS c_name, p.name AS p_name, e.name AS e_name, ct.name AS ct_name FROM cards AS c JOIN prizetype as p JOIN expansion AS e JOIN cardtype AS ct WHERE p.id = c.prizetype_id AND c.expansion_id = e.id AND c.type_id = ct.id AND " . $tuhinalimit;
	$query_mid = $query_cheap . "c.prize = 4 ORDER BY RAND() " . $tuhinaorder  . "LIMIT " . $limit_mid;
	$query_exp = $query_cheap . "c.prize > 4 ORDER BY RAND() " . $tuhinaorder  . "LIMIT " . $limit_exp;
	$query_cheap .= "c.prize < 4 ORDER BY RAND() " . $tuhinaorder  . "LIMIT " . $limit_cheap;

} else {
	$query_cheap = "SELECT c.tuhinakerroin, c.prize, c.name AS c_name, p.name AS p_name, e.name AS e_name, ct.name AS ct_name FROM cards AS c JOIN prizetype as p JOIN expansion AS e JOIN cardtype AS ct WHERE p.id = c.prizetype_id AND c.expansion_id = e.id AND c.type_id = ct.id AND ". $tuhinalimit ."(c.expansion_id = '" . $exp[0] ."' ";
	foreach($exp as $e) {
		if ($i > 0) {
			$query_cheap .= "OR c.expansion_id = '" . $e . "' ";
		}
		$i++;
	}
	$query_mid = $query_cheap;
	$query_exp = $query_cheap;
	$query_cheap .= ") ORDER BY RAND() " . $tuhinaorder  . "LIMIT $limit_cheap";
	$query_mid .= ") ORDER BY RAND() " . $tuhinaorder  . "LIMIT $limit_mid";
	$query_exp .= ") ORDER BY RAND() " . $tuhinaorder  . "LIMIT $limit_exp";
}
/*
$result = mysqli_query($conn, $query_cheap);
while ($row = mysqli_fetch_assoc($result)) {
	var_dump($row);
	die('');
}
*/

/* Output cards */
echo '<table class="structure"><tr><td>';
print_cards($conn, $query_cheap, "Cards &lt; 4", $tuhinafactor);
echo '</td><td>';
print_cards($conn, $query_mid, "Cards 4", $tuhinafactor);
echo '</td><td>';
print_cards($conn, $query_exp, "Cards &gt; 4", $tuhinafactor);
echo '</td></tr></table>';

mysqli_close($conn);

require 'include/footer.php';

?>
