<?php

require '../include/db.php';
require '../include/header.php';
require '../include/dominion_common.php';

if (isset($_POST['nofinnish'])) {
	$nofin = $_POST['nofinnish'];

	foreach ($nofin AS $id) {
		$id = mysqli_real_escape_string($conn, $id);
		$query = "UPDATE parsed_cards SET no_finnish = 1 WHERE id = $id";
		echo $query . "<br/>\n";
		$result = mysqli_query($conn, $query);
	}
}

if (isset($_POST['cards'])) {
	$cards = $_POST['cards'];
	/*	print_r($cards); */

	foreach ($cards AS $cardid => $pcid) {
		$cardid = mysqli_real_escape_string($conn, $cardid);
		$pcid = mysqli_real_escape_string($conn, $pcid);

		$query = "UPDATE cards SET en_name = ( SELECT name FROM parsed_cards WHERE id = " . $pcid . ") WHERE id = " .$cardid;
		echo $query . "</br>\n";		
		$result = mysqli_query($conn, $query);
		if (!$result)
			die(mysqli_error($conn));

		$query = "UPDATE parsed_cards SET en_added = 1 WHERE id = " . $pcid;
		echo $query . "</br>\n";
		$result = mysqli_query($conn, $query);
		echo "---</br>\n";
	}
}


function get_added_expansions($conn)
{
	$ids = array();
	$query = "SELECT DISTINCT expansion_id FROM cards";

	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no known expanisons");

	if (mysqli_num_rows($result) <= 0)
		die("still no expansions");
	while ($row = mysqli_fetch_assoc($result))
		$ids[] = $row['expansion_id'];

	mysqli_free_result($result);

	return $ids;
}

function add_card_pair($pcname, $cardsname, $pcid, $cardsid, &$output)
{
	$output .= "'".$pcname . "' -- '" . $cardsname."'".' <input type="checkbox" id="'.$cardsid.'" name="cards[' . $cardsid . ']" value="' . $pcid . '">';
	$output .= ' <input type="checkbox" id="'.$cardsid.'" name="nofinnish[]" value="' . $pcid . '">';
	$output .= "</br>\n";
}

//$exp_ids = get_added_expansions($conn);

$query = "SELECT pc.id AS pc_id, pc.name AS pc_name, cards.id AS c_id, cards.name AS c_name FROM parsed_cards AS pc JOIN cards ON pc.exp_id = cards.expansion_id WHERE pc.en_added != 1 AND cards.en_name IS NULL AND pc.no_finnish = 0";
/*
$query = "SELECT * from parsed_cards WHERE (added != 1 AND (";

$i = 0;
foreach($exp_ids as $eid) {
	if ($i != 0)
		$where .= " OR ";
	$where .= "exp_id ='" . $eid . "'";
	$i = 1;
}
$query .= $where . "))";
 */

$result = mysqli_query($conn, $query);
if (!$result)
	die("no parsed cards to compare to added");

$output = '<form action="" method="post">
	select corresponding:</br>';

	$prev = "";

	while ($row = mysqli_fetch_assoc($result)) {
		if ($prev != $row['pc_id']) {
			$output .= "-----------------<br/>\n";
			$prev = $row['pc_id'];
		}
		add_card_pair($row['pc_name'], $row['c_name'], $row['pc_id'], $row['c_id'], $output);
	}
	$output .= '<input type="submit" value="send">';
	$output .= '</form>';

	echo $output;
?>
