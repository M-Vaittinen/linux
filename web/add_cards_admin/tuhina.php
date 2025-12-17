<?php

/* Change this when you wish to allow updates to database. */
$NO_UPDATE = false;

if (isset($_POST['expansion'])) 
	$expansion = $_POST['expansion'];
else
	$expansion = null;

if (isset($expansion))
	foreach($expansion as $e)
		if (!is_numeric($e))
			die("Bad expansion ID $e");

$new_id = null;
$new_tuhinakerroin = null;
if (isset($_POST['id']) && isset($_POST['tuhinakerroin'])) {
	if (is_numeric($_POST['id']) && is_numeric($_POST['tuhinakerroin'])) {
		$new_id = $_POST['id'];
		$new_tuhinakerroin = $_POST['tuhinakerroin'];
	}
}

require '../include/db.php';
require '../include/header.php';
require '../include/dominion_common.php';

do_head("Tuhina-page");

$output  = '';

$output .= '<form action="" method="post" id="expaform">';
/* Print expansion checkboxes */
$tmp_exp_idx = 0;

$result = get_expansions($conn, false);
while ($row = mysqli_fetch_assoc($result)) {
	if ($expansion && $row['id'] == $expansion[$tmp_exp_idx]) {
		$checked = " checked";
		$tmp_exp_idx ++;
	} else {
		$checked = "";
	}

	if (!is_numeric($row['id']))
		die("bad data in database - expansion id");

	$output .= '<input form="expaform" type="checkbox" id="' . htmlspecialchars($row['name']) . '" name="expansion[]" value="' . $row['id'] . '"'."$checked>"."\n";
	$output .= '<label for="' . htmlspecialchars($row['name']) . '">' . htmlspecialchars($row['name']) . '</label><br>'."\n";
}
$output .= '<input form="expaform" type="submit" value="Submit">'."\n";
$output .= '</form>'."\n";


if ($new_id) {
	$new_query = "UPDATE cards SET tuhinakerroin = $new_tuhinakerroin WHERE id = $new_id LIMIT 1";

	if ($NO_UPDATE) {
		echo "<br />dry-run (no real update): <br />";
		echo $new_query."<br />\n";
	} else {
		if (!mysqli_query($conn, $new_query))
			die("Failed to update card ID $new_id to tuhina $new_tuhinakerroin <br />".mysqli_error($conn));
		echo "Updated <br />";
	}
}

$query = 'SELECT c.id AS c_id, c.name AS c_name, c.tuhinakerroin, e.name AS e_name FROM cards AS c JOIN expansion as e WHERE c.expansion_id = e.id';
$expwhere = SQL_add_expansion_where('c.expansion_id', $expansion);
if ($expwhere)
	$query .= ' AND '.$expwhere;
$query .= ' ORDER BY c.tuhinakerroin';

$result = query_cards($conn, $query);

//$output .= '<table><tr><th>card</th><th>expansion</th><th>tuhina</th><th></th></tr>'."\n";

$output .= '<div class="table">';
 while ($row = mysqli_fetch_assoc($result)) {
    $output .= '<form class="tr" method="post" action="">'."\n";
    $output .= '<span class="td">'."\n";
	$output .= '<input type="hidden" name="id" value="'.$row['c_id'].'">'."\n";
	if ($expansion)
		foreach($expansion as $e)
			$output .= '<input type="hidden" name="expansion[]" value="'.$e.'">'."\n";
	$output .= htmlspecialchars($row['c_name']).'</span>'."\n";
	$output .= '<span class="td">';
	$output .= 	htmlspecialchars($row['e_name']);
	$output .= '</span>'."\n";
	$output .= '<span class="td">';
	$output .= 	'<select name="tuhinakerroin" id="tuhinakerroin">'."\n";
	$output .=		'<option value="'.$row['tuhinakerroin'].'" selected>'.$row['tuhinakerroin']."</option>\n";
	for ($i = 0; $i <= 10; $i++)
		if ($i != $row['tuhinakerroin'])
			$output .= '<option value="'.$i.'">'.$i."</option>\n";
	$output .= '</select>'."\n";
	$output .= '</span><td>'."\n";
	$output .= '<span class="td">'."\n";
	$output .= '<input type="submit" value="Change"></span></form>'."\n";
}
$output .= '</div>'."\n";

echo $output;
require '../include/footer.php';

?>

