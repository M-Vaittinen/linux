<?php
if (isset($_POST['expansion'])) {
	if (is_numeric($_POST['expansion']))
		$expansion = $_POST['expansion'];
}

if (isset($_POST['data_sent']) && $_POST['data_sent'] == 1)
	$data_sent = true;
else
	$data_sent = false;

require '../include/db.php';
require '../include/header.php';
require '../include/dominion_common.php';

do_head("Top Secret Admin Page");

echo '<h1>Add Card</h1>';

function view_last_added_cards($conn, &$output)
{
	$sql = "SELECT c.*, e.name AS expansion FROM cards AS c JOIN expansion as e ON e.id = c.expansion_id ORDER BY c.id DESC LIMIT 10";
	$result = mysqli_query($conn, $sql);

	if (mysqli_num_rows($result) > 0) {
	    $output .= "<table border='1' cellpadding='5'>";

	    // Fetch and print table headers
	    $fields = mysqli_fetch_fields($result);
	    $output .= "<tr>";
	    foreach ($fields as $field) {
	        $output .= "<th>" . htmlspecialchars($field->name) . "</th>";
	    }
	    $output .= "</tr>";

	    // Fetch and print rows
	    while ($row = mysqli_fetch_assoc($result)) {
	        $output .= "<tr>";
	        foreach ($row as $value) {
		       
			$output .= "<td>" . htmlspecialchars($value) . "</td>";
	        }
	        $output .= "</tr>";
	    }

	    $output .= "</table>";
	} else {
	    $output .= "No cards found.";
	}
}

function die_bad_input($reason)
{
	echo $reason."</br>\n";
	echo '<button onclick="history.go(-1);">Back </button>';
	die();
}

function pc_card_extract_types($typestr, &$typeout) {
	$tstr = str_replace(array("\n", "\r"), '', $typestr);
	$typeout = explode(" - ", $tstr);
}

function get_type_id($typearray)
{
	$type = $typearray[0];

	switch ($type)
	{
	case "Action":
		foreach ($typearray as $t)
			if ($t == "Treasure")
				return 3;
		return 2;
	case "Victory":
		return 4;
	case "Treasure":
		foreach ($typearray as $t)
			if ($t == "Action")
				return 3;
		return 1;
	case "Curse":
		return 4;
	case "Reaction":
		return 4;
	case "Event":
		return 4;
	case "Landmark":
		return 4;
	case "Night":
		return 4;
	case "Boon":
		return 4;
	case "Hex":
		return 4;
	case "State":
		return 4;
	case "Project":
		return 4;
	case "Artifact":
		return 4;
	case "Way":
		return 4;
	case "Ally":
		return 4;
	case "Trait":
		return 4;
	case "Prophecy":
		return 4;
	default:
		echo "Bad types: ";
		foreach ($typearray as $t)
			echo "$t, ";
		echo "Unknown type[0] ".$type;
		break;
	}
	return -1;
}

function get_cost($str)
{
	if (is_numeric($str))
		return $str;

	/* Set cost as 0 if we have no proper cost value set */
	return "0";
}

/*
 * Due to errors in previous stages of processing, the cost
 * which isn't a coin is NULL in the database.
 *
 * Treat all non numeric cost values as debt for now.
 */
function guess_if_cost_is_debt($str)
{
	return !is_numeric($str);
}

function get_dummy($dummy)
{
	if ($dummy == NULL)
		$dummy = '';
	return $dummy;
}

function __contains_type($typearray, $type)
{
	foreach ($typearray as $t)
		if ($t == $type)
			return true;

	return false;

}

function get_hex($typearray)
{
	return __contains_type($typearray, "Doom");
}

function get_boon($typearray)
{
	return __contains_type($typearray, "Fate");
}

function get_night($typearray)
{
	return __contains_type($typearray, "Night");
}

function get_heirloom($typearray)
{
	return __contains_type($typearray, "Heirloom");
}

function get_attack($typearray)
{
	return __contains_type($typearray, "Attack");
}

function get_endure($typearray)
{
	return __contains_type($typearray, "Duration");
}

function get_gather($typearray)
{
	return __contains_type($typearray, "Gathering");
}
function get_thrash($str)
{
	return ($str != NULL);
}
/*
 * There is not such official category as a 'defend' for the cards.
 * However, most of the 'Reaction' cards can be seen defensive. Not all
 * though.
 */
function guess_defend($typearray)
{
	foreach ($typearray as $t)
		if ($t == "Reaction")
			return true;

	return false;
}

/*
 * The parsed information did not contain explicit data when some of the cards
 * can cause curses to be picked. Let's provide card adder a hint about cursing
 * by scanniong the free text field for 'Curse' keyword.
 */
function guess_curse($text)
{
	return stripos($text, 'Curse');
}

function guess_drop($text)
{
	/*
	 * We assume a card may cause other players to drop cards from their hands if
	 * the free text contains a word 'discard'. In addition to this, the card should
	 * probably be an attack card - which should be checked prior calling this function.
	 */
	return stripos($text, 'discard');
}
function add_card_to_form($conn, $row, $c_index, &$output)
{
	$known_types[1] = 'raha';
	$known_types[2] = 'toiminto';
	$known_types[3] = 'raha/toiminto';
	$known_types[4] = 'sekalainen';

	/*
	 * This may break the form if parsed_cards contains bad data.
	 * Well, the idea has been that the database data hasn't been processed
	 * with the htmlspecialchars() - only with mysqli_real_escape_string()
	 * because the data we get from card adding form is POST'd by user - so
	 * we need to treat it as unsafe anyways. And, as we don't know if the
	 * data we get from the card adding form comes from the user or
	 * parsed_cards, we need to escape it. If the parsed_cards contained
	 * data processed by htmlspecialchars() - then we might end up processing
	 * it _again_ when handling the form data. So, we decide to trust the
	 * parsed_cards data here and dump it to the form unprocessed.
	 */
	$pcid = $row['id'];
	$en_name = $row['name'];
	$no_finnish = $row['no_finnish'];
	$exp_id = $row['exp_id'];
	$type = $row['type'];
	$typearray = array();

	pc_card_extract_types($type, $typearray);
	$type_id = get_type_id($typearray);
	if ($type_id < 0)
		die ("ID: $pcid, name $en_name bad type");
	$cost = get_cost($row['cost']);
	$coins = get_dummy($row['coins_coffer']);
	$debt = guess_if_cost_is_debt($row['cost']);
	$draws = get_dummy($row['draws']);
	$buys = get_dummy($row['buys']);
	$thrash = get_thrash($row['trash_ret']);

	$heirloom = get_heirloom($typearray);
	$hex = get_hex($typearray);
	$boon = get_boon($typearray);
	$night = get_night($typearray);

	$attack = get_attack($typearray);
	$defend = guess_defend($typearray);
	$endure = get_endure($typearray);
	$gather = get_gather($typearray);
	$curse = guess_curse($row['text']);
	/*
	 * We also have no proper information whether a card can cause others to drop cards from hand
	 * in the parsed data. We give card adder a hint based on two things:
	 * 1. Card is an attack. Non attack cards aren't likely to cause others to drop cards from hand.
	 * 2. Free text heuristics as implemented in guess_drop()
	 */
	$drop =  ($attack && guess_drop($row['text']));

	$output .= '<tr>'."\n";
	$output .= '<td><input type="hidden" name="c' . $c_index .'_pcid" value="'.$pcid.'">
		<input type="checkbox" name="c' . $c_index .'_skip_id" value="'.$pcid.'">'."\n";
	$output .= '<td><input class="mandatory" type="text" name="c' . $c_index .'_en_name" value="'. $en_name .'" required></td>'."\n";
	$output .= '<td><input type="text" name="c' . $c_index .'_fi_name" value=""></td>'."\n";
	$output .= '<td><select name="c' . $c_index .'_type" id="c' . $c_index .'_type">'."\n";
	foreach($known_types as $tid => $typ) {
		if ($type_id != $tid)
			$output .= '<option value="'.$tid.'">' .$typ. '</option>'."\n";
		else
			$output .= '<option value="'.$tid.'" selected>' .$typ. '</option>'."\n";
	}
	$output .= '</select></td>'."\n";

	$checked = ($debt) ? ' checked' : '';

	$output .= '<td><input class="mandatory" type="text" size="10" name="c' . $c_index .'_cost" value="'.$cost.'" required>
			<input type="checkbox" name="c' . $c_index .'_debt" value="1"'.$checked.'></td>'."\n";
	/*$output .= '<td><input class="mandatory" type="text" size="10" name="c' . $c_index .'_coins" value="'.$coins.'" required></td>'."\n"; */
	$output .= '<td><input type="text" size="10" name="c' . $c_index .'_coins" value="'.$coins.'"></td>'."\n";
	$output .= '<td><input type="text" size="10" name="c' . $c_index .'_draws" value="'.$draws.'"></td>'."\n";
	$output .= '<td><input type="text" size="10" name="c' . $c_index .'_buys" value="'.$buys.'"></td>'."\n";

	$checked = ($thrash) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_thrash" value="1"'.$checked.'></td>';

	$checked = ($attack) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_attack" value="1"'.$checked.'></td>';

	$checked = ($defend) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_defend" value="1"'.$checked.'></td>';

	$checked = ($endure) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_endure" value="1"'.$checked.'></td>';

	$checked = ($gather) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_gather" value="1"'.$checked.'></td>';

	$checked = ($curse) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_curse" value="1"'.$checked.'></td>';

	$checked = ($heirloom) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_heirloom" value="1"'.$checked.'></td>';

	$checked = ($hex) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_hex" value="1"'.$checked.'></td>';

	$checked = ($boon) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_boon" value="1"'.$checked.'></td>';

	$checked = ($night) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_night" value="1"'.$checked.'></td>';

	$checked = ($drop) ? ' checked' : '';
	$output .= '<td><input type="checkbox" name="c' . $c_index .'_drop" value="1"'.$checked.'></td>';

	$output .= '<td><input type="text" size="4" name="c' . $c_index .'_setup_extras_id" value=""></td>';
	$output .= '<td><input type="text" size="4" name="c' . $c_index .'_tuhina" value=""></td>';
	$output .= '</tr>';

	/*
	 * These need to be filled. All need to be confirmed by user.
	 * Some can be pre-filled from the parsed_cards values.
	 * TODO: Build form with card infos and ask user to fill missing pieces
	 */
//	$dual_top_of_id = '';
//	$dual_below_id = '';
	/*	$prizetype_id =  */
	/*	$prize = */
}

function get_not_added_cards($conn, $exp_id)
{
	$query = "SELECT * FROM parsed_cards WHERE exp_id = ".$exp_id." AND added != 1 AND skip != 1 LIMIT 5";
	$result = mysqli_query($conn, $query);
	if (!$result)
		die_bad_input("no more cards to add for expansion ".$exp_id.". ".mysqli_error($conn));

	if (mysqli_num_rows($result) <= 0)
		die_bad_input("still no more cards to add for expansion ".$exp_id);

	return $result;
}

function get_post_str_data($conn, $idx, $name, &$value)
{
	$key = 'c' . $idx .'_'.$name;

	if (!isset($_POST[$key]) || $_POST[$key] == "" || strlen(trim($_POST[$key])) == 0)
		return false;

	$value = mysqli_real_escape_string($conn, $_POST[$key]);

	return true;
}

function require_post_num_data_default_zero($conn, $idx, $name, &$value)
{
	$key = 'c' . $idx .'_'.$name;

	if (!isset($_POST[$key]) || $_POST[$key] == "") {
		$value = 0;
		/*		echo "$name [$idx]: set to $value </br>"; */
		return true;
	}
	if (!is_numeric($_POST[$key]))
		die_bad_input("Card $idx: '$name': Not numeric ($_POST[$key])");

	$value = $_POST[$key];
		/*	echo "$name [$idx]: set to $value </br>"; */
	return true;
}

function get_post_num_data($conn, $idx, $name, &$value)
{
	$key = 'c' . $idx .'_'.$name;

	if (!isset($_POST[$key]))
		return false;

	if (!is_numeric($_POST[$key]))
		return false;

	$value = $_POST[$key];

	return true;
}

function get_post_unsigned_num_data_range($conn, $idx, $name, $max, &$value)
{
	$ret = get_post_num_data($conn, $idx, $name, $tmp);
	if (!$ret)
		return false;

	if ($tmp < 0 || $tmp > $max)
		return false;

	$value = $tmp;

	return true;
}

function require_post_num_data_default_zero_range($conn, $idx, $name, $max, &$value)
{
	$key = 'c' . $idx .'_'.$name;

	require_post_num_data_default_zero($conn, $idx, $name, $value);
	if ($value < 0 || $value > $max)
		die_bad_input("Card $idx: $name out of range ($value, [0 .. $max])");
}

function do_skip($conn, $id)
{
	$query = "UPDATE parsed_cards SET skip = 1 WHERE id = $id LIMIT 1";

	if (!mysqli_query($conn, $query))
		die("failed to set $id skipped ".mysqli_error($conn));

	/* echo "card ID $id marked to be skipped </br>"; */
}

function validate_and_skip($conn, $id)
{
	$validate_sql = "SELECT id FROM parsed_cards WHERE id = $id AND added != 1 AND skip != 1";

	$result = mysqli_query($conn, $validate_sql);
	if (!$result || mysqli_num_rows($result) != 1)
		die_bad_input("Can't skip card ID $id".mysqli_error($conn));

	do_skip($conn, $id);
}

function __valid_name($str, $fatal)
{
	if (!$str || $str == "")
		if ($fatal)
			die_bad_input('No required card name given');
		else
			return false;
/*
 * I'd like to also accept names like "Devil's Workshop", where "'" is not alphanumeric.
	if (!preg_match('/^[\p{L} ]+$/u', $str))
		if ($fatal)
			die_bad_input("Bad card name ($str)</br> Expected alphanumeric only");
		else
			return false;
 */
	if (strlen($str) < 2)
		if ($fatal)
			die_bad_input("Card name too short: '$str'");
		else
			return false;

	if (strlen($str) > 254)
		if ($fatal)
			die_bad_input("Card name too long: '$str'");
		else
			return false;

	return true;
}

function valid_name($str)
{
	return __valid_name($str, false);
}

function valid_name_or_die($str)
{
	__valid_name($str, true);
}

function check_n_add_to_cards($conn, $i, $expansion, $setup_extras_max_id)
{
	$known_types[1] = 'raha';
	$known_types[2] = 'toiminto';
	$known_types[3] = 'raha/toiminto';
	$known_types[4] = 'sekalainen';

	if (!get_post_unsigned_num_data_range($conn, $i, "pcid", 10000, $pcid))
		die_bad_input("Card$i: bad ID $pcid");

	if (get_post_unsigned_num_data_range($conn, $i, "skip_id", 10000, $ret))
		return validate_and_skip($conn, $ret);

	if (get_post_str_data($conn, $i, "en_name", $en_name))
		valid_name_or_die($en_name);
	else
		die_bad_input('No en_name for card $i');

	if (get_post_str_data($conn, $i, "fi_name", $fi_name)) {
		if (!valid_name($fi_name))
			die_bad_input("Bad fi_name given for card $i");
	} else {
		$fi_name = $en_name;
	}

	if (!get_post_unsigned_num_data_range($conn, $i, "type", 4, $type_id) || $type_id == 0)
		die_bad_input("Card$i: Unknown or unsupported type-id '$type_id'");

	if (!get_post_unsigned_num_data_range($conn, $i, "cost", 100, $cost))
		die_bad_input("Card$i: Unknown or unsupported cost '$cost'");

	require_post_num_data_default_zero_range($conn, $i, "coins", 20, $coins);
	require_post_num_data_default_zero_range($conn, $i, "draws", 20, $draws);
	require_post_num_data_default_zero_range($conn, $i, "buys", 20, $buys);
	require_post_num_data_default_zero_range($conn, $i, "setup_extras_id", $setup_extras_max_id, $setup_extras_id);
	require_post_num_data_default_zero_range($conn, $i, "tuhina", 10, $tuhina);
	require_post_num_data_default_zero_range($conn, $i, "debt", 1, $debt);
	if ($debt == 1)
		$prizetype = 2;
	else
		$prizetype = 1;

	$setfields = array("name = '$fi_name'");
	$setfields[] = "en_name = '$en_name'";
	$setfields[] = "expansion_id = $expansion";
	$setfields[] = "type_id = $type_id";
	$setfields[] = "prize = $cost";
	$setfields[] = "drawcards = $draws";
	$setfields[] = "buys = $buys";
	$setfields[] = "prizetype_id = $prizetype";
	$setfields[] = "tuhinakerroin = $tuhina";
	$setfields[] = "actionmoney = $coins";

	/* Hack. The HEX/BOON IDX defines should match the "hex" / "boon" index in arrays below. This is used to automatically set the
	 * setup extras ID for hex and boon cards - to defined index. This works _only_ for as long as the boon/hex setup_extras_idx
	 * is not altered in the database, and for as long as the arrays and the define below stay in sync.
	 */
	$HEX_IDX = 7; /* Keep in sync with $bool_vals / $columns */
	$HEX_DATABASE_SETUP_ID = 2; /* Must match the database */
	$BOON_IDX = 8; /* Keep in sync with $bool_vals / $columns */
	$BOON_DATABASE_SETUP_ID = 1; /* Must match the database */
	$bool_vals = array("thrash", "attack", "defend", "endure", "gather", "curse", "heirloom", "hex", "boon", "night", "drop");
	$columns = array("destroy", "attack", "defence", "endure", "gather", "curse", "heirloom", "hex", "boon", "night", "dropcards");

	foreach ($bool_vals as $index => $bv) {
		$ret = 0;
		if (get_post_unsigned_num_data_range($conn, $i, $bv, 1, $ret))
			$setfields[] = $columns[$index]." = $ret";
		/* Set Hex/Boon setup extras if no specific extra was given */
		if (!$setup_extras_id) {
		       if ($index == $HEX_IDX && $ret)
			       $setup_extras_id = $HEX_DATABASE_SETUP_ID;
		       else if ($index == $BOON_IDX && $ret)
			       $setup_extras_id = $BOON_DATABASE_SETUP_ID;
		}
	}
	$setfields[] = "setup_extras_id = $setup_extras_id";
	$query = "INSERT INTO cards SET";
	foreach ($setfields as $idx => $set) {
//		echo "add element[$idx] $set";
		if ($idx == 0)
			$query .= " $set";
		else
			$query .= ", $set";
	}
	$query2 .= "UPDATE parsed_cards SET added = 1 WHERE id = $pcid LIMIT 1"; 

	/* Start transaction */
	mysqli_begin_transaction($conn);

	try {
		debug_print($query);
		mysqli_query($conn, $query);
		debug_print($query2);
		mysqli_query($conn, $query2);
		mysqli_commit($conn);
		//echo "card '$en_name' Added </br>\n";
	} catch (\Throwable $e) {
		mysqli_rollback($conn);
		die("Sorry. Mysterious error. Failed to add card '$en_name'");
	}
}

function print_recv($conn, $i)
{
	$known_types[1] = 'raha';
	$known_types[2] = 'toiminto';
	$known_types[3] = 'raha/toiminto';
	$known_types[4] = 'sekalainen';

	if (get_post_unsigned_num_data_range($conn, $i, "skip_id", 10000, $ret))
		echo "skip_id$i: $ret </br>";
	if (get_post_str_data($conn, $i, "en_name", $ret))
		echo "en_name$i: $ret </br>";
	if (get_post_str_data($conn, $i, "fi_name", $ret))
		echo "fi_name$i: $ret </br>";
	if (get_post_unsigned_num_data_range($conn, $i, "type", 4, $type_id)) {
		echo "type_id$i: $type_id ";
		if ($type_id > 0)
			echo "which means type '".$known_types[$type_id]."' </br>";
	}
	if (get_post_unsigned_num_data_range($conn, $i, "cost", 100, $ret))
		echo "cost$i: $ret </br>";
	if (get_post_unsigned_num_data_range($conn, $i, "coins", 100, $ret))
		echo "coins$i: $ret </br>";
	if (get_post_unsigned_num_data_range($conn, $i, "draws", 100, $ret))
		echo "draws$i: $ret </br>";
	if (get_post_unsigned_num_data_range($conn, $i, "buys", 100, $ret))
		echo "buys$i: $ret </br>";

	$bool_vals = array("debt", "thrash", "attack", "defend", "endure", "gather", "curse", "heirloom", "hex", "boon", "night", "drop");
	foreach ($bool_vals as $bv) {
		if (get_post_unsigned_num_data_range($conn, $i, $bv, 1, $ret))
			echo "$bv$i: $ret </br>";
	}

	if (get_post_unsigned_num_data_range($conn, $i, "tuhina", 10, $ret))
		echo "tuhina$i: $ret </br>";
}

function valid_expansion_id_or_die($conn, $id)
{
	$valid = false;

	$result = get_expansions($conn, true);
	while ($row = mysqli_fetch_assoc($result)) {
		if ($row['id'] == $id) {
			$valid = true;
			break;
		}
	}

	if (!$valid)
		die('Bad expansion ID ');
}

function create_setup_extras_table($setup_extras_res)
{
	$output = '<table><tr><th>id</th><th>text</th></tr>'."\n";
	while ($row = mysqli_fetch_assoc($setup_extras_res))
		$output .= '<tr><td>'.$row['id'].'</td><td>'.$row['text'].'</td></tr>';
	$output .= '</table>';

	return $output;
}

/*
 * Script starts
 */

if (!isset($expansion)) {

	$result = get_expansions($conn, true);
	$output = 'Select expansion to add cards for:</br>'."\n";
	$output .= '<form action="" method="post">'."\n";
	$output .= '<select name="expansion" id="expansion">'."\n";
	while ($row = mysqli_fetch_assoc($result)) {
		$output .= '<option value="'. $row['id'] . '">' .$row['name'] . '</option>'."\n";
	}
	$output .= "</select>\n";
	$output .= '<input type="submit" value="pick expansion"></form>'."\n";

	echo $output;
	die();
} else {
	valid_expansion_id_or_die($conn, $expansion);
}

$setup_extras_query = "SELECT id, text FROM setup_extras ORDER BY id";
$setup_extras_res = mysqli_query($conn, $setup_extras_query);
if (!$setup_extras_res)
	die("Ei setup_extras_res".mysqli_error($conn));

mysqli_data_seek($setup_extras_res, mysqli_num_rows($setup_extras_res) - 1);
$row = mysqli_fetch_assoc($setup_extras_res);
$setup_extras_max_id = $row['id'];
mysqli_data_seek($setup_extras_res, 0);

if ($data_sent) {
	if (!isset($expansion))
		die('Missing Expansion');
	for ($i = 0; $i < 5; $i++)
		check_n_add_to_cards($conn, $i, $expansion, $setup_extras_max_id);
}

$tuhina_query = "SELECT name, en_name, tuhinakerroin FROM cards ORDER BY tuhinakerroin DESC LIMIT 5";
$tuhina_hi_res = mysqli_query($conn, $tuhina_query);
if (!$tuhina_hi_res)
	die("Ei hi tuhise ".mysqli_error($conn));

$tuhina_query = "SELECT name, en_name, tuhinakerroin FROM cards WHERE tuhinakerroin != 0 ORDER BY tuhinakerroin ASC LIMIT 5";
$tuhina_lo_res = mysqli_query($conn, $tuhina_query);
if (!$tuhina_lo_res)
	die("Ei lo tuhise ".mysqli_error($conn));

$bool_opts = array("Thrashes", "Attacks", "Defends", "Endures", "Gathers", "Curses", "Heirloom", "Hex", "Boon", "Night" ,"Drops (from other playeres hands)");
$output = '';

$output .= create_setup_extras_table($setup_extras_res);

$output .= '<form action="" method="post">';
$output .= '<input type="hidden" name="expansion" value="'.$expansion.'">';
$output .= '<input type="hidden" name="data_sent" value="1">';
$output .= "Please check all the fields and fill missing information";
$output .= "<p> Set the 'Draws' and 'Buys' to a single value. ";
$output .= '<table class="cardlist"><tr>';
$output .= '<th>never add this</th><th>En name</th><th>Fi name</th><th>Type</th><th>Cost [debt]</th><th>Coins</th><th>Draws</th><th>Buys</th>';
//$output .= '<th>never add this</th><th>En name</th><th>Fi name</th><th>Type</th><th>Cost [debt]</th><th>Draws</th><th>Buys</th>';
foreach ($bool_opts as $opt)
	$output .= '<th>'.$opt.'</th>';
$output .= '<th>setup extras ID</th><th>tuhinakerroin 0-10</th>';
$output .= '</tr>';

$res = get_not_added_cards($conn, $expansion);
for ($idx = 0; $row = mysqli_fetch_assoc($res); $idx++) {
	add_card_to_form($conn, $row, $idx, $output);
}
$output .= '</table>';
$output .= '<input type="submit" value="Add cards">'."\n";
$output .= '</form>';
$output .= '<h3>Examples of cards and their respective tuhinafactors:</h3>';
$output .= '<table class="structure"><tr><th>name</th><th>name en</th><th>tuhinafactor</th></tr>';
while ($row = mysqli_fetch_assoc($tuhina_hi_res))
	$output .= '<tr><td>'.$row['name'].'</td><td>'.$row['en_name'].'</td><td>'.$row['tuhinakerroin'].'</td><tr>';

while($row = mysqli_fetch_assoc($tuhina_lo_res))
	$lores[] = $row;

array_reverse($lores, true);

foreach($lores as $lore)
	$output .= '<tr><td>'.$lore['name'].'</td><td>'.$lore['en_name'].'</td><td>'.$lore['tuhinakerroin'].'</td><tr>';

$output .= '</table>';

$output .= '<h3>Lastly added cards</h3>';
view_last_added_cards($conn, $output);

echo $output;

?>

