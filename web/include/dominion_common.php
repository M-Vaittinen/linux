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

$CARDTYPE_MONEY = 1;

function debug_print($str)
{
	global $DBG;

	if (!$DBG)
		return;

	echo "$str.<br />";
}

function get_expansions($conn, $include_nocards = true, $plain = false)
{
	if ($include_nocards)
		$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN parsed_cards as pc WHERE e.id = pc.exp_id";
	else
		$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN cards as c WHERE c.expansion_id = e.id";

	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no expansions");

	if (mysqli_num_rows($result) <= 0)
		die("still no expansions");

	return $result;
}

function query_cards($conn, $query)
{
	debug_print($query . '<br />');
	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no cards");

	if (($numc = mysqli_num_rows($result)) <= 0)
		die("still no cards");

	debug_print("Found $numc cards<br />");

	return $result;
}

function SQL_add_expansion_where($expansion_key, $expansion_id_array)
{
	$expansion_where = '';

	if (!isset($expansion_key))
		die ('expansion_key missing');

	if (isset($expansion_id_array) && $expansion_id_array) {
	$i = 0;
		$expansion_where .= '(';
		foreach($expansion_id_array as $e) {
			if (!is_numeric($e))
				die('Bad expansion ID');
			if ($i == 0)
				$expansion_where .= "$expansion_key = '" . $e . "'";
			else
				$expansion_where .= " OR $expansion_key = '" . $e . "'";
			$i++;
		}
		$expansion_where .= ')';
	}

	return $expansion_where;
}

function output_input_form($conn, $mobile, $exp)
{
	$result = get_expansions($conn, false, true);

	/* Output the form table */
	if (!$mobile) {
		$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Tuhina\'o-meter</th> <th>Tupina\'o-meter</th><th>Kapita\'o-meter</th></tr><tr><td>';
		} else {
		$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Painotukset</th></tr><tr><td>';
		}
	$output .= '<form action="" method="post" id="theform">';

	/* Print expansion checkboxes */
	$tmp_exp_idx = 0;
	while ($row = mysqli_fetch_assoc($result)) {
		if (isset($exp) && $exp && $row['id'] == $exp[$tmp_exp_idx]) {
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
  <span class="label">-10</span>
  <div class="slider-wrapper">
    <input type="range" min="-10" max="10" value="0" class="slider" name="tuhinarange">
    <div class="value-bubble">0</div>
  </div>
  <span class="label">+10</span>
</div> ';

	$output .= '<div class="help-tip">
	    <p>Tuhina\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa v&auml;hent&auml;m&auml;&auml;n tai lis&auml;&auml;m&auml;&auml;n toimintoketjuja lis&auml;&auml;vi&auml; kortteja.</p>
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
  <span class="label">-10</span>
  <div class="slider-wrapper">
    <input type="range" min="-10" max="10" value="0" class="slider" name="tupinarange">
    <div class="value-bubble">0</div>
  </div>
  <span class="label">+10</span>
</div>';


	$output .= '<input type="checkbox" id="add_nihilism" name="add_nihilism" value="1">';
	$output .= '<label for="add_nihilism">...ripauksella nihilismi&auml;</label><br>';
	$output .= '<div class="help-tip">
	    <p>Tupina\'o-meter&copy; :ll&auml; v&auml;henn&auml;t tai lis&auml;&auml;t peliin tupinaa ja jupinaa aiheuttavia elementtej&auml;.<br /><br />Ja jos todella haluat koetella k&auml;rsiv&auml;llisyytesi rajoja niin voit h&ouml;yst&auml;&auml; peli&auml; ripauksella nihilismi&auml; ja pienent&auml;&auml; rahaa ja vastavetoja tuovien toimintakorttien mahdollisuutta.</p>
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
  <span class="label">-10</span>
  <div class="slider-wrapper">
    <input type="range" min="-10" max="10" value="0" class="slider" name="kapitarange">
    <div class="value-bubble">0</div>
  </div>
  <span class="label">+10</span>
</div>';

	$output .= '<div class="help-tip">
	    <p>Kapita\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa priorisoimaan raha- ja rahaa lis&auml;&auml;vi&auml; toimintakortteja.</p>
	</div>';
	$output .= '</td>';

	/* End of the form table and form */
	$output .= '</tr></table>';
	$output .= '<input type="submit" value="Submit">';
	$output .= '</form>';

	return $output;
}

?>
