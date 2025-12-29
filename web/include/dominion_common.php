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

function get_expansions($conn, $include_nocards = false, $plain = false)
{
	/*
	 * This could probably be improved. For the 'admin' stuff (nocards), we should only get the expansions
	 * which have (addable) cards in 'parsed_cards' table. This is not really an issue though, as there is
	 * a limited number of expansions.
	 *
	 * For the user-selectible expansion set (Eg, show which expansions user can use to fetch cards from)
	 * must display only those expansions which have cards added to randomizer - and which aren't disabled.
	 * This is working Ok with the current query, but perhaps we could do more efficient (RIGHT?) JOIN.
	 */
	if ($include_nocards)
		$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN parsed_cards as pc WHERE e.id = pc.exp_id";
	else
		$query = "SELECT DISTINCT e.id, e.name, events.expansion_id AS expansion_events, landmarks.expansion_id AS expansion_landmarks FROM expansion AS e JOIN cards as c LEFT JOIN events AS events ON events.expansion_id = e.id LEFT JOIN landmarks as landmarks ON landmarks.expansion_id = e.id WHERE e.disabled != 1 AND c.expansion_id = e.id";

	debug_print("Querying expanisons: $query");

	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no expansions".mysql_error($conn));

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

function output_input_form($conn, $mobile, $exp, $land_exp, $event_exp, $tuh_inafactor, $tup_inafactor, $kap_itafactor)
{
	$onloads = null;
	$result = get_expansions($conn, false, true);

	$mobile = 1;
	/* Output the form table */

	$output = '<form action="" method="post" id="theform">'."\n";
	/*
	if (!$mobile) {
	//	$output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Tuhina\'o-meter</th> <th>Tupina\'o-meter</th><th>Kapita\'o-meter</th></tr><tr><td>';
		//$output .= '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Tuhina\'o-meter</th> <th>Tupina\'o-meter</th><th>Kapita\'o-meter</th></tr>';
		$output .= '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Painotukset</th></tr>'."\n";
		} else {
		// $output = '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Painotukset</th></tr><tr><td>';
		$output .= '<table class="structure"><tr><th>Käytettävät lisäosat</th><th>Painotukset</th></tr>'."\n";
		}

	$output .= '<tr><td>'."\n";
	 */
$output .= '<h3>Käytettävät lisäosat</h3>';
	$output .= '<table class="structure">'."\n"; //expansion table
	// $output .= '<form action="" method="post" id="theform">';
	/* Print expansion checkboxes */
	$tmp_exp_idx = 0;

	$onload_ids = null;
	while ($row = mysqli_fetch_assoc($result)) {
		$hidden_onclick = '';
		$hidden_onclick_landmark = '';

		if ($row['expansion_events'] != NULL || $row['expansion_landmarks'] != NULL) {
			$related_id = htmlspecialchars('exp_'.$row['id']).'Related';
			$hidden_onclick = 'onclick="toggleCheckboxes(this, \''.$related_id.'\')"';
//			$hidden_onclick = 'onclick="toggleCheckboxes(document.getElementById(\''.htmlspecialchars($row['name']).'\'), \''.$related_id.'\')"';
			$onload_ids[] = $related_id;
		}

		if (isset($exp) && $exp && isset($exp[$tmp_exp_idx]) && $row['id'] == $exp[$tmp_exp_idx]) {
			$checked = 'checked';
			$tmp_exp_idx ++;
		} else {
			$checked = "";
		}
	
		if (!is_numeric($row['id']))
			die("bad data in database - expansion id");
	
		$output .= '<tr><td>'."\n";
		$output .= '<div class="checkbox-container">'."\n";
		$output .= '<input class="checkboxes" type="checkbox" ';
		$output .= 'id="' . htmlspecialchars($row['name']) . '" ';
		$output .= 'name="expansion[]" ';
		$output .= $hidden_onclick .' ';
		$output .= 'value="' . $row['id'] . '" ';
		$output .= $checked.">\n";

		$output .= '<label class="checkboxes" for="' . htmlspecialchars($row['name']) . '">' . htmlspecialchars($row['name']) . '</label>'."\n";
		if ($row['expansion_events'] != NULL) {
			$evchecked = "";
			if ($event_exp) {
				foreach($event_exp AS $e_exp) {
					if ($e_exp == $row['id']) {
						$evchecked = ' checked';
					}
				}
			}
			$output .= '<input type="checkbox" id="exp_'. $related_id .'" name="event_expansions[]" value="' . $row['id'] . '" class="'.$related_id.' hidden"'."$evchecked>\n";
			$output .= '<label for="exp_' . $related_id . '" class="'.$related_id.' hidden">Tapahtumat</label>'."\n";
		}
		if ($row['expansion_landmarks'] != NULL) {
			$landchecked = "";
			if ($land_exp){
				foreach($land_exp AS $l_exp) {
					if ($l_exp == $row['id']) {
						$landchecked = ' checked';
						break;
					}
				}
			}

			$output .= '<input type="checkbox" id="land_'. $related_id .'" name="landmark_expansions[]" value="' . $row['id'] . '" class="'.$related_id.' hidden"'."$landchecked>";
			$output .= '<label for="land_' . $related_id . '" class="'.$related_id.' hidden">Maamerkit</label>'."\n";
		}
		if ($row['expansion_events'] != NULL || $row['expansion_landmarks'] != NULL) {
			$onloads[] = 'toggleCheckboxes(document.getElementById(\''. htmlspecialchars($row['name']) . '\'), \''.$related_id.'\');'."\n";
/*			$output .= '<script>';
			$output .= 'window.onload = function() {'."\n";
			$output .= 'toggleCheckboxes(document.getElementById(\''. htmlspecialchars($row['name']) . '\'), \''.$related_id.'\');'."\n";
			$output .= '}</script>'."\n"; */
		}
		$output .= '</div>'."\n";
		$output .= '</td></tr>'."\n";
	}
	$output .= '</table>'."\n"; // expansion table

	if ($onloads) {
		$output .= '<script>';
		$output .= 'window.onload = function() {'."\n";
		foreach($onloads AS $onload)
			$output .= $onload;
		$output .= '}</script>'."\n";
	}

	$output .= '<h3>Painotukset</h3>';
/*	$output .= '</td> <td>'."\n"; // Overall structure table
 */
	$output .= '<table class="structure">'."\n"; //ometer table
	$output .= '<tr><td>'."\n";

	/* ...Tuhina cell: */
	//$output .= '<td>';
	if ($mobile)
		$output .= '<b>Tuhina\'o-meter</b> <br />'."\n";

	$output .= '<div class="slidecontainer">'."\n".'
  <span class="label">-10</span>'."\n".'
  <div class="slider-wrapper">'."\n".'
    <input type="range" min="-10" max="10" value="'.$tuh_inafactor.'" class="slider" name="tuhinarange">'."\n".'
    <div class="value-bubble">0</div>'."\n".'
  </div>'."\n".'
  <span class="label">+10</span>'."\n".'
</div> '."\n";

	$output .= '<div class="help-tip">'."\n".'
	    <p>Tuhina\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa v&auml;hent&auml;m&auml;&auml;n tai lis&auml;&auml;m&auml;&auml;n toimintoketjuja lis&auml;&auml;vi&auml; kortteja.</p>
	</div>'."\n";
	$output .= '</td>'."\n";
	
	if ($mobile) {
		/* On a mobile we end the row here */
		//$output .= '</tr><tr><td></td>';
		$output .= '</tr><tr>'."\n";
	}

	/* Tupina cell: */
	$output .= '<td>'."\n";
	if ($mobile)
		$output .= '<b>Tupina\'o-meter</b><br />'."\n";
	$output .= '<div class="slidecontainer">
  <span class="label">-10</span>
  <div class="slider-wrapper">
    <input type="range" min="-10" max="10" value="'.$tup_inafactor.'" class="slider" name="tupinarange">
    <div class="value-bubble">0</div>
  </div>
  <span class="label">+10</span>
</div>'."\n";


	$output .= '<input type="checkbox" id="add_nihilism" name="add_nihilism" value="1">'."\n";
	$output .= '<label for="add_nihilism">...ripauksella nihilismi&auml;</label><br>'."\n";
	$output .= '<div class="help-tip">
	    <p>Tupina\'o-meter&copy; :ll&auml; v&auml;henn&auml;t tai lis&auml;&auml;t peliin tupinaa ja jupinaa aiheuttavia elementtej&auml;.<br /><br />Ja jos todella haluat koetella k&auml;rsiv&auml;llisyytesi rajoja niin voit h&ouml;yst&auml;&auml; peli&auml; ripauksella nihilismi&auml; ja pienent&auml;&auml; rahaa ja vastavetoja tuovien toimintakorttien mahdollisuutta.</p>
	</div>'."\n";
	$output .= '</td>'."\n";

	if ($mobile) {
		/* On a mobile we end the row here */
		//$output .= '</tr><tr><td></td>';
		$output .= '</tr><tr>'."\n";
	}

	/* ...Kapita cell: */
	$output .= '<td>'."\n";
	if ($mobile)
		$output .= '<b>Kapita\'o-meter</b><br />'."\n";

	$output .= '<div class="slidecontainer">
  <span class="label">-10</span>
  <div class="slider-wrapper">
    <input type="range" min="-10" max="10" value="'.$kap_itafactor.'" class="slider" name="kapitarange">
    <div class="value-bubble">0</div>
  </div>
  <span class="label">+10</span>
</div>'."\n";

	$output .= '<div class="help-tip">
	    <p>Kapita\'o-meter&copy; :ll&auml; voit muuttaa korttiarvontaa priorisoimaan raha- ja rahaa lis&auml;&auml;vi&auml; toimintakortteja.</p>
	</div>'."\n";
	$output .= '</td>'."\n";

	/* End of the form table and form */
	$output .= '</tr></table>'."\n";

	/*	$output .= '</td></tr></table>'."\n"; */

	$output .= '<input type="submit" value="Arvo kortit">'."\n";
	$output .= '</form>'."\n";
/*
	if ($onload_ids) {
		$output .= '<script>';
		$output .= 'window.onload = function() {'."\n";
		foreach($onload_ids AS $oid) {
			$output .= 'toggleCheckboxes(document.getElementById(\'foo\'), \''.$oid.'\');'."\n";
		}
		$output .= '};</script>';
	}
 */
	return $output;
}

?>
