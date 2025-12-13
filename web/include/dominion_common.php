<?php

$CARDTYPE_MONEY = 1;

function debug_print($str)
{
	global $DBG;

	if (!$DBG)
		return;

	echo "$str.<br />";
}

function get_expansions($conn, $include_nocards = true)
{
	if ($include_nocards)
		$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN parsed_cards as pc WHERE e.id = pc.exp_id";
	else
		$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN parsed_cards as pc JOIN cards as c WHERE e.id = pc.exp_id AND c.expansion_id = e.id";

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


?>
