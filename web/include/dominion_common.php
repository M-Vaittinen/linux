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

?>
