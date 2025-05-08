<?php

function get_expansions($conn)
{
	$query = "SELECT DISTINCT e.id, e.name FROM expansion AS e JOIN parsed_cards as pc WHERE e.id = pc.exp_id";
	$result = mysqli_query($conn, $query);
	if (!$result)
		die("no expansions");

	if (mysqli_num_rows($result) <= 0)
		die("still no expansions");

	return $result;
}

?>
