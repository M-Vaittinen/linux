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

$servername = "host";
$username = "username";
$password = "password";
$dbname = "database";
echo '
<!DOCTYPE html>
<html>
<head>
<style>
body {
  background-color: linen;
}

h1 {
  color: maroon;
  margin-left: 40px;
}

table.structure {
  border: none;
  border-spaceing: 18px;
  text-align: left;
  vertical-align: top;
}
.structure td {
  border: none;
  text-align: left;
  vertical-align: top;
}
.structure th {
}
.structure {
  border: none;
  text-align: left;
  vertical-align: top;
}
table.cardlist {
  border: 1px solid black;
  border-radius: 10px;
  border-collapse: collapse;
}

.cardlist th {
  border: 1px solid black;
  padding-top: 12px;
  padding-bottom: 12px;
  text-align: center;
  vertical-align: top;
  background-color: #d2691e;
  color: white;
  border-collapse: collapse;
}

.cardlist td {
  border: 1px solid black;
  border-collapse: collapse;
  padding-top: 12px;
  padding-bottom: 12px;
  vertical-align: top;
  text-align: center;
  background-color: ##ffdead;
  color: brown;
  border-radius: 10px;
}

</style>
</head>
<body>';
?>
