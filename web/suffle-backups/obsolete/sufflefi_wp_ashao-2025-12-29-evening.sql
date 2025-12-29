-- phpMyAdmin SQL Dump
-- version 5.2.2
-- https://www.phpmyadmin.net/
--
-- Host: localhost:3306
-- Generation Time: Dec 29, 2025 at 09:50 PM
-- Server version: 8.0.44-cll-lve
-- PHP Version: 8.4.16

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
START TRANSACTION;
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

--
-- Database: `sufflefi_wp_ashao`
--

-- --------------------------------------------------------

--
-- Table structure for table `cards`
--

DROP TABLE IF EXISTS `cards`;
CREATE TABLE `cards` (
  `id` int UNSIGNED NOT NULL,
  `dual_top_of_id` int UNSIGNED DEFAULT '0',
  `dual_below_id` int UNSIGNED DEFAULT '0',
  `setup_extras_id` int UNSIGNED DEFAULT '0',
  `heirloom` int UNSIGNED DEFAULT NULL,
  `name` varchar(255) NOT NULL,
  `en_name` varchar(255) DEFAULT NULL,
  `expansion_id` int UNSIGNED DEFAULT NULL,
  `type_id` int UNSIGNED NOT NULL,
  `prizetype_id` int UNSIGNED NOT NULL,
  `prize` int UNSIGNED NOT NULL,
  `drawcards` int DEFAULT '0',
  `buys` int DEFAULT NULL,
  `attack` tinyint(1) DEFAULT NULL,
  `defence` tinyint(1) DEFAULT NULL,
  `endure` tinyint(1) DEFAULT '0',
  `gather` tinyint(1) DEFAULT '0',
  `destroy` tinyint(1) DEFAULT '0',
  `curse` tinyint(1) DEFAULT '0',
  `hex` tinyint UNSIGNED NOT NULL DEFAULT '0',
  `boon` tinyint UNSIGNED NOT NULL DEFAULT '0',
  `night` tinyint UNSIGNED NOT NULL DEFAULT '0',
  `tuhinakerroin` int UNSIGNED DEFAULT '0',
  `dropcards` tinyint(1) DEFAULT '0',
  `actionmoney` int UNSIGNED DEFAULT '0'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `cards`
--

INSERT INTO `cards` (`id`, `dual_top_of_id`, `dual_below_id`, `setup_extras_id`, `heirloom`, `name`, `en_name`, `expansion_id`, `type_id`, `prizetype_id`, `prize`, `drawcards`, `buys`, `attack`, `defence`, `endure`, `gather`, `destroy`, `curse`, `hex`, `boon`, `night`, `tuhinakerroin`, `dropcards`, `actionmoney`) VALUES
(1, 0, 0, 0, NULL, 'Rakennusmestari', 'Engineer', 11, 2, 2, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(2, 0, 0, 0, NULL, 'Kaupunginosa', 'City Quarter', 11, 2, 2, 8, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 0),
(3, 0, 0, 0, NULL, 'Ylipäällikkö', 'Overlord', 11, 2, 2, 8, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(4, 0, 0, 0, NULL, 'Kuninkaanseppä', 'Royal Blacksmith', 11, 2, 2, 8, 5, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(6, 7, 0, 0, NULL, 'Leiri', 'Encampment', 11, 2, 1, 2, 2, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0),
(7, 0, 6, 0, NULL, 'Ryöstösaalis', 'Plunder', 11, 1, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(8, 9, 0, 0, NULL, 'Patriisi', 'Patrician', 11, 2, 1, 2, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(9, 0, 8, 0, NULL, 'Markkinapaikka', 'Emporium', 11, 2, 1, 5, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 1),
(10, 11, 0, 0, NULL, 'Uudisasukkaat', 'Settlers', 11, 2, 1, 2, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(11, 0, 10, 0, NULL, 'Vilkas kylä', 'Bustling Village', 11, 2, 1, 5, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0),
(12, 13, 0, 0, NULL, 'Katapultti', 'Catapult', 11, 2, 1, 3, 0, NULL, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1),
(13, 0, 12, 0, NULL, 'Kivet', 'Rocks', 11, 1, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(14, 0, 0, 0, NULL, 'Linnat', NULL, 11, 4, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(15, 0, 0, 0, NULL, 'Vaunukisat', 'Chariot Race', 11, 2, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(16, 0, 0, 0, NULL, 'Lumoajatar', 'Enchantress', 11, 2, 1, 3, 2, NULL, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(17, 0, 0, 0, NULL, 'Maalaismarkkinat', 'Farmers\' Market', 11, 2, 1, 3, 0, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2),
(18, 19, 0, 0, NULL, 'Gladiaattori', 'Gladiator', 11, 2, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(19, 0, 18, 0, NULL, 'Palkinto', 'Fortune', 11, 1, 2, 16, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(20, 0, 0, 0, NULL, 'Uhraus', 'Sacrifice', 11, 2, 1, 4, 1, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2, 0, 1),
(21, 0, 0, 0, NULL, 'Temppeli', 'Temple', 11, 2, 1, 4, 0, NULL, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0),
(22, 0, 0, 0, NULL, 'Huvila', 'Villa', 11, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 1),
(23, 0, 0, 0, NULL, 'Arkisto', 'Archive', 11, 2, 1, 5, 0, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(24, 0, 0, 0, NULL, 'Pääoma', 'Capital', 11, 1, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(25, 0, 0, 0, NULL, 'Onnenkalu', 'Charm', 11, 1, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(26, 0, 0, 0, NULL, 'Kruunu', 'Crown', 11, 3, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(27, 0, 0, 0, NULL, 'Foorumi', 'Forum', 11, 2, 1, 5, 3, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(28, 0, 0, 0, NULL, 'Henkijahti', 'Wild Hunt', 11, 2, 1, 5, 3, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0),
(29, 0, 0, 0, NULL, 'Legioonalainen', 'Legionary', 11, 2, 1, 5, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3),
(30, 0, 0, 0, NULL, 'Puutarhuri', 'Groundskeeper', 11, 2, 1, 5, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(31, 0, 0, 0, NULL, 'Byrokraatti', 'Bureaucrat', 1, 2, 1, 4, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(32, 0, 0, 0, NULL, 'Juhlat', 'Festival', 1, 2, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 2),
(33, 0, 0, 0, NULL, 'Kaivos', 'Mine', 1, 2, 1, 5, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(34, 0, 0, 0, NULL, 'Kansleri', 'Chancellor', 1, 2, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(35, 0, 0, 0, NULL, 'Kappeli', 'Chapel', 1, 2, 1, 2, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(36, 0, 0, 0, NULL, 'Kellari', 'Cellar', 1, 2, 1, 2, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(37, 0, 0, 0, NULL, 'Kirjasto', 'Library', 1, 2, 1, 5, 3, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(38, 0, 0, 0, NULL, 'Koronkiskuri', 'Moneylender', 1, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3),
(39, 0, 0, 0, NULL, 'Kylä', 'Village', 1, 2, 1, 3, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(40, 0, 0, 0, NULL, 'Laboratorio', 'Laboratory', 1, 2, 1, 5, 2, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(41, 0, 0, 0, NULL, 'Metsuri', 'Woodcutter', 1, 2, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(42, 0, 0, 0, NULL, 'Muutostyö', 'Remodel', 1, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(43, 0, 0, 0, NULL, 'Noita', 'Witch', 1, 2, 1, 5, 2, NULL, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0),
(44, 0, 0, 0, NULL, 'Nostoväki', 'Militia', 1, 2, 1, 4, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2),
(45, 0, 0, 0, NULL, 'Pidot', 'Feast', 1, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(46, 0, 0, 0, NULL, 'Puutarha', 'Gardens', 1, 4, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(47, 0, 0, 0, NULL, 'Raatihuone', 'Council Room', 1, 2, 1, 5, 4, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(48, 0, 0, 0, NULL, 'Seikkailija', 'Adventurer', 1, 2, 1, 6, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(49, 0, 0, 0, NULL, 'Takomo', 'Smithy', 1, 2, 1, 4, 3, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(50, 0, 0, 0, NULL, 'Tori', 'Market', 1, 2, 1, 5, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(51, 0, 0, 0, NULL, 'Työpaja', 'Workshop', 1, 2, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(52, 0, 0, 0, NULL, 'Vakooja', 'Spy', 1, 2, 1, 4, 1, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(53, 0, 0, 0, NULL, 'Vallihauta', 'Moat', 1, 2, 1, 2, 2, NULL, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(54, 0, 0, 0, NULL, 'Valtaistuinsali', 'Throne Room', 1, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(55, 0, 0, 0, NULL, 'Varas', 'Thief', 1, 2, 1, 4, 0, NULL, 1, 0, 0, 0, 1, 0, 0, 0, 0, 3, 0, 0),
(56, 0, 0, 0, NULL, 'Spurgutyö', 'Dismantle', 17, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(57, 0, 0, 0, NULL, 'Linnoitettu-kylä', 'Walled Village', 17, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(58, 0, 0, 0, NULL, 'Mustapörssi', 'Black Market', 17, 2, 1, 3, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(59, 0, 0, 0, NULL, 'Laina', 'Loan', 5, 1, 1, 3, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(60, 0, 0, 0, NULL, 'Kauppareitti', 'Trade Route', 5, 2, 1, 3, 0, NULL, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 2),
(61, 0, 0, 0, NULL, 'Vartiotorni', 'Watchtower', 5, 2, 1, 3, 1, NULL, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(62, 0, 0, 0, NULL, 'Piispa', 'Bishop', 5, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1),
(63, 0, 0, 0, NULL, 'Monumentti', 'Monument', 5, 2, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(64, 0, 0, 0, NULL, 'Louhos', 'Quarry', 5, 1, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(65, 0, 0, 0, NULL, 'Talismaani', 'Talisman', 5, 1, 1, 4, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(66, 0, 0, 0, NULL, 'Työläiskylä', 'Worker\'s Village', 5, 2, 1, 4, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(67, 0, 0, 0, NULL, 'Kaupunki', 'City', 5, 2, 1, 5, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0),
(68, 0, 0, 0, NULL, 'Hämärät varat', 'Contraband', 5, 1, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(69, 0, 0, 0, NULL, 'Tilitoimisto', 'Counting House', 5, 2, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(70, 0, 0, 0, NULL, 'Rahapaja', 'Mint', 5, 2, 1, 5, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(71, 0, 0, 0, NULL, 'Petkuttaja', 'Mountebank', 5, 2, 1, 5, 0, NULL, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 2),
(72, 0, 0, 0, NULL, 'Rahvas', 'Rabble', 5, 2, 1, 5, 3, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0),
(73, 0, 0, 0, NULL, 'Sinetti', 'Royal Seal', 5, 1, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(74, 0, 0, 0, NULL, 'Holvi', 'Vault', 5, 2, 1, 5, 2, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1),
(75, 0, 0, 0, NULL, 'Palkkio', 'Venture', 5, 1, 1, 5, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(76, 0, 0, 0, NULL, 'Roistot', 'Goons', 5, 2, 1, 6, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2),
(78, 0, 0, 0, NULL, 'Aarre', 'Hoard', 5, 1, 1, 6, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(80, 0, 0, 0, NULL, 'Suurtori', 'Grand Market', 5, 2, 1, 6, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 2),
(81, 0, 0, 0, NULL, 'Pankki', 'Bank', 5, 1, 1, 7, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(82, 0, 0, 0, NULL, 'Laajennus', 'Expand', 5, 2, 1, 7, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(83, 0, 0, 0, NULL, 'Ahjo', 'Forge', 5, 2, 1, 7, 0, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(84, 0, 0, 0, NULL, 'Kuninkaan Hovi', 'King\'s Court', 5, 2, 1, 7, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(85, 0, 0, 0, NULL, 'Kulkukauppias', 'Peddler', 5, 2, 1, 8, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1),
(86, 0, 0, 0, NULL, 'Paroni', 'Baron', 2, 2, 1, 4, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(87, 0, 0, 0, NULL, 'Silta', 'Bridge', 2, 2, 1, 4, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(88, 0, 0, 0, NULL, 'Vehkeilijä', 'Conspirator', 2, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2),
(89, 0, 0, 0, NULL, 'Kupariseppä', 'Coppersmith', 2, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(90, 0, 0, 0, NULL, 'Sisäpiha', 'Courtyard', 2, 2, 1, 2, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(91, 0, 0, 0, NULL, 'Herttua', 'Duke', 2, 4, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(92, 0, 0, 0, NULL, 'Suuri Sali', 'Great Hall', 2, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(93, 0, 0, 0, NULL, 'Haaremi', 'Harem Farm', 2, 1, 1, 6, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(94, 0, 0, 0, NULL, 'Rautapaja', 'Ironworks', 2, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1),
(95, 0, 0, 0, NULL, 'Naamiaiset', 'Masquerade', 2, 2, 1, 3, 2, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0),
(96, 0, 0, 0, NULL, 'Kaivoskylä', 'Mining Village', 2, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 2),
(97, 0, 0, 0, NULL, 'Kätyri', 'Minion', 2, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2),
(98, 0, 0, 0, NULL, 'Aateliset', 'Nobles', 2, 2, 1, 6, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(99, 0, 0, 0, NULL, 'Apuri', 'Pawn', 2, 2, 1, 2, 1, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1),
(100, 0, 0, 0, NULL, 'Tihulainen', 'Saboteur', 2, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0),
(101, 0, 0, 0, NULL, 'Tiedustelija', 'Scout', 2, 2, 1, 4, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(102, 0, 0, 0, NULL, 'Salainen Kammio', 'Secret Chamber', 2, 2, 1, 2, 0, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(103, 0, 0, 0, NULL, 'Hökkelikylä', 'Shanty Town', 2, 2, 1, 3, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(110, 0, 0, 0, NULL, 'Käskynhaltija', 'Steward', 2, 2, 1, 3, 2, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2),
(111, 0, 0, 0, NULL, 'Huijari', 'Swindler', 2, 2, 1, 3, 0, 0, 1, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2),
(112, 0, 0, 0, NULL, 'Kiduttaja', 'Torturer', 2, 2, 1, 5, 3, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0),
(113, 0, 0, 0, NULL, 'Kauppa Asema', 'Trading Post', 2, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(114, 0, 0, 0, NULL, 'Kymmenykset', 'Tribute', 2, 2, 1, 5, 2, 2, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2),
(115, 0, 0, 0, NULL, 'Parannustyö', 'Upgrade', 2, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0),
(116, 0, 0, 0, NULL, 'Toivomuskaivo', 'Wishing Well', 2, 2, 1, 3, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(117, 0, 0, 0, NULL, 'Alley', 'Alley', 18, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(118, 0, 0, 0, NULL, 'Aristocrat', 'Aristocrat', 18, 2, 1, 3, 3, 3, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 3),
(119, 0, 0, 0, NULL, 'Artist', 'Artist', 18, 2, 2, 8, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(120, 0, 0, 0, NULL, 'Change', 'Change', 18, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3),
(121, 0, 0, 0, NULL, 'Craftsman', 'Craftsman', 18, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(122, 0, 0, 0, NULL, 'Daimyo', 'Daimyo', 18, 2, 2, 6, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(123, 0, 0, 0, NULL, 'Fishmonger', 'Fishmonger', 18, 2, 1, 2, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(124, 0, 0, 0, NULL, 'Mun Kulta', 'Gold Mine', 18, 2, 1, 5, 1, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(125, 0, 0, 0, NULL, 'Imperial Envoy', 'Imperial Envoy', 18, 2, 1, 5, 5, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(126, 0, 0, 0, NULL, 'Kitsune', 'Kitsune', 18, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 1, 0, 2),
(127, 0, 0, 0, NULL, 'Litter', 'Litter', 18, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(128, 0, 0, 0, NULL, 'Mountain Shrine', 'Mountain Shrine', 18, 2, 2, 5, 2, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2),
(129, 0, 0, 0, NULL, 'Ninja', 'Ninja', 18, 2, 1, 4, 1, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0),
(130, 0, 0, 0, NULL, 'Poet', 'Poet', 18, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(131, 0, 0, 0, NULL, 'Rice', 'Rice', 18, 1, 1, 7, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(132, 0, 0, 0, NULL, 'Riisinsärkijä', 'Rice Broker', 18, 2, 1, 5, 3, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0),
(133, 0, 0, 0, NULL, 'River Shrine', 'River Shrine', 18, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(134, 0, 0, 0, NULL, 'Riverboat', 'Riverboat', 18, 2, 1, 3, 0, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(135, 0, 0, 0, NULL, 'Ronin', 'Ronin', 18, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(136, 0, 0, 0, NULL, 'Pääkäyttäjän Pommisuoja', 'Root Cellar', 18, 2, 1, 3, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(137, 0, 0, 0, NULL, 'Turtola', 'Rustic Village', 18, 2, 1, 4, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(138, 0, 0, 0, NULL, 'Samurai', 'Samurai', 18, 2, 1, 6, 0, 0, 1, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1),
(139, 0, 0, 0, NULL, 'Snake Witch', 'Snake Witch', 18, 2, 1, 2, 1, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0),
(140, 0, 0, 0, NULL, 'Tanuki', 'Tanuki', 18, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(141, 0, 0, 0, NULL, 'Lettutalo', 'Tea House', 18, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2),
(142, 0, 0, 0, NULL, 'Alttari', 'Altar', 8, 2, 1, 6, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(143, 0, 0, 0, NULL, 'Asevarasto', 'Armory', 8, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(144, 0, 0, 0, NULL, 'Hurja Joukko', 'Band of Misfits', 8, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(145, 0, 0, 0, NULL, 'Rosvoleiri', 'Bandit Camp', 8, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(146, 0, 0, 0, NULL, 'Kerjäläinen', 'Beggar', 8, 2, 1, 2, 0, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3),
(147, 0, 0, 0, NULL, 'Katakombit', 'Catacombs', 8, 2, 1, 5, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(148, 0, 0, 0, NULL, 'Jaarli', 'Count', 8, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 3),
(149, 0, 0, 0, NULL, 'Väärennös', 'Counterfeit', 8, 1, 1, 5, 0, 1, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1),
(150, 0, 0, 0, NULL, 'Okkultisti', 'Cultist', 8, 2, 1, 5, 2, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(152, 0, 0, 0, NULL, 'Ruumisvaunu', 'Death Cart', 8, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 5),
(153, 0, 0, 0, NULL, 'Sarka', 'Feodum', 8, 4, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(154, 0, 0, 0, NULL, 'Keräilijä', 'Forager', 8, 2, 1, 3, 0, 1, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1),
(155, 0, 0, 0, NULL, 'Linnoitus', 'Fortress', 8, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(156, 0, 0, 0, NULL, 'Haudanryöstäjä', 'Graverobber', 8, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(157, 0, 0, 0, NULL, 'Erakko', 'Hermit', 8, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(158, 0, 0, 0, NULL, 'Metsästysmaat', 'Hunting Grounds', 8, 2, 1, 6, 4, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(159, 0, 0, 0, NULL, 'Reppuryssä', 'Ironmonger', 8, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1),
(160, 0, 0, 0, NULL, 'Romukauppias', 'Junk Dealer', 8, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 1),
(161, 0, 0, 0, NULL, 'Vandaali', 'Marauder', 8, 2, 1, 4, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(162, 0, 0, 0, NULL, 'Kauppatori', 'Market Square', 8, 2, 1, 3, 1, 1, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(163, 0, 0, 0, NULL, 'Mystikko', 'Mystic', 8, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2),
(164, 0, 0, 0, NULL, 'Ryöstöretki', 'Pillage', 8, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0),
(165, 0, 0, 0, NULL, 'Köyhäintalo', 'Poor House', 8, 2, 1, 1, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4),
(166, 0, 0, 0, NULL, 'Virkanimitys', 'Procession', 8, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0),
(167, 0, 0, 0, NULL, 'Rotta', 'Rats', 8, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 3, 0, 0),
(168, 0, 0, 0, NULL, 'Ritarit', 'Knights', 8, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0),
(169, 0, 0, 0, NULL, 'Kunnostus', 'Rebuild', 8, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0),
(170, 0, 0, 0, NULL, 'Ryöstäjä', 'Rogue', 8, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2),
(171, 0, 0, 0, NULL, 'Tietäjä', 'Sage', 8, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(172, 0, 0, 0, NULL, 'Haaskalintu', 'Scavenger', 8, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(173, 0, 0, 0, NULL, 'Aseenkantaja', 'Squire', 8, 2, 1, 2, 0, 2, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(174, 0, 0, 0, NULL, 'Kellarivarasto', 'Storeroom', 8, 2, 1, 3, 3, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(175, 0, 0, 0, NULL, 'Katulapsi', 'Urchin', 8, 2, 1, 3, 1, 0, 1, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 1, 0),
(176, 0, 0, 0, NULL, 'Kulkuri', 'Vagrant', 8, 2, 1, 2, 1, 0, NULL, NULL, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0),
(177, 0, 0, 0, NULL, 'Trubaduuri', 'Wandering Minstrel', 8, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(178, 0, 0, 0, NULL, 'Rajakylä', 'Border Village', 7, 2, 1, 6, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(179, 0, 0, 0, NULL, 'Piilo', 'Cache', 7, 1, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3),
(180, 0, 0, 0, NULL, 'Kartanpiirtäjä', 'Cartographer', 7, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(181, 0, 0, 0, NULL, 'Tienristeys', 'Crossroads', 7, 2, 1, 2, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(182, 0, 0, 0, NULL, 'Grynderi', 'Develop', 7, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(183, 0, 0, 0, NULL, 'Herttuatar', 'Duchess', 7, 2, 1, 2, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(184, 0, 0, 0, NULL, 'Lähetystö', 'Embassy', 7, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(185, 0, 0, 0, NULL, 'Viljelysmaat', 'Farmland', 7, 4, 1, 6, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(186, 0, 0, 0, NULL, 'Katinkulta', 'Fools Gold', 7, 1, 1, 2, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2),
(187, 0, 0, 0, NULL, 'Tinkijä', 'Haggler', 7, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(188, 0, 0, 0, NULL, 'Maantie', 'Highway', 7, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(190, 0, 0, 0, NULL, 'Epäonnen saalis', 'Ill Gotten Gains', 7, 1, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1),
(191, 0, 0, 0, NULL, 'Majatalo', 'Inn', 7, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(192, 0, 0, 0, NULL, 'Monitaituri', 'Jack of All Trades', 7, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(193, 0, 0, 0, NULL, 'Mandariini', 'Mandarin', 7, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3),
(194, 0, 0, 0, NULL, 'Markiisi', 'Margrave', 7, 2, 1, 5, 3, 1, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0),
(195, 0, 0, 0, NULL, 'Jalo maantierosvo', 'Noble Brigand', 7, 2, 1, 4, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(196, 0, 0, 0, NULL, 'Paimentolaisleiri', 'Nomad Camp', 7, 2, 1, 4, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(197, 0, 0, 0, NULL, 'Keidas', 'Oasis', 7, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1),
(198, 0, 0, 0, NULL, 'Oraakkeli', 'Oracle', 7, 2, 1, 3, 2, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0),
(199, 0, 0, 0, NULL, 'Juoni', 'Scheme', 7, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(200, 0, 0, 0, NULL, 'Silkkitie', 'Silk Road', 7, 4, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(201, 0, 0, 0, NULL, 'Maustekauppias', 'Spice Merchant', 7, 2, 1, 4, 2, 1, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 2),
(202, 0, 0, 0, NULL, 'Hevostallit', 'Stables', 7, 2, 1, 5, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(203, 0, 0, 0, NULL, 'Kaupustelija', 'Trader', 7, 2, 1, 4, 0, 0, NULL, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(204, 0, 0, 0, NULL, 'Tunneli', 'Tunnel', 7, 4, 1, 3, 0, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(205, 0, 0, 0, NULL, 'Neuvonantaja', 'Advisor', 9, 2, 1, 4, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(206, 0, 0, 0, NULL, 'Leipuri', 'Baker', 9, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(207, 0, 0, 0, NULL, 'Teurastaja', 'Butcher', 9, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2),
(208, 0, 0, 0, NULL, 'Kynttilänvalaja', 'Candlestick Maker', 9, 2, 1, 2, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1),
(209, 0, 0, 0, NULL, 'Carnival', 'Carnival', 9, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(210, 0, 0, 0, NULL, 'Farrier', 'Farrier', 9, 2, 1, 2, 1, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(211, 0, 0, 0, NULL, 'Footpad', 'Footpad', 9, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2),
(212, 0, 0, 0, NULL, 'Airut', 'Herald', 9, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(213, 0, 0, 0, NULL, 'Infirmary', 'Infirmary', 9, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0),
(214, 0, 0, 0, NULL, 'Reissumies', 'Journeyman', 9, 2, 1, 5, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(215, 0, 0, 0, NULL, 'Joust', 'Joust', 9, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(216, 0, 0, 0, NULL, 'Kilta', 'Merchant Guild', 9, 2, 1, 5, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(217, 0, 0, 0, NULL, 'Suihkulähde', 'Plaza', 9, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(218, 0, 0, 0, NULL, 'Manaaja', 'Soothsayer', 9, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0),
(219, 0, 0, 0, NULL, 'Kivenhakkaaja', 'Stonemason', 9, 2, 1, 2, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(220, 0, 0, 0, NULL, 'Markkinat', 'Fairgrounds', 6, 4, 1, 6, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(221, 0, 0, 0, NULL, 'Farmhands', 'Farmhands', 6, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(222, 0, 0, 0, NULL, 'Ferryman', 'Ferryman', 6, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(223, 0, 0, 0, NULL, 'Kyläraitti', 'Hamlet', 6, 2, 1, 2, 1, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(224, 0, 0, 0, NULL, 'Runsaudensarvi', 'Horn of Plenty', 6, 1, 1, 5, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(225, 0, 0, 0, NULL, 'Jahti', 'Hunting Party', 6, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(226, 0, 0, 0, NULL, 'Narri', 'Jester', 6, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 1, 2),
(227, 0, 0, 0, NULL, 'Villieläintarha', 'Menagerie', 6, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(228, 0, 0, 0, NULL, 'Jälleenrakennus', 'Remake', 6, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(229, 0, 0, 0, NULL, 'Shop', 'Shop', 6, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(230, 0, 0, 0, NULL, 'Nuori noita', 'Young Witch', 6, 2, 1, 4, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0),
(231, 232, 0, 0, NULL, 'Sauna', 'Sauna', 17, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 3, 0, 0),
(232, 231, 0, 0, NULL, 'Avanto', 'Avanto', 17, 2, 1, 5, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0),
(233, 0, 0, 1, NULL, 'Bard', 'Bard', 12, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2),
(234, 0, 0, 1, NULL, 'Kärväskylä', 'Blessed Village', 12, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 1, 0, 4, 0, 0),
(235, 0, 0, 3, 1, 'Cemetery', 'Cemetery', 12, 4, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(236, 0, 0, 0, NULL, 'Changeling', 'Changeling', 12, 4, 1, 3, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
(237, 0, 0, 0, NULL, 'Cobbler', 'Cobbler', 12, 4, 1, 5, 1, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0),
(238, 0, 0, 0, NULL, 'Conclave', 'Conclave', 12, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2),
(239, 0, 0, 0, NULL, 'Crypt', 'Crypt', 12, 4, 1, 5, 1, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0),
(240, 0, 0, 2, NULL, 'Muonio', 'Cursed Village', 12, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 1, 0, 0, 4, 0, 0),
(241, 0, 0, 0, NULL, 'Den of Sin', 'Den of Sin', 12, 4, 1, 5, 2, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0),
(242, 0, 0, 4, NULL, 'Devil\'s Workshop', 'Devil\'s Workshop', 12, 4, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0),
(243, 0, 0, 7, NULL, 'Druid', 'Druid', 12, 2, 1, 2, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0),
(244, 0, 0, 11, NULL, 'Exorcist', 'Exorcist', 12, 4, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
(245, 0, 0, 0, NULL, 'Faithful Hound', 'Faithful Hound', 12, 2, 1, 2, 2, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(246, 0, 0, 12, NULL, 'Fool', 'Fool', 12, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0),
(247, 0, 0, 0, NULL, 'Ghost Town', 'Ghost Town', 12, 4, 1, 3, 1, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 1, 2, 0, 0),
(248, 0, 0, 0, NULL, 'Guardian', 'Guardian', 12, 4, 1, 2, 0, 0, NULL, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1),
(249, 0, 0, 1, NULL, 'Idol', 'Idol', 12, 1, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 1, 0, 0, 0, 2),
(250, 0, 0, 9, NULL, 'Leprechaun', 'Leprechaun', 12, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0),
(251, 0, 0, 0, NULL, 'Monastery', 'Monastery', 12, 4, 1, 2, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
(252, 0, 0, 6, NULL, 'Necromancer', 'Necromancer', 12, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(253, 0, 0, 0, NULL, 'Night Watchman', 'Night Watchman', 12, 4, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0),
(254, 0, 0, 13, NULL, 'Pixie', 'Pixie', 12, 2, 1, 2, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 1, 0, 2, 0, 0),
(255, 0, 0, 14, NULL, 'Pooka', 'Pooka', 12, 2, 1, 5, 4, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(256, 0, 0, 0, NULL, 'Raider', 'Raider', 12, 4, 1, 6, 0, 0, 1, NULL, 1, 0, 0, 0, 0, 0, 1, 0, 1, 3),
(257, 0, 0, 1, NULL, 'Sacred Grove', 'Sacred Grove', 12, 2, 1, 5, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 1, 0, 0, 0, 3),
(258, 0, 0, 15, NULL, 'Secret Cave', 'Secret Cave', 12, 2, 1, 3, 1, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 2, 0, 3),
(259, 0, 0, 16, NULL, 'Shepherd', 'Shepherd', 12, 2, 1, 4, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(260, 0, 0, 2, NULL, 'Skulk', 'Skulk', 12, 2, 1, 4, 0, 1, 1, NULL, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0),
(261, 0, 0, 2, NULL, 'Tormentor', 'Tormentor', 12, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 0, 1, 0, 0, 1, 0, 2),
(262, 0, 0, 17, NULL, 'Tracker', 'Tracker', 12, 2, 1, 2, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1),
(263, 0, 0, 0, NULL, 'Tragic Hero', 'Tragic Hero', 12, 2, 1, 5, 3, 1, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(264, 0, 0, 8, NULL, 'Vampire', 'Vampire', 12, 4, 1, 5, 0, 0, 1, NULL, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0),
(265, 0, 0, 2, NULL, 'Werewolf', 'Werewolf', 12, 2, 1, 5, 3, 0, 1, NULL, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0),
(266, 0, 0, 0, NULL, 'Amuletti', 'Amulet', 10, 2, 1, 3, 0, 0, NULL, NULL, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1),
(267, 0, 0, 0, NULL, 'Käsityöläinen', 'Artificer', 10, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(268, 0, 0, 24, NULL, 'Siltapeikko', 'Bridge Troll', 10, 2, 1, 5, 0, 1, 1, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(269, 0, 0, 0, NULL, 'Karavaanivartija', 'Caravan Guard', 10, 2, 1, 3, 1, 0, NULL, 1, 1, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(270, 0, 0, 20, NULL, 'Kuninkaallinen Raha', 'Coin of the Realm', 10, 1, 1, 2, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1),
(271, 0, 0, 20, NULL, 'Kaukomaat', 'Distant Lands', 10, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(272, 0, 0, 0, NULL, 'Tyrmä', 'Dungeon', 10, 2, 1, 3, 2, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(273, 0, 0, 20, NULL, 'Kopio', 'Duplicate', 10, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(274, 0, 0, 0, NULL, 'Varusteet', 'Gear', 10, 2, 1, 3, 2, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(275, 0, 0, 21, NULL, 'Jättiläinen', 'Giant', 10, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 0, 1, 3),
(276, 0, 0, 20, NULL, 'Opas', 'Guide', 10, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(277, 0, 0, 0, NULL, 'Kummitusmetsä', 'Haunted Woods', 10, 2, 1, 5, 3, 0, 1, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(278, 0, 0, 0, NULL, 'Palkollinen', 'Hireling', 10, 2, 1, 6, 1, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(279, 0, 0, 0, NULL, 'Kadonnut Kaupunki', 'Lost City', 10, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(280, 0, 0, 0, NULL, 'Harakka', 'Magpie', 10, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(281, 0, 0, 0, NULL, 'Kuriiri', 'Messenger', 10, 2, 1, 4, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(282, 0, 0, 20, NULL, 'Saituri', 'Miser', 10, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(283, 0, 0, 22, NULL, 'Kisälli', 'Page', 10, 2, 1, 2, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0),
(284, 0, 0, 23, NULL, 'Maalainen', 'Peasant', 10, 2, 1, 2, 0, 1, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1),
(285, 0, 0, 0, NULL, 'Satamakaupunki', 'Port', 10, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(286, 0, 0, 21, NULL, 'Samooja', 'Ranger', 10, 2, 1, 4, 3, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(287, 0, 0, 20, NULL, 'Rotanmetsästäjä', 'Ratcatcher', 10, 2, 1, 2, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0),
(288, 0, 0, 0, NULL, 'Hävitys', 'Raze', 10, 2, 1, 2, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 3, 0, 0),
(289, 0, 0, 24, NULL, 'Reliikki', 'Relic', 10, 1, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(290, 0, 0, 20, NULL, 'Kuninkaalliset Vaunut', 'Royal Carriage', 10, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(291, 0, 0, 0, NULL, 'Tarinankertoja', 'Storyteller', 10, 2, 1, 5, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1),
(292, 0, 0, 0, NULL, 'Suonoita', 'Swamp Hag', 10, 2, 1, 5, 0, 0, 1, NULL, 1, 0, 0, 1, 0, 0, 0, 0, 0, 3),
(293, 0, 0, 20, NULL, 'Muodonmuutos', 'Transmogrify', 10, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 3, 0, 0),
(294, 0, 0, 0, NULL, 'Aarrekasa', 'Treasure Trove', 10, 1, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(295, 0, 0, 20, NULL, 'Viinikauppias', 'Wine Merchant', 10, 2, 1, 5, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4),
(297, 0, 0, 0, NULL, 'Animal Fair', 'Animal Fair', 14, 2, 2, 7, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4),
(298, 0, 0, 0, NULL, 'Barge', 'Barge', 14, 2, 1, 5, 3, 1, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(299, 0, 0, 0, NULL, 'Black Cat', 'Black Cat', 14, 2, 1, 2, 2, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0),
(300, 0, 0, 25, NULL, 'Bounty Hunter', 'Bounty Hunter', 14, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3),
(301, 0, 0, 25, NULL, 'Camel Train', 'Camel Train', 14, 2, 1, 3, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(302, 0, 0, 25, NULL, 'Cardinal', 'Cardinal', 14, 2, 1, 4, 0, 0, 1, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(303, 0, 0, 26, NULL, 'Cavalry', 'Cavalry', 14, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(304, 0, 0, 25, NULL, 'Coven', 'Coven', 14, 2, 1, 5, 0, 0, 1, NULL, 0, 0, 0, 1, 0, 0, 0, 1, 0, 2),
(305, 0, 0, 0, NULL, 'Destrier', 'Destrier', 14, 2, 2, 6, 2, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 1),
(306, 0, 0, 25, NULL, 'Jemmata', 'Displace', 14, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(307, 0, 0, 0, NULL, 'Falconer', 'Falconer', 14, 2, 1, 5, 0, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(308, 0, 0, 0, NULL, 'Fisherman', 'Fisherman', 14, 2, 2, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1),
(309, 0, 0, 25, NULL, 'Gatekeeper', 'Gatekeeper', 14, 2, 1, 5, 0, 0, 1, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3),
(310, 0, 0, 0, NULL, 'Goatherd', 'Goatherd', 14, 2, 1, 3, 1, 0, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 0),
(311, 0, 0, 26, NULL, 'Groom', 'Groom', 14, 2, 1, 4, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(312, 0, 0, 26, NULL, 'Hostelry', 'Hostelry', 14, 2, 1, 4, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0),
(313, 0, 0, 0, NULL, 'Välitupa', 'Hunting Lodge', 14, 2, 1, 5, 1, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(314, 0, 0, 0, NULL, 'Kiln', 'Kiln', 14, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2),
(315, 0, 0, 26, NULL, 'Livery', 'Livery', 14, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 3),
(316, 0, 0, 0, NULL, 'Mastermind', 'Mastermind', 14, 2, 1, 5, 0, 0, NULL, NULL, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(317, 0, 0, 26, NULL, 'Paddock', 'Paddock', 14, 2, 1, 5, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2),
(318, 0, 0, 25, NULL, 'Sanctuary', 'Sanctuary', 14, 2, 1, 5, 1, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0),
(319, 0, 0, 25, NULL, 'Scrap', 'Scrap', 14, 2, 1, 3, 1, 1, NULL, NULL, 0, 0, 1, 0, 0, 0, 0, 2, 0, 1),
(320, 0, 0, 26, NULL, 'Sheepdog', 'Sheepdog', 14, 2, 1, 3, 2, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
(321, 0, 0, 0, NULL, 'Sleigh', 'Sleigh', 14, 2, 1, 2, 0, 0, NULL, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0),
(322, 0, 0, 0, NULL, 'Hetta', 'Snowy Village', 14, 2, 1, 3, 1, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(323, 0, 0, 25, NULL, 'Stockpile', 'Stockpile', 14, 1, 1, 3, 0, 1, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3),
(324, 0, 0, 26, NULL, 'Taravoita', 'Supplies', 14, 1, 1, 2, 0, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1),
(325, 0, 0, 0, NULL, 'Village Green', 'Village Green', 14, 2, 1, 4, 1, 0, NULL, 1, 1, 0, 0, 0, 0, 0, 0, 3, 0, 0),
(326, 0, 0, 0, NULL, 'Wayfarer', 'Wayfarer', 14, 2, 2, 6, 3, 0, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

-- --------------------------------------------------------

--
-- Table structure for table `cardtype`
--

DROP TABLE IF EXISTS `cardtype`;
CREATE TABLE `cardtype` (
  `id` int UNSIGNED NOT NULL,
  `name` varchar(255) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `cardtype`
--

INSERT INTO `cardtype` (`id`, `name`) VALUES
(1, 'raha'),
(2, 'toiminto'),
(3, 'raha/toiminto'),
(4, 'sekalainen');

-- --------------------------------------------------------

--
-- Table structure for table `events`
--

DROP TABLE IF EXISTS `events`;
CREATE TABLE `events` (
  `id` int UNSIGNED NOT NULL,
  `event_type_id` int UNSIGNED NOT NULL,
  `expansion_id` int UNSIGNED NOT NULL,
  `setup_id` int UNSIGNED DEFAULT NULL,
  `name` varchar(255) NOT NULL,
  `prize` int UNSIGNED DEFAULT '0',
  `debt` tinyint(1) DEFAULT '0',
  `curses` tinyint(1) DEFAULT '0'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `events`
--

INSERT INTO `events` (`id`, `event_type_id`, `expansion_id`, `setup_id`, `name`, `prize`, `debt`, `curses`) VALUES
(1, 1, 11, NULL, 'Valloitus', 6, 0, 0),
(2, 1, 11, NULL, 'Juomingit', 3, 0, 0),
(3, 1, 11, NULL, 'Nöyryytys', 14, 0, 0),
(4, 1, 11, NULL, 'Rituaali', 4, 0, 1),
(5, 1, 11, 27, 'Verotus', 2, 0, 0),
(6, 1, 11, NULL, 'Lahjoitus', 8, 1, 0),
(7, 1, 11, NULL, 'Häät', 7, 1, 0),
(8, 1, 11, NULL, 'Voitonjuhla', 5, 1, 0),
(9, 1, 11, NULL, 'Pakkoliitos', 8, 1, 0),
(10, 1, 11, NULL, 'Onnenkantamoinen', 5, 0, 0),
(11, 1, 11, NULL, 'Kaivanto', 2, 0, 0),
(12, 1, 11, NULL, 'Ylennys', 0, 0, 0),
(13, 1, 11, NULL, 'Myrkytetty Maa', 4, 0, 0),
(14, 1, 18, NULL, 'Continue', 8, 1, 0),
(15, 1, 18, NULL, 'Amass', 2, 0, 0),
(16, 1, 18, NULL, 'Ascetism', 2, 0, 0),
(17, 1, 18, 33, 'Credit', 2, 0, 0),
(18, 1, 18, NULL, 'Foresight', 2, 0, 0),
(19, 1, 18, NULL, 'Kintsugi', 3, 0, 0),
(20, 1, 18, NULL, 'Practice', 3, 0, 0),
(21, 1, 18, NULL, 'Sea Trade', 4, 0, 0),
(22, 1, 18, NULL, 'Receive Tribute', 5, 0, 0),
(23, 1, 18, NULL, 'Gather', 7, 0, 0),
(24, 1, 14, NULL, 'Delay', 0, 0, 0),
(25, 1, 14, NULL, 'Desperation', 0, 0, 1),
(26, 1, 14, NULL, 'Gamble', 2, 0, 0),
(27, 1, 14, NULL, 'Pursue', 2, 0, 0),
(28, 1, 14, NULL, 'Toil', 2, 0, 0),
(29, 1, 14, 26, 'Ride', 2, 0, 0),
(30, 1, 14, NULL, 'Enchance', 3, 0, 0),
(31, 1, 14, NULL, 'March', 3, 0, 0),
(32, 1, 14, 25, 'Transport', 3, 0, 0),
(33, 1, 14, 25, 'Banish', 4, 0, 0),
(34, 1, 14, 26, 'Bargain', 4, 0, 0),
(35, 1, 14, 25, 'Invest', 4, 0, 0),
(36, 1, 14, NULL, 'Seize The Day', 4, 0, 0),
(37, 1, 14, NULL, 'Commerce', 5, 0, 0),
(38, 1, 14, 26, 'Demand', 5, 0, 0),
(39, 1, 14, 26, 'Stampede', 5, 0, 0),
(40, 1, 14, NULL, 'Reap', 7, 0, 0),
(41, 1, 14, 25, 'Enclave', 8, 0, 0),
(42, 1, 14, NULL, 'Alliance', 10, 0, 0),
(43, 1, 14, NULL, 'Populate', 10, 0, 0),
(44, 1, 10, 21, 'Pyhiinvaellus', 4, 0, 0),
(45, 1, 10, NULL, 'Almut', 0, 0, 0),
(46, 1, 10, 24, 'Tanssiaiset', 5, 0, 0),
(47, 1, 10, NULL, 'Kokko', 3, 0, 0),
(48, 1, 10, 24, 'Vippi', 0, 0, 0),
(49, 1, 10, NULL, 'Tutkimusmatka', 3, 0, 0),
(50, 1, 10, 34, 'Lautta', 3, 0, 0),
(51, 1, 10, 35, 'Perintö', 7, 0, 0),
(52, 1, 10, 39, 'Merireitti', 5, 0, 0),
(53, 1, 10, 36, 'Unohdetut Kyvyt', 6, 0, 0),
(54, 1, 10, 37, 'Reitin Etsintä', 8, 0, 0),
(55, 1, 10, 38, 'Suunnitelma', 3, 0, 0),
(56, 1, 10, NULL, 'Tehtävä', 4, 0, 0),
(57, 1, 10, NULL, 'Kiertelevät Markkinat', 2, 0, 0),
(58, 1, 10, 40, 'Koulutus', 6, 0, 0),
(59, 1, 10, NULL, 'Vaihtokauppa', 5, 0, 0),
(60, 1, 10, NULL, 'Tiedustelupartio', 2, 0, 0),
(61, 1, 10, NULL, 'Säästöt', 1, 0, 0),
(62, 1, 10, 41, 'Ryöstöretki', 5, 0, 0),
(63, 1, 10, NULL, 'Seikkailu', 0, 0, 0);

-- --------------------------------------------------------

--
-- Table structure for table `event_type`
--

DROP TABLE IF EXISTS `event_type`;
CREATE TABLE `event_type` (
  `id` int UNSIGNED NOT NULL,
  `name` varchar(255) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `event_type`
--

INSERT INTO `event_type` (`id`, `name`) VALUES
(1, 'Tapahtuma'),
(2, 'Maamerkki');

-- --------------------------------------------------------

--
-- Table structure for table `expansion`
--

DROP TABLE IF EXISTS `expansion`;
CREATE TABLE `expansion` (
  `id` int UNSIGNED NOT NULL,
  `name` varchar(255) NOT NULL,
  `ordernumber` int UNSIGNED NOT NULL DEFAULT '99',
  `disabled` tinyint(1) NOT NULL DEFAULT '0'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `expansion`
--

INSERT INTO `expansion` (`id`, `name`, `ordernumber`, `disabled`) VALUES
(1, 'Valtakunta', 0, 0),
(2, 'Hovin Juonet', 1, 0),
(3, 'Kaukaiset rannat', 2, 0),
(4, 'Alkemia', 3, 0),
(5, 'Nousukausi', 4, 0),
(6, 'Elonkorjuu', 5, 0),
(7, 'Rajaseudut', 6, 0),
(8, 'Katovuodet', 7, 0),
(9, 'Killat', 8, 0),
(10, 'Seikkailut', 9, 0),
(11, 'Keisarikunta', 10, 0),
(12, 'Nocturne', 11, 0),
(13, 'Renaissance', 12, 0),
(14, 'Menagerie', 13, 0),
(15, 'Allies', 14, 0),
(16, 'Plunder', 15, 0),
(17, 'Extras', 99, 0),
(18, 'Riisivä Arska (Rising Sun)', 16, 0);

-- --------------------------------------------------------

--
-- Table structure for table `landmarks`
--

DROP TABLE IF EXISTS `landmarks`;
CREATE TABLE `landmarks` (
  `id` int UNSIGNED NOT NULL,
  `expansion_id` int UNSIGNED NOT NULL,
  `name` varchar(255) NOT NULL,
  `description` varchar(1024) DEFAULT NULL,
  `setup_id` int UNSIGNED DEFAULT '0'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `landmarks`
--

INSERT INTO `landmarks` (`id`, `expansion_id`, `name`, `description`, `setup_id`) VALUES
(1, 11, 'Muuri', 'Pistelasku: -1 pts jokaisesta yli 15 kortista jotka sinulla on pakassasi.', 0),
(2, 11, 'Viinitila', 'Pistelasku: +4 pts Jokaista eri nimistä toimintakorttia kohti, joita sinulla on vähintään 3 kappaletta', 0),
(3, 11, 'Palatsi', 'Pistelasku: +3 pts jokaista kulta-hopea-kupari -settiä kohti jotka sinulla on', 0),
(4, 11, 'Toivomuslähde', 'Pistelasku: +15 pts jos sinulla on vähintään 10 kuparia', 0),
(5, 11, 'Sudenpesä', 'Pistelasku: -3 pts jokaista korttia kohden joita sinulla on 1 kappale', 0),
(6, 11, 'Rosvojen Linnake', 'Pistelasku: -2 pts jokaista pakassasi olevaa kultaa ja hopeaa kohden', 0),
(7, 11, 'Linnake', 'Pistelasku: +5 pts jokaista eri nimistä Rahakorttia kohden, joita sinulla on eniten tai yhtä monta kuin seuraavaksi eniten omaavalla on', 0),
(8, 11, 'Museo', 'Pistelasku: +2 pts jokaista eri nimistä korttiasi kohden', 0),
(9, 11, 'Torni', 'Pistelasku: +1 pts per kortti(si), joka ei ole Pistekortti ja jonka varastopino on tyhjentynyt', 0),
(10, 11, 'Riemukaari', 'Pistelasku: +3 pts jokaista toiseksi yleisintä toimintakorttiasi kohden. Jos tasamäärä, valitse kumpi vain', 0),
(11, 11, 'Obeliski', 'Pistelasku: +2 pts jokaista korttia kohden, joka sinulla (alkutoimien aikana, sattumanvaraisesti) valitusta toimintakorttien varastopinosta', 28),
(12, 11, 'Taistelutanner', 'Kun otat Pistekortin, ota maamerkiltä 2 pts, (Joita alkutoimien aikana laitettu 6 per pelaaja)', 29),
(13, 11, 'Pylväskäytävä', 'Kun otat Toimintakortin ja sinulla on sellainen myös pelissä, ota maamerkiltä 2 pts. (Joita alkutoimien aikana laitettu 6 per pelaaja)', 29),
(14, 11, 'Häväisty Pyhättö', 'Kun otat toimintakortin, siirrä sen kasan päältä 1 pts tälle maamerkille. Kun otat kirouksen, ota pisteet tältä maamerkiltä.', 31),
(15, 11, 'Akvedukti', 'Kun ostat Rahakortin, siirrä sen varastopinon päältä 1 pts tämän kortin päälle. Kun otat Pistekortin, ota pisteet tältä maamerkiltä.', 32),
(16, 11, 'Basilika', 'Kun otat kortin ja sinulla jää 2 rahaa tai enemmän, ota 2 pts tältä maamerkiltä.', 29),
(17, 11, 'Labyrintti', 'Kun omalla vuorollasin otat vuorosi toisen kortin, ota 2 pts tältä maamerkiltä.', 29),
(18, 11, 'Vuoristoreitti', 'Kun ensimmäisen pelaajan joka ottaa läänin vuoro loppuu, käydään huutokauppa. Aktiiviseen pelaajaan loppuen, (muut???) pelaajat tarjoavat enintään 40-velkaa. Korkeimman tarjouksen tehnyt saa 8 pts ja tarjoamansa velan.', 0),
(19, 11, 'Hauta', 'Kun tuhoat kortin, +1 pts.', 0),
(20, 11, 'Kylpylä', 'Ota 2 pistettä tältä maamerkiltä, jos lopetat vuorosi ottamatta korttiakaan.', 29),
(21, 11, 'Areena', 'Ostovuorosi alussa voit poistaa toimintakortin kädestäsi. Jos teet niin, ota 2 pts tältä maamerkiltä.', 29);

-- --------------------------------------------------------

--
-- Table structure for table `parsed_cards`
--

DROP TABLE IF EXISTS `parsed_cards`;
CREATE TABLE `parsed_cards` (
  `id` int NOT NULL,
  `added` tinyint(1) DEFAULT '0',
  `skip` tinyint(1) DEFAULT '0',
  `en_added` tinyint(1) DEFAULT '0',
  `no_finnish` tinyint(1) DEFAULT '0',
  `name` varchar(255) DEFAULT NULL,
  `expansion` varchar(255) DEFAULT NULL,
  `exp_id` int UNSIGNED DEFAULT NULL,
  `type` varchar(255) DEFAULT NULL,
  `cost` varchar(255) DEFAULT NULL,
  `text` varchar(1023) DEFAULT NULL,
  `act_villager` varchar(255) DEFAULT NULL,
  `draws` varchar(255) DEFAULT NULL,
  `buys` varchar(255) DEFAULT NULL,
  `coins_coffer` varchar(255) DEFAULT NULL,
  `trash_ret` varchar(255) DEFAULT NULL,
  `exile` varchar(255) DEFAULT NULL,
  `junk` varchar(255) DEFAULT NULL,
  `gain` varchar(255) DEFAULT NULL,
  `victorypts` varchar(255) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `parsed_cards`
--

INSERT INTO `parsed_cards` (`id`, `added`, `skip`, `en_added`, `no_finnish`, `name`, `expansion`, `exp_id`, `type`, `cost`, `text`, `act_villager`, `draws`, `buys`, `coins_coffer`, `trash_ret`, `exile`, `junk`, `gain`, `victorypts`) VALUES
(2537, 1, 0, 1, 0, 'Adventurer', 'Base, 1E', 1, 'Action', '6', 'Reveal cards from your deck until you reveal 2 Treasure\ncards. Put those Treasure cards into your hand and discard the\nother revealed cards.', NULL, '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2538, 0, 0, 0, 0, 'Artisan', 'Base, 2E', 1, 'Action', '6', 'Gain a card to your hand costing up to 5.Put a card from your\nhand onto your deck.', NULL, '(+1), -1', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2539, 0, 0, 0, 0, 'Bandit', 'Base, 2E', 1, 'Action - Attack', '5', 'Gain a Gold. Each other player reveals the top 2 cards of\ntheir deck, trashes a revealed Treasure other than Copper, and\ndiscards the rest.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2540, 1, 0, 1, 0, 'Bureaucrat', 'Base', 1, 'Action - Attack', '4', 'Gain a Silver onto your deck. Each other player reveals a\nVictory card from their hand and puts it onto their deck (or\nreveals a hand with no Victory cards).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2541, 1, 0, 1, 0, 'Cellar', 'Base', 1, 'Action', '2', '+1 ActionDiscard any number of cards. +1 Card per\ncard discarded.', '+1', '-X, +X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2542, 1, 0, 1, 0, 'Chancellor', 'Base, 1E', 1, 'Action', '3', '+2You may immediately\nput your deck into your discard pile.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2543, 1, 0, 1, 0, 'Chapel', 'Base', 1, 'Action', '2', 'Trash up to 4 cards from your hand.', NULL, NULL, NULL, NULL, '0-4', NULL, NULL, NULL, NULL),
(2544, 1, 0, 1, 0, 'Council Room', 'Base', 1, 'Action', '5', '+4 Cards+1 BuyEach other player draws a card.', NULL, '+4', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2545, 1, 0, 1, 0, 'Feast', 'Base, 1E', 1, 'Action', '4', 'Trash this card. Gain a card costing up to 5.', NULL, NULL, NULL, NULL, 'Self', NULL, NULL, '1', NULL),
(2546, 1, 0, 1, 0, 'Festival', 'Base', 1, 'Action', '5', '+2 Actions+1 Buy+2', '+2', NULL, '+1', '+2', NULL, NULL, NULL, NULL, NULL),
(2547, 1, 0, 1, 0, 'Gardens', 'Base', 1, 'Victory', '4', 'Worth 1  per 10 cards you\nhave (round down).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2548, 0, 0, 0, 0, 'Harbinger', 'Base, 2E', 1, 'Action', '3', '+1 Card+1 ActionLook through your discard pile. You\nmay put a card from it onto your deck.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2549, 1, 0, 1, 0, 'Laboratory', 'Base', 1, 'Action', '5', '+2 Cards+1 Action', '+1', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2550, 1, 0, 1, 0, 'Library', 'Base', 1, 'Action', '5', 'Draw until you have 7 cards in hand, skipping any Action\ncards you choose to; set those aside, discarding them\nafterwards.', NULL, '=7', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2551, 1, 0, 1, 0, 'Market', 'Base', 1, 'Action', '5', '+1 Card+1 Action+1 Buy+1', '+1', '+1', '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(2552, 0, 0, 0, 0, 'Merchant', 'Base, 2E', 1, 'Action', '3', '+1 Card+1 ActionThe first time you play a Silver this\nturn, +1.', '+1', '+1', NULL, '!+1', NULL, NULL, NULL, NULL, NULL),
(2553, 1, 0, 1, 0, 'Militia', 'Base', 1, 'Action - Attack', '4', '+2Each other player\ndiscards down to 3 cards in hand.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2554, 1, 0, 1, 0, 'Mine', 'Base', 1, 'Action', '5', 'You may trash a Treasure from your hand. Gain a Treasure to\nyour hand costing up to 3 more than it.', NULL, '!(+1)', NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(2555, 1, 0, 1, 0, 'Moat', 'Base', 1, 'Action - Reaction', '2', '+2 CardsWhen another player plays an Attack card, you may\nfirst reveal this from your hand, to be unaffected by it.', NULL, '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2556, 1, 0, 1, 0, 'Moneylender', 'Base', 1, 'Action', '4', 'You may trash a Copper from your hand for +3.', NULL, NULL, NULL, '!+3', '1?', NULL, NULL, NULL, NULL),
(2557, 0, 0, 0, 0, 'Poacher', 'Base, 2E', 1, 'Action', '4', '+1 Card+1 Action+1Discard a card per\nempty Supply pile.', '+1', '+1, -X', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2558, 1, 0, 1, 0, 'Remodel', 'Base', 1, 'Action', '4', 'Trash a card from your hand. Gain a card costing up to\n2 more than it.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '!1', NULL),
(2559, 0, 0, 0, 0, 'Sentry', 'Base, 2E', 1, 'Action', '5', '+1 Card+1 ActionLook at the top 2 cards of your\ndeck. Trash and/or discard any number of them. Put the rest back on\ntop in any order.', '+1', '+1', NULL, NULL, '0-2?', NULL, NULL, NULL, NULL),
(2560, 1, 0, 1, 0, 'Smithy', 'Base', 1, 'Action', '4', '+3 Cards', NULL, '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2561, 1, 0, 1, 0, 'Spy', 'Base, 1E', 1, 'Action - Attack', '4', '+1 Card+1 ActionEach player (including you) reveals\nthe top card of their deck and either discards it or puts it back,\nyour choice.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2562, 1, 0, 1, 0, 'Thief', 'Base, 1E', 1, 'Action - Attack', '4', 'Each other player reveals the top 2 cards of their deck.\nIf they revealed any Treasure cards, they trash one of them that\nyou choose. You may gain any or all of these trashed cards. They\ndiscard the other revealed cards.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(2563, 1, 0, 1, 0, 'Throne Room', 'Base', 1, 'Action', '4', 'You may play an Action card from your hand twice.', 'P2?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2564, 0, 0, 0, 0, 'Vassal', 'Base, 2E', 1, 'Action', '3', '+2Discard the top card\nof your deck. If it\'s an Action card, you may play it.', '!P1?', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2565, 1, 0, 1, 0, 'Village', 'Base', 1, 'Action', '3', '+1 Card+2 Actions', '+2', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2566, 1, 0, 1, 0, 'Witch', 'Base', 1, 'Action - Attack', '5', '+2 CardsEach other player gains a Curse.', NULL, '+2', NULL, NULL, NULL, NULL, '1', NULL, NULL),
(2567, 1, 0, 1, 0, 'Woodcutter', 'Base, 1E', 1, 'Action', '3', '+1 Buy+2', NULL, NULL, '+1', '+2', NULL, NULL, NULL, NULL, NULL),
(2568, 1, 0, 1, 0, 'Workshop', 'Base', 1, 'Action', '3', 'Gain a card costing up to 4.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2569, 0, 0, 0, 0, 'Copper', 'Base', 1, 'Treasure', NULL, '1', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2570, 0, 0, 0, 0, 'Silver', 'Base', 1, 'Treasure', '3', '2', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2571, 0, 0, 0, 0, 'Gold', 'Base', 1, 'Treasure', '6', '3', NULL, NULL, NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(2572, 0, 0, 0, 0, 'Estate', 'Base', 1, 'Victory', '2', '1 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1 '),
(2573, 0, 0, 0, 0, 'Duchy', 'Base', 1, 'Victory', '5', '3 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '3 '),
(2574, 0, 0, 0, 0, 'Province', 'Base', 1, 'Victory', '8', '6 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '6 '),
(2575, 0, 0, 0, 0, 'Curse', 'Base', 1, 'Curse', NULL, '-1 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-1 '),
(2576, 1, 0, 0, 0, 'Baron', 'Intrigue', 2, 'Action', '4', '+1 BuyYou may discard an Estate for +4. If you don\'t, gain\nan Estate.', NULL, '-1?', '+1', '!+4', NULL, NULL, NULL, '!1', NULL),
(2577, 1, 0, 0, 0, 'Bridge', 'Intrigue', 2, 'Action', '4', '+1 Buy+1This turn, cards\n(everywhere) cost 1 less.', NULL, NULL, '+1', '+1, R1', NULL, NULL, NULL, NULL, NULL),
(2578, 1, 0, 0, 0, 'Conspirator', 'Intrigue', 2, 'Action', '4', '+2If you\'ve played\n3 or more Actions this turn (counting this), +1 Card and\n+1 Action.', '!+1', '!+1', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2579, 1, 0, 0, 0, 'Coppersmith', 'Intrigue, 1E', 2, 'Action', '4', 'Copper produces an extra 1 this turn.', NULL, NULL, NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2580, 0, 1, 0, 0, 'Courtier', 'Intrigue, 2E', 2, 'Action', '5', 'Reveal a card from your hand. For each type it has (Action,\nAttack, etc.), choose one: +1 Action; or +1 Buy; or\n+3; or gain a Gold. The\nchoices must be different.', '+1?', NULL, '+1?', '+3?', NULL, NULL, NULL, '1?', NULL),
(2581, 1, 0, 0, 0, 'Courtyard', 'Intrigue', 2, 'Action', '2', '+3 CardsPut a card from your hand onto your deck.', NULL, '+3, -1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2582, 0, 1, 0, 0, 'Diplomat', 'Intrigue, 2E', 2, 'Action - Reaction', '4', '+2 CardsIf you have 5 or fewer cards in hand (after\ndrawing), +2 Actions.When another player plays an Attack card,\nyou may first reveal this from a hand of 5 or more cards, to\ndraw 2 cards then discard 3.', '!+2', '+2**+2, **-3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2583, 1, 0, 0, 0, 'Duke', 'Intrigue', 2, 'Victory', '5', 'Worth 1  per Duchy you\nhave.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2584, 1, 0, 0, 0, 'Great Hall', 'Intrigue, 1E', 2, 'Action - Victory', '3', '+1 Card+1 Action1 ', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, '1 '),
(2585, 1, 0, 0, 0, 'Harem Farm', 'Intrigue', 2, 'Treasure - Victory', '6', '22 ', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, '2 '),
(2586, 1, 0, 0, 0, 'Ironworks', 'Intrigue', 2, 'Action', '4', 'Gain a card costing up to 4.If the gained card is\nan…Action card, +1 ActionTreasure card, +1Victory card,\n+1 Card', '!+1', '!+1', NULL, '!+1', NULL, NULL, NULL, '1', NULL),
(2587, 0, 1, 0, 0, 'Lurker', 'Intrigue, 2E', 2, 'Action', '2', '+1 ActionChoose one: Trash an Action card from the Supply;\nor gain an Action card from the trash.', '+1', NULL, NULL, NULL, '1 Supply?', NULL, NULL, '1?', NULL),
(2588, 1, 0, 0, 0, 'Masquerade', 'Intrigue', 2, 'Action', '3', '+2 CardsEach player with any cards in hand passes one to\nthe next such player to their left, at once. Then you may trash a\ncard from your hand.', NULL, '+2, -1', NULL, NULL, '1?', NULL, '1?, !Self', '1', NULL),
(2589, 0, 1, 0, 0, 'Mill', 'Intrigue, 2E', 2, 'Action - Victory', '4', '+1 Card+1 ActionYou may discard 2 cards. If you\ndo, +2.1 ', '+1', '+1, - 0-2?(X)', NULL, '!+', NULL, NULL, NULL, NULL, '1 '),
(2590, 1, 0, 0, 0, 'Mining Village', 'Intrigue', 2, 'Action', '4', '+1 Card+2 ActionsYou may trash this for +.', '+2', '+1', NULL, '!+2', 'Self?', NULL, NULL, NULL, NULL),
(2591, 1, 0, 0, 0, 'Minion', 'Intrigue', 2, 'Action - Attack', '5', '+1 ActionChoose one: +2; or discard your\nhand, +4 Cards, and each other player with at least\n5 cards in hand discards their hand and draws\n4 cards.', '+1', '-X +4 ?', NULL, '+2?', NULL, NULL, NULL, NULL, NULL),
(2592, 1, 0, 0, 0, 'Nobles', 'Intrigue', 2, 'Action - Victory', '6', 'Choose one: +3 Cards; or +2 Actions.2 ', '+2?', '+3?', NULL, NULL, NULL, NULL, NULL, NULL, '2 '),
(2593, 0, 1, 0, 0, 'Patrol', 'Intrigue, 2E', 2, 'Action', '5', '+3 CardsReveal the top 4 cards of your deck. Put the\nVictory cards and Curses into your hand. Put the rest back in any\norder.', NULL, '+ 3-7', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2594, 1, 0, 0, 0, 'Pawn', 'Intrigue', 2, 'Action', '2', 'Choose two: +1 Card; +1 Action; +1 Buy;\n+1. The choices must be\ndifferent.', '+1?', '+1?', '+1?', '+1?', NULL, NULL, NULL, NULL, NULL),
(2595, 0, 1, 0, 0, 'Replace', 'Intrigue, 2E', 2, 'Action - Attack', '5', 'Trash a card from your hand. Gain a card costing up to\n2 more than it. If the\ngained card is an Action or Treasure, put it onto your deck; if\nit\'s a Victory card, each other player gains a Curse.', NULL, NULL, NULL, NULL, '1', NULL, '!1', '1', NULL),
(2596, 1, 0, 0, 0, 'Saboteur', 'Intrigue, 1E', 2, 'Action - Attack', '5', 'Each other player reveals cards from the top of their deck\nuntil revealing one costing 3 or more. They trash\nthat card and may gain a card costing at most 2 less than it. They\ndiscard the other revealed cards.', NULL, NULL, NULL, NULL, NULL, NULL, '!1', NULL, NULL),
(2597, 1, 0, 0, 0, 'Scout', 'Intrigue, 1E', 2, 'Action', '4', '+1 ActionReveal the top 4 cards of your deck. Put the\nrevealed Victory cards into your hand. Put the other cards on top\nof your deck in any order.', '+1', '+ 0-4', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2598, 1, 0, 0, 0, 'Secret Chamber', 'Intrigue, 1E', 2, 'Action - Reaction', '2', 'Discard any number of cards. +1 per card\ndiscarded.When another player plays an Attack card, you may reveal\nthis from your hand. If you do, +2 Cards, then put\n2 cards from your hand on top of your deck.', NULL, '-X', NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2599, 0, 1, 0, 0, 'Secret Passage', 'Intrigue, 2E', 2, 'Action', '4', '+2 Cards+1 ActionTake a card from your hand and put\nit anywhere in your deck.', '+1', '+2, -1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2600, 1, 0, 0, 0, 'Shanty Town', 'Intrigue', 2, 'Action', '3', '+2 ActionsReveal your hand. If you have no Action cards in\nhand, +2 Cards.', '+2', '+2?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2601, 1, 0, 0, 0, 'Steward', 'Intrigue', 2, 'Action', '3', 'Choose one: +2 Cards; or +2; or trash\n2 cards from your hand.', NULL, '+2?', NULL, '+2?', '2?', NULL, NULL, NULL, NULL),
(2602, 1, 0, 0, 0, 'Swindler', 'Intrigue', 2, 'Action - Attack', '3', '+2Each other player\ntrashes the top card of their deck and gains a card with the same\ncost that you choose.', NULL, NULL, NULL, '+2', NULL, NULL, '!1', NULL, NULL),
(2603, 1, 0, 0, 0, 'Torturer', 'Intrigue', 2, 'Action - Attack', '5', '+3 CardsEach other player either discards 2 cards or\ngains a Curse to their hand, their choice. (They may pick an option\nthey can\'t do.)', NULL, '+3', NULL, NULL, NULL, NULL, '1?', NULL, NULL),
(2604, 1, 0, 0, 0, 'Trading Post', 'Intrigue', 2, 'Action', '5', 'Trash 2 cards from your hand. If you did, gain a Silver to\nyour hand.', NULL, '!(+1)', NULL, NULL, '2', NULL, NULL, '!1', NULL),
(2605, 1, 0, 0, 0, 'Tribute', 'Intrigue, 1E', 2, 'Action', '5', 'The player to your left reveals then discards the top\n2 cards of their deck. For each differently named card\nrevealed, if it is an…Action Card, +2 ActionsTreasure Card,\n+2Victory Card,\n+2 Cards', '+2Z', '+2Y', NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2606, 1, 0, 0, 0, 'Upgrade', 'Intrigue', 2, 'Action', '5', '+1 Card+1 ActionTrash a card from your hand. Gain a\ncard costing exactly 1 more than it.', '+1', '+1', NULL, NULL, '1', NULL, NULL, '1', NULL),
(2607, 1, 0, 0, 0, 'Wishing Well', 'Intrigue', 2, 'Action', '3', '+1 Card+1 ActionName a card, then reveal the top card\nof your deck. If you named it, put it into your hand.', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2608, 0, 0, 0, 0, 'Ambassador', 'Seaside, 1E', 3, 'Action - Attack', '3', 'Reveal a card from your hand. Return up to 2 copies of it\nfrom your hand to the Supply. Then each other player gains a copy\nof it.', NULL, NULL, NULL, NULL, '0-2?', NULL, '!1', NULL, NULL),
(2609, 0, 0, 0, 0, 'Astrolabe', 'Seaside, 2E', 3, 'Treasure - Duration', '3', 'Now and at the start of your next turn:1+1 Buy', NULL, NULL, '+1, N +1', '+1, N +1', NULL, NULL, NULL, NULL, NULL),
(2610, 0, 0, 0, 0, 'Bazaar', 'Seaside', 3, 'Action', '5', '+1 Card+2 Actions+1', '+2', '+1', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2611, 0, 0, 0, 0, 'Blockade', 'Seaside, 2E', 3, 'Action - Duration - Attack', '4', 'Gain a card costing up to 4, setting it aside.At\nthe start of your next turn, put it into your hand. While it\'s set\naside, when another player gains a copy of it on their turn, they\ngain a Curse.', NULL, 'N +1', NULL, NULL, NULL, NULL, '**1?', '1', NULL),
(2612, 0, 0, 0, 0, 'Caravan', 'Seaside', 3, 'Action - Duration', '4', '+1 Card+1 ActionAt the start of your next turn,\n+1 Card.', '+1', '+1, N +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2613, 0, 0, 0, 0, 'Corsair', 'Seaside, 2E', 3, 'Action - Duration - Attack', '5', '+2At the start of your\nnext turn, +1 Card. Until then, each other player trashes the\nfirst Silver or Gold they play each turn.', NULL, 'N +1', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2614, 0, 0, 0, 0, 'Cutpurse', 'Seaside', 3, 'Action - Attack', '4', '+2Each other player\ndiscards a Copper (or reveals a hand with no Copper).', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2615, 0, 0, 0, 0, 'Embargo', 'Seaside, 1E', 3, 'Action', '2', '+2Trash this to add an\nEmbargo token to a Supply pile. (For the rest of the game, when a\nplayer buys a card from that pile, they gain a Curse.)', NULL, NULL, NULL, '+2', 'Self', NULL, '**1?', NULL, NULL),
(2616, 0, 0, 0, 0, 'Explorer', 'Seaside, 1E', 3, 'Action', '5', 'You may reveal a Province from your hand. If you do, gain a\nGold to your hand. If you don\'t, gain a Silver to your hand.', NULL, '!(+1)', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(2617, 0, 0, 0, 0, 'Fishing Village', 'Seaside', 3, 'Action - Duration', '3', '+2 Actions+1At the start of your\nnext turn: +1 Action and +1.', '+2, N +1', NULL, NULL, '+1, N +1', NULL, NULL, NULL, NULL, NULL),
(2618, 0, 0, 0, 0, 'Ghost Ship', 'Seaside, 1E', 3, 'Action - Attack', '5', '+2 CardsEach other player with 4 or more cards in\nhand puts cards from their hand onto their deck until they have\n3 cards in hand.', NULL, '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2619, 0, 0, 0, 0, 'Haven', 'Seaside', 3, 'Action - Duration', '2', '+1 Card+1 ActionSet aside a card from your hand face\ndown (under this). At the start of your next turn, put it into your\nhand.', '+1', '+1, -1, N +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2620, 0, 0, 0, 0, 'Island', 'Seaside', 3, 'Action - Victory', '4', 'Put this and a card from your hand onto your Island\nmat.2 ', NULL, NULL, NULL, NULL, '1, Self', NULL, NULL, NULL, '2 '),
(2621, 0, 0, 0, 0, 'Lighthouse', 'Seaside', 3, 'Action - Duration', '2', '+1 Action+1At the start of your\nnext turn, +1. Until then, when\nanother player plays an Attack card, it doesn\'t affect you.', '+1', NULL, NULL, '+1, N +1', NULL, NULL, NULL, NULL, NULL),
(2622, 0, 0, 0, 0, 'Lookout', 'Seaside', 3, 'Action', '3', '+1 ActionLook at the top 3 cards of your deck. Trash\none of them. Discard one of them. Put the other one back on top of\nyour deck.', '+1', NULL, NULL, NULL, '1', NULL, NULL, NULL, NULL),
(2623, 0, 0, 0, 0, 'Merchant Ship', 'Seaside', 3, 'Action - Duration', '5', 'Now and at the start of your next turn: +2.', NULL, NULL, NULL, '+2, N +2', NULL, NULL, NULL, NULL, NULL),
(2624, 0, 0, 0, 0, 'Monkey', 'Seaside, 2E', 3, 'Action - Duration', '3', 'Until your next turn, when the player to your right gains a\ncard, +1 Card.At the start of your next turn,\n+1 Card.', NULL, '**+1, N +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2625, 0, 0, 0, 0, 'Native Village', 'Seaside', 3, 'Action', '2', '+2 ActionsChoose one: Put the top card of your deck face\ndown on your Native Village mat (you may look at those cards at any\ntime); or put all the cards from your mat into your hand.', '+2', '+X?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2626, 0, 0, 0, 0, 'Navigator', 'Seaside, 1E', 3, 'Action', '4', '+2Look at the top\n5 cards of your deck. Either discard them all, or put them\nback in any order.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2627, 0, 0, 0, 0, 'Outpost', 'Seaside', 3, 'Action - Duration', '5', 'You only draw 3 cards for your next hand. Take an extra\nturn after this one (but not a 3rd turn in a row).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2628, 0, 0, 0, 0, 'Pearl Diver', 'Seaside, 1E', 3, 'Action', '2', '+1 Card+1 ActionLook at the bottom card of your deck.\nYou may put it on top.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2629, 0, 0, 0, 0, 'Pirate', 'Seaside, 2E', 3, 'Action - Duration - Reaction', '5', 'At the start of your next turn, gain a Treasure costing up to\n6 to your hand.When any\nplayer gains a Treasure, you may play this from your hand.', '**Pself', 'N (+1)', NULL, NULL, NULL, NULL, NULL, 'N 1', NULL),
(2630, 0, 0, 0, 0, 'Pirate Ship', 'Seaside, 1E', 3, 'Action - Attack', '4', 'Choose one: +1 per Coin token on\nyour Pirate Ship mat; or each other player reveals the top\n2 cards of their deck, trashes one of those Treasures that you\nchoose, and discards the rest, and then if anyone trashed a\nTreasure, you add a Coin token to your Pirate Ship mat.', NULL, NULL, NULL, '+?', NULL, NULL, NULL, NULL, NULL),
(2631, 0, 0, 0, 0, 'Sailor', 'Seaside, 2E', 3, 'Action - Duration', '4', '+1 ActionOnce this turn, when you gain a Duration card,\nyou may play it.At the start of your next turn, +2 and you may trash a\ncard from your hand.', '+1, !P1?', NULL, NULL, 'N +2', 'N 1?', NULL, NULL, NULL, NULL),
(2632, 0, 0, 0, 0, 'Salvager', 'Seaside', 3, 'Action', '4', '+1 BuyTrash a card from your hand. +1 per 1 it costs.', NULL, NULL, '+1', '+', '1', NULL, NULL, NULL, NULL),
(2633, 0, 0, 0, 0, 'Sea Chart', 'Seaside, 2E', 3, 'Action', '3', '+1 Card+1 ActionReveal the top card of your deck. If\nyou have a copy of it in play, put it into your hand.', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2634, 0, 0, 0, 0, 'Sea Hag', 'Seaside, 1E', 3, 'Action - Attack', '4', 'Each other player discards the top card of their deck, then\ngains a Curse onto their deck.', NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL, NULL),
(2635, 0, 0, 0, 0, 'Sea Witch', 'Seaside, 2E', 3, 'Action - Duration - Attack', '5', '+2 CardsEach other player gains a Curse.At the start of\nyour next turn, +2 Cards, then discard 2 cards.', NULL, '+2, N +2, N -2', NULL, NULL, NULL, NULL, '1', NULL, NULL),
(2636, 0, 0, 0, 0, 'Smugglers', 'Seaside', 3, 'Action', '3', 'Gain a copy of a card costing up to 6 that the player to\nyour right gained on their last turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2637, 0, 0, 0, 0, 'Tactician', 'Seaside', 3, 'Action - Duration', '5', 'If you have at least one card in hand: Discard your hand, and\nat the start of your next turn, +5 Cards, +1 Action, and\n+1 Buy.', '!N +1', '-X, !N +5', '!N +1', NULL, NULL, NULL, NULL, NULL, NULL),
(2638, 0, 0, 0, 0, 'Tide Pools', 'Seaside, 2E', 3, 'Action - Duration', '4', '+3 Cards+1 ActionAt the start of your next turn,\ndiscard 2 cards.', '+1', '+3, N -2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2639, 0, 0, 0, 0, 'Treasure Map', 'Seaside', 3, 'Action', '4', 'Trash this and a Treasure Map from your hand. If you trashed\ntwo Treasure Maps, gain 4 Golds onto your deck.', NULL, NULL, NULL, NULL, '1?, Self', NULL, NULL, '!4', NULL),
(2640, 0, 0, 0, 0, 'Treasury', 'Seaside', 3, 'Action', '5', '+1 Card+1 Action+1At the end of your Buy\nphase this turn, if you didn\'t gain a Victory card in it, you may\nput this onto your deck.', '+1', '+1', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2641, 0, 0, 0, 0, 'Warehouse', 'Seaside', 3, 'Action', '3', '+3 Cards+1 ActionDiscard 3 cards.', '+1', '+3, -3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2642, 0, 0, 0, 0, 'Wharf', 'Seaside', 3, 'Action - Duration', '5', 'Now and at the start of your next turn: +2 Cards and\n+1 Buy.', NULL, '+2, N +2', '+1, N +1', NULL, NULL, NULL, NULL, NULL, NULL),
(2643, 0, 0, 0, 0, 'Alchemist', 'Alchemy', 4, 'Action', '3', '+2 Cards+1 ActionAt the start of Clean-up this turn,\nif you have a Potion in play, you may put this onto your deck.', '+1', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2644, 0, 0, 0, 0, 'Apothecary', 'Alchemy', 4, 'Action', '2', '+1 Card+1 ActionReveal the top 4 cards of your\ndeck. Put the Coppers and Potions into your hand. Put the rest back\nin any order.', '+1', '+ 1-5', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2645, 0, 0, 0, 0, 'Apprentice', 'Alchemy', 4, 'Action', '5', '+1 ActionTrash a card from your hand.+1 Card per\n1 it\ncosts.+2 Cards if it has  in its cost.', '+1', '+X', NULL, NULL, '1', NULL, NULL, NULL, NULL),
(2646, 0, 0, 0, 0, 'Familiar', 'Alchemy', 4, 'Action - Attack', '3', '+1 Card+1 ActionEach other player gains a Curse.', '+1', '+1', NULL, NULL, NULL, NULL, '1', NULL, NULL),
(2647, 0, 0, 0, 0, 'Golem', 'Alchemy', 4, 'Action', '4', 'Reveal cards from your deck until you reveal 2 Action\ncards other than Golems. Discard the other cards, then play the\nAction cards in either order.', 'P1, P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2648, 0, 0, 0, 0, 'Herbalist', 'Alchemy', 4, 'Action', '2', '+1 Buy+1Once this turn, when\nyou discard a Treasure from play, you may put it onto your\ndeck.', NULL, NULL, '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(2649, 0, 0, 0, 0, 'Philosopher\'s Stone', 'Alchemy', 4, 'Treasure', '3', 'Count your deck and discard pile. +1 per 5 cards\ntotal between them (round down).', NULL, NULL, NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2650, 0, 0, 0, 0, 'Possession', 'Alchemy', 4, 'Action', '6', 'The player to your left takes an extra turn after this one (but\nnot a 2nd extra turn in a row), in which you can see all cards they\ncan and make all decisions for them. Any cards or  they would gain on\nthat turn, you gain instead; any cards of theirs that are trashed\nare set aside and put in their discard pile at end of turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(2651, 0, 0, 0, 0, 'Scrying Pool', 'Alchemy', 4, 'Action - Attack', '2', '+1 ActionEach player (including you) reveals the top card\nof their deck and either discards it or puts it back, your choice.\nThen reveal cards from your deck until revealing one that isn\'t an\nAction. Put all of those revealed cards into your hand.', '+1', '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2652, 0, 0, 0, 0, 'Transmute', 'Alchemy', 4, 'Action', NULL, 'Trash a card from your hand. If it is an…Action card, gain a\nDuchyTreasure card, gain a TransmuteVictory card, gain a Gold', NULL, NULL, NULL, NULL, '1', NULL, NULL, '1', '!(3 )'),
(2653, 0, 0, 0, 0, 'University', 'Alchemy', 4, 'Action', '2', '+2 ActionsYou may gain an Action card costing up to\n5.', '+2', NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2654, 0, 0, 0, 0, 'Vineyard', 'Alchemy', 4, 'Victory', NULL, 'Worth 1  per 3 Action\ncards you have (rounded down).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2655, 0, 0, 0, 0, 'Potion', 'Alchemy', 4, 'Treasure', '4', NULL, NULL, NULL, NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2656, 0, 0, 0, 1, 'Anvil', 'Prosperity, 2E', 5, 'Treasure', '3', '1You may discard a\nTreasure to gain a card costing up to 4.', NULL, '-1?', NULL, '+1', NULL, NULL, NULL, '!1', NULL),
(2657, 1, 0, 1, 0, 'Bank', 'Prosperity', 5, 'Treasure', '7', '+1 per Treasure card you\nhave in play (counting this).', NULL, NULL, NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2658, 1, 0, 1, 0, 'Bishop', 'Prosperity', 5, 'Action', '4', '+1+1 Trash a card from your\nhand. +1  per 2 it costs (round\ndown). Each other player may trash a card from their hand.', NULL, NULL, NULL, '+1', '1', NULL, NULL, NULL, '+1 , +X '),
(2659, 0, 0, 0, 1, 'Charlatan', 'Prosperity, 2E', 5, 'Action - Attack', '5', '+3Each other player\ngains a Curse.In games using this, Curse is also a Treasure worth\n1.', NULL, NULL, NULL, '+3', NULL, NULL, '1', NULL, NULL),
(2660, 1, 0, 1, 0, 'City', 'Prosperity', 5, 'Action', '5', '+1 Card+2 ActionsIf there are one or more empty\nSupply piles, +1 Card. If there are two or more, +1 Buy\nand +1.', '+2', '+1, !+1', '!+1', '!+1', NULL, NULL, NULL, NULL, NULL),
(2661, 0, 0, 0, 1, 'Clerk', 'Prosperity, 2E', 5, 'Action - Reaction\n- Attack', '4', '+2Each other player with\n5 or more cards in hand puts one onto their deck.At the start of\nyour turn, you may play this from your hand.', 'Pself', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2662, 0, 0, 0, 1, 'Collection', 'Prosperity, 2E', 5, 'Treasure', '5', '2+1 BuyThis turn,\nwhen you gain an Action card, +1 .', NULL, NULL, '+1', '+2', NULL, NULL, NULL, NULL, '+X '),
(2663, 1, 0, 1, 0, 'Contraband', 'Prosperity, 1E', 5, 'Treasure', '5', '3+1 BuyThe player\nto your left names a card. You can\'t buy that card this turn.', NULL, NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(2664, 1, 0, 1, 0, 'Counting House', 'Prosperity, 1E', 5, 'Action', '5', 'Look through your discard pile, reveal any number of Coppers\nfrom it, and put them into your hand.', NULL, '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2665, 0, 0, 0, 1, 'Crystal Ball', 'Prosperity, 2E', 5, 'Treasure', '5', '1Look at the top card\nof your deck. You may trash it, discard it, or, if it\'s an Action\nor Treasure, play it.', '!P1?', NULL, NULL, '+1, !P1?', '1?', NULL, NULL, NULL, NULL),
(2666, 1, 0, 1, 0, 'Expand', 'Prosperity', 5, 'Action', '7', 'Trash a card from your hand. Gain a card costing up to\n3 more than it.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '1', NULL),
(2667, 1, 0, 1, 0, 'Forge', 'Prosperity', 5, 'Action', '7', 'Trash any number of cards from your hand. Gain a card with cost\nexactly equal to the total cost in  of the trashed\ncards.', NULL, NULL, NULL, NULL, 'X', NULL, NULL, '1', NULL),
(2668, 1, 0, 1, 0, 'Goons', 'Prosperity, 1E', 5, 'Action - Attack', '6', '+1 Buy+2Each other player\ndiscards down to 3 cards in hand.While you have this in play,\nwhen you buy a card, +1 .', NULL, NULL, '+1', '+2', NULL, NULL, NULL, NULL, '+X '),
(2669, 1, 0, 1, 0, 'Grand Market', 'Prosperity', 5, 'Action', NULL, '+1 Card+1 Action+1 Buy+2You can’t buy this if\nyou have any Coppers in play.', '+1', '+1', '+1', '+2', NULL, NULL, NULL, NULL, NULL),
(2670, 1, 0, 1, 0, 'Hoard', 'Prosperity', 5, 'Treasure', '6', '2This turn, when you\ngain a Victory card, if you bought it, gain a Gold.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, 'X', NULL),
(2671, 0, 0, 0, 1, 'Investment', 'Prosperity, 2E', 5, 'Treasure', '4', 'Trash a card from your hand. Choose one: +1; or trash this to\nreveal your hand for +1  per differently named\nTreasure there.', NULL, NULL, NULL, '+1?', '1', NULL, NULL, NULL, '+X ?'),
(2672, 1, 0, 1, 0, 'King\'s Court', 'Prosperity', 5, 'Action', '7', 'You may play an Action card from your hand three times.', 'P3?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2673, 1, 0, 1, 0, 'Loan', 'Prosperity, 1E', 5, 'Treasure', '3', '1Reveal cards from your\ndeck until you reveal a Treasure. Discard it or trash it. Discard\nthe other cards.', NULL, NULL, NULL, '+1', '1?', NULL, NULL, NULL, NULL),
(2674, 0, 0, 0, 1, 'Magnate', 'Prosperity, 2E', 5, 'Action', '5', 'Reveal your hand. +1 Card per Treasure in it.', NULL, '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2675, 1, 0, 1, 0, 'Mint', 'Prosperity', 5, 'Action', '5', 'You may reveal a Treasure card from your hand. Gain a copy of\nit.When you gain this, trash all non-Duration Treasures you have in\nplay.', NULL, NULL, NULL, NULL, '*X', NULL, NULL, '1?', NULL),
(2676, 1, 0, 1, 0, 'Monument', 'Prosperity', 5, 'Action', '4', '+2+1 ', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, '+1 '),
(2677, 1, 0, 1, 0, 'Mountebank', 'Prosperity, 1E', 5, 'Action - Attack', '5', '+2Each other player may\ndiscard a Curse. If they don\'t, they gain a Curse and a\nCopper.', NULL, NULL, NULL, '+2', NULL, NULL, '2?', NULL, NULL),
(2678, 1, 0, 1, 0, 'Peddler', 'Prosperity', 5, 'Action', NULL, '+1 Card+1 Action+1During a player\'s Buy\nphase, this costs 2 less per Action card\nthey have in play.', '+1', '+1', NULL, '+1, RXself', NULL, NULL, NULL, NULL, NULL),
(2679, 1, 0, 1, 0, 'Quarry', 'Prosperity', 5, 'Treasure', '4', '1This turn, Actions\ncost 2 less.', NULL, NULL, NULL, '+1, !R2', NULL, NULL, NULL, NULL, NULL),
(2680, 1, 0, 1, 0, 'Rabble', 'Prosperity', 5, 'Action - Attack', '5', '+3 CardsEach other player reveals the top 3 cards of\ntheir deck, discards the Actions and Treasures, and puts the rest\nback in any order they choose.', NULL, '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2681, 1, 0, 1, 0, 'Royal Seal', 'Prosperity, 1E', 5, 'Treasure', '5', '2While you have this in\nplay, when you gain a card, you may put that card onto your\ndeck.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2682, 1, 0, 1, 0, 'Talisman', 'Prosperity, 1E', 5, 'Treasure', '4', '1While you have this in\nplay, when you buy a non-Victory card costing 4 or less, gain a copy\nof it.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, '!1', NULL),
(2683, 0, 0, 0, 1, 'Tiara', 'Prosperity, 2E', 5, 'Treasure', '4', '+1 BuyThis turn, when you gain a card, you may put it onto\nyour deck.You may play a Treasure from your hand twice.', NULL, NULL, '+1', 'P2?', NULL, NULL, NULL, NULL, NULL),
(2684, 1, 0, 1, 0, 'Trade Route', 'Prosperity, 1E', 5, 'Action', '3', '+1 BuyTrash a card from your hand. +1 per Coin token on the\nTrade Route mat.Setup: Add a Coin token to each Victory Supply\npile; move that token to the Trade Route mat when a card is gained\nfrom the pile.', NULL, NULL, '+1', '+', '1', NULL, NULL, NULL, NULL),
(2685, 1, 0, 1, 0, 'Vault', 'Prosperity', 5, 'Action', '5', '+2 CardsDiscard any number of cards for +1 each.Each other\nplayer may discard 2 cards, to draw a card.', NULL, '+2, -X', NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2686, 1, 0, 1, 0, 'Venture', 'Prosperity, 1E', 5, 'Treasure', '5', '1Reveal cards from your\ndeck until you reveal a Treasure. Discard the other cards. Play\nthat Treasure.', NULL, NULL, NULL, '+1, P1', NULL, NULL, NULL, NULL, NULL),
(2687, 0, 0, 0, 1, 'War Chest', 'Prosperity, 2E', 5, 'Treasure', '5', 'The player to your left names a card. Gain a card costing up to\n5 that hasn\'t been\nnamed for War Chests this turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2688, 1, 0, 1, 0, 'Watchtower', 'Prosperity', 5, 'Action - Reaction', '3', 'Draw until you have 6 cards in hand.When you gain a card,\nyou may reveal this from your hand, to either trash that card or\nput it onto your deck.', NULL, '=6', NULL, NULL, '**1?', NULL, NULL, NULL, NULL),
(2689, 1, 0, 1, 0, 'Worker\'s Village', 'Prosperity', 5, 'Action', '4', '+1 Card+2 Actions+1 Buy', '+2', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2690, 0, 0, 0, 0, 'Platinum', 'Prosperity', 5, 'Treasure', '9', '5', NULL, NULL, NULL, '+5', NULL, NULL, NULL, NULL, NULL),
(2691, 0, 0, 0, 0, 'Colony', 'Prosperity', 5, 'Victory', '11', '10 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '10 '),
(2692, 1, 0, 0, 0, 'Advisor', 'Cornucopia & Guilds', 9, 'Action', '4', '+1 ActionReveal the top 3 cards of your deck. The\nplayer to your left chooses one of them. Discard that card and put\nthe rest into your hand.', '+1', '+3, !-1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2693, 1, 0, 0, 0, 'Baker', 'Cornucopia & Guilds', 9, 'Action', '5', '+1 Card+1 Action+1 CoffersSetup: Each player\ngets +1 Coffers.', '+1', '+1', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2694, 1, 0, 0, 0, 'Butcher', 'Cornucopia & Guilds', 9, 'Action', '5', '+2 CoffersYou may trash a card from your hand, to gain a\ncard, costing up to 1 more than it per\nCoffers you spend.', NULL, NULL, NULL, '+2, -', '1?', NULL, NULL, '!1', NULL),
(2695, 1, 0, 0, 0, 'Candlestick Maker', 'Cornucopia & Guilds', 9, 'Action', '2', '+1 Action+1 Buy+1 Coffers', '+1', NULL, '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(2696, 1, 0, 0, 0, 'Carnival', 'Cornucopia & Guilds, 2E', 9, 'Action', '5', 'Reveal the top 4 cards of your deck. Put one of each\ndifferently named card into your hand and discard the rest.', NULL, '+ 1-4', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2697, 0, 1, 0, 0, 'Doctor', 'Cornucopia & Guilds, 1E', 9, 'Action', NULL, 'Name a card. Reveal the top 3 cards of your deck. Trash\nthe matches. Put the rest back in any order.Overpay: Per  overpaid, look at the\ntop card of your deck; trash it, discard it, or put it back.', NULL, NULL, NULL, NULL, '!0-3*X', NULL, NULL, NULL, NULL),
(2698, 1, 0, 0, 0, 'Fairgrounds', 'Cornucopia & Guilds', 6, 'Victory', '6', 'Worth 2  per\n5 differently named cards you have (round down).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2699, 1, 0, 0, 0, 'Farmhands', 'Cornucopia & Guilds, 2E', 6, 'Action', '4', '+1 Card+2 ActionsWhen you gain this, you may set\naside an Action or Treasure from your hand, and play it at the\nstart of your next turn.', '+2, N P1?', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2700, 0, 1, 0, 0, 'Farming Village', 'Cornucopia & Guilds, 1E', 6, 'Action', '4', '+2 ActionsReveal cards from your deck until you reveal a\nTreasure or Action card. Put that card into your hand and discard\nthe rest.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2701, 1, 0, 0, 0, 'Farrier', 'Cornucopia & Guilds, 2E', 9, 'Action', NULL, '+1 Card+1 Action+1 BuyOverpay: +1 Card at\nthe end of this turn per 1 overpaid.', '+1', '+1, *+X', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2702, 1, 0, 0, 0, 'Ferryman', 'Cornucopia & Guilds, 2E', 6, 'Action', '5', '+2 Cards+1 ActionDiscard a card.Setup: Choose an\nunused Kingdom card pile costing 3 or 4. Gain one when you\ngain a Ferryman.', '+1', '+2, -1', NULL, NULL, NULL, NULL, NULL, '*1', NULL),
(2703, 1, 0, 0, 0, 'Footpad', 'Cornucopia & Guilds, 2E', 9, 'Action - Attack', '5', '+2 CoffersEach other player discards down to 3 cards\nin hand.In games using this, when you gain a card in an Action\nphase, +1 Card.', NULL, 'N... **+X', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2704, 0, 1, 0, 0, 'Fortune Teller', 'Cornucopia & Guilds, 1E', 6, 'Action - Attack', '3', '+2Each other player\nreveals cards from the top of their deck until they reveal a\nVictory card or a Curse. They put it on top and discard the\nrest.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2705, 1, 0, 0, 0, 'Hamlet', 'Cornucopia & Guilds', 6, 'Action', '2', '+1 Card+1 ActionYou may discard a card for\n+1 Action.You may discard a card for +1 Buy.', '+1, !+1', '+1, -2?', '!+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2706, 0, 1, 0, 0, 'Harvest', 'Cornucopia & Guilds, 1E', 6, 'Action', '5', 'Reveal the top 4 cards of your deck, then discard them.\n+1 per differently named\ncard revealed.', NULL, NULL, NULL, '+', NULL, NULL, NULL, NULL, NULL),
(2707, 1, 0, 0, 0, 'Herald', 'Cornucopia & Guilds', 9, 'Action', NULL, '+1 Card+1 ActionReveal the top card of your deck. If\nit\'s an Action, play it.Overpay: Per 1 overpaid, put any\ncard from your discard pile onto your deck.', '+1, !P1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2708, 1, 0, 0, 0, 'Horn of Plenty', 'Cornucopia & Guilds', 6, 'Treasure', '5', 'Gain a card costing up to 1 per differently named\ncard you have in play (counting this). If it\'s a Victory card,\ntrash this.', NULL, NULL, NULL, NULL, 'Self?', NULL, NULL, '1', NULL),
(2709, 0, 1, 0, 0, 'Horse Traders', 'Cornucopia & Guilds, 1E', 6, 'Action - Reaction', '4', '+1 Buy+3Discard\n2 cards.When another player plays an Attack card, you may\nfirst set this aside from your hand. If you do, then at the start\nof your next turn, +1 Card and return this to your hand.', NULL, '-2*N +1', '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(2710, 1, 0, 0, 0, 'Hunting Party', 'Cornucopia & Guilds', 6, 'Action', '5', '+1 Card+1 ActionReveal your hand. Reveal cards from\nyour deck until you reveal a card that isn\'t a copy of one in your\nhand. Put it into your hand and discard the rest.', '+1', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2711, 1, 0, 0, 0, 'Infirmary', 'Cornucopia & Guilds, 2E', 9, 'Action', NULL, '+1 CardYou may trash a card from your hand.Overpay: Play\nthis once per 1 overpaid.', '*PselfX', '+1', NULL, NULL, '1?', NULL, NULL, NULL, NULL),
(2712, 1, 0, 0, 0, 'Jester', 'Cornucopia & Guilds', 6, 'Action - Attack', '5', '+2Each other player\ndiscards the top card of their deck. If it\'s a Victory card they\ngain a Curse; otherwise they gain a copy of the discarded card or\nyou do, your choice.', NULL, NULL, NULL, '+2', NULL, NULL, '!1?', '!1?', NULL),
(2713, 1, 0, 0, 0, 'Journeyman', 'Cornucopia & Guilds', 9, 'Action', '5', 'Name a card. Reveal cards from your deck until you reveal\n3 cards without that name. Put those cards into your hand and\ndiscard the rest.', NULL, '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2714, 1, 0, 0, 0, 'Joust', 'Cornucopia & Guilds, 2E', 9, 'Action', '5', '+1 Card+1 Action+1You may set aside a\nProvince from your hand to gain any Reward to your hand. Discard\nthe Province in Clean-up.', '+1', '+1, !+1', NULL, '+1', NULL, NULL, NULL, '!1', NULL),
(2715, 0, 1, 0, 0, 'Masterpiece', 'Cornucopia & Guilds, 1E', 9, 'Treasure', NULL, '1Overpay: Gain a Silver\nper 1 overpaid.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, '*X', NULL),
(2716, 1, 0, 0, 0, 'Menagerie', 'Cornucopia & Guilds', 6, 'Action', '3', '+1 ActionReveal your hand. If the revealed cards all have\ndifferent names, +3 Cards. Otherwise, +1 Card.', '+1', '+1, !+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2717, 1, 0, 0, 0, 'Merchant Guild', 'Cornucopia & Guilds', 9, 'Action', '5', '+1 Buy+1At the end of your Buy\nphase this turn, +1 Coffers per card you gained in it.', NULL, NULL, '+1', '+1*+', NULL, NULL, NULL, NULL, NULL),
(2718, 1, 0, 0, 0, 'Plaza', 'Cornucopia & Guilds', 9, 'Action', '4', '+1 Card+2 ActionsYou may discard a Treasure for\n+1 Coffers.', '+2', '+1, -1?', NULL, '!+1', NULL, NULL, NULL, NULL, NULL),
(2719, 1, 0, 0, 0, 'Remake', 'Cornucopia & Guilds', 6, 'Action', '4', 'Do this twice: Trash a card from your hand, then gain a card\ncosting exactly 1 more than it.', NULL, NULL, NULL, NULL, '2', NULL, NULL, '2', NULL),
(2720, 1, 0, 0, 0, 'Shop', 'Cornucopia & Guilds, 2E', 6, 'Action', '3', '+1 Card+1You may play an Action\ncard from your hand that you don\'t have a copy of in play.', 'P1?', '+1', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2721, 1, 0, 0, 0, 'Soothsayer', 'Cornucopia & Guilds', 9, 'Action - Attack', '5', 'Gain a Gold. Each other player gains a Curse, and if they did,\ndraws a card.', NULL, NULL, NULL, NULL, NULL, NULL, '1', '1', NULL),
(2722, 1, 0, 0, 0, 'Stonemason', 'Cornucopia & Guilds', 9, 'Action', NULL, 'Trash a card from your hand. Gain 2 cards each costing\nless than it.Overpay: Gain 2 Action cards each costing the\namount overpaid.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '2*2?', NULL),
(2723, 0, 1, 0, 0, 'Taxman', 'Cornucopia & Guilds, 1E', 9, 'Action - Attack', '4', 'You may trash a Treasure from your hand. Each other player with\n5 or more cards in hand discards a copy of it (or reveals they\ncan\'t). Gain a Treasure onto your deck costing up to 3 more than it.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(2724, 0, 1, 0, 0, 'Tournament', 'Cornucopia & Guilds, 1E', 6, 'Action', '4', '+1 ActionEach player may reveal a Province from their\nhand. If you do, discard it and gain any Prize (from the Prize\npile) or a Duchy, onto your deck. If no-one else does, +1 Card\nand +1.', '+1', '!+1', NULL, '!+1', NULL, NULL, NULL, '!1', '!(3 ?)'),
(2725, 1, 0, 0, 0, 'Young Witch', 'Cornucopia & Guilds', 6, 'Action - Attack', '4', '+2 CardsDiscard 2 cards. Each other player gains a\nCurse unless they reveal a Bane from their hand.Setup: Add an extra\nKingdom card pile costing 2 or 3 to the Supply. Its\ncards are Banes.', NULL, '+2, -2', NULL, NULL, NULL, NULL, '!1', NULL, NULL),
(2726, 0, 1, 0, 0, 'Bag of Gold', 'Cornucopia & Guilds, 1E', NULL, 'Action - Prize', NULL, '+1 ActionGain a Gold onto your deck.<i>(This is not in the\nSupply.)</i>', '+1', NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2727, 0, 1, 0, 0, 'Coronet', 'Cornucopia & Guilds, 2E', NULL, 'Action - Treasure - Reward', NULL, 'You may play a non-Reward Action from your hand twice.You may\nplay a non-Reward Treasure from your hand twice.<i>(This is not in\nthe Supply.)</i>', 'P2?', NULL, NULL, 'P2?', NULL, NULL, NULL, NULL, NULL),
(2728, 0, 1, 0, 0, 'Courser', 'Cornucopia & Guilds, 2E', NULL, 'Action - Reward', NULL, 'Choose two different options: +2 Cards; +2 Actions;\n+2; gain 4\nSilvers.<i>(This is not in the Supply.)</i>', '+2?', '+2?', NULL, '+2?', NULL, NULL, NULL, '4?', NULL),
(2729, 0, 1, 0, 0, 'Demesne', 'Cornucopia & Guilds, 2E', NULL, 'Action - Victory - Reward', NULL, '+2 Actions+2 BuysGain a Gold.Worth 1  per Gold you\nhave.<i>(This is not in the Supply.)</i>', '+2', NULL, '+2', NULL, NULL, NULL, NULL, '1', 'X '),
(2730, 0, 1, 0, 0, 'Diadem', 'Cornucopia & Guilds, 1E', NULL, 'Treasure - Prize', NULL, '2+1 per unused Action you\nhave (Action, not Action card).<i>(This is not in the\nSupply.)</i>', NULL, NULL, NULL, '+2, +', NULL, NULL, NULL, NULL, NULL),
(2731, 0, 1, 0, 0, 'Followers', 'Cornucopia & Guilds, 1E', NULL, 'Action - Attack - Prize', NULL, '+2 CardsGain an Estate. Each other player gains a Curse\nand discards down to 3 cards in hand.<i>(This is not in the\nSupply.)</i>', NULL, '+2', NULL, NULL, NULL, NULL, '1', '1', '(1 )'),
(2732, 0, 1, 0, 0, 'Housecarl', 'Cornucopia & Guilds, 2E', NULL, 'Action - Reward', NULL, '+1 Card per differently named Action card you have in\nplay.<i>(This is not in the Supply.)</i>', NULL, '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2733, 0, 1, 0, 0, 'Huge Turnip', 'Cornucopia & Guilds, 2E', NULL, 'Treasure - Reward', NULL, '+2 Coffers+1 per Coffers you\nhave.<i>(This is not in the Supply.)</i>', NULL, NULL, NULL, '+2, +', NULL, NULL, NULL, NULL, NULL),
(2734, 0, 1, 0, 0, 'Princess', 'Cornucopia & Guilds, 1E', NULL, 'Action - Prize', NULL, '+1 BuyThis turn, cards cost 2 less.<i>(This is not\nin the Supply.)</i>', NULL, NULL, '+1', 'R2', NULL, NULL, NULL, NULL, NULL),
(2735, 0, 1, 0, 0, 'Renown', 'Cornucopia & Guilds, 2E', NULL, 'Action - Reward', NULL, '+1 BuyThis turn, cards cost 2 less.<i>(This is not\nin the Supply.)</i>', NULL, NULL, '+1', 'R2', NULL, NULL, NULL, NULL, NULL),
(2736, 0, 1, 0, 0, 'Trusty Steed', 'Cornucopia & Guilds, 1E', NULL, 'Action - Prize', NULL, 'Choose two: +2 Cards; or +2 Actions; or +; or gain\n4 Silvers and put your deck into your discard pile. The\nchoices must be different.<i>(This is not in the Supply.)</i>', '+2?', '+2?', NULL, '+2?', NULL, NULL, NULL, '4?', NULL),
(2737, 0, 1, 0, 0, 'Berserker', 'Hinterlands, 2E', 7, 'Action - Attack', '5', 'Gain a card costing less than this. Each other player discards\ndown to 3 cards in hand.When you gain this, if you have an\nAction in play, play this.', '*Pself', NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2738, 1, 0, 0, 0, 'Border Village', 'Hinterlands', 7, 'Action', '6', '+1 Card+2 ActionsWhen you gain this, gain a cheaper\ncard.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, '*1', NULL),
(2739, 1, 0, 0, 0, 'Cache', 'Hinterlands, 1E', 7, 'Treasure', '5', '3When you gain this,\ngain 2 Coppers.', NULL, NULL, NULL, '+3', NULL, NULL, '2Self', '2', NULL),
(2740, 1, 0, 0, 0, 'Cartographer', 'Hinterlands', 7, 'Action', '5', '+1 Card+1 ActionLook at the top 4 cards of your\ndeck. Discard any number of them, then put the rest back in any\norder.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2741, 0, 1, 0, 0, 'Cauldron', 'Hinterlands, 2E', 7, 'Treasure - Attack', '5', '2+1 BuyThe third\ntime you gain an Action this turn, each other player gains a\nCurse.', NULL, NULL, '+1', '+2', NULL, NULL, '!1', NULL, NULL),
(2742, 1, 0, 0, 0, 'Crossroads', 'Hinterlands', 7, 'Action', '2', 'Reveal your hand. +1 Card per Victory card revealed. If\nthis is the first time you played a Crossroads this turn,\n+3 Actions.', '!+3', '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2743, 1, 0, 0, 0, 'Develop', 'Hinterlands', 7, 'Action', '3', 'Trash a card from your hand. Gain two cards onto your deck,\nwith one costing exactly 1 more than it, and one\ncosting exactly 1 less than it, in\neither order.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '2', NULL),
(2744, 1, 0, 0, 0, 'Duchess', 'Hinterlands, 1E', 7, 'Action', '2', '+2Each player (including\nyou) looks at the top card of their deck and may discard it.In\ngames using this, when you gain a Duchy, you may gain a\nDuchess.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, '**Self', NULL),
(2745, 1, 0, 0, 0, 'Embassy', 'Hinterlands, 1E', 7, 'Action', '5', '+5 CardsDiscard 3 cards.When you gain this, each\nother player gains a Silver.', NULL, '+5, -3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2746, 1, 0, 0, 0, 'Farmland', 'Hinterlands', 7, 'Victory', '6', '2 When you gain this,\ntrash a card from your hand and gain a non-Farmland card costing\nexactly 2 more than it.', NULL, NULL, NULL, NULL, '*1', NULL, NULL, '*1', '2 '),
(2747, 1, 0, 0, 0, 'Fool\'s Gold', 'Hinterlands', 7, 'Treasure - Reaction', '2', 'If this is the first time you played a Fool\'s Gold this turn,\n+1, otherwise +.When another player\ngains a Province, you may trash this from your hand, to gain a Gold\nonto your deck.', NULL, NULL, NULL, '+1, !+3', '**Self', NULL, NULL, '**1', NULL),
(2748, 0, 1, 0, 0, 'Guard Dog', 'Hinterlands, 2E', 7, 'Action - Reaction', '3', '+2 CardsIf you have 5 or fewer cards in hand,\n+2 Cards.When another player plays an Attack, you may first\nplay this from your hand.', '**Pself', '+2, !+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2749, 1, 0, 0, 0, 'Haggler', 'Hinterlands', 7, 'Action', '5', '+2This turn, when you\ngain a card, if you bought it, gain a cheaper non-Victory\ncard.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, 'X', NULL),
(2750, 1, 0, 0, 0, 'Highway', 'Hinterlands', 7, 'Action', '5', '+1 Card+1 ActionThis turn, cards cost 1 less.', '+1', '+1', NULL, 'R1', NULL, NULL, NULL, NULL, NULL),
(2751, 1, 0, 0, 0, 'Ill-Gotten Gains', 'Hinterlands, 1E', 7, 'Treasure', '5', '1You may gain a Copper\nto your hand.When you gain this, each other player gains a\nCurse.', NULL, '(+1)?', NULL, '+1', NULL, NULL, 'Self?*1', '1?', NULL),
(2752, 1, 0, 0, 0, 'Inn', 'Hinterlands', 7, 'Action', '5', '+2 Cards+2 ActionsDiscard 2 cards.When you gain\nthis, reveal any number of Action cards from your discard pile and\nshuffle them into your deck.', '+2', '+2, -2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2753, 1, 0, 0, 0, 'Jack of All Trades', 'Hinterlands', 7, 'Action', '4', 'Gain a Silver. Look at the top card of your deck; you may\ndiscard it. Draw until you have 5 cards in hand. You may trash\na non-Treasure card from your hand.', NULL, '=5', NULL, NULL, '1?', NULL, NULL, '1', NULL),
(2754, 1, 0, 0, 0, 'Mandarin', 'Hinterlands, 1E', 7, 'Action', '5', '+3Put a card from your\nhand onto your deck.When you gain this, put all Treasures you have\nin play onto your deck in any order.', NULL, '-1', NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(2755, 1, 0, 0, 0, 'Margrave', 'Hinterlands', 7, 'Action - Attack', '5', '+3 Cards+1 BuyEach other player draws a card, then\ndiscards down to 3 cards in hand.', NULL, '+3', '+1', NULL, NULL, NULL, NULL, NULL, NULL);
INSERT INTO `parsed_cards` (`id`, `added`, `skip`, `en_added`, `no_finnish`, `name`, `expansion`, `exp_id`, `type`, `cost`, `text`, `act_villager`, `draws`, `buys`, `coins_coffer`, `trash_ret`, `exile`, `junk`, `gain`, `victorypts`) VALUES
(2756, 1, 0, 0, 0, 'Noble Brigand', 'Hinterlands, 1E', 7, 'Action - Attack', '4', '+1Each other player\nreveals the top 2 cards of their deck, trashes a revealed\nSilver or Gold you choose, discards the rest, and gains a Copper if\nthey didn\'t reveal a Treasure. You gain the trashed cards.When you\nbuy this, do its attack.', NULL, NULL, NULL, '+1', NULL, NULL, '1?*1?', 'X*X', NULL),
(2757, 1, 0, 0, 0, 'Nomad Camp', 'Hinterlands, 1E', 7, 'Action', '4', '+1 Buy+2This is gained onto\nyour deck (instead of to your discard pile).', NULL, NULL, '+1', '+2', NULL, NULL, NULL, NULL, NULL),
(2758, 0, 1, 0, 0, 'Nomads', 'Hinterlands, 2E', 7, 'Action', '4', '+1 Buy+2When you gain or trash\nthis, +2.', NULL, NULL, '+1', '+2*+2', NULL, NULL, NULL, NULL, NULL),
(2759, 1, 0, 0, 0, 'Oasis', 'Hinterlands', 7, 'Action', '3', '+1 Card+1 Action+1Discard a card.', '+1', '+1, -1', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2760, 1, 0, 0, 0, 'Oracle', 'Hinterlands, 1E', 7, 'Action - Attack', '3', 'Each player (including you) reveals the top 2 cards of\ntheir deck, and discards them or puts them back, your choice (they\nchoose the order). Then, +2 Cards.', NULL, '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2761, 1, 0, 0, 0, 'Scheme', 'Hinterlands', 7, 'Action', '3', '+1 Card+1 ActionThis turn, you may put one of your\nAction cards onto your deck when you discard it from play.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2762, 1, 0, 0, 0, 'Silk Road', 'Hinterlands, 1E', 7, 'Victory', '4', 'Worth 1  for every\n4 Victory cards you have (round down).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2763, 0, 1, 0, 0, 'Souk', 'Hinterlands, 2E', 7, 'Action', '5', '+1 Buy+7–1 per card in your hand\n(you can\'t go below ).When you gain this,\ntrash up to 2 cards from your hand.', NULL, NULL, '+1', '+7, -', '*0-2?', NULL, NULL, NULL, NULL),
(2764, 1, 0, 0, 0, 'Spice Merchant', 'Hinterlands', 7, 'Action', '4', 'You may trash a Treasure from your hand to choose one:\n+2 Cards and +1 Action; or +1 Buy and +2.', '!+1?', '!+2?', '!+1?', '!+2?', '1?', NULL, NULL, NULL, NULL),
(2765, 1, 0, 0, 0, 'Stables', 'Hinterlands', 7, 'Action', '5', 'You may discard a Treasure, for +3 Cards and\n+1 Action.', '!+1', '-1?, !+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2766, 1, 0, 0, 0, 'Trader', 'Hinterlands', 7, 'Action - Reaction', '4', 'Trash a card from your hand. Gain a Silver per 1 it costs.When you\ngain a card, you may reveal this from your hand, to exchange the\ncard for a Silver.', NULL, NULL, NULL, NULL, '1', NULL, NULL, 'X', NULL),
(2767, 0, 1, 0, 0, 'Trail', 'Hinterlands, 2E', 7, 'Action - Reaction', '4', '+1 Card+1 ActionWhen you gain, trash, or discard\nthis, other than in Clean-up, you may play it.', '+1*Pself', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2768, 1, 0, 0, 0, 'Tunnel', 'Hinterlands', 7, 'Victory - Reaction', '3', '2 When you discard this\nother than during Clean-up, you may reveal it to gain a Gold.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!1', '2 '),
(2769, 0, 1, 0, 0, 'Weaver', 'Hinterlands, 2E', 7, 'Action - Reaction', '4', 'Gain two Silvers or a card costing up to 4.When you discard this\nother than in Clean-up, you may play it.', '*Pself', NULL, NULL, NULL, NULL, NULL, NULL, '1-2?', NULL),
(2770, 0, 1, 0, 0, 'Wheelwright', 'Hinterlands, 2E', 7, 'Action', '5', '+1 Card+1 ActionYou may discard a card to gain an\nAction card costing as much as it or less.', '+1', '+1, -1?', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(2771, 0, 1, 0, 0, 'Witch\'s Hut', 'Hinterlands, 2E', 7, 'Action - Attack', '5', '+4 CardsDiscard 2 cards, revealed. If they\'re both\nActions, each other player gains a Curse.', NULL, '+4, -2', NULL, NULL, NULL, NULL, '!1', NULL, NULL),
(2772, 1, 0, 0, 0, 'Altar', 'Dark Ages', 8, 'Action', '6', 'Trash a card from your hand. Gain a card costing up to\n5.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '1', NULL),
(2773, 1, 0, 0, 0, 'Armory', 'Dark Ages', 8, 'Action', '4', 'Gain a card onto your deck costing up to 4.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2774, 1, 0, 0, 0, 'Band of Misfits', 'Dark Ages', 8, 'Action - Command', '5', 'Play a non-Command non-Duration Action card from the Supply\nthat costs less than this, leaving it there.', '(P1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2775, 1, 0, 0, 0, 'Bandit Camp', 'Dark Ages', 8, 'Action', '5', '+1 Card+2 ActionsGain a Spoils.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(2776, 1, 0, 0, 0, 'Beggar', 'Dark Ages', 8, 'Action - Reaction', '2', 'Gain 3 Coppers to your hand.When another player plays an Attack\ncard, you may first discard this to gain 2 Silvers, putting\none onto your deck.', NULL, '(+3)', NULL, '(+3)', NULL, NULL, '3Self', '3*2', NULL),
(2777, 1, 0, 0, 0, 'Catacombs', 'Dark Ages', 8, 'Action', '5', 'Look at the top 3 cards of your deck. Choose one: Put them\ninto your hand; or discard them and +3 Cards.When you trash\nthis, gain a cheaper card.', NULL, '+3', NULL, NULL, NULL, NULL, NULL, '*1', NULL),
(2778, 1, 0, 0, 0, 'Count', 'Dark Ages', 8, 'Action', '5', 'Choose one: Discard 2 cards; or put a card from your hand\nonto your deck; or gain a Copper.Choose one: +3; or trash your hand;\nor gain a Duchy.', NULL, '-1?, -2?', NULL, '+3?', 'X?', NULL, 'Self?', '1?, 1?', '(3 ?)'),
(2779, 1, 0, 0, 0, 'Counterfeit', 'Dark Ages', 8, 'Treasure', '5', '1+1 BuyYou may\nplay a non-Duration Treasure from your hand twice. Trash it.', NULL, NULL, '+1', '+1, P2?', '!1', NULL, NULL, NULL, NULL),
(2780, 1, 0, 0, 0, 'Cultist', 'Dark Ages', 8, 'Action - Attack - Looter', '5', '+2 CardsEach other player gains a Ruins. You may play a\nCultist from your hand.When you trash this, +3 Cards.', 'P1?', '+2*+3', NULL, NULL, NULL, NULL, '1', NULL, NULL),
(2781, 1, 0, 0, 0, 'Dame Anna', 'Dark Ages', 8, 'Action - Attack - Knight', '5', 'You may trash up to 2 cards from your hand. Each other\nplayer reveals the top 2 cards of their deck, trashes one of\nthem costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', NULL, NULL, NULL, NULL, '0-2?, !Self', NULL, NULL, NULL, NULL),
(2782, 0, 1, 0, 0, 'Dame Josephine', 'Dark Ages', 8, 'Action - Attack - Knight - Victory', '5', 'Each other player reveals the top 2 cards of their deck,\ntrashes one of them costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.2 ', NULL, NULL, NULL, NULL, '!Self', NULL, NULL, NULL, '2 '),
(2783, 0, 1, 0, 0, 'Dame Molly', 'Dark Ages', 8, 'Action - Attack - Knight', '5', '+2 ActionsEach other player reveals the top 2 cards\nof their deck, trashes one of them costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', '+2', NULL, NULL, NULL, '!Self', NULL, NULL, NULL, NULL),
(2784, 0, 1, 0, 0, 'Dame Natalie', 'Dark Ages', 8, 'Action - Attack - Knight', '5', 'You may gain a card costing up to 3. Each other player\nreveals the top 2 cards of their deck, trashes one of them\ncosting from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', NULL, NULL, NULL, NULL, '!Self', NULL, NULL, '1?', NULL),
(2785, 0, 1, 0, 0, 'Dame Sylvia', 'Dark Ages', 8, 'Action - Attack - Knight', '5', '+2Each other player\nreveals the top 2 cards of their deck, trashes one of them\ncosting from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', NULL, NULL, NULL, '+2', '!Self', NULL, NULL, NULL, NULL),
(2786, 1, 0, 0, 0, 'Death Cart', 'Dark Ages', 8, 'Action - Looter', '4', 'You may trash this or an Action card from your hand, for\n+5.When you gain this,\ngain 2 Ruins.', NULL, NULL, NULL, '+5', '1 or Self?', NULL, '2Self', '2', NULL),
(2787, 1, 0, 0, 0, 'Feodum', 'Dark Ages', 8, 'Victory', '4', 'Worth 1  per 3 Silvers\nyou have (round down).When you trash this, gain\n3 Silvers.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '*3', 'X '),
(2788, 1, 0, 0, 0, 'Forager', 'Dark Ages', 8, 'Action', '3', '+1 Action+1 BuyTrash a card from your hand, then\n+1 per differently named\nTreasure in the trash.', '+1', NULL, '+1', '+', '1', NULL, NULL, NULL, NULL),
(2789, 1, 0, 0, 0, 'Fortress', 'Dark Ages', 8, 'Action', '4', '+1 Card+2 ActionsWhen you trash this, put it into\nyour hand.', '+2', '+1*(+Self)', NULL, NULL, NULL, NULL, NULL, '*(1)', NULL),
(2790, 1, 0, 0, 0, 'Graverobber', 'Dark Ages', 8, 'Action', '5', 'Choose one: Gain a card from the trash costing from  to 6, onto your deck; or\ntrash an Action card from your hand and gain a card costing up to\n3 more than it.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '1?', NULL),
(2791, 1, 0, 0, 0, 'Hermit', 'Dark Ages', 8, 'Action', '3', 'Look through your discard pile. You may trash a non-Treasure\nfrom it or from your hand. Gain a card costing up to 3. At the end of your\nBuy phase this turn, if you didn\'t gain any cards in it, exchange\nthis for a Madman.', NULL, NULL, NULL, NULL, '1?, !*Self', NULL, NULL, '1, !1', NULL),
(2792, 1, 0, 0, 0, 'Hunting Grounds', 'Dark Ages', 8, 'Action', '6', '+4 CardsWhen you trash this, gain a Duchy or\n3 Estates.', NULL, '+4', NULL, NULL, NULL, NULL, NULL, '*1 or 3', '*(3 )'),
(2793, 1, 0, 0, 0, 'Ironmonger', 'Dark Ages', 8, 'Action', '4', '+1 Card+1 ActionReveal the top card of your deck; you\nmay discard it. Either way, if it is an…Action card,\n+1 ActionTreasure card, +1Victory card,\n+1 Card', '+1, !+1', '+1, !+1', NULL, '!+1', NULL, NULL, NULL, NULL, NULL),
(2794, 1, 0, 0, 0, 'Junk Dealer', 'Dark Ages', 8, 'Action', '5', '+1 Card+1 Action+1Trash a card from your\nhand.', '+1', '+1', NULL, '+1', '1', NULL, NULL, NULL, NULL),
(2795, 1, 0, 0, 0, 'Marauder', 'Dark Ages', 8, 'Action - Attack - Looter', '4', 'Gain a Spoils. Each other player gains a Ruins.', NULL, NULL, NULL, NULL, NULL, NULL, '1', '1', NULL),
(2796, 1, 0, 0, 0, 'Market Square', 'Dark Ages', 8, 'Action - Reaction', '3', '+1 Card+1 Action+1 BuyWhen one of your cards is\ntrashed, you may discard this from your hand to gain a Gold.', '+1', '+1', '+1', NULL, NULL, NULL, NULL, '*1', NULL),
(2797, 1, 0, 0, 0, 'Mystic', 'Dark Ages', 8, 'Action', '5', '+1 Action+2Name a card, then\nreveal the top card of your deck. If you named it, put it into your\nhand.', '+1', '!+1', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2798, 1, 0, 0, 0, 'Pillage', 'Dark Ages', 8, 'Action - Attack', '5', 'Trash this. If you did, gain 2 Spoils, and each other\nplayer with 5 or more cards in hand reveals their hand and\ndiscards a card that you choose.', NULL, NULL, NULL, NULL, 'Self', NULL, NULL, '2', NULL),
(2799, 1, 0, 0, 0, 'Poor House', 'Dark Ages', 8, 'Action', '1', '+4Reveal your hand.\n–1 per Treasure card in\nyour hand. (You can\'t go below .)', NULL, NULL, NULL, '+4, -', NULL, NULL, NULL, NULL, NULL),
(2800, 1, 0, 0, 0, 'Procession', 'Dark Ages', 8, 'Action', '4', 'You may play a non-Duration Action card from your hand twice.\nTrash it. Gain an Action card costing exactly 1 more than it.', 'P2?', NULL, NULL, NULL, '!1', NULL, NULL, '!1', NULL),
(2801, 1, 0, 0, 0, 'Rats', 'Dark Ages', 8, 'Action', '4', '+1 Card+1 ActionGain a Rats. Trash a card from your\nhand other than a Rats (or reveal a hand of all Rats).When you\ntrash this, +1 Card.', '+1', '+1*1', NULL, NULL, '1', NULL, NULL, '1', NULL),
(2802, 1, 0, 0, 0, 'Rebuild', 'Dark Ages', 8, 'Action', '5', '+1 ActionName a card. Reveal cards from your deck until\nyou reveal a Victory card you did not name. Discard the rest, trash\nthe Victory card, and gain a Victory card costing up to  more than it.', '+1', NULL, NULL, NULL, '1', NULL, NULL, '1', NULL),
(2803, 1, 0, 0, 0, 'Rogue', 'Dark Ages', 8, 'Action - Attack', '5', '+2If there are any cards\nin the trash costing from 3 to 6, gain one of them.\nOtherwise, each other player reveals the top 2 cards of their\ndeck, trashes one of them costing from 3 to 6, and discards the\nrest.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, '!1', NULL),
(2804, 1, 0, 0, 0, 'Sage', 'Dark Ages', 8, 'Action', '3', '+1 ActionReveal cards from the top of your deck until you\nreveal one costing 3 or more. Put that\ncard into your hand and discard the rest.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2805, 1, 0, 0, 0, 'Scavenger', 'Dark Ages', 8, 'Action', '4', '+2You may put your deck\ninto your discard pile. Put a card from your discard pile onto your\ndeck.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2806, 0, 1, 0, 0, 'Sir Bailey', 'Dark Ages', 8, 'Action - Attack - Knight', '5', '+1 Card+1 ActionEach other player reveals the top\n2 cards of their deck, trashes one of them costing from\n3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', '+1', '+1', NULL, NULL, '!Self', NULL, NULL, NULL, NULL),
(2807, 0, 1, 0, 0, 'Sir Destry', 'Dark Ages', 8, 'Action - Attack - Knight', '5', '+2 CardsEach other player reveals the top 2 cards of\ntheir deck, trashes one of them costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', NULL, '+2', NULL, NULL, '!Self', NULL, NULL, NULL, NULL),
(2808, 0, 1, 0, 0, 'Sir Martin', 'Dark Ages', 8, 'Action - Attack - Knight', '4', '+2 BuysEach other player reveals the top 2 cards of\ntheir deck, trashes one of them costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', NULL, NULL, '+2', NULL, '!Self', NULL, NULL, NULL, NULL),
(2809, 0, 1, 0, 0, 'Sir Michael', 'Dark Ages', 8, 'Action - Attack - Knight', '5', 'Each other player discards down to 3 cards in hand. Each\nother player reveals the top 2 cards of their deck, trashes\none of them costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.', NULL, NULL, NULL, NULL, '!Self', NULL, NULL, NULL, NULL),
(2810, 0, 1, 0, 0, 'Sir Vander', 'Dark Ages', 8, 'Action - Attack - Knight', '5', 'Each other player reveals the top 2 cards of their deck,\ntrashes one of them costing from 3 to 6, and discards the\nrest. If a Knight is trashed by this, trash this.When you trash\nthis, gain a Gold.', NULL, NULL, NULL, NULL, '!Self', NULL, NULL, '*1', NULL),
(2811, 1, 0, 0, 0, 'Squire', 'Dark Ages', 8, 'Action', '2', '+1Choose one:\n+2 Actions; or +2 Buys; or gain a Silver.When you trash\nthis, gain an Attack card.', '+2?', NULL, '+2?', '+1', NULL, NULL, NULL, '1?*1', NULL),
(2812, 1, 0, 0, 0, 'Storeroom', 'Dark Ages', 8, 'Action', '3', '+1 BuyDiscard any number of cards, then draw that many.\nThen discard any number of cards for +1 each.', NULL, '-Y, +Y, -X', '+1', '+', NULL, NULL, NULL, NULL, NULL),
(2813, 1, 0, 0, 0, 'Urchin', 'Dark Ages', 8, 'Action - Attack', '3', '+1 Card+1 ActionEach other player discards down to\n4 cards in hand.When you play another Attack card with this in\nplay, you may first trash this, to gain a Mercenary.', '+1', '+1', NULL, NULL, '!Self?', NULL, NULL, '!1', NULL),
(2814, 1, 0, 0, 0, 'Vagrant', 'Dark Ages', 8, 'Action', '2', '+1 Card+1 ActionReveal the top card of your deck. If\nit\'s a Curse, Ruins, Shelter, or Victory card, put it into your\nhand.', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2815, 1, 0, 0, 0, 'Wandering Minstrel', 'Dark Ages', 8, 'Action', '4', '+1 Card+2 ActionsReveal the top 3 cards of your\ndeck. Put the Action cards back in any order and discard the\nrest.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2816, 0, 0, 0, 0, 'Abandoned Mine', 'Dark Ages', 8, 'Action - Ruins', NULL, '+1', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2817, 0, 0, 0, 0, 'Ruined Library', 'Dark Ages', 8, 'Action - Ruins', NULL, '+1 Card', NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2818, 0, 0, 0, 0, 'Ruined Market', 'Dark Ages', 8, 'Action - Ruins', NULL, '+1 Buy', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2819, 0, 0, 0, 0, 'Ruined Village', 'Dark Ages', 8, 'Action - Ruins', NULL, '+1 Action', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2820, 0, 0, 0, 0, 'Survivors', 'Dark Ages', 8, 'Action - Ruins', NULL, 'Look at the top 2 cards of your deck. Discard them or put\nthem back in any order.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2821, 0, 0, 0, 0, 'Hovel', 'Dark Ages', 8, 'Reaction - Shelter', '1', 'When you gain a Victory card, you may trash this from your\nhand.', NULL, NULL, NULL, NULL, '*Self', NULL, NULL, NULL, NULL),
(2822, 0, 0, 0, 0, 'Madman', 'Dark Ages', 8, 'Action', NULL, '+2 ActionsReturn this to the Madman pile. If you do,\n+1 Card per card in your hand.<i>(This is not in the\nSupply.)</i>', '+2', '+X', NULL, NULL, 'Self', NULL, NULL, NULL, NULL),
(2823, 0, 0, 0, 0, 'Mercenary', 'Dark Ages', 8, 'Action - Attack', NULL, 'You may trash 2 cards from your hand. If you did,\n+2 Cards, +2, and each other\nplayer discards down to 3 cards in hand.<i>(This is not in the\nSupply.)</i>', NULL, '!+2', NULL, '!+2', '2?', NULL, NULL, NULL, NULL),
(2824, 0, 0, 0, 0, 'Necropolis', 'Dark Ages', 8, 'Action - Shelter', '1', '+2 Actions', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2825, 0, 0, 0, 0, 'Overgrown Estate', 'Dark Ages', 8, 'Victory - Shelter', '1', '0 When you trash this,\n+1 Card.', NULL, '*+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2826, 0, 0, 0, 0, 'Spoils', 'Dark Ages', 8, 'Treasure', NULL, '3When you play this,\nreturn it to the Spoils pile.<i>(This is not in the\nSupply.)</i>', NULL, NULL, NULL, '+3', 'Self', NULL, NULL, NULL, NULL),
(2827, 1, 0, 0, 0, 'Amulet', 'Adventures', 10, 'Action - Duration', '3', 'Now and at the start of your next turn, choose one: +; or trash a card from\nyour hand; or gain a Silver.', NULL, NULL, NULL, '+1?, N +1?', '1?, N 1?', NULL, NULL, '1?, N 1?', NULL),
(2828, 1, 0, 0, 0, 'Artificer', 'Adventures', 10, 'Action', '5', '+1 Card+1 Action+1Discard any number of\ncards. You may gain a card onto your deck, costing exactly\n1 per card\ndiscarded.', '+1', '+1, -X', NULL, '+1', NULL, NULL, NULL, '1?', NULL),
(2829, 1, 0, 0, 0, 'Bridge Troll', 'Adventures', 10, 'Action - Duration - Attack', '5', 'Each other player takes their –1 token. On this turn\nand your next turn, cards cost 1 less. Now and at the\nstart of your next turn: +1 Buy.', NULL, NULL, '+1, N +1', 'R1, N R1', NULL, NULL, NULL, NULL, NULL),
(2830, 1, 0, 0, 0, 'Caravan Guard', 'Adventures', 10, 'Action - Duration - Reaction', '3', '+1 Card+1 ActionAt the start of your next turn,\n+1.When another player\nplays an Attack card, you may first play this from your hand.', '+1**Pself', '+1', NULL, 'N +1', NULL, NULL, NULL, NULL, NULL),
(2831, 1, 0, 0, 0, 'Coin of the Realm', 'Adventures', 10, 'Treasure - Reserve', '2', '1Put this on your\nTavern mat.After you play an Action card, you may call this, for\n+2 Actions.', 'C+2', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2832, 1, 0, 0, 0, 'Distant Lands', 'Adventures', 10, 'Action - Reserve - Victory', '5', 'Put this on your Tavern mat.Worth 4  if on your Tavern mat\nat the end of the game (otherwise worth 0 ).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!4 '),
(2833, 1, 0, 0, 0, 'Dungeon', 'Adventures', 10, 'Action - Duration', '3', '+1 ActionNow and at the start of your next turn:\n+2 Cards, then discard 2 cards.', '+1', '+2, -2, N +2, N -2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2834, 1, 0, 0, 0, 'Duplicate', 'Adventures', 10, 'Action - Reserve', '4', 'Put this on your Tavern mat.When you gain a card costing up to\n6, you may call this,\nto gain a copy of that card.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'C1', NULL),
(2835, 1, 0, 0, 0, 'Gear', 'Adventures', 10, 'Action - Duration', '3', '+2 CardsSet aside up to 2 cards from your hand face\ndown (under this). At the start of your next turn, put them into\nyour hand.', NULL, '+2, - 0-2?(X), N +X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2836, 1, 0, 0, 0, 'Giant', 'Adventures', 10, 'Action - Attack', '5', 'Turn your Journey token over (it starts face up). Then if it\'s\nface down, +1. If it\'s face up,\n+5, and each other\nplayer reveals the top card of their deck, trashes it if it costs\nfrom 3 to 6, and otherwise\ndiscards it and gains a Curse.', NULL, NULL, NULL, '+1, !+4', NULL, NULL, '!1', NULL, NULL),
(2837, 1, 0, 0, 0, 'Guide', 'Adventures', 10, 'Action - Reserve', '3', '+1 Card+1 ActionPut this on your Tavern mat.At the\nstart of your turn, you may call this, to discard your hand and\ndraw 5 cards.', '+1', '+1, C-X?, !+5', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2838, 1, 0, 0, 0, 'Haunted Woods', 'Adventures', 10, 'Action - Duration - Attack', '5', 'At the start of your next turn: +3 Cards.Until then, when\nany other player gains a card they bought, they put their hand onto\ntheir deck in any order.', NULL, 'N +3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2839, 1, 0, 0, 0, 'Hireling', 'Adventures', 10, 'Action - Duration', '6', 'At the start of each of your turns for the rest of the game:\n+1 Card.', NULL, 'N... +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2840, 1, 0, 0, 0, 'Lost City', 'Adventures', 10, 'Action', '5', '+2 Cards+2 ActionsWhen you gain this, each other\nplayer draws a card.', '+2', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2841, 1, 0, 0, 0, 'Magpie', 'Adventures', 10, 'Action', '4', '+1 Card+1 ActionReveal the top card of your deck. If\nit\'s a Treasure, put it into your hand. If it\'s an Action or\nVictory card, gain a Magpie.', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(2842, 1, 0, 0, 0, 'Messenger', 'Adventures', 10, 'Action', '4', '+1 Buy+2You may put your deck\ninto your discard pile.When this is the first card you gain in your\nBuy phase, gain a card costing up to 4, and each other\nplayer gains a copy of it.', NULL, NULL, '+1', '+2', NULL, NULL, NULL, '!1', NULL),
(2843, 1, 0, 0, 0, 'Miser', 'Adventures', 10, 'Action', '4', 'Choose one: Put a Copper from your hand onto your Tavern mat;\nor +1 per Copper on your\nTavern mat.', NULL, NULL, NULL, '+?', '1?', NULL, NULL, NULL, NULL),
(2844, 1, 0, 0, 0, 'Page', 'Adventures', 10, 'Action - Traveller', '2', '+1 Card+1 ActionWhen you discard this from play, you\nmay exchange it for a Treasure Hunter.', '+1', '+1', NULL, NULL, 'Self?', NULL, NULL, NULL, NULL),
(2845, 1, 0, 0, 0, 'Peasant', 'Adventures', 10, 'Action - Traveller', '2', '+1 Buy+1When you discard this\nfrom play, you may exchange it for a Soldier.', NULL, NULL, '+1', '+1', 'Self?', NULL, NULL, NULL, NULL),
(2846, 1, 0, 0, 0, 'Port', 'Adventures', 10, 'Action', '4', '+1 Card+2 ActionsWhen you gain this, gain another\nPort (that doesn\'t come with another).', '+2', '+1', NULL, NULL, NULL, NULL, NULL, '*1', NULL),
(2847, 1, 0, 0, 0, 'Ranger', 'Adventures', 10, 'Action', '4', '+1 BuyTurn your Journey token over (it starts face up).\nThen if it\'s face up, +5 Cards.', NULL, '!+5', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2848, 1, 0, 0, 0, 'Ratcatcher', 'Adventures', 10, 'Action - Reserve', '2', '+1 Card+1 ActionPut this on your Tavern mat.At the\nstart of your turn, you may call this to trash a card from your\nhand.', '+1', '+1', NULL, NULL, 'C1', NULL, NULL, NULL, NULL),
(2849, 1, 0, 0, 0, 'Raze', 'Adventures', 10, 'Action', '2', '+1 ActionTrash this or a card from your hand. Look at one\ncard from the top of your deck per 1 the trashed card\ncosts. Put one of them into your hand and discard the rest.', '+1', '!+1', NULL, NULL, '1 or Self?', NULL, NULL, NULL, NULL),
(2850, 1, 0, 0, 0, 'Relic', 'Adventures', 10, 'Treasure - Attack', '5', '2Each other player puts\ntheir –1 Card token on their deck.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2851, 1, 0, 0, 0, 'Royal Carriage', 'Adventures', 10, 'Action - Reserve', '5', '+1 ActionPut this on your Tavern mat.After you play an\nAction card, if it\'s still in play, you may call this, to replay\nthat Action.', '+1C P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2852, 1, 0, 0, 0, 'Storyteller', 'Adventures', 10, 'Action', '5', '+1 ActionPlay up to 3 Treasures from your hand. Then\n+1 Card, and pay all of your  for +1 Card per\n1 you paid.', '+1', '+1, +X', NULL, '-', NULL, NULL, NULL, NULL, NULL),
(2853, 1, 0, 0, 0, 'Swamp Hag', 'Adventures', 10, 'Action - Duration - Attack', '5', 'At the start of your next turn: +3.Until then, when any\nother player gains a card they bought, they gain a Curse.', NULL, NULL, NULL, 'N +3', NULL, NULL, '!1', NULL, NULL),
(2854, 1, 0, 0, 0, 'Transmogrify', 'Adventures', 10, 'Action - Reserve', '4', '+1 ActionPut this on your Tavern mat.At the start of your\nturn, you may call this, to trash a card from your hand, and gain a\ncard to your hand costing up to 1 more than it.', '+1', 'C(+1)', NULL, NULL, 'C1', NULL, NULL, 'C1', NULL),
(2855, 1, 0, 0, 0, 'Treasure Trove', 'Adventures', 10, 'Treasure', '5', '2Gain a Gold and a\nCopper.', NULL, NULL, NULL, '+2', NULL, NULL, 'Self', '2', NULL),
(2856, 1, 0, 0, 0, 'Wine Merchant', 'Adventures', 10, 'Action - Reserve', '5', '+1 Buy+4Put this on your\nTavern mat.At the end of your Buy phase, if you have at least\n2 unspent, you may\ndiscard this from your Tavern mat.', NULL, NULL, '+1', '+4, C-2?', NULL, NULL, NULL, NULL, NULL),
(2857, 0, 1, 0, 0, 'Champion', 'Adventures', 10, 'Action - Duration', NULL, '+1 ActionFor the rest of the game, when another player\nplays an Attack, it doesn\'t affect you, and when you play an Action\ncard, you first get +1 Action.<i>(This is not in the\nSupply.)</i>', '+1, **+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2858, 0, 1, 0, 0, 'Disciple', 'Adventures', 10, 'Action - Traveller', NULL, 'You may play an Action card from your hand twice. Gain a copy\nof it.When you discard this from play, you may exchange it for a\nTeacher.<i>(This is not in the Supply.)</i>', 'P2?', NULL, NULL, NULL, 'Self?', NULL, NULL, '!1', NULL),
(2859, 0, 1, 0, 0, 'Fugitive', 'Adventures', 10, 'Action - Traveller', NULL, '+2 Cards+1 ActionDiscard a card.When you discard this\nfrom play, you may exchange it for a Disciple.<i>(This is not in\nthe Supply.)</i>', '+1', '+2, -1', NULL, NULL, 'Self?', NULL, NULL, NULL, NULL),
(2860, 0, 1, 0, 0, 'Hero', 'Adventures', 10, 'Action - Traveller', NULL, '+2Gain a Treasure.When\nyou discard this from play, you may exchange it for a\nChampion.<i>(This is not in the Supply.)</i>', NULL, NULL, NULL, '+2', 'Self?', NULL, NULL, '1', NULL),
(2861, 0, 1, 0, 0, 'Soldier', 'Adventures', 10, 'Action - Attack - Traveller', NULL, '+2+1 per other Attack you\nhave in play. Each other player with 4 or more cards in hand\ndiscards a card.When you discard this from play, you may exchange\nit for a Fugitive. <i>(This is not in the Supply.)</i>', NULL, NULL, NULL, '+2, +', 'Self?', NULL, NULL, NULL, NULL),
(2862, 0, 1, 0, 0, 'Teacher', 'Adventures', 10, 'Action - Reserve', NULL, 'Put this on your Tavern mat.At the start of your turn, you may\ncall this, to move your +1 Card, +1 Action, +1 Buy,\nor +1 token to an Action\nSupply pile you have no tokens on. (When you play a card from that\npile, you first get that bonus.)<i>(This is not in the\nSupply.)</i>', 'C**+1?', 'C**+1?', 'C**+1?', 'C**+1?', NULL, NULL, NULL, NULL, NULL),
(2863, 0, 1, 0, 0, 'Treasure Hunter', 'Adventures', 10, 'Action - Traveller', NULL, '+1 Action+1Gain a Silver per card\nthe player to your right gained in their last turn.When you discard\nthis from play, you may exchange it for a Warrior.<i>(This is not\nin the Supply.)</i>', '+1', NULL, NULL, '+1', 'Self?', NULL, NULL, 'X', NULL),
(2864, 0, 1, 0, 0, 'Warrior', 'Adventures', 10, 'Action - Attack - Traveller', NULL, '+2 CardsFor each Traveller you have in play (including\nthis), each other player discards the top card of their deck and\ntrashes it if it costs 3 or 4.When you discard this\nfrom play, you may exchange it for a Hero.<i>(This is not in the\nSupply.)</i>', NULL, '+2', NULL, NULL, 'Self?', NULL, NULL, NULL, NULL),
(2865, 1, 0, 0, 0, 'Alms', 'Adventures', 10, 'Event', NULL, 'Once per turn: If you have no Treasures in play, gain a card\ncosting up to 4.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(2866, 0, 0, 0, 0, 'Ball', 'Adventures', 10, 'Event', '5', 'Take your –1 token. Gain\n2 cards each costing up to 4.', NULL, NULL, NULL, '-1', NULL, NULL, NULL, '2', NULL),
(2867, 0, 0, 0, 0, 'Bonfire', 'Adventures', 10, 'Event', '3', 'Trash up to 2 Coppers you have in play.', NULL, NULL, NULL, NULL, '0-2?', NULL, NULL, NULL, NULL),
(2868, 0, 0, 0, 0, 'Borrow', 'Adventures', 10, 'Event', NULL, 'Once per turn: +1 Buy. If your –1 Card token isn\'t on\nyour deck, put it there and +1.', NULL, '(N -1)', '+1', '!+1', NULL, NULL, NULL, NULL, NULL),
(2869, 0, 0, 0, 0, 'Expedition', 'Adventures', 10, 'Event', '3', 'Draw 2 extra cards for your next hand.', NULL, '(N +2)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2870, 0, 0, 0, 0, 'Ferry', 'Adventures', 10, 'Event', '3', 'Move your –2 token to an Action\nSupply pile. (Cards from that pile cost 2 less on your\nturns.)', NULL, NULL, NULL, '**R2', NULL, NULL, NULL, NULL, NULL),
(2871, 0, 0, 0, 0, 'Inheritance', 'Adventures', 10, 'Event', '7', 'Once per game: Set aside a non-Command non-Duration Action card\nfrom the Supply costing up to 4. Move your Estate\ntoken to it. (During your turns, Estates are also Command Actions\nwith \"Play the card with your Estate token, leaving it\nthere.\")', '(P1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2872, 0, 0, 0, 0, 'Lost Arts', 'Adventures', 10, 'Event', '6', 'Move your +1 Action token to an Action Supply pile. (When\nyou play a card from that pile, you first get +1 Action.)', '**+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2873, 0, 0, 0, 0, 'Mission', 'Adventures', 10, 'Event', '4', 'Take an extra turn after this one (but not a 3rd turn in a\nrow), during which you can\'t buy cards.<i>(You can still buy\nEvents.)</i>', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2874, 0, 0, 0, 0, 'Pathfinding', 'Adventures', 10, 'Event', '8', 'Move your +1 Card token to an Action Supply pile. (When\nyou play a card from that pile, you first get +1 Card.)', NULL, '**+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2875, 0, 0, 0, 0, 'Pilgrimage', 'Adventures', 10, 'Event', '4', 'Once per turn: Turn your Journey token over (it starts face\nup); then if it\'s face up, choose up to 3 differently named\ncards you have in play and gain a copy of each.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!0-3?', NULL),
(2876, 0, 0, 0, 0, 'Plan', 'Adventures', 10, 'Event', '3', 'Move your Trashing token to an Action Supply pile. (When you\ngain a card from that pile, you may trash a card from your\nhand.)', NULL, NULL, NULL, NULL, '**1?', NULL, NULL, NULL, NULL),
(2877, 0, 0, 0, 0, 'Quest', 'Adventures', 10, 'Event', NULL, 'You may discard an Attack, two Curses, or six cards. If you do,\ngain a Gold.', NULL, '-X', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(2878, 0, 0, 0, 0, 'Raid', 'Adventures', 10, 'Event', '5', 'Gain a Silver per Silver you have in play. Each other player\nputs their –1 Card token on their deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(2879, 0, 0, 0, 0, 'Save', 'Adventures', 10, 'Event', '1', 'Once per turn: +1 Buy. Set aside a card from your hand,\nand put it into your hand at end of turn (after drawing).', NULL, '-1, (N +1)', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2880, 0, 0, 0, 0, 'Scouting Party', 'Adventures', 10, 'Event', '2', '+1 BuyLook at the top 5 cards of your deck. Discard\n3 and put the rest back in any order.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2881, 0, 0, 0, 0, 'Seaway', 'Adventures', 10, 'Event', '5', 'Gain an Action card costing up to 4. Move your\n+1 Buy token to its pile. (When you play a card from that\npile, you first get +1 Buy.)', NULL, NULL, '**+1', NULL, NULL, NULL, NULL, '1', NULL),
(2882, 0, 0, 0, 0, 'Trade', 'Adventures', 10, 'Event', '5', 'Trash up to 2 cards from your hand. Gain a Silver per card\nyou trashed.', NULL, NULL, NULL, NULL, '0-2?', NULL, NULL, '!0-2', NULL),
(2883, 0, 0, 0, 0, 'Training', 'Adventures', 10, 'Event', '6', 'Move your +1 token to an Action\nSupply pile. (When you play a card from that pile, you first get\n+1.)', NULL, NULL, NULL, '**+1', NULL, NULL, NULL, NULL, NULL),
(2884, 0, 0, 0, 0, 'Travelling Fair', 'Adventures', 10, 'Event', '2', '+2 BuysWhen you gain a card this turn, you may put it onto\nyour deck.', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL, NULL),
(2885, 1, 0, 1, 0, 'Archive', 'Empires', 11, 'Action - Duration', '5', '+1 ActionSet aside the top 3 cards of your deck face\ndown (you may look at them). Now and at the start of your next two\nturns, put one into your hand.', '+1', '+1, N +1, NN +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2886, 1, 0, 1, 0, 'Bustling Village', 'Empires', 11, 'Action', '5', '+1 Card+3 ActionsYou may reveal a Settlers from your\ndiscard pile and put it into your hand.', '+3', '+1, +1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2887, 1, 0, 1, 0, 'Capital', 'Empires', 11, 'Treasure', '5', '6+1 BuyWhen you\ndiscard this from play, +.', NULL, NULL, '+1', '+6, *+', NULL, NULL, NULL, NULL, NULL),
(2888, 1, 0, 1, 0, 'Catapult', 'Empires', 11, 'Action - Attack', '3', '+1Trash a card from your\nhand. If it costs 3 or more, each other\nplayer gains a Curse. If it\'s a Treasure, each other player\ndiscards down to 3 cards in hand.', NULL, NULL, NULL, '+1', '1', NULL, '!1', NULL, NULL),
(2889, 1, 0, 1, 0, 'Chariot Race', 'Empires', 11, 'Action', '3', '+1 Action+1 Card, revealing it. The player to your\nleft reveals the top card of their deck. If your card costs more,\n+1 and +1 .', '+1', '+1', NULL, '!+1', NULL, NULL, NULL, NULL, '!+1 '),
(2890, 1, 0, 1, 0, 'Charm', 'Empires', 11, 'Treasure', '5', 'Choose one: +1 Buy and +2; or the next time you\ngain a card this turn, you may also gain a differently named card\nwith the same cost.', NULL, NULL, '+1?', '+2?', NULL, NULL, NULL, '1?', NULL),
(2891, 1, 0, 1, 0, 'City Quarter', 'Empires', 11, 'Action', NULL, '+2 ActionsReveal your hand. +1 Card per Action card\nrevealed.', '+2', '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2892, 1, 0, 1, 0, 'Crown', 'Empires', 11, 'Action - Treasure', '5', 'If it\'s your Action phase, you may play an Action from your\nhand twice.If it\'s your Buy phase, you may play a Treasure from\nyour hand twice.', '!P2?', NULL, NULL, '!P2?', NULL, NULL, NULL, NULL, NULL),
(2893, 0, 0, 0, 0, 'Crumbling Castle', 'Empires', 11, 'Victory - Castle', '4', '1 When you gain or trash\nthis, +1  and gain a\nSilver.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '*1', '1 *+1 '),
(2894, 1, 0, 1, 0, 'Emporium', 'Empires', 11, 'Action', '5', '+1 Card+1 Action+1When you gain this, if\nyou have at least 5 Action cards in play, +2 .', '+1', '+1', NULL, '+1', NULL, NULL, NULL, NULL, '*+2 '),
(2895, 1, 0, 1, 0, 'Encampment', 'Empires', 11, 'Action', '2', '+2 Cards+2 ActionsYou may reveal a Gold or Plunder\nfrom your hand. If you do not, set this aside, and return it to its\npile at the start of Clean-up.', '+2', '+2', NULL, NULL, '!Self', NULL, NULL, NULL, NULL),
(2896, 1, 0, 1, 0, 'Enchantress', 'Empires', 11, 'Action - Attack -\nDuration', '3', 'Until your next turn, the first time each other player plays an\nAction card on their turn, they get +1 Card and +1 Action\ninstead of following its instructions.At the start of your next\nturn, +2 Cards', NULL, 'N +2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2897, 1, 0, 1, 0, 'Engineer', 'Empires', 11, 'Action', NULL, 'Gain a card costing up to 4. You may trash this.\nIf you do, gain a card costing up to 4.', NULL, NULL, NULL, NULL, 'Self?', NULL, NULL, '1, !1', NULL),
(2898, 1, 0, 1, 0, 'Farmers\' Market', 'Empires', 11, 'Action - Gathering', '3', '+1 BuyIf there are 4  or more on the\nFarmers\' Market pile, take them and trash this. Otherwise, add\n1  to the pile and then\n+1 per 1  on the pile.', NULL, NULL, '+1', '!+ 1-4', '!Self', NULL, NULL, NULL, '!+4 '),
(2899, 1, 0, 1, 0, 'Fortune', 'Empires', 11, 'Treasure', '8', '+1 BuyDouble your  if you haven\'t yet\nthis turn.When you gain this, gain a Gold per Gladiator you have in\nplay.', NULL, NULL, '+1', '!x2', NULL, NULL, NULL, '*X', NULL),
(2900, 1, 0, 1, 0, 'Forum', 'Empires', 11, 'Action', '5', '+3 Cards+1 ActionDiscard 2 cards.When you gain\nthis, +1 Buy.', '+1', '+3, -2', '*+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2901, 1, 0, 1, 0, 'Gladiator', 'Empires', 11, 'Action', '3', '+2Reveal a card from\nyour hand. The player to your left may reveal a copy from their\nhand. If they don\'t, +1 and trash a Gladiator\nfrom its pile.', NULL, NULL, NULL, '+2, !+1', '!1 Supply', NULL, NULL, NULL, NULL),
(2902, 0, 0, 0, 0, 'Grand Castle', 'Empires', 11, 'Victory - Castle', '9', '5 When you gain this,\nreveal your hand. +1  per Victory card in\nyour hand and/or in play.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '5 *+X '),
(2903, 1, 0, 1, 0, 'Groundskeeper', 'Empires', 11, 'Action', '5', '+1 Card+1 ActionThis turn, when you gain a Victory\ncard, +1 .', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, '+X '),
(2904, 0, 0, 0, 0, 'Haunted Castle', 'Empires', 11, 'Victory - Castle', '6', '2 When you gain this\nduring your turn, gain a Gold, and each other player with 5 or\nmore cards in hand puts 2 of them onto their deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '*1', '2 '),
(2905, 0, 0, 0, 0, 'Humble Castle', 'Empires', 11, 'Treasure - Victory - Castle', '3', '1Worth 1  per Castle you\nhave.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, 'X '),
(2906, 0, 0, 0, 0, 'King\'s Castle', 'Empires', 11, 'Victory - Castle', '10', 'Worth 2  per Castle you\nhave.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2907, 1, 0, 1, 0, 'Legionary', 'Empires', 11, 'Action - Attack', '5', '+3You may reveal a Gold\nfrom your hand. If you do, each other player discards down to\n2 cards in hand, then draws a card.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(2908, 0, 0, 0, 0, 'Opulent Castle', 'Empires', 11, 'Action - Victory - Castle', '7', 'Discard any number of Victory cards, revealed. +2 per card\ndiscarded.3 ', NULL, '-X', NULL, '+2xX', NULL, NULL, NULL, NULL, '3 '),
(2909, 1, 0, 1, 0, 'Overlord', 'Empires', 11, 'Action - Command', NULL, 'Play a non-Command non-Duration Action card from the Supply\ncosting up to 5, leaving it\nthere.', '(P1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2910, 1, 0, 1, 0, 'Patrician', 'Empires', 11, 'Action', '2', '+1 Card+1 ActionReveal the top card of your deck. If\nit costs 5 or more, put it into\nyour hand.', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2911, 1, 0, 1, 0, 'Plunder', 'Empires', 11, 'Treasure', '5', '2+1 ', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, '+1 '),
(2912, 1, 0, 1, 0, 'Rocks', 'Empires', 11, 'Treasure', '4', '1When you gain or trash\nthis: If it\'s your Buy phase, gain a Silver onto your deck,\notherwise gain a Silver to your hand.', NULL, '*(+1)', NULL, '+1', NULL, NULL, NULL, '*1', NULL),
(2913, 1, 0, 1, 0, 'Royal Blacksmith', 'Empires', 11, 'Action', NULL, '+5 CardsReveal your hand; discard the Coppers.', NULL, '+5, -X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2914, 1, 0, 1, 0, 'Sacrifice', 'Empires', 11, 'Action', '4', 'Trash a card from your hand.If it\'s an…Action card,\n+2 Cards and +2 ActionsTreasure card, +2Victory card,\n+2 ', '!+2', '!+2', NULL, '!+2', '1', NULL, NULL, NULL, '!+2 '),
(2915, 1, 0, 1, 0, 'Settlers', 'Empires', 11, 'Action', '2', '+1 Card+1 ActionYou may reveal a Copper from your\ndiscard pile and put it into your hand.', '+1', '+1, +1?', NULL, '(+1)?', NULL, NULL, NULL, NULL, NULL),
(2916, 0, 0, 0, 0, 'Small Castle', 'Empires', 11, 'Action - Victory - Castle', '5', 'Trash this or a Castle from your hand. If you do, gain a\nCastle.2 ', NULL, NULL, NULL, NULL, '1 or Self?', NULL, NULL, '1', '2 , (X )'),
(2917, 0, 0, 0, 0, 'Sprawling Castle', 'Empires', 11, 'Victory - Castle', '8', '4 When you gain this,\ngain a Duchy or 3 Estates.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '*1 or 3', '4 *(3 )'),
(2918, 1, 0, 1, 0, 'Temple', 'Empires', 11, 'Action - Gathering', '4', '+1 Trash from 1 to 3\ndifferently named cards from your hand. Add 1  to the Temple\npile.When you gain this, take the   from the Temple\npile.', NULL, NULL, NULL, NULL, '1-3', NULL, NULL, NULL, '+1 *+X '),
(2919, 1, 0, 1, 0, 'Villa', 'Empires', 11, 'Action', '4', '+2 Actions+1 Buy+1When you gain this,\nput it into your hand, +1 Action, and if it\'s your Buy phase\nreturn to your Action phase.', '+2*+1', '*(+Self)', '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(2920, 1, 0, 1, 0, 'Wild Hunt', 'Empires', 11, 'Action - Gathering', '5', 'Choose one: +3 Cards and add 1  to the Wild Hunt\npile; or gain an Estate, and if you do, take the   from the pile.', NULL, '+3?', NULL, NULL, NULL, NULL, NULL, '1?', '(1 ?), +X '),
(2921, 0, 0, 0, 0, 'Advance', 'Empires', 11, 'Event', NULL, 'You may trash an Action card from your hand. If you do, gain an\nAction card costing up to 6.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(2922, 0, 0, 0, 0, 'Annex', 'Empires', 11, 'Event', NULL, 'Look through your discard pile. Shuffle all but up to\n5 cards from it into your deck. Gain a Duchy.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', '(3 )'),
(2923, 0, 0, 0, 0, 'Banquet', 'Empires', 11, 'Event', '3', 'Gain 2 Coppers and a non-Victory card costing up to\n5.', NULL, NULL, NULL, NULL, NULL, NULL, '2Self', '3', NULL),
(2924, 0, 0, 0, 0, 'Conquest', 'Empires', 11, 'Event', '6', 'Gain 2 Silvers. +1  per Silver you\'ve\ngained this turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '2', '+X '),
(2925, 0, 0, 0, 0, 'Delve', 'Empires', 11, 'Event', '2', '+1 BuyGain a Silver.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, '1', NULL),
(2926, 0, 0, 0, 0, 'Dominate', 'Empires', 11, 'Event', NULL, 'Gain a Province. If you do, +9 .', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', '(6 ), !+9 '),
(2927, 0, 0, 0, 0, 'Donate', 'Empires', 11, 'Event', NULL, 'At the start of your next turn, first, put your deck and\ndiscard pile into your hand, trash any number of cards from it,\nthen shuffle the rest into your deck and draw 5 cards.', NULL, 'N -X, N +5', NULL, NULL, 'X', NULL, NULL, NULL, NULL),
(2928, 0, 0, 0, 0, 'Ritual', 'Empires', 11, 'Event', '4', 'Gain a Curse. If you do, trash a card from your hand.\n+1  per 1 it costs.', NULL, NULL, NULL, NULL, '1', NULL, 'Self', '1', '(-1 ), +X '),
(2929, 0, 0, 0, 0, 'Salt the Earth', 'Empires', 11, 'Event', '4', '+1 Trash a Victory card\nfrom the Supply.', NULL, NULL, NULL, NULL, '1 Supply', NULL, NULL, NULL, '+1 '),
(2930, 0, 0, 0, 0, 'Tax', 'Empires', 11, 'Event', '2', 'Add  to a Supply pile.Setup:\nAdd  to each Supply pile.\nWhen a player gains a card in their Buy phase, they take the\n from its pile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2931, 0, 0, 0, 0, 'Triumph', 'Empires', 11, 'Event', NULL, 'Gain an Estate. If you did, +1  per card you\'ve\ngained this turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', '(1 ), +X '),
(2932, 0, 0, 0, 0, 'Wedding', 'Empires', 11, 'Event', '4', '+1 Gain a Gold.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', '+1 '),
(2933, 0, 0, 0, 0, 'Windfall', 'Empires', 11, 'Event', '5', 'If your deck and discard pile are empty, gain\n3 Golds.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!3', NULL),
(2934, 0, 0, 0, 0, 'Aqueduct', 'Empires', 11, 'Landmark', NULL, 'When you gain a Treasure, move 1  from its pile to\nthis. When you gain a Victory card, take the   from this.Setup: Put\n8  on the Silver and\nGold piles.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+X '),
(2935, 0, 0, 0, 0, 'Arena', 'Empires', 11, 'Landmark', NULL, 'At the start of your Buy phase, you may discard an Action card.\nIf you do, take 2  from here.Setup: Put\n6  here per player.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+2 '),
(2936, 0, 0, 0, 0, 'Bandit Fort', 'Empires', 11, 'Landmark', NULL, 'When scoring, -2  for each Silver and\neach Gold you have.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-X '),
(2937, 0, 0, 0, 0, 'Basilica', 'Empires', 11, 'Landmark', NULL, 'When you gain a card in your Buy phase, if you have  or more, take\n2  from here.Setup: Put\n6  here per player.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+2 '),
(2938, 0, 0, 0, 0, 'Baths', 'Empires', 11, 'Landmark', NULL, 'When you end your turn without having gained a card, take\n2  from here.Setup: Put\n6  here per player.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+2 '),
(2939, 0, 0, 0, 0, 'Battlefield', 'Empires', 11, 'Landmark', NULL, 'When you gain a Victory card, take 2  from here.Setup: Put\n6  here per player.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+2 '),
(2940, 0, 0, 0, 0, 'Colonnade', 'Empires', 11, 'Landmark', NULL, 'When you gain an Action card in your Buy phase, if you have a\ncopy of it in play, take 2  from here.Setup: Put\n6  here per player.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+2 '),
(2941, 0, 0, 0, 0, 'Defiled Shrine', 'Empires', 11, 'Landmark', NULL, 'When you gain an Action, move 1  from its pile to\nthis. When you gain a Curse in your Buy phase, take the   from this.Setup: Put\n2  on each non-Gathering\nAction Supply pile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+X '),
(2942, 0, 0, 0, 0, 'Fountain', 'Empires', 11, 'Landmark', NULL, 'When scoring, 15  if you have at least\n10 Coppers.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!15 '),
(2943, 0, 0, 0, 0, 'Keep', 'Empires', 11, 'Landmark', NULL, 'When scoring, 5  per differently named\nTreasure you have, that you have more copies of than each other\nplayer, or tied for most.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2944, 0, 0, 0, 0, 'Labyrinth', 'Empires', 11, 'Landmark', NULL, 'When you gain a 2nd card in one of your turns, take 2  from here.Setup: Put\n6  here per player.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+2 '),
(2945, 0, 0, 0, 0, 'Mountain Pass', 'Empires', 11, 'Landmark', NULL, 'When you are the first player to gain a Province, each player\nbids once, up to , ending with you. High\nbidder gets +8  and takes the\n they bid.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!+8 '),
(2946, 0, 0, 0, 0, 'Museum', 'Empires', 11, 'Landmark', NULL, 'When scoring, 2  per differently named\ncard you have.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2947, 0, 0, 0, 0, 'Obelisk', 'Empires', 11, 'Landmark', NULL, 'When scoring, 2  per card you have\nfrom the chosen pile.Setup: Choose a random Action Supply\npile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2948, 0, 0, 0, 0, 'Orchard', 'Empires', 11, 'Landmark', NULL, 'When scoring, 4  per differently named\nAction card you have 3 or more copies of.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2949, 0, 0, 0, 0, 'Palace', 'Empires', 11, 'Landmark', NULL, 'When scoring, 3  per set you have of\nCopper - Silver - Gold.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2950, 0, 0, 0, 0, 'Tomb', 'Empires', 11, 'Landmark', NULL, 'When you trash a card, +1 .', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '+1 '),
(2951, 0, 0, 0, 0, 'Tower', 'Empires', 11, 'Landmark', NULL, 'When scoring, 1  per non-Victory card\nyou have from an empty Supply pile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2952, 0, 0, 0, 0, 'Triumphal Arch', 'Empires', 11, 'Landmark', NULL, 'When scoring, 3  per copy you have of\nthe 2nd most common Action card among your cards (if it\'s a tie,\ncount either).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(2953, 0, 0, 0, 0, 'Wall', 'Empires', 11, 'Landmark', NULL, 'When scoring, -1  per card you have\nafter the first 15.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-X '),
(2954, 0, 0, 0, 0, 'Wolf Den', 'Empires', 11, 'Landmark', NULL, 'When scoring, -3  per card you have\nexactly one copy of.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-X '),
(2955, 1, 0, 0, 0, 'Bard', 'Nocturne', 12, 'Action - Fate', '4', '+2Receive a Boon.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2956, 1, 0, 0, 0, 'Blessed Village', 'Nocturne', 12, 'Action - Fate', '4', '+1 Card+2 ActionsWhen you gain this, take a Boon.\nReceive it now or at the start of your next turn.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2957, 1, 0, 0, 0, 'Cemetery', 'Nocturne', 12, 'Victory', '4', '2 When you gain this,\ntrash up to 4 cards from your hand.<i>Heirloom: Haunted\nMirror</i>', NULL, NULL, NULL, NULL, '*0-4', NULL, NULL, NULL, '2 '),
(2958, 1, 0, 0, 0, 'Changeling', 'Nocturne', 12, 'Night', '3', 'Trash this. Gain a copy of a card you have in play.In games\nusing this, when you gain a card costing 3 or more, you may\nexchange it for a Changeling.', NULL, NULL, NULL, NULL, 'Self', NULL, NULL, '1*Self', NULL),
(2959, 1, 0, 0, 0, 'Cobbler', 'Nocturne', 12, 'Night - Duration', '5', 'At the start of your next turn, gain a card to your hand\ncosting up to 4.', NULL, '(+1)', NULL, NULL, NULL, NULL, NULL, 'N 1', NULL),
(2960, 1, 0, 0, 0, 'Conclave', 'Nocturne', 12, 'Action', '4', '+2You may play an Action\ncard from your hand that you don\'t have a copy of in play. If you\ndo, +1 Action.', 'P1?, +1?', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(2961, 1, 0, 0, 0, 'Crypt', 'Nocturne', 12, 'Night - Duration', '5', 'Set aside any number of non-Duration Treasures you have in\nplay, face down (under this). While any remain, at the start of\neach of your turns, put one of them into your hand.', NULL, 'N +1, NN +1 ...', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2962, 1, 0, 0, 0, 'Cursed Village', 'Nocturne', 12, 'Action - Doom', '5', '+2 ActionsDraw until you have 6 cards in hand.When\nyou gain this, receive a Hex.', '+2', '=6', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2963, 1, 0, 0, 0, 'Den of Sin', 'Nocturne', 12, 'Night - Duration', '5', 'At the start of your next turn, +2 Cards.This is gained to\nyour hand (instead of your discard pile).', NULL, '*(+Self)N +2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2964, 1, 0, 0, 0, 'Devil\'s Workshop', 'Nocturne', 12, 'Night', '4', 'If the number of cards you\'ve gained this turn is:2+, gain an\nImp;1, gain a card costing up to 4;0, gain a Gold.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL);
INSERT INTO `parsed_cards` (`id`, `added`, `skip`, `en_added`, `no_finnish`, `name`, `expansion`, `exp_id`, `type`, `cost`, `text`, `act_villager`, `draws`, `buys`, `coins_coffer`, `trash_ret`, `exile`, `junk`, `gain`, `victorypts`) VALUES
(2965, 1, 0, 0, 0, 'Druid', 'Nocturne', 12, 'Action - Fate', '2', '+1 Buy Receive one of the set-aside Boons (leaving it\nthere).Setup: Set aside the top 3 Boons face up.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(2966, 1, 0, 0, 0, 'Exorcist', 'Nocturne', 12, 'Night', '4', 'Trash a card from your hand. Gain a cheaper Spirit from one of\nthe Spirit piles.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '1', NULL),
(2967, 1, 0, 0, 0, 'Faithful Hound', 'Nocturne', 12, 'Action - Reaction', '2', '+2 CardsWhen you discard this other than during Clean-up,\nyou may set it aside, and put it into your hand at end of\nturn.', NULL, '+2(N +Self)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2968, 1, 0, 0, 0, 'Fool', 'Nocturne', 12, 'Action - Fate', '3', 'If you aren\'t the player with Lost in the Woods: take it, take\n3 Boons, and receive the Boons in any order.<i>Heirloom: Lucky\nCoin</i>', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2969, 1, 0, 0, 0, 'Ghost Town', 'Nocturne', 12, 'Night - Duration', '3', 'At the start of your next turn, +1 Card and\n+1 Action.This is gained to your hand (instead of your discard\npile).', 'N +1', '*(+Self)N +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2970, 1, 0, 0, 0, 'Guardian', 'Nocturne', 12, 'Night - Duration', '2', 'At the start of your next turn, +1. Until then, when\nanother player plays an Attack card, it doesn\'t affect you. This is\ngained to your hand (instead of your discard pile).', NULL, '*(+Self)', NULL, 'N +1', NULL, NULL, NULL, NULL, NULL),
(2971, 1, 0, 0, 0, 'Idol', 'Nocturne', 12, 'Treasure - Attack - Fate', '5', '2If you have an odd\nnumber of Idols in play (counting this), receive a Boon; otherwise,\neach other player gains a Curse.', NULL, NULL, NULL, '+2', NULL, NULL, '!1', NULL, NULL),
(2972, 1, 0, 0, 0, 'Leprechaun', 'Nocturne', 12, 'Action - Doom', '3', 'Gain a Gold. If you have exactly 7 cards in play, gain a\nWish. Otherwise, receive a Hex.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1, 1?', NULL),
(2973, 1, 0, 0, 0, 'Monastery', 'Nocturne', 12, 'Night', '2', 'For each card you\'ve gained this turn, you may trash a card\nfrom your hand or a Copper you have in play.', NULL, NULL, NULL, NULL, 'X', NULL, NULL, NULL, NULL),
(2974, 1, 0, 0, 0, 'Necromancer', 'Nocturne', 12, 'Action', '4', 'Choose a face up, non-Duration Action card in the trash. Turn\nit face down for the turn, and play it, leaving it there.Setup: Put\nthe 3 Zombies into the trash.', '(P1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2975, 1, 0, 0, 0, 'Night Watchman', 'Nocturne', 12, 'Night', '3', 'Look at the top 5 cards of your deck, discard any number,\nand put the rest back in any order.This is gained to your hand\n(instead of your discard pile).', NULL, '*(+Self)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2976, 1, 0, 0, 0, 'Pixie', 'Nocturne', 12, 'Action - Fate', '2', '+1 Card+1 ActionDiscard the top Boon. You may trash\nthis to receive that Boon twice.<i>Heirloom: Goat</i>', '+1', '+1', NULL, NULL, 'Self?', NULL, NULL, NULL, NULL),
(2977, 1, 0, 0, 0, 'Pooka', 'Nocturne', 12, 'Action', '5', 'You may trash a Treasure other than Cursed Gold from your hand,\nfor +4 Cards.<i>Heirloom: Cursed Gold</i>', NULL, '!+4', NULL, NULL, '1?', NULL, NULL, NULL, NULL),
(2978, 1, 0, 0, 0, 'Raider', 'Nocturne', 12, 'Night - Duration - Attack', '6', 'Each other player with 5 or more cards in hand discards a\ncopy of a card you have in play (or reveals they can\'t).At the\nstart of your next turn, +3.', NULL, NULL, NULL, 'N +3', NULL, NULL, NULL, NULL, NULL),
(2979, 1, 0, 0, 0, 'Sacred Grove', 'Nocturne', 12, 'Action - Fate', '5', '+1 Buy+3Receive a Boon. If it\ndoesn\'t give +1, each other player\nmay receive it.', NULL, NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(2980, 1, 0, 0, 0, 'Secret Cave', 'Nocturne', 12, 'Action - Duration', '3', '+1 Card+1 ActionYou may discard 3 cards. If you\ndid, then at the start of your next turn, +3.<i>Heirloom: Magic\nLamp</i>', '+1', '+1, -3?', NULL, '!N +3', NULL, NULL, NULL, NULL, NULL),
(2981, 1, 0, 0, 0, 'Shepherd', 'Nocturne', 12, 'Action', '4', '+1 ActionDiscard any number of Victory cards, revealing\nthem. +2 Cards per card discarded.<i>Heirloom:\nPasture</i>', '+1', '-X, +2X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2982, 1, 0, 0, 0, 'Skulk', 'Nocturne', 12, 'Action - Attack - Doom', '4', '+1 BuyEach other player receives the next Hex.When you\ngain this, gain a Gold.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, '*1', NULL),
(2983, 1, 0, 0, 0, 'Tormentor', 'Nocturne', 12, 'Action - Attack - Doom', '5', '+2If you have no other\ncards in play, gain an Imp. Otherwise, each other player receives\nthe next Hex.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, '!1', NULL),
(2984, 1, 0, 0, 0, 'Tracker', 'Nocturne', 12, 'Action - Fate', '2', '+1This turn, when you\ngain a card, you may put it onto your deck. Receive a\nBoon.<i>Heirloom: Pouch</i>', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(2985, 1, 0, 0, 0, 'Tragic Hero', 'Nocturne', 12, 'Action', '5', '+3 Cards+1 BuyIf you have 8 or more cards in\nhand (after drawing), trash this and gain a Treasure.', NULL, '+3', '+1', NULL, '!Self', NULL, NULL, '!1', NULL),
(2986, 1, 0, 0, 0, 'Vampire', 'Nocturne', 12, 'Night - Attack - Doom', '5', 'Each other player receives the next Hex. Gain a card costing up\nto 5 other than a\nVampire.Exchange this for a Bat.', NULL, NULL, NULL, NULL, 'Self', NULL, NULL, '2', NULL),
(2987, 1, 0, 0, 0, 'Werewolf', 'Nocturne', 12, 'Action - Night - Attack - Doom', '5', 'If it\'s your Night phase, each other player receives the next\nHex. Otherwise, +3 Cards.', NULL, '!+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2988, 0, 1, 0, 0, 'Bat', 'Nocturne', 12, 'Night', NULL, 'Trash up to 2 cards from your hand. If you trashed at\nleast one, exchange this for a Vampire.<i>(This is not in the\nSupply.)</i>', NULL, NULL, NULL, NULL, '0-2?, !Self', NULL, NULL, '!1', NULL),
(2989, 0, 1, 0, 0, 'Cursed Gold', 'Nocturne', 12, 'Treasure - Heirloom', '4', '3Gain a Curse.', NULL, NULL, NULL, '+3', NULL, NULL, 'Self', '1', NULL),
(2990, 0, 0, 0, 0, 'Ghost', 'Nocturne', 12, 'Night - Duration - Spirit', NULL, 'Reveal cards from your deck until you reveal an Action. Discard\nthe other cards and set aside the Action. At the start of your next\nturn, play it twice.<i>(This is not in the Supply.)</i>', 'N P2', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2991, 0, 0, 0, 0, 'Goat', 'Nocturne', 12, 'Treasure - Heirloom', '2', '1You may trash a card\nfrom your hand.', NULL, NULL, NULL, '+1', '1?', NULL, NULL, NULL, NULL),
(2992, 0, 0, 0, 0, 'Haunted Mirror', 'Nocturne', 12, 'Treasure - Heirloom', NULL, '1When you trash this,\nyou may discard an Action card, to gain a Ghost.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, '*1?', NULL),
(2993, 0, 0, 0, 0, 'Imp', 'Nocturne', 12, 'Action - Spirit', NULL, '+2 CardsYou may play an Action card from your hand that\nyou don\'t have a copy of in play.<i>(This is not in the\nSupply.)</i>', '!P1?', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2994, 0, 0, 0, 0, 'Lucky Coin', 'Nocturne', 12, 'Treasure - Heirloom', '4', '1Gain a Silver.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, '1', NULL),
(2995, 0, 0, 0, 0, 'Magic Lamp', 'Nocturne', 12, 'Treasure - Heirloom', NULL, '1If there are at least\n6 cards that you have exactly 1 copy of in play (counting\nthis), trash this. If you did, gain 3 Wishes.', NULL, NULL, NULL, '+1', '!Self', NULL, NULL, '!3', NULL),
(2996, 0, 0, 0, 0, 'Pasture', 'Nocturne', 12, 'Treasure - Victory - Heirloom', '2', '1Worth 1  per Estate you\nhave.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, 'X '),
(2997, 0, 0, 0, 0, 'Pouch', 'Nocturne', 12, 'Treasure - Heirloom', '2', '1+1 Buy', NULL, NULL, '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(2998, 0, 0, 0, 0, 'Will-o\'-Wisp', 'Nocturne', 12, 'Action - Spirit', NULL, '+1 Card+1 ActionReveal the top card of your deck. If\nit costs 2 or less, put it into\nyour hand.<i>(This is not in the Supply.)</i>', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(2999, 0, 0, 0, 0, 'Wish', 'Nocturne', 12, 'Action', NULL, '+1 ActionReturn this to its pile. If you did, gain a card\nto your hand costing up to 6.<i>(This is not in\nthe Supply.)</i>', '+1', '(+1)', NULL, NULL, 'Self', NULL, NULL, '!1', NULL),
(3000, 0, 0, 0, 0, 'Zombie Apprentice', 'Nocturne', 12, 'Action - Zombie', '3', 'You may trash an Action card from your hand for +3 Cards\nand +1 Action.', '!+1', '!+3', NULL, NULL, '1?', NULL, NULL, NULL, NULL),
(3001, 0, 0, 0, 0, 'Zombie Mason', 'Nocturne', 12, 'Action - Zombie', '3', 'Trash the top card of your deck. You may gain a card costing up\nto 1 more than it.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '1?', NULL),
(3002, 0, 0, 0, 0, 'Zombie Spy', 'Nocturne', 12, 'Action - Zombie', '3', '+1 Card+1 ActionLook at the top card of your deck.\nDiscard it or put it back.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3003, 0, 0, 0, 0, 'The Earth\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'You may discard a Treasure to gain a card costing up to\n4.', NULL, '-1?', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(3004, 0, 0, 0, 0, 'The Field\'s Gift', 'Nocturne', 12, 'Boon', NULL, '+1 Action+1<i>(Keep this until\nClean-up.)</i>', '+1', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(3005, 0, 0, 0, 0, 'The Flame\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'You may trash a card from your hand.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, NULL, NULL),
(3006, 0, 0, 0, 0, 'The Forest\'s Gift', 'Nocturne', 12, 'Boon', NULL, '+1 Buy+1<i>(Keep this until\nClean-up.)</i>', NULL, NULL, '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(3007, 0, 0, 0, 0, 'The Moon\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'Look through your discard pile. You may put a card from it onto\nyour deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3008, 0, 0, 0, 0, 'The Mountain\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'Gain a Silver.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3009, 0, 0, 0, 0, 'The River\'s Gift', 'Nocturne', 12, 'Boon', NULL, '+1 Card at the end of this turn.<i>(Keep this until\nClean-up.)</i>', NULL, '(N +1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3010, 0, 0, 0, 0, 'The Sea\'s Gift', 'Nocturne', 12, 'Boon', NULL, '+1 Card', NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3011, 0, 0, 0, 0, 'The Sky\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'You may discard 3 cards to gain a Gold.', NULL, '-3?', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(3012, 0, 0, 0, 0, 'The Sun\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'Look at the top 4 cards of your deck. Discard any number\nof them and put the rest back in any order.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3013, 0, 0, 0, 0, 'The Swamp\'s Gift', 'Nocturne', 12, 'Boon', NULL, 'Gain a Will-o\'-Wisp.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3014, 0, 0, 0, 0, 'The Wind\'s Gift', 'Nocturne', 12, 'Boon', NULL, '+2 CardsDiscard 2 cards.', NULL, '+2, -2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3015, 0, 0, 0, 0, 'Bad Omens', 'Nocturne', 12, 'Hex', NULL, 'Put your deck into your discard pile. Look through it and put\n2 Coppers from it onto your deck (or reveal you can\'t).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3016, 0, 0, 0, 0, 'Delusion', 'Nocturne', 12, 'Hex', NULL, 'If you don\'t have Deluded or Envious, take Deluded.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3017, 0, 0, 0, 0, 'Envy', 'Nocturne', 12, 'Hex', NULL, 'If you don\'t have Deluded or Envious, take Envious.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3018, 0, 0, 0, 0, 'Famine', 'Nocturne', 12, 'Hex', NULL, 'Reveal the top 3 cards of your deck. Discard the Actions.\nShuffle the rest into your deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3019, 0, 0, 0, 0, 'Fear', 'Nocturne', 12, 'Hex', NULL, 'If you have at least 5 cards in hand, discard an Action or\nTreasure (or reveal you can\'t).', NULL, '!-1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3020, 0, 0, 0, 0, 'Greed', 'Nocturne', 12, 'Hex', NULL, 'Gain a Copper onto your deck.', NULL, NULL, NULL, NULL, NULL, NULL, 'Self', '1', NULL),
(3021, 0, 0, 0, 0, 'Haunting', 'Nocturne', 12, 'Hex', NULL, 'If you have at least 4 cards in hand, put one of them onto\nyour deck.', NULL, '!-1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3022, 0, 0, 0, 0, 'Locusts', 'Nocturne', 12, 'Hex', NULL, 'Trash the top card of your deck. If it\'s Copper or Estate, gain\na Curse. Otherwise, gain a cheaper card that shares a type with\nit.', NULL, NULL, NULL, NULL, '1', NULL, '!Self', '1', '!(-1 )'),
(3023, 0, 0, 0, 0, 'Misery', 'Nocturne', 12, 'Hex', NULL, 'If this is your first Misery this game, take Miserable.\nOtherwise, flip it over to Twice Miserable.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-2 '),
(3024, 0, 0, 0, 0, 'Plague', 'Nocturne', 12, 'Hex', NULL, 'Gain a Curse to your hand.', NULL, '(+1)', NULL, NULL, NULL, NULL, 'Self', '1', '(-1 )'),
(3025, 0, 0, 0, 0, 'Poverty', 'Nocturne', 12, 'Hex', NULL, 'Discard down to 3 cards in hand.', NULL, '-X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3026, 0, 0, 0, 0, 'War', 'Nocturne', 12, 'Hex', NULL, 'Reveal cards from your deck until revealing one costing\n3 or 4. Trash it and discard\nthe rest.', NULL, NULL, NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3027, 0, 0, 0, 0, 'Deluded', 'Nocturne', 12, 'State', NULL, 'At the start of your Buy phase, return this, and you can\'t buy\nActions this turn.', NULL, NULL, '!(=0)', NULL, NULL, NULL, NULL, NULL, NULL),
(3028, 0, 0, 0, 0, 'Envious', 'Nocturne', 12, 'State', NULL, 'At the start of your Buy phase, return this, and Silver and\nGold make 1 this turn.', NULL, NULL, NULL, '(-)', NULL, NULL, NULL, NULL, NULL),
(3029, 0, 0, 0, 0, 'Lost in the Woods', 'Nocturne', 12, 'State', NULL, 'At the start of your turn, you may discard a card to receive a\nBoon.', NULL, '-1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3030, 0, 0, 0, 0, 'Miserable', 'Nocturne', 12, 'State', NULL, '-2 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-2 '),
(3031, 0, 0, 0, 0, 'Twice Miserable', 'Nocturne', 12, 'State', NULL, '-4 ', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, '-4 '),
(3032, 0, 0, 0, 0, 'Acting Troupe', 'Renaissance', 13, 'Action', '3', '+4 VillagersTrash this.', '+4', NULL, NULL, NULL, 'Self', NULL, NULL, NULL, NULL),
(3033, 0, 0, 0, 0, 'Border Guard', 'Renaissance', 13, 'Action', '2', '+1 ActionReveal the top 2 cards of your deck. Put one\ninto your hand and discard the other. If both were Actions, take\nthe Lantern or Horn.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3034, 0, 0, 0, 0, 'Cargo Ship', 'Renaissance', 13, 'Action - Duration', '3', '+2Once this turn, when\nyou gain a card, you may set it aside face up (on this). At the\nstart of your next turn, put it into your hand.', NULL, 'N (+1)', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3035, 0, 0, 0, 0, 'Ducat', 'Renaissance', 13, 'Treasure', '2', '+1 Coffers+1 BuyWhen you gain this, you may trash a\nCopper from your hand.', NULL, NULL, '+1', '+1', '*1', NULL, NULL, NULL, NULL),
(3036, 0, 0, 0, 0, 'Experiment', 'Renaissance', 13, 'Action', '3', '+2 Cards+1 ActionReturn this to its pile.When you\ngain this, gain another Experiment (that doesn\'t come with\nanother).', '+1', '+2', NULL, NULL, 'Self', NULL, NULL, '*1', NULL),
(3037, 0, 0, 0, 0, 'Flag Bearer', 'Renaissance', 13, 'Action', '4', '+2When you gain or trash\nthis, take the Flag.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3038, 0, 0, 0, 0, 'Hideout', 'Renaissance', 13, 'Action', '4', '+1 Card+2 ActionsTrash a card from your hand. If it\'s\na Victory card, gain a Curse.', '+2', '+1', NULL, NULL, '1', NULL, '!Self', '!1', NULL),
(3039, 0, 0, 0, 0, 'Improve', 'Renaissance', 13, 'Action', '3', '+2At the start of\nClean-up, you may trash an Action card you would discard from play\nthis turn, to gain a card costing exactly 1 more than it.', NULL, NULL, NULL, '+2', '*1?', NULL, NULL, '!1', NULL),
(3040, 0, 0, 0, 0, 'Inventor', 'Renaissance', 13, 'Action', '4', 'Gain a card costing up to 4, then cards cost\n1 less this turn.', NULL, NULL, NULL, 'R1', NULL, NULL, NULL, '1', NULL),
(3041, 0, 0, 0, 0, 'Lackeys', 'Renaissance', 13, 'Action', '2', '+2 CardsWhen you gain this, +2 Villagers.', '*+2', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3042, 0, 0, 0, 0, 'Mountain Village', 'Renaissance', 13, 'Action', '4', '+2 ActionsLook through your discard pile and put a card\nfrom it into your hand; if you can\'t, +1 Card.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3043, 0, 0, 0, 0, 'Old Witch', 'Renaissance', 13, 'Action - Attack', '5', '+3 CardsEach other player gains a Curse and may trash a\nCurse from their hand.', NULL, '+3', NULL, NULL, NULL, NULL, '1', NULL, NULL),
(3044, 0, 0, 0, 0, 'Patron', 'Renaissance', 13, 'Action - Reaction', '4', '+1 Villager+2When something causes\nyou to reveal this (using the word \"reveal\") in an Action phase,\n+1 Coffers.', '+1', NULL, NULL, '+2*+1', NULL, NULL, NULL, NULL, NULL),
(3045, 0, 0, 0, 0, 'Priest', 'Renaissance', 13, 'Action', '4', '+2Trash a card from your\nhand. For the rest of this turn, when you trash a card, +.', NULL, NULL, NULL, '+2, +', '1', NULL, NULL, NULL, NULL),
(3046, 0, 0, 0, 0, 'Recruiter', 'Renaissance', 13, 'Action', '5', '+2 CardsTrash a card from your hand. +1 Villager per\n1 it costs.', '+X', '+2', NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3047, 0, 0, 0, 0, 'Research', 'Renaissance', 13, 'Action - Duration', '4', '+1 ActionTrash a card from your hand. Per 1 it costs, set aside a\ncard from your deck face down (on this). At the start of your next\nturn, put those cards into your hand.', '+1', 'N +X', NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3048, 0, 0, 0, 0, 'Scepter', 'Renaissance', 13, 'Treasure - Command', '5', 'Choose one: +2; or replay an\nnon-Command Action card you played this turn that\'s still in\nplay.', 'P1?', NULL, NULL, '+2?', NULL, NULL, NULL, NULL, NULL),
(3049, 0, 0, 0, 0, 'Scholar', 'Renaissance', 13, 'Action', '5', 'Discard your hand. +7 Cards.', NULL, '-X, +7', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3050, 0, 0, 0, 0, 'Sculptor', 'Renaissance', 13, 'Action', '5', 'Gain a card to your hand costing up to 4. If it\'s a Treasure,\n+1 Villager.', '!+1', '(+1)', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3051, 0, 0, 0, 0, 'Seer', 'Renaissance', 13, 'Action', '5', '+1 Card+1 ActionReveal the top 3 cards of your\ndeck. Put the ones costing from 2 to 4 into your hand. Put\nthe rest back in any order.', '+1', '+1, !+ 0-3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3052, 0, 0, 0, 0, 'Silk Merchant', 'Renaissance', 13, 'Action', '4', '+2 Cards+1 BuyWhen you gain or trash this,\n+1 Coffers and +1 Villager.', '*+1', '+2', '+1', '*+1', NULL, NULL, NULL, NULL, NULL),
(3053, 0, 0, 0, 0, 'Spices', 'Renaissance', 13, 'Treasure', '5', '2+1 BuyWhen you\ngain this, +2 Coffers.', NULL, NULL, '+1', '+2*+2', NULL, NULL, NULL, NULL, NULL),
(3054, 0, 0, 0, 0, 'Swashbuckler', 'Renaissance', 13, 'Action', '5', '+3 CardsIf your discard pile has any cards in it:\n+1 Coffers, then if you have at least 4 Coffers tokens,\ntake the Treasure Chest.', NULL, '+3', NULL, '!+1', NULL, NULL, NULL, NULL, NULL),
(3055, 0, 0, 0, 0, 'Treasurer', 'Renaissance', 13, 'Action', '5', '+3Choose one: Trash a\nTreasure from your hand; or gain a Treasure from the trash to your\nhand; or take the Key.', NULL, '(+1)?', NULL, '+3', '1?', NULL, NULL, '1?', NULL),
(3056, 0, 0, 0, 0, 'Villain', 'Renaissance', 13, 'Action - Attack', '5', '+2 CoffersEach other player with 5 or more cards in\nhand discards one costing 2 or more (or reveals\nthey can\'t).', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3057, 0, 0, 0, 0, 'Academy', 'Renaissance', 13, 'Project', '5', 'When you gain an Action card, +1 Villager.', '**+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3058, 0, 0, 0, 0, 'Barracks', 'Renaissance', 13, 'Project', '6', 'At the start of your turn, +1 Action.', 'N... +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3059, 0, 0, 0, 0, 'Canal', 'Renaissance', 13, 'Project', '7', 'During your turns, cards cost 1 less.', NULL, NULL, NULL, 'R1, N... R1', NULL, NULL, NULL, NULL, NULL),
(3060, 0, 0, 0, 0, 'Capitalism', 'Renaissance', 13, 'Project', '5', 'During your turns, Actions with + amounts in their text\nare also Treasures.', '(PX)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3061, 0, 0, 0, 0, 'Cathedral', 'Renaissance', 13, 'Project', '3', 'At the start of your turn, trash a card from your hand.', NULL, NULL, NULL, NULL, 'N... 1', NULL, NULL, NULL, NULL),
(3062, 0, 0, 0, 0, 'Citadel', 'Renaissance', 13, 'Project', '8', 'The first time you play an Action card during each of your\nturns, replay it afterwards.', '**P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3063, 0, 0, 0, 0, 'City Gate', 'Renaissance', 13, 'Project', '3', 'At the start of your turn, +1 Card, then put a card from\nyour hand onto your deck.', NULL, 'N... +1 -1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3064, 0, 0, 0, 0, 'Crop Rotation', 'Renaissance', 13, 'Project', '6', 'At the start of your turn, you may discard a Victory card for\n+2 Cards.', NULL, 'N... -1? !+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3065, 0, 0, 0, 0, 'Exploration', 'Renaissance', 13, 'Project', '4', 'At the end of your Buy phase, if you didn\'t gain any cards\nduring it, +1 Coffers and +1 Villager.', '**+1', NULL, NULL, '**+1', NULL, NULL, NULL, NULL, NULL),
(3066, 0, 0, 0, 0, 'Fair', 'Renaissance', 13, 'Project', '4', 'At the start of your turn, +1 Buy.', NULL, NULL, 'N... +1', NULL, NULL, NULL, NULL, NULL, NULL),
(3067, 0, 0, 0, 0, 'Fleet', 'Renaissance', 13, 'Project', '5', 'After the game ends, there\'s an extra round of turns just for\nplayers with this.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3068, 0, 0, 0, 0, 'Guildhall', 'Renaissance', 13, 'Project', '5', 'When you gain a Treasure, +1 Coffers.', NULL, NULL, NULL, '**+1', NULL, NULL, NULL, NULL, NULL),
(3069, 0, 0, 0, 0, 'Innovation', 'Renaissance', 13, 'Project', '6', 'Once during each of your turns, when you gain an Action card,\nyou may play it.', '**P1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3070, 0, 0, 0, 0, 'Pageant', 'Renaissance', 13, 'Project', '3', 'At the end of your Buy phase, you may pay 1 for\n+1 Coffers.', NULL, NULL, NULL, 'N... -1? !+1', NULL, NULL, NULL, NULL, NULL),
(3071, 0, 0, 0, 0, 'Piazza', 'Renaissance', 13, 'Project', '5', 'At the start of your turn, reveal the top card of your deck. If\nit\'s an Action, play it.', 'N... !P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3072, 0, 0, 0, 0, 'Road Network', 'Renaissance', 13, 'Project', '5', 'When another player gains a Victory card, +1 Card.', NULL, '**+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3073, 0, 0, 0, 0, 'Sewers', 'Renaissance', 13, 'Project', '3', 'When you trash a card other than with this, you may trash a\ncard from your hand.', NULL, NULL, NULL, NULL, '**1?', NULL, NULL, NULL, NULL),
(3074, 0, 0, 0, 0, 'Silos', 'Renaissance', 13, 'Project', '4', 'At the start of your turn, discard any number of Coppers,\nrevealed, and draw that many cards.', NULL, 'N... -X +X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3075, 0, 0, 0, 0, 'Sinister Plot', 'Renaissance', 13, 'Project', '4', 'At the start of your turn, add a token here, or remove your\ntokens here for +1 Card each.', NULL, '+X?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3076, 0, 0, 0, 0, 'Star Chart', 'Renaissance', 13, 'Project', '3', 'When shuffling, you may pick one of the cards to go on\ntop.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3077, 0, 0, 0, 0, 'Flag', 'Renaissance', 13, 'Artifact', NULL, 'When drawing your hand, +1 Card.', NULL, '(N +1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3078, 0, 0, 0, 0, 'Horn', 'Renaissance', 13, 'Artifact', NULL, 'Once per turn, when you discard a Border Guard from play, you\nmay put it onto your deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3079, 0, 0, 0, 0, 'Key', 'Renaissance', 13, 'Artifact', NULL, 'At the start of your turn, +1.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(3080, 0, 0, 0, 0, 'Lantern', 'Renaissance', 13, 'Artifact', NULL, 'Border Guards you play reveal 3 cards and discard 2.\n(It takes all 3 being Actions to take the Horn.)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3081, 0, 0, 0, 0, 'Treasure Chest', 'Renaissance', 13, 'Artifact', NULL, 'At the start of your Buy phase, gain a Gold.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3082, 1, 0, 0, 0, 'Animal Fair', 'Menagerie', 14, 'Action', NULL, '+4+1 Buy per empty\nsupply pile.Instead of paying this card\'s cost, you may trash an\nAction card from your hand.', NULL, NULL, '+X', '+4, !Rself', NULL, NULL, NULL, NULL, NULL),
(3083, 1, 0, 0, 0, 'Barge', 'Menagerie', 14, 'Action - Duration', '5', 'Either now or at the start of your next turn, +3 Cards and\n+1 Buy.', NULL, '+3?, N +3?', '+1?, N +1?', NULL, NULL, NULL, NULL, NULL, NULL),
(3084, 1, 0, 0, 0, 'Black Cat', 'Menagerie', 14, 'Action - Attack - Reaction', '2', '+2 CardsIf it isn\'t your turn, each other player gains a\nCurse.When another player gains a Victory card, you may play this\nfrom your hand.', '**Pself', '+2', NULL, NULL, NULL, NULL, '!1', NULL, NULL),
(3085, 1, 0, 0, 0, 'Bounty Hunter', 'Menagerie', 14, 'Action', '4', '+1 ActionExile a card from your hand. If you didn\'t have a\ncopy of it in Exile, +3.', '+1', '-1', NULL, '!+3', NULL, '1', NULL, NULL, NULL),
(3086, 1, 0, 0, 0, 'Camel Train', 'Menagerie', 14, 'Action', '3', 'Exile a non-Victory card from the Supply.When you gain this,\nExile a Gold from the Supply.', NULL, NULL, NULL, NULL, NULL, '1 Supply*1 Supply', NULL, NULL, NULL),
(3087, 1, 0, 0, 0, 'Cardinal', 'Menagerie', 14, 'Action - Attack', '4', '+2Each other player\nreveals the top 2 cards of their deck, Exiles one costing from\n3 to 6, and discards the\nrest.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3088, 1, 0, 0, 0, 'Cavalry', 'Menagerie', 14, 'Action', '4', 'Gain 2 Horses.When you gain this, +2 Cards,\n+1 Buy, and if it\'s your Buy phase return to your Action\nphase.', NULL, '*+2', '*+1', NULL, NULL, NULL, NULL, '2', NULL),
(3089, 1, 0, 0, 0, 'Coven', 'Menagerie', 14, 'Action - Attack', '5', '+1 Action+2Each other player\nExiles a Curse from the Supply. If they can\'t, they discard their\nExiled Curses.', '+1', NULL, NULL, '+2', NULL, NULL, '!1', NULL, NULL),
(3090, 1, 0, 0, 0, 'Destrier', 'Menagerie', 14, 'Action', NULL, '+2 Cards+1 ActionDuring your turns, this costs\n1 less per card you\'ve\ngained this turn.', '+1', '+2', NULL, '!Rself', NULL, NULL, NULL, NULL, NULL),
(3091, 1, 0, 0, 0, 'Displace', 'Menagerie', 14, 'Action', '5', 'Exile a card from your hand. Gain a differently named card\ncosting up to 2 more than it.', NULL, '-1', NULL, NULL, NULL, '1', NULL, '1', NULL),
(3092, 1, 0, 0, 0, 'Falconer', 'Menagerie', 14, 'Action - Reaction', '5', 'Gain a card to your hand costing less than this.When any player\ngains a card with 2 or more types (Action, Attack, etc.), you\nmay play this from your hand.', NULL, '(+1)', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3093, 1, 0, 0, 0, 'Fisherman', 'Menagerie', 14, 'Action', NULL, '+1 Card+1 Action+1During your turns, if\nyour discard pile is empty, this costs 3 less.', '+1', '+1', NULL, '+1, !R3self', NULL, NULL, NULL, NULL, NULL),
(3094, 1, 0, 0, 0, 'Gatekeeper', 'Menagerie', 14, 'Action - Duration - Attack', '5', 'At the start of your next turn, +3. Until then, when\nanother player gains an Action or Treasure card they don\'t have an\nExiled copy of, they Exile it.', NULL, NULL, NULL, 'N +3', NULL, NULL, NULL, NULL, NULL),
(3095, 1, 0, 0, 0, 'Goatherd', 'Menagerie', 14, 'Action', '3', '+1 ActionYou may trash a card from your hand.+1 Card\nper card the player to your right trashed on their last turn.', '+1', '+X', NULL, NULL, '1?', NULL, NULL, NULL, NULL),
(3096, 1, 0, 0, 0, 'Groom', 'Menagerie', 14, 'Action', '4', 'Gain a card costing up to 4. If it\'s an…Action\ncard, gain a Horse;Treasure card, gain a Silver;Victory card,\n+1 Card and +1 Action.', '!+1', '!+1', NULL, NULL, NULL, NULL, NULL, '1, !1', NULL),
(3097, 1, 0, 0, 0, 'Hostelry', 'Menagerie', 14, 'Action', '4', '+1 Card+2 ActionsWhen you gain this, you may discard\nany number of Treasures, revealed, to gain that many Horses.', '+2', '+1*-X', NULL, NULL, NULL, NULL, NULL, '*X', NULL),
(3098, 1, 0, 0, 0, 'Hunting Lodge', 'Menagerie', 14, 'Action', '5', '+1 Card+2 ActionsYou may discard your hand for\n+5 Cards.', '+2', '+1, -X?, !+5', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3099, 1, 0, 0, 0, 'Kiln', 'Menagerie', 14, 'Action', '5', '+2The next time you play\na card this turn, you may first gain a copy of it.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, '1?', NULL),
(3100, 1, 0, 0, 0, 'Livery', 'Menagerie', 14, 'Action', '5', '+3This turn, when you\ngain a card costing 4 or more, gain a\nHorse.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, 'X', NULL),
(3101, 1, 0, 0, 0, 'Mastermind', 'Menagerie', 14, 'Action - Duration', '5', 'At the start of your next turn, you may play an Action card\nfrom your hand three times.', 'N P3?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3102, 1, 0, 0, 0, 'Paddock', 'Menagerie', 14, 'Action', '5', '+2Gain\n2 Horses.+1 Action per empty Supply pile.', '+X', NULL, NULL, '+2', NULL, NULL, NULL, '2', NULL),
(3103, 1, 0, 0, 0, 'Sanctuary', 'Menagerie', 14, 'Action', '5', '+1 Card+1 Action+1 BuyYou may Exile a card from\nyour hand.', '+1', '+1, -1?', '+1', NULL, NULL, '1?', NULL, NULL, NULL),
(3104, 1, 0, 0, 0, 'Scrap', 'Menagerie', 14, 'Action', '3', 'Trash a card from your hand. Choose a different thing per\n1 it costs:\n+1 Card; +1 Action; +1 Buy; +1; gain a Silver; gain\na Horse.', '+1?', '+1?', '+1?', '+1?', '1', NULL, NULL, '0-2?', NULL),
(3105, 1, 0, 0, 0, 'Sheepdog', 'Menagerie', 14, 'Action - Reaction', '3', '+2 CardsWhen you gain a card, you may play this from your\nhand.', '*Pself', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3106, 1, 0, 0, 0, 'Sleigh', 'Menagerie', 14, 'Action - Reaction', '2', 'Gain 2 Horses.When you gain a card, you may discard this,\nto put that card into your hand or onto your deck.', NULL, '*-Self? !(+1)?', NULL, NULL, NULL, NULL, NULL, '2', NULL),
(3107, 1, 0, 0, 0, 'Snowy Village', 'Menagerie', 14, 'Action', '3', '+1 Card+4 Actions+1 BuyIgnore any further\n+Actions you get this turn.', '+4, -X', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3108, 1, 0, 0, 0, 'Stockpile', 'Menagerie', 14, 'Treasure', '3', '3+1 BuyExile\nthis.', NULL, NULL, '+1', '+3', NULL, 'Self', NULL, NULL, NULL),
(3109, 1, 0, 0, 0, 'Supplies', 'Menagerie', 14, 'Treasure', '2', '1Gain a Horse onto your\ndeck.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, '1', NULL),
(3110, 1, 0, 0, 0, 'Village Green', 'Menagerie', 14, 'Action - Duration - Reaction', '4', 'Either now or at the start of your next turn, +1 Card and\n+2 Actions.When you discard this other than during Clean-up,\nyou may reveal it to play it.', '+2?, N +2?*Pself', '+1?, N +1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3111, 1, 0, 0, 0, 'Wayfarer', 'Menagerie', 14, 'Action', NULL, '+3 CardsYou may gain a Silver.This has the same cost as\nthe last other card gained this turn, if any.', NULL, '+3', NULL, '!Rself', NULL, NULL, NULL, '1?', NULL),
(3112, 0, 0, 0, 0, 'Horse', 'Menagerie', 14, 'Action', NULL, '+2 Cards+1 ActionReturn this to its pile.<i>(This is\nnot in the Supply.)</i>', '+1', '+2', NULL, NULL, 'Self', NULL, NULL, NULL, NULL),
(3113, 0, 0, 0, 0, 'Alliance', 'Menagerie', 14, 'Event', '10', 'Gain a Province, a Duchy, an Estate, a Gold, a Silver, and a\nCopper.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '6', '(10 )'),
(3114, 0, 0, 0, 0, 'Banish', 'Menagerie', 14, 'Event', '4', 'Exile any number of cards with the same name from your\nhand.', NULL, '-X', NULL, NULL, NULL, 'X', NULL, NULL, NULL),
(3115, 0, 0, 0, 0, 'Bargain', 'Menagerie', 14, 'Event', '4', 'Gain a non-Victory card costing up to 5. Each other player\ngains a Horse.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3116, 0, 0, 0, 0, 'Commerce', 'Menagerie', 14, 'Event', '5', 'Gain a Gold per differently named card you\'ve gained this\nturn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(3117, 0, 0, 0, 0, 'Delay', 'Menagerie', 14, 'Event', NULL, 'You may set aside an Action card from your hand. At the start\nof your next turn, play it.', '!N P1', '-1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3118, 0, 0, 0, 0, 'Demand', 'Menagerie', 14, 'Event', '5', 'Gain a Horse and a card costing up to 4, both onto your\ndeck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '2', NULL),
(3119, 0, 0, 0, 0, 'Desperation', 'Menagerie', 14, 'Event', NULL, 'Once per turn: You may gain a Curse. If you do, +1 Buy and\n+2.', NULL, NULL, '!+1', '!+2', NULL, NULL, 'Self?', '1?', NULL),
(3120, 0, 0, 0, 0, 'Enclave', 'Menagerie', 14, 'Event', '8', 'Gain a Gold. Exile a Duchy from the Supply.', NULL, NULL, NULL, NULL, NULL, '1 Supply', NULL, '1', '(3 )'),
(3121, 0, 0, 0, 0, 'Enhance', 'Menagerie', 14, 'Event', '3', 'You may trash a non-Victory card from your hand, to gain a card\ncosting up to 2 more than it.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(3122, 0, 0, 0, 0, 'Gamble', 'Menagerie', 14, 'Event', '2', '+1 BuyDiscard the top card of your deck. If it\'s an Action\nor Treasure, you may play it.', '!P1?', NULL, '+1', '!P1?', NULL, NULL, NULL, NULL, NULL),
(3123, 0, 0, 0, 0, 'Invest', 'Menagerie', 14, 'Event', '4', 'Exile an Action card from the Supply. While it\'s in Exile, when\nanother player gains or Invests in a copy of it,\n+2 Cards.', NULL, '**+2', NULL, NULL, NULL, '1 Supply', NULL, NULL, NULL),
(3124, 0, 0, 0, 0, 'March', 'Menagerie', 14, 'Event', '3', 'Look through your discard pile. You may play an Action card\nfrom it.', 'P1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3125, 0, 0, 0, 0, 'Populate', 'Menagerie', 14, 'Event', '10', 'Gain one card from each Action Supply pile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(3126, 0, 0, 0, 0, 'Pursue', 'Menagerie', 14, 'Event', '2', '+1 BuyName a card. Reveal the top 4 cards from your\ndeck. Put the matches back and discard the rest.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3127, 0, 0, 0, 0, 'Reap', 'Menagerie', 14, 'Event', '7', 'Gain a Gold, setting it aside. At the start of your next turn,\nplay it.', NULL, NULL, NULL, '(N +3)', NULL, NULL, NULL, '1', NULL),
(3128, 0, 0, 0, 0, 'Ride', 'Menagerie', 14, 'Event', '2', 'Gain a Horse.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3129, 0, 0, 0, 0, 'Seize the Day', 'Menagerie', 14, 'Event', '4', 'Once per game: Take an extra turn after this one.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3130, 0, 0, 0, 0, 'Stampede', 'Menagerie', 14, 'Event', '5', 'If you have 5 or fewer cards in play, gain 5 Horses onto\nyour deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '5', NULL),
(3131, 0, 0, 0, 0, 'Toil', 'Menagerie', 14, 'Event', '2', '+1 BuyYou may play an Action card from your hand.', 'P1?', NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3132, 0, 0, 0, 0, 'Transport', 'Menagerie', 14, 'Event', '3', 'Choose one: Exile an Action card from the Supply; or put an\nAction card you have in Exile onto your deck.', NULL, NULL, NULL, NULL, NULL, '1 Supply?', NULL, NULL, NULL),
(3133, 0, 0, 0, 0, 'Way of the Butterfly', 'Menagerie', 14, 'Way', NULL, 'You may return this to its pile to gain a card costing exactly\n1 more than it.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(3134, 0, 0, 0, 0, 'Way of the Camel', 'Menagerie', 14, 'Way', NULL, 'Exile a Gold from the Supply.', NULL, NULL, NULL, NULL, NULL, '1 Supply', NULL, NULL, NULL),
(3135, 0, 0, 0, 0, 'Way of the Chameleon', 'Menagerie', 14, 'Way', NULL, 'Follow this card\'s instructions; each time that would give you\n+Cards this turn, you get + instead, and\nvice-versa.', NULL, '-X, +Y', NULL, '+, -', NULL, NULL, NULL, NULL, NULL),
(3136, 0, 0, 0, 0, 'Way of the Frog', 'Menagerie', 14, 'Way', NULL, '+1 ActionWhen you discard this from play this turn, put it\nonto your deck.', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3137, 0, 0, 0, 0, 'Way of the Goat', 'Menagerie', 14, 'Way', NULL, 'Trash a card from your hand.', NULL, NULL, NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3138, 0, 0, 0, 0, 'Way of the Horse', 'Menagerie', 14, 'Way', NULL, '+2 Cards+1 ActionReturn this to its pile.', '+1', '+2', NULL, NULL, 'Self', NULL, NULL, NULL, NULL),
(3139, 0, 0, 0, 0, 'Way of the Mole', 'Menagerie', 14, 'Way', NULL, '+1 ActionDiscard your hand. +3 Cards.', '+1', '-X, +3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3140, 0, 0, 0, 0, 'Way of the Monkey', 'Menagerie', 14, 'Way', NULL, '+1 Buy+1', NULL, NULL, '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(3141, 0, 0, 0, 0, 'Way of the Mouse', 'Menagerie', 14, 'Way', NULL, 'Play the set-aside card, leaving it there.Setup: Set aside an\nunused non-Duration Action costing 2 or 3.', '(P1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3142, 0, 0, 0, 0, 'Way of the Mule', 'Menagerie', 14, 'Way', NULL, '+1 Action+1', '+1', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(3143, 0, 0, 0, 0, 'Way of the Otter', 'Menagerie', 14, 'Way', NULL, '+2 Cards', NULL, '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3144, 0, 0, 0, 0, 'Way of the Ox', 'Menagerie', 14, 'Way', NULL, '+2 Actions', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3145, 0, 0, 0, 0, 'Way of the Owl', 'Menagerie', 14, 'Way', NULL, 'Draw until you have 6 cards in hand.', NULL, '=6', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3146, 0, 0, 0, 0, 'Way of the Pig', 'Menagerie', 14, 'Way', NULL, '+1 Card+1 Action', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3147, 0, 0, 0, 0, 'Way of the Rat', 'Menagerie', 14, 'Way', NULL, 'You may discard a Treasure to gain a copy of this.', NULL, '-1', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3148, 0, 0, 0, 0, 'Way of the Seal', 'Menagerie', 14, 'Way', NULL, '+1This turn, when you\ngain a card, you may put it onto your deck.', NULL, NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(3149, 0, 0, 0, 0, 'Way of the Sheep', 'Menagerie', 14, 'Way', NULL, '+2', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3150, 0, 0, 0, 0, 'Way of the Squirrel', 'Menagerie', 14, 'Way', NULL, '+2 Cards at the end of this turn.', NULL, '(N +2)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3151, 0, 0, 0, 0, 'Way of the Turtle', 'Menagerie', 14, 'Way', NULL, 'Set this aside. If you did, play it at the start of your next\nturn.', '!N Pself', '-1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3152, 0, 0, 0, 0, 'Way of the Worm', 'Menagerie', 14, 'Way', NULL, 'Exile an Estate from the Supply.', NULL, NULL, NULL, NULL, NULL, '1 Supply', NULL, NULL, '(1 )'),
(3153, 0, 0, 0, 0, 'Acolyte', 'Allies', 15, 'Action - Augur', '4', 'You may trash an Action or Victory card from your hand to gain\na Gold.You may trash this to gain an Augur.', NULL, NULL, NULL, NULL, '1?, Self?', NULL, NULL, '!1, !1', NULL),
(3154, 0, 0, 0, 0, 'Archer', 'Allies', 15, 'Action - Attack - Clash', '4', '+2Each other player with\n5 or more cards in hand reveals all but one, and discards one\nof those you choose.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3155, 0, 0, 0, 0, 'Barbarian', 'Allies', 15, 'Action - Attack', '5', '+2Each other player\ntrashes the top card of their deck. If it costs 3 or more they gain a\ncheaper card sharing a type with it; otherwise they gain a\nCurse.', NULL, NULL, NULL, '+2', NULL, NULL, '!1', NULL, NULL),
(3156, 0, 0, 0, 0, 'Battle Plan', 'Allies', 15, 'Action - Clash', '3', '+1 Card+1 ActionYou may reveal an Attack card from\nyour hand for +1 Card.You may rotate any Supply pile.', '+1', '+1, +1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3157, 0, 0, 0, 0, 'Bauble', 'Allies', 15, 'Treasure - Liaison', '2', 'Choose two different options: +1 Buy; +1; +1 Favor; this\nturn, when you gain a card, you may put it onto your deck.', NULL, NULL, '+1?', '+1?', NULL, NULL, NULL, NULL, NULL),
(3158, 0, 0, 0, 0, 'Blacksmith', 'Allies', 15, 'Action - Townsfolk', '3', 'Choose one: Draw until you have 6 cards in hand; or\n+2 Cards; or +1 Card and +1 Action.', '+1?', '=6?, +2?, +1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3159, 0, 0, 0, 0, 'Broker', 'Allies', 15, 'Action - Liaison', '4', 'Trash a card from your hand and choose one:+1 Card per\n1 it costs;or\n+1 Action per 1 it costs; or\n+1 per 1 it costs; or\n+1 Favor per 1 it costs.', '+X?', '+X?', NULL, '+?', '1', NULL, NULL, NULL, NULL),
(3160, 0, 0, 0, 0, 'Capital City', 'Allies', 15, 'Action', '5', '+1 Card+2 ActionsYou may discard 2 cards for\n+2.You may pay  for\n+2 Cards.', '+2', '+1, -2?, !+2', NULL, '-2?, !+', NULL, NULL, NULL, NULL, NULL),
(3161, 0, 0, 0, 0, 'Carpenter', 'Allies', 15, 'Action', '4', 'If no Supply piles are empty, +1 Action and gain a card\ncosting up to 4.Otherwise, trash a\ncard from your hand and gain a card costing up to 2 more than it.', '!+1', NULL, NULL, NULL, '!1', NULL, NULL, '1', NULL),
(3162, 0, 0, 0, 0, 'Conjurer', 'Allies', 15, 'Action - Duration - Wizard', '4', 'Gain a card costing up to 4.At the start of your\nnext turn, put this into your hand.', NULL, 'N +Self', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3163, 0, 0, 0, 0, 'Contract', 'Allies', 15, 'Treasure - Duration - Liaison', '5', '2+1 FavorYou may\nset aside an Action from your hand to play it at the start of your\nnext turn.', 'N P1?', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3164, 0, 0, 0, 0, 'Courier', 'Allies', 15, 'Action', '4', '+1Discard the top card\nof your deck. Look through your discard pile; you may play an\nAction or Treasure from it.', 'P1?', NULL, NULL, '+1, P1?', NULL, NULL, NULL, NULL, NULL),
(3165, 0, 0, 0, 0, 'Distant Shore', 'Allies', 15, 'Action - Victory - Odyssey', '6', '+2 Cards+1 ActionGain an Estate.2 ', '+1', '+2', NULL, NULL, NULL, NULL, NULL, '1', '2 , (1 )'),
(3166, 0, 0, 0, 0, 'Elder', 'Allies', 15, 'Action - Townsfolk', '5', '+2You may play an Action\ncard from your hand. When it gives you a choice of abilities (with\n\"choose\") this turn, you may choose an extra (different)\noption.', 'P1?', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3167, 0, 0, 0, 0, 'Emissary', 'Allies', 15, 'Action - Liaison', '5', '+3 CardsIf this made you shuffle (at least one card),\n+1 Action and +2 Favors.', '!+1', '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3168, 0, 0, 0, 0, 'Galleria', 'Allies', 15, 'Action', '5', '+3This turn, when you\ngain a card costing 3 or 4, +1 Buy.', NULL, NULL, '+X', '+3', NULL, NULL, NULL, NULL, NULL),
(3169, 0, 0, 0, 0, 'Garrison', 'Allies', 15, 'Action - Duration - Fort', '4', '+2This turn, when you\ngain a card, add a token here. At the start of your next turn,\nremove them for +1 Card each.', NULL, 'N +X', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3170, 0, 0, 0, 0, 'Guildmaster', 'Allies', 15, 'Action - Liaison', '5', '+3This turn, when you\ngain a card, +1 Favor.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(3171, 0, 0, 0, 0, 'Herb Gatherer', 'Allies', 15, 'Action - Augur', '3', '+1 BuyPut your deck into your discard pile. Look through\nit and you may play a Treasure from it.You may rotate the\nAugurs.', NULL, NULL, '+1', 'P1?', NULL, NULL, NULL, NULL, NULL),
(3172, 0, 0, 0, 0, 'Highwayman', 'Allies', 15, 'Action - Duration - Attack', '5', 'At the start of your next turn, discard this from play and\n+3 Cards.Until then, the first Treasure each other player\nplays each turn does nothing.', NULL, 'N +3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3173, 0, 0, 0, 0, 'Hill Fort', 'Allies', 15, 'Action - Fort', '5', 'Gain a card costing up to 4. Choose one: Put it\ninto your hand; or +1 Card and +1 Action.', '+1?', '(+1)?, +1?', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3174, 0, 0, 0, 0, 'Hunter', 'Allies', 15, 'Action', '5', '+1 ActionReveal the top 3 cards of your deck. From\nthose cards, put an Action, a Treasure, and a Victory card into\nyour hand. Discard the rest.', '+1', '+ 0-3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3175, 0, 0, 0, 0, 'Importer', 'Allies', 15, 'Action - Duration - Liaison', '3', 'At the start of your next turn, gain a card costing up to\n5.Setup: Each player\ngets +4 Favors.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'N 1', NULL),
(3176, 0, 0, 0, 0, 'Innkeeper', 'Allies', 15, 'Action', '4', '+1 ActionChoose one: +1 Card; or +3 Cards, then\ndiscard 3 cards; or +5 Cards, then discard\n6 cards.', '+1', '+1 or +3 -3 or +5 -6 ?', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3177, 0, 0, 0, 0, 'Lich', 'Allies', 15, 'Action - Wizard', '6', '+6 Cards+2 ActionsSkip a turn.When you trash this,\ndiscard it and gain a cheaper card from the trash.', '+2', '+6', NULL, NULL, NULL, NULL, NULL, '*Self 1', NULL),
(3178, 0, 0, 0, 0, 'Marquis', 'Allies', 15, 'Action', '6', '+1 Buy+1 Card per card in your hand. Discard down to\n10 cards in hand.', NULL, '+X, -Y', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3179, 0, 0, 0, 0, 'Merchant Camp', 'Allies', 15, 'Action', '3', '+2 Actions+1When you discard this\nfrom play, you may put it onto your deck.', '+2', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(3180, 0, 0, 0, 0, 'Miller', 'Allies', 15, 'Action - Townsfolk', '4', '+1 ActionLook at the top 4 cards of your deck. Put one\ninto your hand and discard the rest.', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3181, 0, 0, 0, 0, 'Modify', 'Allies', 15, 'Action', '5', 'Trash a card from your hand. Choose one: +1 Card and\n+1 Action; or gain a card costing up to 2 more than the trashed\ncard.', '+1?', '+1?', NULL, NULL, '1', NULL, NULL, '1?', NULL),
(3182, 0, 0, 0, 0, 'Old Map', 'Allies', 15, 'Action - Odyssey', '3', '+1 Card+1 ActionDiscard a card. +1 Card.You may\nrotate the Odysseys.', '+1', '+1, -1, +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3183, 0, 0, 0, 0, 'Royal Galley', 'Allies', 15, 'Action - Duration', '4', '+1 CardYou may play a non-Duration Action card from your\nhand. Set it aside; if you did, then at the start of your next\nturn, play it.', 'P1, N P1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3184, 0, 0, 0, 0, 'Sentinel', 'Allies', 15, 'Action', '3', 'Look at the top 5 cards of your deck. You may trash up to\n2 of them. Put the rest back in any order.', NULL, NULL, NULL, NULL, '0-2?', NULL, NULL, NULL, NULL),
(3185, 0, 0, 0, 0, 'Sibyl', 'Allies', 15, 'Action - Augur', '6', '+4 Cards+1 ActionPut a card from your hand on top of\nyour deck, and another on the bottom.', '+1', '+4, -2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3186, 0, 0, 0, 0, 'Skirmisher', 'Allies', 15, 'Action - Attack', '5', '+1 Card+1 Action+1This turn, when you\ngain an Attack card, each other player discards down to\n3 cards in hand.', '+1', '+1', NULL, '+1', NULL, NULL, NULL, NULL, NULL),
(3187, 0, 0, 0, 0, 'Sorcerer', 'Allies', 15, 'Action - Attack - Wizard', '5', '+1 Card+1 ActionEach other player names a card, then\nreveals the top card of their deck. If wrong, they gain a\nCurse.', '+1', '+1', NULL, NULL, NULL, NULL, '!1', NULL, NULL),
(3188, 0, 0, 0, 0, 'Sorceress', 'Allies', 15, 'Action - Attack - Augur', '5', '+1 ActionName a card. Reveal the top card of your deck and\nput it into your hand. If it\'s the named card, each other player\ngains a Curse.', '+1', '+1', NULL, NULL, NULL, NULL, '!1', NULL, NULL),
(3189, 0, 0, 0, 0, 'Specialist', 'Allies', 15, 'Action', '5', 'You may play an Action or Treasure from your hand. Choose one:\nPlay it again; or gain a copy of it.', 'P1, P1?', NULL, NULL, 'P1, P1?', NULL, NULL, NULL, '1?', NULL),
(3190, 0, 0, 0, 0, 'Stronghold', 'Allies', 15, 'Action - Victory - Duration - Fort', '6', 'Choose one: +3; or at the start of\nyour next turn, +3 Cards.2 ', NULL, 'N +3?', NULL, '+3?', NULL, NULL, NULL, NULL, '2 '),
(3191, 0, 0, 0, 0, 'Student', 'Allies', 15, 'Action - Wizard - Liaison', '3', '+1 ActionYou may rotate the Wizards.Trash a card from your\nhand. If it\'s a Treasure, +1 Favor and put this onto your\ndeck.', '+1', NULL, NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3192, 0, 0, 0, 0, 'Sunken Treasure', 'Allies', 15, 'Treasure - Odyssey', '5', 'Gain an Action card you don\'t have a copy of in play.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3193, 0, 0, 0, 0, 'Swap', 'Allies', 15, 'Action', '5', '+1 Card+1 ActionYou may return an Action from your\nhand to its pile, to gain to your hand a different Action costing\nup to 5.', '+1', '+1, !(+1)', NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(3194, 0, 0, 0, 0, 'Sycophant', 'Allies', 15, 'Action - Liaison', '2', '+1 ActionDiscard 3 cards. If you discarded at least\none, +3.When you gain or\ntrash this, +2 Favors.', '+1', '-3', NULL, '!+3', NULL, NULL, NULL, NULL, NULL),
(3195, 0, 0, 0, 0, 'Tent', 'Allies', 15, 'Action - Fort', '3', '+2You may rotate the\nForts.When you discard this from play, you may put it onto your\ndeck.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3196, 0, 0, 0, 0, 'Territory', 'Allies', 15, 'Victory - Clash', '6', 'Worth 1  per differently named\nVictory card you have.When you gain this, gain a Gold per empty\nSupply pile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '*X', 'X '),
(3197, 0, 0, 0, 0, 'Town', 'Allies', 15, 'Action', '4', 'Choose one: +1 Card and +2 Actions; or +1 Buy\nand +2.', '+2?', '+1?', '+1?', '+2?', NULL, NULL, NULL, NULL, NULL),
(3198, 0, 0, 0, 0, 'Town Crier', 'Allies', 15, 'Action - Townsfolk', '2', 'Choose one: +2; or gain a Silver; or\n+1 Card and +1 Action.You may rotate the Townsfolk.', '+1?', '+1?', NULL, '+2?', NULL, NULL, NULL, '1?', NULL),
(3199, 0, 0, 0, 0, 'Underling', 'Allies', 15, 'Action - Liaison', '3', '+1 Card+1 Action+1 Favor', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3200, 0, 0, 0, 0, 'Voyage', 'Allies', 15, 'Action - Duration - Odyssey', '4', '+1 ActionTake an extra turn after this one (but not a 3rd\nturn in a row), during which you can only play 3 cards from\nyour hand.', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
INSERT INTO `parsed_cards` (`id`, `added`, `skip`, `en_added`, `no_finnish`, `name`, `expansion`, `exp_id`, `type`, `cost`, `text`, `act_villager`, `draws`, `buys`, `coins_coffer`, `trash_ret`, `exile`, `junk`, `gain`, `victorypts`) VALUES
(3201, 0, 0, 0, 0, 'Warlord', 'Allies', 15, 'Action - Duration - Attack - Clash', '5', '+1 ActionAt the start of your next turn, +2 Cards.\nUntil then, other players can\'t play an Action from their hand that\nthey have 2 or more copies of in play.', '+1', 'N +2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3202, 0, 0, 0, 0, 'Architects\' Guild', 'Allies', 15, 'Ally', NULL, 'When you gain a card, you may spend 2 Favors to gain a\ncheaper non-Victory card.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(3203, 0, 0, 0, 0, 'Band of Nomads', 'Allies', 15, 'Ally', NULL, 'When you gain a card costing 3 or more, you may\nspend a Favor, for +1 Card, or +1 Action, or\n+1 Buy.', '!+1', '!+1', '!+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3204, 0, 0, 0, 0, 'Cave Dwellers', 'Allies', 15, 'Ally', NULL, 'At the start of your turn, you may spend a Favor, to discard a\ncard then draw a card. Repeat as desired.', NULL, '-1, +1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3205, 0, 0, 0, 0, 'Circle of Witches', 'Allies', 15, 'Ally', NULL, 'After playing a Liaison, you may spend 3 Favors to have\neach other player gain a Curse.', NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL, NULL),
(3206, 0, 0, 0, 0, 'City-state', 'Allies', 15, 'Ally', NULL, 'When you gain an Action card during your turn, you may spend\n2 Favors to play it.', '*P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3207, 0, 0, 0, 0, 'Coastal Haven', 'Allies', 15, 'Ally', NULL, 'When discarding your hand in Clean-up, you may spend any number\nof Favors to keep that many cards in hand for next turn (you still\ndraw 5).', NULL, '(N +X)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3208, 0, 0, 0, 0, 'Crafters\' Guild', 'Allies', 15, 'Ally', NULL, 'At the start of your turn, you may spend 2 Favors to gain a\ncard costing up to 4 onto your deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3209, 0, 0, 0, 0, 'Desert Guides', 'Allies', 15, 'Ally', NULL, 'At the start of your turn, you may spend a Favor to discard\nyour hand and draw 5 cards. Repeat as desired.', NULL, '-X +5', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3210, 0, 0, 0, 0, 'Family of Inventors', 'Allies', 15, 'Ally', NULL, 'At the start of your Buy phase, you may put a Favor token you\nhave on a non-Victory Supply pile. Cards cost 1 less per Favor token\non their piles.', NULL, NULL, NULL, '!R1', NULL, NULL, NULL, NULL, NULL),
(3211, 0, 0, 0, 0, 'Fellowship of Scribes', 'Allies', 15, 'Ally', NULL, 'After playing an Action, if you have 4 or fewer cards in\nhand, you may spend a Favor for +1 Card.', NULL, '!+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3212, 0, 0, 0, 0, 'Forest Dwellers', 'Allies', 15, 'Ally', NULL, 'At the start of your turn, you may spend a Favor to look at the\ntop 3 cards of your deck, discard any number and put the rest\nback in any order.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3213, 0, 0, 0, 0, 'Gang of Pickpockets', 'Allies', 15, 'Ally', NULL, 'At the start of your turn, discard down to 4 cards in hand\nunless you spend a Favor.', NULL, '-X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3214, 0, 0, 0, 0, 'Island Folk', 'Allies', 15, 'Ally', NULL, 'At the end of your turn, you may spend 5 Favors to take an\nextra turn after this one (but not a 3rd turn in a row).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3215, 0, 0, 0, 0, 'League of Bankers', 'Allies', 15, 'Ally', NULL, 'At the start of your Buy phase, +1 per 4 Favors you\nhave (round down).', NULL, NULL, NULL, '**+', NULL, NULL, NULL, NULL, NULL),
(3216, 0, 0, 0, 0, 'League of Shopkeepers', 'Allies', 15, 'Ally', NULL, 'After playing a Liaison, if you have 5 or more Favors,\n+1, and if 10 or more,\n+1 Action and +1 Buy.', '**+1?', NULL, '**+1?', '**+1?', NULL, NULL, NULL, NULL, NULL),
(3217, 0, 0, 0, 0, 'Market Towns', 'Allies', 15, 'Ally', NULL, 'At the start of your Buy phase, you may spend a Favor to play\nan Action card from your hand. Repeat as desired.', 'P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3218, 0, 0, 0, 0, 'Mountain Folk', 'Allies', 15, 'Ally', NULL, 'At the start of your turn, you may spend 5 Favors for\n+3 Cards.', NULL, '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3219, 0, 0, 0, 0, 'Order of Astrologers', 'Allies', 15, 'Ally', NULL, 'When shuffling, you may pick one card per Favor you spend to go\non top.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3220, 0, 0, 0, 0, 'Order of Masons', 'Allies', 15, 'Ally', NULL, 'When shuffling, you may pick up to 2 cards per Favor you\nspend to put into your discard pile.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3221, 0, 0, 0, 0, 'Peaceful Cult', 'Allies', 15, 'Ally', NULL, 'At the start of your Buy phase, you may spend any number of\nFavors to trash that many cards from your hand.', NULL, NULL, NULL, NULL, 'X', NULL, NULL, NULL, NULL),
(3222, 0, 0, 0, 0, 'Plateau Shepherds', 'Allies', 15, 'Ally', NULL, 'When scoring, pair up your Favors with cards you have costing\n2, for 2  per pair.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X '),
(3223, 0, 0, 0, 0, 'Trappers\' Lodge', 'Allies', 15, 'Ally', NULL, 'When you gain a card, you may spend a Favor to put it onto your\ndeck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3224, 0, 0, 0, 0, 'Woodworkers\' Guild', 'Allies', 15, 'Ally', NULL, 'At the start of your Buy phase, you may spend a Favor to trash\nan Action card from your hand. If you did, gain an Action\ncard.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(3225, 0, 0, 0, 0, 'Abundance', 'Plunder', 16, 'Treasure - Duration', '4', 'The next time you gain an Action card: +1 Buy and\n+3.', NULL, NULL, '**+1', '**+3', NULL, NULL, NULL, NULL, NULL),
(3226, 0, 0, 0, 0, 'Buried Treasure', 'Plunder', 16, 'Treasure - Duration', '5', 'At the start of your next turn, +1 Buy and +3.When you gain this,\nplay it.', '*Pself', NULL, 'N +1', 'N +3', NULL, NULL, NULL, NULL, NULL),
(3227, 0, 0, 0, 0, 'Cabin Boy', 'Plunder', 16, 'Action - Duration', '4', '+1 Card+1 ActionAt the start of your next turn,\nchoose one: +2; or trash this to\ngain a Duration card.', '+1', '+1', NULL, 'N +2?', 'N self?', NULL, NULL, '!N 1', NULL),
(3228, 0, 0, 0, 0, 'Cage', 'Plunder', 16, 'Treasure - Duration', '2', 'Set aside up to 4 cards from your hand face down (on\nthis). The next time you gain a Victory card, trash this, and put\nthe set aside cards into your hand at end of turn.', NULL, '- 0-4(X)**(N +X)', NULL, NULL, '**Self', NULL, NULL, NULL, NULL),
(3229, 0, 0, 0, 0, 'Crew', 'Plunder', 16, 'Action - Duration', '5', '+3 CardsAt the start of your next turn, put this onto your\ndeck.', NULL, '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3230, 0, 0, 0, 0, 'Crucible', 'Plunder', 16, 'Treasure', '4', 'Trash a card from your hand. +1 per 1 it costs.', NULL, NULL, NULL, '+', '1', NULL, NULL, NULL, NULL),
(3231, 0, 0, 0, 0, 'Cutthroat', 'Plunder', 16, 'Action - Duration - Attack', '5', 'Each other player discards down to 3 cards in hand.The\nnext time anyone gains a Treasure costing 5 or more, gain a\nLoot.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '**1', NULL),
(3232, 0, 0, 0, 0, 'Enlarge', 'Plunder', 16, 'Action - Duration', '5', 'Now and at the start of your next turn: Trash a card from your\nhand, and gain one costing up to 2 more.', NULL, NULL, NULL, NULL, '1, N 1', NULL, NULL, '1, N 1', NULL),
(3233, 0, 0, 0, 0, 'Figurine', 'Plunder', 16, 'Treasure', '5', '+2 CardsYou may discard an Action card for +1 Buy and\n+1.', NULL, '+2, -1?', '!+1', '!+1', NULL, NULL, NULL, NULL, NULL),
(3234, 0, 0, 0, 0, 'First Mate', 'Plunder', 16, 'Action', '5', 'Play any number of Action cards with the same name from your\nhand, then draw until you have 6 cards in hand.', 'PX', '=6', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3235, 0, 0, 0, 0, 'Flagship', 'Plunder', 16, 'Action - Duration - Command', '4', '+2The next time you play\na non-Command Action card, replay it.', '**P1', NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3236, 0, 0, 0, 0, 'Fortune Hunter', 'Plunder', 16, 'Action', '4', '+2Look at the top\n3 cards of your deck. You may play a Treasure from them. Put\nthe rest back in any order.', NULL, NULL, NULL, '+2, P1?', NULL, NULL, NULL, NULL, NULL),
(3237, 0, 0, 0, 0, 'Frigate', 'Plunder', 16, 'Action - Duration - Attack', '5', '+3Until the start of\nyour next turn, each time another player plays an Action card, they\ndiscard down to 4 cards in hand afterwards.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(3238, 0, 0, 0, 0, 'Gondola', 'Plunder', 16, 'Treasure - Duration', '4', 'Either now or at the start of your next turn: +2.When you gain this,\nyou may play an Action card from your hand.', '*P1', NULL, NULL, '+2?, N +2?', NULL, NULL, NULL, NULL, NULL),
(3239, 0, 0, 0, 0, 'Grotto', 'Plunder', 16, 'Action - Duration', '2', '+1 ActionSet aside up to 4 cards from your hand face\ndown (on this). At the start of your next turn, discard them, then\ndraw as many.', '+1', '- 0-4(X)N +Y (X=Y)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3240, 0, 0, 0, 0, 'Harbor Village', 'Plunder', 16, 'Action', '4', '+1 Card+2 ActionsAfter the next Action you play this\nturn, if it gave you +, +1.', '+2', '+1', NULL, '**+1', NULL, NULL, NULL, NULL, NULL),
(3241, 0, 0, 0, 0, 'Jewelled Egg', 'Plunder', 16, 'Treasure', '2', '1+1 BuyWhen you trash\nthis, gain a Loot.', NULL, NULL, '+1', '+1', NULL, NULL, NULL, '*1', NULL),
(3242, 0, 0, 0, 0, 'King\'s Cache', 'Plunder', 16, 'Treasure', '7', 'You may play a Treasure from your hand 3 times.', NULL, NULL, NULL, 'P3?', NULL, NULL, NULL, NULL, NULL),
(3243, 0, 0, 0, 0, 'Landing Party', 'Plunder', 16, 'Action - Duration', '4', '+2 Cards+2 ActionsThe next time the first card you\nplay on a turn is a Treasure, put this onto your deck\nafterwards.', '+2', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3244, 0, 0, 0, 0, 'Longship', 'Plunder', 16, 'Action - Duration', '5', '+2 ActionsAt the start of your next turn,\n+2 Cards.', '+2', 'N +2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3245, 0, 0, 0, 0, 'Mapmaker', 'Plunder', 16, 'Action - Reaction', '4', 'Look at the top 4 cards of your deck. Put 2 into your\nhand and discard the rest.When any player gains a Victory card, you\nmay play this from your hand.', '**Pself', '+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3246, 0, 0, 0, 0, 'Maroon', 'Plunder', 16, 'Action', '4', 'Trash a card from your hand. +2 Cards per type it has\n(Action, Attack, etc.).', NULL, '+X', NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3247, 0, 0, 0, 0, 'Mining Road', 'Plunder', 16, 'Action', '5', '+1 Action+1 Buy+2Once this turn, when\nyou gain a Treasure, you may play it.', '+1', NULL, '+1', '+2, !P1', NULL, NULL, NULL, NULL, NULL),
(3248, 0, 0, 0, 0, 'Pendant', 'Plunder', 16, 'Treasure', '5', '+1 per differently named\nTreasure you have in play.', NULL, NULL, NULL, '+', NULL, NULL, NULL, NULL, NULL),
(3249, 0, 0, 0, 0, 'Pickaxe', 'Plunder', 16, 'Treasure', '5', '1Trash a card from your\nhand. If it costs 3 or more, gain a Loot\nto your hand.', NULL, '!(+1)', NULL, '+1', '1', NULL, NULL, '!1', NULL),
(3250, 0, 0, 0, 0, 'Pilgrim', 'Plunder', 16, 'Action', '5', '+4 CardsPut a card from your hand onto your deck.', NULL, '+4, -1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3251, 0, 0, 0, 0, 'Quartermaster', 'Plunder', 16, 'Action - Duration', '5', 'At the start of each of your turns for the rest of the game,\nchoose one: Gain a card costing up to 4, setting it aside on\nthis; or put a card from this into your hand.', NULL, 'N... +1?', NULL, NULL, NULL, NULL, NULL, 'N... 1?', NULL),
(3252, 0, 0, 0, 0, 'Rope', 'Plunder', 16, 'Treasure - Duration', '4', '1+1 BuyAt the\nstart of your next turn, +1 Card and you may trash a card from\nyour hand.', NULL, 'N +1', '+1', '+1', 'N 1?', NULL, NULL, NULL, NULL),
(3253, 0, 0, 0, 0, 'Sack of Loot', 'Plunder', 16, 'Treasure', '6', '1+1 BuyGain a\nLoot.', NULL, NULL, '+1', '+1', NULL, NULL, NULL, '1', NULL),
(3254, 0, 0, 0, 0, 'Search', 'Plunder', 16, 'Action - Duration', '2', '+2The next time a Supply\npile empties, trash this and gain a Loot.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, '**1', NULL),
(3255, 0, 0, 0, 0, 'Secluded Shrine', 'Plunder', 16, 'Action - Duration', '3', '+1The next time you gain\na Treasure, trash up to 2 cards from your hand.', NULL, NULL, NULL, '+1', '**2', NULL, NULL, NULL, NULL),
(3256, 0, 0, 0, 0, 'Shaman', 'Plunder', 16, 'Action', '2', '+1 Action+1You may trash a card\nfrom your hand.In games using this, at the start of your turn, gain\na card from the trash costing up to 6.', '+1', NULL, NULL, '+1', '1?', NULL, 'N... !Self', 'N... 1', 'N... !(X )'),
(3257, 0, 0, 0, 0, 'Silver Mine', 'Plunder', 16, 'Treasure', '5', 'Gain a Treasure costing less than this to your hand.', NULL, '(+1)', NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3258, 0, 0, 0, 0, 'Siren', 'Plunder', 16, 'Action - Duration - Attack', '3', 'Each other player gains a Curse. At the start of your next\nturn, draw until you have 8 cards in hand.When you gain this,\ntrash it unless you trash an Action from your hand.', NULL, 'N =8', NULL, NULL, '*1 or Self ?', NULL, '1', NULL, NULL),
(3259, 0, 0, 0, 0, 'Stowaway', 'Plunder', 16, 'Action - Duration - Reaction', '3', 'At the start of your next turn, +2 Cards.When anyone gains\na Duration card, you may play this from your hand.', '**Pself', 'N +2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3260, 0, 0, 0, 0, 'Swamp Shacks', 'Plunder', 16, 'Action', '4', '+2 Actions+1 Card per 3 cards you have in play\n(round down).', '+2', '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3261, 0, 0, 0, 0, 'Taskmaster', 'Plunder', 16, 'Action - Duration', '3', '+1 Action, +1, and if you gain a\ncard costing exactly 5 this turn, then at\nthe start of your next turn, repeat this ability.', '+1, !N +1', NULL, NULL, '+1, !N +1', NULL, NULL, NULL, NULL, NULL),
(3262, 0, 0, 0, 0, 'Tools', 'Plunder', 16, 'Treasure', '4', 'Gain a copy of a card anyone has in play.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3263, 0, 0, 0, 0, 'Trickster', 'Plunder', 16, 'Action - Attack', '5', 'Each other player gains a Curse.Once this turn, when you\ndiscard a Treasure from play, you may set it aside. Put it in your\nhand at end of turn.', NULL, '!(N +1)', NULL, NULL, NULL, NULL, '1', NULL, NULL),
(3264, 0, 0, 0, 0, 'Wealthy Village', 'Plunder', 16, 'Action', '5', '+1 Card+2 ActionsWhen you gain this, if you have at\nleast 3 differently named Treasures in play, gain a Loot.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, '*1', NULL),
(3265, 0, 0, 0, 0, 'Amphora', 'Plunder', 16, 'Treasure - Duration - Loot', NULL, 'Either now or at the start of your next turn: +1 Buy and\n+3.', NULL, NULL, '+1?, N +1?', '+3?, N +3?', NULL, NULL, NULL, NULL, NULL),
(3266, 0, 0, 0, 0, 'Doubloons', 'Plunder', 16, 'Treasure - Loot', NULL, '3When you gain this,\ngain a Gold.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, '1', NULL),
(3267, 0, 0, 0, 0, 'Endless Chalice', 'Plunder', 16, 'Treasure - Duration - Loot', NULL, 'Now and at the start of each of your turns for the rest of the\ngame:1+1 Buy', NULL, NULL, 'N... +1', 'N... +1', NULL, NULL, NULL, NULL, NULL),
(3268, 0, 0, 0, 0, 'Figurehead', 'Plunder', 16, 'Treasure - Duration - Loot', NULL, '3At the start of your\nnext turn, +2 Cards.', NULL, 'N +2', NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(3269, 0, 0, 0, 0, 'Hammer', 'Plunder', 16, 'Treasure - Loot', NULL, '3Gain a card costing up\nto 4.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, '1', NULL),
(3270, 0, 0, 0, 0, 'Insignia', 'Plunder', 16, 'Treasure - Loot', NULL, '3This turn, when you\ngain a card, you may put it onto your deck.', NULL, NULL, NULL, '+3', NULL, NULL, NULL, NULL, NULL),
(3271, 0, 0, 0, 0, 'Jewels', 'Plunder', 16, 'Treasure - Duration - Loot', NULL, '3+1 BuyAt the\nstart of your next turn, put this on the bottom of your deck.', NULL, NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(3272, 0, 0, 0, 0, 'Orb', 'Plunder', 16, 'Treasure - Loot', NULL, 'Look through your discard pile. Choose one: Play an Action or\nTreasure from it; or +1 Buy and +3.', 'P1?', NULL, '+1?', 'P1?, +3?', NULL, NULL, NULL, NULL, NULL),
(3273, 0, 0, 0, 0, 'Prize Goat', 'Plunder', 16, 'Treasure - Loot', NULL, '3+1 BuyYou may\ntrash a card from your hand.', NULL, NULL, '+1', '+3', '1?', NULL, NULL, NULL, NULL),
(3274, 0, 0, 0, 0, 'Puzzle Box', 'Plunder', 16, 'Treasure - Loot', NULL, '3+1 BuyYou may set\naside a card from your hand face down. Put it into your hand at end\nof turn.', NULL, '-1, !(N +1)', '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(3275, 0, 0, 0, 0, 'Sextant', 'Plunder', 16, 'Treasure - Loot', NULL, '3+1 BuyLook at the\ntop 5 cards of your deck. Discard any number. Put the rest\nback in any order.', NULL, NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(3276, 0, 0, 0, 0, 'Shield', 'Plunder', 16, 'Treasure - Reaction - Loot', NULL, '3+1 BuyWhen\nanother player plays an Attack, you may first reveal this from your\nhand to be unaffected.', NULL, NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(3277, 0, 0, 0, 0, 'Spell Scroll', 'Plunder', 16, 'Action - Treasure - Loot', NULL, 'Trash this to gain a cheaper card. If it\'s an Action or\nTreasure, you may play it.', '!P1?', NULL, NULL, '!P1?', 'Self', NULL, NULL, '1', NULL),
(3278, 0, 0, 0, 0, 'Staff', 'Plunder', 16, 'Treasure - Loot', NULL, '3+1 BuyYou may\nplay an Action from your hand.', 'P1?', NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(3279, 0, 0, 0, 0, 'Sword', 'Plunder', 16, 'Treasure - Attack - Loot', NULL, '3+1 BuyEach other\nplayer discards down to 4 cards in hand.', NULL, NULL, '+1', '+3', NULL, NULL, NULL, NULL, NULL),
(3280, 0, 0, 0, 0, 'Avoid', 'Plunder', 16, 'Event', '2', '+1 BuyThe next time you shuffle this turn, pick up to\n3 of those cards to put into your discard pile.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3281, 0, 0, 0, 0, 'Bury', 'Plunder', 16, 'Event', '1', '+1 BuyPut any card from your discard pile on the bottom of\nyour deck.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3282, 0, 0, 0, 0, 'Deliver', 'Plunder', 16, 'Event', '2', '+1 BuyThis turn, each time you gain a card, set it aside,\nand put it into your hand at end of turn.', NULL, '(N +X)', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3283, 0, 0, 0, 0, 'Foray', 'Plunder', 16, 'Event', '3', 'Discard 3 cards, revealing them. If they have\n3 different names, gain a Loot.', NULL, '-3', NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(3284, 0, 0, 0, 0, 'Invasion', 'Plunder', 16, 'Event', '10', 'You may play an Attack from your hand. Gain a Duchy. Gain an\nAction onto your deck. Gain a Loot; play it.', 'P1?', NULL, NULL, 'P1?, P1', NULL, NULL, NULL, '3', '(3 )'),
(3285, 0, 0, 0, 0, 'Journey', 'Plunder', 16, 'Event', '4', 'You don\'t discard cards from play in Clean-up this turn. Take\nan extra turn after this one (but not a 3rd turn in a row).', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3286, 0, 0, 0, 0, 'Launch', 'Plunder', 16, 'Event', '3', 'Once per turn: Return to your Action phase. +1 Card,\n+1 Action, and +1 Buy.', '+1', '+1', '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3287, 0, 0, 0, 0, 'Looting', 'Plunder', 16, 'Event', '6', 'Gain a Loot.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3288, 0, 0, 0, 0, 'Maelstrom', 'Plunder', 16, 'Event', '4', 'Trash 3 cards from your hand. Each other player with\n5 or more cards in hand trashes one of them.', NULL, NULL, NULL, NULL, '3', NULL, NULL, NULL, NULL),
(3289, 0, 0, 0, 0, 'Mirror', 'Plunder', 16, 'Event', '3', '+1 BuyThe next time you gain an Action card this turn,\ngain a copy of it.', NULL, NULL, '+1', NULL, NULL, NULL, NULL, '1', NULL),
(3290, 0, 0, 0, 0, 'Peril', 'Plunder', 16, 'Event', '2', 'You may trash an Action card from your hand to gain a\nLoot.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '!1', NULL),
(3291, 0, 0, 0, 0, 'Prepare', 'Plunder', 16, 'Event', '3', 'Set aside your hand face up. At the start of your next turn,\nplay those Actions and Treasures in any order, then discard the\nrest.', 'N PX', NULL, NULL, 'N PX', NULL, NULL, NULL, NULL, NULL),
(3292, 0, 0, 0, 0, 'Prosper', 'Plunder', 16, 'Event', '10', 'Gain a Loot, plus any number of differently named\nTreasures.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(3293, 0, 0, 0, 0, 'Rush', 'Plunder', 16, 'Event', '2', '+1 BuyThe next time you gain an Action card this turn,\nplay it.', '!P1', NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3294, 0, 0, 0, 0, 'Scrounge', 'Plunder', 16, 'Event', '3', 'Choose one: Trash a card from your hand; or gain an Estate from\nthe trash, and if you did, gain a card costing up to 5.', NULL, NULL, NULL, NULL, '1?', NULL, NULL, '1?, !1', '(1 ?)'),
(3295, 0, 0, 0, 0, 'Cheap', 'Plunder', 16, 'Trait', NULL, 'Cheap cards cost 1 less.', NULL, NULL, NULL, 'R1self', NULL, NULL, NULL, NULL, NULL),
(3296, 0, 0, 0, 0, 'Cursed', 'Plunder', 16, 'Trait', NULL, 'When you gain a Cursed card, gain a Loot and a Curse.', NULL, NULL, NULL, NULL, NULL, NULL, 'Self', '2', '(-1 )'),
(3297, 0, 0, 0, 0, 'Fated', 'Plunder', 16, 'Trait', NULL, 'When shuffling, you may look through the cards and reveal Fated\ncards to put them on the top or bottom.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3298, 0, 0, 0, 0, 'Fawning', 'Plunder', 16, 'Trait', NULL, 'When you gain a Province, gain a Fawning card.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3299, 0, 0, 0, 0, 'Friendly', 'Plunder', 16, 'Trait', NULL, 'At the start of your Clean-up phase, you may discard a Friendly\ncard to gain a Friendly card.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '1?', NULL),
(3300, 0, 0, 0, 0, 'Hasty', 'Plunder', 16, 'Trait', NULL, 'When you gain a Hasty card, set it aside, and play it at the\nstart of your next turn.', 'N P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3301, 0, 0, 0, 0, 'Inherited', 'Plunder', 16, 'Trait', NULL, 'Setup: You start the game with an Inherited card in place of a\nstarting card you choose.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3302, 0, 0, 0, 0, 'Inspiring', 'Plunder', 16, 'Trait', NULL, 'After playing an Inspiring card on your turn, you may play an\nAction from your hand that you don\'t have a copy of in play.', 'P1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3303, 0, 0, 0, 0, 'Nearby', 'Plunder', 16, 'Trait', NULL, 'When you gain a Nearby card, +1 Buy.', NULL, NULL, '*+1', NULL, NULL, NULL, NULL, NULL, NULL),
(3304, 0, 0, 0, 0, 'Patient', 'Plunder', 16, 'Trait', NULL, 'At the start of your Clean-up phase, you may set aside Patient\ncards from your hand to play them at the start of your next\nturn.', 'N P1?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3305, 0, 0, 0, 0, 'Pious', 'Plunder', 16, 'Trait', NULL, 'When you gain a Pious card, you may trash a card from your\nhand.', NULL, NULL, NULL, NULL, '*1', NULL, NULL, NULL, NULL),
(3306, 0, 0, 0, 0, 'Reckless', 'Plunder', 16, 'Trait', NULL, 'Follow the instructions of played Reckless cards twice. When\ndiscarding one from play, return it to its pile.', '(Pself)', NULL, NULL, NULL, 'Self', NULL, NULL, NULL, NULL),
(3307, 0, 0, 0, 0, 'Rich', 'Plunder', 16, 'Trait', NULL, 'When you gain a Rich card, gain a Silver.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '*1', NULL),
(3308, 0, 0, 0, 0, 'Shy', 'Plunder', 16, 'Trait', NULL, 'At the start of your turn, you may discard one Shy card for\n+2 Cards.', NULL, '-Self?, !+2', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3309, 0, 0, 0, 0, 'Tireless', 'Plunder', 16, 'Trait', NULL, 'When you discard a Tireless card from play, set it aside, and\nput it onto your deck at end of turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3310, 1, 0, 0, 0, 'Alley', 'Rising Sun', 18, 'Action - Shadow', '4', '+1 Card+1 ActionDiscard a card.You can play this from\nyour deck as if in your hand.', '+1', '+1, +1?, -1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3311, 1, 0, 0, 0, 'Aristocrat', 'Rising Sun', 18, 'Action', '3', 'If the number of Aristocrats you have in play is:1 or 5:\n+3 Actions;2 or 6: +3 Cards;3 or 7: +3;4 or 8:\n+3 Buys.', '!+3', '!+3', '!+3', '!+3', NULL, NULL, NULL, NULL, NULL),
(3312, 1, 0, 0, 0, 'Artist', 'Rising Sun', 18, 'Action', NULL, '+1 Action+1 Card per card you have exactly one copy\nof in play.', '+1', '+X', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3313, 1, 0, 0, 0, 'Change', 'Rising Sun', 18, 'Action', '4', 'If you have any , +3. Otherwise, trash a\ncard from your hand, and gain a card costing more  than it. + equal to the\ndifference in .', NULL, NULL, NULL, '!+3, !+', '!1', NULL, NULL, '!1', NULL),
(3314, 1, 0, 0, 0, 'Craftsman', 'Rising Sun', 18, 'Action', '3', '+Gain a card costing up\nto 5.', NULL, NULL, NULL, '+', NULL, NULL, NULL, '1', NULL),
(3315, 1, 0, 0, 0, 'Daimyo', 'Rising Sun', 18, 'Action - Command', NULL, '+1 Card+1 ActionThe next time you play a non-Command\nAction card this turn, replay it afterwards.', '+1,(P1)', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3316, 1, 0, 0, 0, 'Fishmonger', 'Rising Sun', 18, 'Action - Shadow', '2', '+1 Buy+1You can play this from\nyour deck as if in your hand.', NULL, NULL, '+1', '+1', NULL, NULL, NULL, NULL, NULL),
(3317, 1, 0, 0, 0, 'Gold Mine', 'Rising Sun', 18, 'Action', '5', '+1 Card+1 Action+1 BuyYou may gain a Gold and\nget +.', '+1', '+1', '+1', '+?', NULL, NULL, NULL, '1?', NULL),
(3318, 1, 0, 0, 0, 'Imperial Envoy', 'Rising Sun', 18, 'Action', '5', '+5 Cards+1 Buy+', NULL, '+5', '+1', '+', NULL, NULL, NULL, NULL, NULL),
(3319, 1, 0, 0, 0, 'Kitsune', 'Rising Sun', 18, 'Action - Attack - Omen', '5', '+1Choose two different\noptions: +2 Actions; +2; each other player\ngains a Curse; gain a Silver.', '+2?', NULL, NULL, '+2?', NULL, NULL, '1?', '1?', NULL),
(3320, 1, 0, 0, 0, 'Litter', 'Rising Sun', 18, 'Action', '5', '+2 Cards+2 Actions+', '+2', '+2', NULL, '+', NULL, NULL, NULL, NULL, NULL),
(3321, 1, 0, 0, 0, 'Mountain Shrine', 'Rising Sun', 18, 'Action - Omen', NULL, '+1+2You may trash a card\nfrom your hand. Then if there are any Action cards in the trash,\n+2 Cards.', NULL, '!+2', NULL, '+2', '1?', NULL, NULL, NULL, NULL),
(3322, 1, 0, 0, 0, 'Ninja', 'Rising Sun', 18, 'Action - Attack - Shadow', '4', '+1 CardEach other player discards down to 3 cards in\nhand.You can play this from your deck as if in your hand.', NULL, '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3323, 1, 0, 0, 0, 'Poet', 'Rising Sun', 18, 'Action - Omen', '4', '+1+1 Card+1 ActionReveal the top card of your deck.\nIf it costs 3 or less, put it into\nyour hand.', '+1', '+1, !+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3324, 1, 0, 0, 0, 'Rice', 'Rising Sun', 18, 'Treasure', '7', '+1 Buy+1 per different type\namong cards you have in play.', NULL, NULL, '+1', '+', NULL, NULL, NULL, NULL, NULL),
(3325, 1, 0, 0, 0, 'Rice Broker', 'Rising Sun', 18, 'Action', '5', '+1 ActionTrash a card from your hand. If it\'s a Treasure,\n+2 Cards. If it\'s an Action, +5 Cards.', '+1', '!+2,!+5', NULL, NULL, '1', NULL, NULL, NULL, NULL),
(3326, 1, 0, 0, 0, 'River Shrine', 'Rising Sun', 18, 'Action - Omen', '4', '+1Trash up to\n2 cards from your hand. At the start of Clean-up, if you\ndidn\'t gain any cards in your Buy phase this turn, gain a card\ncosting up to 4.', NULL, NULL, NULL, NULL, '0-2', NULL, NULL, '!1', NULL),
(3327, 1, 0, 0, 0, 'Riverboat', 'Rising Sun', 18, 'Action - Duration', '3', 'At the start of your next turn, play the set aside card,\nleaving it there.Setup: Set aside an unused non-Duration Action\ncard costing 5.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3328, 1, 0, 0, 0, 'Ronin', 'Rising Sun', 18, 'Action - Shadow', '5', 'Draw until you have 7 cards in hand.You can play this from\nyour deck as if in your hand.', NULL, '=7', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3329, 1, 0, 0, 0, 'Root Cellar', 'Rising Sun', 18, 'Action', '3', '+3 Cards+1 Action+', '+1', '+3', NULL, '+', NULL, NULL, NULL, NULL, NULL),
(3330, 1, 0, 0, 0, 'Rustic Village', 'Rising Sun', 18, 'Action - Omen', '4', '+1+1 Card+2 ActionsYou may discard 2 cards for\n+1 Card.', '+2', '+1,-2?,!+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3331, 1, 0, 0, 0, 'Samurai', 'Rising Sun', 18, 'Action - Duration - Attack', '6', 'Each other player discards down to 3 cards in hand\n(once).At the start of each of your turns this game, +1.<i>(This stays in\nplay.)</i>', NULL, NULL, NULL, 'N... +1', NULL, NULL, NULL, NULL, NULL),
(3332, 1, 0, 0, 0, 'Snake Witch', 'Rising Sun', 18, 'Action - Attack', '2', '+1 Card+1 ActionIf your hand has no duplicate cards,\nyou may reveal it and return this to its pile, to have each other\nplayer gain a Curse.', '+1', '+1', NULL, NULL, '!Self', NULL, '!1', NULL, NULL),
(3333, 1, 0, 0, 0, 'Tanuki', 'Rising Sun', 18, 'Action - Shadow', '5', 'Trash a card from your hand. Gain a card costing up to\n2 more than it.You can\nplay this from your deck as if in your hand.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '1', NULL),
(3334, 1, 0, 0, 0, 'Tea House', 'Rising Sun', 18, 'Action - Omen', '5', '+1+1 Card+1 Action+2', '+1', '+1', NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3335, 0, 0, 0, 0, 'Amass', 'Rising Sun', 18, 'Event', '2', 'If you have no Action cards in play, gain an Action card\ncosting up to 5.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '!1', NULL),
(3336, 0, 0, 0, 0, 'Asceticism', 'Rising Sun', 18, 'Event', '2', 'Pay any amount of  to trash that many\ncards from your hand.', NULL, NULL, NULL, '-', 'X', NULL, NULL, NULL, NULL),
(3337, 0, 0, 0, 0, 'Continue', 'Rising Sun', 18, 'Event', NULL, 'Once per turn: Gain a non-Attack Action card costing up to\n4. Return to your\nAction phase and play it. +1 Action and +1 Buy.', '+1', NULL, '+1', NULL, NULL, NULL, NULL, '1', NULL),
(3338, 0, 0, 0, 0, 'Credit', 'Rising Sun', 18, 'Event', '2', 'Gain an Action or Treasure costing up to 8. + equal to its\ncost.', NULL, NULL, NULL, '+', NULL, NULL, NULL, '1', NULL),
(3339, 0, 0, 0, 0, 'Foresight', 'Rising Sun', 18, 'Event', '2', 'Reveal cards from your deck until revealing an Action. Set it\naside and discard the rest. Put it into your hand at end of\nturn.', NULL, '(N+1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3340, 0, 0, 0, 0, 'Gather', 'Rising Sun', 18, 'Event', '7', 'Gain a card costing exactly 3, a card costing\nexactly 4, and a card costing\nexactly 5.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '3', NULL),
(3341, 0, 0, 0, 0, 'Kintsugi', 'Rising Sun', 18, 'Event', '3', 'Trash a card from your hand. If you\'ve gained a Gold this game,\ngain a card costing up to 2 more than the trashed\ncard.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '!1', NULL),
(3342, 0, 0, 0, 0, 'Practice', 'Rising Sun', 18, 'Event', '3', 'You may play an Action card from your hand twice.', 'P2?', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3343, 0, 0, 0, 0, 'Receive Tribute', 'Rising Sun', 18, 'Event', '5', 'If you\'ve gained at least 3 cards this turn, gain up to\n3 differently named Action cards you don\'t have copies of in\nplay.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '0-3', NULL),
(3344, 0, 0, 0, 0, 'Sea Trade', 'Rising Sun', 18, 'Event', '4', '+1 Card per Action card you have in play. Trash up to that\nmany cards from your hand.', NULL, '+X', NULL, NULL, NULL, NULL, NULL, 'X', NULL),
(3345, 0, 0, 0, 0, 'Approaching Army', 'Rising Sun', 18, 'Prophecy', NULL, 'After you play an Attack card, +1.Setup: Add an Attack\nkingdom card pile to the Supply.', NULL, NULL, NULL, '**+1', NULL, NULL, NULL, NULL, NULL),
(3346, 0, 0, 0, 0, 'Biding Time', 'Rising Sun', 18, 'Prophecy', NULL, 'At the start of your Clean-up, set aside your hand face down.\nAt the start of your next turn, put those cards into your\nhand.', NULL, '(-X,N+X)', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3347, 0, 0, 0, 0, 'Bureaucracy', 'Rising Sun', 18, 'Prophecy', NULL, 'When you gain a card that doesn\'t cost , gain a Copper.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '**1', NULL),
(3348, 0, 0, 0, 0, 'Divine Wind', 'Rising Sun', 18, 'Prophecy', NULL, 'When you remove the last , remove all Kingdom\ncard piles from the Supply, and set up 10 new random\npiles.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3349, 0, 0, 0, 0, 'Enlightenment', 'Rising Sun', 18, 'Prophecy', NULL, 'Treasures are also Actions. When you play a Treasure in an\nAction phase, instead of following its instructions, +1 Card\nand +1 Action.', '**+1', '**+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3350, 0, 0, 0, 0, 'Flourishing Trade', 'Rising Sun', 18, 'Prophecy', NULL, 'Cards cost 1 less. You may use\nAction plays as Buys.', NULL, NULL, '+X', 'R1, N... R1', NULL, NULL, NULL, NULL, NULL),
(3351, 0, 0, 0, 0, 'Good Harvest', 'Rising Sun', 18, 'Prophecy', NULL, 'The first time you play each differently named Treasure each\nturn, first, +1 Buy and +1.', NULL, NULL, '**+1', '**+1', NULL, NULL, NULL, NULL, NULL),
(3352, 0, 0, 0, 0, 'Great Leader', 'Rising Sun', 18, 'Prophecy', NULL, 'After each Action card you play, +1 Action.', '**+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3353, 0, 0, 0, 0, 'Growth', 'Rising Sun', 18, 'Prophecy', NULL, 'When you gain a Treasure, gain a cheaper card.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, '**1', NULL),
(3354, 0, 0, 0, 0, 'Harsh Winter', 'Rising Sun', 18, 'Prophecy', NULL, 'When you gain a card on your turn, if there\'s  on its pile, take it;\notherwise put  on its pile.', NULL, NULL, NULL, '**+', NULL, NULL, NULL, NULL, NULL),
(3355, 0, 0, 0, 0, 'Kind Emperor', 'Rising Sun', 18, 'Prophecy', NULL, 'At the start of your turn, and when you remove the last\n: Gain an Action to\nyour hand.', NULL, '**+1', NULL, NULL, NULL, NULL, NULL, '**1', NULL),
(3356, 0, 0, 0, 0, 'Panic', 'Rising Sun', 18, 'Prophecy', NULL, 'When you play a Treasure, +2 Buys, and when you discard\none from play, return it to its pile.', NULL, NULL, '**+2', NULL, '**1', NULL, NULL, NULL, NULL),
(3357, 0, 0, 0, 0, 'Progress', 'Rising Sun', 18, 'Prophecy', NULL, 'When you gain a card, put it onto your deck.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3358, 0, 0, 0, 0, 'Rapid Expansion', 'Rising Sun', 18, 'Prophecy', NULL, 'When you gain an Action or Treasure, set it aside, and play it\nat the start of your next turn.', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3359, 0, 0, 0, 0, 'Sickness', 'Rising Sun', 18, 'Prophecy', NULL, 'At the start of your turn, choose one: Gain a Curse onto your\ndeck; or discard 3 cards.', NULL, '**-3?', NULL, NULL, NULL, NULL, '**Self?', '**1?', NULL),
(3360, 1, 0, 0, 0, 'Avanto', 'Promo', 17, 'Action', '5', '+3 CardsYou may play a Sauna from your hand.', 'P1?', '+3', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3361, 1, 0, 1, 0, 'Black Market', 'Promo', 17, 'Action', '3', '+2Reveal the top\n3 cards of the Black Market deck. Play any number of Treasures\nfrom your hand. You may buy one of the revealed cards. Put the rest\non the bottom of the Black Market deck in any order.Setup: Make a\nBlack Market deck out of different unused Kingdom cards.', NULL, NULL, '(+1)', '+2', NULL, NULL, NULL, NULL, NULL),
(3362, 0, 0, 0, 0, 'Captain', 'Promo', 17, 'Action - Duration - Command', '6', 'Now and at the start of your next turn: Play a non-Duration,\nnon-Command Action card from the Supply costing up to 4, leaving it\nthere.', '(P1), (N P1)', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3363, 0, 0, 0, 0, 'Church', 'Promo', 17, 'Action - Duration', '3', '+1 ActionSet aside up to 3 cards from your hand face\ndown. At the start of your next turn, put them into your hand, then\nyou may trash a card from your hand.', '+1', '- 0-3(X), N +X', NULL, NULL, 'N 1?', NULL, NULL, NULL, NULL),
(3364, 1, 0, 1, 0, 'Dismantle', 'Promo', 17, 'Action', '4', 'Trash a card from your hand. If it costs 1 or more, gain a\ncheaper card and a Gold.', NULL, NULL, NULL, NULL, '1', NULL, NULL, '!2', NULL),
(3365, 0, 0, 0, 0, 'Envoy', 'Promo', 17, 'Action', '4', 'Reveal the top 5 cards of your deck. The player to your\nleft chooses one. Discard that one and put the rest into your\nhand.', NULL, '+5, !-1', NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3366, 0, 0, 0, 0, 'Governor', 'Promo', 17, 'Action', '5', '+1 ActionChoose one; you get the version in parentheses:\nEach player gets +1 (+3) Cards; or each player gains a\nSilver (Gold); or each player may trash a card from their hand and\ngain a card costing exactly 1 (2) more.', '+1', '+3?', NULL, NULL, '1?', NULL, NULL, '1?, !1', NULL),
(3367, 0, 0, 0, 0, 'Marchland', 'Promo', 17, 'Victory', '5', 'Worth 1  per 3 Victory\ncards you have (round down).When you gain this, +1 Buy, and\ndiscard any number of cards for +1 each.', NULL, NULL, '*+1', '*+', NULL, NULL, NULL, NULL, 'X '),
(3368, 0, 0, 0, 0, 'Prince', 'Promo', 17, 'Action - Duration - Command', '8', 'You may set aside (on this) a non-Duration, non-Command Action\ncard from your hand costing up to 4.At the start of each\nof your turns, play that card, leaving it set aside.', 'N... P1', NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL),
(3369, 1, 0, 0, 0, 'Sauna', 'Promo', 17, 'Action', '4', '+1 Card+1 ActionYou may play an Avanto from your\nhand.This turn, when you play a Silver, you may trash a card from\nyour hand.', '+1, P1?', '+1', NULL, NULL, 'X?', NULL, NULL, NULL, NULL),
(3370, 0, 0, 0, 0, 'Stash', 'Promo', 17, 'Treasure', '5', '2When shuffling this,\nyou may look through your remaining deck, and may put this anywhere\nin the shuffled cards.', NULL, NULL, NULL, '+2', NULL, NULL, NULL, NULL, NULL),
(3371, 0, 0, 0, 0, 'Summon', 'Promo', 17, 'Event', '5', 'Gain an Action card costing up to 4. Set it aside. If you\ndid, then at the start of your next turn, play it.', 'N P1', NULL, NULL, NULL, NULL, NULL, NULL, '1', NULL),
(3372, 1, 0, 1, 0, 'Walled Village', 'Promo', 17, 'Action', '4', '+1 Card+2 ActionsAt the start of Clean-up, if you\nhave this and no more than one other Action card in play, you may\nput this onto your deck.', '+2', '+1', NULL, NULL, NULL, NULL, NULL, NULL, NULL);

-- --------------------------------------------------------

--
-- Table structure for table `position`
--

DROP TABLE IF EXISTS `position`;
CREATE TABLE `position` (
  `id` int NOT NULL,
  `pos` varchar(255) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `position`
--

INSERT INTO `position` (`id`, `pos`) VALUES
(1, 'itse'),
(2, 'vasemmalla puolella oleva'),
(3, 'oikealla puolella oleva'),
(4, 'vastapäätä oleva'),
(5, 'oikealla puolella ISTUVA'),
(6, 'vasemmalla puolella ISTUVA'),
(7, 'vasemmalla puolella SEISOVA'),
(8, 'oikealla puolella SEISOVA'),
(9, 'kauimpana oleva'),
(10, 'lähimpänä oleva');

-- --------------------------------------------------------

--
-- Table structure for table `prizetype`
--

DROP TABLE IF EXISTS `prizetype`;
CREATE TABLE `prizetype` (
  `id` int UNSIGNED NOT NULL,
  `name` varchar(255) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `prizetype`
--

INSERT INTO `prizetype` (`id`, `name`) VALUES
(1, 'raha'),
(2, 'velka');

-- --------------------------------------------------------

--
-- Table structure for table `setup_extras`
--

DROP TABLE IF EXISTS `setup_extras`;
CREATE TABLE `setup_extras` (
  `id` int UNSIGNED NOT NULL,
  `extras_id` int UNSIGNED DEFAULT NULL,
  `text` varchar(1025) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `setup_extras`
--

INSERT INTO `setup_extras` (`id`, `extras_id`, `text`) VALUES
(0, NULL, NULL),
(1, NULL, '\"Boonit\" ja \"Will-o-Wisp\" mukaan'),
(2, NULL, '\"Hexit\" (ja \"Deluded\", \"Envious\" + \"Miserablet\") mukaan'),
(3, NULL, 'Heirloom: Haunted Mirror (Ghost mukaan)'),
(4, NULL, '\"Imp\"-pinkka mukaan'),
(5, NULL, '\"Lost in the Woods\" ja \"Boonit\" (ja \"Will-o-Wisp\") mukaan'),
(6, NULL, '\"Zombit\" mukaan'),
(7, NULL, '3 \"Boonia\" (ja tarvittaessa \"Will-o-Wisp\") mukaan'),
(8, NULL, '\"Bat\" ja \"Hex\" -pinkat mukaan (ja \"Deluded\", \"Envious\" + \"Miserablet\")'),
(9, NULL, '\"Wish\" ja \"Hex\" -pinkat mukaan (ja \"Deluded\", \"Envious\" + \"Miserablet\")'),
(10, NULL, '\"Ghost\"-pinkka mukaan'),
(11, NULL, 'Kaikki 3 \"Spirit\"-pinkkaa (Will-o-Wisp, Imp ja Ghost) mukaan'),
(12, NULL, 'Heirloom: Lucky Coin ja Boonit (ja \"Will-o-Wisp\") mukaan'),
(13, NULL, 'Heirloom: Goat ja Boonit (ja \"Will-o-Wisp\")'),
(14, NULL, 'Heirloom: Cursed Gold (Kiroukset mukaan)'),
(15, NULL, 'Heirloom: Magic Lamp (Wish:it mukaan)'),
(16, NULL, 'Heirloom: Pasture'),
(17, NULL, 'Heirloom: Pouch ja Boonit (ja \"Will-o-Wisp\").'),
(20, NULL, 'Tavernamatot peliin'),
(21, NULL, '\"Matka\" -pelimerkit mukaan.'),
(22, NULL, 'Ota Aarteenmetsästäjä, Soturi, Sankari ja Mestari mukaan'),
(23, NULL, 'Ota Sotilas, Pakolainen, Oppilas ja Opettaja mukaan. Tavernamatot.'),
(24, NULL, '\"-1 Raha\" -pelimerkit mukaan'),
(25, NULL, 'Omaa sukua Ile? (Exile)'),
(26, NULL, 'Hummat mukaan'),
(27, NULL, 'Velkaa varastopinoihin'),
(28, NULL, 'Valitkaa sattumanvaraisesti yksi (jo valituista) toimintakorttien varastopinoista'),
(29, NULL, 'Laittakaa maamerkin päälle 6 pts per pelaaja'),
(31, NULL, 'Laittakaa 2 pts jokaisen varastossa olevan, ei-keräävän, toimintakorttipinon päälle. Ottakaa kiroukset peliin.'),
(32, NULL, 'Laittakaa 8 pts kulta- ja hopeakorttipinon päälle.'),
(33, NULL, 'Velkamerkit mukaan'),
(34, NULL, '\"-2 Rahaa\" -pelimerkit mukaan'),
(35, NULL, '\"Tila\" -pelimerkit mukaan'),
(36, NULL, '\"+1 Toiminta\" -pelimerkit mukaan'),
(37, NULL, '\"+1 Kortti\" -pelimerkit mukaan'),
(38, NULL, '\"Tuhoa\" -pelimerkit mukaan'),
(39, NULL, '\"+1 Osto\" -pelimerkit mukaan'),
(40, NULL, '\"+1 Raha\" -pelimerkit mukaan'),
(41, NULL, '\"-1 Kortti\" -pelimerkit mukaan');

-- --------------------------------------------------------

--
-- Table structure for table `starter`
--

DROP TABLE IF EXISTS `starter`;
CREATE TABLE `starter` (
  `id` int NOT NULL,
  `aloittaja` varchar(1024) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `starter`
--

INSERT INTO `starter` (`id`, `aloittaja`) VALUES
(1, 'Pelaaja jonka puhelin on kauimpana'),
(2, 'Viimeksi puhunut pelaaja'),
(3, 'Pelaaja joka nukkui pisimpään'),
(4, 'Pelaaja joka heräsi ensin'),
(5, 'Isokenkäisin pelaaja'),
(6, 'Vähähiuksisin pelaaja'),
(7, 'Viimeksi autossa istunut pelaaja'),
(8, 'Viimeksi ruokaa laittanut pelaaja'),
(9, 'Viimeksi jotain purrut pelaaja'),
(10, 'Pitkähiuksisin pelaaja'),
(11, 'Lyhin pelaaja'),
(12, 'Viimeksi reppua kantanut pelaaja'),
(13, 'Viimeksi televisiota katsonut pelaaja'),
(14, 'Viimeksi hammaslääkärissä käynyt pelaaja'),
(15, 'Vähiten tietokonetta käyttävä pelaaja'),
(16, 'Pieninenäisin pelaaja'),
(17, 'Punahuulisin pelaaja'),
(18, 'Siloposkisin pelaaja'),
(19, 'Pitkäpartaisin pelaaja'),
(20, 'Pelaaja joka on kauimpana synnyinpaikkaansa'),
(21, 'Pelaaja joka on lähimpänä vanhempiaan'),
(22, 'Viimeksi WC:ssä käynyt pelaaja'),
(23, 'Viimeksi jotain laulanaut/hyräillyt pelaaja'),
(24, 'Viimeksi ulkona käynyt pelaaja'),
(25, 'Pelaaja jonka puhelimessa on vähiten akkua jäljellä'),
(26, 'Pelaaja joka haukotteli viimeksi'),
(27, 'Pelaaja joka on viimeksi tehnyt käsitöitä'),
(28, 'Pelaaja jolla on eniten värejä sukissaan'),
(29, 'Pelaaja joka on käynyt viimeksi liikuntahallissa');

-- --------------------------------------------------------

--
-- Table structure for table `suggestions`
--

DROP TABLE IF EXISTS `suggestions`;
CREATE TABLE `suggestions` (
  `id` int NOT NULL,
  `ehdotus` varchar(1024) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

--
-- Dumping data for table `suggestions`
--

INSERT INTO `suggestions` (`id`, `ehdotus`) VALUES
(1, 'Pelaaja jolla on vahvimmat silmälasit päässään'),
(2, 'Pelaaja jolla on eniten koruja'),
(4, 'Viimeksi palapeliä koonnut pelaaja'),
(5, 'Aaro\r\nOlli\r\nEmmi\r\nIida\r\nViivi\r\nJulia\r\nSamppa');

--
-- Indexes for dumped tables
--

--
-- Indexes for table `cards`
--
ALTER TABLE `cards`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `cardtype`
--
ALTER TABLE `cardtype`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `events`
--
ALTER TABLE `events`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `event_type`
--
ALTER TABLE `event_type`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `expansion`
--
ALTER TABLE `expansion`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `landmarks`
--
ALTER TABLE `landmarks`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `parsed_cards`
--
ALTER TABLE `parsed_cards`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `position`
--
ALTER TABLE `position`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `prizetype`
--
ALTER TABLE `prizetype`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `setup_extras`
--
ALTER TABLE `setup_extras`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `starter`
--
ALTER TABLE `starter`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `suggestions`
--
ALTER TABLE `suggestions`
  ADD PRIMARY KEY (`id`);

--
-- AUTO_INCREMENT for dumped tables
--

--
-- AUTO_INCREMENT for table `cards`
--
ALTER TABLE `cards`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=327;

--
-- AUTO_INCREMENT for table `cardtype`
--
ALTER TABLE `cardtype`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=5;

--
-- AUTO_INCREMENT for table `events`
--
ALTER TABLE `events`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=64;

--
-- AUTO_INCREMENT for table `event_type`
--
ALTER TABLE `event_type`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3;

--
-- AUTO_INCREMENT for table `expansion`
--
ALTER TABLE `expansion`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=19;

--
-- AUTO_INCREMENT for table `landmarks`
--
ALTER TABLE `landmarks`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=22;

--
-- AUTO_INCREMENT for table `parsed_cards`
--
ALTER TABLE `parsed_cards`
  MODIFY `id` int NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3373;

--
-- AUTO_INCREMENT for table `position`
--
ALTER TABLE `position`
  MODIFY `id` int NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=11;

--
-- AUTO_INCREMENT for table `prizetype`
--
ALTER TABLE `prizetype`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3;

--
-- AUTO_INCREMENT for table `setup_extras`
--
ALTER TABLE `setup_extras`
  MODIFY `id` int UNSIGNED NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=42;

--
-- AUTO_INCREMENT for table `starter`
--
ALTER TABLE `starter`
  MODIFY `id` int NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=30;

--
-- AUTO_INCREMENT for table `suggestions`
--
ALTER TABLE `suggestions`
  MODIFY `id` int NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=6;
COMMIT;

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
