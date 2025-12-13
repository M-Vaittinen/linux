-- MySQL dump 10.13  Distrib 8.0.41, for Linux (x86_64)
--
-- Host: localhost    Database: dominion
-- ------------------------------------------------------
-- Server version	8.0.41

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `cards`
--

DROP TABLE IF EXISTS `cards`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `cards` (
  `id` int unsigned NOT NULL AUTO_INCREMENT,
  `dual_top_of_id` int unsigned DEFAULT '0',
  `dual_below_id` int unsigned DEFAULT '0',
  `name` varchar(255) NOT NULL,
  `en_name` varchar(255) DEFAULT NULL,
  `expansion_id` int unsigned DEFAULT NULL,
  `type_id` int unsigned NOT NULL,
  `prizetype_id` int unsigned NOT NULL,
  `prize` int unsigned NOT NULL,
  `drawcards` int DEFAULT '0',
  `attack` tinyint(1) DEFAULT NULL,
  `defence` tinyint(1) DEFAULT NULL,
  `endure` tinyint(1) DEFAULT '0',
  `gather` tinyint(1) DEFAULT '0',
  `destroy` tinyint(1) DEFAULT '0',
  `curse` tinyint(1) DEFAULT '0',
  `tuhinakerroin` int unsigned DEFAULT '0',
  `dropcards` tinyint(1) DEFAULT '0',
  `actionmoney` int unsigned DEFAULT '0',
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=86 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cards`
--

LOCK TABLES `cards` WRITE;
/*!40000 ALTER TABLE `cards` DISABLE KEYS */;
INSERT INTO `cards` VALUES (1,0,0,'Rakennusmestari','Engineer',11,2,2,4,0,0,0,0,0,0,0,0,0,0),(2,0,0,'Kaupunginosa','City Quarter',11,2,2,8,0,0,0,0,0,0,0,8,0,0),(3,0,0,'Ylipäällikkö','Overlord',11,2,2,8,0,0,0,0,0,0,0,2,0,0),(4,0,0,'Kuninkaanseppä','Royal Blacksmith',11,2,2,8,5,0,0,0,0,0,0,0,0,0),(6,7,0,'Leiri','Encampment',11,2,1,2,2,0,0,0,0,0,0,5,0,0),(7,0,6,'Ryöstösaalis','Plunder',11,1,1,5,0,0,0,0,0,0,0,0,0,0),(8,9,0,'Patriisi','Patrician',11,2,1,2,1,0,0,0,0,0,0,3,0,0),(9,0,8,'Markkinapaikka','Emporium',11,2,1,5,1,0,0,0,0,0,0,4,0,1),(10,11,0,'Uudisasukkaat','Settlers',11,2,1,2,1,0,0,0,0,0,0,3,0,0),(11,0,10,'Vilkas kylä','Bustling Village',11,2,1,5,1,0,0,0,0,0,0,5,0,0),(12,13,0,'Katapultti','Catapult',11,2,1,3,0,1,0,0,0,1,1,0,1,1),(13,0,12,'Kivet','Rocks',11,1,1,4,0,0,0,0,0,0,0,0,0,0),(14,0,0,'Linnat',NULL,11,4,1,3,0,0,0,0,0,0,0,0,0,1),(15,0,0,'Vaunukisat','Chariot Race',11,2,1,3,0,0,0,0,0,0,0,1,0,0),(16,0,0,'Lumoajatar','Enchantress',11,2,1,3,2,1,0,1,0,0,0,1,0,0),(17,0,0,'Maalaismarkkinat','Farmers\' Market',11,2,1,3,0,0,0,0,1,0,0,0,0,2),(18,19,0,'Gladiaattori','Gladiator',11,2,1,3,0,0,0,0,0,0,0,0,0,2),(19,0,18,'Palkinto','Fortune',11,1,2,16,0,0,0,0,0,0,0,0,0,0),(20,0,0,'Uhraus','Sacrifice',11,2,1,4,1,0,0,0,0,1,0,2,0,1),(21,0,0,'Temppeli','Temple',11,2,1,4,0,0,0,0,1,1,0,0,0,0),(22,0,0,'Huvila','Villa',11,2,1,4,0,0,0,0,0,0,0,9,0,1),(23,0,0,'Arkisto','Archive',11,2,1,5,0,0,0,1,0,0,0,2,0,0),(24,0,0,'Pääoma','Capital',11,1,1,5,0,0,0,0,0,0,0,0,0,0),(25,0,0,'Onnenkalu','Charm',11,1,1,5,0,0,0,0,0,0,0,0,0,0),(26,0,0,'Kruunu','Crown',11,3,1,5,0,0,0,0,0,0,0,0,0,0),(27,0,0,'Foorumi','Forum',11,2,1,5,3,0,0,0,0,0,0,3,0,0),(28,0,0,'Henkijahti','Wild Hunt',11,2,1,5,3,0,0,0,1,0,0,0,0,0),(29,0,0,'Legioonalainen','Legionary',11,2,1,5,0,1,0,0,0,0,0,0,1,3),(30,0,0,'Puutarhuri','Groundskeeper',11,2,1,5,1,0,0,0,0,0,0,2,0,0),(31,0,0,'Byrokraatti','Bureaucrat',1,2,1,4,0,1,0,0,0,0,0,0,0,0),(32,0,0,'Juhlat','Festival',1,2,1,5,0,0,0,0,0,0,0,4,0,2),(33,0,0,'Kaivos','Mine',1,2,1,5,0,0,0,0,0,1,0,0,0,0),(34,0,0,'Kansleri','Chancellor',1,2,1,3,0,0,0,0,0,0,0,0,0,2),(35,0,0,'Kappeli','Chapel',1,2,1,2,0,0,0,0,0,1,0,0,0,0),(36,0,0,'Kellari','Cellar',1,2,1,2,0,0,0,0,0,0,0,2,0,0),(37,0,0,'Kirjasto','Library',1,2,1,5,3,0,0,0,0,0,0,0,0,0),(38,0,0,'Koronkiskuri','Moneylender',1,2,1,4,0,0,0,0,0,1,0,0,0,3),(39,0,0,'Kylä','Village',1,2,1,3,1,0,0,0,0,0,0,4,0,0),(40,0,0,'Laboratorio','Laboratory',1,2,1,5,2,0,0,0,0,0,0,3,0,0),(41,0,0,'Metsuri','Woodcutter',1,2,1,3,0,0,0,0,0,0,0,0,0,2),(42,0,0,'Muutostyö','Remodel',1,2,1,4,0,0,0,0,0,1,0,0,0,0),(43,0,0,'Noita','Witch',1,2,1,5,2,1,0,0,0,0,1,0,0,0),(44,0,0,'Nostoväki','Militia',1,2,1,4,0,1,0,0,0,0,0,0,1,2),(45,0,0,'Pidot','Feast',1,2,1,4,0,0,0,0,0,1,0,0,0,0),(46,0,0,'Puutarha','Gardens',1,4,1,4,0,0,0,0,0,0,0,0,0,0),(47,0,0,'Raatihuone','Council Room',1,2,1,5,4,0,0,0,0,0,0,2,0,0),(48,0,0,'Seikkailija','Adventurer',1,2,1,6,1,0,0,0,0,0,0,0,0,0),(49,0,0,'Takomo','Smithy',1,2,1,4,3,0,0,0,0,0,0,0,0,0),(50,0,0,'Tori','Market',1,2,1,5,1,0,0,0,0,0,0,0,0,1),(51,0,0,'Työpaja','Workshop',1,2,1,3,0,0,0,0,0,0,0,0,0,0),(52,0,0,'Vakooja','Spy',1,2,1,4,1,1,0,0,0,0,0,0,0,0),(53,0,0,'Vallihauta','Moat',1,2,1,2,2,0,1,0,0,0,0,0,0,0),(54,0,0,'Valtaistuinsali','Throne Room',1,2,1,4,0,0,0,0,0,0,0,3,0,0),(55,0,0,'Varas','Thief',1,2,1,4,0,1,0,0,0,1,0,3,0,0),(56,0,0,'Spurgutyö','Dismantle',17,2,1,4,0,0,0,0,0,1,0,0,0,0),(57,0,0,'Linnoitettu-kylä','Walled Village',17,2,1,4,0,0,0,0,0,0,0,4,0,0),(58,0,0,'Mustapörssi','Black Market',17,2,1,3,0,0,0,0,0,0,0,0,0,0),(59,0,0,'Laina','Loan',5,1,1,3,0,0,0,0,0,1,0,0,0,0),(60,0,0,'Kauppareitti','Trade Route',5,2,1,3,0,0,0,0,1,1,0,0,0,2),(61,0,0,'Vartiotorni','Watchtower',5,2,1,3,1,0,1,0,0,1,0,0,0,0),(62,0,0,'Piispa','Bishop',5,2,1,4,0,0,0,0,0,1,0,0,0,1),(63,0,0,'Monumentti','Monument',5,2,1,4,0,0,0,0,0,0,0,0,0,2),(64,0,0,'Louhos','Quarry',5,1,1,4,0,0,0,0,0,0,0,1,0,0),(65,0,0,'Talismaani','Talisman',5,1,1,4,0,0,0,0,0,0,0,0,0,0),(66,0,0,'Työläiskylä','Worker\'s Village',5,2,1,4,1,0,0,0,0,0,0,4,0,0),(67,0,0,'Kaupunki','City',5,2,1,5,1,0,0,0,0,0,0,5,0,0),(68,0,0,'Hämärät varat','Contraband',5,1,1,5,0,0,0,0,0,0,0,0,0,0),(69,0,0,'Tilitoimisto','Counting House',5,2,1,5,0,0,0,0,0,0,0,0,0,0),(70,0,0,'Rahapaja','Mint',5,2,1,5,0,0,0,0,0,1,0,0,0,0),(71,0,0,'Petkuttaja','Mountebank',5,2,1,5,0,1,0,0,0,0,1,0,0,2),(72,0,0,'Rahvas','Rabble',5,2,1,5,3,1,0,0,0,0,0,0,1,0),(73,0,0,'Sinetti','Royal Seal',5,1,1,5,0,0,0,0,0,0,0,0,0,0),(74,0,0,'Holvi','Vault',5,2,1,5,2,0,0,0,0,0,0,1,0,1),(75,0,0,'Palkkio','Venture',5,1,1,5,0,0,0,0,0,0,0,0,0,0),(76,0,0,'Roistot','Goons',5,2,1,6,0,1,0,0,0,0,0,0,1,2),(78,0,0,'Aarre','Hoard',5,1,1,6,0,0,0,0,0,0,0,0,0,0),(80,0,0,'Suurtori','Grand Market',5,2,1,6,1,0,0,0,0,0,0,3,0,2),(81,0,0,'Pankki','Bank',5,1,1,7,0,0,0,0,0,0,0,0,0,0),(82,0,0,'Laajennus','Expand',5,2,1,7,0,0,0,0,0,1,0,0,0,0),(83,0,0,'Ahjo','Forge',5,2,1,7,0,0,0,0,0,1,0,0,0,0),(84,0,0,'Kuninkaan Hovi','King\'s Court',5,2,1,7,0,0,0,0,0,0,0,0,0,0),(85,0,0,'Kulkukauppias','Peddler',5,2,1,8,1,0,0,0,0,0,0,3,0,1);
/*!40000 ALTER TABLE `cards` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-05-05 15:05:27
