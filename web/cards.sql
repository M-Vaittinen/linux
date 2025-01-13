-- MySQL dump 10.13  Distrib 8.0.40, for Linux (x86_64)
--
-- Host: localhost    Database: dominion
-- ------------------------------------------------------
-- Server version	8.0.40

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
  `expansion_id` int unsigned DEFAULT NULL,
  `type_id` int unsigned NOT NULL,
  `prizetype_id` int unsigned NOT NULL,
  `prize` int unsigned NOT NULL,
  `attack` tinyint(1) DEFAULT NULL,
  `defence` tinyint(1) DEFAULT NULL,
  `endure` tinyint(1) DEFAULT '0',
  `gather` tinyint(1) DEFAULT '0',
  `destroy` tinyint(1) DEFAULT '0',
  `curse` tinyint(1) DEFAULT '0',
  `tuhinakerroin` int unsigned DEFAULT '0',
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=86 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cards`
--

LOCK TABLES `cards` WRITE;
/*!40000 ALTER TABLE `cards` DISABLE KEYS */;
INSERT INTO `cards` VALUES (1,0,0,'Rakennusmestari',11,2,2,4,0,0,0,0,0,0,0),(2,0,0,'Kaupunginosa',11,2,2,8,0,0,0,0,0,0,8),(3,0,0,'Ylipäällikkö',11,2,2,8,0,0,0,0,0,0,2),(4,0,0,'Kuninkaanseppä',11,2,2,8,0,0,0,0,0,0,0),(6,7,0,'Leiri',11,2,1,2,0,0,0,0,0,0,5),(7,0,6,'Ryöstösaalis',11,1,1,5,0,0,0,0,0,0,0),(8,9,0,'Patriisi',11,2,1,2,0,0,0,0,0,0,3),(9,0,8,'Markkinapaikka',11,2,1,5,0,0,0,0,0,0,4),(10,11,0,'Uudisasukkaat',11,2,1,2,0,0,0,0,0,0,3),(11,0,10,'Vilkas kylä',11,2,1,5,0,0,0,0,0,0,5),(12,13,0,'Katapultti',11,2,1,3,1,0,0,0,1,1,0),(13,0,12,'Kivet',11,1,1,4,0,0,0,0,0,0,0),(14,0,0,'Linnat',11,4,1,3,0,0,0,0,0,0,0),(15,0,0,'Vaunukisat',11,2,1,3,0,0,0,0,0,0,1),(16,0,0,'Lumoajatar',11,2,1,3,1,0,1,0,0,0,1),(17,0,0,'Maalaismarkkinat',11,2,1,3,0,0,0,1,0,0,0),(18,19,0,'Gladiaattori',11,2,1,3,0,0,0,0,0,0,0),(19,0,18,'Palkinto',11,1,2,16,0,0,0,0,0,0,0),(20,0,0,'Uhraus',11,2,1,4,0,0,0,0,1,0,2),(21,0,0,'Temppeli',11,2,1,4,0,0,0,1,1,0,0),(22,0,0,'Huvila',11,2,1,4,0,0,0,0,0,0,9),(23,0,0,'Arkisto',11,2,1,5,0,0,1,0,0,0,2),(24,0,0,'Pääoma',11,1,1,5,0,0,0,0,0,0,0),(25,0,0,'Onnenkalu',11,1,1,5,0,0,0,0,0,0,0),(26,0,0,'Kruunu',11,3,1,5,0,0,0,0,0,0,0),(27,0,0,'Foorumi',11,2,1,5,0,0,0,0,0,0,3),(28,0,0,'Henkijahti',11,2,1,5,0,0,0,1,0,0,0),(29,0,0,'Legioonalainen',11,2,1,5,1,0,0,0,0,0,0),(30,0,0,'Puutarhuri',11,2,1,5,0,0,0,0,0,0,2),(31,0,0,'Byrokraatti',1,2,1,4,1,0,0,0,0,0,0),(32,0,0,'Juhlat',1,2,1,5,0,0,0,0,0,0,4),(33,0,0,'Kaivos',1,2,1,5,0,0,0,0,1,0,0),(34,0,0,'Kansleri',1,2,1,3,0,0,0,0,0,0,0),(35,0,0,'Kappeli',1,2,1,2,0,0,0,0,1,0,0),(36,0,0,'Kellari',1,2,1,2,0,0,0,0,0,0,2),(37,0,0,'Kirjasto',1,2,1,5,0,0,0,0,0,0,0),(38,0,0,'Koronkiskuri',1,2,1,4,0,0,0,0,1,0,0),(39,0,0,'Kylä',1,2,1,3,0,0,0,0,0,0,4),(40,0,0,'Laboratorio',1,2,1,5,0,0,0,0,0,0,3),(41,0,0,'Metsuri',1,2,1,3,0,0,0,0,0,0,0),(42,0,0,'Muutostyö',1,2,1,4,0,0,0,0,1,0,0),(43,0,0,'Noita',1,2,1,5,1,0,0,0,0,1,0),(44,0,0,'Nostoväki',1,2,1,4,1,0,0,0,0,0,0),(45,0,0,'Pidot',1,2,1,4,0,0,0,0,1,0,0),(46,0,0,'Puutarha',1,4,1,4,0,0,0,0,0,0,0),(47,0,0,'Raatihuone',1,2,1,5,0,0,0,0,0,0,2),(48,0,0,'Seikkailija',1,2,1,6,0,0,0,0,0,0,0),(49,0,0,'Takomo',1,2,1,4,0,0,0,0,0,0,0),(50,0,0,'Tori',1,2,1,5,0,0,0,0,0,0,0),(51,0,0,'Työpaja',1,2,1,3,0,0,0,0,0,0,0),(52,0,0,'Vakooja',1,2,1,4,1,0,0,0,0,0,0),(53,0,0,'Vallihauta',1,2,1,2,0,1,0,0,0,0,0),(54,0,0,'Valtaistuinsali',1,2,1,4,0,0,0,0,0,0,3),(55,0,0,'Varas',1,2,1,4,1,0,0,0,1,0,3),(56,0,0,'Spurgutyö',17,2,1,4,0,0,0,0,1,0,0),(57,0,0,'Linnoitettu-kylä',17,2,1,4,0,0,0,0,0,0,4),(58,0,0,'Mustapörssi',17,2,1,3,0,0,0,0,0,0,0),(59,0,0,'Laina',5,1,1,3,0,0,0,0,1,0,0),(60,0,0,'Kauppareitti',5,2,1,3,0,0,0,1,1,0,0),(61,0,0,'Vartiotorni',5,2,1,3,0,1,0,0,1,0,0),(62,0,0,'Piispa',5,2,1,4,0,0,0,0,1,0,0),(63,0,0,'Monumentti',5,2,1,4,0,0,0,0,0,0,0),(64,0,0,'Louhos',5,1,1,4,0,0,0,0,0,0,1),(65,0,0,'Talismaani',5,1,1,4,0,0,0,0,0,0,0),(66,0,0,'Työläiskylä',5,2,1,4,0,0,0,0,0,0,4),(67,0,0,'Kaupunki',5,2,1,5,0,0,0,0,0,0,5),(68,0,0,'Hämärät varat',5,1,1,5,0,0,0,0,0,0,0),(69,0,0,'Tilitoimisto',5,2,1,5,0,0,0,0,0,0,0),(70,0,0,'Rahapaja',5,2,1,5,0,0,0,0,1,0,0),(71,0,0,'Petkuttaja',5,2,1,5,1,0,0,0,0,1,0),(72,0,0,'Rahvas',5,2,1,5,1,0,0,0,0,0,0),(73,0,0,'Sinetti',5,1,1,5,0,0,0,0,0,0,0),(74,0,0,'Holvi',5,2,1,5,0,0,0,0,0,0,1),(75,0,0,'Palkkio',5,1,1,5,0,0,0,0,0,0,0),(76,0,0,'Roistot',5,2,1,6,1,0,0,0,0,0,0),(78,0,0,'Aarre',5,1,1,6,0,0,0,0,0,0,0),(80,0,0,'Suurtori',5,2,1,6,0,0,0,0,0,0,3),(81,0,0,'Pankki',5,1,1,7,0,0,0,0,0,0,0),(82,0,0,'Laajennus',5,2,1,7,0,0,0,0,1,0,0),(83,0,0,'Ahjo',5,2,1,7,0,0,0,0,1,0,0),(84,0,0,'Kuninkaan Hovi',5,2,1,7,0,0,0,0,0,0,0),(85,0,0,'Kulkukauppias',5,2,1,8,0,0,0,0,0,0,3);
/*!40000 ALTER TABLE `cards` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `cardtype`
--

DROP TABLE IF EXISTS `cardtype`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `cardtype` (
  `id` int unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `cardtype`
--

LOCK TABLES `cardtype` WRITE;
/*!40000 ALTER TABLE `cardtype` DISABLE KEYS */;
INSERT INTO `cardtype` VALUES (1,'raha'),(2,'toiminto'),(3,'raha/toiminto'),(4,'sekalainen');
/*!40000 ALTER TABLE `cardtype` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `expansion`
--

DROP TABLE IF EXISTS `expansion`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `expansion` (
  `id` int unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(255) NOT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=18 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `expansion`
--

LOCK TABLES `expansion` WRITE;
/*!40000 ALTER TABLE `expansion` DISABLE KEYS */;
INSERT INTO `expansion` VALUES (1,'Valtakunta'),(2,'Hovin Juonet'),(3,'Kaukaiset rannat'),(4,'Alkemia'),(5,'Nousukausi'),(6,'Elonkorjuu'),(7,'Rajaseudut'),(8,'Katovuodet'),(9,'Killat'),(10,'Seikkailut'),(11,'Keisarikunta'),(12,'Nocturne'),(13,'Renaissance'),(14,'Menagerie'),(15,'Allies'),(16,'Plunder'),(17,'Extras');
/*!40000 ALTER TABLE `expansion` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `prizetype`
--

DROP TABLE IF EXISTS `prizetype`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `prizetype` (
  `id` int unsigned NOT NULL AUTO_INCREMENT,
  `name` varchar(255) DEFAULT NULL,
  PRIMARY KEY (`id`)
) ENGINE=InnoDB AUTO_INCREMENT=3 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `prizetype`
--

LOCK TABLES `prizetype` WRITE;
/*!40000 ALTER TABLE `prizetype` DISABLE KEYS */;
INSERT INTO `prizetype` VALUES (1,'raha'),(2,'velka');
/*!40000 ALTER TABLE `prizetype` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-01-11 20:23:02
