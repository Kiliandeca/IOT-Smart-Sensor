
# Mini-Projet IoT

## 0 - Introduction

Ce mini-projet nous a été proposé dans le cadre du module **Architecture et protocoles réseaux pour IoT**. 
L'objectif de ce mini-projet est de créer une petite infrastructure connecté comprenant :
 * Un capteur d'humidité, de luminosité et de température sont branchés à un microcontrôleur, ces données sont affichées sur un écran LCD. 
 * Ce microcontrôleur envoie les données des capteurs par radio-fréquence à un second microcontrôleur appelé passerelle. La passerelle retransmet les données reçus au Raspberry Pi. 
 * Le Raspberry Pi retransmet également ces données (via un réseau Wifi) et un smartphone les reçoit et affiches les données à l'écran. 
 * Le smartphone est également capable de changer l'ordre d'affichage des données sur l'écran LCD du microcontrôleur.

*L'architecture du système est résumé dans le schéma ci-dessous.*

![alt text](https://image.noelshack.com/fichiers/2019/48/2/1574797645-untitled-diagram.png "Logo Title Text 1")

## I - Mise en place d'une infrastructure objet-passerelle

### 1 - Mise en place du réseau

#### 1.1 Protocole de communication

Pour contrôler les communications, nous avons mis en place un protocole de communication. Voici le format de trame que notre protocole prévoie d'utiliser :

  * Adresse Source : Identifiant du µ-contrôleur émetteur du paquet
  * Adresse Destination : Identifiant du µ-contrôleur récepteur du paquet
  * Taille de la trame
  * Contrôle d'intégrité : on additionne les valeurs de chaque champs de la trame (sauf celui-ci) pour que la destination puisse vérifier l'intégrité de la trame
  * Données variables : valeur des capteurs ou ordre d'affichage des données

#### 1.2 Envoie des données

Pour envoyer les données en radio fréquence, nous avons créer un programme en C. Nous utilisons la librairie fournie pour ce projet. Il nous suffit de placer les données dans un buffer, que nous passons à une fonction de cette librairie.

#### 1.3 Réception des données

Tout comme l'envoi, la réception des données est gérée grâce à une fonction de la librairie, qui va extraire le contenu de la trame vers un buffer.
Voici les étapes effectuées à la réception d'un message :
* Calcule du CRC : Taille du paquet + Adresse destination + Adresse source
* Contrôle de l'adresse source / destination
* Contrôle du CRC reçu avec le CRC calculé
* Copie des données dans un buffer
* Sauvegarde des données dans des variables

#### 1.4 Chiffrement
Nous avons eus beaucoup de mal a implémenter un chiffrement correcte, après de nombreuses tentatives pour utiliser des algorithmes AES nous avons mis en place un algorithme de chiffrement Cesar.
Ensuite après de nombreuses recherches en collaboration avec un autre groupe nous avons réussis à utiliser un chiffrement AES pour l'envoie des données.
Données chiffrées :
* Les valeurs des capteurs
* L'ordre d'affichage "THL"

### 2 - Configuration des capteurs

#### 2.1 Collecte des données
Nous avons utilisé le code présent dans le Git "modules-techno-innov" pour réaliser la récupération des données du capteur.
Les anciens TP réalisé nous ont également permis de réussir cette partie rapidement.
Le capteur récupère les données suivantes :
* Humidité
* Lumière
* Température

#### 2.2 Affichage des données
Nous avons utilisé le code présent dans le Git "modules-techno-innov" pour réaliser l'écrire des données sur l'écran LCD.
Nous avons ajouté au code existant les fonctionnalités suivantes :
* Fonction "shiftLeft" : permet de décaler une liste de char --> pour afficher un text déffilant sur l'écran
* Fonction "gestionAffichage" : permet de spécifier l'ordre des éléments à afficher.

#### 2.3 Communications avec la passerelle - format des données
La communication avec la passerelle est en radio-fréquence.
Le format des données envoyés :
[Taille total paquet] [Adresse source] [Adresse destination] [CRC] [Données]
* Le CRC est calculé avec les trois premiers champs (mis en place avant l'envoie des données, nous avons oublié de les ajoutées au CRC)

#### 2.4 Communications avec la passerelle - envoyer données sur l'interface UART
Nous avons utilisé le code présent dans le Git "modules-techno-innov" pour réaliser la lecture des données sur l'interface UART0.
Nous avons ajouté un bout de code permettant de signaler la fin de lecture sur l'UART dès que 3 caractères ont été reçus, ensuite nous récupérons des données dans le buffer puis nous réinitialisation les compteurs.

#### 2.4 Communications avec la passerelle - lire données sur l'interface UART
Le microcontrôleur branché au Raspberry Pi envoie les données reçu sur UART au format JSON.
Données JSON :
```json
{ 
	"Lux" : 200
	"Temp" : 23,5
	"Humidity" : 65,5
}
```

### 3 - Configuration du serveur 
La Raspberry π nous sert de serveur applicatif, il enregistre les données et embarque notamment Grafana afin de générer des vues de ses donnés. Il communique aussi avec l'application Android pour lui envoyé des données et recevoir l'ordre d'affichage. 

#### 3.1 Enregistrement des données
Les données reçues depuis le microcontroller sont en format JSON, nous les enregistrons sur une base de données InfluxDB. C'est une BDD spécialement conçue pour stocker des données horodatés. 

#### 3.2 InfluxDB & Grafana : Tableaux de bord
Nous avons mis en place un dashboard Grafana pour permettre la visualisation des données reçues (température, humidité, liminausité) en fonction du temps. Notre instance Grafana se connecte directement à la base de données InfluxDB pour récupérer les données. Nous pouvons visualiser l'évolution de la température, de l'humidité et de la luminosité en choissisant l'échelle (5 minutes, 1 heure, 1 journée).

Nous avons également ajouté les graphiques d'usages des ressources de la raspberry (RAM, espace dique, CPU). Cela nous permet de détecter facilement un problème de performance.

#### 3.3 Message "HelloBack"
Pour commencer à utiliser
Pour se connecter à la Raspberry l'application envoi un message UDP "Hello", vous valider la connection la raspberry renvoi un message "HelloBack". 


## II - Création de l'application Android

### 1 - Choix d'affichage
Une fois la connexion effectuée, nous affichons les données du capteurs sur l'application et nous faisons des requêtes au serveur pour actualiser les données affichées.
Depuis l'application, l'utilisateur peut choisir l'ordre dans lequel les données sont affichés. Les trois types de données sont affichés et l'utilisateur peut modifier l'ordre grâce à un simple glisser-déposer. Cette modification est communiquée au serveur raspberry puis envoyé jusqu'au micro controller qui controle l'affichage OLED sur lequel les données seront affichés dans le nouvel ordre. 

### 2 - Choix du serveur destination
Pour se connecter au server, l'utilisateur saisie l'adresse et le port. L'application va ensuite vérifier que ces informations sont valident en envoyent le message UDP "Hello". Si le serveur répond bien par "HelloBack" alors la connection est réussite.

### 3 - Communication bidirectionnelle et filtrage des données reçus



### III - Retour d'expérience

### IV - Conclusion
