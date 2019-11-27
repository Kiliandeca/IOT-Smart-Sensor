# Mini-Projet IoT

## 0 - Introduction

Ce mini-projet nous a été proposé dans le cadre du module **Architecture et protocoles réseaux pour IoT**. 
L'objectif de ce mini-projet est de créer une petite infrastructure connecté :
 * Un capteur d'humidité, de luminosité et de température sont branchés à un microcontrôleur, ces données sont affichées sur un écran LCD. 
 * Ce microcontrôleur envoie les données des capteurs par radio-fréquence à un second microcontrôleur appelé passerelle. La passerelle retransmet les données reçus au Raspberry Pi. 
 * Le Raspberry Pi retransmet également ces données (via un réseau Wifi) et un smartphone les reçoit et affiches les données à l'écran. 
 * Le smartphone est également capable de changer l'ordre d'affichage des données sur l'écran LCD du microcontrôleur.

*L'architecture du système est résumé dans le schéma ci-dessous.*

![alt text](https://image.noelshack.com/fichiers/2019/48/2/1574797645-untitled-diagram.png "Logo Title Text 1")

## 1 - Mise en place d'une infrastructure objet-passerelle

### Mise en place du réseau

#### Protocole de communication

Pour contrôler les communication, nous avons mis en place un protocole de communication. Voici le format de trame que notre protocole prévoie d'utiliser :

  * Adresse Source : Identifiant du µ-contrôleur émetteur du paquet
  * Adresse Destination : Identifiant du µ-contrôleur récepteur du paquet
  * Taille de la trame
  * Contrôle d'intégrité : on additionne les valeurs de chaque champs de la trame (sauf celui-ci) pour que la destination puisse vérifier l'intégrité de la trame
  * Données

#### Envoie des données

Pour envoyer les données sur le canal radio, nous avons créer un programme en C. Nous utilisons la librairie fournie pour ce projet. Il nous suffit de placer les données dans un buffer, que nous passons à une fonction de cette librairie.

Voici le code correspondant à l'envoi des donnnées :

#### Réception des données

Tout comme l'envoi, la réception des données est gérée grâce à une fonction de la librairie, qui va extraire la trame vers un buffer. Lors de la réception, on vérifie que l'adresse source 


#### Chiffrement

### Configuration des capteurs

#### Collecte des données

#### Affichage des données

#### Communications avec la passerelle - format des données

#### Communications avec la passerelle - envoyer données sur l'interface UART

### Configuration du serveur 

#### InfluxDB & Grafana

#### Hello back



## 2 - Création de l'application Android

### Choix d'affichage

### Choix du serveur destination

### Communication bidirectionnelle et filtrage des données reçus




### 1 - Le µ-Contrôleur avec ses capteurs
**Rôle :** le rôle de ce microcontrôleur est de récupérer les données des capteurs, les afficher sur un écran (l'affichage varie en fonction des ordres envoyé par un smartphone) et d'envoyer ses données via radio-fréquence à un autre microcontrôleur.
#### Fonctionnalités implémentées :
* 



#################################################################################################

#################################################################################################

#################################################################################################
* Les 3 capteurs envoi leurs données au microcontrôleur via le bus I2C. 
Ce microcontrôleur reçoit les données et les communique sur une liaison radio à un autre contrôleur identique.

##### Réception du bus I2C

##### Envoi sur liaison radio


### 2 - Le µ-contrôleur passerelle

C'est le deuxième µ-contrôleur qui sert de passerelle avec la raspberry π . Il traite les données reçues sur la liaison radio afin de les envoyer à la raspberry π et s'occupe de fonctionnalités comme le chiffrement.

##### Chiffrement

##### Réception sur la liaison radio

##### Envoi à la Raspberry π

### 3 - Le Raspberry π 

La Raspberry π nous sert de serveur applicatif, puisqu'il enregistre les données et embarque notamment Grafana afin de générer des vues de ses donnés. Il communique aussi avec l'application Android pour lui envoyé des données et recevoir des ordres. 

Pour se connecter à la respberry l'application envoi un message UDP "Hello", vous valider la connection la raspberry renvoi un message "HelloBack". 

##### Enregistrement des données
Les données reçus depuis le microcontroller sont en format JSON, nous les enregistrons sur une base de donnée InfluxDB. C'est une BDD spécialement conçue pour stocker des données horodatés. 

##### Tableau de bord Grafana
Nous avons mis en place un dashboard Grafana pour permettre la visualisation des données reçues (température, humidité, liminausité) enf onction du temps. Notre instance Grafana se connecte directement à la base de données InfluxDB pour récupérer les données. Nous pouvons visualiser l'évolution de la température, de l'humidité et de la luminosité en choissisant l'échelle (5 minutes, 1 heure, 1 journée).

Nous avons également ajouté les graphiques d'usages des ressources de la raspberry (RAM, espace dique, CPU). Cela nous permet de detecter facilement un problème de performance.


### 4 - L'Application Mobile

### Connection au server Raspberry


#### Affichage des données


#### Ordre d'affichage des données
Depuis l'application, l'utilisateur, peut choisir l'ordre dans lequel les données sont affichés. Les trois types de données sont affichés et l'utilisateur peut modifier l'ordre grâce à un simple glisser déposer. Cette modification est communiqué au serveur raspberry qui puis envoyer jusqu'au micro controller qui controler l'affichage OLED sur lequel les données seront affichés dans le nouvel ordre. 

### Retour d'expérience

### Conclusion
