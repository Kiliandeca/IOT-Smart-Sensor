This a game where a raspberry will listen to udp packages and light a ledstrip of neopixels.

It will listen for:
- "(0)" to reset the game
- "(1)" player 1 moves
- "(2)" player 2 moves

To install the libraries in a raspberry pi from zero you need to follow this procedure:
1. Install last version of raspbian
2. Issue a `sudo apt-get update && sudo apt-get upgrade`
3. Install git 
4. Clone the neopixel driver repository : `https://github.com/jgarff/rpi_ws281x` and follow the install instruction and those of python wrapper.
5. Connect GND and GPIO18 to the GND and data input of the ledstrip.
6. Run the game !
```
sudo python androidgame.py
```
7. If you want to start the game at each boot of the raspberry, put this line on `/etc/rc.local` just before the line `exit 0`
```
sudo /usr/bin/python /home/pi/git/4irc_aiot/stripled\ controller/androidgame.py &
```

### Reduce image size
To clone the SD card ready to play the game:
1. Create an image of the SD card:
```
sudo dd if=/dev/sdb of=~/SDCardBackup.img
```
2. Reduce the image size with `pishrink` : https://github.com/Drewsif/PiShrink
3. Write the image to the new SD card :
```
sudo dd bs=4M if=~/SDCardBackup.img of=/dev/sdb
``` 


