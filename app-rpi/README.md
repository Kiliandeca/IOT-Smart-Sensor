#Mini-projet for 4IRC students at CPE Lyon

## Installing the server
In a new raspbian installation:
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install git python python-serial minicom
sudo usermod -aG dialout pi
```

Add a line into `rc.local`to start the game