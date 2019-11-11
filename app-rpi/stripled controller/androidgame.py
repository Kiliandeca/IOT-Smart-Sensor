#NeoPixel library strandtest example
# Author: Tony DiCola (tony@tonydicola.com)
#
# Direct port of the Arduino NeoPixel library strandtest example.  Showcases
# various animations on a strip of NeoPixels.
import time
import argparse
import signal
import sys
import socket

from neopixel import *

def signal_handler(signal, frame):
    colorWipe(strip, Color(0, 0, 0))
    sys.exit(0)

def opt_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', action='store_true', help='clear the display on exit')
    args = parser.parse_args()
    if args.c:
        signal.signal(signal.SIGINT, signal_handler)

# LED strip configuration:
LED_COUNT      = 30      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53
#LED_STRIP      = ws.WS2811_STRIP_GRB   # Strip type and colour ordering
LED_STRIP       = ws.SK6812_STRIP_RGBW
UDP_PORT       = 10000
PLAYER1_ADDRESS = ("127.0.0.1", UDP_PORT)
PLAYER2_ADDRESS= ("127.0.0.1", UDP_PORT)
STEP           = .5
white = Color(127, 127, 127)
blue = Color(0, 0, 127)
green = Color(0, 127, 0)
# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=50):
        """Wipe color across display a pixel at a time."""
        for i in range(strip.numPixels()):
                strip.setPixelColor(i, color)
                strip.show()
                time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
        """Movie theater light style chaser animation."""
        for j in range(iterations):
                for q in range(3):
                        for i in range(0, strip.numPixels(), 3):
                                strip.setPixelColor(i+q, color)
                        strip.show()
                        time.sleep(wait_ms/1000.0)
                        for i in range(0, strip.numPixels(), 3):
                                strip.setPixelColor(i+q, 0)

def wheel(pos):
        """Generate rainbow colors across 0-255 positions."""
        if pos < 85:
                return Color(pos * 3, 255 - pos * 3, 0)
        elif pos < 170:
                pos -= 85
                return Color(255 - pos * 3, 0, pos * 3)
        else:
                pos -= 170
                return Color(0, pos * 3, 255 - pos * 3)

def rainbow(strip, wait_ms=20, iterations=1):
        """Draw rainbow that fades across all pixels at once."""
        for j in range(256*iterations):
                for i in range(strip.numPixels()):
                        strip.setPixelColor(i, wheel((i+j) & 255))
                strip.show()
                time.sleep(wait_ms/1000.0)

def rainbowCycle(strip, wait_ms=20, iterations=5):
        """Draw rainbow that uniformly distributes itself across all pixels."""
        for j in range(256*iterations):
                for i in range(strip.numPixels()):
                        strip.setPixelColor(i, wheel((int(i * 256 / strip.numPixels()) + j) & 255))
                strip.show()
                time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50):
        """Rainbow movie theater light style chaser animation."""
        for j in range(256):
                for q in range(3):
                        for i in range(0, strip.numPixels(), 3):
                                strip.setPixelColor(i+q, wheel((i+j) % 255))
                        strip.show()
                        time.sleep(wait_ms/1000.0)
                        for i in range(0, strip.numPixels(), 3):
                                strip.setPixelColor(i+q, 0)

def initGame(strip):
        theaterChase(strip, Color(0, 0, 127)) # Blue theater chase
        clearStrip(strip)
        print ('Number of Pixels: ')
        print (str(strip.numPixels()))
        if strip.numPixels()%2 == 0:
                strip.setPixelColor(strip.numPixels()/2, white)
                strip.setPixelColor(strip.numPixels()/2-1, white)
                strip.setPixelColor(0, blue)
                strip.setPixelColor(strip.numPixels()-1, green)
        else:
                strip.setPixelColor(strip.numPixels()/2, white)
        strip.show()
        return (strip.numPixels()-1)/2.0

def setWinner(strip,winner):
        if winner==1:
                colorWipe(strip,blue)
                colorWipe(strip,blue)
                colorWipe(strip,blue)
                sendMessageToClient("(1)",PLAYER1_ADDRESS)
                sendMessageToClient("(0)",PLAYER2_ADDRESS)
        else:
                colorWipe(strip,green)
                colorWipe(strip,green)
                colorWipe(strip,green)
                sendMessageToClient("(0)",PLAYER1_ADDRESS)
                sendMessageToClient("(1)",PLAYER2_ADDRESS)
        return initGame(strip)

def sendMessageToClient(msg,udp_address):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.sendto(msg,udp_address)
        sock.close()

def clearStrip(strip):
        for i in range(strip.numPixels()):
                strip.setPixelColor(i,Color(0,0,0))

def move(strip, position,sens,wait_ms=50):
        position=position + (sens * STEP)
        if position.is_integer:
                clearStrip(strip)
                strip.setPixelColor(int(position), white)
                strip.setPixelColor(0, blue)
                strip.setPixelColor(strip.numPixels()-1, green)
                strip.show()
                time.sleep(wait_ms/1000.0)
                if int(position) == 0:
                        position=setWinner(strip, 1)
                elif int(position) == strip.numPixels()-1:
                        position=setWinner(strip, 2)
        return position

# Main program logic follows:
if __name__ == '__main__':
        # Process arguments
        opt_parse()

        # Create NeoPixel object with appropriate configuration.
        strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP)
        # Intialize the library (must be called once before other functions).
        strip.begin()

        print ('Press Ctrl-C to quit.')
        pair = strip.numPixels()%2 == 0
        position = initGame(strip)
        tour = 0
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server_address = ('', UDP_PORT)
        sock.bind(server_address)
        print("Listening on port ", str(UDP_PORT))
        # socket.listen(5)
        while True:
                print("Position: " + str(position))
                # client, address = socket.accept()
                response, address = sock.recvfrom(4096)
                print "{} connected".format( address )
                # response = client.recv(255)
                if response != "":
                        print(response)
                        if response == "(0)": # Reset game
                                print("The game will be restarted")
                                position=initGame(strip)
                                tour = 0
                        elif response == "(1)": # Player 1 moved
                                print("Player 1 moved")
                                print("Type address: " + str(type(address)))
                                PLAYER1_ADDRESS=address
                                print ("Type PLAYER1_ADDRESS: " + str(type(PLAYER1_ADDRESS)))
                                position=move(strip,position,-1)
                        elif response == "(2)": # Player 2 moved
                                print("Player 2 moved")
                                print("Type address: " + str(type(address)))
                                PLAYER2_ADDRESS=address
                                print ("Type PLAYER2_ADDRESS: " + str(type(PLAYER2_ADDRESS)))
                                position = move(strip,position,1)
                        else:
                                print("Unknown message: ",response)
        # client.close()
        sock.close()

        # while True:
        #       print ('Color wipe animations.')
        #       colorWipe(strip, Color(255, 0, 0))  # Red wipe
        #       colorWipe(strip, Color(0, 255, 0))  # Blue wipe
        #       colorWipe(strip, Color(0, 0, 255))  # Green wipe
        #       print ('Theater chase animations.')
        #       theaterChase(strip, Color(127, 127, 127))  # White theater chase
        #       theaterChase(strip, Color(127,   0,   0))  # Red theater chase
        #       theaterChase(strip, Color(  0,   0, 127))  # Blue theater chase
        #       print ('Rainbow animations.')
        #       rainbow(strip)
        #       rainbowCycle(strip)
        #       theaterChaseRainbow(strip)
