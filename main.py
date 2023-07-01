import time                   # Allows use of time.sleep() for delays
from lib.mqtt import MQTTClient   # For use of MQTT protocol to talk to Adafruit IO
import ubinascii              # Conversions between binary data and various encodings
import machine                # Interfaces with hardware components
from machine import Pin, PWM
import random                 # Random number generator
import dht
from machine import Pin       # Define pin


# BEGIN SETTINGS
# These need to be change to suit your environment
RANDOMS_INTERVAL = 20000    # milliseconds
last_random_sent_ticks = 0  # milliseconds
led = Pin("LED", Pin.OUT)   # led pin initialization for Raspberry Pi Pico W
tempSensor = dht.DHT11(Pin(27, Pin.IN))
buzzer = PWM(Pin(26))

# Wireless network
WIFI_SSID = "TeliaGateway30-91-8F-6E-4F-61"
WIFI_PASS = "0D06E192F6" # No this is not our regular password. :)

# Adafruit IO (AIO) configuration
AIO_SERVER = "io.adafruit.com"
MQ_SERVER = "192.168.1.237"
MQ_PORT = 3998
AIO_PORT = 1883
AIO_USER = "pranav97"
AIO_KEY = "aio_IQQY434x55kZFuIsjQbNgqVrgBW7"
AIO_CLIENT_ID = ubinascii.hexlify(machine.unique_id())
MQ_CLIENT_ID = ubinascii.hexlify(machine.unique_id())  # Can be anything
MQ_BUZZER_TOPIC = "sensors/buzzer"
AIO_RANDOMS_FEED = "pranav97/feeds/random"
AIO_TEMP_FEED = "pranav97/feeds/temperature"
AIO_HUMIDITY_FEED = "pranav97/feeds/humidity"
MQ_TEMP_FEED = "sensors/temperature"
MQ_HUMIDITY_FEED = "sensors/humidity"

# BUZZER Note Configuration

tones = {
"E5": 659,
"G5": 784,
"A5": 880,
}

songStart = ["E5","G5","A5"]
songStop = ["A5","G5","E5"]

# END SETTINGS



# FUNCTIONS

# Function to connect Pico to the WiFi
def do_connect():
    import network
    from time import sleep
    import machine
    wlan = network.WLAN(network.STA_IF)         # Put modem on Station mode

    if not wlan.isconnected():                  # Check if already connected
        print('connecting to network...')
        wlan.active(True)                       # Activate network interface
        # set power mode to get WiFi power-saving off (if needed)
        wlan.config(pm = 0xa11140)
        wlan.connect(WIFI_SSID, WIFI_PASS)  # Your WiFi Credential
        print('Waiting for connection...', end='')
        # Check if it is connected otherwise wait
        while not wlan.isconnected() and wlan.status() >= 0:
            print('.', end='')
            sleep(1)
    # Print the IP assigned by router
    ip = wlan.ifconfig()[0]
    print('\nConnected on {}'.format(ip))
    return ip 



# Callback Function to respond to messages from Adafruit IO
def sub_cb(topic, msg):          # sub_cb means "callback subroutine"
    print((topic, msg))          # Outputs the message that was received. Debugging use.
    if msg == b"ON":             # If message says "ON" but previous message says "OFF"...
        led.on()
        playsong(songStart)
    elif msg == b"OFF":          # If message says "OFF" ...
        led.off()
        playsong(songStop)
    else:                        # If any other message is received ...
        print("Unknown message") # ... do nothing but output that it happened.

# Function to generate a random number between 0 and the upper_bound
def random_integer(upper_bound):
    return random.getrandbits(32) % upper_bound

def playtone(frequency):
    buzzer.duty_u16(1000)
    buzzer.freq(frequency)

def bequiet():
    buzzer.duty_u16(0)

def playsong(mysong):
    for i in range(len(mysong)):
        if (mysong[i] == "P"):
            bequiet()
        else:
            playtone(tones[mysong[i]])
        time.sleep(0.3)
    bequiet()

# Function to publish random number to Adafruit IO MQTT server at fixed interval
def send_random():
    global last_random_sent_ticks
    global RANDOMS_INTERVAL
    tempSensor.measure()
    temperature = tempSensor.temperature()
    humidity = tempSensor.humidity()
    if ((time.ticks_ms() - last_random_sent_ticks) < RANDOMS_INTERVAL):
        return; # Too soon since last one sent.

    some_number = random_integer(100)
    print("Publishing: {0} and {1} to {2} and {3} respectively ... ".format(temperature, humidity, MQ_TEMP_FEED, MQ_HUMIDITY_FEED), end='')
    try:
        
        AIOclient.publish(topic=AIO_TEMP_FEED, msg=str(temperature))
        AIOclient.publish(topic=AIO_HUMIDITY_FEED, msg=str(humidity))
        MQclient.publish(topic=MQ_TEMP_FEED, msg=str(temperature))
        MQclient.publish(topic=MQ_HUMIDITY_FEED, msg=str(humidity))
        time.sleep(2)
        print("DONE")
    except Exception as e:
        print("FAILED")
    finally:
        last_random_sent_ticks = time.ticks_ms()


# Try WiFi Connection
try:
    ip = do_connect()
except KeyboardInterrupt:
    print("Keyboard interrupt")

# Use the MQTT protocol to connect to Adafruit IO and Mosquitto
MQclient = MQTTClient(AIO_CLIENT_ID, MQ_SERVER, MQ_PORT, keepalive=30)
AIOclient = MQTTClient(AIO_CLIENT_ID, AIO_SERVER, AIO_PORT, AIO_USER, AIO_KEY)
led.off()


# Subscribed messages will be delivered to this callback
MQclient.connect()
MQclient.set_callback(sub_cb)
AIOclient.connect()
MQclient.subscribe(MQ_BUZZER_TOPIC)
print("Connected to %s, subscribed to %s topic" % (MQ_SERVER, MQ_BUZZER_TOPIC))



try:                      # Code between try: and finally: may cause an error
                          # so ensure the client disconnects the server if
                          # that happens.
    while 1:              # Repeat this loop forever
        MQclient.check_msg()# Action a message if one is received. Non-blocking.
        send_random()     # Send a random number to Adafruit IO if it's time.
finally:                  # If an exception is thrown ...
    MQclient.disconnect()
    AIOclient.disconnect()   # ... disconnect the client and clean up.
    AIOclient = None
    MQclient = None
    print("Disconnected from Adafruit IO and Mosquitto.")

