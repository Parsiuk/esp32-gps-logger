from machine import Pin

if Pin(25, Pin.IN, Pin.PULL_UP).value():
  import gps_logger
