# Read BMS status notifications sent via BLE
#
# Usage:
#   python ble.py "<MAC ADDR>"
#

import bluepy
import struct
import sys
import uuid

mac = sys.argv[1]
characteristic_uuid = uuid.UUID('8D9D7801-5B61-412A-AB71-5C7E0E559086')

def display(values):
    print('cell 1: %dmV' % values[0])
    print('cell 2: %dmV' % values[1])
    print('cell 3: %dmV' % values[2])
    print('cell 4: %dmV' % values[3])
    print('current: %dmA' % values[4])
    print('charge: %d' % values[5])
    print('soc: %d' % values[6])

class NotificationDelegate(bluepy.btle.DefaultDelegate):

    def __init__(self):
        bluepy.btle.DefaultDelegate.__init__(self)

    def handleNotification(self, handle, data):
        print('bytes=%d' % len(data))
        values = struct.unpack('<HHHHhiB', data)
        display(values)

print('connecting to %s...' % mac)
peripheral = bluepy.btle.Peripheral(mac, 'random')
peripheral.setDelegate(NotificationDelegate())

try:
    characteristic = peripheral.getCharacteristics(uuid=characteristic_uuid)[0]

    print('listening...')
    notify_handle = characteristic.getHandle() + 1
    peripheral.writeCharacteristic(notify_handle, b'\x01\x00', withResponse=True)

    while True:
        if peripheral.waitForNotifications(5.0):
            continue
finally:
    print('disconnect')
    peripheral.disconnect()
