from hashlib import new
import serial
import argparse
from tokenize import String

class ProgrammingInterface:
  def __init__(self, port):
    baudrate = 1000000
    self.serial = serial.Serial(port, baudrate, timeout = 1)

  def initialize(self):
    done = False
    while not done:
      try:
        self.serial.write(b"\x01\x00")
        result = self.serial.read(1)
        assert result == b"\x81"
        done = True
      except:
        print("Error: Could not establish connection - try resetting your Arduino")
        while self.serial.read(1) != "":
          pass

    print("Got response from interface")

  def read(self, destination, size=0x3FFF, chunksize=0x10):
    file = open(destination, "w")
    self.initialize()

    for i in range(size)[::chunksize]:
      request = [
          0x5, 0x5,
          chunksize,
          (i >> 16) & 0xFF,
          (i >> 8) & 0xFF,
          i & 0xFF, 0,
      ]

      self.serial.write(request)
      response = self.serial.read(chunksize + 1)
      body = response[1:]

      if len(response) > 0 and len(body) > 0:
        print("===============================================")
        print("address: %s" % hex(i))
        print("request: %s" % bytes(request).hex())
        print("response code: %s" % hex(response[0]))
        print("response body: %s" % body.hex())

        line = bytearray([chunksize, (i >> 8) & 0xFF, i & 0xFF, 0]) + body
        crc = 0
        for nextbyte in line:
            crc = crc + nextbyte

        crc = (~crc + 1) & 0xFF
        line.append(crc)
        file.write(":%s\n" % line.hex())

      else:
        break

    file.write(":00000001FF\n")

    print("===============================================")
    print("Sucessfully dumped flash to '%s'" % destination)

  def erase(self):
    print("erase")

parser = argparse.ArgumentParser(description='Interact with the Arduino based EFM8 C2 interface')
parser.add_argument('action', metavar='ACTION', type=str,
                    help='Action to perform: read, write or erase',
                    choices=['read', 'write', 'erase'],)
parser.add_argument('port', metavar='PORT', type=str,
                    help='Port to use')
parser.add_argument('destination', metavar='DESTINATION', type=str, nargs='?', default=None,
                    help='Destination to write to or read from')

args = parser.parse_args()
interface = ProgrammingInterface(args.port)

if args.action == 'read':
  if not args.destination:
    parser.print_usage()
    parser.exit()

  interface.read(args.destination)

if args.action == 'erase':
  interface.erase()