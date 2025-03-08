import os
import time
import sys

import asyncio
import struct

ip = "192.168.50.147:80"
header = "/hello"
command = f"curl {ip}{header}"

def decode_state(msg) -> int:
    return(0)

class GameServerProtocol:
    def __init__(self, end_of_game, username):
        self.transport = None
        self.username = username
        self.end_of_game = end_of_game

    def connection_made(self, transport):
        print("Connection made")
        self.transport = transport
        transport.sendto(('CONNECT %s\n' % self.username).encode())

    def datagram_received(self, data, peer):
        global index
        if data.startswith(b"PLAYAS"):
            print("OK, starting game")
            begin, b_index = data.split()
            index = int(b_index)
            print(index)            
        elif data.startswith(b"NO VACANCY"):
            print("No such vacancy, please start with another username")
            self.end_of_game.set_result(True)
        elif data.startswith(b"STAT"):
            # state block got
            dt = decode_state(data)

    def connection_lost(self, exc):
        print("Connection lost: %s" % exc)
        self.end_of_game.set_result(True)


async def main():
    global name
    if len(sys.argv) < 3:
        print("Usage: %s <server> <user>" % (sys.argv[0]))
        sys.exit(1)
    server = sys.argv[1]
    username = sys.argv[2]
    name = username
    loop = asyncio.get_running_loop()
    end_of_game = loop.create_future()

    transport, protocol = await loop.create_datagram_endpoint(
            lambda: GameServerProtocol(end_of_game, username),
            remote_addr=(server, 9921))
    try:
        await end_of_game
    finally:
        transport.close()

if __name__ == '__main__':
    asyncio.run(main())



# while(1):
#     os.system(command)
#     time.sleep(0.25)