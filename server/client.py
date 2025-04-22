import os
import time
import sys

import asyncio
import struct

ip = "192.168.50.147:80"
# ip = "192.168.50.61:80"

value = 0

while(1):
    try:
        
        inp = str(input()).split()
        if(inp[0] == "set"):
            value = int(inp[1])
            command = f"curl --data-binary '{value}' {ip}/control"
        elif(inp[0] == "get"):
            command = f"curl {ip}/hello"
        elif(inp[0] == "end"):
            break
        else:
            command = ""
    except:
        print("error")
        command = ""
        
    os.system(command)
    print("")
