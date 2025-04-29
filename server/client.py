import os

# ip = "192.168.50.147:80" #big floppa ip
ip = "192.168.48.219:80" #mobile network ip
value = 0

while(1):
    try:
        
        inp = str(input()).split()
        if(inp[0] == "set"):
            value = int(inp[1])
            command = f"curl --data-binary '{value}' {ip}/control"
        elif(inp[0] == "end"):
            break
        else:
            command = ""
    except:
        print("error")
        command = ""
        
    os.system(command)
    print("")
