import sys

import requests
from tqdm import tqdm

ip = "192.168.1.249"
listEndpoint = "/list"
downloadEndpoint = "/download?file="
try:
    numfiles= int(sys.argv[1])
except:
    print("please using an arg to provide number of pictures to download")
    exit(-1)

print("Fetching file list")
r = requests.get("http://" + ip + listEndpoint)
files = r.text.split("\n")
print("found", len(files), "files on ESP32-CAM SD");
files = files[len(files) - numfiles:]
print("Downloading files...")
for file in tqdm(files):
    if len(file) >4:
        try:
            url = "http://" + ip + downloadEndpoint + file
            r = requests.get(url)
            open("images/"+file[1:], 'wb').write(r.content)
        except Exception as e:
            pass