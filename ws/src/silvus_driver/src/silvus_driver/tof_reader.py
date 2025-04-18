#!/usr/bin/env python3
import requests

class ToFReader:
    def __init__(self, radio_ip="192.168.0.136"):
        self.url = f"http://{radio_ip}/streamscape_api"
    
    def fetch_tof(self):
        """
        Perform JSON-RPC call to 'current_tof' and return a list
        of a float array: [nodeid1, tof1, age1, nodeid2, tof2, age2, ...]
        """
        headers = {"Content-Type": "application/json"}
        payload = {
            "jsonrpc": "2.0",
            "method": "current_tof",
            "id": "tof_reader"
        }

        resp = requests.post(self.url, json=payload, headers=headers, timeout=0.1)
        resp.raise_for_status()
        data = resp.json()

        if "error" in data:
            raise RuntimeError(f"Silvus API error: {data['error']}")
        
        result = data.get("result", [])
        current_tof = []
        # raw result is [nodeid1, tof1, age1, nodeid2, tof2, age2, ...]
        for i in range(0, len(result), 3):
            node = int(result[i])
            tof = int(result[i+1])
            age = int(result[i+2])

            current_tof.extend((node, tof, age))
        
        print(current_tof)
        return current_tof
    
    def run(self):
        print("Polling ToF API at", self.url)
        while True:
            try:
                current_tof = self.fetch_tof()
                print(current_tof)
            except Exception as e:
                print("Error:", e)
