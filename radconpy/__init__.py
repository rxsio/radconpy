import time

from radconpy.radcon import RadCon

def on_con():
    print("Connected")

def on_dis():
    print("Disconnected")
    
def on_data(t, hv, p):
    print("Data", t, hv, p)


if __name__ == "__main__":
    r = RadCon("COM3", reconnect_cooldown=1)
    
    r.on_connected.add(on_con)
    r.on_disconnected.add(on_dis)
    r.on_data.add(on_data)
    
    r.start()

    try:
        while True:
            time.sleep(1)
    finally:  
        r.stop()
