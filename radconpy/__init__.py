import time

from radconpy.radcon import RadCon

if __name__ == "__main__":
    r = RadCon("com3", reconnect_cooldown=1)
    print(r)
    r.start()

    while True:
        time.sleep(1)
