import time
import sys
while True:
    try:
        for i in range(3):
            print("all good")
            time.sleep(0.5)
        raise RuntimeError
        
    except KeyboardInterrupt:
        sys.exit()
        pass
    except:    
        print("got exception")
