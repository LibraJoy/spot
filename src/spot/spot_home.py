from spot.spot_spot import connect, move

if __name__ == '__main__':
    while True:
        if connect():
            break
        else:
            print("connection fails")
    
    print("begin")
    move(-0.5, 0.5)