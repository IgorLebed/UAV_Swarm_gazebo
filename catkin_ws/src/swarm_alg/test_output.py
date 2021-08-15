work = True
while (work == True):
    print("Enter 1 to go to survey the territory")
    print("Enter 2 to go landing")
    print("Enter 3 to go landing")
    try:
        exit_p_num = int(raw_input("Input: "))
        print("This is number: ", exit_p_num)

    except ValueError:
        print("This is not num!")
        work = True
    else:
        if (exit_p_num == 1):
            print("This is coverage input: ")
            work = False
        elif (exit_p_num == 2):
            print("Exit from program")
            #self.set_mode("AUTO.LAND", 5)
            #self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 45, 0)
            #self.set_arm(False, 5)
            work = False
        elif (exit_p_num == 3):
            print("Return to launch")
            #self.set_mode("AUTO.RTL", 5)
            work = False
        elif (exit_p_num != 1 or exit_p_num != 2 or exit_p_num != 3):
            print("Try again!")
            work = True
