class First(object):
    def __init__(self):
        super(First, self).__init__()
        print("first")
        global param_1_1
        global param_1_2
        param_1_1 = 0
        param_1_2 = 1

class Second(object):
    def __init__(self):
        super(Second, self).__init__()
        print("second")
        global param_2_1
        global param_2_2
        param_2_1 = 2
        param_2_2 = 3
        

class Third(First, Second):
    #def __init__(self):
    #    super(Third, self).__init__()
    #    print("third")
    def get_param(self):
        print(param_1_1)
        print(param_2_1)


        

if __name__ == '__main__':
    con = Third()
    con.get_param()
    
