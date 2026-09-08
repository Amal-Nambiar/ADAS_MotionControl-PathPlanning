class DynamicObstacle:

    def __init__(self, s, d, v):

        self.s =  s
        self.d =  d
        self.v =  v


    def predict(self,t):

        s = self.s + self.v * t
        d = self.d 

        return s, d

    