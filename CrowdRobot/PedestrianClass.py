class Pedestrian():
    def __init__(self, x=0, y=0, vx=0, vy=0, c='red'):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.c = c
        
    def step(self, ax=0, ay=0):
        dt = 0.01
        
        self.x = self.x + ( self.vx + 0.5*ax*dt ) * dt
        self.y = self.y + ( self.vy + 0.5*ay*dt ) * dt
        
        self.vx = self.vx + ax * dt
        self.vy = self.vy + ay * dt
        
#        self.x = self.x + self.vx * dt
#        self.y = self.y + self.vy * dt