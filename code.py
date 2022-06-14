import numpy as np
import matplotlib.pyplot as plt

class Nbody():
    #class vairables
    D = 2
    N = 30
    m = 10**4
    k = -10**-10
    side = 100
    init_side = 1
    init_avg_speed = 0.1
    total_time = 1
    h = 0.00001 #time_step
    objects = []
    K_data = []
    H_data = []
    E_data = []

    #class functions
    def __init__(self):
        self.q = Nbody.init_side * np.random.standard_normal(size=Nbody.D)  
        self.p = Nbody.m* Nbody.init_avg_speed * np.random.standard_normal(size=Nbody.D) 
        Nbody.objects.append(self)
        
    def kin():
        retval = sum((np.linalg.norm(Nbody.obj(i).p)**2)/(2*Nbody.m) for i in range(1,Nbody.N+1) )
        return retval
    
    def H():
      U = 0
      for i in range(1,Nbody.N+1):
        for j in range(1,Nbody.N+1):
          if(i!=j):
            U += Nbody.k*np.log(np.linalg.norm(Nbody.obj(i).q-Nbody.obj(j).q))
        return -U/2 + Nbody.kin()

    def E():
      U = 0
      for i in range(1,Nbody.N+1):
        for j in range(1,Nbody.N+1):
          if(i!=j):
            U +=Nbody.k* np.log(np.linalg.norm(Nbody.obj(i).q-Nbody.obj(j).q))
        return U/2 + Nbody.kin()
    
    def obj(i):
        return Nbody.objects[i-1]
    
    def next(): 
        for i in range(1,Nbody.N+1):
                     Nbody.obj(i).p += Nbody.delta_X(i)[0]
                     Nbody.obj(i).q += Nbody.delta_X(i)[1]
                     Nbody.obj(i).wall2()
                    
        
    def generator():
        for i in range(int(Nbody.total_time/Nbody.h)):
          #Nbody.H_data.append(Nbody.H())
          Nbody.K_data.append(Nbody.kin())
          #Nbody.E_data.append(Nbody.E())
          Nbody.next()
            
    def create():
        for i in range(1,Nbody.N+1):
            program = f"O_{i}=Nbody()"
            exec(program)
            
    def wall1(self):
      for i in range(Nbody.D):
        if (self.q[i]>Nbody.side):
          self.q[i] -= Nbody.side
        elif self.q[i]<0:
          self.q[1] += Nbody.side


    def wall2(self):
      for i in range(Nbody.D):
        if (self.q[i]>Nbody.side):
          self.p[i] *= -1
          self.q[i] = 2*Nbody.side - self.q[i]
        elif self.q[i]<0:
          self.p[i] *= -1
          self.q[i] *= -1

        
    def dist(i,j):
      return np.linalg.norm(Nbody.obj(i).q -Nbody.obj(j).q)

    def delta_X(i):
      X = [Nbody.obj(i).q,Nbody.obj(i).p]
      k1 = Nbody.F(X,i) 
      k2 = Nbody.F(X +Nbody.h*k1/2,i)
      k3 = Nbody.F(X +Nbody.h*k2/2,i)
      k4 = Nbody.F(X +Nbody.h*k3,i)
      return Nbody.h* (k1 + 2*k2 + 2*k3 + k4)/6

    def F(X,i):
      f = X[1]/Nbody.m
      g = 0  
      for j in range(1,i):
        g += (X[0] -Nbody.obj(j).q)/np.inner((X[0] -Nbody.obj(j).q),(X[0] -Nbody.obj(j).q))
      for j in range(i+1,Nbody.N+1):
        g += (X[0] -Nbody.obj(j).q)/np.inner((X[0] -Nbody.obj(j).q),(X[0] -Nbody.obj(j).q))
      g *= -Nbody.k

      print(f)
      print(g)
      return np.array([f,g])

    def delta_q(i):
      return Nbody.h*Nbody.obj(i).p/Nbody.m
    
    def delta_p(i):
      ret1 = 0
      for j in range(1,i):
        ret1 += np.sign(Nbody.obj(i).q -Nbody.obj(j).q)/np.linalg.norm(Nbody.obj(i).q -Nbody.obj(j).q)
      for j in range(i+1,Nbody.N+1):
        ret1 += np.sign(Nbody.obj(i).q -Nbody.obj(j).q)/np.linalg.norm(Nbody.obj(i).q -Nbody.obj(j).q)

      return -ret1*Nbody.h*Nbody.k/2



    def animate_2d():
      return 0
  

    def animate_3d():
      return 0
        

    def plot_H():
      plt.plot(range(int(Nbody.total_time/Nbody.h)),Nbody.H_data)

    
    def plot_K():
      plt.plot(range(int(Nbody.total_time/Nbody.h)),Nbody.K_data)  

    def plot_E():
      plt.plot(range(int(Nbody.total_time/Nbody.h)),Nbody.E_data)  



Nbody.create()

Nbody.generator()

Nbody.plot_K()
