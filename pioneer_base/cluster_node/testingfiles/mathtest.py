import numpy as np
import math
import sympy as sp 
from enum import Enum 

r_sym = sp.symbols('r0:9') #symbols for robot space state variables
theta_C = sp.symbols('0c') #symbol for cluster heading

#Derived FKine equations for cluster space configuration
x_c = (r_sym[0] + r_sym[3] + r_sym[6]) / 3
y_c = (r_sym[1] + r_sym[4] + r_sym[7]) / 3
theta_c = sp.atan2(2 / 3 * r_sym[0] - 1 / 3 * (r_sym[3] + r_sym[6]), 2 / 3 * r_sym[1] - 1 / 3 * (r_sym[4] + r_sym[7]))
phi1 = r_sym[2] + theta_C
phi2 = r_sym[5] + theta_C
phi3 = r_sym[8] + theta_C
p = sp.sqrt((r_sym[0]-r_sym[3])**2 + (r_sym[1]-r_sym[4])**2)
q = sp.sqrt((r_sym[6]-r_sym[0])**2 + (r_sym[1]-r_sym[7])**2)
B = sp.acos((p**2 + q**2 - (r_sym[6]-r_sym[3])**2 - (r_sym[7]-r_sym[4])**2)/(2*p*q))

FKine =  sp.Matrix([[x_c], [y_c], [theta_c], [phi1], [phi2], [phi3], [p], [q], [B]])
#print("Forward Kinematics Equations:")
#print(self.FKine)

c_sym = sp.symbols('c0:9') #symbols for cluster space state variables
r = sp.sqrt((c_sym[7]+c_sym[6]*sp.cos(c_sym[8]))**2 + (c_sym[6]*sp.sin(c_sym[8]))**2)
#Derived IKine equations for cluster space configuration
x_1 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2])
y_1 = c_sym[1] + 1/3 * r * sp.cos(c_sym[2])
theta_1 = c_sym[3] - c_sym[2]
x_2 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2]) - c_sym[6] * sp.sin(c_sym[8]/2 + c_sym[2])
y_2 = c_sym[1] + 1/3 * r * sp.cos(c_sym[2]) - c_sym[6] * sp.cos(c_sym[8]/2 + c_sym[2])
theta_2 = c_sym[4] - c_sym[2]
x_3 = c_sym[0] + 1/3 * r * sp.sin(c_sym[2]) + c_sym[7] * sp.sin(c_sym[8]/2 - c_sym[2])
y_3 = c_sym[1] + 1/3 * r * sp.cos(c_sym[2]) - c_sym[7] * sp.cos(c_sym[8]/2 - c_sym[2])
theta_3 = c_sym[5] - c_sym[2]
IKine = sp.Matrix([[x_1], [y_1], [theta_1], [x_2], [y_2], [theta_2], [x_3], [y_3], [theta_3]])
#print("Inverse Kinematics Equations:")
#print(self.IKine)

Jacob = FKine.jacobian(r_sym)
JacobInv = IKine.jacobian(c_sym)

def testTransforms(r):
    r_sym = sp.symbols('r0:9')
    subs_dict = {r_sym[i]: r[i, 0] for i in range(len(r_sym))} #map symbols to values
    subs_dict[sp.symbols('0c')] = 0 #pass in dummy
    print(FKine[2].subs(subs_dict))
    c = np.array(FKine.subs(subs_dict).evalf()).astype(np.float64)
    
    c_sym = sp.symbols('c0:9')
    subs_dict = {c_sym[i]: c[i, 0] for i in range(len(c_sym))} #map symbols to values
    r = np.array(IKine.subs(subs_dict).evalf()).astype(np.float64)

    return c, r

r = np.array([[0],
              [7.41299511],
              [0],
              [-8.7891157],
              [-3.70649286],
              [0],
              [8.7891157],
              [-3.70649286],
              [0]])
sub = np.array([[0],[0],[2],[1.4],[-1.4],[0],[-1.4],[-1.4],[0]])
test = np.array([[0], [5.77], [0], [-5], [-2.886], [0], [5], [-2.886], [0]])
test_c, test_r = testTransforms(test)
print(test_c)
print("_______________-")
print(test_r)
t, c = testTransforms(r)

cdes = np.zeros((3*3, 1))
cdes[(3-1)*3:(3)*3] = [[10],[10],[math.pi/3]]
c_sym = sp.symbols('c0:9')
subs_dict = {c_sym[i]: cdes[i, 0] for i in range(len(c_sym))} #map symbols to values
r = np.array(IKine.subs(subs_dict).evalf()).astype(np.float64)
print(cdes)
#print("----------------")
print(r)


x = 10
y = 5
distance = math.sqrt(x**2 + y**2)
angle = math.atan2(y, x)
x_1 = distance * math.cos(angle)
y_1 = distance * math.sin(angle)
print("Difference in x and y") 
print(x-x_1)
print(y-y_1)
print(0.1%(math.pi))
print((math.pi-0.1)%(math.pi))
print((-math.pi+0.1)%(math.pi))
