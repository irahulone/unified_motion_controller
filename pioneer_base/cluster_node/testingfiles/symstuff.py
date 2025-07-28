# import sympy 
from sympy import * 
  
x = symbols('x') 
exp = acos(x) + 7.0
print("Before Substitution : {}".format(exp))  
    
# Use sympy.subs() method 
res_exp = exp.subs(x, 0)  
res_exp = res_exp.evalf()
print("After Substitution : {}".format(res_exp)) 