syms a b c d t
f=(a*X+b*Y+c*Z-d)^2 /(a^2+b^2+c^2)- t*(a^2+b^2+c^2-1)
fa=diff(f, 'a')
A = jacobian(f,[a,b,c,d,t])
collect(A)




