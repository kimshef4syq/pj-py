import numpy as np

np1 = np.array([[1,2,3],[2,5,5]]).reshape(3,2,order='F')
#print(np1)
a = np.arange(5)
b = np.arange(4,6)
print(a,b)
print(np.meshgrid(a,b))
print(np1.shape)
print(np1.T)

#for i in np1.flat:
    #print (i)

print(a[2:4])

