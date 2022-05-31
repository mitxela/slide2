from scipy.interpolate import interp1d
import math


x = [ 0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250]

y = [ 443.156317780377,453.256358588556,473.667695788782,498.750025287373,525.014667845346,556.355958515414,584.476459604109,616.956346704914,653.755311317455,689.577461101061,728.22371101336,767.889645379502,812.182635775354,861.237680458741,909.310655899075,960.193551603307,1011.54138033293,1062.0470239238,1113.50495237062,1158.60722572882,1204.55747820289,1243.74200751718,1284.44765310464,1316.12070070545,1340.34322331136,1354.58104929122 ]

f = interp1d(x, y, kind='cubic')


table=[]
n = 67.0
while (n<91):
  targetFreq = 440*math.pow(2, (n-69)/12.0 )

  print('\t', n, end='\r')

  if targetFreq <= y[0]:
    guess = 0
  elif targetFreq >= y[-1]:
    guess = x[-1]
  else:
    guess=0
    for k in range(0,251):
      error = abs(f(k)-targetFreq)
      if ( error < abs(f(guess)-targetFreq) ): guess=k
  table.append(guess)
  n+= 1/32.0

print("")

for angle in table:
  speed = int( 640 + (( angle /250)**1.5)*450 )
  print("{%d, %d}," % (speed, angle))
