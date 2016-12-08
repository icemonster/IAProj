f = open('track10.txt','w')

mapa = []

#Obstacles
for l in range(50):
	mapa.append([])
	for c in range(40):
		mapa[l].append('X')

#Track
for l in range(50):
	for c in range(40):
		if l > 5 and l < len(mapa)-5 and c > 5 and c < len(mapa[0])-5:
			mapa[l][c] = '0'
		if l > 96 and l < 110 and c > 6 and c < 99:
			mapa[l][c] = 'X' #being a betch

#Goals
mapa[24][-6] = 'E'
mapa[25][-6] = 'E'
mapa[26][-6] = 'E'
mapa[27][-6] = 'E'

#Start position
mapa[6][6] = 'S'


for l in mapa:
	f.write(''.join(l)+'\n')
f.close()