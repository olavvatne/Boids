def spiral(w, h):
	x = y = 0
	dx = 0
	dy = -1
	for i in range(max(w, h)**2):
		if (-w/2 < x <= w/2) and (-h/2 < y <= h/2):
			print (x, y)
			# DO STUFF...
		if x == y or (x < 0 and x == -y) or (x > 0 and x == 1-y):
			dx, dy = -dy, dx
		x, y = x+dx, y+dy

spiral(3, 3)