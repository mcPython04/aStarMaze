from pyamaze import maze, agent
from queue import PriorityQueue

# heuristic for A* alg
def h(cell1, cell2):
	x1, y1 = cell1
	x2, y2 = cell2
	
	# returns the manhattan distance
	return abs(x1-x2) + abs(y1-y2)
	
def aStar(m):
	# initialize start state
	start = (m.rows, m.cols)
	
	# initialize values for all grids
	g = {cell:float('inf') for cell in m.grid}
	g[start] = 0
	f = {cell:float('inf') for cell in m.grid}
	f[start] = h(start, (1,1))
	
	open = PriorityQueue()
	open.put((h(start, (1,1)), h(start, (1,1)), start))
	
	aPath = {}
	
	while not open.empty():
		currCell = open.get()[2]
		if currCell == (1,1):
			break
		for d in 'ESNW':
			if m.maze_map[currCell][d] == True:
				if d == 'E':
					childCell = (currCell[0], currCell[1]+1)
				if d == 'W':
					childCell = (currCell[0], currCell[1]-1)
				if d == 'N':
					childCell = (currCell[0]-1, currCell[1])
				if d == 'S':
					childCell = (currCell[0]+1, currCell[1])
				
				temp_g = g[currCell] + 1
				temp_f = temp_g + h(childCell, (1,1))
				
				if temp_f < f[childCell]:
					g[childCell] = temp_g
					f[childCell] = temp_f
					open.put((temp_f, h(childCell, (1,1)), childCell))
					aPath[childCell] = currCell
	fwdPath = {}
	cell = (1,1)
	
	while cell != start:
		fwdPath[aPath[cell]] = cell
		cell = aPath[cell]
	return fwdPath

m = maze(20, 20)
m.CreateMaze()
path = aStar(m)

a = agent(m, footprints = True)
m.tracePath({a:path})

print(m.maze_map)

m.run()
