module a_Star

using DataStructures
using NearestNeighbors

"Key coordinates"
sourceX = 1 #row
sourceY = 1 #column
goalX = 4
goalY = 7

type Node
	x::Int64
	y::Int64
	costG::Float64
	costH::Float64
	pInd::Int64
end

#Heuristic Estimate Cost - Euclidean Distance
function calc_h(x::Int64, y::Int64, goalX::Int64, goalY::Int64)
	return sqrt((x-goalX)^2 + (y-goalY)^2)
	#return max((x-goalX), (y-goalY))
end

function isValid(x::Int64, y::Int64, Grid)
	if (x < 1 || x > xMax || y < 1 || y > yMax ||
		Grid[x,y] == false)
		return false
	end
	return true
end

function evaluateNextNode(xOld::Int64, yOld::Int64, 
							xNew::Int64, yNew::Int64, 
							Adjacent::Bool, Open_List,
							pq, currentNode, cIndex:: Int64, 
							newIndex::Int64)
	if (Adjacent == true)
		addG = 1
    else
    	addG = sqrt(2)
    end
    
    gNew = currentNode.costG + addG

    if (!haskey(Open_List, newIndex))
    	costH = calc_h(xNew, yNew, goalX, goalY)
    	newNode = Node(xNew, yNew, gNew, costH, cIndex)
    	Open_List[newIndex] = newNode
    	enqueue!(pq, newIndex, gNew + costH)
    else
    	newNode = Open_List[newIndex]
		if (newNode.costG > gNew)
        	newNode.costG = gNew
        	newNode.pInd = cIndex
        	pq[newIndex] = gNew + newNode.costH
    	end
    end
end

function isDestination(x::Int64,y::Int64)
	return (x == goalX && y == goalY)
end

function a_StarSearch(xMax::Int64, yMax::Int64, sourceX::Int64, sourceY::Int64, 
				goalX::Int64, goalY::Int64, Grid::Array{Int64})

	if (isValid(sourceX,sourceY,Grid) == false)
		println("Source is not valid on Grid.")
		return
	end

	if (isValid(goalX,goalY,Grid) == false)
		println("Goal is not valid on Grid.")
		return
	end

	if (sourceX == goalX && sourceY == goalY)
		println("Goal is at source.")
		return
	end

	goalN = Node(goalX, goalY, 0, -1, -1) #initiallize goal node
	sourceN = Node(sourceX, sourceY, 0, -1, -1) #initialize source node
	cIndex = Index(sourceX,sourceY)

	Open_List, Closed_List = Dict{Int64,Node}(), Dict{Int64,Node}()
	Open_List[cIndex] = sourceN

	pq = PriorityQueue()
	enqueue!(pq, cIndex, 0)
	
	while (length(Open_List) > 0)

		cIndex = dequeue!(pq) #pull lowest costing open Node
		currentNode = Open_List[cIndex]
		delete!(Open_List, cIndex)
	    Closed_List[cIndex] = currentNode

	    i = currentNode.x
	    j = currentNode.y
	    
	    #Check Next Nodes
	   	for iNew = i-1:i+1
	   		for jNew = j-1:j+1
	   			if (iNew == i && jNew == j)
	   				continue
	   			elseif (iNew == i || jNew == j)
	   				Adjacent = true
	   			else
	   				Adjacent = false
	   			end

    			if (isValid(iNew, jNew, Grid))
    				newIndex = Index(iNew,jNew)
    				if (!haskey(Closed_List, newIndex))
        				evaluateNextNode(i, j, iNew, jNew, Adjacent, Open_List, pq,
        					currentNode, cIndex, newIndex)
        			end

        			if (isDestination(iNew, jNew))
        				println("Goal Found!")
        				println("Printing Path:")
        				goalN.pInd = cIndex
        				Closed_List[Index(goalN.x,goalN.y)] = goalN
        				px, py = GetPath(Closed_List)
        				ex, ey = GetExpandedNodes(Closed_List)
        				return px, py, ex, ey
        			end
        		end
        	end
        end   	
	end

	println("Unable to find the goal cell.")
	return

end

function Index(x::Int64, y::Int64)
	index = (xMax*y - (xMax-x))
	return index
end

function GetPath(Closed_List)
	px = []
	py = []
	index = Index(goalX,goalY)
	while(true)
		n = Closed_List[index]
		push!(px,n.x)
		push!(py,n.y)
		index = n.pInd
		if (index == Index(sourceX,sourceY))
			break
		end
	end

	return px, py
end

function GetExpandedNodes(Closed_List)
	ex = []
	ey = []
	for i in values(Closed_List)
		push!(ex,i.x)
		push!(ey,i.y)
	end

	return ex, ey
end

function initializeObstacles(ox::Array{Float64}, oy::Array{Float64}, vr::Int64)
	Open = fill(1, (60,60))
	kdtree = KDTree(hcat(ox, oy)')
    for x in 1:60
        for y in 1:60
            idxs, onedist = knn(kdtree, [x, y] , 1)
            if onedist[1] <= vr 
                Open[x,y] = 0
            end
        end
    end
    return Open
end

# Build Test Cases
Open = [1 0 1 1;
		1 1 1 1;
		1 1 0 1;
		1 0 1 1] #goes backwards because of heuristic.
#

Open = [1 0 1 0 1 1 1;
		1 1 1 0 0 0 1;
		0 1 0 1 0 1 0;
		0 1 1 1 1 0 1] #Interesting path due to heuristic. Would work with different one.

	xMax = 60
	yMax = 60
    sourceX = 10  # [m]
    sourceY = 10  # [m]
    goalX = 50  # [m]
    goalY = 50  # [m]

    ox = Float64[]
    oy = Float64[]

    for i in 1:60.0
        push!(ox, Float64(i))
        push!(oy, 1.0)
    end
    for i in 1:60.0
        push!(ox, 60.0)
        push!(oy, Float64(i))
    end
    for i in 1:60.0
        push!(ox, Float64(i))
        push!(oy, 60.0)
    end
    for i in 1:60.0
        push!(ox, 1.0)
        push!(oy, Float64(i))
    end
    for i in 1:40.0
        push!(ox, 20.0)
        push!(oy, Float64(i))
    end
    for i in 1:40.0
        push!(ox, 40.0)
        push!(oy, 60.0-Float64(i))
    end

vr = 0
Open = initializeObstacles(ox, oy, vr)


@time px, py, ex, ey = a_StarSearch(xMax, yMax, sourceX, sourceY, 
				goalX, goalY, Open)

# using PyPlot
# plot(ox, oy, ".k",label="obstacles")
# plot(sourceX, sourceY, "xr",label="start")
# plot(goalX, goalY, "xb",label="goal")
# plot(px, py, "-r",label="A* path")
# legend()
# grid(true)
# axis("equal")
# show()

using Plots
display(plot(ex,ey,seriestype=:scatter,title="Expanded Nodes"))

end #module