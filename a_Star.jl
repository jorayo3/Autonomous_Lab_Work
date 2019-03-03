module a_Star

using DataStructures
using PyPlot

"Key coordinates"
sourceX = 1 #row
sourceY = 1 #column
goalX = 4
goalY = 7

"Grid Size"
xMax = 4
yMax = 4

type Node
	x::Int64
	y::Int64
	costf::Float64
	costg::Float64
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
    
    gNew = currentNode.costg + addG
   	fNew = gNew + calc_h(xNew, yNew, goalX, goalY)

    if (!haskey(Open_List, newIndex))
    	newNode = Node(xNew, yNew, gNew, fNew, cIndex)
    	Open_List[newIndex] = newNode
    	enqueue!(pq, newIndex, newNode.costf)
    else
    	newNode = Open_List[newIndex]
		if (newNode.costf > fNew)
        	newNode.costf = fNew
        	newNode.costg = gNew
        	newNode.pInd = cIndex
        	pq[newIndex] = fNew
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

	goalN = Node(goalX, goalY, 0, 0, -1) #initiallize goal node
	sourceN = Node(sourceX, sourceY, 0, 0,-1) #initialize source node
	cIndex = Index(sourceX,sourceY)

	Open_List, Closed_List = Dict{Int64,Node}(), Dict{Int64,Node}()
	Open_List[cIndex] = sourceN

	pq = PriorityQueue()
	enqueue!(pq, cIndex, 0)
	
	println("Printing expanded nodes:")
	while (length(Open_List) > 0)

		cIndex = dequeue!(pq) #pull lowest costing open Node
		currentNode = Open_List[cIndex]
		delete!(Open_List, cIndex)
	    Closed_List[cIndex] = currentNode

	    i = currentNode.x
	    j = currentNode.y
	    println(i,j)
	    
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
        				printMap(Closed_List)
        				return
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

function printMap(Closed_List)
	s = []
	index = Index(goalX,goalY)
	while(index != Index(sourceX,sourceY))
		push!(s,index)
		index = Closed_List[index].pInd
	end
	push!(s,index)
	while(!isempty(s))
		index = pop!(s)
		println(index)
	end
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

xMax = size(Open,1)
yMax = size(Open,2)
a_StarSearch(xMax, yMax, sourceX, sourceY, 
				goalX, goalY, Open)

end #module