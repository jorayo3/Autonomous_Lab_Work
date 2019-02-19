module a_Star

using DataStructures

"Key coordinates"
sourceX = 1 #row
sourceY = 1 #column
goalX = 4
goalY = 4

"Grid Size"
xMax = 4
yMax = 4

type Node
	x::Int64
	y::Int64
	costf::Float64
	costg::Float64
	unblocked::Bool
	pInd::Int64
end

"Heuristic Estimate Cost - Euclidean Distance"
function calc_h(x::Int64, y::Int64, goalX::Int64, goalY::Int64)
	return sqrt((x-goalX)^2 + (y-goalY)^2)
end

function verifyNode(node::Node, xMax::Int64, yMax::Int64)
	if (node.x < 1 || node.x > xMax || node.y < 1 || node.y > yMax) 
		return false
	end
	return node.unblocked
end

function isValid(x::Int64, y::Int64)
	if (x < 1 || x > xMax || y < 1 || y > yMax)
		return false
	end
	return true
end

#@To-DO find a better way to implement
function findNextStep(Open_List, Grid)
	minNode = Node(0,0,Inf,Inf,false,0)
	for i = 1:xMax
		for j = 1:yMax
			if (Open_List[i,j] == true 
				&& minNode.costf > Grid[i,j].costf)
				minNode = Grid[i,j]
			end
		end
	end

	return minNode
end

function evaluateNextNode(xOld::Int64, yOld::Int64, 
							xNew::Int64, yNew::Int64, 
							Adjacent::Bool, Open_List)
	if (Adjacent == true)
		addG = 1
    else
    	addG = sqrt(2)
    end
	gNew = Grid[xOld, yOld].costg + addG
    fNew = gNew + calc_h(xNew, yNew, goalX, goalY)
	if (Grid[xNew, yNew].costf > fNew)
		Open_List[xNew, yNew] = true
        Grid[xNew, yNew].costf = fNew
        Grid[xNew, yNew].costg = gNew
        Grid[xNew, yNew].pInd = calc_Index(xOld,yOld)
    end

end

function isDestination(x::Int64,y::Int64)
	return (x == goalX && y == goalY)
end

function a_StarSearch(xMax::Int64, yMax::Int64, sourceX::Int64, sourceY::Int64, 
				goalX::Int64, goalY::Int64, Grid::Array{Node})

	if (isValid(sourceX,sourceY) == false)
		println("Source is not valid on Grid.")
	end

	if (isValid(goalX,goalY) == false)
		println("Goal is not valid on Grid.")
	end

	if (sourceX == goalX && sourceY == goalY)
		println("Goal is at source.")
		return
	end

	Open_List = falses(xMax,yMax)
	Closed_List = falses(xMax,yMax)

	i = sourceX
	j = sourceY
	Grid[i,j].costf = 0
	Grid[i,j].costg = 0
	Open_List[i,j] = true

	while (Open_List != falses(xMax,yMax))

		currentNode = findNextStep(Open_List, Grid) #pull lowest costing open Node
	    i = currentNode.x
	    j = currentNode.y

	    Open_List[i,j] = false
	    Closed_List[i,j] = true

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

    			if (isValid(iNew, jNew))
    				if (Closed_List[iNew, jNew] == false && 
        						Grid[iNew, jNew].unblocked == true)
        				evaluateNextNode(i, j, iNew, jNew, Adjacent, Open_List)
        			end

        			if (isDestination(iNew, jNew))
        				println("Goal Found")
        				println("Printing Path")
        				printMap()
        				println("Printing Searched Nodes")
        				println(Closed_List)
        				return
        			end
        		end
        	end
        end   	
	end

	println("Unable to find the goal cell.")
	return

end

function calc_Index(x::Int64, y::Int64)
	index = (xMax*y - (xMax-x))
	return index
end

function printMap()
	s = []
	index = calc_Index(goalX,goalY)
	while(true)
		push!(s,index)
		if index == calc_Index(sourceX,sourceY)
			break
		else index = Grid[index].pInd
		end
	end

	while(!isempty(s))
		index = pop!(s)
		println(index)
	end
	return
end

function constructGrid(map::Array{Int64})
	Grid = Array{Node}(xMax, yMax)
	for i=1:xMax
		for j=1:yMax
			Grid[i,j] = Node(i,j,Inf,Inf,map[i,j],-1)
		end
	end
	return Grid
end

Open = [1 0 1 1;
		1 0 1 1;
		1 1 1 1;
		1 0 1 1]
Grid = constructGrid(Open)
a_StarSearch(xMax, yMax, sourceX, sourceY, 
				goalX, goalY, Grid)

end #module