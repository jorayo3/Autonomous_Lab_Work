module a_Star

using DataStructures

"Key coordinates"
sourceX = 0 #row
sourceY = 0 #column
goalX = 3
goalY = 3

"Grid Size"
xMax = 3
yMax = 3

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
	if (node.x < 0 || node.x > xMax || node.y < 0 || node.y > yMax) 
		return false
	end
	return node.unblocked
end

function isValid(x::Int64, y::Int64)
	if (x < 0 || x > xMax || y < 0 || y > yMax)
		return false
	end
	return true
end

#@To-DO find a better way to implement
function findNextStep(Open_List, Grid)
	minNode = Node(-1,-1,Inf,Inf,false,0)
	for i = 1:yMax
		for j = 1:xMax
			if (Open_List[i,j] == true 
				&& minNode.costf > Grid[i,j].costf)
				minNode = Grid[i,j]
			end
		end
	end

	return minNode
end

function evaluateNextNode(xOld::Int32, yOld::Int32, 
							xNew::Int32, yNew::Int32, Adjacent::Bool)
	if (Adjacent == true)
		addG = 1
    else
    	addG = sqrt(2)
    end

	gNew = Grid[xOld, yOld].g + addG
    fNew = gNew + calc_h(xNew, yNew, goalX, goalY)

	if (Grid[xNew, yNew].f > fNew)
		Open_List[xNew, yNew] = true
        Grid[xNew, yNew].f = fNew
        Grid[xNew, yNew].g = gNew
        Grid[xNew, yNew].pInd = calc_Index(xOld,yOld)
    end

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

	i = sourceY
	j = sourceX
	Grid[i,j].f = 0
	Grid[i,j].g = 0
	Open_List[i,j] = true

	while (Open_List != false)

		currentNode = findNextStep(Open_List, Grid) #pull lowest costing open Node
	    i = currentNode.y
	    j = currentNode.x

	    Open_List[i,j] = false
	    Closed_List[i,j] = true

	    fNew::Float64
	    gNew::Float64

	    #Check Next Nodes
	    iNew::Int64
	    jNew::Int64
	   	for iNew = i-1:i+1
	   		for jNew = j-1:j+1
	   			Adjacent::Bool
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
        				evaluateNextNode(i, j, iNew, jNew, Adjacent)
        			end

        			if (isDestination(iNew, jNew))
        				println("Goal Found")
        				println("Printing Path")
        				printMap()
        				return
        			end
        		end
        	end
        end   	
	end

	println("Unable to find the goal cell.")
	return

end

function calc_Index(i::Int64, j::Int64)
	index = (yMax*j - (yMax-i))
	return index
end

function printMap()
	s = Stack{Int}()
	index = calc_Index(goalY,goalX)

	while(index != calc_Index(sourceY,sourceX))
		push!(s,index)
		index = Grid[index].pInd
	end

	while(true)
		index = pop!(s)
		println(index)
		if (index == calc_Index(sourceY,sourceX))
			return
		end
	end
end

function constructGrid(map::Array{Int64})
	Grid = Array{Node}(xMax, yMax)
	for i=1:yMax
		for j=1:xMax
			Grid[i,j] = Node(j,i,Inf,Inf,map[i,j],-1)
		end
	end
	return Grid
end

Open = [1 0 0;
		0 1 0;
		1 0 1]
Grid = constructGrid(Open)
println(Grid[1,3])
println(findNextStep(Open,Grid))
println(Open[2,3])

end #module