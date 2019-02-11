module a_Star

type Node
	x::int32
	y::int32
	costf::float32
	costg::float32
	costh::float32
	unblocked::bool
end

"Key coordinates"
sourceX = 0
sourceY = 0
goalX = 3
goalY = 3

"Grid Size"
xMax = 3
yMax = 3

"Heuristic Estimate Cost - Euclidean Distance"
function calc_h(int32 x, int32 y, goalX, goalY){
	return sqrt((x-goalX^2 + (y-goalY)^2)
	}
end

function verifyNode(node::Node, xMax, yMax){ #not sure about syntax here
	if (node.x < 0 || node.x > xMax || node.y < 0 || node.y > yMax) {
		return false
		}
	end
	return node.unblocked
	}
end

function isValid(x::int32, y::int32)
	if (x < 0 || x > xMax || y < 0 || y > yMax) {
		return false
	return true
end

#@To-DO find a better way to implement
function findNextStep(Open_List::Array{bool,(xMax,yMax)}, Grid) { 
	minNode::Node
	minNode.costf = Inf

	for (i = 0, i < xMax, i++)
		for (j = 0, j < yMax, j++)
			if (Open_List(i,j) == true && minNode.costf > Grid(i,j))
				minNode = Grid(i,j)
			end
		end
	end

	return minNode
end

function a_StarSearch(){
	if (isValid(sourceX,sourceY) == false)
		println("Source is not valid on Grid")
	end

	if (isValid(goalX,goalY) == false)
		println("Goal is not valid on Grid")
	end

	Open_List = falses(xMax,yMax)
	Closed_List = falses(xMax,yMax)
	Grid = Array{Node}(xMax, yMax)

	i::int32
	j::int32

	for (i=0, i < yMax, i++)
		for (j=0, j < xMax, j++)
			Grid(i,j).f = typemax(float32)
			Grid(i,j).g = typemax(float32)
			Grid(i,j).h = typemax(float32)
		end
	end

	i = sourceX
	j = sourceY
	Grid(i,j).f = 0
	Grid(i,j).g = 0
	Grid(i,j).h = 0
	Open_List(i,j) = true

	foundDest = false
	while (Open_List != false)

		currentNode = findNextStep(Open_List, Grid) #pull lowest costing open Node
	    i = currentNode.x
	    j = currentNode.y

	    Open_List(i,j) = false
	    Closed_List(i,j) = true

	    fNew::double
	    gNew::double
	    hNew::double

	    #Check Next Nodes
	   	for (iNew = i-1, iNew <= i+1, iNew++)
	   		for (jNew = j-1, jNew <= j+1, jNew++)
	   			if (iNew == i && jNew == j)
	   				continue
	   			elseif (iNew == i || jNew == j)
	   				Adjacent = true
	   			else
	   				Adjacent = false
	   			end

    			if (isValid(iNew, jNew))
        			if (isDestination(iNew, jNew))

        			end
        		elseif (Closed_List(iNew, jNew) == false && Grid(iNew, jNew).unblocked == true)
        			evaluateNextNode(i, j, iNew, jNew, Adjacent)
        		end
        	end
        end   	
	end
	
	if (foundDest == false)
	    println ("Unable to find the goal cell")
	end
end

function evaluateNextNode(iOld::int32, jOld::int32, iNew::int32, jNew::int32, Adjacent::bool)

        	if (Adjacent == true)
        	    addG = 1
        	else
        		addG = sqrt(2)
        	end

			gNew = Grid(iOld, jOld).g + addG
        	hNew = calc_h(iNew, jNew, goalX, goalY)
        	fNew = gNew + hNew
	        	if (Grid(iNew, jNew).f == typemax(float32) || Grid(iNew, jNew).f > fNew)
        			Open_List(iNew, jNew) = true
        			Grid(iNew, jNew).f = fNew
        			Grid(iNew, jNew).g = gNew
        			Grid(iNew, jNew).h = hNew
        	end
        end
    end
end

