module RRT

using PyPlot
using NearestNeighbors

#Vehicle Dimensions
const Lr = 1.5
const Lf = 1
const w = 0.5

# Vehicle Dynamics Constants
const steerALimit = pi/4
const v = [-1,1]

#Control Constant
const tCostWeight = 1

#Map Construction
reso = 1

type Node
	x::Float64
	y::Float64
	theta::Float64
	pInd::Int64
end

#vehicle dynamics
function vehicleDynamicsStep(x::Float64, y::Float64, theta::Float64, 
													v::Int64, steerA::Float64)
	beta = atan(Lr*tan(steerA), Lf+Lr)
	dx = v*cos(theta + beta)
	dy = v*sin(theta + beta)
	dtheta = v/lr*sin(beta)
	return x+dx, y+dy, theta+dtheta
end

function isValid(x::Int64, y::Int64, Grid)
	if (x < 1 || x > xMax || y < 1 || y > yMax ||
		Grid[x,y] == false)
		return false
	end
	return true
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

function calcDistance(node::Node, goal::Node)
	if (node.theta >= 2*pi)
		node.theta -= 2*pi
	elseif (node.theta <= -2*pi)
		node.theta += 2*pi

	return sqrt((node.x - goal.x)^2 + (node.y - goal.y)^2 
		+ tCostWeight*(node.theta - goal.theta)^2)
end

function checkCol(node::Node)
	#use KDTREE
end

function Index(x::Float64, y::Float64)
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

end