module traverse

using DataStructures
using NearestNeighbors
using PyPlot
using PyCall
@pyimport matplotlib.animation as anim

include("./hybrid_a_star.jl")

type Position
	x::Float64
	y::Float64
	yaw::Float64
end

const yMin = -10.0
const yMax = 10.0
const laneW = 5.0

const safetyR = 2
const obstProb = 0.01
const sightRange = 4

x0 = Position(0, -5, 0.0)
xF = Position(40,  -5, 0.0)
    ox = Float64[]
    oy = Float64[]
    for i = -2:50
        push!(ox, Float64(i))
        push!(oy, yMin)
    end
    # obstacle 3
    for i = -2:50
        push!(ox, Float64(i))
        push!(oy, yMax)
    end

function updatePosition(car:: Position, x::Float64, y::Float64, yaw::Float64)
    car.x = x
    car.y = y
    car.yaw = yaw
end

function updatePosition(yaw::Float64, dist::Float64, rad::Float64, ref::Position)
    #yaw = dir wrt North
    #dist = length to ref
    #rad = angle from nose of car in radians
    x = ref.x - dist*sin(yaw + rad)
    y = ref.y - dist*cos(yaw + rad)
    pos = Position(x, y, yaw)
    return pos
end

function generateObstacle(x::Float64, y::Float64)
    push!(ox, x)
    push!(oy, y)
    return [x, y]
end

function generateObstacle(pos::Position, dist::Float64, rad::Float64)
    x = pos.x + dist*sin(rad)
    y = pos.y + dist*cos(rad)
    push!(ox, x)
    push!(oy, y)
end

function main()
	cla()
	car = x0
	rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(car.x, car.y, car.yaw, xF.x, xF.y, xF.yaw, 
            ox, oy, hybrid_a_star.XY_GRID_RESOLUTION, hybrid_a_star.YAW_GRID_RESOLUTION, hybrid_a_star.OB_MAP_RESOLUTION)
	pathkdtree = KDTree(hcat(rx, ry)')
	scatter(car.x, car.y, label="car")
	   	plot(ox, oy, ".k",label="obstacles")
	   	if rx != nothing
	       	plot(rx, ry, ".r",label="Hybrid A* path")
	   	end
    	grid(true)
    	axis("equal")
    	show()
    	frameno = 1
    	savefig(@sprintf("%d.png", frameno))
    	cla()
	pos = 1
	while car.x != xF.x || car.y != xF.y

		if rand() < obstProb && xF.x - car.x > 8
			obst = generateObstacle(car.x + sightRange, car.y)
			if (!isempty(inrange(pathkdtree, obst, safetyR)))
				rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(car.x, car.y, car.yaw, xF.x, xF.y, xF.yaw, 
            		ox, oy, hybrid_a_star.XY_GRID_RESOLUTION, hybrid_a_star.YAW_GRID_RESOLUTION, hybrid_a_star.OB_MAP_RESOLUTION)
				pathkdtree = KDTree(hcat(rx, ry)')
				frameno += 1
				pos = 1
				# scatter(car.x, car.y, label="car")
	   			plot(ox, oy, ".k",label="obstacles")
	   			if rx != nothing
	       			plot(rx, ry, ".r",label="Hybrid A* path")
	   			end
    			grid(true)
    			axis("equal")
    			savefig(@sprintf("%d.png", frameno))
    			cla()
			end
		end
		# scatter(car.x, car.y, label="car")
	 #   	plot(ox, oy, ".k",label="obstacles")
	 #   	if rx != nothing
	 #       	plot(rx, ry, ".r",label="Hybrid A* path")
	 #   	end
  #   	grid(true)
  #   	axis("equal")
  #   	show()
  # ffmpeg
	   	pos += 1
	   	updatePosition(car, rx[pos], ry[pos], ryaw[pos])
	end
end

main()

end #module