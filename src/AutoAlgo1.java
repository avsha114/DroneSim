import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

public class AutoAlgo1 {
	
	int map_size = 3000;
	enum PixelState {blocked,explored,unexplored,visited};
	PixelState map[][];
	Drone drone;
	Point droneStartingPoint;
	
	ArrayList<Point> points;
	
	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;
	
	boolean isSpeedUp = false;
	
	Graph mGraph = new Graph();
	
	CPU ai_cpu;
	public AutoAlgo1(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func =  new ArrayList<>();
		points = new ArrayList<Point>();
		
		drone = new Drone(realMap);
		drone.addLidar(0);
		drone.addLidar(90);
		drone.addLidar(-90);

		
		initMap();
		
		isRotating = 0;
		ai_cpu = new CPU(200,"Auto_AI");
		ai_cpu.addFunction(this::update);
	}
	
	public void initMap() {
		map = new PixelState[map_size][map_size];
		for(int i=0;i<map_size;i++) {
			for(int j=0;j<map_size;j++) {
				map[i][j] = PixelState.unexplored;
			}
		}
		
		droneStartingPoint = new Point(map_size/2,map_size/2);
	}
	
	public void play() {
		drone.play();
		ai_cpu.play();
	}

	
	public void update(int deltaTime) {
		updateVisited();
		updateMapByLidars();
		
		ai(deltaTime);
		
		
		if(isRotating != 0) {
			updateRotating(deltaTime);
		}
		if(isSpeedUp) {
			drone.speedUp(deltaTime);
		} else {
			drone.slowDown(deltaTime);
		}
		
	}
	
	public void speedUp() {
		isSpeedUp = true;
	}
	
	public void speedDown() {
		isSpeedUp = false;
	}
	
	public void updateMapByLidars() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		
		for(int i=0;i<drone.lidars.size();i++) {
			Lidar lidar = drone.lidars.get(i);
			double rotation = drone.getGyroRotation() + lidar.degrees;
			for(int distanceInCM=0;distanceInCM < lidar.current_distance;distanceInCM++) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
				setPixel(p.x,p.y,PixelState.explored);
			}
			
			if(lidar.current_distance > 0 && lidar.current_distance < WorldParams.lidarLimit - WorldParams.lidarNoise) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
				setPixel(p.x,p.y,PixelState.blocked);
				//fineEdges((int)p.x,(int)p.y);
			}
		}
	}
	
	public void updateVisited() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x,dronePoint.y + droneStartingPoint.y);
		
		setPixel(fromPoint.x,fromPoint.y,PixelState.visited);
			
	}
	
	public void setPixel(double x, double y,PixelState state) {
		int xi = (int)x;
		int yi = (int)y;
		
		if(state == PixelState.visited) {
			map[xi][yi] = state; 
			return;
		}
		
		if(map[xi][yi] == PixelState.unexplored) {
			map[xi][yi] = state; 
		}
	}
	
	public void paintBlindMap(Graphics g) {
		Color c = g.getColor();
		
		int i = (int)droneStartingPoint.y - (int)drone.startPoint.x;
		int startY = i;
		for(;i<map_size;i++) {
			int j = (int)droneStartingPoint.x - (int)drone.startPoint.y;
			int startX = j;
			for(;j<map_size;j++) {
				if(map[i][j] != PixelState.unexplored)  {
					if(map[i][j] == PixelState.blocked) {
						g.setColor(Color.RED);
					} 
					else if(map[i][j] == PixelState.explored) {
						g.setColor(Color.YELLOW);
					}
					else if(map[i][j] == PixelState.visited) {
						g.setColor(Color.BLUE);
					}
					g.drawLine(i-startY, j-startX, i-startY, j-startX);
				}
			}
		}
		g.setColor(c);
	}
	
	public void paintPoints(Graphics g) {
		for(int i=0;i<points.size();i++) {
			Point p = points.get(i);
			g.drawOval((int)p.x + (int)drone.startPoint.x - 10, (int)p.y + (int)drone.startPoint.y-10, 20, 20);
		}
		
	}
	
	public void paint(Graphics g) {
		if(SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}
		
		paintBlindMap(g);
		paintPoints(g);
		
		drone.paint(g);
		
		
	}
	
	boolean is_init = true;
	
	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double  risky_dis = 0;
	int max_angle_risky = 10;
	
	boolean is_lidars_max = false;

	// OUR CHANGE from 100 to 50:
	double max_distance_between_points = 50;
	

	// Our extensions:
	long startTime;
	Point startPoint;

	Point init_point;
	boolean first_return = false;
	int very_far = 250;
	public void ai(int deltaTime) {
		if(!SimulationWindow.toogleAI) {
			return;
		}
	
		
		if(is_init) {
			speedUp();

			// OUR CHANGES:
			startTime = System.currentTimeMillis();
			startPoint = drone.getOpticalSensorLocation();
			Point dronePoint = startPoint;

			init_point = new Point(dronePoint);
			points.add(dronePoint);
			mGraph.addVertex(dronePoint);
			is_init = false;
		}

		// OUR CHANGE: if battery/time has run out - return home and doing rotation of 180 :
		if ((System.currentTimeMillis() - startTime)/1000 >= 60 && (System.currentTimeMillis() - startTime)/1000 < 62) {
			SimulationWindow.return_home = true;
			first_return = true;
		}
		Point dronePoint = drone.getOpticalSensorLocation();

		
		if(SimulationWindow.return_home) {
			
			if( Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) <  max_distance_between_points) {
				if(points.size() <= 1 && Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) <  max_distance_between_points/5) {
					speedDown();
				} else {
					removeLastPoint();
				}
			}
			else if(first_return){
				spinBy(180);
				first_return = false;
			}
		} else {
			if( Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >=  max_distance_between_points) {
				points.add(dronePoint);
				mGraph.addVertex(dronePoint);
			}
		}
		
		if(!is_risky) {
			Lidar lidar = drone.lidars.get(0);
			if(lidar.current_distance <= max_risky_distance ) {
				is_risky = true;
				risky_dis = lidar.current_distance;
				
			}
			
			
			Lidar lidar1 = drone.lidars.get(1);
			if(lidar1.current_distance <= max_risky_distance/3 ) {
				is_risky = true;
			}
			
			Lidar lidar2 = drone.lidars.get(2);
			if(lidar2.current_distance <= max_risky_distance/3 ) {
				is_risky = true;
			}
			
		} else {
			if(!try_to_escape) {
				try_to_escape = true;

				Lidar lidar0 = drone.lidars.get(0);
				double frontLidarDistance = lidar0.current_distance;

				Lidar lidar1 = drone.lidars.get(1);
				double rightLidarDistance = lidar1.current_distance;
				
				Lidar lidar2 = drone.lidars.get(2);
				double leftLidarDistance = lidar2.current_distance;

				int rotationAngle = 0;

				//OUR CHANGE: if we dont go home
				if (!SimulationWindow.return_home){
					//When we are very far from the left wall we change the degrees to the left direction
					if (leftLidarDistance > very_far ) {
						rotationAngle = -90;
					}
					//when the lidar very close to the left wall, we go right
					if (leftLidarDistance < 90 && leftLidarDistance > 0 ) {
						rotationAngle = 5;
					}
					//when the lidar very close to the right wall, we go left
					if (rightLidarDistance < 80 && rightLidarDistance > 0 ) {
						rotationAngle = -10;
					}

					if (leftLidarDistance > 120 ) {
						rotationAngle = -5;
					}

					if (frontLidarDistance < 120 && frontLidarDistance > 0) {
						rotationAngle = 90;
					}

				} else {
					//When we are very far from the right wall we change the degrees to the right direction
					if (rightLidarDistance > very_far) {
						rotationAngle = 90;
					}
					//when the lidar very close to the right wall, we go left
					if (rightLidarDistance < 90 && rightLidarDistance > 0) {
						rotationAngle = -5;
					}
					//when the lidar very close to the left wall, we go right
					if (leftLidarDistance < 80 && leftLidarDistance > 0) {
						rotationAngle = 10;
					}

					if (rightLidarDistance > 120 ) {
						rotationAngle = 5;
					}

					if (frontLidarDistance < 120 && frontLidarDistance > 0) {
						rotationAngle = -90;
					}

					if (Tools.getDistanceBetweenPoints(this.getLastPoint(), dronePoint) < this.max_distance_between_points) {
						this.removeLastPoint();
					}
				}

				spinBy(rotationAngle,true,new Func() {
						@Override
						public void method() {
							try_to_escape = false;
							is_risky = false;
						}
				});
			}
		}

	}

	int counter = 0;


	double lastGyroRotation = 0;
	public void updateRotating(int deltaTime) {
		
		if(degrees_left.size() == 0) {
			return;
		}
		
		double degrees_left_to_rotate = degrees_left.get(0);
		boolean isLeft = true;
		if(degrees_left_to_rotate > 0) {
			isLeft = false;
		}
		
		double curr =  drone.getGyroRotation();
		double just_rotated = 0;
		
		if(isLeft) {
			
			just_rotated = curr - lastGyroRotation;
			if(just_rotated > 0) {
				just_rotated = -(360 - just_rotated);
			}
		} else {
			just_rotated = curr - lastGyroRotation;
			if(just_rotated < 0) {
				just_rotated = 360 + just_rotated;
			}
		}


		 
		lastGyroRotation = curr;
		degrees_left_to_rotate-=just_rotated;
		degrees_left.remove(0);
		degrees_left.add(0,degrees_left_to_rotate);
		
		if((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
			degrees_left.remove(0);
			
			Func func = degrees_left_func.get(0);
			if(func != null) {
				func.method();
			}
			degrees_left_func.remove(0);
			
			
			if(degrees_left.size() == 0) {
				isRotating = 0;
			}
			return; 
		}
		
		int direction = (int)(degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
		drone.rotateLeft(deltaTime * direction);
		
	}
	
	public void spinBy(double degrees,boolean isFirst,Func func) {
		lastGyroRotation = drone.getGyroRotation();
		if(isFirst) {
			degrees_left.add(0,degrees);
			degrees_left_func.add(0,func);
		
			
		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(func);
		}
		
		isRotating =1;
	}



	public void spinBy(double degrees) {
		lastGyroRotation = drone.getGyroRotation();

		degrees_left.add(degrees);
		degrees_left_func.add(null);
		isRotating = 1;
	}
	
	public Point getLastPoint() {
		if(points.size() == 0) {
			return init_point;
		}
		
		Point p1 = points.get(points.size()-1);
		return p1;
	}
	
	public Point removeLastPoint() {
		if(points.isEmpty()) {
			return init_point;
		}
		
		return points.remove(points.size()-1);
	}
	
	
	public Point getAvgLastPoint() {
		if(points.size() < 2) {
			return init_point;
		}
		
		Point p1 = points.get(points.size()-1);
		Point p2 = points.get(points.size()-2);
		return new Point((p1.x + p2.x) /2, (p1.y + p2.y) /2);
	}
	

}
