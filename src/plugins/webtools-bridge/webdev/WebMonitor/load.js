var ros ;
var ros_2;
var ros_3;
var product_facts;// to hold the updated json contaning the "product" facts
var order_facts;// to hold the updated json contaning the "order" facts

function load(){
	//initialize all the connections to the bridge
	init();

	map();
	simple_monitor();
	orders();
	products();

}

function init() {
  // Connect to ROS.
  ros = new ROSLIB.Ros({
    url : 'ws://localhost:6060'
  });

 ros_2 = new ROSLIB.Ros({
    url : 'ws://localhost:5050'
  });

 ros_3 = new ROSLIB.Ros({
    url : 'ws://localhost:4040'
  });

}
