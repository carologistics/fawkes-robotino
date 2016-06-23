
var product_facts;// to hold the updated json contaning the "product" facts
var order_facts;// to hold the updated json contaning the "order" facts

__robots();

function __robots(){
  var R_1;
  var R_2; 
  var R_3;
};//To  dynamicly store conncetion information for differnt robots

//temp to store the connection for the wedgits that do not yet use the __robot
var ros;
var ros_2;
var ros_3;

function load(){
	//initialize all the connections to the bridge
	init();
  exploration_results();
	map();
	simple_monitor();
	orders();
	products();
  robotInfo("R-1" , ros);
  robotInfo("R-2" , ros_2);
  robotInfo("R-3" , ros_3);


}

function init() {
  // Connect to ROS.
  r1_ros = new ROSLIB.Ros({ url : 'ws://localhost:6060' });
  
  r1_ros.on('connection', function() {
    __robots.R_1 = { name: "R 1"  , connection : r1_ros , alive: true} ;  
  });
  ros = r1_ros;


 r2_ros = new ROSLIB.Ros({ url : 'ws://localhost:5050' });

 r2_ros.on('connection', function() {
    var R_2 = { name: "R 2"  , connection : r2_ros , alive: true} ;  
    __robots ["R_2"] = R_2 ;
  })
 ros_2 = r2_ros ;

 
 r3_ros= new ROSLIB.Ros({  url : 'ws://localhost:4040' });

  r3_ros.on('connection', function() {
    var R_3 = { name: "R 3"  , connection : r3_ros , alive: true} ;  
    __robots ["R_3"] = R_3 ;
  })
  ros_3 = r3_ros;


 // ros = new ROSLIB.Ros({
 //    url : 'ws://localhost:9090'
 //  });

 // ros_2 = new ROSLIB.Ros({
 //    url : 'ws://localhost:8080'
 //  });

 // ros_3 = new ROSLIB.Ros({
 //    url : 'ws://localhost:7070'
 //  });

}
