
var product_facts;// to hold the updated json contaning the "product" facts
var order_facts;// to hold the updated json contaning the "order" facts


var __robots = [];
//__robots();
// function __robots(){
//   var R_1;
//   var R_2; 
//   var R_3;
// };//To  dynamicly store conncetion information for differnt robots

var ros ; 
var ros_2;
var ros_3 ; 


var intialiazed_ = false;
var exploration_results_wedgit  ;
var mps_monitor;


function connect() {
 
  // Connect to ROS.
  // var r1_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-1:6060' });
  // var r2_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-2:6060' });
  // var r3_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-3:6060' });
  var r1_con = new ROSLIB.Ros({ url : 'ws://localhost:6060' });
  var r2_con = new ROSLIB.Ros({ url : 'ws://localhost:5050' });
  var r3_con = new ROSLIB.Ros({ url : 'ws://localhost:4040' }); 

  //  __robots.push ( { name: "R 1"  , connection : r1_con , alive: false } ) ;  
  //  __robots.push ( { name: "R 2"  , connection : r2_con , alive: false } ) ;
  // __robots.push ( { name: "R 3"  , connection : r3_con , alive: false } ) ;

  

   ros = r1_con; 
   ros_2 = r2_con;
   ros3 = r3_con; 

  //load();  

  r1_con.on('connection', function() {
    var robot_info = { name: "R 1"  , connection : r1_con , alive: false }  ;
    __robots.push (  robot_info ) ;  
    load( robot_info );
  });

 r2_con.on('connection', function() {
  var robot_info = { name: "R 2"  , connection : r2_con , alive: false } ;
   __robots.push ( robot_info  ) ;
  load( robot_info );
  })
 

  r3_con.on('connection', function() {
    var robot_info = { name: "R 3"  , connection : r3_con , alive: false } ;
    __robots.push ( robot_info ) ;
    load( robot_info );
  })


}

window.  $layout_container = $("<div> <div>") . addClass ("layout_container"); 
window . $layout_side = $("<div> <div>") . addClass ("layout_side") ;

function init(){
  if ( intialiazed_ ) { return ; }

  $("body").append(window. $layout_container) ;
  $("body").append(window. $layout_side) ;
  
  exploration_results_wedgit = new exploration_results ( window. $layout_container ) ;
  mps_monitor = new MpsMonitor ( ) ;
  
 // map();
  simple_monitor();
  products();
  orders();

  intialiazed_ = true;
}


function load( robot_info ){
  init();

  exploration_results_wedgit . visualize ( robot_info );
  mps_monitor                . visualize ( robot_info );

  robotInfo( robot_info.name , robot_info.connection );
}