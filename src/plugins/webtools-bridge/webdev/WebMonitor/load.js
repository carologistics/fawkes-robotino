
var product_facts;// to hold the updated json contaning the "product" facts
var order_facts;// to hold the updated json contaning the "order" facts

window.throttle_rate = 5000; 


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

var remote = false; 

var intialiazed_ = false;
//var exploration_results_wedgit  ;
var mps_monitor;
var map;


function connect() {
 
  // Connect to ROS.
  var r1_con ;
  var r2_con ;
  var r3_con ; 

  //  __robots.push ( { name: "R 1"  , connection : r1_con , alive: false } ) ;  
  //  __robots.push ( { name: "R 2"  , connection : r2_con , alive: false } ) ;
  // __robots.push ( { name: "R 3"  , connection : r3_con , alive: false } ) ;

  if(remote){
 
    r1_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-1:6060' });
    r2_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-2:6060' });
    r3_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-3:6060' });

    // r1_con.on('error', function() 
    // {  
    //     r1_con = new ROSLIB.Ros({ url : 'ws://localhost:6060' });

    //     r1_con.on('connection', function() {
    //       var robot_info = { name: "R1"  , connection : r1_con , alive: false }  ;
    //       __robots.push (  robot_info ) ;  
    //       load( robot_info );
    //     });
    // });

    // r2_con.on('error', function() 
    // {
    //    r2_con = new ROSLIB.Ros({ url : 'ws://localhost:5050' });

    //    r2_con.on('connection', function() {
    //     var robot_info = { name: "R2"  , connection : r2_con , alive: false } ;
    //      __robots.push ( robot_info  ) ;
    //     load( robot_info );
    //     });

    // });
   

    // r3_con.on('error', function() 
    // {
    //     r3_con = new ROSLIB.Ros({ url : 'ws://localhost:4040' });

    //     r3_con.on('connection', function() {
    //       var robot_info = { name: "R3"  , connection : r3_con , alive: false } ;
    //       __robots.push ( robot_info ) ;
    //       load( robot_info );
    //     });
    // });
  }
  else
  {
    r1_con = new ROSLIB.Ros({ url : 'ws://localhost:6060' });
    r2_con = new ROSLIB.Ros({ url : 'ws://localhost:5050' });
    r3_con = new ROSLIB.Ros({ url : 'ws://localhost:4040' }); 

  }

  
   ros = r1_con; 
   ros_2 = r2_con;
   ros3 = r3_con; 

  //load();  



  r1_con.on('connection', function() {
    var robot_info = { name: "R1"  , connection : r1_con , alive: false }  ;
    __robots.push (  robot_info ) ;  
    load( robot_info );
  });

 r2_con.on('connection', function() {
  var robot_info = { name: "R2"  , connection : r2_con , alive: false } ;
   __robots.push ( robot_info  ) ;
  load( robot_info );
  });
 

  r3_con.on('connection', function() {
    var robot_info = { name: "R3"  , connection : r3_con , alive: false } ;
    __robots.push ( robot_info ) ;
    load( robot_info );
  });
}

window.  $layout_container = $("<div> <div>") . addClass ("layout_container"); 
window . $layout_side = $("<div> <div>") . addClass ("layout_side") ;

function init(){
  if ( intialiazed_ ) { return ; }
  intialiazed_ = true;

  $("body").append(window. $layout_container) ;
  $("body").append(window. $layout_side) ;
  
  //exploration_results_wedgit = new exploration_results ( window. $layout_container ) ;
  map = new Map();
  mps_monitor = new MpsMonitor ( ) ;
  simple_monitor();
  products();
  orders();

}


function load( robot_info ){
  init();

 // exploration_results_wedgit . visualize ( robot_info );
  mps_monitor                . visualize ( robot_info );
  map                        . visualize_marker (robot_info);
  
  if(robot_info . name == "R1"){
    map                        . visualize_map (robot_info);
  //  map                        . visualize_marker_array (robot_info);
  }

  robotInfo( robot_info.name , robot_info.connection );
}