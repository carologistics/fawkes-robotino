
var product_facts;// to hold the updated json contaning the "product" facts
var order_facts;// to hold the updated json contaning the "order" facts

window.throttle_rate = 1000;
window.tf_throttle_rate = 1000;
var remote = false;

var __robots = [];

var ros ;
var ros_2;
var ros_3 ;

var intialiazed_ = false;

var game_info_monitor;
var map;
var order_wedgit ;
var product_wedgit;
var exploration_results_wedgit  ;
var mps_monitor;



window . $layout_container = $("<div> <div>") . addClass ("layout_container");
window . $layout_side = $("<div> <div>") . addClass ("layout_side") ;
window . $production = $("<div> </div>"). addClass ("wedgit").addClass("production");



function connect() {

  // Connect to ROS.
  var r1_con ;
  var r2_con ;
  var r3_con ;


  var marker_connection= new ROSLIB.Ros({ url : 'ws://localhost:9090' });
  var robot_info = { name: "marker"  , connection : marker_connection , alive: false }  ;
  __robots.push (  robot_info ) ;

  // init();
  // map                        . visualize_map (__robots[0]);
  // map                        . visualize_marker (__robots[0] , __robots[0] );


  if(remote){

    r1_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-1:6060' });
    r2_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-2:6060' });
    r3_con = new ROSLIB.Ros({ url : 'ws://robotino-laptop-3:6060' });

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
    var robot_info = { name: "R1"  , connection : r1_con , alive: true }  ;
    __robots.push (  robot_info ) ;
    load( robot_info );
  });

 r2_con.on('connection', function() {
  var robot_info = { name: "R2"  , connection : r2_con , alive: true} ;
   __robots.push ( robot_info  ) ;
  load( robot_info );
  });


  r3_con.on('connection', function() {
    var robot_info = { name: "R3"  , connection : r3_con , alive: true } ;
    __robots.push ( robot_info ) ;
    load( robot_info );
  });
}

function init(){
  if ( intialiazed_ ) { return ; }
    intialiazed_ = true;

    $("body").append(window. $layout_container) ;
    $("body").append(window. $layout_side) ;

    //exploration_results_wedgit = new exploration_results ( window. $layout_container ) ;
    mps_monitor = new MpsMonitor ( ) ;
    game_info_monitor =  new GameInfoMonitor()
    map = new Map();
    simple_monitor();

    window. $production.appendTo(window. $layout_container) ;

  order_wedgit = new   orders();
  product_wedgit = new products();

}


function load( robot_info ){
  init();

  game_info_monitor          .visualize(robot_info);
  mps_monitor                . visualize ( robot_info );

 // map                        . visualize_marker (robot_info , __robots[0] );
  map                        . visualize_map ( robot_info );
  map                        . visualize_marker ( robot_info , robot_info );

  // if(robot_info. name == "R1"){
    map                        . visualize_marker_array (robot_info);
  // }

  product_wedgit             . visualize( robot_info );
  order_wedgit               . visualize( robot_info );

  robotInfo( robot_info.name , robot_info.connection );
}
