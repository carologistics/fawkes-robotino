

function Map() {

  var that = this ;

    var $map = $("<div>  </div>") .addClass("wedgit") .attr("id" , "map") ;
    $(window.$layout_container).append($map) ;//the wedgit container

    // ====================================Create Viewer Scence where the map and the robots will be displaied.
    this. viewer = new ROS3D.Viewer({
      divID : 'map',
      width :  document.getElementById("map").offsetWidth ,
      height : 500 ,
      antialias : true,
      cameraPose : {
                    x : 0,
                    y : -10,
                    z : 25
                    }
    });




    this. visualize_marker =  function (  tf_robot_info  ,  marker_connection_on ) {

        //====================================== TFClinet for R1
         // Setup a client to listen to TF of R_1.
        var tfClient = new ROSLIB.TFClient({
          ros : tf_robot_info.connection   ,
          angularThres :0.01,
          transThres : 0.01,
          fixedFrame : '/map' ,
          rate: 10
        });

        //=======Marker arrow shape to Show robot frame /base_link of TF
        var markerClient= new ROS3D.MarkerClient({
          ros : marker_connection_on.connection   ,
          tfClient : tfClient,
          topic : '/arrow_marker_'+ tf_robot_info.name ,
          rootObject : that. viewer.scene
        });

    };


  this . visualize_map = function( robot_info ){

      var gridClient = new ROS3D.OccupancyGridClient({
        ros : robot_info .connection ,
        rootObject : that. viewer.scene
      });
  };


  this .visualize_marker_array = function( robot_info ){

     var tfClient = new ROSLIB.TFClient({
          ros : robot_info.connection   ,
          angularThres : 0.01,
          transThres : 0.01,
          fixedFrame : '/map',
          rate : 10
      });

      //=============================================================Navgraph Marker Array to show the
      var markerArrayClient = new ROS3D.MarkerArrayClient({
        ros : robot_info.connection ,
        tfClient : tfClient ,
        topic : '/visualization_marker_array',
        rootObject : that. viewer.scene
      });

    };



    // this. visualize_laser_lines = function ( robot_info) {

    //     var tfClient = new ROSLIB.TFClient({
    //       ros : robot_info.connection   ,
    //       angularThres : 0.01,
    //       transThres : 0.01,
    //       fixedFrame : '/map',
    //       rate : 10
    //   });

    //   var myMapObject = that ;

    //   //=============================================================Navgraph Marker Array to show the
    //   var LaserScan = new ROS3D.LaserScan({
    //     ros : robot_info.connection ,
    //     tfClient : tfClient ,
    //     topic : 'fawkes_scans/Laser_urg_filtered_360'
    //     rootObject : myMapObject. viewer.scene
    //     // color :
    //   });

    // };


// /fawkes_scans/Laser_urg_filtered_360

    // new ROS3D.LaserScan = function(options) {
    //       options = options || {};
    //       var ros = options.ros;
    //       var topic = options.topic || '/scan';
    //       this.color = options.color || 0xFFA500;
    //       var that = this;

    //       this.particles = new ROS3D.Particles(options);

    //       var rosTopic = new ROSLIB.Topic({
    //         ros : ros,
    //         name : topic,
    //         messageType : 'sensor_msgs/LaserScan'
    //       });


}


// /colli_cells_far
// /colli_cells_free green
// /colli_cells_mid yellow
// /colli_cells_near orang
// /colli_cells_occupied red
