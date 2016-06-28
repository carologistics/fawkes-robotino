

function map() {


  $("body").append("<div id=map  class=wedgit container>  </dev>");//the wedgit container
 


  $(document) . ready (function()
  {   


    // ====================================Create Viewer Scence where the map and the robots will be displaied.
    var viewer = new ROS3D.Viewer({
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
    


    //============================================================================>Robot 1

    //====================================== TFClinet for R1
     // Setup a client to listen to TF of R_1.
    var tfClient_R1 = new ROSLIB.TFClient({
      ros : ros ,
      angularThres : 0.01,
      transThres : 0.01,
      fixedFrame : '/map' 
    });

    //=======Marker arrow shape to Show robot frame /base_link of TF
    var markerClient_R1= new ROS3D.MarkerClient({
      ros : ros ,
      tfClient : tfClient_R1,
      topic : '/arrow_marker_R1',
      rootObject : viewer.scene
    });

  // //============================================================================>Robot 2

  // //====================================== TFClinet for R2

    // Setup a client to listen to TF of R2.
    var tfClient_R2= new ROSLIB.TFClient({
      ros : ros_2 ,
      angularThres : 0.01,
      transThres : 0.01,
      fixedFrame : '/map'
    });

    //=======Marker arrow shape to Show robot frame /base_link of TF
    var markerClient_R2 = new ROS3D.MarkerClient({
      ros : ros_2 ,
      tfClient : tfClient_R2,
      topic : '/arrow_marker_R2',
      rootObject : viewer.scene
    });


  // ///============================================================================>Robot 3

  // //====================================== TFClinet for R3

  //   // Setup a client to listen to TF of R3.
  //   var tfClient_R3 = new ROSLIB.TFClient({
  //     ros : ros_3 ,
  //     angularThres : 0.01,
  //     transThres : 0.01,
  //     fixedFrame : '/map'
  //   });


  //   //=======Marker arrow shape to Show robot frame /base_link of TF
  //   var markerClient = new ROS3D.MarkerClient({
  //     ros : ros_3 ,
  //     tfClient : tfClient_R3,
  //     topic : '/arrow_marker_R3',
  //     rootObject : viewer.scene
  //   });
  // //----------------------------------------------




   

    var master_connection_ = ros ; 
    var master_tf_connection_ = tfClient_R1 ;
   
    var gridClient = new ROS3D.OccupancyGridClient({
      ros : master_connection_,
      rootObject : viewer.scene
    });


    //=============================================================Navgraph Marker Array to show the 
    var markerArrayClient = new ROS3D.MarkerArrayClient({
      ros : master_connection_ ,
      tfClient : master_tf_connection_ ,
      topic : '/visualization_marker_array',
      rootObject : viewer.scene
    });


 });

}

